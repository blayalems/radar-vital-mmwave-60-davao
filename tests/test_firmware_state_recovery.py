import re
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "radar_vital_v16_4_0.ino"


def _firmware() -> str:
    return FW.read_text(encoding="utf-8", errors="ignore")


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
    raise AssertionError(f"unterminated function: {signature}")


def _uint8_constant(source: str, name: str) -> int:
    match = re.search(rf"static const uint8_t\s+{name}\s*=\s*(\d+)", source)
    assert match, f"missing {name}"
    return int(match.group(1))


@dataclass
class _PacketPresenceModel:
    present_votes: int
    absent_votes: int
    debounced: bool = False
    present_score: int = 0
    absent_score: int = 0
    last_positive_ms: int = 0

    def update(
        self,
        raw_present: bool,
        now_ms: int,
        *,
        present_boost: int = 1,
        absent_boost: int = 1,
    ) -> None:
        if raw_present:
            self.last_positive_ms = now_ms
            self.present_score = min(
                self.present_votes, self.present_score + present_boost
            )
            self.absent_score = 0
        else:
            self.absent_score = min(
                self.absent_votes, self.absent_score + absent_boost
            )
            self.present_score = 0

        if not self.debounced and self.present_score >= self.present_votes:
            self.debounced = True
        elif self.debounced and self.absent_score >= self.absent_votes:
            self.debounced = False


@dataclass
class _LcdLifecycleModel:
    connected: bool = False
    allocated: bool = False
    pointer: bool = False
    address: int = 0
    successful_recoveries: int = 0

    def detach(self) -> None:
        self.pointer = False
        self.allocated = False
        self.connected = False
        self.address = 0

    def attach(self, address: int, *, recovery: bool) -> None:
        if (
            self.connected
            and self.allocated
            and self.pointer
            and self.address == address
        ):
            return
        self.detach()
        self.pointer = True
        self.allocated = True
        self.connected = True
        self.address = address
        if recovery:
            self.successful_recoveries += 1


@dataclass
class _FirmwareVersionCaptureModel:
    state: str = "idle"
    deadline_ms: int = 0
    version_valid: bool = False

    def arm(self, now_ms: int, window_ms: int) -> None:
        self.state = "armed"
        self.deadline_ms = now_ms + window_ms

    def service(self, now_ms: int, *, version_available: bool) -> None:
        if self.state != "armed":
            return
        if version_available:
            self.version_valid = True
            self.state = "captured"
            self.deadline_ms = 0
        elif now_ms >= self.deadline_ms:
            self.state = "expired"
            self.deadline_ms = 0


def test_packet_presence_debounce_does_not_refresh_on_absence():
    source = _firmware()
    model = _PacketPresenceModel(
        present_votes=_uint8_constant(source, "RAW_PRESENT_VOTES"),
        absent_votes=_uint8_constant(source, "RAW_ABSENT_VOTES"),
    )

    for index in range(model.present_votes):
        model.update(True, 1_000 + index * 250)

    assert model.debounced
    last_positive_ms = model.last_positive_ms

    # Sustained negative packets may leave the packet debounce latched for its
    # bounded absence window, but they must not manufacture fresh evidence.
    for index in range(model.absent_votes - 1):
        model.update(False, 2_500 + index * 250)
        assert model.last_positive_ms == last_positive_ms
    assert model.debounced

    model.update(False, 2_500 + (model.absent_votes - 1) * 250)
    assert not model.debounced
    assert model.last_positive_ms == last_positive_ms


def test_firmware_presence_layers_have_one_way_ownership():
    source = _firmware()
    packet_owner = _function_body(
        source,
        "static void updateRadarPacketPresence(bool rawPresent, unsigned long now)",
    )

    assert "radarIsPresentRaw = rawPresent;" in packet_owner
    assert "lastRadarPresencePacketMs = now;" in packet_owner
    assert "radarPresentScore =" in packet_owner
    assert "radarAbsentScore =" in packet_owner
    assert "radarIsPresent = true;" in packet_owner
    assert "radarIsPresent = false;" in packet_owner
    assert source.count(
        "updateRadarPacketPresence(radarIsPresentInstant, now);"
    ) == 1

    assert (
        "bool radarPresenceEvidence = radarIsPresent && "
        "!radarPacketExpired;"
    ) in source
    assert "hardNoPresenceEvidence = !radarIsPresent &&" in source
    assert "fsm_fallback" not in source
    assert "if (radarIsPresent) lastPresenceUpdateMs=now;" not in source

    fsm_projection = source[
        source.index("// Backward-compatible boolean used by the rest")
        :source.index("// Preserve lastGoodDistance across brief absence")
    ]
    for upstream_name in (
        "radarIsPresent",
        "radarIsPresentRaw",
        "radarPresentScore",
        "radarAbsentScore",
        "lastRadarPresencePacketMs",
    ):
        assert not re.search(rf"\b{upstream_name}\s*=", fsm_projection)

    skip_dsp = source[
        source.index("if (skipDSPForcedAbsent) {")
        :source.index("// [v11.0.0] DUAL-PIPELINE", source.index("if (skipDSPForcedAbsent) {"))
    ]
    assert "humanDetected = false;" in skip_dsp
    assert "radarIsPresent =" not in skip_dsp
    assert "radarIsPresentRaw =" not in skip_dsp


def test_lcd_lifecycle_is_idempotent_and_counts_only_recovery():
    lcd = _LcdLifecycleModel()

    lcd.attach(0x27, recovery=False)
    assert (lcd.connected, lcd.allocated, lcd.pointer, lcd.address) == (
        True,
        True,
        True,
        0x27,
    )
    assert lcd.successful_recoveries == 0

    # Reattaching the live object is a no-op rather than reconstruction.
    lcd.attach(0x27, recovery=True)
    assert lcd.successful_recoveries == 0

    lcd.detach()
    lcd.detach()
    assert (lcd.connected, lcd.allocated, lcd.pointer, lcd.address) == (
        False,
        False,
        False,
        0,
    )

    lcd.attach(0x27, recovery=True)
    assert lcd.connected
    assert lcd.successful_recoveries == 1
    lcd.attach(0x27, recovery=True)
    assert lcd.successful_recoveries == 1


def test_firmware_lcd_lifecycle_has_one_owner_and_truthful_counter():
    source = _firmware()
    detach = _function_body(
        source,
        "static void lcdDetach(unsigned long now, bool scheduleRetry, const char* reason)",
    )
    attach = _function_body(
        source,
        "static bool lcdAttach(uint8_t addr, unsigned long now, bool recovery)",
    )
    scan = _function_body(source, "bool scanForLCD(bool recovery)")
    reinit = _function_body(source, "void lcdReInit()")

    assert "allocatedObject->~LiquidCrystal_I2C();" in detach
    assert "lcdPtr = nullptr;" in detach
    assert "lcdObjAllocated = false;" in detach
    assert "lcdConnected = false;" in detach
    assert "if (scheduleRetry && wasAttached)" in detach

    assert "lcdDetach(now, false, nullptr);" in attach
    assert "lcdPtr = new (lcdObjBuf) LiquidCrystal_I2C" in attach
    assert "lcdObjAllocated = true;" in attach
    assert "lcdConnected = true;" in attach
    assert "if (recovery) diagLcdReinitCount++;" in attach
    assert source.count("diagLcdReinitCount++") == 1

    assert "lcdAttach(addr, millis(), recovery)" in scan
    assert "return attached;" in scan
    assert source.count("alignas(LiquidCrystal_I2C) static uint8_t lcdObjBuf") == 1
    assert scan.count("Wire.setTimeOut(50);") == 1
    assert scan.count("Wire.setTimeOut(100);") == 1
    assert scan.count("return ") == 1
    assert source.count("new (lcdObjBuf) LiquidCrystal_I2C") == 1
    assert "lcdConnected=scanForLCD" not in source
    assert "lcdConnected = scanForLCD" not in source
    assert "scanForLCD(false);" in source
    assert "scanForLCD(true);" in source
    assert 'lcdDetach(now, true, "runtime_probe_failed");' in source
    for lifecycle_body in (detach, attach, scan, reinit):
        assert "delay(" not in lifecycle_body


def test_presence_fsm_globally_exits_when_radar_packets_expire():
    source = _firmware()
    loop = _function_body(source, "void loop()")

    guard = loop.index("if (radarPacketExpired) {")
    transition = loop.index(
        'enterPresenceState(PRESENCE_ABSENT, now, "radar_packet_stale");'
    )
    state_switch = loop.index("switch (presenceState) {", guard)

    assert "lastRadarPresencePacketMs > 0 && !isPresenceFreshAt(now)" in loop
    assert "tickStrongEvidence = false;" in loop[guard:state_switch]
    assert "tickWeakEvidence = false;" in loop[guard:state_switch]
    assert 'handlePersonLeft(now, "radar_packet_stale");' in loop[guard:state_switch]
    assert guard < transition < state_switch


def test_firmware_version_capture_is_bounded_and_rearmable():
    capture = _FirmwareVersionCaptureModel()
    assert capture.state == "idle"

    capture.arm(1_000, 1_500)
    capture.service(2_499, version_available=False)
    assert capture.state == "armed"
    capture.service(2_500, version_available=False)
    assert capture.state == "expired"
    assert not capture.version_valid

    capture.arm(3_000, 1_500)
    capture.service(3_200, version_available=True)
    assert capture.state == "captured"
    assert capture.version_valid

    # A parser restart always rearms capture, but expiration does not erase a
    # previously observed module identity.
    capture.arm(4_000, 1_500)
    assert capture.state == "armed"
    capture.service(5_500, version_available=False)
    assert capture.state == "expired"
    assert capture.version_valid


def test_firmware_version_capture_state_machine_never_waits():
    source = _firmware()
    arm = _function_body(
        source,
        "static void armModuleFirmwareVersionCapture(unsigned long now)",
    )
    service = _function_body(
        source,
        "static void serviceModuleFirmwareVersionCapture(unsigned long now)",
    )
    setup_pump = _function_body(
        source,
        "static void pumpModuleFirmwareVersionCaptureDuringSetup()",
    )
    setup = _function_body(source, "void setup()")
    loop = _function_body(source, "void loop()")

    for state in ("IDLE", "ARMED", "CAPTURED", "EXPIRED"):
        assert f"MODULE_FW_CAPTURE_{state}" in source
    assert "MODULE_FW_CAPTURE_WINDOW_MS = 1500UL" in source
    assert "moduleFwCaptureState = MODULE_FW_CAPTURE_ARMED;" in arm
    assert "moduleFwCaptureDeadlineMs = now + MODULE_FW_CAPTURE_WINDOW_MS;" in arm
    assert "moduleFwCaptureState = MODULE_FW_CAPTURE_CAPTURED;" in service
    assert "moduleFwCaptureState = MODULE_FW_CAPTURE_EXPIRED;" in service

    assert "mmWave.update(5);" not in service
    assert "mmWave.update(5);" in setup_pump
    for state_machine_body in (arm, service, setup_pump):
        assert "while (" not in state_machine_body
        assert "delay(" not in state_machine_body

    assert source.count("mmWave.begin(&mmWaveSerial);") == 2
    assert source.count("armModuleFirmwareVersionCapture(") == 3
    assert "pumpModuleFirmwareVersionCaptureDuringSetup();" in setup
    assert "pumpModuleFirmwareVersionCaptureDuringSetup();" not in loop
    assert "serviceModuleFirmwareVersionCapture(now);" in source
    assert "pollModuleFirmwareVersionWindow" not in source
    assert "pollModuleFirmwareVersionNonBlocking" not in source
    assert "fwVersionCheckedThisBoot" not in source
