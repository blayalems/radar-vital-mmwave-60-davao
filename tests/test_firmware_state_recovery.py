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
        "isPresenceFreshAt(now);"
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
