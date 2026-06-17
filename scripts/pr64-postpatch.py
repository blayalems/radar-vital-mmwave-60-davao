from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OLD = "Record<string, string | number>"
NEW = "Record<string, string | number | null | undefined>"

expected = {
    "web/src/app/services/i18n.service.ts": 3,
    "web/src/app/i18n/translate.pipe.ts": 1,
}

for relative, expected_count in expected.items():
    path = ROOT / relative
    text = path.read_text(encoding="utf-8")
    count = text.count(OLD)
    if count != expected_count:
        raise RuntimeError(
            f"{relative}: expected {expected_count} interpolation parameter types, found {count}"
        )
    path.write_text(text.replace(OLD, NEW), encoding="utf-8")

print("Expanded i18n interpolation parameter types for optional template values.")
