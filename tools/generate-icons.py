#!/usr/bin/env python3
"""Generate PWA PNG icons from assets/icons/icon.svg.

Outputs:
  assets/icons/icon-192.png            (192x192, transparent corners via SVG)
  assets/icons/icon-512.png            (512x512)
  assets/icons/icon-maskable-512.png   (512x512, safe-zone padded — SVG rendered inside the 80% inner circle)
  assets/icons/apple-touch-icon-180.png

Re-run whenever icon.svg changes:
  python3 tools/generate-icons.py
"""
from pathlib import Path
import cairosvg
from PIL import Image
import io

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "assets" / "icons" / "icon.svg"
OUT = ROOT / "assets" / "icons"


def render(size: int) -> Image.Image:
    png_bytes = cairosvg.svg2png(url=str(SRC), output_width=size, output_height=size)
    return Image.open(io.BytesIO(png_bytes)).convert("RGBA")


def render_maskable(size: int) -> Image.Image:
    # Maskable icons need the actual logo inside the inner 80% safe zone with the
    # outer 10% on each side being part of a continuous background fill.
    safe = int(size * 0.78)
    inner = render(safe)
    bg = Image.new("RGBA", (size, size), (59, 130, 246, 255))  # solid theme color
    pad = (size - safe) // 2
    bg.paste(inner, (pad, pad), inner)
    return bg


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    targets = [
        (192, "icon-192.png", render),
        (512, "icon-512.png", render),
        (512, "icon-maskable-512.png", render_maskable),
        (180, "apple-touch-icon-180.png", render),
    ]
    for size, name, fn in targets:
        img = fn(size)
        img.save(OUT / name, "PNG", optimize=True)
        print(f"wrote {OUT / name}  ({size}x{size}, {img.size[0]*img.size[1]} px, {len(img.tobytes())//1024} KB raw)")


if __name__ == "__main__":
    main()
