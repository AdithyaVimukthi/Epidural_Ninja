"""
Make the app icon from packaging/icon/source.png (a black robot arm on white):
the arm cut out by darkness (anti-aliased edges kept), on a light rounded tile
so it reads on dark and light docks alike. Writes openarm-control.png (512 px)
and the hicolor sizes the installer copies into ~/.local/share/icons.

    ../../../.venv/bin/python make_icon.py        (Pillow; it comes with matplotlib)
"""
import os

from PIL import Image, ImageChops, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
SIZE = 512
GLYPH = (30, 32, 39)          # the GUI's dark background colour
TILE = (244, 246, 248)
BORDER = (196, 202, 212)
SIZES = (16, 24, 32, 48, 64, 128, 256, 512)


def glyph(src: Image.Image) -> Image.Image:
    lum = src.convert("L")
    alpha = lum.point(lambda v: 0 if v > 235 else 255 - v)        # white -> clear, black -> solid
    if "A" in src.getbands():                                      # and clear stays clear
        alpha = ImageChops.multiply(alpha, src.getchannel("A"))
    bbox = alpha.point(lambda v: 255 if v > 60 else 0).getbbox()  # ignore the faint floor shadow
    alpha = alpha.crop(bbox)
    out = Image.new("RGBA", alpha.size, GLYPH + (0,))
    out.putalpha(alpha)
    return out


def main() -> None:
    src = Image.open(os.path.join(HERE, "source.png")).convert("RGBA")
    g = glyph(src)
    scale = 4                                   # draw big, shrink: smooth corners
    S = SIZE * scale
    tile = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    d = ImageDraw.Draw(tile)
    pad, radius = 16 * scale, 104 * scale
    shadow = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    ImageDraw.Draw(shadow).rounded_rectangle((pad, pad + 6 * scale, S - pad, S - pad + 6 * scale), radius,
                                             fill=(0, 0, 0, 70))
    tile = Image.alpha_composite(tile, shadow.filter(ImageFilter.GaussianBlur(8 * scale)))
    d = ImageDraw.Draw(tile)
    d.rounded_rectangle((pad, pad, S - pad, S - pad), radius, fill=TILE + (255,), outline=BORDER + (255,),
                        width=4 * scale)
    inner = S - 2 * pad - 2 * 58 * scale        # glyph box inside the tile
    k = min(inner / g.width, inner / g.height)
    gw, gh = int(g.width * k), int(g.height * k)
    gl = g.resize((gw, gh), Image.LANCZOS)
    tile.alpha_composite(gl, ((S - gw) // 2, (S - gh) // 2 + 6 * scale))
    icon = tile.resize((SIZE, SIZE), Image.LANCZOS)
    icon.save(os.path.join(HERE, "openarm-control.png"))
    for s in SIZES:
        os.makedirs(os.path.join(HERE, "hicolor", f"{s}x{s}", "apps"), exist_ok=True)
        icon.resize((s, s), Image.LANCZOS).save(os.path.join(HERE, "hicolor", f"{s}x{s}", "apps",
                                                             "openarm-control.png"))
    print("wrote openarm-control.png and", ", ".join(f"{s}px" for s in SIZES))


if __name__ == "__main__":
    main()
