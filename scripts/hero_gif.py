#!/usr/bin/env python3
"""Tiles gallery GIFs into one looping README hero GIF (needs Pillow).

    python3 scripts/hero_gif.py OUT.gif TITLE file.gif:CAPTION [file.gif:CAPTION ...]

Each source GIF loses its title bar, is fitted into a tile and plays at its own
speed (looping) on a shared 10 fps clock; 3 tiles per row.
"""
import sys
from PIL import Image, ImageDraw, ImageFont

TW, TH, CAP, HEAD, COLS = 320, 196, 22, 34, 3
FPS, SECONDS, CROP_TOP = 10, 12, 31
INK, BG = (40, 40, 40), (255, 255, 255)


def font(size):
    for f in ["DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"]:
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            pass
    return ImageFont.load_default()


def load(path):
    im = Image.open(path)
    frames, t = [], 0
    for i in range(im.n_frames):
        im.seek(i)
        d = im.info.get("duration", 60) or 60
        f = im.convert("RGB")
        f = f.crop((0, CROP_TOP, f.width, f.height))
        f.thumbnail((TW, TH - CAP), Image.LANCZOS)
        frames.append((t, f))
        t += d
    return frames, t


def at(frames, total, ms):
    ms %= total
    cur = frames[0][1]
    for t, f in frames:
        if t > ms:
            break
        cur = f
    return cur


def main():
    out, title, items = sys.argv[1], sys.argv[2], sys.argv[3:]
    srcs = [(load(p), cap) for p, cap in (i.split(":", 1) for i in items)]
    rows = (len(srcs) + COLS - 1) // COLS
    W, H = COLS * TW, HEAD + rows * TH
    fh, fc = font(18), font(13)
    frames = []
    for k in range(FPS * SECONDS):
        ms = k * 1000 // FPS
        c = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(c)
        d.rectangle((0, 0, W, HEAD), fill=INK)
        d.text((12, 7), title, font=fh, fill=BG)
        for i, ((fr, total), cap) in enumerate(srcs):
            x, y = (i % COLS) * TW, HEAD + (i // COLS) * TH
            f = at(fr, total, ms)
            c.paste(f, (x + (TW - f.width) // 2, y + (TH - CAP - f.height) // 2))
            d.text((x + TW // 2, y + TH - CAP // 2 - 1), cap, font=fc, fill=INK, anchor="mm")
            d.rectangle((x, y, x + TW - 1, y + TH - 1), outline=(215, 215, 215))
        frames.append(c)
    pal = frames[0].quantize(colors=255, method=Image.MEDIANCUT)
    # palette from a strip of frames spread over the loop
    strip = Image.new("RGB", (W, H * 4))
    for j in range(4):
        strip.paste(frames[j * len(frames) // 4], (0, j * H))
    pal = strip.quantize(colors=255, method=Image.MEDIANCUT)
    q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    q[0].save(out, save_all=True, append_images=q[1:], duration=1000 // FPS, loop=0, optimize=True, disposal=1)


if __name__ == "__main__":
    main()
