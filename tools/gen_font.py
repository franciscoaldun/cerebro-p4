# Genera main/font.h: glifos A8 (alfa 0-255) de Consolas Bold para el HUD del P4.
# Uso: python tools/gen_font.py
from PIL import Image, ImageDraw, ImageFont
import os

W, H, SIZE = 12, 24, 20
FONT = r"C:\Windows\Fonts\consolab.ttf"
EXTRA = "°áéíóúñÑ¿¡·→"
chars = [chr(c) for c in range(32, 127)] + list(EXTRA)

font = ImageFont.truetype(FONT, SIZE)
asc, desc = font.getmetrics()
top = (H - (asc + desc)) // 2

out = []
out.append("// Generado por tools/gen_font.py - Consolas Bold %dpx, glifos %dx%d A8\n" % (SIZE, W, H))
out.append("#pragma once\n#include <stdint.h>\n")
out.append("#define FONT_W %d\n#define FONT_H %d\n#define FONT_N %d\n" % (W, H, len(chars)))
out.append("static const uint32_t font_cp[FONT_N] = {%s};\n" % ",".join(str(ord(c)) for c in chars))
out.append("static const uint8_t font_a8[FONT_N][FONT_W*FONT_H] = {\n")
for ch in chars:
    img = Image.new("L", (W, H), 0)
    d = ImageDraw.Draw(img)
    bbox = font.getbbox(ch)
    gw = bbox[2] - bbox[0]
    x = (W - gw) // 2 - bbox[0]
    d.text((x, top), ch, font=font, fill=255)
    px = list(img.getdata())
    out.append("  {" + ",".join(str(p) for p in px) + "},\n")
out.append("};\n")

dst = os.path.join(os.path.dirname(__file__), "..", "main", "font.h")
with open(dst, "w", encoding="utf-8") as f:
    f.write("".join(out))
print("font.h:", len(chars), "glifos", W, "x", H)
