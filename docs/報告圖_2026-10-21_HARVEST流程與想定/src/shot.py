import sys, asyncio
from pathlib import Path
from playwright.sync_api import sync_playwright
out = Path(sys.argv[1]); scale = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
with sync_playwright() as p:
    b = p.chromium.launch()
    for svg in sorted(out.glob("fig*.svg")):
        if svg.name.endswith(".css.svg"):
            continue
        txt = svg.read_text(encoding="utf-8")
        import re
        m = re.search(r'viewBox="0 0 (\d+) (\d+)"', txt); w, h = int(m.group(1)), int(m.group(2))
        pg = b.new_page(viewport={"width": w, "height": h}, device_scale_factor=scale)
        pg.set_content(f'<html><body style="margin:0">{txt}</body></html>')
        pg.wait_for_timeout(200)
        pg.screenshot(path=str(svg.with_suffix(".png")), clip={"x": 0, "y": 0, "width": w, "height": h})
        pg.close()
    b.close()
print("ok")
