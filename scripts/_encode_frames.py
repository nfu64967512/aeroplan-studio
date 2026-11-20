"""把錄幀資料夾編成 mp4（PyAV）。

    python scripts/_encode_frames.py <frames_dir> <out.mp4> [fps]

⚠️ 這台機器**沒有 cv2**，但有 PyAV 18.1.0。不要照抄別處用 cv2.VideoWriter
   的寫法 —— 那會在第一行就 ImportError。

⚠️ libx264 不吃奇數邊長，而本 App 在 HiDPI 下的視窗尺寸經常是奇數
   （例如 1920x1080 會被 DPR 放大成 2400x1350，但別的尺寸就不一定）。
   奇數會變成一個看不懂的外部函式庫錯誤，不是有用的訊息。所以一律先裁到偶數。
"""
from __future__ import annotations

import sys
from pathlib import Path

import av
import numpy as np
from PIL import Image

TARGET_W = 1600          # 等比縮小，控制檔案大小與編碼時間


def main() -> int:
    src = Path(sys.argv[1])
    out = Path(sys.argv[2])
    fps = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

    files = sorted(src.glob("*.png"))
    if not files:
        print("無幀可編碼:", src)
        return 1

    first = Image.open(files[0]).convert("RGB")
    scale = min(1.0, TARGET_W / first.width)
    w = int(first.width * scale)
    h = int(first.height * scale)
    w -= w % 2                      # ⚠️ libx264：邊長必須是偶數
    h -= h % 2

    container = av.open(str(out), mode="w")
    stream = container.add_stream("libx264", rate=int(round(fps)))
    stream.width = w
    stream.height = h
    stream.pix_fmt = "yuv420p"
    stream.options = {"crf": "23", "preset": "medium"}

    n = 0
    for f in files:
        try:
            im = Image.open(f).convert("RGB")
        except Exception:
            continue                # 半寫入的幀直接跳過，不要讓整支掛掉
        if (im.width, im.height) != (w, h):
            im = im.resize((w, h), Image.LANCZOS)
        frame = av.VideoFrame.from_ndarray(np.asarray(im), format="rgb24")
        for pkt in stream.encode(frame):
            container.mux(pkt)
        n += 1

    for pkt in stream.encode():
        container.mux(pkt)
    container.close()

    mb = out.stat().st_size / 1e6
    print("編碼 %d 幀 -> %s  (%dx%d @ %.0f fps, %.1f MB, %.1f 秒)"
          % (n, out, w, h, fps, mb, n / fps))
    return 0


if __name__ == "__main__":
    sys.exit(main())
