"""把擬合後的 UTC、位置、速度寫進影像 EXIF／XMP（輸出到另一個目錄，原檔不動）。

為什麼一定要改寫（評估 §8 陷阱一、phase2_verify V5 #1）
=======================================================
ODM／ODX 的滾動快門校正（OpenSfM ``correct_rolling_shutter.py:36-86``）在**任何一張**照片沒有
速度時，就把整個重建的照片依 EXIF 拍攝時間排序、用相鄰兩張估速度；相鄰兩張是不同相機時
速度設 0（"# Change of camera, assume 0"）。三機同時飛、每台 Model 唯一 ⇒ 排序後交錯 ⇒
大多數照片速度為 0，實測的讀出時間等於被靜默吃掉。對策：每張都寫速度，時間也寫對。

ODM 實際讀哪些標籤（OpenDroneMap/ODM master opendm/photo.py，2026-10-14 取得）
-------------------------------------------------------------------------------
* 時間：``EXIF DateTimeOriginal`` ＋ ``EXIF SubSecTime``（:271-288，注意是 SubSecTime 不是
  SubSecTimeOriginal），當作 UTC。小數秒的算法是 ``float('0.{}'.format(int(subsec)))``：
  "050"（50 ms）會讀成 0.5 s、"007" 讀成 0.7 s。⇒ 本模組把時間量化到 0.1 s，小數秒**只寫一位
  數字**（"0"–"9"），任何寫法都不會有前導零。SubSecTimeOriginal／Digitized 寫同一個值，
  OpenSfM 自己的 exif.py（:524-537，``"%Y:%m:%d %H:%M:%S.%f"``）也解成同一個時刻。
  量化 ≤50 ms 只影響排序；速度另外直接給，不靠時間差估。
* 速度：XMP ``@drone-dji:FlightXSpeed/YSpeed/ZSpeed``（:413-425，三個都在才讀），以
  rdf:Description 的**屬性**形式（xmltodict 的 '@' 前綴）；``to_opensfm_exif``（:888-890）寫成
  ``d['speed'] = [speed_y, speed_x, speed_z]``，只在有 GPS 時才寫。OpenSfM 的重建座標是以
  reference_lla 為原點的區域 ENU（x 東、y 北、z 上），correct_rolling_shutter 直接拿它和
  相機中心相減 ⇒ **FlightXSpeed = 北向、FlightYSpeed = 東向、FlightZSpeed = 向上**（m/s）。
  DJI 自己的 Z 據稱是向下為正（未證實）；我們以 ODM 這個唯一的讀者為準寫「向上」。
  ``get_xmp`` 以 ``b'<x:xmpmeta'`` 搜尋 ⇒ 前綴必須正好是 x:、rdf:、drone-dji:。
* 焦距先驗：``extract_focal``（:518-559）有 FocalLengthIn35mmFilm 就優先用它；否則用 FocalLength
  ÷ 感測器寬（由 FocalPlaneXResolution 與 FocalPlaneResolutionUnit 換算），都沒有就退回 0.85。
  ⇒ 寫 FocalLength、FocalPlaneX/YResolution（單位 3 = cm），並刪掉 Pi 可能寫的 35 mm 等效值，
  讓實測的像素間距生效。
* 相機分組：``camera_id()`` = make + model + 寬高 + 投影 + focal_ratio ⇒ 每台唯一的 Model 讓
  ODM 分別率定內參。

寫法：用 Pillow 的 EXIF 編碼器產生 APP1，直接替換 JPEG 標頭段，**影像壓縮資料逐位元組不動**
（用 Pillow 重存會重新壓縮，攝影測量不能接受）。
"""

from __future__ import annotations

import io
import math
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

from PIL import Image
from PIL.TiffImagePlugin import IFDRational

from survey import datum
from survey.geotag import export_filename

SUBSEC_RESOLUTION_S = 0.1
_EPOCH = datetime(1970, 1, 1, tzinfo=timezone.utc)
_XMP_HEADER = b"http://ns.adobe.com/xap/1.0/\x00"
_XMP_EXT_HEADER = b"http://ns.adobe.com/xmp/extension/\x00"
_EXIF_HEADER = b"Exif\x00\x00"

# EXIF 標籤
TAG_MAKE, TAG_MODEL, TAG_DATETIME = 0x010F, 0x0110, 0x0132
IFD_EXIF, IFD_GPS = 0x8769, 0x8825
TAG_DTO, TAG_DTD = 0x9003, 0x9004
TAG_OFFSET, TAG_OFFSET_O, TAG_OFFSET_D = 0x9010, 0x9011, 0x9012
TAG_SUBSEC, TAG_SUBSEC_O, TAG_SUBSEC_D = 0x9290, 0x9291, 0x9292
TAG_FOCAL, TAG_FOCAL35 = 0x920A, 0xA405
TAG_FPXRES, TAG_FPYRES, TAG_FPUNIT = 0xA20E, 0xA20F, 0xA210


def exif_time_fields(utc: datetime):
    """UTC → (DateTimeOriginal 字串, SubSec 一位數字字串)。先量化到 0.1 s 再拆（進位正確處理）。"""
    if utc.tzinfo is None:
        raise ValueError("utc 必須含時區")
    us = (utc - _EPOCH) // timedelta(microseconds=1)
    ds = (us + 50_000) // 100_000                    # 四捨五入到 0.1 s
    base = _EPOCH + timedelta(seconds=ds // 10)
    return base.strftime("%Y:%m:%d %H:%M:%S"), str(int(ds % 10))


def _rat(x: float, den: int) -> IFDRational:
    return IFDRational(int(round(abs(x) * den)), den)


def _dms(deg: float):
    deg = abs(deg)
    d = int(deg)
    m_full = (deg - d) * 60.0
    m = int(m_full)
    s = (m_full - m) * 60.0
    if round(s, 6) >= 60.0:            # 進位
        s, m = 0.0, m + 1
    if m >= 60:
        m, d = m - 60, d + 1
    return (IFDRational(d, 1), IFDRational(m, 1), _rat(s, 1_000_000))


def build_xmp(speed_n: float, speed_e: float, speed_up: float, *, image_id: str = "") -> bytes:
    """只含 ODM 會讀的速度屬性（加我們自己的影像代號方便追溯）。"""
    for v in (speed_n, speed_e, speed_up):
        if v is None or not math.isfinite(v):
            raise ValueError(f"速度必須是有限數值，收到 {(speed_n, speed_e, speed_up)}")
    body = (
        '<x:xmpmeta xmlns:x="adobe:ns:meta/">'
        '<rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">'
        '<rdf:Description rdf:about=""'
        ' xmlns:drone-dji="http://www.dji.com/drone-dji/1.0/"'
        ' xmlns:aeroplan="https://aeroplan.local/ns/survey/1.0/"'
        f' drone-dji:FlightXSpeed="{speed_n:.3f}"'
        f' drone-dji:FlightYSpeed="{speed_e:.3f}"'
        f' drone-dji:FlightZSpeed="{speed_up:.3f}"'
        f' aeroplan:ImageId="{image_id}"/>'
        '</rdf:RDF></x:xmpmeta>'
    )
    return ('<?xpacket begin="﻿" id="W5M0MpCehiHzreSzNTczkc9d"?>' + body
            + '<?xpacket end="w"?>').encode("utf-8")


def _segments(data: bytes):
    """切出 SOS 之前的標頭段 [(marker, payload)] 與 SOS 起的其餘位元組。"""
    if data[:2] != b"\xff\xd8":
        raise ValueError("不是 JPEG（缺 SOI）")
    pos, segs = 2, []
    while True:
        if pos >= len(data) or data[pos] != 0xFF:
            raise ValueError(f"JPEG 標頭在 {pos} 位元組處損毀")
        while data[pos] == 0xFF:
            pos += 1
        marker = data[pos]
        pos += 1
        if marker == 0xDA:                               # SOS：之後是壓縮資料，原封不動
            return segs, data[pos - 2:]
        if marker == 0x01 or 0xD0 <= marker <= 0xD7:
            segs.append((marker, b""))
            continue
        length = int.from_bytes(data[pos:pos + 2], "big")
        segs.append((marker, data[pos + 2:pos + length]))
        pos += length


def _app(marker: int, payload: bytes) -> bytes:
    if len(payload) + 2 > 0xFFFF:
        raise ValueError(f"APP{marker - 0xE0} 段太長（{len(payload)} B）")
    return bytes([0xFF, marker]) + (len(payload) + 2).to_bytes(2, "big") + payload


@dataclass(frozen=True)
class CameraExif:
    make: str
    model: str                 # 每台唯一（session.json unit_model）
    focal_mm: float
    pixel_um: float
    width: int
    height: int

    @classmethod
    def from_session(cls, session, make: str):
        c = session.camera
        return cls(make=make, model=session.unit_model, focal_mm=float(c["lens_mm"]),
                   pixel_um=float(c["pixel_um"]), width=int(c["width"]), height=int(c["height"]))


def tag_jpeg_bytes(data: bytes, row: dict, cam: CameraExif, *, height: str) -> bytes:
    """回傳改好標籤的 JPEG 位元組。``height`` 選寫入 GPSAltitude 的高度（與 geo.txt 一致）。"""
    key = {"ellipsoidal": "h_ell_m", "orthometric": "h_orth_m"}.get(height)
    if key is None:
        raise ValueError(f"height 必須是 'ellipsoidal' 或 'orthometric'，收到 {height!r}")
    need = ["lat_deg", "lon_deg", key, "t_gpst_s", "speed_n_mps", "speed_e_mps", "speed_d_mps"]
    missing = [k for k in need if row.get(k) is None]
    if missing:
        raise ValueError(f"{row.get('image_id')}: 缺 {missing}（ODM 只要有一張沒速度就整個重建改用估計值）")
    if data[:2] != b"\xff\xd8":
        raise ValueError(f"{row.get('image_id')}: 不是 JPEG（缺 SOI）")
    with Image.open(io.BytesIO(data)) as im:
        if im.format != "JPEG":
            raise ValueError(f"{row.get('image_id')}: 不是 JPEG")
        if im.size != (cam.width, cam.height):
            raise ValueError(f"{row.get('image_id')}: 影像 {im.size} 與 session.json 的 {cam.width}x{cam.height} "
                             "不符（像素間距會錯）")
        exif = im.getexif()
    utc = datum.gpst_to_utc(row["t_gpst_s"])
    dto, subsec = exif_time_fields(utc)
    exif[TAG_MAKE] = cam.make
    exif[TAG_MODEL] = cam.model
    exif[TAG_DATETIME] = dto
    e = exif.get_ifd(IFD_EXIF)
    e[TAG_DTO] = dto
    e[TAG_DTD] = dto
    for t in (TAG_SUBSEC, TAG_SUBSEC_O, TAG_SUBSEC_D):
        e[t] = subsec
    for t in (TAG_OFFSET, TAG_OFFSET_O, TAG_OFFSET_D):
        e[t] = "+00:00"
    e[TAG_FOCAL] = _rat(cam.focal_mm, 1000)
    px_per_cm = 10_000.0 / cam.pixel_um
    e[TAG_FPXRES] = _rat(px_per_cm, 1000)
    e[TAG_FPYRES] = _rat(px_per_cm, 1000)
    e[TAG_FPUNIT] = 3
    e.pop(TAG_FOCAL35, None)
    g = exif.get_ifd(IFD_GPS)
    g.clear()
    lat, lon, alt = row["lat_deg"], row["lon_deg"], row[key]
    g[0] = b"\x02\x03\x00\x00"
    g[1] = "N" if lat >= 0 else "S"
    g[2] = _dms(lat)
    g[3] = "E" if lon >= 0 else "W"
    g[4] = _dms(lon)
    g[5] = b"\x00" if alt >= 0 else b"\x01"           # 0：海平面以上（ODM：Ref>0 就取負）
    g[6] = _rat(alt, 1000)
    t_of_day = utc - utc.replace(hour=0, minute=0, second=0, microsecond=0)
    sec = t_of_day.total_seconds()
    g[7] = (IFDRational(int(sec // 3600), 1), IFDRational(int(sec % 3600 // 60), 1), _rat(sec % 60, 1000))
    g[29] = utc.strftime("%Y:%m:%d")
    if row.get("yaw_deg") is not None:
        g[16] = "T"                                     # 真北
        g[17] = _rat(row["yaw_deg"] % 360.0, 100)
    exif_payload = exif.tobytes()
    if not exif_payload.startswith(_EXIF_HEADER):
        exif_payload = _EXIF_HEADER + exif_payload
    xmp = build_xmp(row["speed_n_mps"], row["speed_e_mps"], -row["speed_d_mps"], image_id=row.get("image_id", ""))

    segs, rest = _segments(data)
    kept = [(m, p) for m, p in segs
            if not (m == 0xE1 and (p.startswith(_EXIF_HEADER) or p.startswith(_XMP_HEADER)
                                   or p.startswith(_XMP_EXT_HEADER)))]
    n_app0 = 0
    while n_app0 < len(kept) and kept[n_app0][0] == 0xE0:
        n_app0 += 1
    out = bytearray(b"\xff\xd8")
    for m, p in kept[:n_app0]:
        out += _app(m, p)
    out += _app(0xE1, exif_payload)
    out += _app(0xE1, _XMP_HEADER + xmp)
    for m, p in kept[n_app0:]:
        out += bytes([0xFF, m]) if (m == 0x01 or 0xD0 <= m <= 0xD7) else _app(m, p)
    out += rest
    return bytes(out)


def export_tagged_images(session, rows, out_dir, *, make: str, height: str, overwrite: bool = False) -> list:
    """把會期裡的影像加上標籤寫到 ``out_dir/<export_filename>``。回傳寫出的路徑。"""
    out = Path(out_dir)
    src_root = Path(session.path).resolve()
    if out.resolve() in (src_root, (src_root / "images").resolve()):
        raise ValueError("輸出目錄不能是會期目錄或其影像目錄（原檔不可改）")
    out.mkdir(parents=True, exist_ok=True)
    cam = CameraExif.from_session(session, make)
    written = []
    for r in rows:
        src = (src_root / r["file"]).resolve()
        dst = (out / export_filename(r)).resolve()
        if dst == src:
            raise ValueError(f"{r['image_id']}: 輸出會覆蓋原檔")
        if dst.exists() and not overwrite:
            raise FileExistsError(f"{dst} 已存在（overwrite=False）")
        dst.write_bytes(tag_jpeg_bytes(src.read_bytes(), r, cam, height=height))
        written.append(dst)
    return written
