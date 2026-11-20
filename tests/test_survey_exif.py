"""survey.exif_write：ODM 讀得懂的時間／速度、無損改寫、原檔不動。

ODM 解析式照抄自 OpenDroneMap/ODM opendm/photo.py:271-295、413-425、888-890（見 exif_write 模組說明）。
環境沒有 exifread／xmltodict，這裡用 Pillow 讀標籤、用 minidom 模擬 xmltodict 的 '@prefix:name' 屬性鍵。
"""

import hashlib
import io
from datetime import datetime, timedelta, timezone
from xml.dom import minidom

import numpy as np
import pytest
from PIL import Image

from survey import datum, exif_write, synth
from survey.exif_write import CameraExif, exif_time_fields, tag_jpeg_bytes
from survey.pi_session import read_pi_session

CAM = CameraExif(make="Raspberry Pi", model="RPiHQ6-H2", focal_mm=6.0, pixel_um=1.55, width=64, height=48)


def odm_subsec_seconds(subsec: str) -> float:
    """ODM photo.py:276-282 的寫法（int 後再接在 '0.' 後面）。"""
    v = int(subsec)
    negative = 1.0
    if v < 0:
        negative, v = -1.0, -v
    return float("0.{}".format(int(v))) * negative


def _row(**kw):
    utc = datetime(2026, 10, 14, 3, 4, 5, 260000, tzinfo=timezone.utc)
    r = {"image_id": "hero2_S_000001", "file": "images/000001.jpg", "lat_deg": 25.1234567891,
         "lon_deg": 121.2345678912, "h_ell_m": 168.543, "h_orth_m": 150.003, "t_gpst_s": datum.utc_to_gpst(utc),
         "yaw_deg": 95.25, "speed_n_mps": 1.25, "speed_e_mps": 21.5, "speed_d_mps": -0.3}
    r.update(kw)
    return r


def _pi_jpeg():
    """仿 Pi 輸出：已有 EXIF（Pi 時鐘、有前導零的 SubSec、35 mm 等效焦距）與一段 XMP。"""
    arr = np.random.default_rng(0).integers(0, 255, (48, 64, 3), dtype=np.uint8)
    ex = Image.Exif()
    ex[0x010F] = "RaspberryPi"
    ex[0x0110] = "imx477"
    e = ex.get_ifd(0x8769)
    e[0x9003] = "1970:01:01 00:00:10"
    e[0x9291] = "050"
    e[0xA405] = 26
    buf = io.BytesIO()
    Image.fromarray(arr, "RGB").save(buf, "JPEG", quality=92, exif=ex.tobytes(),
                                     xmp=b'<x:xmpmeta xmlns:x="adobe:ns:meta/"><old/></x:xmpmeta>')
    return buf.getvalue()


def test_subsec_never_has_leading_zero_and_parses_correctly():
    # 先示範陷阱：毫秒三位數寫法在 ODM 會錯
    assert odm_subsec_seconds("050") == 0.5 and odm_subsec_seconds("007") == 0.7
    prev = None
    base = datetime(2026, 10, 14, 3, 4, 5, tzinfo=timezone.utc)
    for ms in range(0, 1000):
        utc = base + timedelta(milliseconds=ms)
        dto, sub = exif_time_fields(utc)
        assert len(sub) == 1 and sub.isdigit()
        t = datetime.strptime(dto, "%Y:%m:%d %H:%M:%S").replace(tzinfo=timezone.utc) + \
            timedelta(seconds=odm_subsec_seconds(sub))
        assert abs((t - utc).total_seconds()) <= 0.05 + 1e-9
        # OpenSfM exif.py 的 "%Y:%m:%d %H:%M:%S.%f" 解出同一時刻
        t2 = datetime.strptime(f"{dto}.{sub}", "%Y:%m:%d %H:%M:%S.%f").replace(tzinfo=timezone.utc)
        assert t2 == t
        assert prev is None or t >= prev                 # 量化是單調的 ⇒ 排序不變
        prev = t
    assert exif_time_fields(base.replace(hour=23, minute=59, second=59) + timedelta(milliseconds=970)) == \
        ("2026:10:15 00:00:00", "0")
    assert exif_time_fields(base + timedelta(milliseconds=260)) == ("2026:10:14 03:04:05", "3")
    with pytest.raises(ValueError):
        exif_time_fields(datetime(2026, 1, 1))


def _xmp_attrs(data: bytes) -> dict:
    """模擬 ODM get_xmp：找 b'<x:xmpmeta' 到 b'</x:xmpmeta'，取 rdf:Description 的屬性（xmltodict 鍵 = '@'+名稱）。"""
    assert data.count(b"<x:xmpmeta") == 1
    s, e = data.find(b"<x:xmpmeta"), data.find(b"</x:xmpmeta")
    doc = minidom.parseString(data[s:e + 12].decode("utf-8"))
    desc = doc.getElementsByTagName("rdf:Description")[0]
    return {"@" + k: v for k, v in desc.attributes.items()}


def test_tags_written_as_odm_reads_them():
    src = _pi_jpeg()
    out = tag_jpeg_bytes(src, _row(), CAM, height="ellipsoidal")
    with Image.open(io.BytesIO(out)) as im:
        ex = im.getexif()
        e, g = ex.get_ifd(0x8769), ex.get_ifd(0x8825)
    assert ex[0x010F] == "Raspberry Pi" and ex[0x0110] == "RPiHQ6-H2"
    assert e[0x9003] == "2026:10:14 03:04:05"
    assert e[0x9290] == "3" and e[0x9291] == "3"        # ODM 讀 SubSecTime（0x9290），不是 Original
    assert e[0x9011] == "+00:00"
    assert float(e[0x920A]) == 6.0
    assert float(e[0xA20E]) == pytest.approx(10000 / 1.55, abs=1e-3) and e[0xA210] == 3
    assert 0xA405 not in e                               # Pi 的 35 mm 等效值會蓋過真焦距 ⇒ 刪掉
    # ODM extract_focal：sensor_width = width / px_per_unit × mm_per_unit(cm=10)
    sensor_w = 64 / float(e[0xA20E]) * 10
    assert 6.0 / sensor_w == pytest.approx(6.0 / (64 * 1.55e-3), rel=1e-6)
    lat = sum(float(x) / 60 ** i for i, x in enumerate(g[2]))
    lon = sum(float(x) / 60 ** i for i, x in enumerate(g[4]))
    assert (g[1], g[3]) == ("N", "E")
    assert lat == pytest.approx(25.1234567891, abs=1e-9) and lon == pytest.approx(121.2345678912, abs=1e-9)
    assert float(g[6]) == pytest.approx(168.543) and g[5] in (0, b"\x00")
    assert [float(x) for x in g[7]] == [3.0, 4.0, pytest.approx(5.26)] and g[29] == "2026:10:14"
    assert g[16] == "T" and float(g[17]) == pytest.approx(95.25)
    xa = _xmp_attrs(out)
    vx, vy, vz = (float(xa[f"@drone-dji:Flight{k}Speed"]) for k in "XYZ")
    # ODM to_opensfm_exif：speed = [speed_y, speed_x, speed_z]，進 OpenSfM 的 ENU（東、北、上）
    assert [vy, vx, vz] == [21.5, 1.25, 0.3]
    assert xa["@aeroplan:ImageId"] == "hero2_S_000001"


def test_image_data_is_not_recompressed():
    src = _pi_jpeg()
    out = tag_jpeg_bytes(src, _row(), CAM, height="orthometric")
    sos = lambda b: b[b.find(b"\xff\xda"):]                     # noqa: E731
    assert sos(out) == sos(src)                                 # 壓縮資料逐位元組相同
    a = np.asarray(Image.open(io.BytesIO(src)))
    b = np.asarray(Image.open(io.BytesIO(out)))
    assert np.array_equal(a, b)
    with Image.open(io.BytesIO(out)) as im:
        assert float(im.getexif().get_ifd(0x8825)[6]) == pytest.approx(150.003)


def test_hemisphere_and_negative_altitude():
    out = tag_jpeg_bytes(_pi_jpeg(), _row(lat_deg=-33.5, lon_deg=-70.25, h_orth_m=-12.5), CAM, height="orthometric")
    with Image.open(io.BytesIO(out)) as im:
        g = im.getexif().get_ifd(0x8825)
    assert (g[1], g[3]) == ("S", "W") and g[5] in (1, b"\x01") and float(g[6]) == pytest.approx(12.5)
    assert sum(float(x) / 60 ** i for i, x in enumerate(g[4])) == pytest.approx(70.25)
    assert exif_write._dms(25.99999999999)[0] == 26 and float(exif_write._dms(25.99999999999)[2]) == 0.0


def test_refuses_incomplete_rows_and_wrong_size():
    with pytest.raises(ValueError, match="speed_e_mps"):
        tag_jpeg_bytes(_pi_jpeg(), _row(speed_e_mps=None), CAM, height="ellipsoidal")
    with pytest.raises(ValueError, match="h_orth_m"):
        tag_jpeg_bytes(_pi_jpeg(), _row(h_orth_m=None), CAM, height="orthometric")
    with pytest.raises(ValueError, match="不符"):
        tag_jpeg_bytes(_pi_jpeg(), _row(), CameraExif("m", "x", 6.0, 1.55, 4056, 3040), height="ellipsoidal")
    with pytest.raises(ValueError, match="height"):
        tag_jpeg_bytes(_pi_jpeg(), _row(), CAM, height="msl")
    with pytest.raises(ValueError, match="不是 JPEG"):
        tag_jpeg_bytes(b"PNG....", _row(), CAM, height="ellipsoidal")


def test_export_never_touches_originals(tmp_path):
    synth.synth_session_from_fc(tmp_path / "s", [10_000_000, 11_000_000], camera={
        "sensor": "imx477", "lens_mm": 6.0, "pixel_um": 1.55, "width": 64, "height": 48, "readout_ms": 80.0,
        "shutter": "rolling", "kernel": "k", "dtoverlay": "d"}, image_bytes=_pi_jpeg())
    sess = read_pi_session(tmp_path / "s")
    before = {f.file: hashlib.sha256((sess.path / f.file).read_bytes()).hexdigest() for f in sess.frames}
    rows = [_row(image_id=f"uav1_S1_{k:06d}", file=f"images/{k:06d}.jpg") for k in (1, 2)]
    out = exif_write.export_tagged_images(sess, rows, tmp_path / "out", make="Raspberry Pi", height="ellipsoidal")
    assert [p.name for p in out] == ["uav1_S1_000001.jpg", "uav1_S1_000002.jpg"]
    after = {f.file: hashlib.sha256((sess.path / f.file).read_bytes()).hexdigest() for f in sess.frames}
    assert before == after
    with Image.open(out[0]) as im:
        assert im.getexif()[0x0110] == "RPiHQ6-SYN1"            # Model 取自 session.json unit_model
    with pytest.raises(FileExistsError):
        exif_write.export_tagged_images(sess, rows, tmp_path / "out", make="Raspberry Pi", height="ellipsoidal")
    with pytest.raises(ValueError, match="原檔不可改"):
        exif_write.export_tagged_images(sess, rows, sess.path / "images", make="x", height="ellipsoidal")
