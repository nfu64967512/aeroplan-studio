"""命令列：python -m pi_capture {run,selfcheck,verify,f9p-config} …（PYTHONPATH 指到 pi_capture 的上一層）

結束碼：0 成功／全部通過；1 檢查失敗；2 設定或參數錯誤。
設定檔是嚴格的：不認得的鍵直接拒絕（打錯字不能被靜默忽略），啟動時把有效值完整寫進日誌。
"""
from __future__ import annotations

import argparse
import json
import logging
import platform
import signal
import sys
import threading
from pathlib import Path

from . import __version__
from . import session as S

log = logging.getLogger("pi_capture")

_TOP = {"storage_root", "uav", "unit_model", "camera", "lever_arm_m", "gnss_lever_arm_m",
        "capture", "trigger", "ready", "health", "imu", "gnss"}
_CAMERA = {"sensor", "lens_mm", "pixel_um", "width", "height", "readout_ms", "shutter", "dtoverlay"}
_CAPTURE_REQ = {"exposure_us", "analogue_gain", "colour_gains"}
_CAPTURE_OPT = {"jpeg_quality": 95, "buffer_count": 4, "frame_timeout_ms": 500.0,
                "max_inflight_writes": 2, "lens_position": None, "camera_num": 0}
_TRIGGER = {"chip", "line", "bias"}
_READY = {"chip", "line"}
_HEALTH = {"max_temp_c", "min_free_gb"}
_IMU = {"type", "spi_bus", "spi_dev", "rate_hz"}
_GNSS = {"device", "baudrate", "fsync_period_s"}


class ConfigError(ValueError):
    pass


def _check_keys(where: str, d, required: set, optional=()):
    if not isinstance(d, dict):
        raise ConfigError(f"{where} 必須是物件")
    unknown = set(d) - required - set(optional)
    missing = required - set(d)
    if unknown:
        raise ConfigError(f"{where} 有不認得的鍵：{sorted(unknown)}")
    if missing:
        raise ConfigError(f"{where} 缺少必填鍵：{sorted(missing)}")


def load_config(path) -> dict:
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    _check_keys("設定檔", cfg, _TOP)
    _check_keys("camera", cfg["camera"], _CAMERA)
    _check_keys("capture", cfg["capture"], _CAPTURE_REQ, _CAPTURE_OPT)
    cfg["capture"] = {**_CAPTURE_OPT, **cfg["capture"]}
    _check_keys("trigger", cfg["trigger"], _TRIGGER)
    if cfg["ready"] is not None:
        _check_keys("ready", cfg["ready"], _READY)
    _check_keys("health", cfg["health"], _HEALTH)
    if cfg["imu"] is not None:
        _check_keys("imu", cfg["imu"], _IMU)
        if cfg["imu"]["type"] != "icm42688p":
            raise ConfigError(f"imu.type 目前只支援 icm42688p，收到 {cfg['imu']['type']!r}")
    if cfg["gnss"] is not None:
        _check_keys("gnss", cfg["gnss"], _GNSS)
    return cfg


def make_meta(cfg: dict, session_id: str, started_realtime_ns: int) -> S.SessionMeta:
    c = cfg["camera"]
    cam = S.CameraInfo(sensor=c["sensor"], lens_mm=c["lens_mm"], pixel_um=c["pixel_um"],
                       width=c["width"], height=c["height"], readout_ms=c["readout_ms"],
                       shutter=c["shutter"], kernel=platform.release(), dtoverlay=c["dtoverlay"])
    return S.SessionMeta(
        session_id=session_id, uav=cfg["uav"], unit_model=cfg["unit_model"], camera=cam,
        lever_arm_m=cfg["lever_arm_m"], gnss_lever_arm_m=cfg["gnss_lever_arm_m"],
        software=f"aeroplan-pi-capture {__version__}; python {platform.python_version()}",
        started_utc_approx=S.utc_iso(started_realtime_ns))


def _new_session_dir(root: Path, base_id: str) -> tuple[str, Path]:
    sid, n = base_id, 1
    while (root / sid).exists():
        n += 1
        sid = f"{base_id}_{n}"
    return sid, root / sid


def _make_camera(cfg: dict):
    from .capture import Picamera2Camera

    cap, cam = cfg["capture"], cfg["camera"]
    return Picamera2Camera(exposure_us=cap["exposure_us"], analogue_gain=cap["analogue_gain"],
                           colour_gains=cap["colour_gains"], model=cfg["unit_model"],
                           lens_mm=cam["lens_mm"], expected_size=(cam["width"], cam["height"]),
                           buffer_count=cap["buffer_count"], camera_num=cap["camera_num"],
                           lens_position=cap["lens_position"])


def _make_edges(cfg: dict):
    from .capture import GpiodEdgeSource

    t = cfg["trigger"]
    return GpiodEdgeSource(t["chip"], t["line"], bias=t["bias"])


def cmd_run(args) -> int:
    from .capture import CaptureService, GpiodReadyOutput, SystemHealth

    cfg = load_config(args.config)
    log.info("有效設定：%s", json.dumps(cfg, ensure_ascii=False))
    clock = S.SystemClock()
    s0 = clock.sample()
    root = Path(cfg["storage_root"])
    sid, sdir = _new_session_dir(root, S.new_session_id(cfg["uav"], s0.realtime_ns))
    meta = make_meta(cfg, sid, s0.realtime_ns)
    gate = True
    if args.selfcheck_result:
        try:
            gate = json.loads(Path(args.selfcheck_result).read_text(encoding="utf-8")).get("ok") is True
        except (OSError, json.JSONDecodeError) as e:
            log.error("讀不到自檢結果 %s：%s", args.selfcheck_result, e)
            gate = False
        if not gate:
            log.error("開機自檢未通過：照常擷取，但就緒腳不會拉高")
    ready = None
    if cfg["ready"] is not None:
        ready = GpiodReadyOutput(cfg["ready"]["chip"], cfg["ready"]["line"])
        ready.open()
    storage = S.LocalStorage(sdir)
    cap = cfg["capture"]
    svc = CaptureService(storage, meta, _make_camera(cfg), _make_edges(cfg), clock,
                         health=SystemHealth(), ready=ready, jpeg_quality=cap["jpeg_quality"],
                         frame_timeout_ms=cap["frame_timeout_ms"],
                         max_inflight_writes=cap["max_inflight_writes"],
                         max_temp_c=cfg["health"]["max_temp_c"],
                         min_free_bytes=int(cfg["health"]["min_free_gb"] * 1e9),
                         startup_gate_ok=gate)
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())

    svc.start()
    side, extra = [], []
    try:
        if cfg["imu"] is not None:
            from .imu import Icm42688Spi, ImuLogger, IMU_CSV

            dev = Icm42688Spi(cfg["imu"]["spi_bus"], cfg["imu"]["spi_dev"])
            dev.open()
            il = ImuLogger(storage, dev, clock, rate_hz=cfg["imu"]["rate_hz"])
            il.open()
            side.append((il, threading.Thread(target=il.run, args=(stop,), daemon=True), dev))
            extra.append(IMU_CSV)
        if cfg["gnss"] is not None:
            from .f9p_config import ROVER_UBX, RoverLogger, SerialStream

            st = SerialStream(cfg["gnss"]["device"], cfg["gnss"]["baudrate"])
            st.open()
            rl = RoverLogger(storage, st, clock, fsync_period_s=cfg["gnss"]["fsync_period_s"])
            rl.open()
            side.append((rl, threading.Thread(target=rl.run, args=(stop,), daemon=True), st))
            extra.append(ROVER_UBX)
        for _, th, _ in side:
            th.start()
        svc.run(stop)
    finally:
        stop.set()
        for lg, th, dev in side:
            if th.ident is not None:            # 沒啟動過的執行緒不能 join
                th.join(timeout=2.0)
            lg.close()
            dev.close()
        svc.stop(extra_files=extra)
        if ready is not None:
            ready.close()
    return 0


def cmd_selfcheck(args) -> int:
    from .capture import SystemHealth, run_selfcheck

    cfg = load_config(args.config)
    clock = S.SystemClock()
    s0 = clock.sample()
    root = Path(cfg["storage_root"]) / "selfcheck"
    sid, sdir = _new_session_dir(root, S.new_session_id(cfg["uav"], s0.realtime_ns))
    res = run_selfcheck(storage=S.LocalStorage(sdir), meta=make_meta(cfg, sid, s0.realtime_ns),
                        camera=_make_camera(cfg), edges=_make_edges(cfg), clock=clock,
                        health=SystemHealth(), planned_photos=args.planned_photos,
                        bytes_per_photo=args.bytes_per_photo,
                        max_temp_c=cfg["health"]["max_temp_c"],
                        external_edge_timeout_s=args.external_edge_s,
                        jpeg_quality=cfg["capture"]["jpeg_quality"],
                        frame_timeout_ms=cfg["capture"]["frame_timeout_ms"])
    text = json.dumps(res, ensure_ascii=False, indent=2)
    print(text)
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
    return 0 if res["ok"] else 1


def cmd_verify(args) -> int:
    rep = S.verify_session(args.session_dir)
    print(json.dumps(rep, ensure_ascii=False, indent=2))
    return 0 if rep["ok"] else 1


def cmd_f9p_config(args) -> int:
    from . import f9p_config as F

    layers = 0
    for name in args.layers.split(","):
        layers |= {"ram": F.LAYER_RAM, "bbr": F.LAYER_BBR, "flash": F.LAYER_FLASH}[name.strip()]
    msgs = F.build_rover_config(args.port, meas_rate_ms=args.meas_ms, baudrate=args.baud,
                                layers=layers, min_photo_interval_s=args.min_photo_interval_s)
    for m in msgs:
        print(f"{m.description}\n  {m.frame.hex(' ')}")
    if args.dry_run:
        Path(args.dry_run).write_bytes(b"".join(m.frame for m in msgs))
        return 0
    st = F.SerialStream(args.device, args.link_baud)
    st.open()
    try:
        for line in F.apply_config(st, msgs):
            print(line)
    finally:
        st.close()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="pi_capture")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("run", help="擷取服務（systemd 用）")
    p.add_argument("--config", required=True)
    p.add_argument("--selfcheck-result", help="ExecStartPre 自檢輸出的 JSON；不是 ok 就不拉高就緒腳")
    p = sub.add_parser("selfcheck", help="起飛前自檢（相機會被佔用：服務要先停）")
    p.add_argument("--config", required=True)
    p.add_argument("--planned-photos", type=int, required=True)
    p.add_argument("--bytes-per-photo", type=float, required=True, help="台架實測的每張 JPEG 大小")
    p.add_argument("--external-edge-s", type=float, default=0.0,
                   help=">0：等真的 GPIO 邊緣（地面站送 DO_DIGICAM_CONTROL param5=1）；0：自己注入")
    p.add_argument("--out")
    p = sub.add_parser("verify", help="檢查會期目錄（卸載前後）")
    p.add_argument("session_dir")
    p = sub.add_parser("f9p-config", help="寫 ZED-F9P 流動站設定")
    p.add_argument("--device", required=True)
    p.add_argument("--link-baud", type=int, default=38400, help="目前與接收機連線的鮑率（USB 無意義）")
    p.add_argument("--port", choices=["USB", "UART1", "UART2"], required=True,
                   help="要輸出 UBX 的接收機埠")
    p.add_argument("--baud", type=int, help="UART 埠的新鮑率（UART 必填）")
    p.add_argument("--meas-ms", type=int, default=100)
    p.add_argument("--layers", default="ram,bbr,flash")
    p.add_argument("--min-photo-interval-s", type=float)
    p.add_argument("--dry-run", metavar="OUT.ubx", help="只產生位元組寫進檔案，不開序列埠")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    handlers = {"run": cmd_run, "selfcheck": cmd_selfcheck, "verify": cmd_verify,
                "f9p-config": cmd_f9p_config}
    try:
        return handlers[args.cmd](args)
    except (ConfigError, ValueError, KeyError) as e:
        log.error("設定或參數錯誤：%s", e)
        return 2


if __name__ == "__main__":
    sys.exit(main())
