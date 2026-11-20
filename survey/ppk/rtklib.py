"""survey/ppk/rtklib.py — 呼叫 RTKLIB（demo5）的 convbin 與 rnx2rtkp 解算相機模組 F9P 的軌跡。

本機**沒有安裝 RTKLIB**；本模組只組命令列與設定檔、找執行檔（shutil.which 或明確路徑），
找不到就丟 :class:`RtklibNotFound`，不會自己下載。版本建議 rtklibexplorer demo5 b34 以上。

命令列依據（rtklibexplorer/RTKLIB demo5 分支原始碼的 help 文字）：
- app/consapp/convbin/convbin.c：``convbin [option ...] file``；-r ubx、-v 3.04（預設）、
  -o 觀測檔、-n 導航檔（RINEX 3 混合星系一個檔）、-od 都卜勒、-os SNR、-oi/-ot/-ol 導航檔表頭的
  電離層／時間改正／閏秒。只給 -n 不給 -o 時不會寫觀測檔 ⇒ 兩個都明確給。
- app/consapp/rnx2rtkp/rnx2rtkp.c：``rnx2rtkp [option]... file file [...]``；
  第一個 OBS 是移動站、第二個 OBS 是基站，至少一個 NAV。main() 先載入 -k 設定檔再解析其他選項
  （命令列優先）⇒ 這裡只傳 -k 與 -o，所有解算設定都在設定檔裡，設定檔＝實際生效值。

設定選項名稱與列舉值依 src/options.c 的 sysopts[]（注意 AR 仰角遮罩叫 pos2-arelmask，不是 elmaskar）。
預設值與理由見 :data:`DEFAULT_OPTIONS` 的註解。
"""

from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Sequence

__all__ = [
    "RtklibNotFound",
    "RtklibError",
    "BaseStation",
    "DEFAULT_OPTIONS",
    "LOCKED_OPTIONS",
    "find_executable",
    "convbin_command",
    "rnx2rtkp_command",
    "rnx2rtkp_options",
    "render_config",
    "PpkRun",
    "run_ppk",
]


class RtklibNotFound(FileNotFoundError):
    """找不到 RTKLIB 執行檔。"""


class RtklibError(RuntimeError):
    """RTKLIB 執行失敗或沒有產出預期的檔案。"""


# 預設 rnx2rtkp 設定（kinematic、前後向合併、雙頻、GPS+GLO+GAL+BDS、輸出 GPST 經緯橢球高）。
DEFAULT_OPTIONS: dict[str, str] = {
    "pos1-posmode": "kinematic",
    "pos1-frequency": "l1+l2",       # F9P 為 L1/L2（E5b、B2I 在同一個 L2 槽）
    # 前向＋後向卡曼濾波各解一次再合併：每個 epoch 兩個獨立解，兩者都 fix 且一致才輸出 Q=1，
    # 不一致就降成 float ⇒ 錯誤 fix 會被看出來（下游標 ppk_float），這是選 fix-and-hold 的前提。
    "pos1-soltype": "combined",
    "pos1-elmask": "15",
    "pos1-navsys": "45",             # 1 GPS + 4 GLO + 8 GAL + 32 BDS
    "pos1-dynamics": "on",           # 航空器有加速度，讓濾波器帶速度／加速度狀態
    "pos1-ionoopt": "brdc",          # 基線 ≤5 km（自架基站），廣播電離層即可
    "pos1-tropopt": "saas",
    "pos1-sateph": "brdc",
    # AR 模式選 fix-and-hold（不是 continuous）：飛機轉彎時衛星幾何與周跳多，hold 把已固定的模糊度
    # 回饋濾波器，轉彎後能快速重新 fix；它的風險是把錯誤 fix「hold」住，而 combined 的前後向一致性
    # 檢查正好把這種情況降成 float。社群的 F9P PPK 指南（ArduSimple demo5 教學）也是 GPS fix-and-hold、
    # BDS off（2026-10-14 查到的二手摘要，未逐字核對 demo5 範例檔）。
    # 若 B9/B10 台架發現 hold 造成偏移，改 continuous 重算並比較（overrides 傳入，會寫進設定檔）。
    "pos2-armode": "fix-and-hold",
    # GLONASS 頻間偏差只有在基站也是 u-blox 時才相同；e-GNSS 站多為其他廠牌 ⇒ 預設不對 GLONASS 做 AR
    # （GLONASS 仍參與 float 解）。自架 F9P 基站時可以 overrides={"pos2-gloarmode": "on"}。
    "pos2-gloarmode": "off",
    "pos2-bdsarmode": "off",         # 同上述指南；BDS 仍參與 float 解
    "pos2-arthres": "3",
    "out-solformat": "llh",
    "out-outhead": "on",
    "out-outopt": "on",
    "out-outvel": "on",              # 讓 interp.py 用速度當 Hermite 切線
    "out-timesys": "gpst",
    "out-timeform": "tow",
    "out-timendec": "3",
    "out-degform": "deg",
    "out-height": "ellipsoidal",
    "out-solstatic": "all",
    "misc-timeinterp": "on",         # e-GNSS RINEX 常是 1 s／30 s，移動站 10 Hz ⇒ 內插基站觀測量
}

# pos.py 只能正確讀這些設定；覆寫成別的值等於讓下游讀錯 ⇒ 直接拒絕
LOCKED_OPTIONS: dict[str, tuple[str, ...]] = {
    "out-timesys": ("gpst",),
    "out-height": ("ellipsoidal",),
    "out-degform": ("deg",),
    "out-outhead": ("on",),
    "out-solformat": ("llh", "xyz"),
    "pos1-soltype": ("combined", "combined-nophasereset"),
}


@dataclass(frozen=True)
class BaseStation:
    """基站座標**必須明確給**（不用 RINEX 表頭或單點平均：那會讓所有照片整體偏公尺級而且看不出來）。"""
    lat_deg: float
    lon_deg: float
    h_ell_m: float                 # 天線參考點（ARP）下方標石的橢球高；天線高另給 antenna_height_m
    frame: str                     # 座標所屬框架，例如 "e-GNSS[2025]"；一路帶到 photos.csv 的 crs_h
    antenna_height_m: float = 0.0  # ARP 高於標石（ant2-antdelu）
    antenna_type: str | None = None

    def __post_init__(self) -> None:
        if not self.frame or not self.frame.strip():
            raise ValueError("BaseStation.frame 必須寫明框架（例如 'e-GNSS[2025]'），不能空白")
        if not (-90.0 <= self.lat_deg <= 90.0 and -180.0 <= self.lon_deg <= 180.0):
            raise ValueError(f"基站經緯度超出範圍：{self.lat_deg}, {self.lon_deg}")


def find_executable(name: str, explicit: str | os.PathLike | None = None) -> str:
    if explicit is not None:
        p = Path(explicit)
        if not p.is_file():
            raise RtklibNotFound(f"指定的 {name} 不存在：{p}")
        return str(p)
    found = shutil.which(name)
    if not found:
        raise RtklibNotFound(
            f"找不到 RTKLIB 的 {name}（PATH 上沒有）—— 請安裝 rtklibexplorer demo5（b34 以上）"
            f"並加入 PATH，或明確傳入執行檔路徑")
    return found


def convbin_command(ubx_path, obs_path, nav_path, *, exe: str = "convbin",
                    rinex_version: str = "3.04", extra_args: Sequence[str] = ()) -> list[str]:
    return [str(exe), "-r", "ubx", "-v", str(rinex_version), "-od", "-os", "-oi", "-ot", "-ol",
            *map(str, extra_args), "-o", str(obs_path), "-n", str(nav_path), str(ubx_path)]


def rnx2rtkp_command(rover_obs, base_obs, nav_paths: Sequence, out_pos, conf_path, *,
                     exe: str = "rnx2rtkp") -> list[str]:
    if not nav_paths:
        raise ValueError("rnx2rtkp 至少需要一個導航檔")
    return [str(exe), "-k", str(conf_path), "-o", str(out_pos), str(rover_obs), str(base_obs),
            *map(str, nav_paths)]


def rnx2rtkp_options(base: BaseStation, overrides: Mapping[str, str] | None = None) -> dict[str, str]:
    """回傳實際生效的完整設定（預設 ⊕ 基站 ⊕ overrides）。"""
    opts = dict(DEFAULT_OPTIONS)
    opts.update({
        "ant2-postype": "llh",
        "ant2-pos1": f"{base.lat_deg:.9f}",
        "ant2-pos2": f"{base.lon_deg:.9f}",
        "ant2-pos3": f"{base.h_ell_m:.4f}",
        "ant2-antdelu": f"{base.antenna_height_m:.4f}",
    })
    if base.antenna_type:
        opts["ant2-anttype"] = base.antenna_type
    for key, val in (overrides or {}).items():
        val = str(val)
        if key.startswith("ant2-pos") or key == "ant2-postype":
            raise ValueError(f"基站座標只能由 BaseStation 給，不能用 overrides 覆寫 {key}")
        allowed = LOCKED_OPTIONS.get(key)
        if allowed is not None and val not in allowed:
            raise ValueError(f"{key}={val} 會讓 pos.py 讀錯，只允許 {allowed}")
        opts[key] = val
    return opts


def render_config(opts: Mapping[str, str], comment: str = "") -> str:
    lines = ["# AeroPlan survey/ppk 產生的 rnx2rtkp 設定（此檔即實際生效值）"]
    if comment:
        lines += [f"# {c}" for c in comment.splitlines()]
    lines += [f"{k:<18} ={v}" for k, v in opts.items()]
    return "\n".join(lines) + "\n"


@dataclass
class PpkRun:
    pos_path: Path
    conf_path: Path
    rover_obs: Path
    rover_nav: Path
    options: dict[str, str]
    commands: list[list[str]] = field(default_factory=list)
    logs: list[str] = field(default_factory=list)


def _run(cmd: list[str], timeout_s: float) -> str:
    try:
        cp = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s, check=False)
    except subprocess.TimeoutExpired as e:
        raise RtklibError(f"{Path(cmd[0]).name} 超過 {timeout_s:.0f} s 未結束") from e
    out = (cp.stdout or "") + (cp.stderr or "")
    if cp.returncode != 0:
        raise RtklibError(f"{Path(cmd[0]).name} 失敗（rc={cp.returncode}）：{out.strip()[-2000:]}")
    return out


def _must_exist(p: Path, what: str) -> None:
    if not p.is_file() or p.stat().st_size == 0:
        raise RtklibError(f"{what} 沒有產生或是空檔：{p}")


def run_ppk(
    rover_ubx,
    base_obs,
    base: BaseStation,
    workdir,
    *,
    extra_nav: Sequence = (),
    overrides: Mapping[str, str] | None = None,
    convbin_exe: str | os.PathLike | None = None,
    rnx2rtkp_exe: str | os.PathLike | None = None,
    timeout_s: float = 3600.0,
) -> PpkRun:
    """rover.ubx →（convbin）RINEX →（rnx2rtkp，kinematic combined）→ rover.pos。"""
    cb = find_executable("convbin", convbin_exe)
    rk = find_executable("rnx2rtkp", rnx2rtkp_exe)
    wd = Path(workdir)
    wd.mkdir(parents=True, exist_ok=True)
    ubx = Path(rover_ubx)
    obs, nav = wd / (ubx.stem + ".obs"), wd / (ubx.stem + ".nav")
    pos, conf = wd / (ubx.stem + ".pos"), wd / "rnx2rtkp.conf"
    opts = rnx2rtkp_options(base, overrides)
    conf.write_text(render_config(opts, f"base frame: {base.frame}"), encoding="utf-8")
    run = PpkRun(pos_path=pos, conf_path=conf, rover_obs=obs, rover_nav=nav, options=opts)

    c1 = convbin_command(ubx, obs, nav, exe=cb)
    run.commands.append(c1)
    run.logs.append(_run(c1, timeout_s))
    _must_exist(obs, "convbin 觀測檔")
    _must_exist(nav, "convbin 導航檔")

    c2 = rnx2rtkp_command(obs, base_obs, [nav, *extra_nav], pos, conf, exe=rk)
    run.commands.append(c2)
    run.logs.append(_run(c2, timeout_s))
    _must_exist(pos, "rnx2rtkp 軌跡檔")
    return run
