"""SITL 參數檔的機體覆寫 —— 值層級鎖。

⚠️⚠️⚠️ **2026-09-05 之前這個洞是開著的**:`generate_vtol_params()` 寫死一份
「通用機」模板(巡航 24 / 最低 20 / 坡度上限 40° / `Q_ASSIST_SPEED` 16.0),
沒有任何機體概念。於是 Pioneer PNP(21.5 / 18 / 20°) 與 HERO PNP(19.0 / 16 / 25°)
在 SITL 裡飛的是**同一台飛機**。

實測證據(FLYBOTH 趟次):兩台的 VTOL→FW 轉換都在空速 **16.1–16.2** 完成,
而 HERO 的規格是 **14** —— 因為 `Q_ASSIST_SPEED` 是模板寫死的 16.0。

⚠️ 這裡鎖的是**生效值**不是「檔案裡有沒有那一行」。ArduPilot 逐行套用同一個
   .parm,後寫的勝出;只 grep 某一行存在會被前面那行的舊值騙過去
   (見 [[value-locks-beat-source-locks]])。所以每條測試都先把整份檔案
   照 ArduPilot 的規則解析成「最後生效值」再斷言。
"""

import tempfile
from pathlib import Path

import pytest

from core.base.airframe import HERO_PNP, PIONEER_PNP
from mission.sitl_launcher import generate_vtol_params

#: 由機體決定、必須逐機不同的參數
AIRFRAME_KEYS = ("AIRSPEED_CRUISE", "AIRSPEED_MIN", "ROLL_LIMIT_DEG",
                 "Q_ASSIST_SPEED")


def effective(path: Path) -> dict:
    """照 ArduPilot 的規則把 .parm 解析成最後生效值(後寫勝出)。"""
    out = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.split("#")[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            try:
                out[parts[0]] = float(parts[1])
            except ValueError:
                pass
    return out


@pytest.fixture()
def outdir(tmp_path):
    return tmp_path


def gen(outdir, airframe=None, tag="p"):
    return effective(generate_vtol_params(outdir / f"{tag}.parm",
                                          airframe=airframe))


# ═══════════════════════════════════════════════════════════════════
#  0. 可觸發性 —— 先證明「不給機體」的模板值真的不等於任何一台
# ═══════════════════════════════════════════════════════════════════
def test_the_generic_template_matches_neither_aircraft(outdir):
    """沒有這一條，下面的斷言可能只是在量一組本來就相等的值。

    模板是通用機:巡航 24、最低 20、坡度上限 40°。兩台真機都不是這組數字。
    """
    base = gen(outdir, None, "base")
    assert base["AIRSPEED_CRUISE"] == pytest.approx(24.0)
    assert base["AIRSPEED_MIN"] == pytest.approx(20.0)
    assert base["ROLL_LIMIT_DEG"] == pytest.approx(40.0)
    for af in (PIONEER_PNP, HERO_PNP):
        assert base["AIRSPEED_CRUISE"] != pytest.approx(af.cruise_airspeed_mps)
        assert base["ROLL_LIMIT_DEG"] != pytest.approx(af.max_bank_angle_deg)


def test_the_two_aircraft_really_differ_from_each_other(outdir):
    """兩台若規格相同，逐機覆寫就無從驗證。"""
    assert PIONEER_PNP.cruise_airspeed_mps != HERO_PNP.cruise_airspeed_mps
    assert PIONEER_PNP.max_bank_angle_deg != HERO_PNP.max_bank_angle_deg
    assert PIONEER_PNP.stall_airspeed_mps != HERO_PNP.stall_airspeed_mps
    assert (PIONEER_PNP.transition_airspeed_mps
            != HERO_PNP.transition_airspeed_mps)


# ═══════════════════════════════════════════════════════════════════
#  1. 生效值必須等於該機體的規格
# ═══════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP], ids=lambda a: a.name)
def test_effective_values_come_from_the_airframe(outdir, af):
    e = gen(outdir, af, af.name.replace(" ", "_"))
    assert e["AIRSPEED_CRUISE"] == pytest.approx(af.cruise_airspeed_mps)
    assert e["AIRSPEED_MIN"] == pytest.approx(af.stall_airspeed_mps)
    assert e["ROLL_LIMIT_DEG"] == pytest.approx(af.max_bank_angle_deg)
    assert e["Q_ASSIST_SPEED"] == pytest.approx(af.transition_airspeed_mps)


def test_hero_transition_speed_is_not_pioneers(outdir):
    """實測抓到的那一項:兩台都在 16 轉換，因為 Q_ASSIST_SPEED 寫死 16.0。

    HERO 的轉換空速是 14，覆寫之後生效值必須是 14 而不是 16。
    """
    h = gen(outdir, HERO_PNP, "hero")
    p = gen(outdir, PIONEER_PNP, "pio")
    assert h["Q_ASSIST_SPEED"] == pytest.approx(14.0)
    assert p["Q_ASSIST_SPEED"] == pytest.approx(16.0)
    assert h["Q_ASSIST_SPEED"] != p["Q_ASSIST_SPEED"]


def test_cruise_must_not_be_clamped_by_the_stall_floor(outdir):
    """HERO 巡航 19.0 曾經低於模板的 AIRSPEED_MIN=20 ⇒ 速度指令被靜默夾住。

    覆寫之後 AIRSPEED_MIN 必須嚴格低於該機巡航，否則整條速度通道無效
    (見 [[interactive-path-formation-speeds-rejected]] 同一類失效)。
    """
    for af in (PIONEER_PNP, HERO_PNP):
        e = gen(outdir, af, af.name.replace(" ", "_"))
        assert e["AIRSPEED_MIN"] < e["AIRSPEED_CRUISE"], (
            f"{af.name}: 最低空速 {e['AIRSPEED_MIN']} 沒有低於巡航 "
            f"{e['AIRSPEED_CRUISE']} —— 速度指令會被夾")
        assert e["AIRSPEED_MIN"] <= af.stall_airspeed_mps


# ═══════════════════════════════════════════════════════════════════
#  2. 覆寫必須是**最後**生效的，而且不可波及其他參數
# ═══════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP], ids=lambda a: a.name)
def test_override_block_comes_last(outdir, af):
    """模板裡本來就有同名行。覆寫段若不在最後，生效值仍是模板值。"""
    path = outdir / "order.parm"
    generate_vtol_params(path, airframe=af)
    text = path.read_text(encoding="utf-8")
    for key in AIRFRAME_KEYS:
        idx = [i for i, ln in enumerate(text.splitlines())
               if ln.strip().startswith(key + " ")]
        assert len(idx) >= 2, f"{key} 只出現一次，覆寫沒寫進去"
        # 最後一次出現的那行，值必須是機體值
        last = text.splitlines()[idx[-1]].split("#")[0].split()
        want = {"AIRSPEED_CRUISE": af.cruise_airspeed_mps,
                "AIRSPEED_MIN": af.stall_airspeed_mps,
                "ROLL_LIMIT_DEG": af.max_bank_angle_deg,
                "Q_ASSIST_SPEED": af.transition_airspeed_mps}[key]
        assert float(last[1]) == pytest.approx(want)


def test_nothing_else_moves(outdir):
    """只有這四個參數該變。其餘一個都不能被機體覆寫動到。"""
    base = gen(outdir, None, "base")
    for af in (PIONEER_PNP, HERO_PNP):
        e = gen(outdir, af, af.name.replace(" ", "_"))
        moved = {k for k in base
                 if k not in AIRFRAME_KEYS and base[k] != e.get(k)}
        assert not moved, f"{af.name} 意外動到 {sorted(moved)}"
        assert set(base) <= set(e), "覆寫把原本的參數弄不見了"


def test_no_airframe_leaves_the_template_untouched(outdir):
    """不給機體時，產出必須跟舊行為逐字元相同(不可悄悄改預設)。"""
    a = (outdir / "a.parm")
    b = (outdir / "b.parm")
    generate_vtol_params(a)
    generate_vtol_params(b, airframe=None)
    assert a.read_text(encoding="utf-8") == b.read_text(encoding="utf-8")
    assert "機體覆寫" not in a.read_text(encoding="utf-8")


# ═══════════════════════════════════════════════════════════════════
#  3. 產生檔要能被 ArduPilot 讀 —— 格式退化的防呆
# ═══════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("af", [PIONEER_PNP, HERO_PNP], ids=lambda a: a.name)
def test_every_override_line_is_parseable(outdir, af):
    """⚠️ 值寫成科學記號(1e+01)或空字串，ArduPilot 會靜默略過整行。

    見 [[jet-conversion-never-applied]]:空字串參數讓幾十趟跑了原廠值。
    """
    path = outdir / "fmt.parm"
    generate_vtol_params(path, airframe=af)
    # ⚠ 切點落在註解行**中間**，該行剩下的字會被當成參數行。
    #   丟掉標記所在那一整行，只留它之後的內容。
    tail = path.read_text(encoding="utf-8").split("機體覆寫")[-1].split("\n", 1)[1]
    seen = 0
    for line in tail.splitlines():
        line = line.split("#")[0].strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        assert len(parts) == 2, f"格式壞掉: {line!r}"
        assert "e" not in parts[1].lower(), f"科學記號會被略過: {line!r}"
        float(parts[1])
        seen += 1
    assert seen == len(AIRFRAME_KEYS), f"覆寫段只有 {seen} 行"


def test_generated_file_is_deterministic(outdir):
    """同樣輸入要產生同樣位元組。

    見 [[export-must-be-deterministic]]:含記憶體位址的產生檔會製造假 diff。
    """
    a, b = outdir / "d1.parm", outdir / "d2.parm"
    generate_vtol_params(a, airframe=PIONEER_PNP)
    generate_vtol_params(b, airframe=PIONEER_PNP)
    assert a.read_bytes() == b.read_bytes()


# ═══════════════════════════════════════════════════════════════════
#  4. ⚠️⚠️⚠️ 端到端:真正送進 SITL 的那一份檔
# ═══════════════════════════════════════════════════════════════════
def test_the_file_that_actually_reaches_sitl_keeps_the_override(monkeypatch, tmp_path):
    """⚠️⚠️⚠️ 2026-09-05 實飛抓到的洞:`generate_vtol_params` 被呼叫**兩次**。

    `start_vtol_sitl()` 先帶著機體產生一次，接著 `launcher.start()` 又
    **無條件**呼叫一次**不帶機體**的版本，寫的是**同一條路徑** ⇒
    機體覆寫被整個蓋回通用機。

    症狀完全靜默。唯一的外顯是 HERO 的 VTOL→FW 轉換仍在空速 16.0 完成，
    而它的規格是 14 —— 而那個數字只有翻飛行 log 才看得到。

    上面那些測試全部只打 `generate_vtol_params` 本身，所以**一條都測不到**。
    這一條打的是「跑完整條啟動路徑之後，檔案裡剩下什麼」。
    """
    import mission.sitl_launcher as SL

    written = []
    real_gen = SL.generate_vtol_params

    def spy(output_path=None, airframe=None):
        # 一律導到 tmp_path 的同一個檔，重現「兩次寫同一條路徑」
        path = tmp_path / "vtol_default.parm"
        written.append(airframe.name if airframe is not None else None)
        return real_gen(path, airframe=airframe)

    monkeypatch.setattr(SL, "generate_vtol_params", spy)
    monkeypatch.setattr(SL.SITLLauncher, "is_available", lambda self: True)

    # 只跑到「產生參數檔」為止，不真的起行程
    class _Stop(RuntimeError):
        pass

    monkeypatch.setattr(SL, "build_defaults_list",
                        lambda *a, **k: (_ for _ in ()).throw(_Stop()))
    launcher = SL.SITLLauncher()
    with pytest.raises(_Stop):
        launcher.start(vehicle="VTOL", lat=25.0, lon=121.0, alt=0.0,
                       heading=0.0, speedup=1.0, instance=0,
                       airframe=HERO_PNP)

    assert written, "generate_vtol_params 一次都沒被呼叫"
    assert None not in written, (
        f"有一次呼叫沒帶機體，會把覆寫蓋掉。各次的機體: {written}")
    eff = effective(tmp_path / "vtol_default.parm")
    assert eff["Q_ASSIST_SPEED"] == pytest.approx(HERO_PNP.transition_airspeed_mps), (
        f"最終檔案的轉換空速是 {eff['Q_ASSIST_SPEED']}，"
        f"應為 HERO 的 {HERO_PNP.transition_airspeed_mps}")
    assert eff["ROLL_LIMIT_DEG"] == pytest.approx(HERO_PNP.max_bank_angle_deg)
    assert eff["AIRSPEED_CRUISE"] == pytest.approx(HERO_PNP.cruise_airspeed_mps)


def test_the_public_entry_point_also_keeps_the_override(monkeypatch, tmp_path):
    """走**公開入口** `start_vtol_sitl()`，不是直接打 `launcher.start()`。

    ⚠️ 上一條測試直接呼叫 `launcher.start(airframe=...)`，所以
    「`start_vtol_sitl` 忘了把機體往下傳」這個突變它抓不到（實測 GREEN）。
    真實的呼叫端走的是這個入口。
    """
    import mission.sitl_launcher as SL

    written = []
    real_gen = SL.generate_vtol_params

    def spy(output_path=None, airframe=None):
        written.append(airframe.name if airframe is not None else None)
        return real_gen(tmp_path / "vtol_default.parm", airframe=airframe)

    class _Stop(RuntimeError):
        pass

    monkeypatch.setattr(SL, "generate_vtol_params", spy)
    monkeypatch.setattr(SL.SITLLauncher, "is_available", lambda self: True)
    monkeypatch.setattr(SL, "build_defaults_list",
                        lambda *a, **k: (_ for _ in ()).throw(_Stop()))
    with pytest.raises(_Stop):
        SL.start_vtol_sitl(SL.SITLLauncher(), lat=25.0, lon=121.0, alt=0.0,
                           heading=0.0, speedup=1.0, instance=0,
                           airframe=PIONEER_PNP)

    assert len(written) >= 2, (
        f"預期這條路徑會產生兩次參數檔，實際 {len(written)} 次 —— "
        "若改成一次，這條測試的前提要重寫")
    assert None not in written, (
        f"有一次呼叫沒帶機體，最後那次會把覆寫蓋掉。各次的機體: {written}")
    eff = effective(tmp_path / "vtol_default.parm")
    assert eff["Q_ASSIST_SPEED"] == pytest.approx(
        PIONEER_PNP.transition_airspeed_mps)
    assert eff["ROLL_LIMIT_DEG"] == pytest.approx(PIONEER_PNP.max_bank_angle_deg)
