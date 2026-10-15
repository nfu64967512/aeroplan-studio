"""電子圍籬要跟著掃描角轉 —— 值層級鎖。

背景：圍籬本來恆為軸對齊矩形（``method='aabb'``，shapely ``.envelope``）。
掃描線斜著走的時候，軸對齊矩形會比實際需要的大很多（45° 掃描角、矩形作業區
實測 **2.06×**）。本檔鎖住「有掃描角就對齊掃描角、沒有就維持軸對齊」。

⚠️⚠️⚠️ 這個 bug 在 0°/45°/90° **看不出來**，理由有兩層，兩層都要防：

  1. 掃描角 0° 或 90° 時，軸對齊矩形**恰好等於**對齊掃描角的矩形
     ⇒ 用軸對齊的角度做驗收會恆綠。
  2. 更隱蔽的一層：實作把羅盤方位 θ 轉成平面數學角時要用 ``φ = 90 − θ``。
     若漏掉那個 ``90 −``（直接拿 θ 當數學角），在 θ 是 **45 的倍數**時
     剛好得到同一個矩形 —— 因為「在旋轉框架裡取軸對齊包絡」是 90° 週期的。
     所以連 45° 都驗不出這個錯，而 45° 正是「浪費最多」那一格，
     最容易被挑去當驗收角度。

⇒ 本檔的主角度一律用 **30°/62°/123.4°** 這種**不是 45 倍數**的角，
  而且每條測試前先斷言「這個設定真的能讓兩者不同」（可觸發性前置斷言），
  否則一條量不到東西的測試和一條通過的測試在報表上長得一模一樣。

⚠️ ``method='mrr'`` **不是**修法：MRR 求「面積最小」，不是「方向對齊」。
  兩者在**作業區外形與掃描方向不一致**時才會分岔，見
  ``test_mrr_is_a_different_answer_when_area_shape_disagrees_with_scan_angle``。

⚠️ 對齊掃描角**不保證面積變小** —— 手動掃描角與作業區外形打架時會變大，
  見 ``test_manual_angle_fighting_the_area_can_enlarge_the_fence``。
"""

import math

import pytest

pyproj = pytest.importorskip("pyproj")
shapely = pytest.importorskip("shapely")

from mission.geofence_manager import GeofenceConstraintManager, GeofenceError


# ═══════════════════════════════════════════════════════════════════
#  量測工具 —— 一律回到 AEQD 平面上量，不用平面近似
# ═══════════════════════════════════════════════════════════════════
REF_LAT, REF_LON = 24.78, 120.99
BUF = 30.0


def _aeqd(lat0, lon0):
    return pyproj.Transformer.from_crs(
        pyproj.CRS.from_epsg(4326),
        pyproj.CRS.from_proj4(
            f"+proj=aeqd +lat_0={lat0} +lon_0={lon0} "
            "+datum=WGS84 +units=m +no_defs"
        ),
        always_xy=True,
    )


def _fence_xy(fence):
    """把圍籬頂點投到以自身形心為切點的 AEQD 平面（x=East, y=North，公尺）。"""
    la0 = sum(v[0] for v in fence.vertices) / len(fence.vertices)
    lo0 = sum(v[1] for v in fence.vertices) / len(fence.vertices)
    fwd = _aeqd(la0, lo0)
    return [fwd.transform(lo, la) for la, lo in fence.vertices]


def _edges(fence):
    """回傳 [(邊長 m, 羅盤方位 deg mod 180), ...]。"""
    xy = _fence_xy(fence)
    out = []
    for i in range(len(xy)):
        a, b = xy[i], xy[(i + 1) % len(xy)]
        dx, dy = b[0] - a[0], b[1] - a[1]
        out.append((math.hypot(dx, dy),
                    (90.0 - math.degrees(math.atan2(dy, dx))) % 180.0))
    return out


def _area_m2(fence):
    xy = _fence_xy(fence)
    n = len(xy)
    return abs(sum(xy[i][0] * xy[(i + 1) % n][1] - xy[(i + 1) % n][0] * xy[i][1]
                   for i in range(n))) / 2.0


def _min_margin_m(fence, pts):
    """每個航點到圍籬邊界的最小距離（m），點在圍籬外回負值。"""
    from shapely.geometry import Point, Polygon
    lat0 = sum(p[0] for p in pts) / len(pts)
    m_lon = 111320.0 * math.cos(math.radians(lat0))
    xy = lambda la, lo: ((lo - pts[0][1]) * m_lon, (la - pts[0][0]) * 111320.0)  # noqa: E731
    poly = Polygon([xy(la, lo) for la, lo in fence.vertices])
    out = []
    for p in pts:
        q = Point(xy(p[0], p[1]))
        d = poly.exterior.distance(q)
        out.append(d if poly.contains(q) else -d)
    return min(out)


def _angdiff90(a, b):
    """兩個方向的夾角，模 90° —— 矩形的邊不是平行就是垂直於掃描線。"""
    d = abs(a - b) % 90.0
    return min(d, 90.0 - d)


def _angdiff180(a, b):
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def _misalignment(fence, scan_compass):
    """四條邊裡最糟的那條，與掃描角的夾角（模 90°）。"""
    return max(_angdiff90(c, scan_compass) for _, c in _edges(fence))


def _long_edge_compass(fence):
    return sorted(_edges(fence), key=lambda e: -e[0])[0][1]


# ═══════════════════════════════════════════════════════════════════
#  測試資料 —— 掃描航線（羅盤慣例：0=北、順時針；掃描線平行該方向）
# ═══════════════════════════════════════════════════════════════════
M_LAT = 111320.0
M_LON = 111320.0 * math.cos(math.radians(REF_LAT))


def scan_waypoints(theta_compass, n_lines=8, length=600.0, spacing=60.0,
                   alt=120.0):
    """在 (REF_LAT, REF_LON) 附近生成 n 條平行掃描線的折返航點。

    羅盤 θ 的方向向量在 (East, North) 上是 (sin θ, cos θ)。
    """
    ux, uy = math.sin(math.radians(theta_compass)), math.cos(math.radians(theta_compass))
    vx, vy = uy, -ux                      # 條帶推進方向（垂直掃描線）
    pts = []
    for i in range(n_lines):
        off = (i - (n_lines - 1) / 2.0) * spacing
        cx, cy = vx * off, vy * off
        for t in (-length / 2.0, length / 2.0):
            e, nn = cx + ux * t, cy + uy * t
            pts.append((REF_LAT + nn / M_LAT, REF_LON + e / M_LON, alt))
    return pts


def _rot_xy(p, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)


def elongated_region(long_axis_compass, length=1200.0, width=300.0):
    """長軸落在指定羅盤方位的細長矩形作業區（回傳 shapely Polygon，公尺）。

    4:1 的長寬比讓 ``minimum_rotated_rectangle`` 的方向**沒有懸念**，
    不會像近似正方或等腰形狀那樣在兩個候選之間跳。
    """
    from shapely.geometry import Polygon
    phi = 90.0 - long_axis_compass
    return Polygon([_rot_xy(p, phi) for p in
                    [(-length / 2, -width / 2), (length / 2, -width / 2),
                     (length / 2, width / 2), (-length / 2, width / 2)]])


def clipped_scan_waypoints(region_poly, theta_compass, spacing=60.0, alt=120.0):
    """在作業區內以 ``theta_compass`` 排平行掃描線，裁切後回傳折返航點。

    這才是真實的覆蓋航線形狀：點集的**整體外形**跟著作業區，
    而**條帶方向**跟著掃描角。兩者不同時，MRR 與「對齊掃描角」才會分岔。
    """
    from shapely.geometry import LineString
    ux, uy = math.sin(math.radians(theta_compass)), math.cos(math.radians(theta_compass))
    vx, vy = uy, -ux
    x0, y0, x1, y1 = region_poly.bounds
    reach = math.hypot(x1 - x0, y1 - y0)
    pts = []
    for i in range(-int(reach / spacing) - 2, int(reach / spacing) + 3):
        cx, cy = vx * i * spacing, vy * i * spacing
        seg = LineString([(cx - ux * reach, cy - uy * reach),
                          (cx + ux * reach, cy + uy * reach)]).intersection(region_poly)
        if seg.is_empty or seg.geom_type != "LineString":
            continue
        cs = list(seg.coords)
        for e, nn in (cs[0], cs[-1]):
            pts.append((REF_LAT + nn / M_LAT, REF_LON + e / M_LON, alt))
    return pts


def build(waypoints, scan_angle_deg=None, method="aabb", buffer_m=BUF):
    return GeofenceConstraintManager(
        buffer_radius_m=buffer_m, method=method
    ).build(waypoints=waypoints, scan_angle_deg=scan_angle_deg).geofence


# ═══════════════════════════════════════════════════════════════════
#  0. 可觸發性 —— 先證明這組設定真的量得到東西
# ═══════════════════════════════════════════════════════════════════
def test_the_setup_can_actually_show_a_difference():
    """前置斷言：30° 掃描角下，軸對齊圍籬**確實**比對齊掃描角的大很多。

    ⚠️ 沒有這一條，下面每一條「對齊了」的斷言都可能是在量一個
       aabb 本來就等於 scan 的退化設定 —— 那時測試全綠而且什麼都沒驗到。
    """
    wps = scan_waypoints(30.0)
    a_aabb = _area_m2(build(wps))
    a_scan = _area_m2(build(wps, scan_angle_deg=30.0))
    assert a_aabb / a_scan > 1.5, (
        f"這組掃描線在 30° 下 aabb/scan 只有 {a_aabb / a_scan:.3f}×，"
        "分辨不出對齊與否 —— 換一組更狹長的掃描線，否則後面的測試沒有鑑別力。"
    )
    assert _misalignment(build(wps), 30.0) > 10.0, (
        "軸對齊圍籬居然已經對齊 30° 掃描角，這組資料不能用來驗收。"
    )


def _rect_at_math_angle(waypoints, phi):
    """在數學角 ``phi`` 的框架裡取軸對齊包絡再轉回來（無 buffer）。"""
    from shapely import affinity
    from shapely.geometry import MultiPoint

    fwd = _aeqd(REF_LAT, REF_LON)
    mp = MultiPoint([fwd.transform(lo, la) for la, lo, _ in waypoints])
    r = affinity.rotate(mp, -phi, origin=(0.0, 0.0)).envelope
    return affinity.rotate(r, phi, origin=(0.0, 0.0)), mp


@pytest.mark.parametrize("theta", [0.0, 45.0, 90.0, 135.0])
def test_multiples_of_45_cannot_detect_the_missing_conversion(theta):
    """反向鎖：證明 0/45/90/135 **驗不出**「漏掉 90− 轉換」，不可拿來當驗收角。

    正確是 ``φ = 90 − θ``；最容易寫錯的是直接拿羅盤角當數學角用（``φ = θ``）。
    兩者相差 ``90 − 2θ``，而「在旋轉框架裡取軸對齊包絡」這件事是 **90° 週期**的
    ⇒ θ 為 45 的倍數時兩者給出**同一個矩形**。

    （順帶：符號鏡射 ``φ = θ − 90`` 與正確值差 ``2θ − 180``，重合條件一模一樣。
    所以三種寫法在 45 的倍數上全都長得一樣。）

    這條測試就是要把那個事實**寫下來並持續檢查**，以免日後有人把驗收角度
    改回 45°（正好是浪費最多、最像好題目的那一格）還覺得測得很紮實。
    """
    good, _ = _rect_at_math_angle(scan_waypoints(theta), 90.0 - theta)
    bad, _ = _rect_at_math_angle(scan_waypoints(theta), theta)
    assert good.area == pytest.approx(bad.area, rel=1e-9), (
        f"θ={theta}° 時正確與錯誤轉換竟然不同 —— 本檔對 45 倍數的假設要重寫。"
    )


@pytest.mark.parametrize("theta", [30.0, 62.0, 17.3])
def test_a_non_multiple_of_45_does_detect_the_missing_conversion(theta):
    """對照組：非 45 倍數的角度**確實**抓得到「漏掉 90− 轉換」。"""
    good, mp = _rect_at_math_angle(scan_waypoints(theta), 90.0 - theta)
    bad, _ = _rect_at_math_angle(scan_waypoints(theta), theta)
    assert bad.area > good.area * 1.2, (
        f"θ={theta}° 下錯誤轉換沒有變大（{bad.area / good.area:.3f}×），"
        "這條對照組失去意義。"
    )
    assert good.area < mp.envelope.area, "正確轉換應該比軸對齊小"


def test_missing_conversion_at_30_degrees_equals_doing_nothing():
    """更尖銳的一刀：θ=30° 時「漏掉 90−」算出來的矩形跟**完全沒轉**一模一樣。

    也就是說這個錯誤不只是「省得比較少」，而是**整個功能等於沒做**，
    卻仍然會回報 ``method='scan'``。
    """
    bad, mp = _rect_at_math_angle(scan_waypoints(30.0), 30.0)
    # rel 放到 2e-3：航點是用平面近似排的，再經 AEQD 投影會有次公尺變形，
    # 所以是「實質相同」而非位元相同。0.09% 的差距離「有做事」還差三個數量級。
    assert bad.area == pytest.approx(mp.envelope.area, rel=2e-3)


# ═══════════════════════════════════════════════════════════════════
#  1. 值層級：方向對齊
# ═══════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("theta", [30.0, 62.0, 123.4, 17.3, 210.0, -25.0])
def test_fence_edges_align_with_scan_angle(theta):
    """驗收判準①：圍籬每一條邊與掃描角的夾角（模 90°）< 1°。

    用「四條邊全部」而不是「長邊」：長邊判準是模 180° 的較弱檢查，
    在掃描線比條帶寬度短的作業區會挑到另一條邊而**意外通過**。
    """
    g = build(scan_waypoints(theta % 180.0), scan_angle_deg=theta)
    worst = _misalignment(g, theta % 180.0)
    assert worst < 1.0, f"θ={theta}° 圍籬最差的一條邊偏了 {worst:.4f}°"
    assert g.method == "scan"


def test_alignment_is_exact_not_merely_within_tolerance():
    """1° 是驗收門檻，但實作應該是**精確**的；留 1° 會讓小錯誤躲過去。"""
    g = build(scan_waypoints(30.0), scan_angle_deg=30.0)
    assert _misalignment(g, 30.0) < 1e-6


def test_long_edge_follows_the_scan_lines():
    """交接文件的原始判準（長邊方向 vs 掃描角，模 180°）也要成立。"""
    g = build(scan_waypoints(30.0), scan_angle_deg=30.0)
    assert _angdiff180(_long_edge_compass(g), 30.0) < 1.0


# ═══════════════════════════════════════════════════════════════════
#  2. 值層級：面積
# ═══════════════════════════════════════════════════════════════════
def test_area_ratio_collapses_to_one_at_45_degrees():
    """驗收判準②：45° 掃描角的矩形作業區，面積要降到必要面積的 1.0×（±2%）。

    ⚠️ 45° 只能驗「面積有沒有省下來」，**不能**驗方向對不對（見上面的
       45 倍數鎖）。方向由 30°/62° 那幾條負責。
    """
    wps = scan_waypoints(45.0)
    a_aabb = _area_m2(build(wps))
    a_scan = _area_m2(build(wps, scan_angle_deg=45.0))
    assert a_aabb / a_scan > 1.8, (
        f"45° 的浪費倍數只有 {a_aabb / a_scan:.2f}× —— 這組資料驗不出省了多少。"
    )
    # 「必要面積」＝ 0° 掃描時的矩形（同一組掃描線轉正之後的外框）
    a_need = _area_m2(build(scan_waypoints(0.0), scan_angle_deg=0.0))
    assert a_scan / a_need == pytest.approx(1.0, abs=0.02)


@pytest.mark.parametrize("theta", [0.0, 30.0, 45.0, 60.0, 90.0, 123.4])
def test_aligned_never_larger_when_scan_angle_matches_the_area(theta):
    """掃描角與作業區外形一致時（＝自動模式），對齊永遠不會比軸對齊大。

    ⚠️ 這個保證**只在這個前提下成立**。手動掃描角與外形打架時會反過來，
       見 ``test_manual_angle_fighting_the_area_can_enlarge_the_fence``。
    """
    wps = scan_waypoints(theta)
    assert _area_m2(build(wps, scan_angle_deg=theta)) <= _area_m2(build(wps)) * 1.001


# ═══════════════════════════════════════════════════════════════════
#  3. 對照組：軸對齊的掃描角不可以有任何變化
# ═══════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("theta", [0.0, 90.0, 180.0, 270.0])
def test_axis_aligned_scan_angles_change_nothing(theta):
    """驗收判準③：0°/90° 掃描角下，面積與方向都**不可以**變（證明沒有亂轉）。"""
    wps = scan_waypoints(theta % 180.0)
    plain = build(wps)
    aligned = build(wps, scan_angle_deg=theta)
    assert _area_m2(aligned) == pytest.approx(_area_m2(plain), rel=1e-6)
    assert _misalignment(aligned, 0.0) < 1e-6, "軸對齊掃描角下圍籬竟然轉了"


def test_no_angle_keeps_the_old_axis_aligned_behaviour():
    """驗收判準④：沒有角度就維持現行 aabb —— 手繪、匯入、讀回的航線靠這個。"""
    wps = scan_waypoints(30.0)
    g = build(wps, scan_angle_deg=None)
    assert g.method == "aabb"
    assert _misalignment(g, 0.0) < 1e-6, "沒給角度卻把圍籬轉走了"


def test_no_angle_result_is_bit_identical_to_before_the_feature():
    """沒給角度時頂點要跟純 aabb 完全一樣（不是「差不多」）。"""
    wps = scan_waypoints(62.0)
    a = build(wps, scan_angle_deg=None).vertices
    b = GeofenceConstraintManager(buffer_radius_m=BUF, method="aabb").build(waypoints=wps).geofence.vertices
    assert a == b


# ═══════════════════════════════════════════════════════════════════
#  4. 反例組：MRR ≠ 對齊掃描角（矩形作業區驗不出來，一定要用三角形）
# ═══════════════════════════════════════════════════════════════════
def test_mrr_is_a_different_answer_when_area_shape_disagrees_with_scan_angle():
    """驗收判準⑤：MRR 與「對齊掃描角」必須是**不同**的答案。

    分岔的條件是**作業區外形**與**掃描方向**不一致 —— 例如長軸在羅盤 75° 的
    細長作業區，卻用手動掃描角 30° 去掃。此時：
      * MRR 追的是點集外形 ⇒ 75°
      * 對齊掃描角追的是條帶方向 ⇒ 30°
    兩者差 45°，是模 90 之下能差到的最大值。

    ⚠️ 為什麼不用交接文件的三角形：那個形狀的 ``minimum_rotated_rectangle``
       有**兩個面積幾乎相同的候選**（651.9 m × 548.5 m 對上 650 m × 550 m，
       差 0.02%）。經緯度往返造成的次公尺誤差就足以讓 shapely 換一個選，
       方向從羅盤 147.5° 跳到 28.7° —— 後者離 30° 只有 1.3°，反例組會失效。
       反例組本身也需要是穩固的。
    """
    region = elongated_region(75.0)
    wps = clipped_scan_waypoints(region, 30.0)

    mrr = build(wps, method="mrr")
    scan = build(wps, scan_angle_deg=30.0)

    mrr_dir = _long_edge_compass(mrr)
    # 可觸發性：先證明這組資料真的能讓兩者分岔
    assert _angdiff90(mrr_dir, 30.0) > 30.0, (
        f"MRR 長邊落在羅盤 {mrr_dir:.1f}°，與掃描角 30° 只差 "
        f"{_angdiff90(mrr_dir, 30.0):.1f}° —— 分不出兩者，反例組失效。"
    )
    assert _misalignment(scan, 30.0) < 1e-6
    assert _misalignment(mrr, 30.0) > 30.0, "MRR 竟然對齊了掃描角，反例組失效"
    # 而且 MRR 確實比較小 —— 它解的是「面積最小」，不是「方向對齊」
    assert _area_m2(mrr) < _area_m2(scan)


def test_manual_angle_fighting_the_area_can_enlarge_the_fence():
    """⚠️⚠️ 已知且刻意的取捨：**對齊掃描角不保證面積變小**。

    使用者要的是「方向跟航線一致」，不是「面積最小」（面積最小是 MRR，
    而 MRR 幾乎垂直於航線）。當**手動**掃描角與作業區外形打架時，
    對齊後的圍籬會比軸對齊**大**。

    實測（細長作業區 4:1，手動掃描角固定 30°）：

        作業區長軸   30°    45°    60°    75°    90°   120°
        對齊/軸對齊 0.36×  0.71×  1.00×  1.36×  2.25×  0.42×

    自動模式不會落在這一帶 —— 自動掃描角就是作業區的最小寬度方向，
    也就是表格最左邊那一格（0.36×）。這條鎖的用意是把「可能變大」
    寫成明文，而不是等使用者在畫面上發現圍籬變胖。
    """
    wps = clipped_scan_waypoints(elongated_region(90.0), 30.0)
    scan = build(wps, scan_angle_deg=30.0)
    plain = build(wps)
    assert _misalignment(scan, 30.0) < 1e-6, "就算變大，方向仍必須是對的"
    assert _area_m2(scan) / _area_m2(plain) > 2.0, (
        "這組資料本來應該示範「對齊反而變大」，現在沒有了 —— "
        "要嘛實作改成在意面積（那就違背需求），要嘛這組資料失去代表性。"
    )


def test_auto_mode_geometry_shrinks_the_fence_a_lot():
    """對照：掃描角與作業區外形一致（＝自動模式）時，面積大幅下降。"""
    wps = clipped_scan_waypoints(elongated_region(30.0), 30.0)
    ratio = _area_m2(build(wps, scan_angle_deg=30.0)) / _area_m2(build(wps))
    assert ratio < 0.5, f"自動模式幾何下只降到 {ratio:.2f}×，預期 <0.5×"


def test_mrr_and_scan_agree_on_a_rectangle_which_is_why_it_is_a_bad_test():
    """把「矩形作業區測不出 MRR 與 scan 的差別」這件事本身鎖起來。"""
    wps = scan_waypoints(30.0)
    mrr_dir = _long_edge_compass(build(wps, method="mrr"))
    assert _angdiff90(mrr_dir, 30.0) < 1.0, (
        "矩形掃描線下 MRR 竟然沒對齊掃描角 —— 上面那條反例組的理由要重寫。"
    )


# ═══════════════════════════════════════════════════════════════════
#  5. 矛盾組合要吵，不要自己仲裁
# ═══════════════════════════════════════════════════════════════════
def test_mrr_plus_angle_raises_instead_of_silently_picking_one():
    with pytest.raises(GeofenceError, match="mrr"):
        build(scan_waypoints(30.0), scan_angle_deg=30.0, method="mrr")


def test_scan_method_without_angle_raises():
    with pytest.raises(GeofenceError, match="scan_angle_deg"):
        build(scan_waypoints(30.0), scan_angle_deg=None, method="scan")


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_angle_raises(bad):
    with pytest.raises(GeofenceError):
        build(scan_waypoints(30.0), scan_angle_deg=bad)


# ═══════════════════════════════════════════════════════════════════
#  6. 形狀不變式 —— 旋轉不可以破壞既有契約
# ═══════════════════════════════════════════════════════════════════
@pytest.mark.parametrize("theta", [None, 0.0, 30.0, 62.0, 123.4])
def test_still_four_vertices_and_alt_limits_unchanged(theta):
    wps = scan_waypoints(30.0)
    g = build(wps, scan_angle_deg=theta)
    plain = build(wps)
    assert g.vertex_count() == 4
    assert len(g.closed_ring()) == 5
    assert g.alt_min_m == pytest.approx(plain.alt_min_m)
    assert g.alt_max_m == pytest.approx(plain.alt_max_m)


@pytest.mark.parametrize("theta", [30.0, 62.0])
def test_buffer_is_applied_in_the_rotated_frame(theta):
    """⚠️ buffer 若在原始框架做，之後的 envelope 會把矩形收斂回軸對齊。

    判別法：對齊後的矩形，兩條邊長要恰好等於「轉正後的點集外框 + 2×buffer」。
    若 buffer 做錯框架，邊長會被撐大（且方向會掉回軸對齊）。
    """
    from shapely import affinity
    from shapely.geometry import MultiPoint

    wps = scan_waypoints(theta)
    fwd = _aeqd(REF_LAT, REF_LON)
    mp = MultiPoint([fwd.transform(lo, la) for la, lo, _ in wps])
    rot = affinity.rotate(mp, -(90.0 - theta), origin=(0.0, 0.0))
    x0, y0, x1, y1 = rot.bounds
    want = sorted([(x1 - x0) + 2 * BUF, (y1 - y0) + 2 * BUF])

    got = sorted({round(L, 3) for L, _ in _edges(build(wps, scan_angle_deg=theta))})
    assert len(got) == 2, f"對齊後的矩形應只有兩種邊長，得到 {got}"
    for a, b in zip(got, want):
        assert a == pytest.approx(b, rel=2e-3), (
            f"邊長 {got} 與「轉正外框 + 2×buffer」{want} 不符 —— "
            "buffer 很可能做在錯誤的框架裡。"
        )


def test_all_waypoints_stay_inside_the_rotated_fence():
    """旋轉過的圍籬仍必須真的包住每一個航點（inclusion 圍籬的本分）。"""
    from shapely.geometry import Point, Polygon

    theta = 62.0
    wps = scan_waypoints(theta)
    g = build(wps, scan_angle_deg=theta)
    fwd = _aeqd(REF_LAT, REF_LON)
    poly = Polygon([fwd.transform(lo, la) for la, lo in g.vertices])
    for la, lo, _ in wps:
        assert poly.contains(Point(fwd.transform(lo, la)))


# ═══════════════════════════════════════════════════════════════════
#  7. 慣例轉換 —— 對著**真正的規劃器**量，不是對著原始碼看
# ═══════════════════════════════════════════════════════════════════
def _longest_leg_compass(latlon_pts):
    best = None
    for i in range(len(latlon_pts) - 1):
        a, b = latlon_pts[i][:2], latlon_pts[i + 1][:2]
        de, dn = (b[1] - a[1]) * M_LON, (b[0] - a[0]) * M_LAT
        L = math.hypot(de, dn)
        if best is None or L > best[0]:
            best = (L, (90.0 - math.degrees(math.atan2(dn, de))) % 180.0)
    return best[1]


@pytest.mark.parametrize("angle", [30.0, 62.0, 123.4])
def test_coverage_angle_to_compass_matches_the_real_planner(angle):
    """``CoverageParameters.angle`` 是「自東順時針」—— 拿真的規劃器來證。"""
    from core.geometry.polygon import coverage_angle_to_compass
    from core.global_planner.coverage_planner import (
        CoveragePlanner, CoverageParameters, ScanPattern,
    )

    half = 400.0
    sq = [(REF_LAT - half / M_LAT, REF_LON - half / M_LON),
          (REF_LAT - half / M_LAT, REF_LON + half / M_LON),
          (REF_LAT + half / M_LAT, REF_LON + half / M_LON),
          (REF_LAT + half / M_LAT, REF_LON - half / M_LON)]
    path = CoveragePlanner().plan_coverage(sq, CoverageParameters(
        spacing=80.0, angle=angle, pattern=ScanPattern.GRID,
        is_fixed_wing=False, circle_base_altitude=100.0,
        grid_base_altitude=100.0,
    ))
    flown = _longest_leg_compass(path)
    predicted = coverage_angle_to_compass(angle) % 180.0
    assert _angdiff180(flown, predicted) < 1.0, (
        f"angle={angle} 實飛掃描線羅盤 {flown:.2f}°，"
        f"轉換函式說 {predicted:.2f}° —— 慣例對不上。"
    )
    # 可觸發性：這個角度真的分辨得出 +90 與 −angle 兩套慣例
    assert _angdiff180(predicted, (-angle) % 180.0) > 10.0


@pytest.mark.parametrize("angle", [30.0, 62.0, 123.4])
def test_fw_scan_angle_to_compass_matches_the_real_planner(angle):
    """``CoveragePathPlanner`` 是「自北逆時針」—— 與上面**不同**的慣例。"""
    from core.geometry.polygon import fw_scan_angle_to_compass
    from core.global_planner.coverage_path_planner import (
        CoveragePathPlanner, FixedWingConstraints,
    )

    sq_m = [(-400.0, -400.0), (400.0, -400.0), (400.0, 400.0), (-400.0, 400.0)]
    res = CoveragePathPlanner(FixedWingConstraints()).plan_coverage(
        sq_m, scan_angle_deg=angle, spacing_m=80.0, coord_type="metric")
    best = None
    for sl in res.scan_lines:
        dx, dy = sl.end[0] - sl.start[0], sl.end[1] - sl.start[1]
        L = math.hypot(dx, dy)
        if best is None or L > best[0]:
            best = (L, (90.0 - math.degrees(math.atan2(dy, dx))) % 180.0)
    predicted = fw_scan_angle_to_compass(angle) % 180.0
    assert _angdiff180(best[1], predicted) < 1.0
    assert _angdiff180(predicted, (angle + 90.0) % 180.0) > 10.0


def test_the_two_planner_conventions_really_do_disagree():
    """把「同一個滑桿值、兩條路徑飛不同方向」這個既有歧異鎖成明文。

    這不是本次改動造成的，但圍籬必須跟著**各自那條路徑**走，
    所以一旦有人把兩邊統一了，這條會變紅，提醒去把 main_window 的
    兩處轉換一起收斂。
    """
    from core.geometry.polygon import (
        coverage_angle_to_compass, fw_scan_angle_to_compass,
    )
    a = 30.0
    assert _angdiff180(coverage_angle_to_compass(a),
                       fw_scan_angle_to_compass(a)) == pytest.approx(30.0, abs=1e-9)


# ═══════════════════════════════════════════════════════════════════
#  8. 接線 —— 角度真的從 display_* 一路走到圍籬
# ═══════════════════════════════════════════════════════════════════
# 用「未綁定方法 + 假 self」呼叫真正的實作（與 tests/test_dccpp_coverage_retention.py
# 同手法），不需要真的建出 Qt widget，但跑的是**生產程式碼本身**，
# 不是原始碼字串比對。
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from ui.widgets.dual_map_widget import DualMapWidget
    _DUAL_OK = True
except Exception:                                    # pragma: no cover
    _DUAL_OK = False

pytestmark_dual = pytest.mark.skipif(
    not _DUAL_OK, reason="ui.widgets.dual_map_widget import failed")


class _FakeMap:
    """吞掉任何 map_2d / map_3d 呼叫並記錄下來。"""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        def _f(*a, **k):
            self.calls.append((name, a, k))
        return _f


class _FakeSignal:
    def __init__(self):
        self.emitted = []

    def emit(self, v):
        self.emitted.append(v)


class _FakeDual:
    """`DualMapWidget` 未綁定方法所需要的最小 self。"""

    def __init__(self):
        self.map_2d = _FakeMap()
        self.map_3d = _FakeMap()
        self.fence_built = _FakeSignal()
        self._fence_visible = True
        self._last_fence_bundle = "STALE-SENTINEL"
        self.seen = []
        # 綁真正的實作 —— 這幾條測試要跑生產程式碼，不是替身
        self._auto_build_fence = DualMapWidget._auto_build_fence.__get__(self)
        self._swarm_raw_to_paths = DualMapWidget._swarm_raw_to_paths
        self._coverage_to_paths = DualMapWidget._coverage_to_paths

    def _set_fence_2d(self, g):
        pass

    def _clear_fence_2d(self):
        pass

    def _spy_auto_build_fence(self, paths_list, altitude=None, buffer_m=30.0,
                              scan_angle_deg=None, method='aabb'):
        self.seen.append(scan_angle_deg)

    def __getattr__(self, n):
        """其餘地圖方法一律當成無害的畫圖動作吞掉。

        ⚠ 只對**不存在**的屬性生效；`_auto_build_fence` / `_last_fence_bundle`
        都在 ``__init__`` 明確設好，不會被這裡蓋掉。
        """
        def _noop(*a, **k):
            return None
        return _noop


def _ring_paths(theta, alt=120.0):
    wps = scan_waypoints(theta, alt=alt)
    return [[(la, lo, al) for la, lo, al in wps]]


@pytestmark_dual
@pytest.mark.parametrize("method,args", [
    ("display_path", lambda p: (p[0], 120.0)),
    ("display_paths", lambda p: (p, 120.0)),
    ("display_fw_paths", lambda p: (p[0], p[0], p[0])),
    # display_swarm_raw 不在這裡：2026-10-08 使用者定調群飛（DCCPP/HARVEST）圍籬改成
    # 「貼合航線的凸多邊形」（method='hull'，不吃掃描角），見 tests/test_fence_hull.py
])
def test_display_methods_forward_the_scan_angle(method, args):
    """五個入口都必須把角度往下傳，而且**原樣**傳（不轉換、不補預設）。"""
    fake = _FakeDual()
    fake._auto_build_fence = fake._spy_auto_build_fence
    paths = _ring_paths(30.0)
    getattr(DualMapWidget, method)(fake, *args(paths), scan_angle_deg=62.0)
    assert fake.seen == [62.0], f"{method} 沒有把 62.0 傳到 _auto_build_fence"


@pytestmark_dual
@pytest.mark.parametrize("method,args", [
    ("display_path", lambda p: (p[0], 120.0)),
    ("display_paths", lambda p: (p, 120.0)),
    ("display_fw_paths", lambda p: (p[0], p[0], p[0])),
    # display_swarm_raw 不在這裡：2026-10-08 使用者定調群飛（DCCPP/HARVEST）圍籬改成
    # 「貼合航線的凸多邊形」（method='hull'，不吃掃描角），見 tests/test_fence_hull.py
])
def test_display_methods_default_to_no_angle(method, args):
    """不傳角度時必須是 None —— 手繪 / 匯入 / 非覆蓋演算法靠這個維持軸對齊。"""
    fake = _FakeDual()
    fake._auto_build_fence = fake._spy_auto_build_fence
    paths = _ring_paths(30.0)
    getattr(DualMapWidget, method)(fake, *args(paths))
    assert fake.seen == [None]


@pytestmark_dual
def test_end_to_end_display_paths_produces_a_hull_fence():
    """整條鏈路的值層級驗收（2026-10-08 新定調：底層預設貼合航線）。

    餵 30° 斜向條帶進 display_paths：圍籬是 'hull'、掃描角給不給結果都一樣（不吃角度）、
    每個航點離圍籬邊至少 buffer，而且比同一組點的軸對齊矩形明顯小。
    """
    fake = _FakeDual()
    DualMapWidget.display_paths(fake, _ring_paths(30.0), 120.0, scan_angle_deg=30.0)
    g = fake._last_fence_bundle.geofence
    assert g.method == "hull"

    plain = _FakeDual()
    DualMapWidget.display_paths(plain, _ring_paths(30.0), 120.0)
    assert plain._last_fence_bundle.geofence.vertices == g.vertices, "給不給掃描角，貼合航線的圍籬應該相同"

    pts = fake._last_fence_bundle.waypoints
    assert _min_margin_m(g, pts) >= BUF - 0.5, "有航點離圍籬邊不到 buffer"
    rect = GeofenceConstraintManager(buffer_radius_m=BUF, method="aabb").build(waypoints=pts).geofence
    assert _area_m2(rect) / _area_m2(g) > 1.3, "貼合航線的圍籬沒有比軸對齊矩形小"


@pytestmark_dual
def test_build_failure_clears_the_stale_fence(monkeypatch):
    """⚠️ 建構失敗必須**清掉**舊圍籬，不能讓上一趟的圍籬留在畫面與上傳來源。

    舊版失敗時只記一行 warning 就 return，`_last_fence_bundle` 保持原值
    ⇒ 兩張圖繼續畫著上一趟的圍籬，而「上傳電子圍籬」會把那個舊圍籬送上飛控。

    （2026-10-08 起底層預設是 'hull'，不吃掃描角 ⇒ 不能再用 NaN 角度觸發失敗，
     改成直接讓 build() 丟 GeofenceError。）
    """
    import mission.geofence_manager as gm

    def _boom(self, *a, **k):
        raise GeofenceError("模擬建構失敗")

    monkeypatch.setattr(gm.GeofenceConstraintManager, "build", _boom)
    fake = _FakeDual()
    DualMapWidget.display_paths(fake, _ring_paths(30.0), 120.0, scan_angle_deg=30.0)
    assert fake._last_fence_bundle is None, "建構失敗卻留著上一趟的圍籬"
    assert fake.fence_built.emitted == [None]
    assert any(c[0] == "clear_geofence" for c in fake.map_3d.calls), (
        "建構失敗時沒有清掉 3D 圖上的舊圍籬"
    )


@pytestmark_dual
def test_the_failure_path_is_reachable_at_all():
    """可觸發性：先證明 NaN 角度**真的**會讓 build() 失敗，
    否則上面那條測試量的是「成功路徑也剛好清了」。"""
    with pytest.raises(GeofenceError):
        build(scan_waypoints(30.0), scan_angle_deg=float("nan"))


# ═══════════════════════════════════════════════════════════════════
#  9. main_window 的接線 —— 圍籬對齊的是「實際飛出來的線」
# ═══════════════════════════════════════════════════════════════════
# ⚠️⚠️⚠️ 2026-09-04 真實事故:有人把 main_window 的兩個轉換函式
#   **互相改名**(`coverage_angle_to_compass as fw_scan_angle_to_compass`
#   與反向),圍籬因此可能偏 90°,而上面那 65 條測試**全綠** ——
#   因為它們鎖的是 core/geometry/polygon.py 裡的函式本身,
#   沒有鎖 main_window「呼叫哪一個」。
#
#   所以這一節不看角度參數、也不看用了哪個轉換函式,
#   直接比對「圍籬邊」與「**實際飛出來的掃描線**」。
#   任何綁錯、改名、傳錯值,都會在這裡變紅。

try:
    from ui.main_window import MainWindow
    _MW_OK = True
except Exception:                                    # pragma: no cover
    _MW_OK = False

pytestmark_mw = pytest.mark.skipif(
    not _MW_OK, reason="ui.main_window import failed")


class _FakeWin:
    """`MainWindow.on_preview_paths` 所需的最小 self(多旋翼網格路徑)。"""

    def __init__(self, angle, half_m=400.0):
        self.current_algorithm = "grid"
        self.current_vehicle_type = "多旋翼"
        self.circle_center = None
        self.corners = [
            (REF_LAT - half_m / M_LAT, REF_LON - half_m / M_LON),
            (REF_LAT - half_m / M_LAT, REF_LON + half_m / M_LON),
            (REF_LAT + half_m / M_LAT, REF_LON + half_m / M_LON),
            (REF_LAT + half_m / M_LAT, REF_LON - half_m / M_LON),
        ]
        self.flight_params = {
            "spacing": 80.0, "angle": angle, "altitude": 100.0, "speed": 8.0,
            "subdivisions": 1, "turn_radius": 50.0, "yaw_speed": 30.0,
            "region_spacing": 0.0, "grid_alt_step": 0.0,
        }
        self.nfz_zones = []
        self.sub_paths = []
        self.waypoints = []
        self.map_widget = _FakeDual()
        for m in ("display_path", "display_paths", "display_fw_paths"):
            setattr(self.map_widget, m,
                    getattr(DualMapWidget, m).__get__(self.map_widget))
        self.waypoint_label = self.distance_label = _FakeMap()
        # staticmethod:從 class 取到的是純函式，不要再 __get__
        self._trim_to_wp_limit = MainWindow._trim_to_wp_limit
        self._haversine = MainWindow._haversine
        self._update_transit_paths = lambda *a, **k: None

    def __getattr__(self, n):
        def _f(*a, **k):
            return None
        return _f


def _flown_scan_line_compass(latlon_pts):
    """量**實際航點**裡最長那一段的羅盤方位(mod 180)。"""
    best = None
    for i in range(len(latlon_pts) - 1):
        a, b = latlon_pts[i][:2], latlon_pts[i + 1][:2]
        de, dn = (b[1] - a[1]) * M_LON, (b[0] - a[0]) * M_LAT
        L = math.hypot(de, dn)
        if best is None or L > best[0]:
            best = (L, (90.0 - math.degrees(math.atan2(dn, de))) % 180.0)
    return best[1]


@pytestmark_mw
@pytest.mark.parametrize("angle", [30.0, 62.0, 123.4])
def test_fence_hugs_the_lines_actually_flown(angle, monkeypatch):
    """固定翼 on_preview_paths：圍籬貼合**實際飛出來的**航線（2026-10-08 新定調，底層預設 'hull'）。

    原本這條驗「圍籬邊平行/垂直於掃描線」（09-04 的矩形對齊）；使用者改成「規劃器出來的圍籬
    一律貼合路徑」後，改驗：每個航點離圍籬邊至少 buffer，且比同一組點的軸對齊矩形小。
    """
    import PyQt6.QtWidgets as QtW
    for _n in ("warning", "information", "critical", "question", "about"):
        monkeypatch.setattr(QtW.QMessageBox, _n,
                            staticmethod(lambda *a, **k: 0), raising=False)

    w = _FakeWin(angle)
    MainWindow.on_preview_paths(w)

    assert w.waypoints, "on_preview_paths 沒有產出航點,這條測試什麼都沒驗到"
    flown = _flown_scan_line_compass(w.waypoints)

    # 可觸發性:飛出來的方向必須不是軸對齊,否則對齊與否分辨不出來
    assert _angdiff90(flown, 0.0) > 5.0, (
        f"實飛掃描線落在羅盤 {flown:.2f}°,太接近軸對齊,這個角度沒有鑑別力"
    )

    bundle = w.map_widget._last_fence_bundle
    assert bundle is not None, "沒有建出圍籬"
    g = bundle.geofence
    assert g.method == "hull", f"固定翼圍籬應貼合航線，實際 {g.method}"
    margin = _min_margin_m(g, bundle.waypoints)
    assert margin >= g.buffer_radius_m - 0.5, (
        f"面板角度 {angle}°：有航點離圍籬邊只有 {margin:.1f} m（buffer {g.buffer_radius_m} m）")
    rect = GeofenceConstraintManager(buffer_radius_m=g.buffer_radius_m, method="aabb").build(
        waypoints=bundle.waypoints).geofence
    assert _area_m2(g) < _area_m2(rect), "貼合航線的圍籬竟然不比軸對齊矩形小"


@pytestmark_mw
def test_swapping_the_two_converters_would_be_caught(monkeypatch):
    """可觸發性的反面:把 main_window 綁到**錯的**轉換函式,上面那條要變紅。

    ⚠️ 這正是 2026-09-04 真實發生過的事故形態。沒有這一條,
       上面那條測試是綠是紅都證明不了它有在看東西。
    """
    import PyQt6.QtWidgets as QtW
    import ui.main_window as mw
    for _n in ("warning", "information", "critical", "question", "about"):
        monkeypatch.setattr(QtW.QMessageBox, _n,
                            staticmethod(lambda *a, **k: 0), raising=False)
    # 綁成另一套慣例(等同把兩個函式互換)
    monkeypatch.setattr(mw, "coverage_angle_to_compass",
                        mw.fw_scan_angle_to_compass)

    w = _FakeWin(30.0)
    MainWindow.on_preview_paths(w)
    flown = _flown_scan_line_compass(w.waypoints)
    worst = _misalignment(w.map_widget._last_fence_bundle.geofence, flown)
    assert worst > 10.0, (
        f"綁錯轉換函式之後偏差只有 {worst:.2f}° —— 上面那條鎖抓不到綁錯，"
        "等於沒有保護。"
    )
