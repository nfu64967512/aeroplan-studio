# -*- coding: utf-8 -*-
"""mission/waypoint_file.py —— 航點檔讀寫與驗證的值層級鎖。

防的是這幾件事：
① 載入把壞檔「悄悄變成空任務」→ 之後 Write WPs 把飛控上原本的航線清掉。
② 欄位順序 / 精度 / seq 偏移的靜默錯誤（例如 %.6f 把 4e-07 壓成 0）。
③ 驗證規則被拿掉卻沒有任何測試發現 ⇒ 每條規則都同時測「會響」與「近似但不該響」。
④ .plan 的 ComplexItem 被丟掉而使用者以為整片掃描都在。

測試不需要 Qt、不開任何序列埠、不連飛控。語料測試在 repo 沒有真實檔案時 skip。
"""
from __future__ import annotations

import ast
import glob
import math
import os
import pathlib
import random

import pytest

from mission import waypoint_file as wf
from mission.waypoint_file import (
    LoadedMission, MissionFileError, MissionIssue, items_to_wpl_text,
    load_mission_file, parse_plan_obj, parse_wpl_text, save_wpl, validate_items,
)

REPO = pathlib.Path(__file__).resolve().parents[1]


# ── 建構小工具 ──────────────────────────────────────────────────────────
def it(seq, cmd=16, lat=23.95, lon=120.9, alt=100.0, frame=3,
       p=(0.0, 0.0, 0.0, 0.0), current=0, ac=1):
    return {"seq": seq, "command": cmd, "frame": frame, "lat": lat, "lon": lon,
            "alt": alt, "param1": p[0], "param2": p[1], "param3": p[2],
            "param4": p[3], "current": current, "autocontinue": ac}


def home():
    return it(0, 16, 23.95, 120.9, 0.0, frame=0, current=1)


def mission(*rest):
    """home + rest，seq 依位置編號。rest 為 (cmd, kwargs) 或現成 dict。"""
    out = [home()]
    for k, r in enumerate(rest, start=1):
        if isinstance(r, dict):
            d = dict(r)
            d["seq"] = k
            out.append(d)
        else:
            cmd, kw = r
            out.append(it(k, cmd, **kw))
    return out


def good():
    return mission((84, dict(alt=30.0)), (16, dict(lat=23.951)), (16, dict(lat=23.952)),
                   (85, dict(lat=23.953, alt=0.0)))


def codes(issues, level=None):
    return {i.code for i in issues if level is None or i.level == level}


def wpl_line(k, cmd=16, frame=3, lat="23.95", lon="120.9", alt="100", cur=0,
             p=("0", "0", "0", "0"), ac=1, sep="\t"):
    return sep.join([str(k), str(cur), str(frame), str(cmd), *p, lat, lon, alt, str(ac)])


GOLDEN = "\n".join([
    "QGC WPL 110",
    "0\t1\t0\t16\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t23.95000000\t120.90000000\t9.000000\t1",
    "1\t0\t3\t84\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t23.95010000\t120.90010000\t60.000000\t1",
    "2\t0\t3\t16\t0.00000000\t30.00000000\t0.00000000\t0.00000000\t23.95020000\t120.90200000\t100.000000\t1",
    "3\t0\t2\t178\t0.00000000\t21.50000000\t-1.00000000\t0.00000000\t0.00000000\t0.00000000\t0.000000\t1",
    "4\t0\t3\t85\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t23.95030000\t120.90300000\t0.000000\t1",
]) + "\n"


# ═══════════════════════════ WPL 解析 ═══════════════════════════════════
class TestWplGolden:
    def test_fields_land_in_the_right_columns(self):
        lm = parse_wpl_text(GOLDEN)
        assert lm.fmt == "wpl" and not lm.has_errors
        assert len(lm.items) == 5
        assert lm.items[0] == it(0, 16, 23.95, 120.9, 9.0, frame=0, current=1)
        assert lm.items[2] == it(2, 16, 23.9502, 120.902, 100.0, frame=3, p=(0, 30, 0, 0))
        j = lm.items[3]
        assert (j["command"], j["frame"], j["param2"], j["param3"]) == (178, 2, 21.5, -1.0)
        assert lm.items[4]["command"] == 85 and lm.items[4]["alt"] == 0.0

    def test_lat_and_lon_are_not_swapped(self):
        lm = parse_wpl_text(GOLDEN)
        assert lm.items[1]["lat"] == 23.9501 and lm.items[1]["lon"] == 120.9001

    def test_item_shape_matches_recv_mission_keys(self):
        keys = {"seq", "command", "frame", "lat", "lon", "alt", "param1", "param2",
                "param3", "param4", "current", "autocontinue"}
        for x in parse_wpl_text(GOLDEN).items:
            assert set(x) == keys

    def test_types(self):
        for x in parse_wpl_text(GOLDEN).items:
            for k in ("seq", "command", "frame", "current", "autocontinue"):
                assert type(x[k]) is int
            for k in ("lat", "lon", "alt", "param1", "param2", "param3", "param4"):
                assert type(x[k]) is float

    def test_spaces_instead_of_tabs(self):
        assert parse_wpl_text(GOLDEN.replace("\t", "  ")).items == parse_wpl_text(GOLDEN).items

    def test_mixed_whitespace_runs(self):
        txt = GOLDEN.replace("\t", " \t ")
        assert parse_wpl_text(txt).items == parse_wpl_text(GOLDEN).items

    def test_bom(self):
        assert parse_wpl_text("﻿" + GOLDEN).items == parse_wpl_text(GOLDEN).items

    def test_bom_through_file_loader(self, tmp_path):
        p = tmp_path / "b.waypoints"
        p.write_bytes(b"\xef\xbb\xbf" + GOLDEN.encode())
        assert load_mission_file(p).items == parse_wpl_text(GOLDEN).items

    def test_crlf_and_mixed_line_endings(self):
        crlf = GOLDEN.replace("\n", "\r\n")
        assert parse_wpl_text(crlf).items == parse_wpl_text(GOLDEN).items
        parts = GOLDEN.split("\n")
        mixed = "\r\n".join(parts[:2]) + "\n" + "\r".join(parts[2:4]) + "\r\n" + "\n".join(parts[4:])
        assert parse_wpl_text(mixed).items == parse_wpl_text(GOLDEN).items

    def test_comments_blank_lines_and_trailing_space(self):
        lines = GOLDEN.split("\n")
        txt = "\n".join([lines[0], "# 註解", "", lines[1] + "   ", "   # 縮排註解",
                         *lines[2:5], "", lines[5]])
        assert parse_wpl_text(txt).items == parse_wpl_text(GOLDEN).items

    def test_comment_before_header(self):
        assert parse_wpl_text("# hi\n\n" + GOLDEN).items == parse_wpl_text(GOLDEN).items

    def test_exponent_number_forms(self):
        txt = "QGC WPL 110\n" + wpl_line(0, alt="1.0E-05", lat="1e-3", lon="-1.5E+2") + "\n"
        x = parse_wpl_text(txt).items[0]
        assert x["alt"] == 1e-5 and x["lat"] == 1e-3 and x["lon"] == -150.0

    def test_version_100_accepted_quietly_other_warns(self):
        assert "bad_version" not in codes(parse_wpl_text(GOLDEN.replace("110", "100", 1)).issues)
        lm = parse_wpl_text(GOLDEN.replace("110", "120", 1))
        assert "bad_version" in codes(lm.issues, "warning") and len(lm.items) == 5

    def test_index_column_disagreement_warns_and_line_order_wins(self):
        lines = GOLDEN.split("\n")
        lines[3] = "7" + lines[3][1:]           # 第 4 行的 INDEX 改成 7
        lm = parse_wpl_text("\n".join(lines))
        w = [i for i in lm.issues if i.code == "index_mismatch"]
        assert len(w) == 1 and w[0].seq == 2 and "第 4 行" in w[0].message
        assert [x["seq"] for x in lm.items] == [0, 1, 2, 3, 4]

    def test_index_matching_is_quiet(self):
        assert "index_mismatch" not in codes(parse_wpl_text(GOLDEN).issues)

    def test_many_index_mismatches_are_summarised(self):
        body = [wpl_line(k + 10, lat=f"23.{k + 1:02d}") for k in range(9)]
        lm = parse_wpl_text("QGC WPL 110\n" + "\n".join(body) + "\n")
        assert sum(i.code == "index_mismatch" for i in lm.issues) == 5
        more = [i for i in lm.issues if i.code == "index_mismatch_more"]
        assert len(more) == 1 and "4" in more[0].message


class TestWplErrorPaths:
    def _err(self, text):
        with pytest.raises(MissionFileError) as ei:
            parse_wpl_text(text)
        return ei.value

    def test_short_row_reports_line_and_text(self):
        good_row = wpl_line(0)
        bad = "\t".join(good_row.split("\t")[:11])
        e = self._err(f"QGC WPL 110\n# c\n\n{good_row}\n{bad}\n")
        assert e.line == 5 and "第 5 行" in str(e) and "11 欄" in str(e)
        assert e.text == bad and repr(bad) in str(e)

    def test_long_row(self):
        e = self._err("QGC WPL 110\n" + wpl_line(0) + "\t9\n")
        assert e.line == 2 and "13 欄" in str(e)

    def test_non_numeric_param(self):
        row = wpl_line(0, p=("0", "abc", "0", "0"))
        e = self._err("QGC WPL 110\n" + wpl_line(0) + "\n" + row + "\n")
        assert e.line == 3 and "P2" in str(e) and "abc" in str(e)

    @pytest.mark.parametrize("field_i,name", [(0, "INDEX"), (1, "CURRENT"), (2, "COORD_FRAME"),
                                              (3, "COMMAND"), (11, "AUTOCONTINUE")])
    def test_non_integer_int_columns(self, field_i, name):
        cols = wpl_line(0).split("\t")
        cols[field_i] = "3.5"
        e = self._err("QGC WPL 110\n" + "\t".join(cols) + "\n")
        assert e.line == 2 and name in str(e)

    @pytest.mark.parametrize("field_i,name", [(8, "LAT"), (9, "LON"), (10, "ALT")])
    def test_non_numeric_position(self, field_i, name):
        cols = wpl_line(0).split("\t")
        cols[field_i] = "x1"
        e = self._err("QGC WPL 110\n" + "\t".join(cols) + "\n")
        assert e.line == 2 and name in str(e)

    def test_bad_header_line_number(self):
        e = self._err("# c\n\nHELLO 110\n" + wpl_line(0) + "\n")
        assert e.line == 3 and "表頭" in str(e)

    def test_nan_token_is_parsed_then_blocked_by_validation(self):
        txt = "QGC WPL 110\n" + wpl_line(0, alt="nan") + "\n" + wpl_line(1) + "\n"
        lm = parse_wpl_text(txt)
        assert lm.has_errors and "not_finite" in codes(lm.issues, "error")


class TestNeverSilentlyEmpty:
    @pytest.mark.parametrize("text", ["", "   \n\t\r\n  ", "﻿", "# 只有註解\n# 二\n"])
    def test_text_without_a_mission_raises(self, text):
        with pytest.raises(MissionFileError):
            parse_wpl_text(text)

    @pytest.mark.parametrize("content", [b"", b"  \r\n\t\n", b"\xef\xbb\xbf"])
    def test_files_without_content_raise(self, tmp_path, content):
        p = tmp_path / "x.waypoints"
        p.write_bytes(content)
        with pytest.raises(MissionFileError):
            load_mission_file(p)

    def test_header_only_file_returns_an_empty_error(self, tmp_path):
        p = tmp_path / "h.waypoints"
        p.write_text("QGC WPL 110\n", encoding="utf-8")
        lm = load_mission_file(p)
        assert lm.items == [] and lm.has_errors and "empty" in codes(lm.errors)

    def test_home_only_is_also_empty(self):
        lm = parse_wpl_text("QGC WPL 110\n" + wpl_line(0, frame=0, cur=1) + "\n")
        assert "empty" in codes(lm.errors)

    def test_plan_with_no_items_is_an_error(self):
        lm = parse_plan_obj({"fileType": "Plan", "mission": {"items": [],
                            "plannedHomePosition": [23.95, 120.9, 0]}})
        assert "empty" in codes(lm.errors)


class TestLoader:
    def test_missing_file(self, tmp_path):
        with pytest.raises(MissionFileError):
            load_mission_file(tmp_path / "nope.waypoints")

    def test_directory(self, tmp_path):
        with pytest.raises(MissionFileError):
            load_mission_file(tmp_path)

    def test_non_utf8(self, tmp_path):
        p = tmp_path / "x.waypoints"
        p.write_bytes(b"QGC WPL 110\n\xff\xfe\x80\n")
        with pytest.raises(MissionFileError, match="UTF-8"):
            load_mission_file(p)

    def test_unrecognised_text_names_supported_formats(self, tmp_path):
        p = tmp_path / "x.txt"
        p.write_text("hello world\n", encoding="utf-8")
        with pytest.raises(MissionFileError) as ei:
            load_mission_file(p)
        assert "QGC WPL" in str(ei.value) and ".plan" in str(ei.value)

    def test_broken_json_reports_position(self, tmp_path):
        p = tmp_path / "x.plan"
        p.write_text('{\n "fileType": "Plan",\n "mission": \n', encoding="utf-8")
        with pytest.raises(MissionFileError, match="JSON"):
            load_mission_file(p)

    def test_content_beats_extension(self, tmp_path):
        a = tmp_path / "wpl_named.plan"
        a.write_text(GOLDEN, encoding="utf-8")
        assert load_mission_file(a).fmt == "wpl"
        b = tmp_path / "plan_named.waypoints"
        b.write_text(_plan_text(), encoding="utf-8")
        lm = load_mission_file(b)
        assert lm.fmt == "plan" and len(lm.items) == 4

    def test_path_object_and_str(self, tmp_path):
        p = tmp_path / "g.waypoints"
        p.write_text(GOLDEN, encoding="utf-8")
        assert load_mission_file(p).items == load_mission_file(str(p)).items
        assert load_mission_file(p).path == str(p)

    def test_json_that_is_not_a_plan(self, tmp_path):
        p = tmp_path / "x.json"
        p.write_text('{"hello": 1}', encoding="utf-8")
        with pytest.raises(MissionFileError):
            load_mission_file(p)

    def test_summary_text(self):
        lm = parse_wpl_text("QGC WPL 110\n" + wpl_line(0, frame=0, cur=1) + "\n")
        s = lm.summary_text()
        assert "1 個錯誤" in s and "[錯誤]" in s and "空的" in s
        ok = parse_wpl_text(GOLDEN).summary_text()
        assert "5 個項目" in ok and "0 個錯誤" in ok


# ═══════════════════════════ .plan ═════════════════════════════════════
def simple(cmd, frame=3, params=None, **kw):
    d = {"type": "SimpleItem", "command": cmd, "frame": frame, "autoContinue": True,
         "params": params if params is not None else [0, 0, 0, 0, 23.95, 120.9, 50.0]}
    d.update(kw)
    return d


def plan(items, home=(23.95, 120.9, 9.0), **mission_kw):
    m = {"items": items, **mission_kw}
    if home is not None:
        m["plannedHomePosition"] = list(home)
    return {"fileType": "Plan", "version": 1, "mission": m}


def _plan_text():
    import json
    return json.dumps(plan([
        simple(84, params=[0, 0, 0, 0, 23.9501, 120.9001, 60.0], doJumpId=1),
        simple(16, params=[0, 30, 0, 0, 23.9502, 120.902, 100.0], doJumpId=2),
        simple(85, params=[0, 0, 0, 0, 23.9503, 120.903, 0.0], doJumpId=3),
    ]))


class TestPlan:
    def test_simple_items_and_synthetic_home(self):
        lm = parse_plan_obj(plan([
            simple(84, params=[0, 0, 0, 0, 23.9501, 120.9001, 60.0]),
            simple(16, params=[1, 30, 0, 0, 23.9502, 120.902, 100.0], autoContinue=False),
            simple(85, params=[0, 0, 0, 0, 23.9503, 120.903, 0.0]),
        ]))
        assert lm.fmt == "plan" and not lm.has_errors
        h = lm.items[0]
        assert (h["seq"], h["command"], h["frame"], h["current"]) == (0, 16, 0, 1)
        assert (h["lat"], h["lon"], h["alt"]) == (23.95, 120.9, 9.0)
        w = lm.items[2]
        assert (w["seq"], w["command"], w["param1"], w["param2"]) == (2, 16, 1.0, 30.0)
        assert (w["lat"], w["lon"], w["alt"], w["autocontinue"]) == (23.9502, 120.902, 100.0, 0)
        assert lm.items[1]["lat"] == 23.9501 and lm.items[1]["lon"] == 120.9001

    def test_frame_field_is_authoritative(self):
        lm = parse_plan_obj(plan([simple(84, frame=0, AltitudeMode=1),
                                  simple(85, frame=10, params=[0, 0, 0, 0, 23.96, 120.9, 0.0])]))
        assert [x["frame"] for x in lm.items[1:]] == [0, 10]

    def test_altitude_mode_disagreement_warns_agreement_is_quiet(self):
        bad = parse_plan_obj(plan([simple(84, frame=3, AltitudeMode=2)]))
        assert "plan_altmode_mismatch" in codes(bad.warnings)
        for mode, frame in ((1, 3), (2, 0), (3, 0), (4, 10), (0, 3)):
            lm = parse_plan_obj(plan([simple(84, frame=frame, AltitudeMode=mode)]))
            assert "plan_altmode_mismatch" not in codes(lm.issues), (mode, frame)

    def test_null_params_are_zero_for_unused_slots(self):
        lm = parse_plan_obj(plan([simple(178, frame=2, params=[None, 21.5, None, None, None, None, None])]))
        x = lm.items[1]
        assert (x["param1"], x["param2"], x["param3"], x["lat"], x["lon"], x["alt"]) == (0, 21.5, 0, 0, 0, 0)
        assert "not_finite" not in codes(lm.issues)

    def test_null_position_on_location_command_is_an_error(self):
        lm = parse_plan_obj(plan([simple(16, params=[0, 0, 0, 0, None, 120.9, 50.0])]))
        assert "not_finite" in codes(lm.errors)

    def test_literal_nan_and_inf_in_location_are_errors(self):
        for bad in (math.nan, math.inf, -math.inf):
            lm = parse_plan_obj(plan([simple(16, params=[0, 0, 0, 0, 23.95, 120.9, bad])]))
            assert "not_finite" in codes(lm.errors)

    def test_unknown_type_is_an_error_not_a_skip(self):
        lm = parse_plan_obj(plan([simple(84), {"type": "MysteryItem"}]))
        assert "plan_unknown_type" in codes(lm.errors) and lm.has_errors

    def test_complex_with_embedded_items_is_flattened_with_warning(self):
        cx = {"type": "ComplexItem", "complexItemType": "survey",
              "TransectStyleComplexItem": {"Items": [
                  simple(16, params=[0, 0, 0, 0, 23.951, 120.901, 50.0]),
                  simple(16, params=[0, 0, 0, 0, 23.952, 120.902, 50.0])]}}
        lm = parse_plan_obj(plan([simple(84), cx, simple(85, params=[0, 0, 0, 0, 23.96, 120.9, 0])]))
        assert [x["command"] for x in lm.items] == [16, 84, 16, 16, 85]
        assert [x["seq"] for x in lm.items] == [0, 1, 2, 3, 4]
        assert lm.items[2]["lat"] == 23.951 and lm.items[3]["lat"] == 23.952
        assert "complex_flattened" in codes(lm.warnings) and not lm.has_errors
        assert "survey" in [i for i in lm.issues if i.code == "complex_flattened"][0].message

    def test_complex_without_embedded_items_is_an_error_naming_the_type(self):
        cx = {"type": "ComplexItem", "complexItemType": "fwLandingPattern", "landingPoint": [1, 2, 3]}
        lm = parse_plan_obj(plan([simple(84), cx]))
        errs = [i for i in lm.errors if i.code == "complex_unresolved"]
        assert len(errs) == 1 and "fwLandingPattern" in errs[0].message
        assert len(lm.items) == 2          # 沒有默默塞東西進去

    def test_complex_with_empty_items_is_also_unresolved(self):
        cx = {"type": "ComplexItem", "complexItemType": "survey",
              "TransectStyleComplexItem": {"Items": []}}
        assert "complex_unresolved" in codes(parse_plan_obj(plan([simple(84), cx])).errors)

    def test_do_jump_dojumpid_is_remapped_to_seq(self):
        # doJumpId 從 10 起算：QGC 的 DO_JUMP 目標是 doJumpId，不是 seq
        lm = parse_plan_obj(plan([
            simple(84, doJumpId=10),
            simple(16, doJumpId=11, params=[0, 0, 0, 0, 23.951, 120.9, 50]),
            simple(16, doJumpId=12, params=[0, 0, 0, 0, 23.952, 120.9, 50]),
            simple(177, doJumpId=13, frame=2, params=[11, 3, 0, 0, 0, 0, 0]),
            simple(85, doJumpId=14, params=[0, 0, 0, 0, 23.953, 120.9, 0])]))
        j = [x for x in lm.items if x["command"] == 177][0]
        assert j["param1"] == 2.0 and j["param2"] == 3.0
        assert not lm.has_errors, lm.summary_text()

    def test_unmapped_jump_target_warns(self):
        lm = parse_plan_obj(plan([simple(84, doJumpId=1),
                                  simple(177, doJumpId=2, frame=2, params=[99, 1, 0, 0, 0, 0, 0])]))
        assert "plan_jump_unmapped" in codes(lm.warnings)

    def test_missing_home_warns_and_synthesises(self):
        lm = parse_plan_obj(plan([simple(84)], home=None))
        assert "home_missing" in codes(lm.warnings) and lm.items[0]["command"] == 16
        assert "home_missing" not in codes(parse_plan_obj(plan([simple(84)])).issues)

    def test_bad_params_length_and_missing_fields(self):
        lm = parse_plan_obj(plan([simple(84, params=[0, 0, 0])]))
        assert "plan_params" in codes(lm.errors)
        bad = {"type": "SimpleItem", "params": [0] * 7}
        assert "plan_item" in codes(parse_plan_obj(plan([bad])).errors)

    def test_non_plan_objects_raise(self):
        for obj in ([], {"mission": 3}, {"fileType": "Plan"}, {"mission": {"items": "x"}}):
            with pytest.raises(MissionFileError):
                parse_plan_obj(obj)

    def test_filetype_warning(self):
        o = plan([simple(84)])
        o["fileType"] = "Other"
        assert "plan_filetype" in codes(parse_plan_obj(o).warnings)
        assert "plan_filetype" not in codes(parse_plan_obj(plan([simple(84)])).issues)


# ═══════════════════════════ validate_items ════════════════════════════
class TestValidateQuietOnGood:
    def test_good_mission_has_no_issues(self):
        assert validate_items(good()) == []


class TestValidateErrors:
    def test_empty(self):
        assert "empty" in codes(validate_items([]), "error")
        assert "empty" in codes(validate_items([home()]), "error")
        assert "empty" not in codes(validate_items(good()))
        assert "empty" not in codes(validate_items(mission((16, {}))))   # 近似：home+1 項

    def test_too_many_boundary(self):
        def m(n):
            return [home()] + [it(k, 16, 23.95 + k * 1e-6) for k in range(1, n)]
        assert "too_many" in codes(validate_items(m(wf.MAX_ITEMS + 1)), "error")
        assert "too_many" not in codes(validate_items(m(wf.MAX_ITEMS)))

    @pytest.mark.parametrize("key", ["lat", "lon", "alt", "param1", "param2", "param3", "param4"])
    @pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
    def test_not_finite_each_field(self, key, bad):
        m = good()
        m[2][key] = bad
        assert "not_finite" in codes(validate_items(m), "error")

    def test_not_finite_near_miss(self):
        m = good()
        m[2]["param4"] = 1e300
        assert "not_finite" not in codes(validate_items(m))

    def test_lat_lon_range_both_sides(self):
        m = good(); m[2]["lat"] = 90.0001
        assert "lat_range" in codes(validate_items(m), "error")
        m[2]["lat"] = -90.0001
        assert "lat_range" in codes(validate_items(m), "error")
        m[2]["lat"] = 90.0; m[2]["lon"] = 180.0
        assert not codes(validate_items(m)) & {"lat_range", "lon_range"}
        m[2]["lon"] = 180.0001
        assert "lon_range" in codes(validate_items(m), "error")
        m[2]["lon"] = -180.0001
        assert "lon_range" in codes(validate_items(m), "error")
        m[2]["lon"] = -180.0
        assert "lon_range" not in codes(validate_items(m))

    def test_lat_and_lon_are_checked_independently(self):
        m = good(); m[2]["lat"] = 120.9   # 把經度貼進緯度
        c = codes(validate_items(m), "error")
        assert "lat_range" in c and "lon_range" not in c

    def test_command_and_frame_ranges(self):
        for bad, code, key in ((70000, "cmd_range", "command"), (-1, "cmd_range", "command"),
                               (256, "frame_range", "frame"), (-1, "frame_range", "frame")):
            m = good(); m[2][key] = bad
            assert code in codes(validate_items(m), "error"), (key, bad)
        m = good(); m[2]["command"] = 65535; m[2]["frame"] = 255
        assert not codes(validate_items(m)) & {"cmd_range", "frame_range"}
        m[2]["command"] = 0; m[2]["frame"] = 0
        assert not codes(validate_items(m)) & {"cmd_range", "frame_range"}

    def test_bad_item(self):
        m = good(); del m[2]["lat"]
        assert "bad_item" in codes(validate_items(m), "error")
        m = good(); m[2]["alt"] = "100"
        assert "bad_item" in codes(validate_items(m), "error")
        m = good(); m[2]["command"] = 16.5
        assert "bad_item" in codes(validate_items(m), "error")

    def test_zero_position(self):
        m = good(); m[2]["lat"] = 0.0; m[2]["lon"] = 0.0
        assert "zero_position" in codes(validate_items(m), "error")
        # 近似：只有一個是 0
        for la, lo in ((0.0, 1.0), (1.0, 0.0)):
            m[2]["lat"], m[2]["lon"] = la, lo
            assert "zero_position" not in codes(validate_items(m))

    @pytest.mark.parametrize("cmd", [16, 17, 18, 19, 31, 82])
    def test_zero_position_applies_to_location_nav_commands(self, cmd):
        m = mission((cmd, dict(lat=0.0, lon=0.0)))
        assert "zero_position" in codes(validate_items(m), "error")

    @pytest.mark.parametrize("cmd", [21, 85, 22, 84, 20, 93, 178, 177])
    def test_zero_position_exemptions(self, cmd):
        m = mission((16, {}), (cmd, dict(lat=0.0, lon=0.0, p=(1, 2, 0, 0))), (16, dict(lat=23.96)))
        assert "zero_position" not in codes(validate_items(m))

    @pytest.mark.parametrize("cmd", [22, 84])
    def test_zero_position_takeoff_at_origin_is_not_an_error(self, cmd):
        """NAV_TAKEOFF(22)/VTOL_TAKEOFF(84) 在 lat=lon=0 跟 LAND 一樣是官方
        「就地起飛」寫法，不是漏填座標——ArduCopter mode_auto.cpp::
        takeoff_start() 水平位置一律用 current_loc，ArduPlane quadplane.cpp::
        do_vtol_takeoff() 更直接把 loc.lat/lng 強制清零（「we always use the
        current location in XY for takeoff」），飛控端根本不採用 mission item
        的座標。修法之前這條 ERROR 會讓 Load WP File／Write WPs 把一份完全
        合法、依 Mission Planner/QGC 慣例產生的起飛航點檔整份拒收
        （has_errors=True），使用者連預覽都看不到。"""
        m = mission((cmd, dict(lat=0.0, lon=0.0, alt=30.0)), (16, dict(lat=23.951)),
                    (85, dict(lat=23.952, alt=0.0)))
        result = validate_items(m)
        assert "zero_position" not in codes(result, "error"), result
        assert "zero_position" not in codes(result), result

    def test_standard_vtol_takeoff_wpl_file_loads_without_errors(self):
        """端對端：一份依 Mission Planner/QGC 慣例撰寫、VTOL_TAKEOFF(84) 在
        (0,0) 的標準 .waypoints 檔，透過 parse_wpl_text（Load WP File 走的
        同一條解析路徑）不該有 has_errors=True——修法之前這種檔案會被
        ui.main_window.on_sitl_load_wp_file 整份拒收，使用者連預覽都看不到。"""
        lines = [
            "QGC WPL 110",
            wpl_line(0, cmd=16, frame=0, lat="23.95", lon="120.90", alt="0", cur=1),
            wpl_line(1, cmd=84, frame=3, lat="0.00000000", lon="0.00000000", alt="30"),
            wpl_line(2, cmd=16, frame=3, lat="23.951", lon="120.901", alt="30"),
            wpl_line(3, cmd=85, frame=3, lat="0.00000000", lon="0.00000000", alt="0"),
        ]
        lm = parse_wpl_text("\n".join(lines) + "\n")
        assert lm.has_errors is False, lm.summary_text()
        assert "zero_position" not in codes(lm.issues)

    def test_zero_position_only_for_global_frames(self):
        for frame in (0, 3, 10):
            m = mission((16, dict(lat=0.0, lon=0.0, frame=frame)))
            assert "zero_position" in codes(validate_items(m), "error"), frame
        m = mission((16, dict(lat=0.0, lon=0.0, frame=2)))
        assert "zero_position" not in codes(validate_items(m))

    def test_home_at_zero_is_a_warning_not_a_zero_position_error(self):
        m = good(); m[0]["lat"] = 0.0; m[0]["lon"] = 0.0
        c = validate_items(m)
        assert "home_at_zero" in codes(c, "warning") and "zero_position" not in codes(c)
        assert "home_at_zero" not in codes(validate_items(good()))

    def test_home_at_zero_needs_both_lat_and_lon_zero(self):
        """⚠️ finding（minor，test_bite）：程式碼是 `lat == 0 and lon == 0`——
        只有 lat 是 0（真實存在的赤道經緯度）不該被當成『Home 在 (0,0)』。
        之前只測過兩者都是 0／都不是 0，測不出來是 AND 還是只看 lat
        （把 AND 弱化成只看 lat 不會讓既有測試變紅）。"""
        m = good(); m[0]["lat"] = 0.0; m[0]["lon"] = 121.0
        assert "home_at_zero" not in codes(validate_items(m)), \
            '只有 lat=0、lon 是真實經度，不是「位於 (0,0)」'
        m = good(); m[0]["lat"] = 23.95; m[0]["lon"] = 0.0
        assert "home_at_zero" not in codes(validate_items(m)), \
            '只有 lon=0、lat 是真實緯度，不是「位於 (0,0)」'

    def _jump(self, target, repeat, extra_before=()):
        return mission((84, dict(alt=30)), (16, dict(lat=23.951)), *extra_before,
                       (177, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(target, repeat, 0, 0))),
                       (16, dict(lat=23.952)), (85, dict(lat=23.953, alt=0.0)))

    def test_jump_target_bounds(self):
        n = len(self._jump(2, 3))
        for bad in (0, n, n + 5, -1, 2.5):
            assert "jump_target" in codes(validate_items(self._jump(bad, 3)), "error"), bad
        for ok in (1, 2, n - 1):
            assert "jump_target" not in codes(validate_items(self._jump(ok, 3))), ok

    def test_jump_self(self):
        m = self._jump(3, 3)                # 跳的位置就是 seq 3
        assert m[3]["command"] == 177
        assert "jump_self" in codes(validate_items(m), "error")
        assert "jump_self" not in codes(validate_items(self._jump(2, 3)))

    def test_jump_repeat(self):
        for bad in (-2, 1.5, -100):
            assert "jump_repeat" in codes(validate_items(self._jump(2, bad)), "error"), bad
        for ok in (-1, 0, 1, 5):
            assert "jump_repeat" not in codes(validate_items(self._jump(2, ok))), ok

    def test_jump_no_nav(self):
        # seq2 是 DO_CHANGE_SPEED、seq3 是 DO_JUMP → 目標 2 起沒有 NAV
        m = mission((16, {}), (178, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(0, 20, -1, 0))),
                    (177, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(2, -1, 0, 0))))
        assert "jump_no_nav" in codes(validate_items(m), "error")
        # 近似 1：有限次數 → 會結束
        m[3]["param2"] = 5.0
        assert "jump_no_nav" not in codes(validate_items(m))
        # 近似 2：無限，但目標涵蓋一個 NAV
        m[3]["param2"] = -1.0; m[3]["param1"] = 1.0
        assert "jump_no_nav" not in codes(validate_items(m))

    def test_jump_no_nav_only_for_backward_jumps(self):
        # 往前跳：一定繼續往下走，即使目標後全是 DO 指令也不是迴圈
        do = dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(0, 20, -1, 0))
        m = mission((16, {}), (177, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(3, -1, 0, 0))),
                    (178, do), (178, do))
        assert "jump_no_nav" not in codes(validate_items(m))
        # 同樣的 DO 區間，改成往回跳 ⇒ 響
        m = mission((16, {}), (178, do), (178, do),
                    (177, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(2, -1, 0, 0))))
        assert "jump_no_nav" in codes(validate_items(m), "error")

    def test_jump_no_nav_catches_a_chained_forward_then_backward_loop(self):
        """⚠️⚠️ 兩個無限 DO_JUMP 互相銜接：一個往前跳、落在另一個往回跳上，
        繞成一圈，中間僅存的 NAV 因此永遠走不到 —— 「只看往回跳自己的區間」
        擋不住這種鏈狀迴圈（見 `_jump_leads_into_a_nav_free_loop`）。"""
        m = mission(
            (22, dict(alt=30)),                                            # seq1 TAKEOFF
            (177, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(4, -1, 0, 0))),  # seq2 往前跳→seq4，無限
            (16, dict(lat=23.96)),                                          # seq3 唯一僅存的 NAV
            (177, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(2, -1, 0, 0))),  # seq4 往回跳→seq2，無限
        )
        codes_ = codes(validate_items(m), "error")
        assert "jump_no_nav" in codes_
        # 兩個跳都參與同一個迴圈，各自都要響——只擋其中一個，另一個仍會讓
        # 飛控從那個入口進來時卡住。
        errs = [i for i in validate_items(m) if i.code == "jump_no_nav"]
        assert {e.seq for e in errs} == {2, 4}

    def test_jump_leads_into_a_nav_free_loop_helper_falls_off_the_end_safely(self):
        """輔助函式對「往前跳到任務尾端、後面沒有其他跳」要判定安全，
        不能把『掉出任務範圍』誤判成迴圈。"""
        items = mission((16, {}), (177, dict(frame=2, lat=0.0, lon=0.0, alt=0.0,
                                             p=(3, -1, 0, 0))), (178, {}), (178, {}))
        ok = [True] * len(items)
        assert wf._jump_leads_into_a_nav_free_loop(3, items, ok, len(items)) is False


class TestValidateWarnings:
    def test_no_takeoff(self):
        m = mission((16, {}), (16, dict(lat=23.96)), (85, dict(lat=23.97, alt=0.0)))
        assert "no_takeoff" in codes(validate_items(m), "warning")
        for tk in (22, 84):
            m[1]["command"] = tk
            assert "no_takeoff" not in codes(validate_items(m)), tk

    def test_no_takeoff_ignores_leading_do_and_nav_delay(self):
        m = mission((178, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(0, 20, -1, 0))),
                    (93, dict(lat=0.0, lon=0.0, alt=0.0, p=(60, -1, -1, -1))),
                    (84, dict(alt=30)), (16, dict(lat=23.96)), (85, dict(lat=23.97, alt=0.0)))
        assert "no_takeoff" not in codes(validate_items(m))
        # 但 NAV_DELAY 後面的第一個動作不是起飛 ⇒ 仍然警告
        m[3]["command"] = 16
        assert "no_takeoff" in codes(validate_items(m), "warning")

    def test_no_nav_command(self):
        m = mission((178, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(0, 20, -1, 0))))
        assert "no_nav_command" in codes(validate_items(m), "warning")
        assert "no_nav_command" not in codes(validate_items(good()))

    def test_after_land(self):
        m = good() + [it(5, 16, 23.96)]
        c = validate_items(m)
        assert "after_land" in codes(c, "warning")
        assert [i.seq for i in c if i.code == "after_land"] == [5]
        assert "after_land" not in codes(validate_items(good()))
        # 降落在中間、後面是 DO_JUMP 之類的也算「之後還有項目」
        m2 = good(); m2[3], m2[4] = m2[4], m2[3]
        assert "after_land" in codes(validate_items(m2))

    def test_last_not_terminal(self):
        m = good()[:-1]
        assert "last_not_terminal" in codes(validate_items(m), "warning")
        for cmd in (20, 21, 85, 17, 18, 19, 31):
            m2 = good()[:-1] + [it(4, cmd, 23.953, alt=10.0)]
            assert "last_not_terminal" not in codes(validate_items(m2)), cmd

    def test_rel_alt_nonpositive(self):
        for alt in (0.0, -5.0):
            m = good(); m[2]["alt"] = alt
            assert "rel_alt_nonpositive" in codes(validate_items(m), "warning"), alt
        m = good(); m[2]["alt"] = 0.0; m[2]["frame"] = 0     # 絕對高度 0 = 海平面，不是相對地面
        assert "rel_alt_nonpositive" not in codes(validate_items(m))
        m = good(); m[2]["alt"] = 0.5
        assert "rel_alt_nonpositive" not in codes(validate_items(m))

    def test_rel_alt_low(self):
        m = mission((84, dict(alt=30)), (17, dict(lat=23.96, alt=0.5)), (85, dict(lat=23.97, alt=0.0)))
        assert "rel_alt_low" in codes(validate_items(m), "warning")
        m[2]["alt"] = 1.0
        assert "rel_alt_low" not in codes(validate_items(m))
        # LAND 的 0 m 是正常的
        assert "rel_alt_low" not in codes(validate_items(good()))
        # 就地起飛（0,0）不重複警告
        m[2]["alt"] = 20.0
        m2 = mission((22, dict(lat=0.0, lon=0.0, alt=0.5, frame=2)), (16, dict(lat=23.96)),
                     (85, dict(lat=23.97, alt=0.0)))
        assert "rel_alt_low" not in codes(validate_items(m2))

    def test_rel_alt_low_fires_for_here_ok_zero_position_takeoff_too(self):
        """⚠️ zero_position 的 (0,0) 例外（TAKEOFF/VTOL_TAKEOFF「就地起飛」）
        跟 rel_alt_low 是兩件不相干的事：座標合法不代表高度就沒打錯。
        之前的程式碼誤把 zero_position 的排除條件整段套到 rel_alt_low，
        導致 (0,0) 的 TAKEOFF 即使相對高度只有 0.2 m 也完全不警告。"""
        for cmd in (22, 84):        # TAKEOFF / VTOL_TAKEOFF
            m = mission((cmd, dict(lat=0.0, lon=0.0, alt=0.2, frame=3)),
                        (16, dict(lat=23.96)), (85, dict(lat=23.97, alt=0.0)))
            c = validate_items(m)
            assert "rel_alt_low" in codes(c, "warning"), cmd
            assert "zero_position" not in codes(c), cmd  # 這裡本來就該被 _HERE_OK 排除
        # 對照組：非零座標的同一種項目一直都會響，確保這條測試真的在鎖
        # 「(0,0) 本身」這個變因，不是別的東西。
        m = mission((22, dict(lat=23.9, lon=120.9, alt=0.2, frame=3)),
                    (16, dict(lat=23.96)), (85, dict(lat=23.97, alt=0.0)))
        assert "rel_alt_low" in codes(validate_items(m), "warning")

    def test_far_apart(self):
        m = mission((84, dict(alt=30)), (16, dict(lat=23.95 + 0.6)), (85, dict(lat=23.95 + 0.6, alt=0.0)))
        c = validate_items(m)
        assert "far_apart" in codes(c, "warning")
        assert [i.seq for i in c if i.code == "far_apart"] == [2]
        m = mission((84, dict(alt=30)), (16, dict(lat=23.95 + 0.3)), (85, dict(lat=23.95 + 0.3, alt=0.0)))
        assert "far_apart" not in codes(validate_items(m))

    def test_far_apart_skips_non_location_items_between(self):
        m = mission((16, dict(lat=23.95)),
                    (178, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(0, 20, -1, 0))),
                    (16, dict(lat=24.6)), (85, dict(lat=24.6, alt=0.0)))
        assert "far_apart" in codes(validate_items(m))

    def test_do_latlon(self):
        m = mission((84, dict(alt=30)), (178, dict(frame=2, lat=23.9, lon=120.9, alt=0.0, p=(0, 20, -1, 0))),
                    (85, dict(lat=23.97, alt=0.0)))
        assert "do_latlon" in codes(validate_items(m), "warning")
        m[2]["lat"] = 0.0; m[2]["lon"] = 0.0
        assert "do_latlon" not in codes(validate_items(m))
        for cmd in (179, 189, 192, 195, 201):        # 本來就用座標
            m[2]["command"] = cmd; m[2]["lat"] = 23.9; m[2]["lon"] = 120.9
            assert "do_latlon" not in codes(validate_items(m)), cmd

    def test_do_latlon_fires_on_either_field_alone(self):
        """⚠️ finding（minor，test_bite）：程式碼是 `lat != 0 or lon != 0`——
        只設 lat、lon 仍是 0（或反過來）也該響。之前只測過兩個都非零／都是零
        的案例，測不出來是 OR 還是 AND（把 OR 改成 AND 不會讓既有測試變紅）。"""
        base = mission((84, dict(alt=30)),
                       (178, dict(frame=2, lat=0.0, lon=0.0, alt=0.0, p=(0, 20, -1, 0))),
                       (85, dict(lat=23.97, alt=0.0)))
        m = [dict(x) for x in base]
        m[2]["lat"] = 23.9; m[2]["lon"] = 0.0
        assert "do_latlon" in codes(validate_items(m), "warning"), '只有 lat 非零也該響'
        m = [dict(x) for x in base]
        m[2]["lat"] = 0.0; m[2]["lon"] = 120.9
        assert "do_latlon" in codes(validate_items(m), "warning"), '只有 lon 非零也該響'

    def test_seq_mismatch(self):
        m = good(); m[2]["seq"] = 9
        c = validate_items(m)
        assert "seq_mismatch" in codes(c, "warning")
        assert "seq_mismatch" not in codes(validate_items(good()))

    def test_unknown_command(self):
        pytest.importorskip("pymavlink")
        m = good(); m[2]["command"] = 9999
        assert "unknown_command" in codes(validate_items(m), "warning")
        m[2]["command"] = 177
        assert "unknown_command" not in codes(validate_items(m))

    def test_errors_and_warnings_are_separate_levels(self):
        m = good(); m[2]["lat"] = 200.0; m[3]["alt"] = 0.0
        c = validate_items(m)
        assert all(i.level in ("error", "warning") for i in c)
        assert "lat_range" in codes(c, "error") and "rel_alt_nonpositive" in codes(c, "warning")

    def test_issue_is_frozen_and_has_traditional_chinese_text(self):
        i = validate_items([])[0]
        assert isinstance(i, MissionIssue)
        with pytest.raises(Exception):
            i.code = "x"
        assert any("一" <= ch <= "鿿" for ch in i.message)


# ═══════════════════════════ 輸出與往返 ═════════════════════════════════
def rich_items():
    return [
        it(0, 16, 23.95, 120.9, 9.0, frame=0, current=1),
        it(1, 84, 23.9501, 120.9001, 60.0, p=(0, 0, 0, 0)),
        it(2, 178, 0.0, 0.0, 0.0, frame=2, p=(0.0, 21.5, -1.0, 0.0)),
        it(3, 16, 23.9502, 120.902, -12.5, p=(0.0, 30.0, 0.0, 0.0)),
        it(4, 16, 4e-07, -4e-07, 4e-07, frame=0, p=(4e-07, -4e-07, 1e-05, 123456.789012)),
        it(5, 177, 0.0, 0.0, 0.0, frame=2, p=(3.0, 2.0, 0.0, 0.0)),
        it(6, 85, 23.9503, 120.903, 0.0, ac=0),
    ]


class TestWriteText:
    def test_format(self):
        txt = items_to_wpl_text(rich_items())
        assert txt.startswith("QGC WPL 110\n") and txt.endswith("\n") and "\r" not in txt
        rows = txt.rstrip("\n").split("\n")
        assert len(rows) == 8
        for k, r in enumerate(rows[1:]):
            cols = r.split("\t")
            assert len(cols) == 12 and cols[0] == str(k)
        c = rows[2].split("\t")
        assert (c[3], c[9].count(".")) == ("84", 1)
        assert c[8] == "23.95010000" and c[9] == "120.90010000"

    def test_column_order(self):
        row = items_to_wpl_text([it(0, 16, 1.5, 2.5, 3.5, frame=3, p=(11, 12, 13, 14), current=1, ac=0)]
                                ).split("\n")[1].split("\t")
        assert row[1] == "1" and row[2] == "3" and row[3] == "16"
        assert [float(x) for x in row[4:8]] == [11, 12, 13, 14]
        assert (float(row[8]), float(row[9]), float(row[10])) == (1.5, 2.5, 3.5)
        assert row[11] == "0"

    def test_precision_is_not_squashed(self):
        txt = items_to_wpl_text(rich_items())
        row = txt.split("\n")[5].split("\t")
        assert float(row[4]) == 4e-07 and float(row[10]) == 4e-07 and float(row[6]) == 1e-05

    def test_seq_is_renumbered_by_position(self):
        items = rich_items()
        for x in items:
            x["seq"] += 40
        rows = items_to_wpl_text(items).rstrip("\n").split("\n")[1:]
        assert [r.split("\t")[0] for r in rows] == [str(k) for k in range(7)]

    def test_non_finite_and_missing_are_refused(self):
        m = rich_items(); m[2]["alt"] = math.nan
        with pytest.raises(MissionFileError):
            items_to_wpl_text(m)
        m = rich_items(); m[2]["param3"] = math.inf
        with pytest.raises(MissionFileError):
            items_to_wpl_text(m)
        m = rich_items(); del m[2]["frame"]
        with pytest.raises(MissionFileError):
            items_to_wpl_text(m)


class TestRoundTrip:
    def test_rich_mission_round_trips_exactly(self):
        items = rich_items()
        back = parse_wpl_text(items_to_wpl_text(items)).items
        assert back == items

    def test_round_trip_keeps_current_and_autocontinue(self):
        items = rich_items(); items[3]["current"] = 1; items[3]["autocontinue"] = 0
        assert parse_wpl_text(items_to_wpl_text(items)).items == items

    def test_seeded_random_round_trip(self):
        rnd = random.Random(20260921)
        cmds = [16, 17, 18, 19, 20, 21, 22, 82, 84, 85, 93, 177, 178, 179, 183, 189, 206, 31, 30]
        for trial in range(60):
            n = rnd.randint(1, 30)
            items = []
            for k in range(n):
                magnitude = rnd.choice([1e-7, 1e-3, 1.0, 100.0, 1e5])
                items.append({
                    "seq": k, "command": rnd.choice(cmds), "frame": rnd.choice([0, 2, 3, 10]),
                    "lat": round(rnd.uniform(-90, 90), 7), "lon": round(rnd.uniform(-180, 180), 7),
                    "alt": rnd.uniform(-500, 500) * magnitude,
                    "param1": rnd.uniform(-1, 1) * magnitude, "param2": float(rnd.randint(-2, 40)),
                    "param3": rnd.choice([0.0, 4e-07, -1.0, rnd.random()]),
                    "param4": rnd.uniform(0, 360),
                    "current": rnd.choice([0, 0, 1]), "autocontinue": rnd.choice([0, 1]),
                })
            back = parse_wpl_text(items_to_wpl_text(items), "t").items
            assert back == items, trial

    def test_generated_text_reparses_to_same_text(self):
        t1 = items_to_wpl_text(rich_items())
        t2 = items_to_wpl_text(parse_wpl_text(t1).items)
        assert t1 == t2


class TestSave:
    def test_save_returns_path_and_content(self, tmp_path):
        p = tmp_path / "out.waypoints"
        r = save_wpl(rich_items(), p)
        assert os.fspath(r) == str(p)
        assert p.read_bytes() == items_to_wpl_text(rich_items()).encode("utf-8")
        assert load_mission_file(p).items == rich_items()

    def test_no_temp_files_left_and_replaces_existing(self, tmp_path):
        p = tmp_path / "out.waypoints"
        p.write_text("OLD", encoding="utf-8")
        save_wpl(rich_items(), p)
        assert [x.name for x in tmp_path.iterdir()] == ["out.waypoints"]
        assert p.read_text(encoding="utf-8").startswith("QGC WPL 110")

    def test_failed_save_leaves_old_file_untouched(self, tmp_path):
        p = tmp_path / "out.waypoints"
        p.write_text("OLD", encoding="utf-8")
        bad = rich_items(); bad[1]["lat"] = math.nan
        with pytest.raises(MissionFileError):
            save_wpl(bad, p)
        assert p.read_text(encoding="utf-8") == "OLD"
        assert [x.name for x in tmp_path.iterdir()] == ["out.waypoints"]

    def test_write_into_missing_directory_is_loud(self, tmp_path):
        with pytest.raises(MissionFileError):
            save_wpl(rich_items(), tmp_path / "no_such_dir" / "x.waypoints")

    def test_replace_is_atomic_step(self, tmp_path, monkeypatch):
        """寫檔失敗（os.replace 出錯）時舊檔要在、暫存檔要清掉。"""
        p = tmp_path / "out.waypoints"
        p.write_text("OLD", encoding="utf-8")

        def boom(a, b):
            raise OSError("模擬 replace 失敗")
        monkeypatch.setattr(wf.os, "replace", boom)
        with pytest.raises(MissionFileError):
            save_wpl(rich_items(), p)
        assert p.read_text(encoding="utf-8") == "OLD"
        assert [x.name for x in tmp_path.iterdir()] == ["out.waypoints"]


# ═══════════════════════════ 真實語料 ═══════════════════════════════════
def _corpus():
    fs = [f for f in glob.glob(str(REPO / "**" / "*.waypoints"), recursive=True)
          if f"{os.sep}.claude{os.sep}" not in f and f"{os.sep}.git{os.sep}" not in f]
    return sorted(fs)


# AeroPlan 自己匯出的檔留下的警告及其原因（每一種都被檢查過，不是照單全收）：
#   last_not_terminal   data/exports/_diamond*：蜂群打擊航線，終點是目標點上的 NAV_WAYPOINT，
#                       沒有 RTL/降落 —— 打擊任務的設計。
#   rel_alt_nonpositive 同上：終點目標高度 0 m。
ALLOWED_CORPUS_WARNINGS = {"last_not_terminal", "rel_alt_nonpositive"}


class TestCorpus:
    def test_every_exported_file_loads_with_zero_errors(self):
        fs = _corpus()
        if len(fs) < 40:
            pytest.skip(f"真實語料不足（{len(fs)} < 40）")
        dirs = {os.path.dirname(f) for f in fs}
        assert len(dirs) >= 10
        for f in fs:
            lm = load_mission_file(f)
            assert lm.fmt == "wpl" and len(lm.items) >= 2, f
            assert not lm.has_errors, f"{f}\n{lm.summary_text()}"
            unexplained = codes(lm.warnings) - ALLOWED_CORPUS_WARNINGS
            assert not unexplained, f"{f}: {unexplained}"

    def test_corpus_round_trips_through_our_writer(self):
        fs = _corpus()
        if len(fs) < 40:
            pytest.skip("真實語料不足")
        for f in fs[::5]:
            items = load_mission_file(f).items
            assert parse_wpl_text(items_to_wpl_text(items)).items == items, f

    def test_plan_and_waypoints_twins_agree(self):
        pairs = [(f, f[:-len(".waypoints")] + ".plan") for f in _corpus()
                 if os.path.exists(f[:-len(".waypoints")] + ".plan")]
        if len(pairs) < 5:
            pytest.skip("沒有成對的 .plan/.waypoints 語料")
        for w, p in pairs:
            a, b = load_mission_file(w).items, load_mission_file(p)
            assert not b.has_errors, b.summary_text()
            assert len(a) == len(b.items), w
            for x, y in zip(a, b.items):
                assert (x["command"], x["frame"]) == (y["command"], y["frame"]), (w, x["seq"])
                assert abs(x["lat"] - y["lat"]) < 1e-6 and abs(x["lon"] - y["lon"]) < 1e-6
                assert abs(x["alt"] - y["alt"]) < 1e-3

    def test_a_corrupted_corpus_file_is_caught(self, tmp_path):
        fs = _corpus()
        if not fs:
            pytest.skip("無語料")
        lines = pathlib.Path(fs[0]).read_text(encoding="utf-8").splitlines()
        k = next(i for i, l in enumerate(lines) if l and not l.startswith(("#", "QGC")))
        lines[k] = "\t".join(lines[k].split("\t")[:-1])
        p = tmp_path / "c.waypoints"
        p.write_text("\n".join(lines) + "\n", encoding="utf-8")
        with pytest.raises(MissionFileError) as ei:
            load_mission_file(p)
        assert ei.value.line == k + 1


# ═══════════════════════════ 結構鎖（AST）═══════════════════════════════
class TestModuleStructure:
    SRC = (REPO / "mission" / "waypoint_file.py").read_text(encoding="utf-8")

    def test_no_qt_no_hv_dccpp_no_closed_enum_imports(self):
        tree = ast.parse(self.SRC)
        banned = ("PyQt", "PySide", "hv_dccpp", "mission.waypoint", "mission.sitl_link")
        for n in ast.walk(tree):
            names = []
            if isinstance(n, ast.Import):
                names = [a.name for a in n.names]
            elif isinstance(n, ast.ImportFrom):
                names = [n.module or ""]
                names += [f"{n.module}.{a.name}" for a in n.names]
            for nm in names:
                assert not nm.startswith(banned), nm

    def test_pymavlink_only_imported_lazily(self):
        tree = ast.parse(self.SRC)
        for n in tree.body:
            if isinstance(n, (ast.Import, ast.ImportFrom)):
                mod = n.module if isinstance(n, ast.ImportFrom) else n.names[0].name
                assert not (mod or "").startswith("pymavlink")

    def test_no_percent_style_precision_for_params(self):
        assert "%.6f" not in self.SRC
