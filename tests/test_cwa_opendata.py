"""utils.cwa_opendata：CWA 授權碼的來源優先序、格式檢查（不退回下一個來源）、settings.json 讀取。不連網。"""
from __future__ import annotations

from types import SimpleNamespace

import pytest

from utils import cwa_opendata as co


def test_env_wins_over_settings_and_blank_values_are_skipped():
    assert co.resolve_cwa_api_key("CWA-ENV-1", "CWA-SET-1") == ("CWA-ENV-1", "環境變數 CWA_API_KEY")
    assert co.resolve_cwa_api_key("  ", "CWA-SET-1") == ("CWA-SET-1", "settings.json weather.cwa_api_key")
    assert co.resolve_cwa_api_key(None, " CWA-SET-1\n") == ("CWA-SET-1", "settings.json weather.cwa_api_key")
    assert co.resolve_cwa_api_key(None, "") == ("", "")


@pytest.mark.parametrize("bad", ["CWA 123", "CWA-1\"2", "key=abc", "CWA-1&x=2"])
def test_invalid_key_is_rejected_without_falling_back(bad):
    assert co.resolve_cwa_api_key(bad, "CWA-SET-1") == ("", "invalid:環境變數 CWA_API_KEY")
    assert co.resolve_cwa_api_key(None, bad) == ("", "invalid:settings.json weather.cwa_api_key")


@pytest.mark.parametrize("bad", [123, 1.5, True, ["a"], {"k": 1}])
def test_non_string_key_is_invalid_not_a_crash_and_does_not_fall_back(bad):
    key, src = co.resolve_cwa_api_key(None, bad)
    assert key == "" and src.startswith("invalid:settings.json weather.cwa_api_key（不是字串：")
    key, src = co.resolve_cwa_api_key(bad, "CWA-SET-1")
    assert key == "" and src.startswith("invalid:環境變數 CWA_API_KEY（不是字串：")


@pytest.mark.parametrize("s, ok", [
    ("CWA-12345678-1234-1234-1234-123456789ABC", True), ("cwb-12345678-1234-1234-1234-123456789abc", True),
    ("A" * 30, True), ("A" * 29, False), ("466920", False), ("C0C480", False), ("桃園", False), ("", False),
    (None, False), ("CWA-SHORT", False), ("x" * 40 + " ", True),
])
def test_looks_like_cwa_api_key(s, ok):
    assert co.looks_like_cwa_api_key(s) is ok


def test_configured_reports_settings_problems_instead_of_saying_no_key(monkeypatch):
    import config.settings as cs
    monkeypatch.delenv("CWA_API_KEY", raising=False)
    warn = "settings.json 的 weather 區塊有未知鍵 ['cwa_apikey']（已忽略；已知鍵 ['cwa_api_key', 'cwa_station']）"
    monkeypatch.setattr(cs, "get_settings", lambda: SimpleNamespace(
        weather=SimpleNamespace(cwa_api_key="", cwa_station=""), weather_load_warning=warn))
    assert co.configured_cwa_api_key() == ("", "invalid:" + warn)
    monkeypatch.setattr(cs, "get_settings", lambda: SimpleNamespace(
        weather=SimpleNamespace(cwa_api_key="CWA-OK", cwa_station=""), weather_load_warning=warn))
    assert co.configured_cwa_api_key() == ("CWA-OK", "settings.json weather.cwa_api_key"), "有鑰匙就照常用"
    monkeypatch.setattr(cs, "get_settings", lambda: SimpleNamespace(
        weather=SimpleNamespace(cwa_api_key=123, cwa_station=""), weather_load_warning=""))
    assert co.configured_cwa_api_key()[1].startswith("invalid:settings.json weather.cwa_api_key（不是字串：int")

    def boom():
        raise RuntimeError("disk")

    monkeypatch.setattr(cs, "get_settings", boom)
    assert co.configured_cwa_api_key() == ("", "invalid:settings 讀取失敗：RuntimeError")
    monkeypatch.setenv("CWA_API_KEY", "CWA-ENV")
    assert co.configured_cwa_api_key() == ("CWA-ENV", "環境變數 CWA_API_KEY"), "設定壞了但環境變數有 ⇒ 用環境變數"


def test_weather_block_with_a_typo_keeps_the_known_keys_and_warns(tmp_path, capsys):
    from config.settings import GlobalSettings
    f = tmp_path / "settings.json"
    f.write_text('{"weather": {"cwa_apikey": "CWA-TYPO", "cwa_station": "466920"}}', encoding="utf-8")
    s = GlobalSettings(config_file=str(f))
    assert s.weather.cwa_station == "466920" and s.weather.cwa_api_key == ""
    assert "cwa_apikey" in s.weather_load_warning and "cwa_api_key" in s.weather_load_warning
    assert "CWA-TYPO" not in s.weather_load_warning and "CWA-TYPO" not in capsys.readouterr().out, "只記鍵名不記值"
    f.write_text('{"weather": "oops"}', encoding="utf-8")
    s = GlobalSettings(config_file=str(f))
    assert s.weather.cwa_api_key == "" and "不是物件" in s.weather_load_warning
    f.write_text('{"weather": {"cwa_api_key": "CWA-X"}}', encoding="utf-8")
    assert GlobalSettings(config_file=str(f)).weather_load_warning == ""


def test_configured_reads_env_then_settings(monkeypatch):
    import config.settings as cs
    monkeypatch.delenv("CWA_API_KEY", raising=False)
    monkeypatch.setattr(cs, "get_settings", lambda: SimpleNamespace(weather=SimpleNamespace(cwa_api_key="CWA-FROM-FILE",
                                                                                              cwa_station="  466920 ")))
    assert co.configured_cwa_api_key() == ("CWA-FROM-FILE", "settings.json weather.cwa_api_key")
    assert co.configured_cwa_station() == "466920"
    monkeypatch.setenv("CWA_API_KEY", "CWA-FROM-ENV")
    assert co.configured_cwa_api_key() == ("CWA-FROM-ENV", "環境變數 CWA_API_KEY")
    monkeypatch.setattr(cs, "get_settings", lambda: SimpleNamespace())          # 舊 settings 物件沒有 weather 區
    monkeypatch.delenv("CWA_API_KEY", raising=False)
    assert co.configured_cwa_api_key() == ("", "") and co.configured_cwa_station() == ""


def test_weather_settings_round_trip_through_json(tmp_path):
    from config.settings import GlobalSettings, WeatherSettings
    assert WeatherSettings().cwa_api_key == "" and WeatherSettings().cwa_station == ""
    f = tmp_path / "settings.json"
    f.write_text('{"weather": {"cwa_api_key": "CWA-X", "cwa_station": "桃園"}}', encoding="utf-8")
    s = GlobalSettings(config_file=str(f))
    assert (s.weather.cwa_api_key, s.weather.cwa_station) == ("CWA-X", "桃園")
    s2 = GlobalSettings(config_file=str(tmp_path / "none.json"))
    assert s2.weather.cwa_api_key == ""
