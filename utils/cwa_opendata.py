"""中央氣象署開放資料平臺 API 授權碼的解析（UI 與核心共用，不依賴 Qt）。

授權碼是祕密：只放被 .gitignore 的 ``config/settings.json`` 的 ``weather.cwa_api_key``，
環境變數 ``CWA_API_KEY`` 優先。**不要寫進原始碼**；log 只記來源與長度，絕不記授權碼，
也不要把帶 ``Authorization=`` 的 URL 寫進 log 或錯誤訊息（core.wind_field.redact_cwa_url）。

授權碼會放進 URL 的查詢字串，所以只接受 ``[A-Za-z0-9-]``（平臺發的是 ``CWA-`` 加 UUID）；
含空白、引號、換行的值（貼上時常帶到）直接判不合法，**不**退回下一個來源。
"""
from __future__ import annotations

import os
import re
from typing import Optional, Tuple

_KEY_OK = re.compile(r'[A-Za-z0-9-]+')
#: 平臺發的授權碼：CWA-（舊 CWB-）加 UUID；或任何 ≥ 30 字的合法字元串也當成授權碼看待（拒絕回顯）
_KEY_SHAPE = re.compile(r'CW[AB]-[0-9A-Fa-f]{8}(-[0-9A-Fa-f]{4}){3}-[0-9A-Fa-f]{12}')


def looks_like_cwa_api_key(s: object) -> bool:
    """使用者把授權碼貼錯到「測站」欄時用：這種字串不能回顯到 log／狀態列。"""
    if not isinstance(s, str):
        return False
    s = s.strip()
    return bool(_KEY_SHAPE.fullmatch(s) or (len(s) >= 30 and _KEY_OK.fullmatch(s)))


def resolve_cwa_api_key(env_key: Optional[str] = None, settings_key: object = '') -> Tuple[str, str]:
    """回傳 ``(授權碼, 來源)``。優先序：環境變數 ``CWA_API_KEY`` > settings.json 的 ``weather.cwa_api_key``。
    都沒有 ⇒ ``('', '')``；格式不合或不是字串 ⇒ ``('', 'invalid:<來源>…')``（不退回下一個來源）。"""
    for source, raw in (('環境變數 CWA_API_KEY', env_key),
                        ('settings.json weather.cwa_api_key', settings_key)):
        if raw is None:
            continue
        if not isinstance(raw, str):
            return '', f'invalid:{source}（不是字串：{type(raw).__name__}）'
        key = raw.strip()
        if not key:
            continue
        if _KEY_OK.fullmatch(key):
            return key, source
        return '', f'invalid:{source}'
    return '', ''


def configured_cwa_api_key() -> Tuple[str, str]:
    """目前生效的授權碼：``(授權碼, 來源)``（讀環境變數與 settings.json）。

    settings.json 的 weather 區塊讀取有問題（未知鍵、不是物件、設定本身讀不到）而且最後沒有授權碼時，
    回 ``('', 'invalid:<原因>')`` 讓面板說出真正原因，而不是「沒有授權碼」。這裡不會丟例外（按鈕 slot 裡呼叫）。
    """
    from_settings: object = ''
    warning = ''
    try:
        from config.settings import get_settings
        s = get_settings()
        from_settings = getattr(getattr(s, 'weather', None), 'cwa_api_key', '')
        warning = str(getattr(s, 'weather_load_warning', '') or '')
    except Exception as e:                                   # noqa: BLE001 - 設定讀不到也要能回報
        warning = f'settings 讀取失敗：{type(e).__name__}'
    try:
        key, src = resolve_cwa_api_key(os.environ.get('CWA_API_KEY'), from_settings)
    except Exception as e:                                   # noqa: BLE001
        return '', f'invalid:授權碼解析失敗：{type(e).__name__}'
    if not key and not src and warning:
        return '', f'invalid:{warning}'
    return key, src


def configured_cwa_station() -> str:
    """settings.json 的 ``weather.cwa_station``（預設測站；空白＝最近測站）。"""
    try:
        from config.settings import get_settings
        return str(getattr(getattr(get_settings(), 'weather', None), 'cwa_station', '') or '').strip()
    except Exception:
        return ''
