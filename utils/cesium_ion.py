"""Cesium ion access token 的解析（UI 與規劃核心共用，不依賴 Qt）。

token 是祕密：只放被 .gitignore 的 ``config/settings.json`` 的 ``map.cesium_ion_token``，
環境變數 ``CESIUM_ION_TOKEN`` 優先。**不要寫進原始碼**。log 只記來源與長度，絕不記 token。

token 會被 ``.replace('__CESIUM_TOKEN__', …)`` 塞進 JS 字串字面量，所以**只接受 JWT 的字元集**
（base64url + 「.」）：含引號、空白、換行的值會讓整份 3D 頁的 script 語法錯誤、3D 地圖靜默變黑。
"""
from __future__ import annotations

import os
import re
from typing import Optional, Tuple

_ION_TOKEN_OK = re.compile(r'[A-Za-z0-9._-]+')


def resolve_cesium_ion_token(class_token: str = '', env_token: Optional[str] = None,
                             settings_token: str = '') -> Tuple[str, str]:
    """回傳 ``(token, 來源)``。優先序：類別屬性 > 環境變數 ``CESIUM_ION_TOKEN`` > settings.json 的
    ``map.cesium_ion_token``。都沒有 ⇒ ``('', '')``。

    格式不合 ⇒ ``('', 'invalid:<來源>')``：**不**退回下一個來源（靜默換一個值生效比沒有更糟），
    由呼叫端記 log。前後空白會被去掉（貼上時常帶換行）。
    """
    for source, raw in (('程式碼 CESIUM_TOKEN', class_token),
                        ('環境變數 CESIUM_ION_TOKEN', env_token),
                        ('settings.json map.cesium_ion_token', settings_token)):
        tok = (raw or '').strip()
        if not tok:
            continue
        if _ION_TOKEN_OK.fullmatch(tok):
            return tok, source
        return '', f'invalid:{source}'
    return '', ''


def configured_ion_token(class_token: str = '') -> Tuple[str, str]:
    """目前生效的 token：``(token, 來源)``（讀環境變數與 settings.json；讀不到設定就當沒有）。"""
    try:
        from config.settings import get_settings
        from_settings = getattr(get_settings().map, 'cesium_ion_token', '') or ''
    except Exception:
        from_settings = ''
    return resolve_cesium_ion_token(class_token, os.environ.get('CESIUM_ION_TOKEN'), from_settings)
