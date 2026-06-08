# 打包成可執行檔

AeroPlan Studio 使用 **PyInstaller** 打包成 Windows one-folder 可執行檔。

## 一鍵打包

```bash
build.bat
```

或手動：

```bash
python -m PyInstaller aeroplan_studio.spec --noconfirm
```

完成後執行檔位於：

```
dist\AeroPlanStudio\AeroPlanStudio.exe
```

整個 `dist\AeroPlanStudio\` 資料夾都要一起發佈（PyQt WebEngine 需要旁邊的資源檔）。

## 為什麼是 one-folder 而不是 one-file？

| 模式 | one-folder | one-file |
|------|-----------|----------|
| 啟動速度 | 立即 | 首次解壓需 10-30 秒 |
| WebEngine | 正常運作 | 子程序常找不到資源 |
| 體積 | ~600 MB | ~600 MB |
| 發佈 | 整個資料夾 | 單一 .exe |

WebEngine（Chromium）會啟動子程序載入 Cesium，one-file 模式下子程序需要重新解壓資源，常導致 3D 地圖空白或崩潰。

## 打包包含的資源

- **Cesium 3D 引擎**：`assets/cesium/Build/Cesium/` (~50 MB)
- **Leaflet 2D 地圖**：`assets/leaflet/`
- **HTML 模板**：`ui/widgets/cesium_templates/main.html`
- **GLB 模型**：`ui/resources/P25-ID.glb`、`copter.glb`
- **MIL-STD-1472H 主題**：`ui/resources/styles/Global_MIL_STD.qss` 等
- **ArduPilot SITL**：`sitl/ArduPlane.exe`、`ArduCopter.exe` + cygwin DLLs
- **預設參數**：`sitl/default_params/`
- **DEM 地形**：`huwei_dem.tif`
- **設定檔**：`config/vehicle_profiles.yaml`

## 體積估算

| 項目 | 大小 |
|------|------|
| PyQt6 + WebEngine | ~250 MB |
| Cesium | ~50 MB |
| ArduPilot SITL + cygwin | ~25 MB |
| numpy + scipy + matplotlib | ~100 MB |
| numba + LLVM | ~80 MB |
| 其他 | ~50 MB |
| **總計** | **~550–650 MB** |

## 縮小體積（可選）

修改 `aeroplan_studio.spec`：

1. 排除不用的科學計算：移除 `numba` 與 `matplotlib` 後重新打包
2. 移除 SITL：若不需要本機模擬，從 `_add_dir('sitl', ...)` 移除
3. 移除 docs：spec 預設已不打包

## 除錯模式

如果打包後執行檔閃退：

1. 把 spec 中 `console=False` 改成 `console=True` 重新打包
2. 直接從 cmd 跑 `dist\AeroPlanStudio\AeroPlanStudio.exe`，看 traceback
3. 常見問題：
   - `ModuleNotFoundError: pymavlink.dialects.v20.X` → 把 X 加到 spec 的 `hiddenimports`
   - WebEngine 黑畫面 → 確認 `PyQt6/Qt6/resources/` 與 `PyQt6/Qt6/translations/` 有被 PyQt 的 hook 帶進來
   - `qt.webengine: failed to load locale` → 把 `PyQt6/Qt6/translations/qtwebengine_locales/` 補進 datas

## 圖示

把 `.ico` 檔放到 `ui/resources/app.ico`，下次打包會自動套用。

```bash
# 從 PNG 轉 ICO（需要 ImageMagick 或 Pillow）
python -c "from PIL import Image; Image.open('ui/resources/milstd_demo.png').save('ui/resources/app.ico', sizes=[(256,256),(128,128),(64,64),(32,32),(16,16)])"
```

## 發佈

把整個 `dist\AeroPlanStudio\` 壓縮成 zip，使用者解壓後直接執行 `AeroPlanStudio.exe` 即可。建議搭配安裝程式（如 Inno Setup）做為正式版。
