# FlightGear 3D 顯示(2026-08-27)

FlightGear 在這裡**只是顯示器**。物理全部來自 ArduPilot + JSBSim
(`--fdm=external`),所以換機體只改**外觀**,對飛行結果沒有任何影響。

## 用法

CAD 匯出後一行轉檔(格式自動判斷:glb / gltf / obj / stl / fbx / dae / ply):

```
"C:/Program Files/Blender Foundation/Blender 5.0/blender.exe" --background \
  --python sitl/flightgear/tools/cad2fg.py -- <輸入檔> <輸出.obj> 40000
```

然後把 `.obj` / `.mtl` 放進 `Barracuda500/Models/`,整包複製到
`~/.fgfs/Aircraft/`,用 `--fg-aircraft=$HOME/.fgfs/Aircraft --aircraft=Barracuda500`。

## ⚠️ 方向是**量**出來的,不是猜的

FlightGear 的模型座標系:**-X = 機鼻、+Y = 上、±Z = 翼展**。

這個結論不是查文件來的,是拿一台**已知在 FG 裡顯示正確**的 c172p 反推:

| 位置 | Z 展寬 | 對照 |
|---|---|---|
| X ≈ 0 | **11.164 m** | C172 翼展 11.0 m ⇒ 主翼 |
| X ≈ +4.5 | **3.5 m** | C172 尾翼展 3.4 m ⇒ 水平尾翼 |

尾翼在 +X ⇒ **機鼻在 -X**。而且 `c172p.xml` 的 `<offsets>` 只有
`pitch-deg -3` 和 `z-m -0.065`,**沒有任何旋轉** ⇒ 那個 `.ac` 本身就是
FG 期望的方向,不是靠 XML 轉正的。

⚠️ 我一開始用「垂直尾翼會讓機尾比較高」當判準,**錯了** —— C172 的
座艙加起落架比尾翼還高。用尾翼的**展寬**才分得出來。

## ⚠️ `cad2fg.py` 為什麼要自動量方向

第一版把「+X 機鼻、+Y 上」寫死。那是**那一個** GLB 的實測值 ——
換一次 Fusion 匯出、換一次座標系設定,方向就變,而寫死的程式碼
**不會報錯,只會把飛機畫成橫著飛或倒著飛**。

自動版的判準和 2026-08-22 解 GLB 軸向用的同一招(網格剖面,不是推理):
縱軸取包圍盒最長者,機鼻取截面 r95 較小的那一端,並印出兩端比值。
**比值 < 1.3 就拒絕猜測並要求人工指定**(第 4 個參數,例如 `+X`)。

⭐ 對照組:對現有的 `ui/resources/barracuda500.glb` 跑自動版,產出的 OBJ
與手工推導版 **md5 完全相同**(`3d55b15c…`,29773 頂點)。
自動判斷重現了已知正確答案,所以它對新檔的判斷可以採信。

## ⚠️⚠️ CAD 不是氣動模型

見 memory `barracuda500-model-facts`:**CAD 外形與 SITL 實際飛的機體不是
同一個東西**。CAD 展長 2.325 m / 全長 3.786 m;飛的是 m=22 / b=1.88 / S=0.45。
換了 CAD **不會**改變任何飛行結果 —— 它和 c172p 一樣只是個殼。
當插圖沒問題,當「已驗證模型」不行。

## ⚠️ WSL 上 fgfs 起不來 = D-Bus

`dbus-run-session -- fgfs ...` 不可省。詳見 `sitl/jsbsim/README.md`
與 memory `flightgear-wsl-dbus`。

## 產生物不進版控

`.obj` / `.mtl` 由 `cad2fg.py` 從 CAD 重現,**刻意不收進 repo**
(memory `export-must-be-deterministic`)。
