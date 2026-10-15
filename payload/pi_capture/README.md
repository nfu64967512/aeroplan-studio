# pi_capture —— 相機模組擷取服務（Raspberry Pi 5）

每架機一套：Pi 5 + RPi 相機（HQ IMX477 + 6 mm 或 Camera Module 3）+ 原始 IMU（選配）+ ZED-F9P（第二階段，選配）。
飛控用 Relay 相機後端（`CAM1_TYPE=2`）每拍一張就打一個 AUX GPIO 脈衝；Pi 被動接收，**不送任何 MAVLink**。
背景與設計見 `docs/評估_2026-10-14_群飛航測高精地圖.md` §3、§4、§7、§8。

這個資料夾**不在 AeroPlan App 的 import 路徑裡**，只部署到 Pi 上；Windows 上的測試是 `tests/test_pi_capture.py`（全部假後端）。

## 做什麼

1. 相機以全解析度 still 設定連續出幀（手動曝光、固定增益、鎖白平衡與焦點、`buffer_count ≥ 3`）。
2. 每個 GPIO 上升緣（libgpiod v2 核心時間戳，CLOCK_MONOTONIC）挑**第一張 SensorTimestamp（換到 MONOTONIC）≥ 邊緣時刻**的影格，只把那一張編成 JPEG 存檔，其他影格立刻放回。
3. 每個邊緣一列 `frames.jsonl`（立即 fsync）：等不到影格 `no_frame`、寫入佇列滿 `dropped`、編碼或寫檔失敗 `write_error`。
4. 每存一個檔就追加一列 `manifest.sha256`（`sha256sum -c` 格式）；`clocks.jsonl` 開始、結束與每 5 s 記一次三個時鐘。
5. 正常停止（SIGTERM）才寫 `summary.json`。**飛完通常是直接拔電池**，所以任何讀取端都不得依賴它。

會期格式 `aeroplan-pi-session/1`（F1）的欄位定義在 `session.py` 開頭；`manifest.sha256` 是 F1 之外的附加檔。

時鐘注意：`edge_ns` 是 MONOTONIC；`sensor_ts_ns` 照原樣寫 libcamera 的 BOOTTIME。兩者差＝系統休眠時間，`clocks.jsonl` 讓地面端驗證它是常數。SensorTimestamp 到底是「第一列開始曝光」還是「第一個像素讀出」，libcamera 與 picamera2 文件互相矛盾，要台架 B3 定案。

## 接線

| 訊號 | Pi 5 | 另一端 | 備註 |
|---|---|---|---|
| 觸發 | GPIO17（範例，`trigger.line`） | 飛控 AUX（`RELAY1_PIN`，`SERVOn_FUNCTION=-1`） | 3.3 V；串 1 kΩ；有地迴路就改光耦。與飛控共地 |
| 就緒（選配） | GPIO27（範例，`ready.line`） | 飛控 AUX 輸入（日後 Lua aux-auth） | 健康時高電位。飛控端加下拉電阻 |
| IMU（選配） | SPI0 CE0 | ICM-42688-P | 剛性固定在相機板上 |
| F9P（第二階段） | USB（建議）或 UART | ZED-F9P | 同一條觸發線經緩衝接 **EXTINT**（以 F9P 的 VCC 3.3 V 為準） |
| 電源 | 5 V/5 A 獨立 BEC | 動力電池 | 不與飛控共用 BEC；Pi 5 滿載約 9 W |

Pi 5 排針的 gpiochip 編號隨核心版本不同（舊核心 `gpiochip4`、新核心 `gpiochip0`），所以 `trigger.chip` 建議填晶片標籤 `pinctrl-rp1`（用 `gpioinfo` 確認，標籤字串 UNVERIFIED）。

## 安裝（Raspberry Pi OS Bookworm 64-bit）

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-pil python3-serial python3-spidev python3-venv
# picamera2 走 apt，所以 venv 要看得到系統套件
sudo python3 -m venv --system-site-packages /opt/aeroplan/venv
# Bookworm 的 apt 套件 python3-libgpiod 是 v1 API，本程式用 v2 API ⇒ 改裝 PyPI 的 gpiod（自帶 libgpiod）
sudo /opt/aeroplan/venv/bin/pip install "gpiod>=2.1"
sudo mkdir -p /opt/aeroplan /etc/pi_capture
sudo cp -r pi_capture /opt/aeroplan/
sudo cp /opt/aeroplan/pi_capture/config.example.json /etc/pi_capture/config.json
```

`/boot/firmware/config.txt`：

```
camera_auto_detect=0
dtoverlay=imx477          # HQ；CM3 用 imx708
dtparam=spi=on            # 有 IMU 才需要
```

**dtoverlay 與核心版本會改變滾動快門讀出時間**：rpi-6.12.y 的 imx477 驅動依 DT 的 `link-frequency` 計算行長（預設 450 MHz 時讀出約 84.0 ms，6.6.y 推算 86.9 ms，兩者都未實測）。所以：

- `config.json` 的 `camera.dtoverlay` 要**逐字照抄** config.txt 那一行的參數（含 `link-frequency=…`），它會原樣寫進每個會期的 session.json；
- 核心版本（`uname -r`）由程式自動寫進 session.json；
- `camera.readout_ms` 只能填台架 B2 在**同一核心、同一 DT 設定**下的實測值，沒量過就留 `null`；換核心（`apt upgrade`）或改 DT 後要重量。

資料碟：NVMe 掛在 `/mnt/nvme`（ext4、`noatime`），systemd 單元以 `RequiresMountsFor` 確保沒掛上就不啟動。

## 設定檔 `/etc/pi_capture/config.json`

嚴格解析：**不認得的鍵直接拒絕**（打錯字不會被靜默忽略），缺必填鍵也拒絕；啟動時把有效設定完整寫進日誌。

- `uav`、`unit_model`：`unit_model` 每台相機唯一（例如 `RPiHQ6-HERO2`），寫進 EXIF Model，讓 ODM/OpenSfM 分開校正各台內參。
- `lever_arm_m`：**必填實測值**，相機中心 − 飛控 GNSS 天線，機身 FRD（公尺）。範例檔故意填 `null`，不量就無法啟動。
- `gnss_lever_arm_m`：相機中心 − 相機模組 F9P 天線（第二階段）；沒有 F9P 填 `null`。
- `capture.exposure_us`／`analogue_gain`／`colour_gains`：手動曝光（建議 500–1000 µs）、固定增益、鎖死的白平衡增益（必填）。`lens_position` 只給 CM3（屈光度，0＝無限遠），HQ 用機械鎖焦。
- `capture.buffer_count` ≥ 3，且 ≥ `max_inflight_writes + 2`（否則相機緩衝用光會停住，程式會拒絕啟動）。
- `health.max_temp_c`、`health.min_free_gb`：就緒腳的健康條件。
- `imu`：`{"type": "icm42688p", "spi_bus": 0, "spi_dev": 0, "rate_hz": 400}` 或 `null`。
- `gnss`：`{"device": "/dev/ttyACM0", "baudrate": 460800, "fsync_period_s": 2}` 或 `null`。

## 啟用服務

```bash
sudo cp /opt/aeroplan/pi_capture/pi-capture.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pi-capture
journalctl -u pi-capture -f
```

每次服務啟動＝一個新會期目錄 `<storage_root>/<uav>_<UTC 時間>/`，絕不覆寫。服務異常重啟會開新會期，地面端以飛控的開機次數把同一架次的會期合併。
開機自檢（`ExecStartPre`）的測試影像寫在 `<storage_root>/selfcheck/`，不混進飛行會期（否則張數守恆會多一張）；它會一直累積，定期手動清。

## 起飛前檢查表

1. 飛控參數讀回：`CAM1_TYPE=2`、`RELAY1_FUNCTION=4`、`RELAY1_PIN`＝實際腳、對應 `SERVOn_FUNCTION=-1`、`CAM1_DURATION=0.1`、`CAM_AUTO_ONLY=1`、`LOG_BITMASK` 含 bit 12。**RELAY 設錯時飛控照樣寫 CAM，但 GPIO 不會動**，只能靠 Pi 端的邊緣數證明。
2. 自檢結果：`cat /run/pi_capture/selfcheck.json`，`ok` 必須為 `true`。
3. 全鏈路試拍（要測接線時）：
   ```bash
   sudo systemctl stop pi-capture        # 相機同時只能被一個行程打開
   /opt/aeroplan/venv/bin/python -m pi_capture selfcheck --config /etc/pi_capture/config.json \
       --planned-photos 2000 --bytes-per-photo 9000000 --external-edge-s 30
   # 30 s 內由地面站送 DO_DIGICAM_CONTROL（param5=1）
   sudo systemctl start pi-capture
   ```
   注意 `LOG_DISARMED=0` 時，未解鎖的試拍不會寫進飛控 CAM 紀錄。
4. 剩餘空間 ≥ 1.5 × 計畫張數 × 台架實測每張大小（自檢的 `free_space` 項）。

## 落地與卸載

1. **先取飛控 SD 卡的 .BIN，再讓飛控斷電或重開**（開機會刪最舊日誌）。
2. **拔電池前先按關機**：`ssh pi@<相機> sudo poweroff`，或接一顆關機按鈕（`/boot/firmware/config.txt` 加 `dtoverlay=gpio-shutdown,gpio_pin=3`），等綠燈熄滅再拔。
   忘了也不會壞：每列紀錄都已 fsync，只是沒有 `summary.json`，最後一張影像可能不完整而被標成缺檔。
3. 卸載（乙太網或拆 NVMe）：
   ```bash
   rsync -a pi@<相機>:/mnt/nvme/sessions/<會期>/ ./<會期>/
   cd <會期> && sha256sum -c manifest.sha256
   python -m pi_capture verify ./<會期>      # 結束碼 0＝完整
   ```
4. Pi 上**不自動刪檔**；地面複本驗證通過、操作員確認後才手動清除。

## ZED-F9P（第二階段 PPK）

一次性設定（寫進 RAM＋BBR＋Flash）：

```bash
# 建議 USB：
python -m pi_capture f9p-config --device /dev/ttyACM0 --port USB
# 或 UART1（預設 38400 塞不下 10 Hz RAWX，程式會拒絕不夠快的鮑率）：
python -m pi_capture f9p-config --device /dev/ttyAMA0 --link-baud 38400 --port UART1 --baud 460800
# 只看位元組不開序列埠：
python -m pi_capture f9p-config --device x --port USB --dry-run /tmp/f9p.ubx
```

內容：`CFG-RATE-MEAS=100 ms`、`CFG-RATE-NAV=1`、`CFG-RATE-TIMEREF=GPS`、`CFG-TP-TIMEGRID_TP1=GPS`（TIM-TM2 的時間基準）、該埠只出 UBX（關 NMEA）、RXM-RAWX／RXM-SFRBX／TIM-TM2／NAV-PVT 每個 epoch 輸出。每則都要收到 ACK-ACK，NAK 或逾時就報錯。key ID 與頁碼見 `f9p_config.py`（對照 UBX-22008968 R01）。

- TIM-TM2 **不需要另外開 EXTINT**：訊息啟用＋EXTINT 有邊緣就會在下一個 epoch 輸出；每個 epoch 只回報最後一個邊緣，所以量測週期必須短於拍照間隔（`--min-photo-interval-s` 會檢查）。
- 地面端必須檢查每筆 TIM-TM2 的 `timeBase`（flags bits 4…3 == 1＝GNSS 時間）；`TIMEGRID_TP1` 對 TIM-TM2 的實際作用未在硬體上驗證。
- UART2 要 HPG 1.30 以上才輸出 UBX。
- 擷取時 `config.json` 的 `gnss` 填上裝置與鮑率，原始位元組寫進 `gnss/rover.ubx`（每 `fsync_period_s` 秒 fsync，停流 5 s 報錯）。

## 未驗證（台架與硬體必做）

- 全部實機後端（picamera2、libgpiod、vcgencmd、spidev、pyserial）在 Windows 上只寫了程式、沒有跑過。
- 12 MP JPEG 持續 ≥1.8 Hz、60 分鐘不掉張（台架 B1）；Pillow 單執行緒編碼速度未知。
- `capture_request()` 在 `stop()` 後是否一定返回；`request.release()` 跨執行緒安全性。
- 讀出時間（B2）、SensorTimestamp 語意（B3）、邊緣時間戳精度與 MONOTONIC/BOOTTIME 偏移恆定（B4）。
- 就緒腳在行程被 `kill -9` 後的電位；飛控端應加下拉，日後考慮改心跳。
- ICM-42688-P 資料暫存器位址與換算只部分核對（見 `imu.py`）；BMI088 未實作。
- Pi 5 的 `pinctrl-rp1` 標籤、`/sys/class/thermal/thermal_zone0/temp`、`vcgencmd get_throttled` 輸出格式。
