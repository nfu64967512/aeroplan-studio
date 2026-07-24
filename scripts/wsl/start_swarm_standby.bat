@echo off
REM start_swarm_standby.bat — 常駐啟動 6 台 SITL + 長僚機編隊節點（待命，不起飛）
REM 以隱藏視窗的 wsl 程序常駐（formation 腳本結尾 wait 保住 SITL 父程序），
REM 關掉這個 .bat 視窗或終端機都不影響。
REM 起飛（在 WSL 內執行）:  bash ~/fixwing_ws/sitl_run/swarm_takeoff.sh
REM 停止（在 WSL 內執行）:  bash ~/fixwing_ws/sitl_run/swarm_formation_down.sh
REM 可改地點/編隊: 編輯下行 LOC=TAOYUAN / FORMATION=v|biomimetic / PATROL=0|1
powershell -NoProfile -Command "Start-Process wsl -ArgumentList '-e','bash','-lc','cd ~/fixwing_ws/sitl_run && LOC=TAOYUAN MANUAL=1 PATROL=1 bash swarm_formation_up.sh > /tmp/formation_up.log 2>&1' -WindowStyle Hidden"
echo [standby] 已於背景啟動 6xSITL + MAVProxy + MAVROS + 編隊節點（待命、不起飛）
echo [standby] 進度: wsl -e bash -lc "tail -f /tmp/formation_up.log"
echo [standby] 起飛: wsl -e bash -lc "bash ~/fixwing_ws/sitl_run/swarm_takeoff.sh"
pause
