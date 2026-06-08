@echo off
REM ============================================================
REM AeroPlan Studio 一鍵打包腳本（Windows）
REM
REM 用法：
REM   build.bat              ── 標準打包
REM   build.bat clean        ── 清理 build/dist 後再打包
REM   build.bat onefile      ── 嘗試單檔模式（不建議；WebEngine 啟動慢）
REM ============================================================
setlocal

cd /d "%~dp0"

REM 檢查 Python
where python >nul 2>nul
if errorlevel 1 (
    echo [錯誤] 找不到 python，請先安裝 Python 3.10+
    pause & exit /b 1
)

REM 檢查 PyInstaller
python -c "import PyInstaller" 2>nul
if errorlevel 1 (
    echo [info] 正在安裝 PyInstaller...
    python -m pip install --user pyinstaller
)

REM 清理（可選）
if /i "%1"=="clean" (
    echo [info] 清理 build/ 與 dist/ ...
    if exist build rmdir /s /q build
    if exist dist rmdir /s /q dist
)

echo.
echo ============================================================
echo  開始打包 AeroPlan Studio
echo ============================================================
echo.

python -m PyInstaller aeroplan_studio.spec --noconfirm
if errorlevel 1 (
    echo.
    echo [錯誤] 打包失敗，請檢視上方訊息
    pause & exit /b 1
)

echo.
echo ============================================================
echo  打包完成！
echo  執行檔：dist\AeroPlanStudio\AeroPlanStudio.exe
echo ============================================================
echo.

REM 順便顯示輸出資料夾大小
for /f "tokens=*" %%a in ('dir /s /-c "dist\AeroPlanStudio" ^| find "個檔案"') do echo  %%a
for /f "tokens=*" %%a in ('dir /s /-c "dist\AeroPlanStudio" ^| find " File(s)"') do echo  %%a

endlocal
