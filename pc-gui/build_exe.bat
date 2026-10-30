@echo off
rem Build the standalone Windows GUI (no Python needed on the target PC) with
rem PyInstaller, into pc-gui\dist\PhotoacousticDetector\ and a zip of it.
rem Run from a python.org Python 3.13 (not Anaconda), e.g. after run_gui.bat
rem created pc-gui\.venv.
setlocal
cd /d "%~dp0"
set "PYTHON=.venv\Scripts\python.exe"
if not exist "%PYTHON%" set "PYTHON=py -3.13"
%PYTHON% -c "import PyInstaller" >nul 2>&1 || %PYTHON% -m pip install pyinstaller || exit /b 1
%PYTHON% -m PyInstaller --noconfirm --clean --windowed --name PhotoacousticDetector ^
    --paths . --collect-submodules pyqtgraph pa_gui.py || exit /b 1
copy /y ..\firmware\release\pa_detector-*.uf2 dist\PhotoacousticDetector\ >nul
copy /y ..\README.md dist\PhotoacousticDetector\README.md >nul
%PYTHON% -c "import shutil; shutil.make_archive('dist/PhotoacousticDetector-windows', 'zip', 'dist', 'PhotoacousticDetector')"
echo Built dist\PhotoacousticDetector\PhotoacousticDetector.exe and dist\PhotoacousticDetector-windows.zip
