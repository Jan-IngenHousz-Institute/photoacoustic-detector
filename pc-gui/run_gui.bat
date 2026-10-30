@echo off
rem Start the photoacoustic detector GUI on Windows.
rem
rem Creates a private Python environment in pc-gui\.venv the first time (a few
rem minutes: downloads PySide6), using a python.org Python 3.10+ found through
rem the "py" launcher. If no such Python exists it installs Python 3.13 with
rem winget. Anaconda/conda Pythons are skipped on purpose: their MSVC runtime
rem is too old for PySide6 ("DLL load failed while importing QtCore").
setlocal
cd /d "%~dp0"
set "VENV=.venv"
set "PY="

for %%V in (3.13 3.12 3.11 3.10) do (
    if not defined PY (
        py -%%V -c "import sys; sys.exit(1 if 'conda' in sys.base_prefix.lower() else 0)" >nul 2>&1 && set "PY=py -%%V"
    )
)

if not defined PY (
    echo No python.org Python 3.10+ found. Installing Python 3.13 with winget ^(per user^) ...
    winget install --id Python.Python.3.13 --exact --accept-package-agreements --accept-source-agreements --silent
    if errorlevel 1 (
        echo.
        echo Automatic install failed. Install Python 3.13 from https://www.python.org/downloads/
        echo ^(keep "py launcher" ticked^) and run this file again.
        pause
        exit /b 1
    )
    set "PY=%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
    if not exist "%LOCALAPPDATA%\Programs\Python\Python313\python.exe" set "PY=py -3.13"
)

if not exist "%VENV%\Scripts\python.exe" (
    echo Creating the GUI's Python environment in %CD%\%VENV% ...
    %PY% -m venv "%VENV%" || goto :fail
)

"%VENV%\Scripts\python.exe" -c "import PySide6, pyqtgraph, serial, numpy" >nul 2>&1
if errorlevel 1 (
    echo Installing the GUI requirements ^(one time^) ...
    "%VENV%\Scripts\python.exe" -m pip install --upgrade pip --quiet
    "%VENV%\Scripts\python.exe" -m pip install -r requirements.txt || goto :fail
)

"%VENV%\Scripts\python.exe" pa_gui.py %*
if errorlevel 1 goto :fail
exit /b 0

:fail
echo.
echo The GUI could not be started. See the messages above and the Troubleshooting
echo section of README.md. Deleting the %VENV% folder makes this script start over.
pause
exit /b 1
