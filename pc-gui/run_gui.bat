@echo off
rem Launch the photoacoustic detector GUI with the system Python 3.13.
rem The anaconda base interpreter ships MSVC runtime 14.44 DLLs in its root folder,
rem which shadow the newer runtime that PySide6 >= 6.11 needs ("DLL load failed
rem while importing QtCore"). Python 3.13 from python.org uses the system runtime.
cd /d "%~dp0"
py -3.13 -c "import PySide6, pyqtgraph, serial, numpy" 2>nul
if errorlevel 1 (
    echo Installing GUI requirements into Python 3.13...
    py -3.13 -m pip install -r requirements.txt
)
py -3.13 pa_gui.py %*
