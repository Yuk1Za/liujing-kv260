@echo off
chcp 65001 >nul
set PYTHONUTF8=1
cd /d "%~dp0"
python "liujing\tools\start_2m_demo.py" %*
pause
