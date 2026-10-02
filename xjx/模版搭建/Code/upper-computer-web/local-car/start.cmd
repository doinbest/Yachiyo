@echo off
chcp 65001 >nul
title Car Console - CMD + Web shared serial
python -u "%~dp0launch.py" %*
if errorlevel 1 pause
