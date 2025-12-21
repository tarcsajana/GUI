@echo off
REM This script rebuilds the ESP-IDF project
REM Run this from an ESP-IDF terminal (ESP-IDF Build terminal in VS Code)

cd /d D:\Code\GUI
idf.py reconfigure
idf.py build
