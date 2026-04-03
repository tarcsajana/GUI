@echo off
REM This script rebuilds the ESP-IDF project
REM Run this from an ESP-IDF terminal (ESP-IDF Build terminal in VS Code)

cd /d "C:\Users\jtarcsa\OneDrive - Tesla\Documents\PlatformIO\Projects\GUI-1"
idf.py reconfigure
idf.py build
