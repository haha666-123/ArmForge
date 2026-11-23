@echo off
rem Thin wrapper so the deploy step works from cmd.exe.
rem   deploy.bat            (Release)
rem   deploy.bat -Config Debug
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0deploy.ps1" %*
if errorlevel 1 exit /b 1
