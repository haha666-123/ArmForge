@echo off
rem Run ArmForge from dist\ when it exists, otherwise from build\.
rem   run.bat            -> demo
rem   run.bat fk --q "0 -1.2 1.2 0 0.9 0"
setlocal
cd /d "%~dp0"
if exist "dist\armforge.exe" (
    "dist\armforge.exe" %*
) else (
    "build\bin\armforge.exe" %*
)
endlocal
