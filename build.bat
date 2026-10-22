@echo off
REM Windows build without make (needs gcc on PATH, e.g. TDM-GCC / MinGW)
cd /d "%~dp0"
gcc -std=c99 -Wall -Wextra -O2 -Isrc/core -Isrc/query -Isrc/dynamic -Isrc/app ^
    src/core/quadtree.c src/query/query.c src/dynamic/dynamic.c ^
    src/app/lidar_io.c src/app/main.c -o quadmap.exe
if %errorlevel% neq 0 (echo BUILD FAILED & exit /b 1)
echo Built quadmap.exe
