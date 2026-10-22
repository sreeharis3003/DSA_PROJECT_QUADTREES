@echo off
REM Windows test runner:  tests\run_tests.bat   (build first with build.bat)
setlocal
cd /d "%~dp0.."
if not exist tests\actual mkdir tests\actual
set fail=0
for %%f in (tests\cases\*.txt) do (
    .\quadmap.exe %%f > tests\actual\%%~nf.out
    fc /W tests\expected\%%~nf.out tests\actual\%%~nf.out >nul && (echo PASS  %%~nf) || (echo FAIL  %%~nf & set fail=1)
)
if %fail%==0 (echo All tests passed) else (echo Some tests FAILED & exit /b 1)
