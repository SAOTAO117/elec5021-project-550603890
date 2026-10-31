@echo off
setlocal
pushd "%~dp0"
set "DSS=%CCS_DSS%"

if defined DSS if exist "%DSS%" goto dss_ready

set "DSS="
for /d %%D in ("C:\ti\ccs*") do if exist "%%~fD\ccs\ccs_base\scripting\bin\dss.bat" set "DSS=%%~fD\ccs\ccs_base\scripting\bin\dss.bat"
for /d %%D in ("C:\ti\ccs*") do if exist "%%~fD\ccs_base\scripting\bin\dss.bat" set "DSS=%%~fD\ccs_base\scripting\bin\dss.bat"

if not defined DSS (
    echo CCS scripting tool dss.bat was not found under C:\ti.
    echo If CCS is installed elsewhere, set the CCS_DSS environment
    echo variable to the full path of dss.bat and run this file again.
    popd
    pause
    exit /b 1
)

:dss_ready
echo Using CCS scripting tool:
echo %DSS%

if not exist "Program\IterativeController.out" (
    echo Program\IterativeController.out was not found.
    popd
    pause
    exit /b 1
)


call "%DSS%" "Program\Program_Iterative_Controller.js"
set "RESULT=%ERRORLEVEL%"
popd

if not "%RESULT%"=="0" (
    echo Board programming failed with error %RESULT%.
    pause
    exit /b %RESULT%
)

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Archive_State.ps1"
if errorlevel 1 (
    echo Programming succeeded, but local state archiving failed.
    echo Keep the DC bus at 0 V. Resolve the state files before an iteration.
    pause
    exit /b 1
)

echo The slow-iteration controller is running on the F28379D.
echo Keep the DC bus at 0 V and run Load_Initial_PWM.m next.
pause
exit /b 0

