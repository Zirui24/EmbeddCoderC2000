cd .

if "%1"=="" ("E:\APPLIC~1\MATLAB~1\MATLAB~2\bin\win64\gmake"  -B -f Fuel_Pump_V3_1_20201113_20KHz.mk all) else ("E:\APPLIC~1\MATLAB~1\MATLAB~2\bin\win64\gmake"  -B -f Fuel_Pump_V3_1_20201113_20KHz.mk %1)
@if errorlevel 1 goto error_exit

exit 0

:error_exit
echo The make command returned an error of %errorlevel%
exit 1
