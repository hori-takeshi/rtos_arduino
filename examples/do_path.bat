@echo off
SET ARDUINO_DIR=%HOMEPATH%\AppData\Local\Arduino15\packages\arduino

FOR /D %%V IN (%ARDUINO_DIR%\hardware\sam\*) DO ( SET _TEMP1=%%~fV)
FOR /D %%V IN (%ARDUINO_DIR%\tools\arm-none-eabi-gcc\*) DO ( SET _TEMP2=%%~fV)

SET PATH=%_TEMP2%\bin;%_TEMP1%\system\CMSIS\Examples\cmsis_example\gcc_arm;%PATH%

