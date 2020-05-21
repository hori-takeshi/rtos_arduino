@echo off

set MAKELEVEL=
ECHO >serials.list
make -f %2\wait4port\poll.mk init >NUL
mode %1: baud=1200 >NUL

:LOOP
FOR /F "usebackq" %%i IN (`make -f %2\wait4port\poll.mk poll`) DO SET _SER=%%i
IF NOT "%_SER%"=="NONE" GOTO :EXITLOOP
timeout /T 1  /NOBREAK >NUL
IF NOT "%_COUNT%"=="ZZZZZZ" GOTO :LOOP

:EXITLOOP
make -f %2\wait4port\poll.mk clean
echo %_SER%

