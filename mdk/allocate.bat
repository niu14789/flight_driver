@echo off
cd /d %~dp0
cd ..
cd common/tools/
stack_allocate.exe  0x08010000  0x20000000 ../../output/irom.h  -h ../../apps/
pause