@echo off
set exe_path=%cd%
cd %exe_path%
cd ../common/tools
UV4.exe %exe_path%\output\driver.hex ../../output/driver.bin -b