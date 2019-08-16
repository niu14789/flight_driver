@echo off
set exe_path=%cd%
cd %exe_path%
cd ../common/tools
stack_allocate_release.exe %exe_path%\..\output\debug.bin %exe_path%\..\output\release.bin %exe_path%\..\apps\driver.ok 1