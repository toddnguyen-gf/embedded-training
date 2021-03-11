@echo off
set "file_name=target/thumbv7em-none-eabihf/debug/examples/hello"
echo Debugging: '%file_name%'
arm-none-eabi-gdb -q %file_name%
