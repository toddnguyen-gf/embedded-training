@echo off
cd %TEMP%
"C:\Users\i1A771792\AppData\Roaming\xPacks\@xpack-dev-tools\openocd\0.10.0-15.1\.content\bin\openocd.exe" -s C:\share\scripts -f interface/stlink.cfg -f target/stm32f3x.cfg
