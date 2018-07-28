cd C:\Program Files (x86)\Nordic Semiconductor\nrf5x\bin
nrfjprog.exe --reset
nrfjprog.exe --eraseall
nrfjprog.exe --program C:\Users\Austin\Downloads\nRF5_SDK_for_Thread_v0.11.0_84a130f\examples\peripheral\pin_change_int\pca10056\blank\arm4\_build\nrf52840_xxaa.hex
@echo off
nrfjprog.exe --reset
set /p id=""