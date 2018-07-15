@ECHO OFF

set LIST=lib\mbedcrypto_software\armgcc^
         lib\openthread\mtd\armgcc^
         lib\openthread\ftd\armgcc^
         lib\openthread\ftd\armgcc^
         lib\openthread_cli\mtd\armgcc^
         lib\openthread_cli\ftd\armgcc^
         lib\openthread_diag\armgcc^
         lib\openthread_ncp\ftd\armgcc^
         lib\openthread_ncp\mtd\armgcc^
         lib\openthread_nrf52840\sdk\uart\armgcc^
         lib\openthread_nrf52840\sdk\usb\armgcc^
         lib\openthread_nrf52840\softdevice\uart\armgcc^
         lib\openthread_nrf52840\softdevice\usb\armgcc^
         lib\openthread_platform_utils\armgcc

FOR %%L IN (%LIST%) DO (
    ECHO Building %%L...
    pushd %%L && make -j && popd
)