@echo off

::1）生成 settings page for current image: gnt_app.hex
::Bootloader settings存储在Flash最后一个page，它将决定复位后芯片的行为，比如是进入DFU模式还是应用模式，同时它还包含image的CRC值和版本等信息。如果要求芯片复位后进入application，必须正确生成该bootloader settings hex

nrfutil settings generate --family NRF52 --application smc_app.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 settings.hex


::2）合成一个hex用于生产烧录
::merge bootloader and settings
mergehex.exe --merge bootloader.hex settings.hex --output bl_temp.hex
::merge bootloader, app and softdevice
mergehex.exe --merge bl_temp.hex smc_app.hex s132_nrf52_6.0.0_softdevice.hex --output whole.hex

::pause 

::merge bootloader and settings


