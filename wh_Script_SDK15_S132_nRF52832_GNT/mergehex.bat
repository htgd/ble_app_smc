@echo off

::1������ settings page for current image: gnt_app.hex
::Bootloader settings�洢��Flash���һ��page������������λ��оƬ����Ϊ�������ǽ���DFUģʽ����Ӧ��ģʽ��ͬʱ��������image��CRCֵ�Ͱ汾����Ϣ�����Ҫ��оƬ��λ�����application��������ȷ���ɸ�bootloader settings hex

nrfutil settings generate --family NRF52 --application smc_app.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 settings.hex


::2���ϳ�һ��hex����������¼
::merge bootloader and settings
mergehex.exe --merge bootloader.hex settings.hex --output bl_temp.hex
::merge bootloader, app and softdevice
mergehex.exe --merge bl_temp.hex smc_app.hex s132_nrf52_6.0.0_softdevice.hex --output whole.hex

::pause 

::merge bootloader and settings


