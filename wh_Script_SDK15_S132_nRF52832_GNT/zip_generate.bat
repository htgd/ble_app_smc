@echo off

::1�����ƹ���hex�ļ���ָ���ļ��У� �������ļ���ַ
::�޸�ָ���ļ��л��ļ��Լ��޸�ObjPathԴ�ļ�λ�ã�HexDestPathĿ���ļ����ļ���λ��
set ObjPath=E:\keil_workspace\NORDIC\nRF52832_htwh_sdk15.0\examples\ble_peripheral\ble_app_smc_freertos-debug\pca10040\s132\arm5_no_packs\_build\smc_app.hex

set HexDestPath=E:\keil_workspace\NORDIC\nRF52832_htwh_sdk15.0\examples\ble_peripheral\ble_app_smc_freertos-debug\wh_Script_SDK15_S132_nRF52832_GNT

::����ָ��·��ָ���ļ����ļ��У���HexDestPath·���ļ���
echo y | xcopy "%ObjPath%" /e /r /k "%HexDestPath%"
::xcopy /e/c/h/z "%~pd0*.*" "%out%"


::2������������zip�ļ�
::nrfutil.exe pkg generate --hw-version 52 --application-version 1 --application smc_app.hex --sd-req 0xA8 --key-file private.key smc_app_Dfu15.zip

nrfutil pkg generate --hw-version 52 --application-version 1 --application smc_app.hex --sd-req 0xA8 --key-file private.key smc_app_Dfu15.zip


::pause 