@echo off

::1�����ƹ���hex�ļ���ָ���ļ��У� �������ļ���ַ
::�޸�ָ���ļ��л��ļ��Լ��޸�ObjPathԴ�ļ�λ�ã�HexDestPathĿ���ļ����ļ���λ��
set ObjPath=E:\keil_workspace\NORDIC\nRF52832_htwh_sdk15.0\examples\dfu\secure_bootloader\pca10040_ble\arm5_no_packs\_build\bootloader.hex

set HexDestPath=E:\keil_workspace\NORDIC\nRF52832_htwh_sdk15.0\examples\ble_peripheral\ble_app_smc_freertos-debug\wh_Script_SDK15_S132_nRF52832_GNT

::����ָ��·��ָ���ļ����ļ��У���HexDestPath·���ļ���
echo y | xcopy "%ObjPath%" /e /r /k "%HexDestPath%"
::xcopy /e/c/h/z "%~pd0*.*" "%out%"

::pause 




