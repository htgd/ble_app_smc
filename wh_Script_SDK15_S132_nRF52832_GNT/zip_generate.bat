@echo off

::1）复制工程hex文件到指定文件夹， 需设置文件地址
::修改指定文件夹或文件自己修改ObjPath源文件位置，HexDestPath目标文件或文件夹位置
set ObjPath=E:\keil_workspace\NORDIC\nRF52832_htwh_sdk15.0\examples\ble_peripheral\ble_app_smc_freertos-debug\pca10040\s132\arm5_no_packs\_build\smc_app.hex

set HexDestPath=E:\keil_workspace\NORDIC\nRF52832_htwh_sdk15.0\examples\ble_peripheral\ble_app_smc_freertos-debug\wh_Script_SDK15_S132_nRF52832_GNT

::复制指定路径指定文件或文件夹，至HexDestPath路径文件夹
echo y | xcopy "%ObjPath%" /e /r /k "%HexDestPath%"
::xcopy /e/c/h/z "%~pd0*.*" "%out%"


::2）生成升级用zip文件
::nrfutil.exe pkg generate --hw-version 52 --application-version 1 --application smc_app.hex --sd-req 0xA8 --key-file private.key smc_app_Dfu15.zip

nrfutil pkg generate --hw-version 52 --application-version 1 --application smc_app.hex --sd-req 0xA8 --key-file private.key smc_app_Dfu15.zip


::pause 