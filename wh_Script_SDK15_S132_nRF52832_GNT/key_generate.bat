@echo off

pause
echo "执行之前需要确认文件路径，修改完成后可注释..."
pause


::1）通过nrfutil生产私钥和公钥文件
::generate private key
nrfutil keys generate priv.pem
::generate public key related with private key: priv.pem
nrfutil keys display --key pk --format code priv.pem --out_file dfu_public_key.c


::2）复制dfu_public_key.c文件到dfu工程
::修改指定文件夹或文件自己修改ObjPath源文件位置，HexDestPath目标文件或文件夹位置
set ObjPath=dfu_public_key.c

set HexDestPath=D:\keil_workspace\NORDIC\nRF52832_htwh_sdk15.0\examples\dfu

::复制指定路径指定文件或文件夹，至HexDestPath路径文件夹
echo y | xcopy "%ObjPath%" /e /r /k "%HexDestPath%"
::xcopy /e/c/h/z "%~pd0*.*" "%out%"

pause




