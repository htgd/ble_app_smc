@echo off

pause
echo "ִ��֮ǰ��Ҫȷ���ļ�·�����޸���ɺ��ע��..."
pause


::1��ͨ��nrfutil����˽Կ�͹�Կ�ļ�
::generate private key
nrfutil keys generate priv.pem
::generate public key related with private key: priv.pem
nrfutil keys display --key pk --format code priv.pem --out_file dfu_public_key.c


::2������dfu_public_key.c�ļ���dfu����
::�޸�ָ���ļ��л��ļ��Լ��޸�ObjPathԴ�ļ�λ�ã�HexDestPathĿ���ļ����ļ���λ��
set ObjPath=dfu_public_key.c

set HexDestPath=D:\keil_workspace\NORDIC\nRF52832_htwh_sdk15.0\examples\dfu

::����ָ��·��ָ���ļ����ļ��У���HexDestPath·���ļ���
echo y | xcopy "%ObjPath%" /e /r /k "%HexDestPath%"
::xcopy /e/c/h/z "%~pd0*.*" "%out%"

pause




