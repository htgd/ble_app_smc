@echo off

::1��ʹ��nrfjprog��¼whole.hex

::����ȫ��flash����
nrfjprog.exe --family NRF52 --eraseall
::��¼�ļ�
nrfjprog.exe --family NRF52 --program whole.hex
::verify�ļ�
::nrfjprog.exe --family NRF52 --verify whole.hex
::��λ
nrfjprog.exe --reset

::nrfjprog.exe --eraseall -f NRF52
::nrfjprog.exe --program whole.hex --verify -f NRF52 
::the following two commands are used to enable PIN RESET          
::nrfjprog.exe --memwr 0x10001200 --val 0x00000015 --verify -f NRF52    
::nrfjprog.exe --memwr 0x10001204 --val 0x00000015 --verify -f NRF52   
::nrfjprog.exe --reset -f NRF52

pause 