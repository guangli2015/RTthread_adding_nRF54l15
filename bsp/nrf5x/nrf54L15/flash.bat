C:\work\env-windows-v2.0.0\env-windows\tools\gnu_gcc\arm_gcc\mingw\arm-none-eabi\bin\objcopy --input-target=binary --output-target=ihex --change-address 0x0000 --gap-fill=0xff  rtthread.bin rtthread.hex



nrfjprog --recover
nrfjprog -e 
nrfjprog --program rtthread.hex --verify
nrfjprog --reset

pause