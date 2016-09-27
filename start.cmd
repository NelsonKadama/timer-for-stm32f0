arm-none-eabi-as -mcpu=cortex-m0 -mthumb -g -o timer.o timer.s

arm-none-eabi-ld -Ttext=0x08000000 -o timer.elf timer.o

arm-none-eabi-gdb timer.elf