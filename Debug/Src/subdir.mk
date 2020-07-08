################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Stm32_keypad_appl.c \
../Src/led_appl.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/Stm32_keypad_appl.o \
./Src/led_appl.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/Stm32_keypad_appl.d \
./Src/led_appl.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/Stm32_keypad_appl.o: ../Src/Stm32_keypad_appl.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/inc" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/external_comp" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/Stm32_keypad_appl.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/led_appl.o: ../Src/led_appl.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/inc" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/external_comp" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/led_appl.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/main.o: ../Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/inc" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/external_comp" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/main.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/inc" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/external_comp" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/inc" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/external_comp" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

