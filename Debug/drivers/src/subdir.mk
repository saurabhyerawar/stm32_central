################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f446xx_gpio_driver.c 

OBJS += \
./drivers/src/stm32f446xx_gpio_driver.o 

C_DEPS += \
./drivers/src/stm32f446xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/stm32f446xx_gpio_driver.o: ../drivers/src/stm32f446xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DDEBUG -DNUCLEO_F446RE -c -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/inc" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/drivers/external_comp" -I"E:/Saurabh Study/stm32/MCU Course/workspace/stm32_central/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f446xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

