################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Inc/i2c-lcd.c 

OBJS += \
./Core/Src/Inc/i2c-lcd.o 

C_DEPS += \
./Core/Src/Inc/i2c-lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Inc/%.o Core/Src/Inc/%.su Core/Src/Inc/%.cyclo: ../Core/Src/Inc/%.c Core/Src/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Inc

clean-Core-2f-Src-2f-Inc:
	-$(RM) ./Core/Src/Inc/i2c-lcd.cyclo ./Core/Src/Inc/i2c-lcd.d ./Core/Src/Inc/i2c-lcd.o ./Core/Src/Inc/i2c-lcd.su

.PHONY: clean-Core-2f-Src-2f-Inc

