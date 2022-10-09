################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/App/rx.c \
../Src/App/table.c \
../Src/App/util.c 

OBJS += \
./Src/App/rx.o \
./Src/App/table.o \
./Src/App/util.o 

C_DEPS += \
./Src/App/rx.d \
./Src/App/table.d \
./Src/App/util.d 


# Each subdirectory must supply rules for building sources it contributes
Src/App/%.o: ../Src/App/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32L476xx -I"D:/Users/Tyler/Documents/GitHub/oil_lamp/mcu_l476rg/Inc" -I"D:/Users/Tyler/Documents/GitHub/oil_lamp/mcu_l476rg/Drivers/STM32L4xx_HAL_Driver/Inc" -I"D:/Users/Tyler/Documents/GitHub/oil_lamp/mcu_l476rg/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"D:/Users/Tyler/Documents/GitHub/oil_lamp/mcu_l476rg/Middlewares/Third_Party/FreeRTOS/Source/include" -I"D:/Users/Tyler/Documents/GitHub/oil_lamp/mcu_l476rg/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"D:/Users/Tyler/Documents/GitHub/oil_lamp/mcu_l476rg/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"D:/Users/Tyler/Documents/GitHub/oil_lamp/mcu_l476rg/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"D:/Users/Tyler/Documents/GitHub/oil_lamp/mcu_l476rg/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


