################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/TerenceLeung/Documents/STM32_Prototyping/MOTOR_CTL_F401_RTOS/MOTOR_CTL_F401_RTOS/Src/system_stm32f4xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32f4xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32f4xx.o: C:/Users/TerenceLeung/Documents/STM32_Prototyping/MOTOR_CTL_F401_RTOS/MOTOR_CTL_F401_RTOS/Src/system_stm32f4xx.c Drivers/CMSIS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/MCLib/F4xx/Inc -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Drivers/CMSIS/DSP/Include -I"C:/Users/TerenceLeung/Documents/STM32_Prototyping/MOTOR_CTL_F401_RTOS/MOTOR_CTL_F401_RTOS/STM32CubeIDE/ESCOOTER" -I"C:/Users/TerenceLeung/Documents/STM32_Prototyping/MOTOR_CTL_F401_RTOS/MOTOR_CTL_F401_RTOS/STM32CubeIDE/POWER_CONTROL" -I"C:/Users/TerenceLeung/Documents/STM32_Prototyping/MOTOR_CTL_F401_RTOS/MOTOR_CTL_F401_RTOS/STM32CubeIDE/DEVICE_SYSTEM_CONTROL" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS

clean-Drivers-2f-CMSIS:
	-$(RM) ./Drivers/CMSIS/system_stm32f4xx.cyclo ./Drivers/CMSIS/system_stm32f4xx.d ./Drivers/CMSIS/system_stm32f4xx.o ./Drivers/CMSIS/system_stm32f4xx.su

.PHONY: clean-Drivers-2f-CMSIS

