################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL.c \
../DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL_LL.c 

OBJS += \
./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL.o \
./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL_LL.o 

C_DEPS += \
./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL.d \
./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL_LL.d 


# Each subdirectory must supply rules for building sources it contributes
DEVICE_SYSTEM_CONTROL/%.o DEVICE_SYSTEM_CONTROL/%.su DEVICE_SYSTEM_CONTROL/%.cyclo: ../DEVICE_SYSTEM_CONTROL/%.c DEVICE_SYSTEM_CONTROL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/MCLib/F4xx/Inc -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Drivers/CMSIS/DSP/Include -I"C:/Users/TerenceLeung/Documents/STM32_Prototyping/MOTOR_CTL_F401_RTOS/MOTOR_CTL_F401_RTOS/STM32CubeIDE/ESCOOTER" -I"C:/Users/TerenceLeung/Documents/STM32_Prototyping/MOTOR_CTL_F401_RTOS/MOTOR_CTL_F401_RTOS/STM32CubeIDE/POWER_CONTROL" -I"C:/Users/TerenceLeung/Documents/STM32_Prototyping/MOTOR_CTL_F401_RTOS/MOTOR_CTL_F401_RTOS/STM32CubeIDE/DEVICE_SYSTEM_CONTROL" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DEVICE_SYSTEM_CONTROL

clean-DEVICE_SYSTEM_CONTROL:
	-$(RM) ./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL.cyclo ./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL.d ./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL.o ./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL.su ./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL_LL.cyclo ./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL_LL.d ./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL_LL.o ./DEVICE_SYSTEM_CONTROL/DEVICE_CONTROL_PROTOCOL_LL.su

.PHONY: clean-DEVICE_SYSTEM_CONTROL

