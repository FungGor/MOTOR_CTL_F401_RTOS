################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ESCOOTER_APPLICATION/ESCOOTER_TASK.c 

OBJS += \
./ESCOOTER_APPLICATION/ESCOOTER_TASK.o 

C_DEPS += \
./ESCOOTER_APPLICATION/ESCOOTER_TASK.d 


# Each subdirectory must supply rules for building sources it contributes
ESCOOTER_APPLICATION/%.o ESCOOTER_APPLICATION/%.su ESCOOTER_APPLICATION/%.cyclo: ../ESCOOTER_APPLICATION/%.c ESCOOTER_APPLICATION/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/MCLib/Any/Inc -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/MCLib/F4xx/Inc -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/UILibrary/Inc -I../../MCSDK_v5.4.8-Full/MotorControl/MCSDK/SystemDriveParams -I../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../../Drivers/CMSIS/DSP/Include -I"C:/Users/TerenceLeung/Documents/STM32_Prototyping/MOTOR_CTL_F401_RTOS/MOTOR_CTL_F401_RTOS/STM32CubeIDE/ESCOOTER_APPLICATION" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ESCOOTER_APPLICATION

clean-ESCOOTER_APPLICATION:
	-$(RM) ./ESCOOTER_APPLICATION/ESCOOTER_TASK.cyclo ./ESCOOTER_APPLICATION/ESCOOTER_TASK.d ./ESCOOTER_APPLICATION/ESCOOTER_TASK.o ./ESCOOTER_APPLICATION/ESCOOTER_TASK.su

.PHONY: clean-ESCOOTER_APPLICATION

