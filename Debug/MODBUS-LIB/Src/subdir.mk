################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/huber/STM32CubeIDE/workspace_1.6.1/MODBUS-LIB/Src/Modbus.c \
C:/Users/huber/STM32CubeIDE/workspace_1.6.1/MODBUS-LIB/Src/UARTCallback.c 

OBJS += \
./MODBUS-LIB/Src/Modbus.o \
./MODBUS-LIB/Src/UARTCallback.o 

C_DEPS += \
./MODBUS-LIB/Src/Modbus.d \
./MODBUS-LIB/Src/UARTCallback.d 


# Each subdirectory must supply rules for building sources it contributes
MODBUS-LIB/Src/Modbus.o: C:/Users/huber/STM32CubeIDE/workspace_1.6.1/MODBUS-LIB/Src/Modbus.c MODBUS-LIB/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG '-DCMSIS_device_header=<stm32g0xx.h>' -DUSE_HAL_DRIVER -DSTM32G041xx -c -I../Core/Inc -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/CMSIS/Include -I"C:/Users/huber/STM32CubeIDE/workspace_1.6.1/MODBUS-LIB/Inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MODBUS-LIB/Src/UARTCallback.o: C:/Users/huber/STM32CubeIDE/workspace_1.6.1/MODBUS-LIB/Src/UARTCallback.c MODBUS-LIB/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG '-DCMSIS_device_header=<stm32g0xx.h>' -DUSE_HAL_DRIVER -DSTM32G041xx -c -I../Core/Inc -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/CMSIS/Include -I"C:/Users/huber/STM32CubeIDE/workspace_1.6.1/MODBUS-LIB/Inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

