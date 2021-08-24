################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/app_freertos.c \
../Core/Src/main.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_hal_timebase_tim.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c 

OBJS += \
./Core/Src/app_freertos.o \
./Core/Src/main.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_hal_timebase_tim.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o 

C_DEPS += \
./Core/Src/app_freertos.d \
./Core/Src/main.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_hal_timebase_tim.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG '-DCMSIS_device_header=<stm32g0xx.h>' -DUSE_HAL_DRIVER -DSTM32G041xx -c -I../Core/Inc -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/STM32G0xx_HAL_Driver/Inc -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Middlewares/Third_Party/FreeRTOS/Source/include -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/CMSIS/Device/ST/STM32G0xx/Include -IC:/Users/huber/STM32Cube/Repository/STM32Cube_FW_G0_V1.5.0/Drivers/CMSIS/Include -I"C:/Users/huber/STM32CubeIDE/workspace_1.6.1/MODBUS-LIB/Inc" -O2 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

