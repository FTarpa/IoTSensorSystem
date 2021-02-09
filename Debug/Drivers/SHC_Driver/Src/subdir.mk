################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SHC_Driver/Src/ESP32_Driver.c \
../Drivers/SHC_Driver/Src/RL78_Sensor.c \
../Drivers/SHC_Driver/Src/SHTC3_Driver.c \
../Drivers/SHC_Driver/Src/retarget.c 

OBJS += \
./Drivers/SHC_Driver/Src/ESP32_Driver.o \
./Drivers/SHC_Driver/Src/RL78_Sensor.o \
./Drivers/SHC_Driver/Src/SHTC3_Driver.o \
./Drivers/SHC_Driver/Src/retarget.o 

C_DEPS += \
./Drivers/SHC_Driver/Src/ESP32_Driver.d \
./Drivers/SHC_Driver/Src/RL78_Sensor.d \
./Drivers/SHC_Driver/Src/SHTC3_Driver.d \
./Drivers/SHC_Driver/Src/retarget.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SHC_Driver/Src/ESP32_Driver.o: ../Drivers/SHC_Driver/Src/ESP32_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../Core/Inc -I../Drivers/SHC_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/SHC_Driver/Src/ESP32_Driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/SHC_Driver/Src/RL78_Sensor.o: ../Drivers/SHC_Driver/Src/RL78_Sensor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../Core/Inc -I../Drivers/SHC_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/SHC_Driver/Src/RL78_Sensor.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/SHC_Driver/Src/SHTC3_Driver.o: ../Drivers/SHC_Driver/Src/SHTC3_Driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../Core/Inc -I../Drivers/SHC_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/SHC_Driver/Src/SHTC3_Driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/SHC_Driver/Src/retarget.o: ../Drivers/SHC_Driver/Src/retarget.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F413xx -DDEBUG -c -I../Core/Inc -I../Drivers/SHC_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/SHC_Driver/Src/retarget.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

