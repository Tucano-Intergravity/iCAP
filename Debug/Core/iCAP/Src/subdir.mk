################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/iCAP/Src/PowerControl.c \
../Core/iCAP/Src/SSRControl.c \
../Core/iCAP/Src/UARTComm.c \
../Core/iCAP/Src/imu.c \
../Core/iCAP/Src/pt.c \
../Core/iCAP/Src/tc.c 

OBJS += \
./Core/iCAP/Src/PowerControl.o \
./Core/iCAP/Src/SSRControl.o \
./Core/iCAP/Src/UARTComm.o \
./Core/iCAP/Src/imu.o \
./Core/iCAP/Src/pt.o \
./Core/iCAP/Src/tc.o 

C_DEPS += \
./Core/iCAP/Src/PowerControl.d \
./Core/iCAP/Src/SSRControl.d \
./Core/iCAP/Src/UARTComm.d \
./Core/iCAP/Src/imu.d \
./Core/iCAP/Src/pt.d \
./Core/iCAP/Src/tc.d 


# Each subdirectory must supply rules for building sources it contributes
Core/iCAP/Src/%.o Core/iCAP/Src/%.su Core/iCAP/Src/%.cyclo: ../Core/iCAP/Src/%.c Core/iCAP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H755xx -DUSE_PWR_DIRECT_SMPS_SUPPLY -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"C:/STM3INTERGRAVITY/iCAP_testproject/CM7/Core/iCAP/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-iCAP-2f-Src

clean-Core-2f-iCAP-2f-Src:
	-$(RM) ./Core/iCAP/Src/PowerControl.cyclo ./Core/iCAP/Src/PowerControl.d ./Core/iCAP/Src/PowerControl.o ./Core/iCAP/Src/PowerControl.su ./Core/iCAP/Src/SSRControl.cyclo ./Core/iCAP/Src/SSRControl.d ./Core/iCAP/Src/SSRControl.o ./Core/iCAP/Src/SSRControl.su ./Core/iCAP/Src/UARTComm.cyclo ./Core/iCAP/Src/UARTComm.d ./Core/iCAP/Src/UARTComm.o ./Core/iCAP/Src/UARTComm.su ./Core/iCAP/Src/imu.cyclo ./Core/iCAP/Src/imu.d ./Core/iCAP/Src/imu.o ./Core/iCAP/Src/imu.su ./Core/iCAP/Src/pt.cyclo ./Core/iCAP/Src/pt.d ./Core/iCAP/Src/pt.o ./Core/iCAP/Src/pt.su ./Core/iCAP/Src/tc.cyclo ./Core/iCAP/Src/tc.d ./Core/iCAP/Src/tc.o ./Core/iCAP/Src/tc.su

.PHONY: clean-Core-2f-iCAP-2f-Src

