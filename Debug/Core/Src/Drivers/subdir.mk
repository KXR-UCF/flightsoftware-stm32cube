################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Drivers/AD7124.c \
../Core/Src/Drivers/pressure_transducer.c \
../Core/Src/Drivers/servo.c \
../Core/Src/Drivers/solinoid.c \
../Core/Src/Drivers/thermocouple.c 

OBJS += \
./Core/Src/Drivers/AD7124.o \
./Core/Src/Drivers/pressure_transducer.o \
./Core/Src/Drivers/servo.o \
./Core/Src/Drivers/solinoid.o \
./Core/Src/Drivers/thermocouple.o 

C_DEPS += \
./Core/Src/Drivers/AD7124.d \
./Core/Src/Drivers/pressure_transducer.d \
./Core/Src/Drivers/servo.d \
./Core/Src/Drivers/solinoid.d \
./Core/Src/Drivers/thermocouple.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Drivers/%.o Core/Src/Drivers/%.su Core/Src/Drivers/%.cyclo: ../Core/Src/Drivers/%.c Core/Src/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Drivers

clean-Core-2f-Src-2f-Drivers:
	-$(RM) ./Core/Src/Drivers/AD7124.cyclo ./Core/Src/Drivers/AD7124.d ./Core/Src/Drivers/AD7124.o ./Core/Src/Drivers/AD7124.su ./Core/Src/Drivers/pressure_transducer.cyclo ./Core/Src/Drivers/pressure_transducer.d ./Core/Src/Drivers/pressure_transducer.o ./Core/Src/Drivers/pressure_transducer.su ./Core/Src/Drivers/servo.cyclo ./Core/Src/Drivers/servo.d ./Core/Src/Drivers/servo.o ./Core/Src/Drivers/servo.su ./Core/Src/Drivers/solinoid.cyclo ./Core/Src/Drivers/solinoid.d ./Core/Src/Drivers/solinoid.o ./Core/Src/Drivers/solinoid.su ./Core/Src/Drivers/thermocouple.cyclo ./Core/Src/Drivers/thermocouple.d ./Core/Src/Drivers/thermocouple.o ./Core/Src/Drivers/thermocouple.su

.PHONY: clean-Core-2f-Src-2f-Drivers

