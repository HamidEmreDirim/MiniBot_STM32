################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/library/EEPROM.c \
../Core/Src/library/INA219.c \
../Core/Src/library/adxl_345_imu.c \
../Core/Src/library/airQuality.c \
../Core/Src/library/encoder.c \
../Core/Src/library/gps.c \
../Core/Src/library/motorDriver.c \
../Core/Src/library/sps30.c 

OBJS += \
./Core/Src/library/EEPROM.o \
./Core/Src/library/INA219.o \
./Core/Src/library/adxl_345_imu.o \
./Core/Src/library/airQuality.o \
./Core/Src/library/encoder.o \
./Core/Src/library/gps.o \
./Core/Src/library/motorDriver.o \
./Core/Src/library/sps30.o 

C_DEPS += \
./Core/Src/library/EEPROM.d \
./Core/Src/library/INA219.d \
./Core/Src/library/adxl_345_imu.d \
./Core/Src/library/airQuality.d \
./Core/Src/library/encoder.d \
./Core/Src/library/gps.d \
./Core/Src/library/motorDriver.d \
./Core/Src/library/sps30.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/library/%.o Core/Src/library/%.su Core/Src/library/%.cyclo: ../Core/Src/library/%.c Core/Src/library/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-library

clean-Core-2f-Src-2f-library:
	-$(RM) ./Core/Src/library/EEPROM.cyclo ./Core/Src/library/EEPROM.d ./Core/Src/library/EEPROM.o ./Core/Src/library/EEPROM.su ./Core/Src/library/INA219.cyclo ./Core/Src/library/INA219.d ./Core/Src/library/INA219.o ./Core/Src/library/INA219.su ./Core/Src/library/adxl_345_imu.cyclo ./Core/Src/library/adxl_345_imu.d ./Core/Src/library/adxl_345_imu.o ./Core/Src/library/adxl_345_imu.su ./Core/Src/library/airQuality.cyclo ./Core/Src/library/airQuality.d ./Core/Src/library/airQuality.o ./Core/Src/library/airQuality.su ./Core/Src/library/encoder.cyclo ./Core/Src/library/encoder.d ./Core/Src/library/encoder.o ./Core/Src/library/encoder.su ./Core/Src/library/gps.cyclo ./Core/Src/library/gps.d ./Core/Src/library/gps.o ./Core/Src/library/gps.su ./Core/Src/library/motorDriver.cyclo ./Core/Src/library/motorDriver.d ./Core/Src/library/motorDriver.o ./Core/Src/library/motorDriver.su ./Core/Src/library/sps30.cyclo ./Core/Src/library/sps30.d ./Core/Src/library/sps30.o ./Core/Src/library/sps30.su

.PHONY: clean-Core-2f-Src-2f-library

