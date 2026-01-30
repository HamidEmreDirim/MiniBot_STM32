################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/library/bme68x/bme68x.c \
../Core/Src/library/bme68x/bme68x_necessary_functions.c 

OBJS += \
./Core/Src/library/bme68x/bme68x.o \
./Core/Src/library/bme68x/bme68x_necessary_functions.o 

C_DEPS += \
./Core/Src/library/bme68x/bme68x.d \
./Core/Src/library/bme68x/bme68x_necessary_functions.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/library/bme68x/%.o Core/Src/library/bme68x/%.su Core/Src/library/bme68x/%.cyclo: ../Core/Src/library/bme68x/%.c Core/Src/library/bme68x/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-library-2f-bme68x

clean-Core-2f-Src-2f-library-2f-bme68x:
	-$(RM) ./Core/Src/library/bme68x/bme68x.cyclo ./Core/Src/library/bme68x/bme68x.d ./Core/Src/library/bme68x/bme68x.o ./Core/Src/library/bme68x/bme68x.su ./Core/Src/library/bme68x/bme68x_necessary_functions.cyclo ./Core/Src/library/bme68x/bme68x_necessary_functions.d ./Core/Src/library/bme68x/bme68x_necessary_functions.o ./Core/Src/library/bme68x/bme68x_necessary_functions.su

.PHONY: clean-Core-2f-Src-2f-library-2f-bme68x

