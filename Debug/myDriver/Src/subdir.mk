################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../myDriver/Src/stm32f103xx.c 

OBJS += \
./myDriver/Src/stm32f103xx.o 

C_DEPS += \
./myDriver/Src/stm32f103xx.d 


# Each subdirectory must supply rules for building sources it contributes
myDriver/Src/%.o: ../myDriver/Src/%.c myDriver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-myDriver-2f-Src

clean-myDriver-2f-Src:
	-$(RM) ./myDriver/Src/stm32f103xx.d ./myDriver/Src/stm32f103xx.o

.PHONY: clean-myDriver-2f-Src

