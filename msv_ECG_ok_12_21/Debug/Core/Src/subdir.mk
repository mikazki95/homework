################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ads1298.c \
../Core/Src/dma.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/led_neopixel.c \
../Core/Src/led_rgb_simple.c \
../Core/Src/main.c \
../Core/Src/neopixel_basic.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tarea_CalcularOffset.c \
../Core/Src/tarea_ControlNeopixel.c \
../Core/Src/tarea_ControlUSB.c \
../Core/Src/tarea_FrecuenciaCardiaca.c \
../Core/Src/tarea_GraficarECG.c \
../Core/Src/tarea_LeerADS.c \
../Core/Src/tarea_Normalizacion.c \
../Core/Src/tarea_Respiracion.c \
../Core/Src/tarea_RevisarComandos.c \
../Core/Src/tarea_RevisarElectrodos.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

C_DEPS += \
./Core/Src/ads1298.d \
./Core/Src/dma.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/led_neopixel.d \
./Core/Src/led_rgb_simple.d \
./Core/Src/main.d \
./Core/Src/neopixel_basic.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tarea_CalcularOffset.d \
./Core/Src/tarea_ControlNeopixel.d \
./Core/Src/tarea_ControlUSB.d \
./Core/Src/tarea_FrecuenciaCardiaca.d \
./Core/Src/tarea_GraficarECG.d \
./Core/Src/tarea_LeerADS.d \
./Core/Src/tarea_Normalizacion.d \
./Core/Src/tarea_Respiracion.d \
./Core/Src/tarea_RevisarComandos.d \
./Core/Src/tarea_RevisarElectrodos.d \
./Core/Src/tim.d \
./Core/Src/usart.d 

OBJS += \
./Core/Src/ads1298.o \
./Core/Src/dma.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/led_neopixel.o \
./Core/Src/led_rgb_simple.o \
./Core/Src/main.o \
./Core/Src/neopixel_basic.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tarea_CalcularOffset.o \
./Core/Src/tarea_ControlNeopixel.o \
./Core/Src/tarea_ControlUSB.o \
./Core/Src/tarea_FrecuenciaCardiaca.o \
./Core/Src/tarea_GraficarECG.o \
./Core/Src/tarea_LeerADS.o \
./Core/Src/tarea_Normalizacion.o \
./Core/Src/tarea_Respiracion.o \
./Core/Src/tarea_RevisarComandos.o \
./Core/Src/tarea_RevisarElectrodos.o \
./Core/Src/tim.o \
./Core/Src/usart.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/CMSIS/DSP/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ads1298.cyclo ./Core/Src/ads1298.d ./Core/Src/ads1298.o ./Core/Src/ads1298.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/led_neopixel.cyclo ./Core/Src/led_neopixel.d ./Core/Src/led_neopixel.o ./Core/Src/led_neopixel.su ./Core/Src/led_rgb_simple.cyclo ./Core/Src/led_rgb_simple.d ./Core/Src/led_rgb_simple.o ./Core/Src/led_rgb_simple.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/neopixel_basic.cyclo ./Core/Src/neopixel_basic.d ./Core/Src/neopixel_basic.o ./Core/Src/neopixel_basic.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.cyclo ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tarea_CalcularOffset.cyclo ./Core/Src/tarea_CalcularOffset.d ./Core/Src/tarea_CalcularOffset.o ./Core/Src/tarea_CalcularOffset.su ./Core/Src/tarea_ControlNeopixel.cyclo ./Core/Src/tarea_ControlNeopixel.d ./Core/Src/tarea_ControlNeopixel.o ./Core/Src/tarea_ControlNeopixel.su ./Core/Src/tarea_ControlUSB.cyclo ./Core/Src/tarea_ControlUSB.d ./Core/Src/tarea_ControlUSB.o ./Core/Src/tarea_ControlUSB.su ./Core/Src/tarea_FrecuenciaCardiaca.cyclo ./Core/Src/tarea_FrecuenciaCardiaca.d ./Core/Src/tarea_FrecuenciaCardiaca.o ./Core/Src/tarea_FrecuenciaCardiaca.su ./Core/Src/tarea_GraficarECG.cyclo ./Core/Src/tarea_GraficarECG.d ./Core/Src/tarea_GraficarECG.o ./Core/Src/tarea_GraficarECG.su ./Core/Src/tarea_LeerADS.cyclo ./Core/Src/tarea_LeerADS.d ./Core/Src/tarea_LeerADS.o ./Core/Src/tarea_LeerADS.su ./Core/Src/tarea_Normalizacion.cyclo ./Core/Src/tarea_Normalizacion.d ./Core/Src/tarea_Normalizacion.o ./Core/Src/tarea_Normalizacion.su ./Core/Src/tarea_Respiracion.cyclo ./Core/Src/tarea_Respiracion.d ./Core/Src/tarea_Respiracion.o ./Core/Src/tarea_Respiracion.su ./Core/Src/tarea_RevisarComandos.cyclo ./Core/Src/tarea_RevisarComandos.d ./Core/Src/tarea_RevisarComandos.o ./Core/Src/tarea_RevisarComandos.su ./Core/Src/tarea_RevisarElectrodos.cyclo ./Core/Src/tarea_RevisarElectrodos.d ./Core/Src/tarea_RevisarElectrodos.o ./Core/Src/tarea_RevisarElectrodos.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

