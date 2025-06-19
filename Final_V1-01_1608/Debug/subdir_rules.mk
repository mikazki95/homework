################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/home/gabriela/ti/ccs1220/ccs/tools/compiler/ti-cgt-armllvm_2.1.2.LTS/bin/tiarmclang" -c -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -mthumb -I"/home/gabriela/ti/ccs1220/ccs/tools/compiler/ti-cgt-armllvm_2.1.2.LTS/include/c" -I"/home/gabriela/ti/mcu_plus_sdk_am64x_09_00_00_35/source" -I"/home/gabriela/ti/mcu_plus_sdk_am64x_09_00_00_35/source/kernel/freertos/FreeRTOS-Kernel/include" -I"/home/gabriela/ti/mcu_plus_sdk_am64x_09_00_00_35/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F" -I"/home/gabriela/ti/mcu_plus_sdk_am64x_09_00_00_35/source/kernel/freertos/config/am64x/r5f" -DSOC_AM64X -D_DEBUG_=1 -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"/home/gabriela/Documentos/Respaldos_Arritmias/FINALES/Final_V1-01_1608/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1057410319: ../rproc_r5f0_0.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"/home/gabriela/ti/ccs1220/ccs/utils/sysconfig_1.15.0/sysconfig_cli.sh" -s "/home/gabriela/ti/mcu_plus_sdk_am64x_09_00_00_35/.metadata/product.json" --script "/home/gabriela/Documentos/Respaldos_Arritmias/FINALES/Final_V1-01_1608/rproc_r5f0_0.syscfg" --context "r5fss0-0" -o "syscfg" --part Default --package ALV --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/error.h: build-1057410319 ../rproc_r5f0_0.syscfg
syscfg/: build-1057410319


