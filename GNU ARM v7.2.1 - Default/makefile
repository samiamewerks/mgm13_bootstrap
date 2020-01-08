################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include protocol/bluetooth/lib/EFR32MG13P/GCC/subdir.mk
-include platform/emlib/src/subdir.mk
-include platform/emdrv/spidrv/src/subdir.mk
-include platform/emdrv/sleep/src/subdir.mk
-include platform/emdrv/rtcdrv/src/subdir.mk
-include platform/emdrv/dmadrv/src/subdir.mk
-include platform/Device/SiliconLabs/MGM13/Source/GCC/subdir.mk
-include platform/Device/SiliconLabs/MGM13/Source/subdir.mk
-include hardware/kit/common/drivers/subdir.mk
-include hardware/kit/common/bsp/subdir.mk
-include app/bluetooth/common/util/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: mgm13_bootstrap.axf

# Tool invocations
mgm13_bootstrap.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\samia\SimplicityStudio\v4_workspace\mgm13_bootstrap\mgm13_bootstrap.ld" -Xlinker --gc-sections -Xlinker -Map="mgm13_bootstrap.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o mgm13_bootstrap.axf -Wl,--start-group "./app/bluetooth/common/util/infrastructure.o" "./app.o" "./application_properties.o" "./gatt_db.o" "./init_app.o" "./init_board_efr32xg1.o" "./init_mcu_efr32xg1.o" "./main.o" "./hardware/kit/common/bsp/bsp_stk.o" "./hardware/kit/common/drivers/retargetio.o" "./hardware/kit/common/drivers/retargetserial.o" "./platform/Device/SiliconLabs/MGM13/Source/GCC/startup_mgm13.o" "./platform/Device/SiliconLabs/MGM13/Source/system_mgm13.o" "./platform/emdrv/dmadrv/src/dmadrv.o" "./platform/emdrv/rtcdrv/src/rtcdriver.o" "./platform/emdrv/sleep/src/sleep.o" "./platform/emdrv/spidrv/src/spidrv.o" "./platform/emlib/src/em_assert.o" "./platform/emlib/src/em_burtc.o" "./platform/emlib/src/em_cmu.o" "./platform/emlib/src/em_core.o" "./platform/emlib/src/em_cryotimer.o" "./platform/emlib/src/em_crypto.o" "./platform/emlib/src/em_emu.o" "./platform/emlib/src/em_gpio.o" "./platform/emlib/src/em_i2c.o" "./platform/emlib/src/em_ldma.o" "./platform/emlib/src/em_msc.o" "./platform/emlib/src/em_rmu.o" "./platform/emlib/src/em_rtcc.o" "./platform/emlib/src/em_se.o" "./platform/emlib/src/em_system.o" "./platform/emlib/src/em_timer.o" "./platform/emlib/src/em_usart.o" "C:\Users\samia\SimplicityStudio\v4_workspace\mgm13_bootstrap\protocol\bluetooth\lib\EFR32MG13P\GCC\libpsstore.a" "C:\Users\samia\SimplicityStudio\v4_workspace\mgm13_bootstrap\protocol\bluetooth\lib\EFR32MG13P\GCC\binapploader.o" "C:\Users\samia\SimplicityStudio\v4_workspace\mgm13_bootstrap\protocol\bluetooth\lib\EFR32MG13P\GCC\libmbedtls.a" "C:\Users\samia\SimplicityStudio\v4_workspace\mgm13_bootstrap\protocol\bluetooth\lib\EFR32MG13P\GCC\libbluetooth.a" "C:\Users\samia\SimplicityStudio\v4_workspace\mgm13_bootstrap\platform\radio\rail_lib\autogen\librail_release\librail_module_efr32xg13_gcc_release.a" "C:\Users\samia\SimplicityStudio\v4_workspace\mgm13_bootstrap\platform\radio\rail_lib\autogen\librail_release\librail_config_mgm13s12f512ga_gcc.a" -lm -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: mgm13_bootstrap.hex'
	arm-none-eabi-objcopy -O ihex "mgm13_bootstrap.axf" "mgm13_bootstrap.hex"
	@echo ' '

	@echo 'Building bin file: mgm13_bootstrap.bin'
	arm-none-eabi-objcopy -O binary "mgm13_bootstrap.axf" "mgm13_bootstrap.bin"
	@echo ' '

	@echo 'Building s37 file: mgm13_bootstrap.s37'
	arm-none-eabi-objcopy -O srec "mgm13_bootstrap.axf" "mgm13_bootstrap.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "mgm13_bootstrap.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) mgm13_bootstrap.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
