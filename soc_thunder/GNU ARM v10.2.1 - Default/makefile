################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/subdir.mk
-include gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_protocol_crypto/src/subdir.mk
-include gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/subdir.mk
-include gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/subdir.mk
-include gecko_sdk_4.0.1/util/silicon_labs/silabs_core/memory_manager/subdir.mk
-include gecko_sdk_4.0.1/protocol/bluetooth/src/subdir.mk
-include gecko_sdk_4.0.1/protocol/bluetooth/lib/EFR32MG12P/GCC/subdir.mk
-include gecko_sdk_4.0.1/platform/service/udelay/src/subdir.mk
-include gecko_sdk_4.0.1/platform/service/system/src/subdir.mk
-include gecko_sdk_4.0.1/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_4.0.1/platform/service/power_manager/src/subdir.mk
-include gecko_sdk_4.0.1/platform/service/mpu/src/subdir.mk
-include gecko_sdk_4.0.1/platform/service/iostream/src/subdir.mk
-include gecko_sdk_4.0.1/platform/service/device_init/src/subdir.mk
-include gecko_sdk_4.0.1/platform/service/cli/src/subdir.mk
-include gecko_sdk_4.0.1/platform/radio/rail_lib/plugin/rail_util_pti/subdir.mk
-include gecko_sdk_4.0.1/platform/radio/rail_lib/plugin/pa-conversions/subdir.mk
-include gecko_sdk_4.0.1/platform/emlib/src/subdir.mk
-include gecko_sdk_4.0.1/platform/emdrv/nvm3/src/subdir.mk
-include gecko_sdk_4.0.1/platform/emdrv/gpiointerrupt/src/subdir.mk
-include gecko_sdk_4.0.1/platform/driver/leddrv/src/subdir.mk
-include gecko_sdk_4.0.1/platform/driver/button/src/subdir.mk
-include gecko_sdk_4.0.1/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_4.0.1/platform/common/src/subdir.mk
-include gecko_sdk_4.0.1/platform/bootloader/app_properties/subdir.mk
-include gecko_sdk_4.0.1/platform/bootloader/api/subdir.mk
-include gecko_sdk_4.0.1/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC/subdir.mk
-include gecko_sdk_4.0.1/platform/Device/SiliconLabs/EFR32MG12P/Source/subdir.mk
-include gecko_sdk_4.0.1/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/subdir.mk
-include gecko_sdk_4.0.1/hardware/board/src/subdir.mk
-include gecko_sdk_4.0.1/app/common/util/app_log/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/simple_timer/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/ota_dfu/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_time_server/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_scheduler_server/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_scene_server/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_provisioning_decorator/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_lighting_server/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_lc_server/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_generic_base/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_friend/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_factory_reset/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_event_log/subdir.mk
-include gecko_sdk_4.0.1/app/bluetooth/common/btmesh_ctl_server/subdir.mk
-include autogen/subdir.mk
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
all: soc_thunder.axf

# Tool invocations
soc_thunder.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "C:\Users\Ander\Documents\GitHub\MG13\ble5_thunder_light\soc_thunder/autogen/linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="soc_thunder.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o soc_thunder.axf -Wl,--start-group "./app.o" "./app_led_rgb.o" "./app_out_log.o" "./local_cli_commands.o" "./main.o" "./rgbled.o" "./autogen/gatt_db.o" "./autogen/sl_bluetooth.o" "./autogen/sl_board_default_init.o" "./autogen/sl_btmesh.o" "./autogen/sl_btmesh_dcd.o" "./autogen/sl_cli_command_table.o" "./autogen/sl_cli_instances.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_iostream_handles.o" "./autogen/sl_iostream_init_usart_instances.o" "./autogen/sl_power_manager_handler.o" "./autogen/sl_simple_button_instances.o" "./autogen/sl_simple_rgbw_pwm_led_instances.o" "./autogen/sli_cli_hooks.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_ctl_server/sl_btmesh_ctl_server.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_ctl_server/sl_btmesh_ctl_signal_transition_handler.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_event_log/sl_btmesh_event_log.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_factory_reset/sl_btmesh_factory_reset.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_friend/sl_btmesh_friend.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_generic_base/sl_btmesh_generic_base.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_lc_server/sl_btmesh_lc_server.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_lighting_server/sl_btmesh_lighting_level_transition_handler.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_lighting_server/sl_btmesh_lighting_server.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_provisioning_decorator/sl_btmesh_provisioning_decorator.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_scene_server/sl_btmesh_scene_server.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_scheduler_server/sl_btmesh_scheduler_server.o" "./gecko_sdk_4.0.1/app/bluetooth/common/btmesh_time_server/sl_btmesh_time_server.o" "./gecko_sdk_4.0.1/app/bluetooth/common/ota_dfu/sl_ota_dfu.o" "./gecko_sdk_4.0.1/app/bluetooth/common/simple_timer/sl_simple_timer.o" "./gecko_sdk_4.0.1/app/common/util/app_log/app_log.o" "./gecko_sdk_4.0.1/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_4.0.1/hardware/board/src/sl_board_init.o" "./gecko_sdk_4.0.1/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/sl_mx25_flash_shutdown.o" "./gecko_sdk_4.0.1/platform/Device/SiliconLabs/EFR32MG12P/Source/GCC/startup_efr32mg12p.o" "./gecko_sdk_4.0.1/platform/Device/SiliconLabs/EFR32MG12P/Source/system_efr32mg12p.o" "./gecko_sdk_4.0.1/platform/bootloader/api/btl_interface.o" "./gecko_sdk_4.0.1/platform/bootloader/api/btl_interface_storage.o" "./gecko_sdk_4.0.1/platform/bootloader/app_properties/app_properties.o" "./gecko_sdk_4.0.1/platform/common/src/sl_slist.o" "./gecko_sdk_4.0.1/platform/common/src/sl_string.o" "./gecko_sdk_4.0.1/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_4.0.1/platform/driver/button/src/sl_button.o" "./gecko_sdk_4.0.1/platform/driver/button/src/sl_simple_button.o" "./gecko_sdk_4.0.1/platform/driver/leddrv/src/sl_led.o" "./gecko_sdk_4.0.1/platform/driver/leddrv/src/sl_pwm_led.o" "./gecko_sdk_4.0.1/platform/driver/leddrv/src/sl_simple_rgbw_pwm_led.o" "./gecko_sdk_4.0.1/platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./gecko_sdk_4.0.1/platform/emdrv/nvm3/src/nvm3_default_common_linker.o" "./gecko_sdk_4.0.1/platform/emdrv/nvm3/src/nvm3_hal_flash.o" "./gecko_sdk_4.0.1/platform/emdrv/nvm3/src/nvm3_lock.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_acmp.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_adc.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_assert.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_cmu.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_core.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_cryotimer.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_crypto.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_csen.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_dbg.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_emu.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_gpcrc.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_gpio.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_i2c.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_idac.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_ldma.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_lesense.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_letimer.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_leuart.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_msc.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_opamp.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_pcnt.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_prs.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_rmu.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_rtcc.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_system.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_timer.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_usart.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_vdac.o" "./gecko_sdk_4.0.1/platform/emlib/src/em_wdog.o" "./gecko_sdk_4.0.1/platform/radio/rail_lib/plugin/pa-conversions/pa_conversions_efr32.o" "./gecko_sdk_4.0.1/platform/radio/rail_lib/plugin/pa-conversions/pa_curves_efr32.o" "./gecko_sdk_4.0.1/platform/radio/rail_lib/plugin/rail_util_pti/sl_rail_util_pti.o" "./gecko_sdk_4.0.1/platform/service/cli/src/sl_cli.o" "./gecko_sdk_4.0.1/platform/service/cli/src/sl_cli_arguments.o" "./gecko_sdk_4.0.1/platform/service/cli/src/sl_cli_command.o" "./gecko_sdk_4.0.1/platform/service/cli/src/sl_cli_input.o" "./gecko_sdk_4.0.1/platform/service/cli/src/sl_cli_io.o" "./gecko_sdk_4.0.1/platform/service/cli/src/sl_cli_tokenize.o" "./gecko_sdk_4.0.1/platform/service/device_init/src/sl_device_init_dcdc_s1.o" "./gecko_sdk_4.0.1/platform/service/device_init/src/sl_device_init_emu_s1.o" "./gecko_sdk_4.0.1/platform/service/device_init/src/sl_device_init_hfxo_s1.o" "./gecko_sdk_4.0.1/platform/service/device_init/src/sl_device_init_lfxo_s1.o" "./gecko_sdk_4.0.1/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_4.0.1/platform/service/iostream/src/sl_iostream.o" "./gecko_sdk_4.0.1/platform/service/iostream/src/sl_iostream_retarget_stdio.o" "./gecko_sdk_4.0.1/platform/service/iostream/src/sl_iostream_stdlib_config.o" "./gecko_sdk_4.0.1/platform/service/iostream/src/sl_iostream_uart.o" "./gecko_sdk_4.0.1/platform/service/iostream/src/sl_iostream_usart.o" "./gecko_sdk_4.0.1/platform/service/mpu/src/sl_mpu.o" "./gecko_sdk_4.0.1/platform/service/power_manager/src/sl_power_manager.o" "./gecko_sdk_4.0.1/platform/service/power_manager/src/sl_power_manager_debug.o" "./gecko_sdk_4.0.1/platform/service/power_manager/src/sl_power_manager_hal_s0_s1.o" "./gecko_sdk_4.0.1/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_4.0.1/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_4.0.1/platform/service/system/src/sl_system_init.o" "./gecko_sdk_4.0.1/platform/service/system/src/sl_system_process_action.o" "./gecko_sdk_4.0.1/platform/service/udelay/src/sl_udelay.o" "./gecko_sdk_4.0.1/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "./gecko_sdk_4.0.1/protocol/bluetooth/src/sl_bt_dynamic_gattdb_config.o" "./gecko_sdk_4.0.1/protocol/bluetooth/src/sl_bt_mbedtls_context.o" "./gecko_sdk_4.0.1/protocol/bluetooth/src/sl_btmesh_lib.o" "./gecko_sdk_4.0.1/protocol/bluetooth/src/sl_btmesh_sensor.o" "./gecko_sdk_4.0.1/protocol/bluetooth/src/sl_btmesh_serdeser.o" "./gecko_sdk_4.0.1/util/silicon_labs/silabs_core/memory_manager/sl_malloc.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/aes.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/bignum.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/ccm.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/cipher.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/cipher_wrap.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/cmac.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/ctr_drbg.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/ecdh.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/ecdsa.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/ecp.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/ecp_curves.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/entropy.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/entropy_poll.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/hmac_drbg.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/md.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/platform.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/platform_util.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/psa_crypto_client.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/sha1.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/sha256.o" "./gecko_sdk_4.0.1/util/third_party/crypto/mbedtls/library/threading.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/crypto_aes.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/crypto_ecp.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/mbedtls_ccm.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/mbedtls_cmac.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/mbedtls_sha.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/sl_entropy_hardware.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_mbedtls_support/src/sl_mbedtls.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_protocol_crypto/src/sli_protocol_crypto_crypto.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/crypto_management.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_aead.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_cipher.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_hash.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_crypto_transparent_driver_mac.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_crypto_trng_driver.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_psa_driver_common.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_psa_driver_init.o" "./gecko_sdk_4.0.1/util/third_party/crypto/sl_component/sl_psa_driver/src/sli_psa_trng.o" "C:/Users/Ander/SimplicityStudio/SDKs/gecko_sdk1//protocol/bluetooth/lib/EFR32MG12P/GCC/binapploader_nvm3.o" "C:/Users/Ander/SimplicityStudio/SDKs/gecko_sdk1//protocol/bluetooth/lib/EFR32MG12P/GCC/libbluetooth.a" "C:/Users/Ander/SimplicityStudio/SDKs/gecko_sdk1//protocol/bluetooth/lib/EFR32MG12P/GCC/libbluetooth_mesh.a" "C:/Users/Ander/SimplicityStudio/SDKs/gecko_sdk1//platform/emdrv/nvm3/lib/libnvm3_CM4_gcc.a" "C:/Users/Ander/SimplicityStudio/SDKs/gecko_sdk1//platform/radio/rail_lib/autogen/librail_release/librail_efr32xg12_gcc_release.a" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: soc_thunder.hex'
	arm-none-eabi-objcopy -O ihex "soc_thunder.axf" "soc_thunder.hex"
	@echo ' '

	@echo 'Building bin file: soc_thunder.bin'
	arm-none-eabi-objcopy -O binary "soc_thunder.axf" "soc_thunder.bin"
	@echo ' '

	@echo 'Building s37 file: soc_thunder.s37'
	arm-none-eabi-objcopy -O srec "soc_thunder.axf" "soc_thunder.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "soc_thunder.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) soc_thunder.axf
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
