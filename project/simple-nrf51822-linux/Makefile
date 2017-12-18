CC          := /home/c_epbori/BLE/tools/gcc-arm-none-eabi-6-2017-q2-update/bin/arm-none-eabi-gcc
OBJCOPY     := /home/c_epbori/BLE/tools/gcc-arm-none-eabi-6-2017-q2-update/bin/arm-none-eabi-objcopy
NRF51_SDK   := /home/c_epbori/BLE/tools/nRF51_SDK
NRF51_OBJ   := /home/c_epbori/BLE/tools/nRF51_SDK/examples/ble_peripheral/ble_app_hrs/pca10028/s110/armgcc/_build

default: main.bin main.hex

main.o: main.c
	$(CC) -mcpu=cortex-m0 -mthumb -DBOARD_PCA10001 -DNRF51 -I$(NRF51_SDK)/components/ble/common -I$(NRF51_SDK)/components/ble/ble_services/ble_hrs -I$(NRF51_SDK)/components/ble/ble_services/ble_bas -I$(NRF51_SDK)/components/libraries/util -I$(NRF51_SDK)/components/softdevice/s110/headers  -I$(NRF51_SDK)/components/softdevice/common/softdevice_handler -I$(NRF51_SDK)/components/drivers_nrf/hal  -I$(NRF51_SDK)/components/drivers_nrf/delay  -I$(NRF51_SDK)/components/toolchain  -I$(NRF51_SDK)/components/device -I$(NRF51_SDK)/components/toolchain/gcc -c $<


main.out: main.o
	$(CC) -L"/home/c_epbori/BLE/tools/nRF51_SDK/examples/ble_peripheral/ble_app_hrs/pca10028/s110/armgcc/_build" -L"/home/c_epbori/BLE/tools/gcc-arm-none-eabi-6-2017-q2-update/lib/armv6-m" -L"/home/c_epbori/BLE/tools/gcc-arm-none-eabi-6-2017-q2-update/lib/gcc/arm-none-eabi/4.7.2/armv6-m" -Xlinker -Map=main.map -mcpu=cortex-m0 -mthumb -mabi=aapcs  -L$(NRF51_SDK)/components/toolchain/gcc/ -T$(NRF51_SDK)/examples/ble_peripheral/ble_app_hrs/pca10028/s110/armgcc/ble_app_hrs_gcc_nrf51.ld main.o $(NRF51_OBJ)/*.o -o main.out

main.bin: main.out
	$(OBJCOPY) -O binary main.out main.bin

main.hex: main.out
	$(OBJCOPY) -O ihex main.out main.hex

install: main.bin
	sed  's#\[\[--filename--\]\]#$(PWD)/main.bin#' segger/burn-template.seg > burn.seg
	./segger/segger.sh $(PWD)/burn.seg

clean:
	rm *.o *.out *.hex *.seg *.map *.bin *.hex
