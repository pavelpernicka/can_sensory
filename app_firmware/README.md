# App Firmware

- MCU: `STM32L432xx`
- CAN ID is get from bootloader

## Build

```bash
cd app_firmware
make -j4
```

## Flash

```bash
cd app_firmware
make flash_openocd
```
Or using bootloader CLI tool:
```bash
cd app_firmware
./app_can_tool.py --interface socketcan --channel slcan0 --device-id 3 enter-bootloader
cd ..
./bl_can_tool.py --interface socketcan --channel slcan0 --device-id 3 update app_firmware/app_firmware.bin --boot
```

When flashing manually, make sure to enter right address `0x08004000`. 



## Python App Tool

```bash
# Ping
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 ping

# Read status summary
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 status

# Configure stream intervals
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 set-interval --stream mag --ms 100
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 get-interval --stream all
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 set-interval --stream mag --ms 100 --save

# Enable / disable streams
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 stream-enable --stream env --off
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 stream-enable --stream env --on
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 stream-enable --stream env --on --save

# Reset into bootloader
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 enter-bootloader

# HMC5883L configuration
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 hmc-config
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 hmc-config --range 8.1 --save

# AHT20 read
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 aht20-read

# Calibration
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-get --field all
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-set --field rotate_xy --value 150
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-capture-earth
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-save
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 calib-load

# Monitor mode
python3 app_can_tool.py --interface socketcan --channel slcan0 --device-id 1 monitor

# GTK visualizer
python3 app_calib_gtk.py --interface socketcan --channel slcan0 --device-id 1
```
