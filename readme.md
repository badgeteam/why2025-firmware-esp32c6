# Why2025 Firmware for ESP32-C6

This repository contains the firmware for the Why2025 project running on the ESP32-C6.

## Quick Start

1. **Prepare the environment**

   ```sh
   make prepare
   ```

   This step prepares the environment for the build process. It only needs to be done once, though it can be repeated if necessary.

2. **Build, flash, and monitor**

   ```sh
   make build flash monitor
   ```

   This command will build the software, flash it onto the device, and start monitoring the device. Use this command for the main operations.

3. **Specify the port**

   ```sh
   make build flash monitor PORT=/dev/ttyACM1
   ```

   If you need to specify a different port, use this command. Replace `/dev/ttyACM1` with the appropriate port for your device.

   Exiting the monitor works the same as with Telnet, 'CTRL + \]' followed by 'quit \<enter\>'.

## Notes

- The `make prepare` command is time-consuming and typically only needs to be run once unless there are changes that require re-preparation.
- Use the port specification command if you need to communicate with a device on a specific port.

## Troubleshooting

If you encounter any issues, ensure that the port specified matches the port your device is connected to and that all necessary preparations have been completed.
