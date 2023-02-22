# CRS Wi-Fi Car Firmware

The CRS platform has car models that feature custom electronics. This repository contains the firmware that runs on them. It is a C/C++ project based on the [ESP-IDF development framework](https://github.com/espressif/esp-idf/) for the ESP32 microcontroller family.

## Getting Started

### Deploying the firmware

Normally, the cars at CRS lab already are flashed with working firmware. However, if a new PCB has not yet had the firmware installed, one can manually flash the microcontroller without the need to download the toolchain.

1. Clone the [`esptool.py` repository](https://github.com/espressif/esptool) to your local machine.

2. Obtain the binary files (`bootloader.bin`, `partition-table.bin` and `car-firmware.bin`), either from the newest release (see [Wiki](https://gitlab.ethz.ch/ics/crs/-/wikis/crs-for-cars/hardware-manual#firmware)), or message one of the developers.

3. Unplug the ribbon cable from the car's PCB, if it is still attached.

4. Connect the ESP-PROG board to your computer using a USB to microUSB cable.

5. Connect the ESP-PROG board to the PCB using the small PROG ribbon cable.

6. Open a terminal on your computer and navigate to the location where the binary files are stored.

7. Find out which serial port was opened from the PROG board (`ls /dev/tty*`)

8. Run the following command, replacing `(PORT)` with the port number you found in the previous step (e.g., `dev/tty.usbserial1000`).
   ```bash
   python esptool.py -p (PORT) 460800 -before default_reset --after hard_reset --chip esp32 write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 ./bootloader.bin 0x8000 ./partition-table.bin 0x10000 ./car-firmware.bin
   ```

9. Wait for the flashing process to complete. Remove all cables and re-connect PCB to ribbon cable.

### Development

If you would like to develop for this firmware, you will need to install the toolchains. For working on the code, both Docker and native platforms are supported.

For more information on installation and prerequisites for development, see [the CONTRIBUTING file](./CONTRIBUTING.md).

## Documentation

The code is documented within the source files themselves. If you would like a browsable version of the documentation, install and run [Doxygen](https://www.doxygen.nl/).

`````shell
cd crs-wifi-car-firmware
doxygen
`````

The documentation will be generated in the `docs/` folder.

## Code Overview

The ESP-IDF toolchain has a CMake-based build system and allows separating out different parts of the code into separate components with dependencies. The entire firmware of the car can be found in `src/`. If you're looking for a specific part of the code, here's a quick overview of what's located where:

* `src/`: root folder of the firmware
  * `main/`: main component, containing the entry point (`app_main()`)
  * `communication`: high-level communication (transport and application layer)
  * `configuration`: persistence of configuration settings and the configuration server
  * `control`: control loop and different controllers for low-level input quantities
  * `hardware`: drivers that interact with hardware components, i.e. actuators, sensors or Wi-Fi
  * `state`: state machine that tracks the high-level operational states of the vehicle

## Developers

[Lukas Vogel](mailto:vogellu@ethz.ch)
