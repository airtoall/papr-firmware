# Firmware for the Air-To-All PAPR One 

This directory contains the firmware that runs the Air-To-All PAPR version 3.

The firmware runs on the ATMega328p microcontroller chip (the "MCU") that is on the PAPR's PCB. Using the various pin inputs and outputs of the MCU, the firmware monitors and controls all the active components of the PAPR, including the buttons, LEDs, fan, buzzer, battery and charger. The firmware also has acceess to a serial port for text input/output for testing and debugging.

This is an Arduino-compatible app, written in C++. The app doesn't run on any actual arduino board, but we use the Arduino IDE and runtime as the base for this app so that we can take advantage of the Arduino APIs, Arduino libraries, and the Arduino community (forums, blogs, etc). If it became necessary to eliminate any Arduino dependencies, I think you could do it with a day or two of work.

## Dev environment setup

To compile or develop this code:
1. get a Windows 10 machine. If you have an Intel-based macintosh, you can use Bootcamp and the free version of Windows 10. (The free Windows 10 is simply Windows 10 that has not been activated with a serial number; it works fine, but certain features are not available and there is an inconspicuous watermark on your screen.)
2. install Visual Studio 2019. The free "Community" edition works fine. Or you can use a paid version if you have it.
3. in Visual Studio, install the Visual Micro extension by following [these instructions](https://www.visualmicro.com). This extension gives the ability to create, build, and configure arduino-compatible projects. The free version expires after a month or two, but it only costs a few dollars for a full license.
4. get a programmer - this is a device that connects to your computer and to the PAPR PCB, and lets you download firmware and data into the MCU's memory. I am using the AVR ISP MK II programmer, which is no longer made by Atmel, but is available from other manufacturers, for example [this one](https://www.amazon.com/waveshare-Compatible-AVRISP-USB-XPII/dp/B00ID98C5K/ref=sr_1_2_sspa?dchild=1&keywords=atmel+avr+isp+mkii&qid=1624736601&sr=8-2-spons&psc=1&smid=A2SA28G0M1VPHD&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUFLUzdNVzRJS1NYVTAmZW5jcnlwdGVkSWQ9QTA1NDQ1NzEzMTU1TkRIUkhMWDhWJmVuY3J5cHRlZEFkSWQ9QTAyMTEwOTIzS0M3Wk5ZMkE1RThYJndpZGdldE5hbWU9c3BfYXRmJmFjdGlvbj1jbGlja1JlZGlyZWN0JmRvTm90TG9nQ2xpY2s9dHJ1ZQ==).
5. download and install [Zadig](https://zadig.akeo.ie/), a Windows application that manages USB drivers. You need this to set up the driver for the AVRISPMKII programmer.
7. connect the AVRISPMKII to your computer, then use Zadig to set the AVRISPMKII's driver. 
    - in Zadig, do Options > List All Devices
    - select AVRISPMKII from the device menu
    - if it doesn't say "driver libusb" then update the driver to libusb-win32.
9. Use Device Manager to verify that Windows 10 recognizes the AVRISPMKII. It might appear under "Microchip Tools", or it might appear in some other category. You sometimes have to dance around a bit to get things working. If Device Manager and/or zadig cannot see the AVRISPMKII, then try rebooting your computer and/or unplugging and plugging in the USB connector.

### Working with the code

In visual studio, open the solution file "Product.sln".

To compile the code, use `Build > Build Solution`.

To download and run the code:
1. in `Build > Configuration Manager...`, make sure Active Solution Configuration is `Release`
1. make sure the PCB has power, either from the battery connector, or the charger connector.
1. connect the AVRISPMKII's USB cable to your computer
1. connect the AVRISPMKII's 6-pin SPI connector to the PCB's 6-pin SPI header
1. use `Debug > Start Debugging`, to compile and download. (Because the configuration is `Release`, this does not actually start a debug session, it just runs the code).

If the PCB's power is coming from the battery connector, you must disconnect the 6-pin SPI after downloading, in order to run the board. If power is coming from the charger connector, you can leave the SPI connected.

If you want to create a new project to run on the PAPR's MCU (for example some new kind of test code, or some experimental code):
1. in Visual Studio, do `File > New > Project`, or select `Create a New Project` from Visual Studio's start page
2. choose `Arduino Empty Project` and click `Next`
3. choose a project name and location, choose `Create New Solution`, and click `Create`
4. Do `Extensions > vMicro` and set IDE to `Arduino 1.6/1.8`
5. Do `Extensions > vMicro` and set Board to `ATMega328P (Old Bootloader) (Arduino Nano)`
6. Do `Extensions > vMicro > Uploader` and set Hardware Programmer to `AVRISP mkII`
7. Do `Extensions > vMicro > Add Code > Add Local Board.txt`
8. Edit the new file `board.txt` and add the line `build.f_cpu=8000000L`
9. Do `Build > Configuration Manager...` and set Active Solution Configuration to `Release`
11. Optional: add to your new project a copy of Hardware.h, Hardware.cpp, MySerial.h, and MySerial.cpp. Use the initialization code and pin defintions in Hardware to help you run the board correctly.

### Setting up `avrdude`

`avrdude` is a command-line app that can read/write firmware and configuration settings to the MCU.

To install avrdude, download and install the latest [avrdude](https://www.nongnu.org/avrdude/). I use version 6.3. I recommend unzipping into a new folder `c:\avrdude`. There are additional docs [here](https://www.ladyada.net/learn/avr/avrdude.html).

Each time you want to use `avrdude`, do this...
1. make sure the PCB has power, either from the battery connector, or the charger connector.
1. connect the AVRISPMKII's USB cable to your computer
1. connect the AVRISPMKII's 6-pin SPI connector to the PCB's 6-pin SPI header
2. run `Command Prompt`
3. `cd c:\avrdude`

Sometimes avrdude will fail because the MCU is running at a speed that is too low. You may be able to fix this by appending ` -b22` to your avrdude command line.

### Setting up a new MCU

If you have a PCB whose microcontroller has never been set up, you must first program the MCU's "fuse bytes" to the correct values:
1. copy the .hex files from this repo's `Product/binaries` folder into your avrdude install folder `c:\avrdude`.
4. `avrdude -c avrispmkII -p m328p -U lfuse:w:lfusefile-PAPR.hex:i`
4. `avrdude -c avrispmkII -p m328p -U hfuse:w:hfusefile-PAPR.hex:i`
4. `avrdude -c avrispmkII -p m328p -U efuse:w:efusefile-PAPR.hex:i`

This will set the fuse bytes to:
- low byte = 0x72
- high byte = 0xDA
- extended byte = 0xFF

These fuse bytes configure the MCU clock to 8 MHz using the internal oscillator (no crystal required), and set the initial clock divider to 8, which results in a clock speed of 1 MHz. For more details on the fuse bytes, see the ATMega328p datasheet.

### Downloading firmware 

When you are in Visual Studio, you can download firmware to the MCU using `Debug > Start Debugging`, as described above.

To download firmware from Command Prompt, do this:
1. use visual studio to build the firmware. This creates a file called `Product.ino.hex` in the `Product/Release` folder.
3. copy `Product.ino.hex` to your `c:\avrdude` folder.
4. OR choose an archived build `Product.ino.yyyymmdd.hex` from `Product/binaries`, and copy that to the avrdude folder.
5. `avrdude -c avrispmkII -p m328p "-Uflash:w:Product.ino.yyyymmdd.hex:i"`

Once firmware is installed, it will immediately start running. To make it run properly, you may need to disconnect the 6-pin programming connector.

### Writing to the serial port

The serial port pins PD0 and PD1 are exposed via the PCB's serial header. To access the serial port from your computer:
- if there is no serial port header, then make a header by soldering a couple of pins to the serial port's GND and TXD terminals
- buy a [FTDI TTL-to-USB Serial adapter](https://www.amazon.com/gp/product/B07BBPX8B8/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1)
- connect RXD on the adapter to TXD on the PCB's serial port header, and connect GND on the adapter to GND on the header.
- connect the adapter's USB to your computer
- run a terminal app on your computer. I like "[Termite](https://termite.software.informer.com/3.4/)", but there are many other choices.
- set the terminal app to baud 57600, 8 data bits, 1 stop bit, no parity, no flow control

In the firmware, use MySerial.h and MySerial.cpp to write to the serial port. If you prefer, you can use Arduino's `Serial` API directly.

### Misc notes

This project uses the Arduino library "Low-Power 1.6". To ensure repeatable builds, we keep a copy of this library in the visual studio project, in the "Libraries" folder. (FYI, in Visual Micro, you can add more libraries `Extensions > vMicro > Add Library` with the *Clone For Solution* option).

When the PCB is powered up, it is in a low power mode that requires the MCU to be at a low speed. This is why the fuse bytes are configured to set the MCU's clock speed to 1 MHz. The firmware will set the board to full power mode and bump up the clock speed after it initializes.

The file `board.txt` is used by the compiler to define certain settings. For this project there is only one setting - the processor speed. By default, the compiler assumes a speed of 16 MHz, however we use a speed of 8 MHz. (FYI, there are several reasons for using a lower speed: 8 MHz doesn't require a crystal so we save a little bit of cost and we free up 2 pins on the MCU, the lower speed consumes less power, and our work is quite simple so we don't need a higher speed.)

Unit testing: at present there is no automated testing of this code. However, the code is broken down into C++ classes in a way that would make it somewhat easy to add unit testing. The repo [markshepherd/papr-firmware](https://github.com/markshepherd/papr-firmware) has a branch "unittest" that contains some very-out-of-date experimental code for this.

Debugger: I have not succeeded in using a debugger on the PAPR. If you figure out how to do it, please update this Readme with instructions.

### Development practices

I haven't included any best practices guidelines for coding because there are a million web sites that talk about this (for example [this one](https://www.thinkful.com/blog/software-engineering-best-practices/)). The PAPR is a medical device and we need the code to be professional, high-quality, maintainable, and reliable. Please follow the naming, formatting, and commenting practices that you see in the code. Make sure your code is as clean and simple as possible. Make sure that other developers will be able to read and understand your code. Make sure you test your code thoroughly - once the product is delivered to the customer it is extremely difficult to fix bugs or update the code. Run the full [Functional Core Verification Test Suite](https://docs.google.com/document/d/1ubjNnj6kYDCdJMwp07bt-jukzoTlcU7nhaN4yKzK2nk/) at least once before shipping a new release. If you find errors or omissions in this Readme, please update it to make life easier for the next developer. If you add new features or change the behavior in any way, make sure to update, where applicable, the Specification, the Verification test suite (see links below), this readme file, and/or the comments in the code.

### Primary documents

These are the "official" documents describing the functionality of the PAPR product firmware, how to use it, how to test it.

[Functional Core Specification](https://docs.google.com/document/d/1O3QTVKkepRBzme7QUEG_r9uPpGkaOA27uIlCKO2OQU8/edit) - detailed description of everything the firmware does. 

[Functional Core Verification Test Suite](https://docs.google.com/document/d/1ubjNnj6kYDCdJMwp07bt-jukzoTlcU7nhaN4yKzK2nk/edit) - a detailed set of tests that fully exercise the firmware. Run this entire suite before shipping a new version of the firmware.

[PCB Production Tests](https://github.com/airtoall/papr-firmware/blob/master/Product/Docs/Production%20Test.pdf] - A suite of tests to be performed on the PAPR PCB after a PCB is assembled but before conformal coating and installation into a fully assembled PAPR unit. The tests are provided by the firmware's Test mode, 

[PCB Validation Test Suite](https://docs.google.com/document/d/1M6ftVc30-DXrsPZTG73OsVQJnZh4xsFBa8cYp-f8Py0/) A test suite to be performed by factory personnel on every newly-manufactured PAPR, to confirm that the PCB is fully working and the peripheral devices (fan, battery, charger, buzzer, buttons, lighs) are properly connected. The tests use various features of the product firmware.

### Reference Documents

Here are documents for other system components that the firmware innteracts with.

[PCB Schematic](https://github.com/airtoall/papr-firmware/blob/master/Product/Docs/PAPR%20schematic%20v3.1.pdf) - The latest schematic for the PCB.

[ATMega328p Info](https://www.microchip.com/wwwproducts/en/ATmega328P) - Info on the ATMega328p MCU

[Fan Info](https://www.digikey.com/en/products/detail/sanyo-denki-america-inc/9GA0412P3K011/6192261) - Info on the 9GA0412P3K011 fan

[Buzzer Info][???) - Info on the XXXX buzzer. (TBD)

[PAPR battery discharge 6-4-21](https://docs.google.com/spreadsheets/d/14-mchRN22HC6OSyAcN329NEcRRjF2_VMbKz3yHDDEoI/edit#gid=1527307635) - measurements of battery voltage and current when the battery is being discharged. This information helps understand the behavior of the battery, and is the basis for some of the magic constants that appear in the firmware.

[PAPR battery charge 6-5-21](https://docs.google.com/spreadsheets/d/1fPnn2ukakk8MpyGW_KrOW2ediHh8FU6yWr4Kfq2UNJs/edit#gid=13224763) - ditto for battery charging.

### Other Documents

You can find additional miscellanous documents (notes, experiments, measurement) [https://github.com/airtoall/papr-firmware/tree/master/Product/Docs](here) and [https://drive.google.com/drive/folders/1pIbtwiE1PP5SLPMRfh0A3RLS38V0ZmLC](here). Caveat: some of the documents may be out of date.





