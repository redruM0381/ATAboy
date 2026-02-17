# PLEASE NOTE!
Most of this "proof of concept" firmware code was written with AI assistance.  It's a mess.
It currently contains IPC re-entry, and other bugs that could cause crashes under certain circumtstances.
It does function for the most part, but can be buggy and slow.
It is also incomplete, and some functionality is not yet enabled (e.g. IORDY/IRQ pins, etc)
# Use at your own risk!

* Firmware written in VS Code with Raspberry Pi Pico Extension
* PCB designed in Kicad
* Case designed in Freecad

-----------------------------------------------------------------------------------

![image](https://github.com/redruM0381/ATAboy/blob/main/logo.png)


ATAboy is designed for retro computing enthusiasts, data recovery experts, and archivists.  Read and write the oldest of IDE hard drives, without the need for an "in-between" vintage computer!

ATAboy is a user-friendly bridge that allows classic IDE (PATA) hard drives to be connected to a modern computer over USB as a standard USB Mass Storage device.  While cheap, modern adapters usually only work with newer "LBA" type drives, ATAboy works all the way back to the earliest CHS only, PIO Mode 0, ATA disks.  (It will also work with newer LBA type IDE disks, but that's not really what it's designed for.)  ATAboy features an "Award BIOS inspired" user friendly UI for drive configuration and setup.

---

When connected:

* The IDE drive appears to your computer like a normal USB Mass Storage Device
* No special drivers are required (Windows, Linux, macOS supported)
* The device translates USB Mass Storage commands into IDE read/write operations

---

## Hardware Requirements

To get started, you will also need:

* Any 40‑pin IDE (PATA) hard drive with jumpers set for MASTER/SINGLE only!
* An external PSU to power the drive.
* A USB-C cable to connect ATAboy to the host computer.
* Any standard ANSI compatible serial terminal emulator (e.g. PuTTY, Tera Term, minicom, screen, etc.)

---

## Basic drive mount using ATABoy

1. Connect the IDE ribbon cable to the drive and ATAboy (Note Pin 1 orientation, usually marked by a stripe on the cable)
2. Power the drive using external supply
3. Connect ATAboy to your computer via USB
4. Connect to serial console with terminal of choice (PuTTY shown)
5. Enter the "Auto Detect & Set Geometry" menu
6. Choose default, or enter geometry manually if needed
7. Choose "Mount HDD to USB Mass Storage"
8. That's it!  You can now read the drive from your modern OS!

---

## Serial Console Access

![image](https://github.com/redruM0381/ATAboy/blob/main/putty.png)

ATAboy exposes a USB serial console UI used to configure the drive, change system settings, mount the drive, etc.
After connecting ATAboy via USB, first find the new serial port:

Windows:
Open Device Manager → Ports (COM & LPT) and look for a new USB Serial Device (COMx).

Linux:
The device will typically appear as /dev/ttyACM0 or /dev/ttyUSB0.
You can list ports with:
ls /dev/ttyACM* /dev/ttyUSB*

macOS:
Look for a device named like /dev/tty.usbmodemXXXX or /dev/tty.usbserialXXXX.

Once we know the serial port, Connect to it with a Terminal Emulator with settings: 9600 8N1, no flow control.
You should see an "Award BIOS inspired" text-based UI appear in the terminal.
From here, it is mostly self explanitory.  Enter the auto-detection and geometry menu to set the drive's geometry as it was originally formatted (this may require experimentation if you don't know).
Once geometry is set, you may mount the drive, and it should appear in your host OS.
If you like, you can save the current settings and geometry to EEPROM, which will be restored the next time the device is connected.  (This allows auto-mounting also.)

IMPORTANT!  If you are working with valuable or irreplaceable data, enable write protection before mounting.  It is enabled by default for safety.  You do NOT want the host OS to touch or bless the disk!
Windows VDS/RPC services can often crash if it is unable to write to a drive with damaged/legacy MBR and/or partition tables.
This can result in various Windows errors, unable to mount the drive, etc.  However, raw data can usually still be accessed via dd, or other raw disk backup software.
Regardless, it is STRONGLY recommended the entire drive be dumped to an image file BEFORE disabling write protection, and mounting the drive in Explorer.

---

## Safe Removal

Always use your operating system’s **Safely Remove / Eject** feature before:

* Unplugging the USB cable
* Powering off the IDE drive

This helps prevent:

* File system corruption
* Incomplete writes
* OS-side errors

---

## Known Limitations

ATAboy is a hobbyist and experimental project. You should be aware that:

* Performance is significantly slower than modern USB to IDE bridges.  We're working over USB 1.1 for now.  This can cause long mount times, especially in Windows if the disk has file system weirdness.
* Many legacy disks use unusual geometry, and may require experimentation.
* Not all OS disk utilities behave the same way

---

ATAboy is meant to be hacked, learned from, and improved.
Have fun, and thank you for trying ATAboy!



# Licensing

"ATAboy" is a trademark of JJ Dasher.
The open source licenses do not grant rights to use these trademarks.

## Firmware
Licensed under GNU GPLv3.
See /firmware/LICENSE.

## Hardware
Licensed under CERN-OHL-W v2.
See /PCB/LICENSE.














