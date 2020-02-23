# odroid-carpc-nano33
Derivate of odroid-carpc-due for the Arduino Nano 33 IoT
An Arduino Due project for controlling an Odroid N2 via BMW Keyfob and iDrive.

## Hardware used
- BMW 1-Series 2009
- Arduino Nano 33 IoT
- Custom board design to incoorporate MCU, Bluetooth and Canbus logic on the same board
- iDrive 7-Button Controller 
- DS18B20 Temperature Probe
- LTC3780 Power Supply Module

## Purpose
This project is intended to support an Odroid N2 in its function as a car-pc. This also involves controlling the display brightness of the Vu7+ display by applying PWM signals to its backlight chip by reacting on the cars' light sensor on the windscreen.
As a bonus it's designed to support an iDrive controller as an input device for the Odroid to have comfortable access to the car-pc functions. To make it function properly an app called "Button Mapper" is used to react on F-Keys.
See: https://play.google.com/store/apps/details?id=flar2.homebutton

## 3rd Party Libraries used
I'm using a modified version of the BPLib that now supports sending and holding multiple keys. That's probably important for future tweaks. The original library only supports sending one single key and modifiers.
See: https://github.com/Neuroquila-n8fall/BPLib

Also: Seeedstudios Canbus Shield library
See: https://github.com/Seeed-Studio/CAN_BUS_Shield

## Usage
### Intended Usage & prerequisites
The main key aspect of the layout is to connect an external power supply which is supplied by the battery of the car through the board. The intention behind this is to keep everything safely connected. Cars are very flammable and therefore the top priority has to be that there is no risk of loose, uninsulated wires dangling around!

A DS18B20 temperature probe is required in order to keep track of the temperatures of the pc and control the fans accordingly.

### Wiring
This new revision has everything on board which makes wiring very easy. It also minimizes the amount of wiring.
![Alt Text](https://github.com/Neuroquila-n8fall/odroid-carpc-nano33/blob/master/EagleFiles/CarPC-CC-Rev3_Wiring.png)
All of the terminals have a Description on the bottom which usually is hidden by the terminal itself. That's why a more descriptive text is added to the top.
You'll find a Terminal for the following auxillary devices:
- Display (VU7A+): One Output for controlling the background lighting nd one for triggering he power button
- CAN-Bus: You can hook up your K-Can to the screw terminal or the pin headers behind.
- PC: This is the output for the Odroid. It receives power directly from the 12V terminal
- 12V: Power supply input from an external regulated power supply. I heavily recommend a supply board with the LTC3780 chip.
- IGN: Ignition. Two terminals are available because we only need one positive input. In case you want to hook up something on the ignition, this is the right place
- IGN-AP: Ignition output for Access Points. I'm using the Teltonika RUT850 which needs this supply in order to power up and down on itself
- BAT: Battery Input. Used by the control logic to check for voltage levels and also delivers power to the DC Supply
- PSU (BOUT): Battery Out. Intended to be used as Output to the power supply. This Terminal is switched by the "1/0" pin header between"IGN-AP2 and "BAT"
- BAT-AP: Used by the Teltonika access point as permanent supply

#### Color blocks explained
- Green: Shared 12v Rail
- Orange/Yellow: Shared Ignition Rail
- Purple: Shared Battery Rail

### Parameters
Parameters are described inside the source file "main.h" (https://github.com/Neuroquila-n8fall/odroid-carpc-nano33/blob/master/src/main.h)
Everything is well tested and works right out of the box. The prototype is already reaching one year of service on my car and shares the same code base.

## Known Issues
- As soon as the car battery is depleted, the arduino will write value 0 to the display brightness pin when the engine starts or doors are fully opened and closed. I have yet to understand why on earth this is happening even when the engine is already running. I can only think of a solution where the brightness is updated frequently, like 10 seconds or so.
- When the car is not shut and goes into hibernation and the car pc is still running, it might happen that the display goes dark. This might also be related to the aforementioned problem and may point to an integer overflow issue on the brightness calculation. Maybe the sensor on the windshield (RLS Control Module) sends some crazy data when it goes into hibernation.

## Questions I got asked...
Why all the hassle if I could just simply upgrade the car to the official iDrive?
Because I always felt the need to include a full blown car pc into my car. Upgrading to iDrive wasn't an option due to the fact that this system is just too limited. If I had a recent model, this wouldn't be a problem but sitting on a rather "old" car leaves one without options. I want a browsable music library with cloud sync, internet, the precious Torque app, ...all the good stuff!
The icing on the cake was the decision to go the extra mile and swap the center console and install an iDrive 7-Button control unit. It looks like OEM but it does much, much more.

