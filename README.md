# Northern Mechatronics Application Template

This repository contains an application template that can be used as a starting
point for firmware development with any NM1801xx module from Northern Mechatronics Inc (NMI).

## Hardware Requirements

* Northern Mechatronics development board (NM180100EVB, NM18041x) *or* a user designed board with a NM1801xx target and an SWD port exposed
* USB cable (for NMI development boards with DAPLink onboard) *or* Segger J-Link 

## Software Requirements 

* Microsoft Visual Studio Code 
* Software prerequisites (see our [Getting Started](doc/getting_started.md) guide for details)


## Build the Application

For instructions on how to build this reference application and flash the binary to the NM1801XX, follow along step-by-step in the [Getting Started](doc/getting_started.md) guide.


## Application Description

This application is designed to be a base template for any application. All other NMI reference examples are built on top of this base, which uses NMSDK2.
The NMAPP2 default application demonstrates the use of FreeRTOS and toggles an LED every 500ms.

The application file `application_task.c` contains the following key application functions:

* `application_task` is the primary application entry point
* `application_task_setup` is similar to `setup` in Arduino, it runs at the beginning of the application
* `application_task_loop` is similar to `loop` in Arduino, it runs continuously after the application setup is complete

The code for blinking the LED is implemented inside `application_task_loop`. In this example it calls a FreeRTOS primitive `vTaskDelay` that blocks the task and yields for the specified delay.  

There are also functions in `application_task.c` for LED effects and button press handling that can be used in any application.

<img src="doc/res/app_setup_loop.png" width="700">

## Interacting with the Application

In addition to blinking an LED, this example also implements a command line interface (CLI) and 
button presses handling that controls LED visual effects.  Short pressing `BTN 0` cycles through LED visual effects.

### Command Line Interface
Users can access the CLI over the UART port *or* the SWD port using Segger RTT (as shown below).  

Only one port can be used at a time and it is specified by the second argument of `console_task_create` called in
`system_start` of `main.c`.  This example defaults to UART0.

If no text appears when using the UART port, press the RESET button and the
welcome text will be printed on boot.

<img src="doc/res/app_cli.png" width="700">

On the NMI development boards (NM180100EVB, NM18041X), users can access a virtual COM (VCOM) port over USB.  To access the CLI, use any serial terminal and connect to the COM port of the board.  The default terminal settings are 115200, 8N1.

*A note to DAPLink users:* 

Users of the Petal development board (NM18041x), or any other DAPLink-enabled development board, will experience a target reset whenever starting the VSCode Serial Monitor plugin. This occurs because the builtin VSCode terminal issues a SEND BREAK signal on connect.

This is a feature of DAPLink that allows a target board to be reset remotely for automated tests, and it is a requirement on Mbed OS test infrastructure.

To avoid this reset, consider using an alternate serial terminal software (such as Putty) which does not issue a SEND BREAK signal on connect.

### Button Press Handling

The button API defined in `button.h` allow users to set up a single button to register different press sequences and perform different tasks. 

In the NMAPP2 default application, a callback is registered to a single short press by calling `button_sequence_register`.  

To setup a button for press sequence handling, call `button_config` to configure the GPIO line that is connected to the button as shown in `setup_button` of `application_task.c`. 

A press sequence is represented by a bit pattern where zero represents a short press and one represents a long press.  

For example, to register a sequence of five presses starting with two short presses and then three long presses, the sequence register would look like:

```
button_sequence_register(button_handle, 5, 0b11100, callback);
```

### LED Control

The LED API defined in `led.h` allow users to execute different LED visual effects.  The LED must be connected to a GPIO with a CTIMER output.

Five effects are available, they are:

* breathing
* single pulse
* double pulse
* triple pulse
* SOS (3 short blinks, 3 long blinks, 3 short blinks)

Additional custom effects can be added by calling `led_register_effect`.

## Drag-and-Drop Firmware Update

The NM18041x Petal development board has an additional feature: mass storage device (MSD) emulation. 

When a NM18041x board is plugged into a host computer, a window will pop up as displayed below and the NMI device will be shown in the File Explorer, similar to a typical USB mass storage device.  Note that this is
not an actual storage medium.

With the pop up window open, drag-and-drop (or copy and paste) a new firmware image to the NMI device will
trigger a firmware update.

After successfully updating the firmware, the pop up window will close automatically. If there is an error during flashing, the window will pop up again with an error file.

<img src="doc/res/app_msd.png" width="700">

## Where to Next

[Explore NMI's reference applications](https://github.com/NorthernMechatronics) for further examples on setting up wireless communications, managing application priorities, sensor interfacing, and more!

