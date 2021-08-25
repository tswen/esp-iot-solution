# ESP32-S USB Dongle Solution

## 1.Overview

This example shows how to set up ESP32-S chip to work as a USB Dongle Device.

Supports the following functions:

* Support Host to surf the Internet wirelessly via USB-RNDIS
* Add BLE devices via USB-BTH, support scan, broadcast, connect and other functions
* Support Host to communicate and control ESP32-S series devices via USB-CDC or UART
* Support multiple system、Wi-Fi control commands
* Support hot swap

## 2.How to use example

### 2.1 Hardware Required

Any ESP boards that have USB-OTG supported.

* ESP32-S2

* ESP32-S3

### 2.2 Common Pin Assignments

Pin assignment is only needed for ESP chips that have an USB-OTG peripheral. If your board doesn't have a USB connector connected to the USB-OTG dedicated GPIOs, you may have to DIY a cable and connect **D+** and **D-** to the pins listed below.

```
ESP BOARD          USB CONNECTOR (type A)
                          --
                         | || VCC
[USBPHY_DM_NUM]  ------> | || D-
[USBPHY_DP_NUM]  ------> | || D+
                         | || GND
                          --
```

Refer to `soc/usb_pins.h` to find the real GPIO number of **USBPHY_DP_NUM** and **USBPHY_DM_NUM**.

|             | USB_DP | USB_DM |
| ----------- | ------ | ------ |
| ESP32-S2/S3 | GPIO20 | GPIO19 |

* ESP32-S2-Saola

<img src=".\_static\ESP32-S2.jpg" alt="ESP32-S2" style="zoom: 15%;" />

* ESP32-S3 DevKitC

<img src=".\_static\ESP32-S3.jpg" alt="ESP32-S3" style="zoom:25%;" />

### 2.3 Software Required

* Confirm that the ESP-IDF environment is successfully set up
* To add ESP-IDF environment variables, the Linux method is as follows. For other platforms, please refer to [Set up the environment variables](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-4-set-up-the-environment-variables)

    ```
    . $HOME/esp/esp-idf/export.sh
    ```

* Set the compilation target to `esp32s2` or `esp32s3`

    ```
    idf.py set-target esp32s2
    ```

### 2.4 Project Configuration

![tinyusb_config](./_static/tinyusb_config.png)

![uart_config](./_static/uart_config.png)

| RNDIS | BTH  | CDC  | UART |
| :---: | :--: | :--: | :--: |
|   √   |      |      |  √   |
|   √   |      |  √   |      |
|   √   |  √   |      |  √   |
|       |  √   |      |  √   |

* UART is disabled by default when CDC is enabled
* The project enables RNDIS and BTH by default, and works with UART for communication and control
* When RNDIS and BTH are enabled at the same time, no other USB Device can be turned on, otherwise an error will be reported during compilation

![error](./_static/error.png)

### 2.5 build & flash & monitor

```
idf.py -p (PORT) build flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type `Ctrl-]`.)



After the system is running, Linux will add the USB device, you can use the following command to view the USB device

```
ifconfig -a
```

<img src=".\_static\ifconfig.png" alt="ifconfig" style="zoom: 80%;" />

```
hciconfig
```

![hciconfig](./_static/hciconfig.png)

```
ls /dev/ttyACM*
```

![ACM](./_static/ACM.png)

You can communicate with the development board through the USB ACM port or UART.

view the currently supported commands and usage through the help command.

## 3.Way of distribution network

You can choose any of the following methods to configure the network

### [1. Use the `sta` command to directly connect to the corresponding AP](./Commands.md#3sta)

**Example**

```
sta -s <ssid> -p [<password>]
```

**Notes**

* `password` is optional

* When the USB Dongle network is switched, the network device needs to be reloaded

    >ifconfig                           //find USB Ethernet name
    >
    >ifconfig ethxxx down 
    >
    >ifconfig ethxxx up    

### [2. smartconfig network configuration](./Commands.md#5smartconfig)

(1) Hardware Required

Download ESPTOUCH APP from app store: [Android source code](https://github.com/EspressifApp/EsptouchForAndroid) [iOS source code](https://github.com/EspressifApp/EsptouchForIOS) is available.

(2) Make sure your phone connect to the target AP (2.4GHz).

(3) Open ESPTOUCH app and input password.

(4) Send commands via USB ACM port

**Example**

```
smartconfig 1
```

**Notes**

* When the USB Dongle network is switched, the network device needs to be reloaded

    >ifconfig                           //find USB Ethernet name
    >
    >ifconfig ethxxx down 
    >
    >ifconfig ethxxx up    

## 4.Command introduction

[Commands](./Commands.md)

Note: Wi-Fi commands can only be used when USB Network Class is enabled
