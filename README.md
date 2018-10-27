## Table of Contents

* [Introduction](#introduction)
* [Navigating the Repository](#navigating-the-repository)
* [Required Tools](#required-tools)
* [Code Examples List](#code-examples-list)
* [References](#references)

# Introduction
This repository contains examples and demos for PSoC 6 MCU family of devices, a single chip solution for the emerging IoT devices. PSoC 6 MCU bridges the gap between expensive, power hungry application processors and low‑performance microcontrollers (MCUs). The ultra‑low‑power, dual-core architecture of PSoC 6 MCU offers the processing performance needed by IoT devices, eliminating the tradeoffs between power and performance.

Cypress provides a wealth of data at [www.cypress.com](http://www.cypress.com/) to help you select the right PSoC device and effectively integrate it into your design. Visit our [PSoC 6 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m4-psoc-6) webpage to explore more about PSoC 6 MCU family of device.
Feel free to explore through the code example source files and let us innovate together!

# Navigating the Repository

Cypress provides PSoC 6 MCU based development kits for various family of PSoC 6 devices. These development kit helps you validate your prototype design before starting the actual design development. Refer to the Hardware section for the list of PSoC 6 development kits.

Each Development Kit is packaged with set of code examples which can get you started with PSoC 6 MCU as well as develop the hardware for your application by using the Kit hardware configuration as a reference. 
This repository contains all the kit related examples and the kit user guide. Refer to the respective kit user guide for the project implementation details.  

If you are new to developing projects with PSoC 6 MCU, we recommend you to refer the [PSoC 6 Getting Started GitHub](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Getting-Started) page which can help you familiarize with device features and guides you to create a simple PSoC 6 design with PSoC Creator IDE. For other block specific design please visit the following GitHub Pages:
#### 1. [Analog Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Analog-Designs)
#### 2. [Digital Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Digital-Designs)
#### 3. [BLE Connectivity Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-BLE-Connectivity-Designs)
#### 4. [Audio Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Audio-Designs)
#### 5. [Device Related Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Device-Related-Design)
#### 6. [System-Level Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-System-Level-Designs)
#### 7. [PSoC 6 MCU based RTOS Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-RTOS-Based-Design)

You can use these block level examples to guide you through the development of a system-level design using PSoC 6 MCU. All the code examples in this repository comes with well documented design guidelines to help you understand the design and how to develop it. The code examples and their associated documentation are in the Code Example folder in the repository.

# Required Tools

## Software
### Integrated Development Environment (IDE)
To use the code examples in this repository, please download and install
[PSoC Creator](http://www.cypress.com/products/psoc-creator)

## Hardware
### PSoC 6 MCU Development Kits
* [CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/psoc-6-ble-pioneer-kit).

* [CY8CKIT-062-WiFi-BT PSoC 6 WiFi-BT Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/psoc-6-wifi-bt-pioneer-kit). 

**Note** Please refer to the code example documentation for selecting the appropriate kit for testing the project

## Code Examples List
### CY8CKIT-062-BLE Pioneer Kit
#### 1. CE218133 - PSoC 6 MCU E-INK Display with CapSense
This code example shows how to create a user-interface solution using an E-INK display and CapSense.
#### 2. CE218135 – PSoC 6 MCU with BLE Connectivity: BLE with Proximity
This code example demonstrates connectivity between the PSoC 6 MCU with Bluetooth Low Energy (BLE) and CySmart
BLE host emulation tool or mobile device running the CySmart mobile application, to transfer CapSense proximity sensing
information.
#### 3. CE220167 – PSoC 6 MCU with BLE Connectivity: BLE with User Interface
This code example demonstrates interfacing PSoC 6 MCU with BLE Connectivity (PSoC 6 MCU) with user interface functions
such as an E-INK display, RGB LED, and touch sensors based on self and mutual capacitance (CapSense CSD and CSX).
These functions provide bi-directional BLE connectivity between the PSoC 6 MCU and a PC running the CySmart™ BLE Host
Emulation tool or a mobile device running the CySmart mobile application.
#### 4. CE220186 – PSoC 6 MCU with BLE Connectivity: RTC with Current Time Service
This code example demonstrates accurate time keeping with PSoC 6 MCU’s real-time clock (RTC), which is synchronized with
a current time server such as an iPhone using the BLE current time service (CTS).
#### 5. CE220272 - PSoC 6 MCU with BLE Connectivity: Direct Test Mode
This code example demonstrates Direct Test Mode (DTM) over the Host Controller Interface (HCI) using PSoC
6 MCU with Bluetooth Low Energy (BLE) Connectivity.
#### 6. CE220335 – PSoC 6 MCU with BLE Connectivity: Eddystone Beacon
This code example demonstrates a Bluetooth Low Energy (BLE) beacon that broadcasts the core frame types (UID, URL, and
TLM) of Google’s Eddystone beacon profile.
#### 7. CE220675 – PSoC 6 MCU: Motion Sensor
This code example demonstrates how to interface a PSoC 6 MCU with a BMI160 motion sensor. This example reads steps
counted by the sensor to emulate a pedometer. Raw motion data is also read and used to estimate the orientation of the board.
#### 8. CE222046 – PSoC 6 BLE Throughput Measurement
This code example demonstrates how to maximize the BLE throughput on PSoC 6 MCU with Bluetooth Low Energy (BLE)
Connectivity device.
#### 9. CE220567 – PSoC 6 MCU with BLE Connectivity: BLE Thermometer
This code example demonstrates interfacing PSoC 6 MCU with a thermistor circuit to read temperature information and sending
the data over Bluetooth Low Energy Health Thermometer Service (HTS) to a mobile device running CySmart mobile
application.


## References
#### 1. PSoC 6 MCU
PSoC 6 bridges the gap between expensive, power hungry application processors and low‑performance microcontrollers (MCUs). The ultra‑low‑power PSoC 6 MCU architecture offers the processing performance needed by IoT devices, eliminating the tradeoffs between power and performance. The PSoC 6 MCU contains a dual‑core architecture, with both cores on a single chip. It has an Arm® Cortex®‑M4 for high‑performance tasks, and an Arm® Cortex®‑M0+ for low-power tasks, and with security built-in, your IoT system is protected.
To learn more on the device, please visit our [PSoC 6 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m4-psoc-6) webpage.

####  2. PSoC 6 MCU Learning resource list
##### 2.1 PSoC 6 MCU Datasheets
Device datasheets list the features and electrical specifications of PSoC 6 families of devices: [PSoC 6 MCU Datasheets](http://www.cypress.com/search/all?f%5B0%5D=meta_type%3Atechnical_documents&f%5B1%5D=resource_meta_type%3A575&f%5B2%5D=field_related_products%3A114026)
##### 2.2 PSoC 6 MCU Application Notes
Application notes are available on the Cypress website to assist you with designing your PSoC application: [A list of PSoC 6 MCU ANs](http://www.cypress.com/psoc6an)
##### 2.3 PSoC 6 MCU Component Datasheets
PSoC Creator utilizes "components" as interfaces to functional Hardware (HW). Each component in PSoC Creator has an associated datasheet that describes the functionality, APIs, and electrical specifications for the HW. You can access component datasheets in PSoC Creator by right-clicking a component on the schematic page or by going through the component library listing. You can also access component datasheets from the Cypress website: [PSoC 6 Component Datasheets](http://www.cypress.com/documentation/component-datasheets)
##### 2.4 PSoC 6 MCU Technical Reference Manuals (TRM)
The TRM provides detailed descriptions of the internal architecture of PSoC 6 devices:[PSoC 6 MCU TRMs](http://www.cypress.com/psoc6trm)

## FAQ

### Technical Support
Need support for your design and development questions? Check out the [Cypress Developer Community 3.0](https://community.cypress.com/welcome).  

Interact with technical experts in the embedded design community and receive answers verified by Cypress' very best applications engineers. You'll also have access to robust technical documentation, active conversation threads, and rich multimedia content. 

You can also use the following support resources if you need quick assistance:
##### Self-help: [Technical Support](http://www.cypress.com/support)
##### Local Sales office locations: [Sales Office](http://www.cypress.com/about-us/sales-offices)
