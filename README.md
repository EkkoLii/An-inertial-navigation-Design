# An-inertial-navigation-Design
## Table of contents
* [About](#about)
* [Requirements](#requirements)
* [Instruction book](#instruction-book)
* [Code structure](#code-structure)
* [License](#license)
* [Authors](#authors)
* [References](#references)
* [Acknowledgements](#acknowledgements)
* [Social media](#social-media)


## About
Inertial navigation is a navigation and positioning system based on the Inertial Measurement Unit to measure the acceleration of the carrier itself and then obtain its velocity and position by integration. It is used in a wide range of aircraft and aerospace applications.

The An-innertial-navigation-Design project is a two-dimensional navigation system developed with a Raspberry Pi and an inertial measurement unit. The project aims to help carriers locate themselves in areas where signals are weak, such as underground garages, unmanned areas, etc. The main compilation language for the development of the project is C++ and the operating system is Linux.

## Requirements
**Hardware:**
* Laptop with Ubuntu
* Raspberry Pi 4B
* IMU：LSM9DS1
* Monitors

**Circuit connections:**
* Connect the 3.3V power (pin 1) and GND (pin 9) to the LSM9DS1.
* Connect the I2C SDA (pin 3) & I2C SCL (pin 5) to the LSM9DS1.
* Connect the GPIO22 (pin 15) to the INT2 output of the LSM9DS1.

**Software:**
* X64 Ubuntu
* OpenCV
* qt creator
* Visual Studio
* Cmake
* Qmake

**Install the QT creator:**

Run the following commands:
```
sudo apt-get install libpigpio-dev
sudo apt-get install qt5-default qtcreator
```
## Code structure
We divide them into 4 parts: Display, Navigation(path_planning), IMU data processing(from IMU), trigger of data update;
    1. Display relies on Qt. In a window, it shows the map, the current location, destination location, and the navigation path. The display update is driven by timer.
    2. Navigation is used to describe the status of the maps, provide the navigation path, calculate the display data and update them for the display.
    3. IMU libs provide the ECEF class with samples. Then ECEF will process the raw data and generate shifts.
    4. The triggers provide the callback function for the ECEF to update the display data.

## License

## Authors
* Chong Cui(2589587C)
* Hanze Li(2690422L)
* Lang Song(2486323S)
* Zijian Li(2631668L)

## References
Dr.Bernd porr's repositories:https://github.com/berndporr/LSM9DS1_RaspberryPi_CPP_Library

## Acknowledgements


## Social media
<br><a href="https://www.youtube.com/watch?v=k6E_uqkDLo0" target="blank"><img align="left" src="https://upload.wikimedia.org/wikipedia/commons/0/09/YouTube_full-color_icon_%282017%29.svg" height="40" width="40"/>
</a>
<a href="https://www.instagram.com/accounts/login/">
<img align="left" alt="Mitul's LinkedIN" width= "40px" src="https://upload.wikimedia.org/wikipedia/commons/e/e7/Instagram_logo_2016.svg" />
</a>
