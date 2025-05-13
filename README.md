# Battery Management System Intelligent (BMSi) 
The BMSi is a open source smart battery management system with the goal of being reliable and affordable with the ability to communicate over USB with robotic systems using ROS or computer with serial terminal, providing all the valuable data needed about the battery.

With the goal of affordability and ease o use the main component is an ESP32, using Arduino Language (C++). The complex part of this project (BQ76942) was simplified with the development of a library, and further development will be done on this to achieve full abstraction on the configuration and use of this IC, making it even more simple to use.

The hardware designs are found under [BMSi-Hardware](https://github.com/MrSerafim/BMSi-Hardware).

Updates require matching firmware and board versions!

### Copyright notice:
Some parts of this software may contain third party libraries and source code licensed under different terms.
The license applying to these files is found in the header of the file.
For all other parts the LICENSE file applies.