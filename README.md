# rpi-xbee-cpp

C++ [XBee Arduino Library](https://github.com/andrewrapp/xbee-arduino "XBee Arduino Library") adopted for Raspberry Pi. 

This code uses [WiringPi Library](http://wiringpi.com/ "WiringPi Library")

Here is a simple example **AtCommand**, which is an analogue of the example for arduino from the library [AtCommand.pde](https://github.com/andrewrapp/xbee-arduino/blob/master/examples/AtCommand/AtCommand.pde "AtCommand.pde"). The rest of the examples can be rewritten in a similar way.

To build and run this example use g++:

```
g++ AtCommand.cpp -o AtCommand -lwiringPi
sudo ./AtCommand
```
Before building, make sure that the WiringPy library is installed and the serial port you are using (in this example /dev/ttyS0 with baudrate 115200 is used `Serial.begin("/dev/ttyS0", 115200);`) is activated
