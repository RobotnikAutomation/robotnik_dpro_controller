# Installation

```
$ cd build 
$ rm *
$ cd ..
$ cd src
$ make; sudo make install
```

# How to set the motor ID

**Connect only one motor in to the bus!!!**

* Be sure that only one motor is connected into the bus
* Go to example/monitor folder
* Compile the monitor program: 
  * $ make
* Run the monitor program:
  * $ ./monitor -d /dev/ttyUSB0 (Set the correct device)
  * $ [CMD] scan (Output the current status of all DXLs)
  * $ [CMD] wrb 7 11 (7 is the memory address to save the ID, 11 is the new address)

# References

* [Dynamixel-Pro Quick Start Guide](http://www.robotis.com/download/doc/Dynamixel_pro/Dynamixel-Pro%20Quick%20Start_en.pdf)
