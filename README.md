NARTOS for AVR
--------------

A non-real time scheduling operating system with bare minimal features and
footprint. This OS is intended to modularise and manage applications on very
small AVRs, having as low as 360 bytes of FLASH and 3 bytes of RAM per task. It
is useful for applications with quite loose real time demands. It does not use
time slicing and relies on programming techniques to manage task flow.

More information about NARTOS is available on:

http://www.jiggerjuice.info/electronics/projects/scheduling-os.html

Also some small libraries written for a limited number of AVRs, notably the
ATMega48 series. These libraries are used in some older projects and will be
removed as the latter are converted to use more comprehensive and better written
libraries, e.g. http://www.procyonengineering.com/embedded/avr/avrlib/.

1. A/D converter library
2. Timer library
3. TWI library (I2C)

(c) K. Sarkies 18 October 2014

