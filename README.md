M2RET
=======

Reverse Engineering Tool running on Macchina M2 Hardware

A fork of the GVRET project.

#### Requirements:

You will need the following to have any hope of compiling and running the firmware:

- [Arduino IDE](https://www.arduino.cc/en/Main/Software) 1.5.4 or higher (tested all of the way up to 1.6.12)
- [due_can](https://github.com/collin80/due_can) - Object oriented canbus library for Arduino Due compatible boards.
- [due_wire](https://github.com/collin80/due_wire) - An alternative I2C library for Due with DMA support.
- All of the M2 support libraries at https://github.com/macchina/m2-libraries

All libraries belong in %USERPROFILE%\Documents\Arduino\libraries (Windows) or ~/Arduino/libraries (Linux/Mac).
You will need to remove -master or any other postfixes. Your library folders should be named as above.

The canbus is supposed to be terminated on both ends of the bus. This should not be a problem as this firmware will be used to reverse engineer existing buses.

#### License:

This software is MIT licensed:

Copyright (c) 2014-2017 Collin Kidder, Michael Neuweiler, Charles Galpin

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

