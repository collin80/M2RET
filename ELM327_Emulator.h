/*
 *  ELM327_Emu.h
 *
 * Class emulates the serial comm of an ELM327 chip - Used to create an OBDII interface
 *
 * Created: 3/23/2017
 *  Author: Collin Kidder
 */

/*
 Copyright (c) 2017 Collin Kidder

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
 */

/*
List of AT commands to support:
AT E0 (turn echo off)
AT H (0/1) - Turn headers on or off - headers are used to determine how many ECU√≠s present (hint: only send one response to 0100 and emulate a single ECU system to save time coding)
AT L0 (Turn linefeeds off - just use CR)
AT Z (reset)
AT SH - Set header address - seems to set the ECU address to send to (though you may be able to ignore this if you wish)
AT @1 - Display device description - ELM327 returns: Designed by Andy Honecker 2011
AT I - Cause chip to output its ID: ELM327 says: ELM327 v1.3a
AT AT (0/1/2) - Set adaptive timing (though you can ignore this)
AT SP (set protocol) - you can ignore this
AT DP (get protocol by name) - (always return can11/500)
AT DPN (get protocol by number) - (always return 6)
AT RV (adapter voltage) - Send something like 14.4V
*/
/* Added TonyD 17/7/2017
ELM327 AT Command set
--- General Commands ---
all commands are prefixed with "at"

@1				display device description
@2				display the device identifier
@3 cccccccccccc	store the device identifier
<CR>			repeat last command
BRD hh			try baud rate divisor hh
BRT hh			set baud rate handshake timeout
D				set all to defaults
WS				warm start
Z				reset all
E0				echo off
E1				echo on
L0				linefeed off
L1				linefeed on
FE				forget events
I				print the ID
LP				go to low power mode
M0				memory off
M1				memory on
RD				read the stored dara
SD hh			store data byte hh

--- OBD Commands ---
AL				Allow Long (<7 Byte) messages
NL				Allow only normal Length messages
AR				automatic receive
AT0				Adaptive Timing Control off
AT1				Adaptive Timing Control auto1
AT2				Adaptive Timing Control auto2
BD				perform a buffer dump
BI				bypass the initialization sequence
DP				describe current protocol
DPN				describe protocol by number
SP 00			set protocol ??: Ah = Auto,h; h = h; 00 = Auto (h = hex number)
SP Ah			set protocol ??: Ah = Auto,h; h = h; 00 = Auto (h = hex number)
SP h			set protocol ??: Ah = Auto,h; h = h; 00 = Auto (h = hex number)
TP Ah			try protocol h with auto search
TP h			try protocol h
H0				header control off
H1				header control on
MA				monitor all
MR hh			monitor for receiver = hh
MT hh			monitor for transmitter = hh
PC				protocol close
R0				responses off
R1				responses on
RA hh			set the receive address to hh
S0				print spaces off
S1				print spaces on
SH ww xx yy zz	** Unknown **
SH xx yy zz		set header
SH xyz			set header
SR hh			set receive address to hh
SS				set standard search order (j1978)
ST hh			set timeout to hh x 4 sec
TA hh			set tester address to hh
--- Voltage ---
CV dddd			Calibrate the Voltage to dd.dd volts
CV 0000			Restore CV value to factory setting
RV				Read the Voltage
--- OBD Data Commands ---
01 hh			Read PID 0xhh

*** Unknown Commands ***
AMC
AMT hh
CAF0 ! ! ! ! ! ! ! ! ! ! !
CAF1 ! ! ! ! ! ! ! ! ! ! !
CEA ! ! ! !
CEA hh ! ! ! !
CF hh hh hh hh ! ! ! ! ! ! ! ! ! ! !
CF hhh ! ! ! ! ! ! ! ! ! ! !
CFC0 ! ! ! ! ! ! ! ! ! ! !
CFC1 ! ! ! ! ! ! ! ! ! ! !
CM hh hh hh hh ! ! ! ! ! ! ! ! ! ! !
CM hhh ! ! ! ! ! ! ! ! ! ! !
CP hh ! ! ! ! ! ! ! ! ! ! !
CRA ! ! !
CRA hhh ! ! ! ! ! !
CRA hhh ! !
CRA hhhhhhhh ! ! ! ! ! !
CRA hhhhhhhh ! !
CS ! ! ! ! ! ! ! ! ! ! !
CSM0 ! ! !
CSM1 ! ! !
CTM1 !
CTM5

D0 ! ! ! ! ! !
D1 ! ! ! ! ! !
DM1 ! ! ! ! ! ! ! !
FC SD [1-5 bytes] ! ! ! ! ! ! ! ! !
FC SH hh hh hh hh ! ! ! ! ! ! ! ! !
FC SH hhh ! ! ! ! ! ! ! ! !
FC SM h ! ! ! ! ! ! ! ! !
FI ! ! ! !
IB 10 ! ! ! ! ! ! ! ! ! ! !
IB 48 ! ! ! !
IB 96 ! ! ! ! ! ! ! ! ! ! !
IFR H ! ! ! ! ! ! ! !
IFR S ! ! ! ! ! ! ! !
IFR0 ! ! ! ! ! ! ! !
IFR1 ! ! ! ! ! ! ! !
IFR2 ! ! ! ! ! ! ! !
IGN ! ! ! !
IIA hh ! ! ! ! ! ! ! !
JE ! ! ! ! ! !
JHF0 ! ! !
JHF1 ! ! !
JS ! ! ! ! ! !
JTM1 ! ! !
JTM5 ! ! !
KW

KW0 ! ! ! ! ! ! ! !
KW1 ! ! ! ! ! ! ! !
MP hhhh ! ! ! ! ! ! ! !
MP hhhh n ! ! !
MP hhhhhh ! ! ! ! ! !
MP hhhhhh n ! ! !
NL ! ! ! ! ! ! ! ! ! ! !
PB xx yy ! ! ! !
PP FF OFF ! ! ! ! ! ! ! ! !
PP FF ON ! ! ! ! ! ! ! ! !
PP xx OFF ! ! ! ! ! ! ! ! !
PP xx ON ! ! ! ! ! ! ! ! !
PP xx SV yy ! ! ! ! ! ! ! ! !
PPS ! ! ! ! ! ! ! ! !
RTR ! ! ! ! ! !
SI

SW hh ! ! ! ! ! ! ! ! ! ! !
V0 ! ! ! ! ! !
V1 ! ! ! ! ! !
WM [1-6 bytes] ! ! ! ! ! ! ! !
WM xxyyzzaa ! ! ! ! ! ! ! ! ! ! !
WM xxyyzzaabb ! ! ! ! ! ! ! ! ! ! !
WM xxyyzzaabbcc ! ! ! ! ! ! ! ! ! ! !
*/


#ifndef ELM327_H_
#define ELM327_H_

#include <Arduino.h>
#include "config.h"
#include "Logger.h"

class ELM327Emu {
public:

    ELM327Emu();
    ELM327Emu(UARTClass *which);
    void setup(); //initialization on start up
    bool testHardware(); // Test if Hardware exists
//    void handleTick(); //periodic processes
    void loop();
    void sendCmd(String cmd);

private:
    UARTClass *serialInterface; //Allows for retargetting which serial port we use
    char incomingBuffer[128]; //storage for one incoming line
    char buffer[30]; // a buffer for various string conversions
    bool bLineFeed; //should we use line feeds?
    bool bHeader; //should we produce a header?
    int tickCounter;
    int ibWritePtr;
    int currReply;

    void processCmd();
    String processELMCmd(char *cmd);
    bool processRequest(uint8_t mode, uint8_t pid, char *inData, char *outData);
    bool processShowData(uint8_t pid, char *inData, char *outData);
    bool processShowCustomData(uint16_t pid, char *inData, char *outData);    
};


  
#endif


