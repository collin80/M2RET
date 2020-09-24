/*
 * SerialConsole.cpp
 *
 Copyright (c) 2014 Collin Kidder

 Shamelessly copied from the version in GEVCU

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

#include "SerialConsole.h"
#include <due_wire.h>
#include <due_can.h>
#include <MCP2515_sw_can.h>
#include <lin_stack.h>
#include "EEPROM.h"
#include "config.h"
#include "sys_io.h"

extern EEPROMCLASS *eeprom;
extern SWcan SWCAN;
extern lin_stack LIN1;
extern lin_stack LIN2;
extern void CANHandler();

SerialConsole::SerialConsole()
{
    init();
}

void SerialConsole::init()
{
    //State variables for serial console
    ptrBuffer = 0;
    state = STATE_ROOT_MENU;
}

void SerialConsole::printMenu()
{
    char buff[80];
    //Show build # here as well in case people are using the native port and don't get to see the start up messages
    SerialUSB.print("Build number: ");
    SerialUSB.println(CFG_BUILD_NUM);
    SerialUSB.println("System Menu:");
    SerialUSB.println();
    SerialUSB.println("Enable line endings of some sort (LF, CR, CRLF)");
    SerialUSB.println();
    SerialUSB.println("Short Commands:");
    SerialUSB.println("h = help (displays this message)");
    SerialUSB.println("R = reset to factory defaults");
    SerialUSB.println("s = Start logging to file");
    SerialUSB.println("S = Stop logging to file");
    SerialUSB.println();
    SerialUSB.println("Config Commands (enter command=newvalue). Current values shown in parenthesis:");
    SerialUSB.println();

    Logger::console("LOGLEVEL=%i - set log level (0=debug, 1=info, 2=warn, 3=error, 4=off)", settings.logLevel);
    Logger::console("SYSTYPE=%i - set board type (0=Macchina M2)", settings.sysType);
    SerialUSB.println();

    Logger::console("CAN0EN=%i - Enable/Disable CAN0 (0 = Disable, 1 = Enable)", settings.CAN0_Enabled);
    Logger::console("CAN0SPEED=%i - Set speed of CAN0 in baud (125000, 250000, etc)", settings.CAN0Speed);
    Logger::console("CAN0LISTENONLY=%i - Enable/Disable Listen Only Mode (0 = Dis, 1 = En)", settings.CAN0ListenOnly);
    /*for (int i = 0; i < 8; i++) {
        sprintf(buff, "CAN0FILTER%i=0x%%x,0x%%x,%%i,%%i (ID, Mask, Extended, Enabled)", i);
        Logger::console(buff, settings.CAN0Filters[i].id, settings.CAN0Filters[i].mask,
                        settings.CAN0Filters[i].extended, settings.CAN0Filters[i].enabled);
    }*/
    SerialUSB.println();

    Logger::console("CAN1EN=%i - Enable/Disable CAN1 (0 = Disable, 1 = Enable)", settings.CAN1_Enabled);
    Logger::console("CAN1SPEED=%i - Set speed of CAN1 in baud (125000, 250000, etc)", settings.CAN1Speed);
    Logger::console("CAN1LISTENONLY=%i - Enable/Disable Listen Only Mode (0 = Dis, 1 = En)", settings.CAN1ListenOnly);
    /*
    for (int i = 0; i < 8; i++) {
        sprintf(buff, "CAN1FILTER%i=0x%%x,0x%%x,%%i,%%i (ID, Mask, Extended, Enabled)", i);
        Logger::console(buff, settings.CAN1Filters[i].id, settings.CAN1Filters[i].mask,
                        settings.CAN1Filters[i].extended, settings.CAN1Filters[i].enabled);
    }*/
    
    SerialUSB.println();

    Logger::console("SWCANEN=%i - Enable/Disable Single Wire CAN (0 = Disable, 1 = Enable)", settings.SWCAN_Enabled);
    Logger::console("SWCANSPEED=%i - Set speed of Single Wire CAN in baud (33000, 93000, etc)", settings.SWCANSpeed);
    Logger::console("SWCANLISTENONLY=%i - Enable/Disable Listen Only Mode (0 = Dis, 1 = En)", settings.SWCANListenOnly);
    SerialUSB.println();
    
    Logger::console("CAN0SEND=ID,LEN,<BYTES SEPARATED BY COMMAS> - Ex: CAN0SEND=0x200,4,1,2,3,4");
    Logger::console("CAN1SEND=ID,LEN,<BYTES SEPARATED BY COMMAS> - Ex: CAN1SEND=0x200,8,00,00,00,10,0xAA,0xBB,0xA0,00");
    Logger::console("SWSEND=ID,LEN,<BYTES SEPARATED BY COMMAS> - Ex: SWSEND=0x100,4,10,20,30,40");
    Logger::console("MARK=<Description of what you are doing> - Set a mark in the log file about what you are about to do.");
    SerialUSB.println();

    Logger::console("BINSERIAL=%i - Enable/Disable Binary Sending of CANBus Frames to Serial (0=Dis, 1=En)", settings.useBinarySerialComm);
    Logger::console("FILETYPE=%i - Set type of file output (0=None, 1 = Binary, 2 = GVRET, 3 = CRTD)", settings.fileOutputType);
    SerialUSB.println();

    Logger::console("FILEBASE=%s - Set filename base for saving", (char *)settings.fileNameBase);
    Logger::console("FILEEXT=%s - Set filename ext for saving", (char *)settings.fileNameExt);
    Logger::console("FILENUM=%i - Set incrementing number for filename", settings.fileNum);
    Logger::console("FILEAPPEND=%i - Append to file (no numbers) or use incrementing numbers after basename (0=Incrementing Numbers, 1=Append)", settings.appendFile);
    Logger::console("FILEAUTO=%i - Automatically start logging at startup (0=No, 1 = Yes)", settings.autoStartLogging);
    SerialUSB.println();

    Logger::console("DIGTOGEN=%i - Enable digital toggling system (0 = Dis, 1 = En)", digToggleSettings.enabled);
    Logger::console("DIGTOGMODE=%i - Set digital toggle mode (0 = Read pin, send CAN, 1 = Receive CAN, set pin)", digToggleSettings.mode & 1);
    Logger::console("DIGTOGLEVEL=%i - Set default level of digital pin (0 = LOW, 1 = HIGH)", digToggleSettings.mode >> 7);
    Logger::console("DIGTOGPIN=%i - Pin to use for digital toggling system (Use Arduino Digital Pin Number)", digToggleSettings.pin);
    Logger::console("DIGTOGID=%X - CAN ID to use for Rx or Tx", digToggleSettings.rxTxID);
    Logger::console("DIGTOGCAN0=%i - Use CAN0 with Digital Toggling System? (0 = No, 1 = Yes)", (digToggleSettings.mode >> 1) & 1);
    Logger::console("DIGTOGCAN1=%i - Use CAN1 with Digital Toggling System? (0 = No, 1 = Yes)", (digToggleSettings.mode >> 2) & 1);
    Logger::console("DIGTOGLEN=%i - Length of frame to send (Tx) or validate (Rx)", digToggleSettings.length);
    Logger::console("DIGTOGPAYLOAD=%X,%X,%X,%X,%X,%X,%X,%X - Payload to send or validate against (comma separated list)", digToggleSettings.payload[0],
                    digToggleSettings.payload[1], digToggleSettings.payload[2], digToggleSettings.payload[3], digToggleSettings.payload[4],
                    digToggleSettings.payload[5], digToggleSettings.payload[6], digToggleSettings.payload[7]);
}

/*	There is a help menu (press H or h or ?)
 This is no longer going to be a simple single character console.
 Now the system can handle up to 80 input characters. Commands are submitted
 by sending line ending (LF, CR, or both)
 */
void SerialConsole::rcvCharacter(uint8_t chr)
{
    if (chr == 10 || chr == 13) { //command done. Parse it.
        handleConsoleCmd();
        ptrBuffer = 0; //reset line counter once the line has been processed
    } else {
        cmdBuffer[ptrBuffer++] = (unsigned char) chr;
        if (ptrBuffer > 79)
            ptrBuffer = 79;
    }
}

void SerialConsole::handleConsoleCmd()
{
    if (state == STATE_ROOT_MENU) {
        if (ptrBuffer == 1) {
            //command is a single ascii character
            handleShortCmd();
        } else { //at least two bytes
            boolean equalSign = false;
            for (int i = 0; i < ptrBuffer; i++) if (cmdBuffer[i] == '=') equalSign = true;
            if (equalSign) handleConfigCmd();
            else handleLawicelCmd(); //single letter lawicel commands handled in handleShortCmd though.
        }
        ptrBuffer = 0; //reset line counter once the line has been processed
    }
}

/*
LAWICEL single letter commands are now mixed in with the other commands here.
*/
void SerialConsole::handleShortCmd()
{
    uint8_t val;

    switch (cmdBuffer[0]) {
    //non-lawicel commands
    case 'h':
    case '?':
    case 'H':
        printMenu();
        break;
    case 'R': //reset to factory defaults.
        settings.version = 0xFF;
        EEPROM.write(EEPROM_ADDR, settings);
        Logger::console("Power cycle to reset to factory defaults");
        break;
    case 's': //start logging canbus to file
        Logger::console("Starting logging to file.");
        SysSettings.logToFile = true;
        break;
    case 'S': //stop logging canbus to file
        Logger::console("Ceasing file logging.");
        SysSettings.logToFile = false;
        break;
        
    //Lawicel specific commands    
    case 'O': //LAWICEL open canbus port (first one only because LAWICEL has no concept of dual canbus
        Can0.disable_autobaud_listen_mode();
        Can0.set_baudrate(settings.CAN0Speed);
        Can0.enable();
        SerialUSB.write(13); //send CR to mean "ok"
        SysSettings.lawicelMode = true;
        break;
    case 'C': //LAWICEL close canbus port (First one)
        Can0.disable();
        SerialUSB.write(13); //send CR to mean "ok"
        break;
    case 'L': //LAWICEL open canbus port in listen only mode
        Can0.enable_autobaud_listen_mode();
        Can0.set_baudrate(settings.CAN0Speed);
        Can0.enable();
        SerialUSB.write(13); //send CR to mean "ok"
        SysSettings.lawicelMode = true;
        break;
    case 'P': //LAWICEL - poll for one waiting frame. Or, just CR if no frames
        if (Can0.available()) SysSettings.lawicelPollCounter = 1;
        else SerialUSB.write(13); //no waiting frames
        break;
    case 'A': //LAWICEL - poll for all waiting frames - CR if no frames
        SysSettings.lawicelPollCounter = Can0.available();
        if (SysSettings.lawicelPollCounter == 0) SerialUSB.write(13);
        break;
    case 'F': //LAWICEL - read status bits
        SerialUSB.print("F00"); //bit 0 = RX Fifo Full, 1 = TX Fifo Full, 2 = Error warning, 3 = Data overrun, 5= Error passive, 6 = Arb. Lost, 7 = Bus Error
        SerialUSB.write(13);
        break;
    case 'V': //LAWICEL - get version number
        SerialUSB.print("V1013\n");
        SysSettings.lawicelMode = true;
        break;
    case 'N': //LAWICEL - get serial number
        SerialUSB.print("M2RET\n");
        SysSettings.lawicelMode = true;
        break;
    case 'x':
        SysSettings.lawicellExtendedMode = !SysSettings.lawicellExtendedMode;
        if (SysSettings.lawicellExtendedMode) {
            SerialUSB.print("V2\n");
        }
        else {
            SerialUSB.print("LAWICEL\n");
        }            
        break;
    case 'B': //LAWICEL V2 - Output list of supported buses
        if (SysSettings.lawicellExtendedMode) {
            for (int i = 0; i < NUM_BUSES; i++) {
                printBusName(i);
                SerialUSB.print("\n");
            }
        }
        break;
    case 'X':
        if (SysSettings.lawicellExtendedMode) {
            SerialUSB.print("Led1 green\n");
            SerialUSB.print("Led2 yellow\n");
            SerialUSB.print("Led3 yellow\n");
            SerialUSB.print("Led4 yellow\n");
            SerialUSB.print("Led5 red\n");
            SerialUSB.print("Led6 rgb\n");            
        }
        break;        
    }
}

void SerialConsole::handleLawicelCmd()
{
    cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
    CAN_FRAME outFrame;
    char buff[80];
    int val;
    
    tokenizeCmdString();

    switch (cmdBuffer[0]) {
    case 't': //transmit standard frame
        outFrame.id = parseHexString(cmdBuffer + 1, 3);
        outFrame.length = cmdBuffer[4] - '0';
        outFrame.extended = false;
        if (outFrame.length < 0) outFrame.length = 0;
        if (outFrame.length > 8) outFrame.length = 8;
        for (int data = 0; data < outFrame.length; data++) {
            outFrame.data.bytes[data] = parseHexString(cmdBuffer + 5 + (2 * data), 2);
        }
        Can0.sendFrame(outFrame);
        if (SysSettings.lawicelAutoPoll) SerialUSB.print("z");
        break;
    case 'T': //transmit extended frame
        outFrame.id = parseHexString(cmdBuffer + 1, 8);
        outFrame.length = cmdBuffer[9] - '0';
        outFrame.extended = false;
        if (outFrame.length < 0) outFrame.length = 0;
        if (outFrame.length > 8) outFrame.length = 8;
        for (int data = 0; data < outFrame.length; data++) {
            outFrame.data.bytes[data] = parseHexString(cmdBuffer + 10 + (2 * data), 2);
        }
        Can0.sendFrame(outFrame);
        if (SysSettings.lawicelAutoPoll) SerialUSB.print("Z");
        break;
    case 'S': 
        if (!SysSettings.lawicellExtendedMode) {
            //setup canbus baud via predefined speeds
            val = parseHexCharacter(cmdBuffer[1]);
            switch (val) {
            case 0:
                settings.CAN0Speed = 10000;
                break;
            case 1:
                settings.CAN0Speed = 20000;
                break;
            case 2:
                settings.CAN0Speed = 50000;
                break;
            case 3:
                settings.CAN0Speed = 100000;
                break;
            case 4:
                settings.CAN0Speed = 125000;
                break;
            case 5:
                settings.CAN0Speed = 250000;
                break;
            case 6:
                settings.CAN0Speed = 500000;
                break;
            case 7:
                settings.CAN0Speed = 800000;
                break;
            case 8:
                settings.CAN0Speed = 1000000;
                break;
            }
        }
        else { //LAWICEL V2 - Send packet out of specified bus - S <Bus> <ID> <Data0> <Data1> <...>
            uint8_t bytes[8];
            uint32_t id;
            int numBytes = 0;
            id = strtol(tokens[2], nullptr, 16);
            for (int b = 0; b < 8; b++) {
                if (tokens[3 + b][0] != 0) {
                    bytes[b] = strtol(tokens[3 + b], nullptr, 16);
                    numBytes++;
                }
                else break; //break for loop because we're obviously done.
            }
            if (!stricmp(tokens[1], "CAN0")) {
                CAN_FRAME outFrame;
                outFrame.id = id;
                outFrame.length = numBytes;
                outFrame.extended = false;
                for (int b = 0; b < numBytes; b++) outFrame.data.bytes[b] = bytes[b];
                Can0.sendFrame(outFrame);
            }
            if (!stricmp(tokens[1], "CAN1")) {
                CAN_FRAME outFrame;
                outFrame.id = id;
                outFrame.length = numBytes;
                outFrame.extended = false;
                for (int b = 0; b < numBytes; b++) outFrame.data.bytes[b] = bytes[b];
                Can1.sendFrame(outFrame);                
            }
            if (!stricmp(tokens[1], "SWCAN")) {
                CAN_FRAME outFrame;
                outFrame.id = id;
                outFrame.length = numBytes;
                outFrame.extended = false;
                outFrame.rtr = 0;
                for (int b = 0; b < numBytes; b++) outFrame.data.bytes[b] = bytes[b];                
                SWCAN.sendFrame(outFrame);
            }
            if (!stricmp(tokens[1], "LIN1")) {
            }
            if (!stricmp(tokens[1], "LIN2")) {
            }
        }
    case 's': //setup canbus baud via register writes (we can't really do that...)
        //settings.CAN0Speed = 250000;
        break;
    case 'r': //send a standard RTR frame (don't really... that's so deprecated its not even funny)
        break;
    case 'R': 
        if (SysSettings.lawicellExtendedMode) { //Lawicel V2 - Set that we want to receive traffic from the given bus - R <BUSID>
            if (!stricmp(tokens[1], "CAN0")) SysSettings.lawicelBusReception[0] = true;
            if (!stricmp(tokens[1], "CAN1")) SysSettings.lawicelBusReception[1] = true; 
            if (!stricmp(tokens[1], "SWCAN")) SysSettings.lawicelBusReception[2] = true;
            if (!stricmp(tokens[1], "LIN1")) SysSettings.lawicelBusReception[3] = true;
            if (!stricmp(tokens[1], "LIN2")) SysSettings.lawicelBusReception[4] = true;                                            
        }
        else { //Lawicel V1 - send extended RTR frame (NO! DON'T DO IT!)
        }
        break;
    case 'X': //Set autopoll off/on
        if (cmdBuffer[1] == '1') SysSettings.lawicelAutoPoll = true;
        else SysSettings.lawicelAutoPoll = false;
        break;
    case 'W': //Dual or single filter mode
        break; //don't actually support this mode
    case 'm': //set acceptance mask - these things seem to be odd and aren't actually implemented yet
    case 'M': 
        if (SysSettings.lawicellExtendedMode) { //Lawicel V2 - Set filter mask - M <busid> <Mask> <FilterID> <Ext?>
            int mask = strtol(tokens[2], nullptr, 16);
            int filt = strtol(tokens[3], nullptr, 16);
            if (!stricmp(tokens[1], "CAN0")) {
                if (!stricmp(tokens[4], "X")) Can0.setRXFilter(0, filt, mask, true);
                    else Can0.setRXFilter(0, filt, mask, false);
            }           
            if (!stricmp(tokens[1], "CAN1")) {
                if (!stricmp(tokens[4], "X")) Can1.setRXFilter(0, filt, mask, true);
                    else Can1.setRXFilter(0, filt, mask, false);                
            }
            if (!stricmp(tokens[1], "SWCAN")) {
                if (!stricmp(tokens[4], "X")) 
                {
                    SWCAN.SetRXFilter(0, filt, true);
                }
                else 
                {
                    SWCAN.SetRXFilter(0, filt, false);
                }
            }
            if (!stricmp(tokens[1], "LIN1")) {
                //set mask on LIN. Can you do that?!
            }
            if (!stricmp(tokens[1], "LIN2")) {
                //set mask on LIN if that's even possible?!
            }            
        }
        else { //Lawicel V1 - set acceptance code
        }        
        break;
    case 'H':
        if (SysSettings.lawicellExtendedMode) { //Lawicel V2 - Halt reception of traffic from given bus - H <busid>
            if (!stricmp(tokens[1], "CAN0")) SysSettings.lawicelBusReception[0] = false;
            if (!stricmp(tokens[1], "CAN1")) SysSettings.lawicelBusReception[1] = false; 
            if (!stricmp(tokens[1], "SWCAN")) SysSettings.lawicelBusReception[2] = false;
            if (!stricmp(tokens[1], "LIN1")) SysSettings.lawicelBusReception[3] = false;
            if (!stricmp(tokens[1], "LIN2")) SysSettings.lawicelBusReception[4] = false;                        
        } 
        break;        
    case 'U': //set uart speed. We just ignore this. You can't set a baud rate on a USB CDC port
        break; //also no action here
    case 'Z': //Turn timestamp off/on
        if (cmdBuffer[1] == '1') SysSettings.lawicelTimestamping = true;
        else SysSettings.lawicelTimestamping =  false;
        break;
    case 'Q': //turn auto start up on/off - probably don't need to actually implement this at the moment.
        break; //no action yet or maybe ever
    case 'C': //Lawicel V2 - configure one of the buses - C <busid> <speed> <any additional needed params> 
        if (SysSettings.lawicellExtendedMode) {
            //at least two parameters separated by spaces. First BUS ID (CAN0, CAN1, SWCAN, etc) then speed (or more params separated by #'s)
            int speed = atoi(tokens[2]);
            if (!stricmp(tokens[1], "CAN0")) {
                Can0.set_baudrate(speed);
            }           
            if (!stricmp(tokens[1], "CAN1")) {
                Can1.set_baudrate(speed);
            }
            if (!stricmp(tokens[1], "SWCAN")) {
                //can't set speed of SWCAN yet
            }
            if (!stricmp(tokens[1], "LIN1")) {
                //can't set speed of LIN1 yet
            }
            if (!stricmp(tokens[1], "LIN2")) {
                //can't set speed of LIN2 yet
            }
        }
        break;
    }
    SerialUSB.write(13);
}

/*For simplicity the configuration setting code uses four characters for each configuration choice. This makes things easier for
 comparison purposes.
 */
void SerialConsole::handleConfigCmd()
{
    int i;
    int newValue;
    char *newString;
    bool writeEEPROM = false;
    bool writeDigEE = false;
    char *dataTok;

    //Logger::debug("Cmd size: %i", ptrBuffer);
    if (ptrBuffer < 6)
        return; //4 digit command, =, value is at least 6 characters
    cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
    String cmdString = String();
    unsigned char whichEntry = '0';
    i = 0;

    while (cmdBuffer[i] != '=' && i < ptrBuffer) {
        cmdString.concat(String(cmdBuffer[i++]));
    }
    i++; //skip the =
    if (i >= ptrBuffer) {
        Logger::console("Command needs a value..ie TORQ=3000");
        Logger::console("");
        return; //or, we could use this to display the parameter instead of setting
    }

    // strtol() is able to parse also hex values (e.g. a string "0xCAFE"), useful for enable/disable by device id
    newValue = strtol((char *) (cmdBuffer + i), NULL, 0); //try to turn the string into a number
    newString = (char *)(cmdBuffer + i); //leave it as a string

    cmdString.toUpperCase();

    if (cmdString == String("CAN0EN")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console("Setting CAN0 Enabled to %i", newValue);
        settings.CAN0_Enabled = newValue;
        if (newValue == 1) {
            Can0.set_baudrate(settings.CAN0Speed);
            Can0.enable();
        } else
            Can0.disable();
        writeEEPROM = true;
    } else if (cmdString == String("CAN1EN")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console("Setting CAN1 Enabled to %i", newValue);
        if (newValue == 1) {
            Can1.set_baudrate(settings.CAN1Speed);
            Can1.enable();
        } else
            Can1.disable();
        settings.CAN1_Enabled = newValue;
        writeEEPROM = true;
    } else if (cmdString == String("SWCANEN")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console("Setting SWCAN Enabled to %i", newValue);
        if (newValue == 1) {
            SWCAN.setupSW(settings.SWCANSpeed);       
            delay(20);
            SWCAN.mode(3); // Go to normal mode. 0 - Sleep, 1 - High Speed, 2 - High Voltage Wake-Up, 3 - Normal
            attachInterrupt(SWC_INT, CANHandler, FALLING); //enable interrupt for SWCAN
        }
        else {
            SWCAN.Reset();
            SWCAN.mode(0); //go to sleep
        }
        settings.SWCAN_Enabled = newValue;
        writeEEPROM = true;        
    } else if (cmdString == String("CAN0SPEED")) {
        if (newValue > 0 && newValue <= 1000000) {
            Logger::console("Setting CAN0 Baud Rate to %i", newValue);
            settings.CAN0Speed = newValue;
            if (settings.CAN0_Enabled) Can0.set_baudrate(settings.CAN1Speed);
            writeEEPROM = true;
        } else Logger::console("Invalid baud rate! Enter a value 1 - 1000000");
    } else if (cmdString == String("CAN1SPEED")) {
        if (newValue > 0 && newValue <= 1000000) {
            Logger::console("Setting CAN1 Baud Rate to %i", newValue);
            settings.CAN1Speed = newValue;
            if (settings.CAN1_Enabled) Can1.set_baudrate(settings.CAN1Speed);
            writeEEPROM = true;
        } else Logger::console("Invalid baud rate! Enter a value 1 - 1000000");
    } else if (cmdString == String("SWCANSPEED")) {
        if (newValue > 0 && newValue <= 1000000) {
            Logger::console("Setting Single Wire CAN Baud Rate to %i", newValue);
            settings.SWCANSpeed = newValue;
            if (settings.SWCAN_Enabled) 
            {               
                SWCAN.setupSW(settings.SWCANSpeed);       
            }
            writeEEPROM = true;
        } else Logger::console("Invalid baud rate! Enter a value 1 - 1000000");
    } else if (cmdString == String("CAN0LISTENONLY")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console("Setting CAN0 Listen Only to %i", newValue);
            settings.CAN0ListenOnly = newValue;
            if (settings.CAN0ListenOnly) {
                Can0.enable_autobaud_listen_mode();
            } else {
                Can0.disable_autobaud_listen_mode();
            }
            writeEEPROM = true;
        } else Logger::console("Invalid setting! Enter a value 0 - 1");
    } else if (cmdString == String("CAN1LISTENONLY")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console("Setting CAN1 Listen Only to %i", newValue);
            settings.CAN1ListenOnly = newValue;
            if (settings.CAN1ListenOnly) {
                Can1.enable_autobaud_listen_mode();
            } else {
                Can1.disable_autobaud_listen_mode();
            }
            writeEEPROM = true;
        } else Logger::console("Invalid setting! Enter a value 0 - 1");
    } else if (cmdString == String("SWCANLISTENONLY")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console("Setting SWCAN Listen Only to %i", newValue);
            settings.SWCANListenOnly = newValue;
            if (settings.CAN1ListenOnly) {
                //Can1.enable_autobaud_listen_mode();
            } else {
                //Can1.disable_autobaud_listen_mode();
            }
            writeEEPROM = true;
        } else Logger::console("Invalid setting! Enter a value 0 - 1");        
    } else if (cmdString == String("CAN0FILTER0")) { //someone should kick me in the face for this laziness... FIX THIS!
        handleFilterSet(0, 0, newString);
    } else if (cmdString == String("CAN0FILTER1")) {
        if (handleFilterSet(0, 1, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN0FILTER2")) {
        if (handleFilterSet(0, 2, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN0FILTER3")) {
        if (handleFilterSet(0, 3, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN0FILTER4")) {
        if (handleFilterSet(0, 4, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN0FILTER5")) {
        if (handleFilterSet(0, 5, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN0FILTER6")) {
        if (handleFilterSet(0, 6, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN0FILTER7")) {
        if (handleFilterSet(0, 7, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN1FILTER0")) {
        if (handleFilterSet(1, 0, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN1FILTER1")) {
        if (handleFilterSet(1, 1, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN1FILTER2")) {
        if (handleFilterSet(1, 2, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN1FILTER3")) {
        if (handleFilterSet(1, 3, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN1FILTER4")) {
        if (handleFilterSet(1, 4, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN1FILTER5")) {
        if (handleFilterSet(1, 5, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN1FILTER6")) {
        if (handleFilterSet(1, 6, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN1FILTER7")) {
        if (handleFilterSet(1, 7, newString)) writeEEPROM = true;
    } else if (cmdString == String("CAN0SEND")) {
        handleCANSend(Can0, newString);
    } else if (cmdString == String("CAN1SEND")) {
        handleCANSend(Can1, newString);
    } else if (cmdString == String("SWSEND")) {
        handleSWCANSend(newString);
    } else if (cmdString == String("MARK")) { //just ascii based for now
        if (settings.fileOutputType == GVRET) Logger::file("Mark: %s", newString);
        if (settings.fileOutputType == CRTD) {
            uint8_t buff[40];
            sprintf((char *)buff, "%f CEV ", millis() / 1000.0f);
            Logger::fileRaw(buff, strlen((char *)buff));
            Logger::fileRaw((uint8_t *)newString, strlen(newString));
            buff[0] = '\r';
            buff[1] = '\n';
            Logger::fileRaw(buff, 2);
        }
        if (!settings.useBinarySerialComm) Logger::console("Mark: %s", newString);

    } else if (cmdString == String("BINSERIAL")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console("Setting Serial Binary Comm to %i", newValue);
        settings.useBinarySerialComm = newValue;
        writeEEPROM = true;
    } else if (cmdString == String("FILETYPE")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 3) newValue = 3;
        Logger::console("Setting File Output Type to %i", newValue);
        settings.fileOutputType = (FILEOUTPUTTYPE)newValue; //the numbers all intentionally match up so this works
        writeEEPROM = true;
    } else if (cmdString == String("FILEBASE")) {
        Logger::console("Setting File Base Name to %s", newString);
        strcpy((char *)settings.fileNameBase, newString);
        writeEEPROM = true;
    } else if (cmdString == String("FILEEXT")) {
        Logger::console("Setting File Extension to %s", newString);
        strcpy((char *)settings.fileNameExt, newString);
        writeEEPROM = true;
    } else if (cmdString == String("FILENUM")) {
        Logger::console("Setting File Incrementing Number Base to %i", newValue);
        settings.fileNum = newValue;
        writeEEPROM = true;
    } else if (cmdString == String("FILEAPPEND")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console("Setting File Append Mode to %i", newValue);
        settings.appendFile = newValue;
        writeEEPROM = true;
    } else if (cmdString == String("FILEAUTO")) {
        if (newValue < 0) newValue = 0;
        if (newValue > 1) newValue = 1;
        Logger::console("Setting Auto File Logging Mode to %i", newValue);
        settings.autoStartLogging = newValue;
        writeEEPROM = true;
    } else if (cmdString == String("SYSTYPE")) {
        if (newValue < 1 && newValue >= 0) {
            settings.sysType = newValue;
            writeEEPROM = true;
            Logger::console("System type updated. Power cycle to apply.");
        } else Logger::console("Invalid system type. Please enter a value between 0 and 0. Yes, just 0");
    } else if (cmdString == String("DIGTOGEN")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console("Setting Digital Toggle System Enable to %i", newValue);
            digToggleSettings.enabled = newValue;
            writeDigEE = true;
        } else Logger::console("Invalid enable value. Must be either 0 or 1");
    } else if (cmdString == String("DIGTOGMODE")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console("Setting Digital Toggle Mode to %i", newValue);
            if (newValue == 0) digToggleSettings.mode &= ~1;
            if (newValue == 1) digToggleSettings.mode |= 1;
            writeDigEE = true;
        } else Logger::console("Invalid mode. Must be either 0 or 1");
    } else if (cmdString == String("DIGTOGLEVEL")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console("Setting Digital Toggle Starting Level to %i", newValue);
            if (newValue == 0) digToggleSettings.mode &= ~0x80;
            if (newValue == 1) digToggleSettings.mode |= 0x80;
            writeDigEE = true;
        } else Logger::console("Invalid default level. Must be either 0 or 1");
    } else if (cmdString == String("DIGTOGPIN")) {
        if (newValue >= 0 && newValue <= 77) {
            Logger::console("Setting Digital Toggle Pin to %i", newValue);
            digToggleSettings.pin = newValue;
            writeDigEE = true;
        } else Logger::console("Invalid pin. Must be between 0 and 77");
    } else if (cmdString == String("DIGTOGID")) {
        if (newValue >= 0 && newValue < (1 << 30)) {
            Logger::console("Setting Digital Toggle CAN ID to %X", newValue);
            digToggleSettings.rxTxID = newValue;
            writeDigEE = true;
        } else Logger::console("Invalid CAN ID. Must be either an 11 or 29 bit ID");
    } else if (cmdString == String("DIGTOGCAN0")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console("Setting Digital Toggle CAN0 Usage to %i", newValue);
            if (newValue == 0) digToggleSettings.mode &= ~2;
            if (newValue == 1) digToggleSettings.mode |= 2;
            writeDigEE = true;
        } else Logger::console("Invalid value. Must be either 0 or 1");
    } else if (cmdString == String("DIGTOGCAN1")) {
        if (newValue >= 0 && newValue <= 1) {
            Logger::console("Setting Digital Toggle CAN1 Usage to %i", newValue);
            if (newValue == 0) digToggleSettings.mode &= ~4;
            if (newValue == 1) digToggleSettings.mode |= 4;
            writeDigEE = true;
        } else Logger::console("Invalid value. Must be either 0 or 1");
    } else if (cmdString == String("DIGTOGLEN")) {
        if (newValue >= 0 && newValue <= 8) {
            Logger::console("Setting Digital Toggle Frame Length to %i", newValue);
            digToggleSettings.length = newValue;
            writeDigEE = true;
        } else Logger::console("Invalid length. Must be between 0 and 8");
    } else if (cmdString == String("DIGTOGPAYLOAD")) {
        dataTok = strtok(newString, ",");
        if (dataTok) {
            digToggleSettings.payload[0] = strtol(dataTok, NULL, 0);
            i = 1;
            while (i < 8 && dataTok) {
                dataTok = strtok(NULL, ",");
                if (dataTok) {
                    digToggleSettings.payload[i] = strtol(dataTok, NULL, 0);
                    i += 1;
                }
            }
            writeDigEE = true;
            Logger::console("Set new payload bytes");
        } else Logger::console("Error processing payload");
    } else if (cmdString == String("LOGLEVEL")) {
        switch (newValue) {
        case 0:
            Logger::setLoglevel(Logger::Debug);
            settings.logLevel = 0;
            Logger::console("setting loglevel to 'debug'");
            writeEEPROM = true;
            break;
        case 1:
            Logger::setLoglevel(Logger::Info);
            settings.logLevel = 1;
            Logger::console("setting loglevel to 'info'");
            writeEEPROM = true;
            break;
        case 2:
            Logger::console("setting loglevel to 'warning'");
            settings.logLevel = 2;
            Logger::setLoglevel(Logger::Warn);
            writeEEPROM = true;
            break;
        case 3:
            Logger::console("setting loglevel to 'error'");
            settings.logLevel = 3;
            Logger::setLoglevel(Logger::Error);
            writeEEPROM = true;
            break;
        case 4:
            Logger::console("setting loglevel to 'off'");
            settings.logLevel = 4;
            Logger::setLoglevel(Logger::Off);
            writeEEPROM = true;
            break;
        }

    } else {
        Logger::console("Unknown command");
    }
    if (writeEEPROM) {
        EEPROM.write(EEPROM_ADDR, settings);
    }
    if (writeDigEE) {
        EEPROM.write(EEPROM_ADDR + 1024, digToggleSettings);
    }
}

//CAN0FILTER%i=%%i,%%i,%%i,%%i (ID, Mask, Extended, Enabled)", i);
bool SerialConsole::handleFilterSet(uint8_t bus, uint8_t filter, char *values)
{
    if (filter < 0 || filter > 7) return false;
    if (bus < 0 || bus > 1) return false;

    //there should be four tokens
    char *idTok = strtok(values, ",");
    char *maskTok = strtok(NULL, ",");
    char *extTok = strtok(NULL, ",");
    char *enTok = strtok(NULL, ",");

    if (!idTok) return false; //if any of them were null then something was wrong. Abort.
    if (!maskTok) return false;
    if (!extTok) return false;
    if (!enTok) return false;

    int idVal = strtol(idTok, NULL, 0);
    int maskVal = strtol(maskTok, NULL, 0);
    int extVal = strtol(extTok, NULL, 0);
    int enVal = strtol(enTok, NULL, 0);

    Logger::console("Setting CAN%iFILTER%i to ID 0x%x Mask 0x%x Extended %i Enabled %i", bus, filter, idVal, maskVal, extVal, enVal);

    if (bus == 0) {
        //settings.CAN0Filters[filter].id = idVal;
        //settings.CAN0Filters[filter].mask = maskVal;
        //settings.CAN0Filters[filter].extended = extVal;
        //settings.CAN0Filters[filter].enabled = enVal;
        //Can0.setRXFilter(filter, idVal, maskVal, extVal);
    } else if (bus == 1) {
        //settings.CAN1Filters[filter].id = idVal;
        //settings.CAN1Filters[filter].mask = maskVal;
        //settings.CAN1Filters[filter].extended = extVal;
        //settings.CAN1Filters[filter].enabled = enVal;
        //Can1.setRXFilter(filter, idVal, maskVal, extVal);
    }

    return true;
}

bool SerialConsole::handleCANSend(CANRaw &port, char *inputString)
{
    char *idTok = strtok(inputString, ",");
    char *lenTok = strtok(NULL, ",");
    char *dataTok;
    CAN_FRAME frame;

    if (!idTok) return false;
    if (!lenTok) return false;

    int idVal = strtol(idTok, NULL, 0);
    int lenVal = strtol(lenTok, NULL, 0);

    for (int i = 0; i < lenVal; i++) {
        dataTok = strtok(NULL, ",");
        if (!dataTok) return false;
        frame.data.byte[i] = strtol(dataTok, NULL, 0);
    }

    //things seem good so try to send the frame.
    frame.id = idVal;
    if (idVal >= 0x7FF) frame.extended = true;
    else frame.extended = false;
    frame.rtr = 0;
    frame.length = lenVal;
    port.sendFrame(frame);
    
    Logger::console("Sending frame with id: 0x%x len: %i", frame.id, frame.length);
    SysSettings.txToggle = !SysSettings.txToggle;
    setLED(SysSettings.LED_CANTX, SysSettings.txToggle);
    return true;
}

bool SerialConsole::handleSWCANSend(char *inputString)
{
    char *idTok = strtok(inputString, ",");
    char *lenTok = strtok(NULL, ",");
    char *dataTok;
    CAN_FRAME frame;

    if (!idTok) return false;
    if (!lenTok) return false;

    int idVal = strtol(idTok, NULL, 0);
    int lenVal = strtol(lenTok, NULL, 0);

    for (int i = 0; i < lenVal; i++) {
        dataTok = strtok(NULL, ",");
        if (!dataTok) return false;
        frame.data.byte[i] = strtol(dataTok, NULL, 0);
    }

    //things seem good so try to send the frame.
    frame.id = idVal;
    if (idVal >= 0x7FF) frame.extended = true;
    else frame.extended = false;
    frame.rtr = 0;
    frame.length = lenVal;
    SWCAN.sendFrame(frame);
    Logger::console("Sending frame with id: 0x%x len: %i", frame.id, frame.length);
    SysSettings.txToggle = !SysSettings.txToggle;
    setLED(SysSettings.LED_CANTX, SysSettings.txToggle);
    return true;
}

unsigned int SerialConsole::parseHexCharacter(char chr)
{
    unsigned int result = 0;
    if (chr >= '0' && chr <= '9') result = chr - '0';
    else if (chr >= 'A' && chr <= 'F') result = 10 + chr - 'A';
    else if (chr >= 'a' && chr <= 'f') result = 10 + chr - 'a';

    return result;
}

unsigned int SerialConsole::parseHexString(char *str, int length)
{
    unsigned int result = 0;
    for (int i = 0; i < length; i++) result += parseHexCharacter(str[i]) << (4 * (length - i - 1));
    return result;
}

//Tokenize cmdBuffer on space boundaries - up to 10 tokens supported
void SerialConsole::tokenizeCmdString() {
   int idx = 0;
   char *tok;
   
   for (int i = 0; i < 13; i++) tokens[i][0] = 0;
   
   tok = strtok(cmdBuffer, " ");
   if (tok != nullptr) strcpy(tokens[idx], tok);
       else tokens[idx][0] = 0;
   while (tokens[idx] != nullptr && idx < 13) {
       idx++;
       tok = strtok(nullptr, " ");
       if (tok != nullptr) strcpy(tokens[idx], tok);
            else tokens[idx][0] = 0;
   }
}

void SerialConsole::uppercaseToken(char *token) {
    int idx = 0;
    while (token[idx] != 0 && idx < 9) {
        token[idx] = toupper(token[idx]);
        idx++;
    }
    token[idx] = 0;
}

void SerialConsole::printBusName(int bus) {
    switch (bus) {
    case 0:
        SerialUSB.print("CAN0");
        break;
    case 1:
        SerialUSB.print("CAN1");
        break;
    case 2:
        SerialUSB.print("SWCAN");
        break;
    case 3:
        SerialUSB.print("LIN1");
        break;
    case 4:
        SerialUSB.print("LIN2");
        break;
    default:
        SerialUSB.print("UNKNOWN");
        break;
    }
}

//Expecting to find ID in tokens[2] then zero or more data bytes
bool SerialConsole::parseLawicelCANCmd(CAN_FRAME &frame) {
    if (tokens[2] == nullptr) return false;
    frame.id = strtol(tokens[2], nullptr, 16);
    int idx = 3;
    int dataLen = 0;
    while (tokens[idx] != nullptr) {
        frame.data.bytes[dataLen++] = strtol(tokens[idx], nullptr, 16);
        idx++;
    }
    frame.length = dataLen;
        
    return true;
}
