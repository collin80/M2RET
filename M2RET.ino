/*
 M2RET.ino

 Created: March 4, 2017
 Author: Collin Kidder

 Revision History:
    344    TonyD 25/11/2017.
        Updated support for Analogue background DMA updates, M2 Digital I/O as well as M2_12VIO library support in SYS_io.cpp 


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

 */

#include "config.h"
#include <due_can.h>
#include <Arduino_Due_SD_HSMCI.h> // This creates the object SD (HSMCI connected sdcard)
#include <due_wire.h>
#include <SPI.h>
#include <lin_stack.h>
#include <MCP2515_sw_can.h>
#include <M2_12VIO.h>
#include "ELM327_Emulator.h"

#include "EEPROM.h"
#include "SerialConsole.h"

#ifdef _M2IO
    #include "M2RET.h"
#else
    #undef _M2IO
#endif


/*
Notes on project:
This code should be autonomous after being set up. That is, you should be able to set it up
then disconnect it and move over to a car or other device to monitor, plug it in, and have everything
use the settings you set up without any external input.
*/

byte i = 0;

typedef struct {
    uint32_t bitsPerQuarter;
    uint32_t bitsSoFar;
    uint8_t busloadPercentage;
} BUSLOAD;

byte serialBuffer[SER_BUFF_SIZE];
int serialBufferLength = 0; //not creating a ring buffer. The buffer should be large enough to never overflow
uint32_t lastFlushMicros = 0;
BUSLOAD busLoad[2];
uint32_t busLoadTimer;
bool markToggle[6];
uint32_t lastMarkTrigger = 0;

EEPROMSettings settings;
SystemSettings SysSettings;
DigitalCANToggleSettings digToggleSettings;

//file system on sdcard (HSMCI connected)
FileStore FS;

SWcan SWCAN(SPI0_CS3, SWC_INT);

lin_stack LIN1(1, 0); // Sniffer
lin_stack LIN2(2, 0); // Sniffer

ELM327Emu elmEmulator;

#ifdef _M2IO
    M2_12VIO M2IO;  // M2_12VIO constructor !!!! DONT CHANGE !!!

    // !!! Uncomment to display M2_IO usage with M2RET Demonstration !!!
    //#define _M2IO_Demo

    #ifdef _M2IO_Demo
        uint32_t Display_Time = 0;   //Debugging M2_12VIO Analohue & digital demonstration usage in main loop()
    #else
        #undef _M2IO_Demo
    #endif
#else
    #undef _M2IO_Demo
#endif

SerialConsole console;

bool digTogglePinState;
uint8_t digTogglePinCounter;




void CANHandler() {
    SWCAN.intHandler();
}

//initializes all the system EEPROM values. Chances are this should be broken out a bit but
//there is only one checksum check for all of them so it's simple to do it all here.
void loadSettings()
{
    Logger::console("Loading settings....");

    EEPROM.read(EEPROM_ADDR, settings);

    if (settings.version != EEPROM_VER) { //if settings are not the current version then erase them and set defaults
        Logger::console("Resetting to factory defaults");
        settings.version = EEPROM_VER;
        settings.appendFile = false;
        settings.CAN0Speed = 500000;
        settings.CAN0_Enabled = true;
        settings.CAN1Speed = 500000;
        settings.CAN1_Enabled = false;
        settings.CAN0ListenOnly = false;
        settings.CAN1ListenOnly = false;
        settings.SWCAN_Enabled = false;
        settings.SWCANListenOnly = false; //TODO: Not currently respected or implemented.
        settings.SWCANSpeed = 33333;
        settings.LIN1_Enabled = false;
        settings.LIN2_Enabled = false;
        settings.LIN1Speed = 19200;
        settings.LIN2Speed = 19200;
        sprintf((char *)settings.fileNameBase, "CANBUS");
        sprintf((char *)settings.fileNameExt, "TXT");
        settings.fileNum = 1;
        settings.fileOutputType = CRTD;
        settings.useBinarySerialComm = false;
        settings.autoStartLogging = false;
        settings.logLevel = 1; //info
        settings.sysType = 0; //CANDUE as default
        settings.valid = 0; //not used right now
        EEPROM.write(EEPROM_ADDR, settings);
    } else {
        Logger::console("Using stored values from EEPROM");
        if (settings.CAN0ListenOnly > 1) settings.CAN0ListenOnly = 0;
        if (settings.CAN1ListenOnly > 1) settings.CAN1ListenOnly = 0;
    }

    EEPROM.read(EEPROM_ADDR + 1024, digToggleSettings);
    if (digToggleSettings.mode == 255) {
        Logger::console("Resetting digital toggling system to defaults");
        digToggleSettings.enabled = false;
        digToggleSettings.length = 0;
        digToggleSettings.mode = 0;
        digToggleSettings.pin = 1;
        digToggleSettings.rxTxID = 0x700;
        for (int c=0 ; c<8 ; c++) digToggleSettings.payload[c] = 0;
        EEPROM.write(EEPROM_ADDR + 1024, digToggleSettings);
    } else {
        Logger::console("Using stored values for digital toggling system");
    }

    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);

    SysSettings.SDCardInserted = false;

//    switch (settings.sysType) {
//    case 0:  //First gen M2 board
        Logger::console("Running on Macchina M2 hardware");
        SysSettings.useSD = true;
        SysSettings.logToFile = false;
        SysSettings.LED_CANTX = RGB_GREEN;
        SysSettings.LED_CANRX = RGB_BLUE;
        SysSettings.LED_LOGGING = RGB_RED;
        SysSettings.logToggle = false;
        SysSettings.txToggle = true;
        SysSettings.rxToggle = true;
        SysSettings.lawicelAutoPoll = false;
        SysSettings.lawicelMode = false;
        SysSettings.lawicellExtendedMode = false;
        SysSettings.lawicelTimestamping = false;
        SysSettings.numBuses = 3; //Currently we support CAN0, CAN1, SWCAN
        for (int rx = 0; rx < NUM_BUSES; rx++) SysSettings.lawicelBusReception[rx] = true; //default to showing messages on RX 
        //set pin mode for all LEDS
        pinMode(RGB_GREEN, OUTPUT);
        pinMode(RGB_RED, OUTPUT);
        pinMode(RGB_BLUE, OUTPUT);
        pinMode(DS2, OUTPUT);
        pinMode(DS3, OUTPUT);
        pinMode(DS4, OUTPUT);
        pinMode(DS5, OUTPUT);
        pinMode(DS6, OUTPUT);
        
        digitalWrite(SWC_M0, LOW); //Mode 0 for SWCAN
        digitalWrite(SWC_M1, LOW); //mode 1
        
        //Set RGB LED to completely off.
        digitalWrite(RGB_GREEN, HIGH);
        digitalWrite(RGB_BLUE, HIGH);
        digitalWrite(RGB_RED, HIGH);
        digitalWrite(DS2, HIGH);
        digitalWrite(DS3, HIGH);
        digitalWrite(DS4, HIGH);
        digitalWrite(DS5, HIGH);
        digitalWrite(DS6, HIGH);
        
        setSWCANSleep();
//        break;
//    }

//	if (settings.singleWireMode && settings.CAN1_Enabled) setSWCANEnabled();
//	else setSWCANSleep(); //start out setting single wire to sleep.

    busLoad[0].bitsSoFar = 0;
    busLoad[0].busloadPercentage = 0;
    busLoad[0].bitsPerQuarter = settings.CAN0Speed / 4;

    busLoad[1].bitsSoFar = 0;
    busLoad[1].busloadPercentage = 0;
    busLoad[1].bitsPerQuarter = settings.CAN1Speed / 4;

    busLoadTimer = millis();
}

void setSWCANSleep()
{
    SWCAN.mode(0);
}

void setSWCANEnabled()
{
    SWCAN.mode(3);
}

void setSWCANWakeup()
{
    SWCAN.mode(2);
}

void setup()
{
    //TODO: I don't remember why these two lines are here... anyone know?
    pinMode(XBEE_PWM, OUTPUT);
    digitalWrite(XBEE_PWM, LOW);

    //delay(5000); //just for testing. Don't use in production

    Serial.begin(115200);

    Wire.begin();
    SPI.begin();

    loadSettings();

    //settings.logLevel = 0; //Also just for testing. Dont use this in production either.
    //Logger::setLoglevel(Logger::Debug);

    if (SysSettings.useSD) {
        if (SD.Init()) {
            FS.Init();
            SysSettings.SDCardInserted = true;
            if (settings.autoStartLogging) {
                SysSettings.logToFile = true;
                Logger::info("Automatically logging to file.");
                //Logger::file("Starting File Logging.");
            }
        } else {
            Logger::error("SDCard not inserted. Cannot log to file!");
            SysSettings.SDCardInserted = false;
        }
    }

    SerialUSB.print("Build number: ");
    SerialUSB.println(CFG_BUILD_NUM);

    sys_early_setup();
    setup_sys_io();


    if (digToggleSettings.enabled) {
        SerialUSB.println("Digital Toggle System Enabled");
        if (digToggleSettings.mode & 1) { //input CAN and output pin state mode
            SerialUSB.println("In Output Mode");
            pinMode(digToggleSettings.pin, OUTPUT);
            if (digToggleSettings.mode & 0x80) {
                digitalWrite(digToggleSettings.pin, LOW);
                digTogglePinState = false;
            } else {
                digitalWrite(digToggleSettings.pin, HIGH);
                digTogglePinState = true;
            }
        } else { //read pin and output CAN mode
            SerialUSB.println("In Input Mode");
            pinMode(digToggleSettings.pin, INPUT);
            digTogglePinCounter = 0;
            if (digToggleSettings.mode & 0x80) digTogglePinState = false;
            else digTogglePinState = true;
        }
    }


    if (settings.CAN0_Enabled) {
        if (settings.CAN0ListenOnly) {
            Can0.setListenOnlyMode(true);
        } else {
            Can0.setListenOnlyMode(false);
        }
        Can0.enable();
        Can0.begin(settings.CAN0Speed, 255);
        SerialUSB.print("Enabled CAN0 with speed ");
        SerialUSB.println(settings.CAN0Speed);
    } else{
        Can0.disable();
    }

    if (settings.CAN1_Enabled) {
        if (settings.CAN1ListenOnly) {
            Can1.setListenOnlyMode(true);
        } else {
            Can1.setListenOnlyMode(false);
        }
        Can1.enable();
        Can1.begin(settings.CAN1Speed, 255);
        SerialUSB.print("Enabled CAN1 with speed ");
        SerialUSB.println(settings.CAN1Speed);        
    } else{
        Can1.disable();
    }

    if (settings.SWCAN_Enabled) {
        SWCAN.setupSW(settings.SWCANSpeed);       
        delay(20);
        SWCAN.mode(3); // Go to normal mode. 0 - Sleep, 1 - High Speed, 2 - High Voltage Wake-Up, 3 - Normal
        if (settings.SWCANListenOnly){
            SWCAN.setListenOnlyMode(true);
        }else{
            SWCAN.setListenOnlyMode(false);
        }
        attachInterrupt(SWC_INT, CANHandler, FALLING); //enable interrupt for SWCAN
        SerialUSB.print("Enabled SWCAN with speed ");
        SerialUSB.println(settings.SWCANSpeed);
        SWCAN.InitFilters(true);
    }

/*
    if (settings.LIN_Enabled) {
        LIN1.setSerial();
        LIN2.setSerial();
    } */
/*
    for (int i = 0; i < 7; i++) {
        if (settings.CAN0Filters[i].enabled) {
            Can0.setRXFilter(i, settings.CAN0Filters[i].id,
                             settings.CAN0Filters[i].mask, settings.CAN0Filters[i].extended);
        }
        if (settings.CAN1Filters[i].enabled) {
            Can1.setRXFilter(i, settings.CAN1Filters[i].id,
                             settings.CAN1Filters[i].mask, settings.CAN1Filters[i].extended);
        }
    }*/

    setPromiscuousMode();

    SysSettings.lawicelMode = false;
    SysSettings.lawicelAutoPoll = false;
    SysSettings.lawicelTimestamping = false;
    SysSettings.lawicelPollCounter = 0;

    for (int i = 0; i < MARK_LIMIT; i++) 
    {
        markToggle[i] = false;
        M2IO.InitButton_12VIO(i + 1);
    }
    
    //elmEmulator.setup();

    SerialUSB.print("Done with init\n");

#ifdef _M2IO
    M2IO.Init_12VIO();
    #ifdef _M2IO_Demo
        Display_Time = 0;
    #endif
#endif
}

void setPromiscuousMode()
{
    //By default there are 7 mailboxes for each device that are RX boxes
    //This sets each mailbox to have an open filter that will accept extended
    //or standard frames
    int filter;
    //extended
    for (filter = 0; filter < 3; filter++) {
        Can0.setRXFilter(filter, 0, 0, true);
        Can1.setRXFilter(filter, 0, 0, true);
    }
    //standard
    for (filter = 3; filter < 7; filter++) {
        Can0.setRXFilter(filter, 0, 0, false);
        Can1.setRXFilter(filter, 0, 0, false);
    }
}

//Get the value of XOR'ing all the bytes together. This creates a reasonable checksum that can be used
//to make sure nothing too stupid has happened on the comm.
uint8_t checksumCalc(uint8_t *buffer, int length)
{
    uint8_t valu = 0;
    for (int c = 0; c < length; c++) {
        valu ^= buffer[c];
    }
    return valu;
}

void addBits(int offset, CAN_FRAME &frame)
{
    if (offset < 0) return;
    if (offset > 1) return;
    busLoad[offset].bitsSoFar += 41 + (frame.length * 9);
    if (frame.extended) busLoad[offset].bitsSoFar += 18;
}

void sendFrame(CAN_COMMON *bus, CAN_FRAME &frame)
{
    int whichBus = 0;
    if (bus == &Can1) whichBus = 1;
    if (bus == &SWCAN) whichBus = 2;
    bus->sendFrame(frame);
    sendFrameToFile(frame, whichBus); //copy sent frames to file as well.
    addBits(whichBus, frame);
    toggleTXLED();
}

void toggleRXLED()
{
    static int counter = 0;
    counter++;
    if (counter >= BLINK_SLOWNESS) {
        counter = 0;
        SysSettings.rxToggle = !SysSettings.rxToggle;
        setLED(SysSettings.LED_CANRX, SysSettings.rxToggle);
    }
}

void toggleTXLED()
{
    static int counter = 0;
    counter++;
    if (counter >= BLINK_SLOWNESS) {
        counter = 0;
        SysSettings.txToggle = !SysSettings.txToggle;
        setLED(SysSettings.LED_CANTX, SysSettings.txToggle);
    }
}

/*
 * Pass bus load in percent 0 - 100
 * The 5 LEDs are Green, Yellow, Yellow, Yellow, Red
 * The values used for lighting up LEDs are very subjective
 * but here is the justification:
 * You want the first LED to light up if there is practically any
 * traffic at all so it comes on over 0% load - This tells the user that some traffic exists
 * The next few are timed to give the user some feedback that the load is increasing
 * and are somewhat logarithmic
 * The last LED comes on at 80% because busload really should never go over 80% for
 * proper functionality so lighing up red past that is the right move.
*/
void updateBusloadLED(uint8_t perc)
{
    Logger::debug("Busload: %i", perc);
    if (perc > 0) digitalWrite(DS6, LOW);
    else digitalWrite(DS6, HIGH);

    if (perc >= 14) digitalWrite(DS5, LOW);
    else digitalWrite(DS5, HIGH);

    if (perc >= 30) digitalWrite(DS4, LOW);
    else digitalWrite(DS4, HIGH);

    if (perc >= 53) digitalWrite(DS3, LOW);
    else digitalWrite(DS3, HIGH);

    if (perc >= 80) digitalWrite(DS2, LOW);
    else digitalWrite(DS2, HIGH);
}

void sendFrameToUSB(CAN_FRAME &frame, int whichBus)
{
    uint8_t buff[22];
    uint8_t temp;
    uint32_t now = micros();

    if (SysSettings.lawicelMode) {
        if (SysSettings.lawicellExtendedMode) {
            SerialUSB.print(micros());
            SerialUSB.print(" - ");
            SerialUSB.print(frame.id, HEX);            
            if (frame.extended) SerialUSB.print(" X ");
            else SerialUSB.print(" S ");
            console.printBusName(whichBus);
            for (int d = 0; d < frame.length; d++) {
                SerialUSB.print(" ");
                SerialUSB.print(frame.data.bytes[d], HEX);
            }
        }else {
            if (frame.extended) {
                SerialUSB.print("T");
                sprintf((char *)buff, "%08x", frame.id);
                SerialUSB.print((char *)buff);
            } else {
                SerialUSB.print("t");
                sprintf((char *)buff, "%03x", frame.id);
                SerialUSB.print((char *)buff);
            }
            SerialUSB.print(frame.length);
            for (int i = 0; i < frame.length; i++) {
                sprintf((char *)buff, "%02x", frame.data.byte[i]);
                SerialUSB.print((char *)buff);
            }
            if (SysSettings.lawicelTimestamping) {
                uint16_t timestamp = (uint16_t)millis();
                sprintf((char *)buff, "%04x", timestamp);
                SerialUSB.print((char *)buff);
            }
        }
        SerialUSB.write(13);
    } else {
        if (settings.useBinarySerialComm) {
            if (frame.extended) frame.id |= 1 << 31;
            serialBuffer[serialBufferLength++] = 0xF1;
            serialBuffer[serialBufferLength++] = 0; //0 = canbus frame sending
            serialBuffer[serialBufferLength++] = (uint8_t)(now & 0xFF);
            serialBuffer[serialBufferLength++] = (uint8_t)(now >> 8);
            serialBuffer[serialBufferLength++] = (uint8_t)(now >> 16);
            serialBuffer[serialBufferLength++] = (uint8_t)(now >> 24);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id & 0xFF);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 8);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 16);
            serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 24);
            serialBuffer[serialBufferLength++] = frame.length + (uint8_t)(whichBus << 4);
            for (int c = 0; c < frame.length; c++) {
                serialBuffer[serialBufferLength++] = frame.data.bytes[c];
            }
            //temp = checksumCalc(buff, 11 + frame.length);
            temp = 0;
            serialBuffer[serialBufferLength++] = temp;
            //SerialUSB.write(buff, 12 + frame.length);
        } else {
            SerialUSB.print(micros());
            SerialUSB.print(" - ");
            SerialUSB.print(frame.id, HEX);
            if (frame.extended) SerialUSB.print(" X ");
            else SerialUSB.print(" S ");
            SerialUSB.print(whichBus);
            SerialUSB.print(" ");
            SerialUSB.print(frame.length);
            for (int c = 0; c < frame.length; c++) {
                SerialUSB.print(" ");
                SerialUSB.print(frame.data.bytes[c], HEX);
            }
            SerialUSB.println();
        }
    }
}

void sendFrameToFile(CAN_FRAME &frame, int whichBus)
{
    uint8_t buff[40];
    //uint8_t temp;
    uint32_t timestamp;
    if (settings.fileOutputType == BINARYFILE) {
        if (frame.extended) frame.id |= 1 << 31;
        timestamp = micros();
        buff[0] = (uint8_t)(timestamp & 0xFF);
        buff[1] = (uint8_t)(timestamp >> 8);
        buff[2] = (uint8_t)(timestamp >> 16);
        buff[3] = (uint8_t)(timestamp >> 24);
        buff[4] = (uint8_t)(frame.id & 0xFF);
        buff[5] = (uint8_t)(frame.id >> 8);
        buff[6] = (uint8_t)(frame.id >> 16);
        buff[7] = (uint8_t)(frame.id >> 24);
        buff[8] = frame.length + (uint8_t)(whichBus << 4);
        for (int c = 0; c < frame.length; c++) {
            buff[9 + c] = frame.data.bytes[c];
        }
        Logger::fileRaw(buff, 9 + frame.length);
    } else if (settings.fileOutputType == GVRET) {
        sprintf((char *)buff, "%i,%x,%i,%i,%i", millis(), frame.id, frame.extended, whichBus, frame.length);
        Logger::fileRaw(buff, strlen((char *)buff));

        for (int c = 0; c < frame.length; c++) {
            sprintf((char *) buff, ",%x", frame.data.bytes[c]);
            Logger::fileRaw(buff, strlen((char *)buff));
        }
        buff[0] = '\r';
        buff[1] = '\n';
        Logger::fileRaw(buff, 2);
    } else if (settings.fileOutputType == CRTD) {
        int idBits = 11;
        if (frame.extended) idBits = 29;
        sprintf((char *)buff, "%f R%i %x", millis() / 1000.0f, idBits, frame.id);
        Logger::fileRaw(buff, strlen((char *)buff));

        for (int c = 0; c < frame.length; c++) {
            sprintf((char *) buff, " %x", frame.data.bytes[c]);
            Logger::fileRaw(buff, strlen((char *)buff));
        }
        buff[0] = '\r';
        buff[1] = '\n';
        Logger::fileRaw(buff, 2);
    }
}

void processDigToggleFrame(CAN_FRAME &frame)
{
    bool gotFrame = false;
    if (digToggleSettings.rxTxID == frame.id) {
        if (digToggleSettings.length == 0) gotFrame = true;
        else {
            gotFrame = true;
            for (int c = 0; c < digToggleSettings.length; c++) {
                if (digToggleSettings.payload[c] != frame.data.byte[c]) {
                    gotFrame = false;
                    break;
                }
            }
        }
    }

    if (gotFrame) { //then toggle output pin
        Logger::console("Got special digital toggle frame. Toggling the output!");
        digitalWrite(digToggleSettings.pin, digTogglePinState?LOW:HIGH);
        digTogglePinState = !digTogglePinState;
    }
}

void sendDigToggleMsg()
{
    CAN_FRAME frame;
    SerialUSB.println("Got digital input trigger.");
    frame.id = digToggleSettings.rxTxID;
    if (frame.id > 0x7FF) frame.extended = true;
    else frame.extended = false;
    frame.length = digToggleSettings.length;
    for (int c = 0; c < frame.length; c++) frame.data.byte[c] = digToggleSettings.payload[c];
    if (digToggleSettings.mode & 2) {
        SerialUSB.println("Sending digital toggle message on CAN0");
        sendFrame(&Can0, frame);
    }
    if (digToggleSettings.mode & 4) {
        SerialUSB.println("Sending digital toggle message on CAN1");
        sendFrame(&Can1, frame);
    }
}

/*
Send a fake frame out USB and maybe to file to show where the mark was triggered at. The fake frame has bits 31 through 3
set which can never happen in reality since frames are either 11 or 29 bit IDs. So, this is a sign that it is a mark frame
and not a real frame. The bottom three bits specify which mark triggered.
*/
void sendMarkTriggered(int which)
{
    CAN_FRAME frame;
    frame.id = 0xFFFFFFF8ull + which;
    frame.extended = true;
    frame.length = 0;
    frame.rtr = 0;
    sendFrameToUSB(frame, 0);
    if (SysSettings.logToFile) sendFrameToFile(frame, 0);
}

/*
Loop executes as often as possible all the while interrupts fire in the background.
The serial comm protocol is as follows:
All commands start with 0xF1 this helps to synchronize if there were comm issues
Then the next byte specifies which command this is.
Then the command data bytes which are specific to the command
Lastly, there is a checksum byte just to be sure there are no missed or duped bytes
Any bytes between checksum and 0xF1 are thrown away

Yes, this should probably have been done more neatly but this way is likely to be the
fastest and safest with limited function calls
*/
void loop()
{
    //static int loops = 0;
    CAN_FRAME incoming;
    static CAN_FRAME build_out_frame;
    static int out_bus;
    int in_byte;
    static byte buff[20];
    static int step = 0;
    static STATE state = IDLE;
    static uint32_t build_int;
    uint8_t temp8;
    uint16_t temp16;
    //uint32_t temp32;    
    bool isConnected = false;
    int serialCnt;
    uint32_t now = micros();

    if (millis() > (busLoadTimer + 250)) {
        busLoadTimer = millis();
        busLoad[0].busloadPercentage = ((busLoad[0].busloadPercentage * 3) + (((busLoad[0].bitsSoFar * 1000) / busLoad[0].bitsPerQuarter) / 10)) / 4;
        busLoad[1].busloadPercentage = ((busLoad[1].busloadPercentage * 3) + (((busLoad[1].bitsSoFar * 1000) / busLoad[1].bitsPerQuarter) / 10)) / 4;
        //Force busload percentage to be at least 1% if any traffic exists at all. This forces the LED to light up for any traffic.
        if (busLoad[0].busloadPercentage == 0 && busLoad[0].bitsSoFar > 0) busLoad[0].busloadPercentage = 1;
        if (busLoad[1].busloadPercentage == 0 && busLoad[1].bitsSoFar > 0) busLoad[1].busloadPercentage = 1;
        busLoad[0].bitsPerQuarter = settings.CAN0Speed / 4;
        busLoad[1].bitsPerQuarter = settings.CAN1Speed / 4;
        busLoad[0].bitsSoFar = 0;
        busLoad[1].bitsSoFar = 0;
        if(busLoad[0].busloadPercentage > busLoad[1].busloadPercentage){
            updateBusloadLED(busLoad[0].busloadPercentage);
        } else{
            updateBusloadLED(busLoad[1].busloadPercentage);
        }
    }

    /*if (SerialUSB)*/ isConnected = true;

    for (int i = 0; i < MARK_LIMIT; i++)
    {
        if ((lastMarkTrigger + 100) < millis()) //prevent jitter on switch closing
        {
            if (M2IO.GetButton_12VIO(i + 1)) {
    	        if (!markToggle[i]) {
    		        markToggle[i] = true;
                    lastMarkTrigger = millis();
    		        if (!settings.useBinarySerialComm) 
                    {
                        Logger::info("MARK %i TRIGGERED", i);
                    }
    		        else
    		        {
                        sendMarkTriggered(i);
    		        }
    	        }
            }
            else 
            {
                if (markToggle[i]) lastMarkTrigger = millis(); //causes it to also not trigger on jitter when switch opens
                markToggle[i] = false;
            }
        }
    }

    //if (!SysSettings.lawicelMode || SysSettings.lawicelAutoPoll || SysSettings.lawicelPollCounter > 0)
    //{
    if (Can0.available() > 0) {
        Can0.read(incoming);
        addBits(0, incoming);
        toggleRXLED();
        if (isConnected) sendFrameToUSB(incoming, 0);
        if (SysSettings.logToFile) sendFrameToFile(incoming, 0);
        if (digToggleSettings.enabled && (digToggleSettings.mode & 1) && (digToggleSettings.mode & 2)) processDigToggleFrame(incoming);
    }

    if (Can1.available() > 0) {
        Can1.read(incoming);
        addBits(1, incoming);
        toggleRXLED();
        if (isConnected) sendFrameToUSB(incoming, 1);
        if (digToggleSettings.enabled && (digToggleSettings.mode & 1) && (digToggleSettings.mode & 4)) processDigToggleFrame(incoming);
        if (SysSettings.logToFile) sendFrameToFile(incoming, 1);
    }
    
    if (SWCAN.GetRXFrame(incoming)) {
        toggleRXLED();
        if (isConnected) sendFrameToUSB(incoming, 2);
        //TODO: Maybe support digital toggle system on swcan too.
        if (SysSettings.logToFile) sendFrameToFile(incoming, 2);      
    }

    
    if (SysSettings.lawicelPollCounter > 0) SysSettings.lawicelPollCounter--;
    //}

    if (digToggleSettings.enabled && !(digToggleSettings.mode & 1)) {
        if (digTogglePinState) { //pin currently high. Look for it going low
            if (!digitalRead(digToggleSettings.pin)) digTogglePinCounter++; //went low, increment debouncing counter
            else digTogglePinCounter = 0; //whoops, it bounced or never transitioned, reset counter to 0

            if (digTogglePinCounter > 3) { //transitioned to LOW for 4 checks in a row. We'll believe it then.
                digTogglePinState = false;
                sendDigToggleMsg();
            }
        } else { //pin currently low. Look for it going high
            if (digitalRead(digToggleSettings.pin)) digTogglePinCounter++; //went high, increment debouncing counter
            else digTogglePinCounter = 0; //whoops, it bounced or never transitioned, reset counter to 0

            if (digTogglePinCounter > 3) { //transitioned to HIGH for 4 checks in a row. We'll believe it then.
                digTogglePinState = true;
                sendDigToggleMsg();
            }
        }
    }

    //delay(100);

    if (micros() - lastFlushMicros > SER_BUFF_FLUSH_INTERVAL) {
        if (serialBufferLength > 0) {
            SerialUSB.write(serialBuffer, serialBufferLength);
            serialBufferLength = 0;
            lastFlushMicros = micros();
        }
    }

    serialCnt = 0;
    while (isConnected && (SerialUSB.available() > 0) && serialCnt < 128) {
        serialCnt++;
        in_byte = SerialUSB.read();
        switch (state) {
            case IDLE:{
                    if(in_byte == 0xF1){
                        state = GET_COMMAND;
                    }else if(in_byte == 0xE7){
                        settings.useBinarySerialComm = true;
                        SysSettings.lawicelMode = false;
                        setPromiscuousMode(); //going into binary comm will set promisc. mode too.
                    } else{
                        console.rcvCharacter((uint8_t) in_byte);
                    }
                    break;
                }
            case GET_COMMAND:{
                    switch(in_byte){
                        case PROTO_BUILD_CAN_FRAME:{
                                state = BUILD_CAN_FRAME;
                                buff[0] = 0xF1;
                                step = 0;
                                break;
                            }
                        case PROTO_TIME_SYNC:{
                                state = TIME_SYNC;
                                step = 0;
                                buff[0] = 0xF1;
                                buff[1] = 1; //time sync
                                buff[2] = (uint8_t) (now & 0xFF);
                                buff[3] = (uint8_t) (now >> 8);
                                buff[4] = (uint8_t) (now >> 16);
                                buff[5] = (uint8_t) (now >> 24);
                                SerialUSB.write(buff, 6);
                                break;
                            }
                        case PROTO_DIG_INPUTS:{
                                //immediately return the data for digital inputs
                                temp8 = getDigital(0) + (getDigital(1) << 1) + (getDigital(2) << 2) + (getDigital(3) << 3) + (getDigital(4) << 4) + (getDigital(5) << 5);
                                buff[0] = 0xF1;
                                buff[1] = 6; //digital inputs
                                buff[2] = temp8;
                                temp8 = checksumCalc(buff, 2);
                                buff[3] = temp8;
                                SerialUSB.write(buff, 4);
                                state = IDLE;
                                break;
                            }
                        case PROTO_ANA_INPUTS:{
                                //immediately return data on analog inputs
                                temp16 = getAnalog(0);  // Analogue input 1
                                buff[0] = 0xF1;
                                buff[1] = 3;
                                buff[2] = temp16 & 0xFF;
                                buff[3] = uint8_t(temp16 >> 8);
                                temp16 = getAnalog(1);  // Analogue input 2
                                buff[4] = temp16 & 0xFF;
                                buff[5] = uint8_t(temp16 >> 8);
                                temp16 = getAnalog(2);  // Analogue input 3
                                buff[6] = temp16 & 0xFF;
                                buff[7] = uint8_t(temp16 >> 8);
                                temp16 = getAnalog(3);  // Analogue input 4
                                buff[8] = temp16 & 0xFF;
                                buff[9] = uint8_t(temp16 >> 8);
                                temp16 = getAnalog(4);  // Analogue input 5
                                buff[10] = temp16 & 0xFF;
                                buff[11] = uint8_t(temp16 >> 8);
                                temp16 = getAnalog(5);  // Analogue input 6
                                buff[12] = temp16 & 0xFF;
                                buff[13] = uint8_t(temp16 >> 8);
                                temp16 = getAnalog(6);  // Vehicle Volts
                                buff[14] = temp16 & 0xFF;
                                buff[15] = uint8_t(temp16 >> 8);
                                temp8 = checksumCalc(buff, 9);
                                buff[16] = temp8;
                                SerialUSB.write(buff, 17);
                                state = IDLE;
                                break;
                            }
                        case PROTO_SET_DIG_OUT:{
                                state = SET_DIG_OUTPUTS;
                                buff[0] = 0xF1;
                                break;
                            }
                        case PROTO_SETUP_CANBUS:{
                                state = SETUP_CANBUS;
                                step = 0;
                                buff[0] = 0xF1;
                                break;
                            }
                        case PROTO_GET_CANBUS_PARAMS:{
                                //immediately return data on canbus params
                                buff[0] = 0xF1;
                                buff[1] = 6;
                                buff[2] = settings.CAN0_Enabled + ((unsigned char) settings.CAN0ListenOnly << 4);
                                buff[3] = settings.CAN0Speed;
                                buff[4] = settings.CAN0Speed >> 8;
                                buff[5] = settings.CAN0Speed >> 16;
                                buff[6] = settings.CAN0Speed >> 24;
                                buff[7] = settings.CAN1_Enabled + ((unsigned char) settings.CAN1ListenOnly << 4); //+ (unsigned char)settings.singleWireMode << 6;
                                buff[8] = settings.CAN1Speed;
                                buff[9] = settings.CAN1Speed >> 8;
                                buff[10] = settings.CAN1Speed >> 16;
                                buff[11] = settings.CAN1Speed >> 24;
                                SerialUSB.write(buff, 12);
                                state = IDLE;
                                break;
                            }
                        case PROTO_GET_DEV_INFO:{
                                //immediately return device information
                                buff[0] = 0xF1;
                                buff[1] = 7;
                                buff[2] = CFG_BUILD_NUM & 0xFF;
                                buff[3] = (CFG_BUILD_NUM >> 8);
                                buff[4] = EEPROM_VER;
                                buff[5] = (unsigned char) settings.fileOutputType;
                                buff[6] = (unsigned char) settings.autoStartLogging;
                                buff[7] = 0; //was single wire mode. Should be rethought for this board.
                                SerialUSB.write(buff, 8);
                                state = IDLE;
                                break;
                            }
                        case PROTO_SET_SW_MODE:{
                                buff[0] = 0xF1;
                                state = SET_SINGLEWIRE_MODE;
                                step = 0;
                                break;
                            }
                        case PROTO_KEEPALIVE:{
                                buff[0] = 0xF1;
                                buff[1] = 0x09;
                                buff[2] = 0xDE;
                                buff[3] = 0xAD;
                                SerialUSB.write(buff, 4);
                                state = IDLE;
                                break;
                            }
                        case PROTO_SET_SYSTYPE:{
                                buff[0] = 0xF1;
                                state = SET_SYSTYPE;
                                step = 0;
                                break;
                            }
                        case PROTO_ECHO_CAN_FRAME:{
                                state = ECHO_CAN_FRAME;
                                buff[0] = 0xF1;
                                step = 0;
                                break;
                            }
                        case PROTO_GET_NUMBUSES:{
                                buff[0] = 0xF1;
                                buff[1] = 12;
                                buff[2] = 3; //number of buses actually supported by this hardware (TODO: will be 5 eventually)
                                SerialUSB.write(buff, 3);
                                state = IDLE;
                                break;
                            }
                        case PROTO_GET_EXT_BUSES:{
                                buff[0] = 0xF1;
                                buff[1] = 13;
                                buff[2] = settings.SWCAN_Enabled + ((unsigned char) settings.SWCANListenOnly << 4);
                                buff[3] = settings.SWCANSpeed;
                                buff[4] = settings.SWCANSpeed >> 8;
                                buff[5] = settings.SWCANSpeed >> 16;
                                buff[6] = settings.SWCANSpeed >> 24;
                                buff[7] = settings.LIN1_Enabled;
                                buff[8] = settings.LIN1Speed;
                                buff[9] = settings.LIN1Speed >> 8;
                                buff[10] = settings.LIN1Speed >> 16;
                                buff[11] = settings.LIN1Speed >> 24;
                                buff[12] = settings.LIN2_Enabled;
                                buff[13] = settings.LIN2Speed;
                                buff[14] = settings.LIN2Speed >> 8;
                                buff[15] = settings.LIN2Speed >> 16;
                                buff[16] = settings.LIN2Speed >> 24;
                                SerialUSB.write(buff, 17);
                                state = IDLE;
                                break;
                            }
                        case PROTO_SET_EXT_BUSES:{
                                state = SETUP_EXT_BUSES;
                                step = 0;
                                buff[0] = 0xF1;
                                break;
                            }
                    }
                    break;
                }
            case BUILD_CAN_FRAME:{
                    buff[1 + step] = in_byte;
                    switch(step){
                        case 0:{
                                build_out_frame.id = in_byte;
                                break;
                            }
                        case 1:{
                                build_out_frame.id |= in_byte << 8;
                                break;
                            }
                        case 2:{
                                build_out_frame.id |= in_byte << 16;
                                break;
                            }
                        case 3:{
                                build_out_frame.id |= in_byte << 24;
                                if(build_out_frame.id & 1 << 31){
                                    build_out_frame.id &= 0x7FFFFFFF;
                                    build_out_frame.extended = true;
                                } else build_out_frame.extended = false;
                                break;
                            }
                        case 4:{
                                out_bus = in_byte & 3;
                                break;
                            }
                        case 5:{
                                build_out_frame.length = in_byte & 0xF;
                                if(build_out_frame.length > 8) build_out_frame.length = 8;
                                break;
                            }
                        default:{
                                if(step < build_out_frame.length + 6)
                                {
                                    build_out_frame.data.bytes[step - 6] = in_byte;
                                } 
                                else
                                {
                                    state = IDLE;
                                    //this would be the checksum byte. Compute and compare.
                                    temp8 = checksumCalc(buff, step);
                                    if ((build_out_frame.id == 0x100) && (out_bus == 2))
                                    {
                                        SWCAN.mode(1);
                                        delay(1);
                                    }                                    
                                    build_out_frame.rtr = 0;
                                    if(out_bus == 0) sendFrame(&Can0, build_out_frame);
                                    if(out_bus == 1) sendFrame(&Can1, build_out_frame);
                                    if(out_bus == 2) sendFrame(&SWCAN, build_out_frame);
                                    
                                    if ((build_out_frame.id == 0x100) && (out_bus == 2))
                                    {
                                        delay(1);
                                        SWCAN.mode(3);   
                                    }
                                }
                                break;
                            }
                    }
                    step++;
                    break;
                }
            case TIME_SYNC:{
                    state = IDLE;
                    break;
                }
            case GET_DIG_INPUTS:{
                    // nothing to do
                    break;
                }
            case GET_ANALOG_INPUTS:{
                    // nothing to do
                    break;
                }
            case SET_DIG_OUTPUTS:{ //todo: validate the XOR byte
                    buff[1] = in_byte;
                    //temp8 = checksumCalc(buff, 2);
                    for(int c = 0; c < 8; c++){
                        if(in_byte & (1 << c)) setOutput(c, true);
                        else setOutput(c, false);
                    }
                    state = IDLE;
                    break;
                }
            case SETUP_CANBUS:{ //todo: validate checksum
                switch(step){
                    case 0:{
                            build_int = in_byte;
                            break;
                        }
                    case 1:{
                            build_int |= in_byte << 8;
                            break;
                        }
                    case 2:{
                            build_int |= in_byte << 16;
                            break;
                        }
                    case 3:{
                            build_int |= in_byte << 24;
                            if(build_int > 0){
                                if(build_int & 0x80000000){ //signals that enabled and listen only status are also being passed
                                    if(build_int & 0x40000000){
                                        settings.CAN0_Enabled = true;
                                        Can0.enable();
                                    } else{
                                        settings.CAN0_Enabled = false;
                                        Can0.disable();
                                    }
                                    if(build_int & 0x20000000){
                                        settings.CAN0ListenOnly = true;
                                        Can0.enable_autobaud_listen_mode();
                                    } else{
                                        settings.CAN0ListenOnly = false;
                                        Can0.disable_autobaud_listen_mode();
                                    }
                                } else{
                                    Can0.enable(); //if not using extended status mode then just default to enabling - this was old behavior
                                    settings.CAN0_Enabled = true;
                                }
                                build_int = build_int & 0xFFFFF;
                                if(build_int > 1000000) build_int = 1000000;
                                Can0.begin(build_int, 255);
                                //Can0.set_baudrate(build_int);
                                settings.CAN0Speed = build_int;
                            } else{ //disable first canbus
                                Can0.disable();
                                settings.CAN0_Enabled = false;
                            }
                            break;
                        }
                    case 4:{
                            build_int = in_byte;
                            break;
                        }
                    case 5:{
                            build_int |= in_byte << 8;
                            break;
                        }
                    case 6:{
                            build_int |= in_byte << 16;
                            break;
                        }
                    case 7:{
                            build_int |= in_byte << 24;
                            if(build_int > 0){
                                if(build_int & 0x80000000){ //signals that enabled and listen only status are also being passed
                                    if(build_int & 0x40000000){
                                        settings.CAN1_Enabled = true;
                                        Can1.enable();
                                    } else{
                                        settings.CAN1_Enabled = false;
                                        Can1.disable();
                                    }
                                    if(build_int & 0x20000000){
                                        settings.CAN1ListenOnly = true;
                                        Can1.enable_autobaud_listen_mode();
                                    } else{
                                        settings.CAN1ListenOnly = false;
                                        Can1.disable_autobaud_listen_mode();
                                    }
                                } else{
                                    Can1.enable(); //if not using extended status mode then just default to enabling - this was old behavior
                                    settings.CAN1_Enabled = true;
                                }
                                build_int = build_int & 0xFFFFF;
                                if(build_int > 1000000) build_int = 1000000;
                                Can1.begin(build_int, 255);
                                //Can1.set_baudrate(build_int);

                                settings.CAN1Speed = build_int;
                            } else{ //disable second canbus
                                Can1.disable();
                                settings.CAN1_Enabled = false;
                            }
                            state = IDLE;
                            //now, write out the new canbus settings to EEPROM
                            EEPROM.write(EEPROM_ADDR, settings);
                            setPromiscuousMode();
                            break;
                        }
                }
                step++;
                break;
            }
            case GET_CANBUS_PARAMS:{
                    // nothing to do
                    break;
                }
            case GET_DEVICE_INFO:{
                    // nothing to do
                    break;
                }
            case SET_SINGLEWIRE_MODE:{
                    if(in_byte == 0x10){
                    } else{
                    }
                    EEPROM.write(EEPROM_ADDR, settings);
                    state = IDLE;
                    break;
                }
            case SET_SYSTYPE:{
                    settings.sysType = in_byte;
                    EEPROM.write(EEPROM_ADDR, settings);
                    loadSettings();
                    state = IDLE;
                    break;
                }
            case ECHO_CAN_FRAME:{
                    buff[1 + step] = in_byte;
                    switch(step){
                        case 0:{
                                build_out_frame.id = in_byte;
                                break;
                            }
                        case 1:{
                                build_out_frame.id |= in_byte << 8;
                                break;
                            }
                        case 2:{
                                build_out_frame.id |= in_byte << 16;
                                break;
                            }
                        case 3:{
                                build_out_frame.id |= in_byte << 24;
                                if(build_out_frame.id & 1 << 31){
                                    build_out_frame.id &= 0x7FFFFFFF;
                                    build_out_frame.extended = true;
                                } else build_out_frame.extended = false;
                                break;
                            }
                        case 4:{
                                out_bus = in_byte & 1;
                                break;
                            }
                        case 5:{
                                build_out_frame.length = in_byte & 0xF;
                                if(build_out_frame.length > 8) build_out_frame.length = 8;
                                break;
                            }
                        default:{
                                if(step < build_out_frame.length + 6){
                                    build_out_frame.data.bytes[step - 6] = in_byte;
                                } else{
                                    state = IDLE;
                                    //this would be the checksum byte. Compute and compare.
                                    temp8 = checksumCalc(buff, step);
                                    //if (temp8 == in_byte)
                                    //{
                                    toggleRXLED();
                                    if(isConnected) sendFrameToUSB(build_out_frame, 0);
                                    //}
                                }
                                break;
                            }
                    }
                    step++;
                    break;
                }
            case SETUP_EXT_BUSES:{ //setup enable/listenonly/speed for SWCAN, Enable/Speed for LIN1, LIN2
                    switch(step){
                        case 0:{
                                build_int = in_byte;
                                break;
                            }
                        case 1:{
                                build_int |= in_byte << 8;
                                break;
                            }
                        case 2:{
                                build_int |= in_byte << 16;
                                break;
                            }
                        case 3:{
                                build_int |= in_byte << 24;
                                if(build_int > 0){
                                    if(build_int & 0x80000000){ //signals that enabled and listen only status are also being passed
                                        if(build_int & 0x40000000){
                                            settings.SWCAN_Enabled = true;
                                            SWCAN.mode(3);
                                        } else{
                                            settings.SWCAN_Enabled = false;
                                            SWCAN.mode(0);
                                        }
                                        if(build_int & 0x20000000){
                                            settings.SWCANListenOnly = true;
                                            //SWCAN.enable_autobaud_listen_mode();
                                        } else{
                                            settings.SWCANListenOnly = false;
                                            //SWCAN.disable_autobaud_listen_mode();
                                        }
                                    } else{
                                        SWCAN.mode(3);
                                        settings.SWCAN_Enabled = true;
                                    }
                                    build_int = build_int & 0xFFFFF;
                                    if(build_int > 100000) build_int = 100000;
                                    SWCAN.setupSW(build_int);
                                    delay(20);
                                    SWCAN.mode(3); // Go to normal mode. 0 - Sleep, 1 - High Speed, 2 - High Voltage Wake-Up, 3 - Normal
                                    attachInterrupt(SWC_INT, CANHandler, FALLING); //enable interrupt for SWCAN
                                    settings.SWCANSpeed = build_int;
                                } else{ //disable first canbus
                                    SWCAN.mode(0);
                                    settings.SWCAN_Enabled = false;
                                }
                                break;
                            }
                        case 4:{
                                build_int = in_byte;
                                break;
                            }
                        case 5:{
                                build_int |= in_byte << 8;
                                break;
                            }
                        case 6:{
                                build_int |= in_byte << 16;
                                break;
                            }
                        case 7:{
                                build_int |= in_byte << 24;
                                /* FIX THIS UP TO INITIALIZE LIN1
                                if (build_int > 0) {
                                    if (build_int & 0x80000000) { //signals that enabled and listen only status are also being passed
                                        if (build_int & 0x40000000) {
                                            settings.CAN1_Enabled = true;
                                            Can1.enable();
                                        } else {
                                            settings.CAN1_Enabled = false;
                                            Can1.disable();
                                        }
                                    } else {
                                        Can1.enable(); //if not using extended status mode then just default to enabling - this was old behavior
                                        settings.CAN1_Enabled = true;
                                    }
                                    build_int = build_int & 0xFFFFF;
                                    if (build_int > 1000000) build_int = 1000000;
                                    Can1.begin(build_int, 255);
                                    //Can1.set_baudrate(build_int);
                                    settings.CAN1Speed = build_int;
                                } else { //disable second canbus
                                    Can1.disable();
                                    settings.CAN1_Enabled = false;
                                }*/
                                break;
                            }
                        case 8:{
                                build_int = in_byte;
                                break;
                            }
                        case 9:{
                                build_int |= in_byte << 8;
                                break;
                            }
                        case 10:{
                                build_int |= in_byte << 16;
                                break;
                            }
                        case 11:{
                                build_int |= in_byte << 24;
                                /* FIX THIS UP TO INITIALIZE LIN2
                                if (build_int > 0) {
                                    if (build_int & 0x80000000) { //signals that enabled and listen only status are also being passed
                                        if (build_int & 0x40000000) {
                                            settings.CAN1_Enabled = true;
                                            Can1.enable();
                                        } else {
                                            settings.CAN1_Enabled = false;
                                            Can1.disable();
                                        }
                                    } else {
                                        Can1.enable(); //if not using extended status mode then just default to enabling - this was old behavior
                                        settings.CAN1_Enabled = true;
                                    }
                                    build_int = build_int & 0xFFFFF;
                                    if (build_int > 1000000) build_int = 1000000;
                                    Can1.begin(build_int, 255);
                                    //Can1.set_baudrate(build_int);
                                    settings.CAN1Speed = build_int;
                                } else { //disable second canbus
                                    Can1.disable();
                                    settings.CAN1_Enabled = false;
                                } */
                                state = IDLE;
                                //now, write out the new canbus settings to EEPROM
                                EEPROM.write(EEPROM_ADDR, settings);
                                //setPromiscuousMode();
                                break;
                            }
                    }
                    step++;
                    break;
                }
            }
    }
    Logger::loop();
    elmEmulator.loop();

#ifdef _M2IO
    #ifdef _M2IO_Demo
        // !!!! M2_12VIO demonstration with M2RET !!!!
    if(millis() > (Display_Time + SER_BUFF_FLUSH_INTERVAL)){   //Debugging sys_io
        Display_Time = millis();

        SerialUSB.print("\nTemp ");
        SerialUSB.print(M2IO.Temperature());
        SerialUSB.print("C");

        SerialUSB.print("   Vehicle Volts ");
        SerialUSB.print(((float)M2IO.Supply_Volts()) / 1000,2);
        SerialUSB.print("V");

        SerialUSB.print("   Analogue_1 ");
        SerialUSB.print(((float) M2IO.Read_12VIO(1)) / 1000, 2);
        SerialUSB.print("V");

        SerialUSB.print("   Load Amps ");
        SerialUSB.print(((float) M2IO.Load_Amps()) / 100, 2);
        SerialUSB.print("A");

        SerialUSB.print("\n");
    }
    #endif
#endif
}

