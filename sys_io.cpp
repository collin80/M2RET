/*
 * sys_io.cpp
 *
 * Handles the low level details of system I/O
 *
Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

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

some portions based on code credited as:
Arduino Due ADC->DMA->USB 1MSPS
by stimmer

*/


/*
 *Revision History:
 *    344    TonyD 25/11/2017.
 *       Updated support for Analogue background DMA updates, M2 Digital I/O as well as M2_12VIO library support in SYS_io.cpp 
 *       Analogue & Digital Functions for Macchina M2 corrected & now working
 *
 * 
 */

#include "sys_io.h"
#include <M2_12VIO.h>

bool useRawADC = false;

#ifdef _M2IO
extern M2_12VIO M2IO;
#endif

// !!! Undefine to use user sequence registers in DMA mode !!!
//#define ADC_User_Seq  // not yet working correctly in DMA mode, once working will save approx 2.2ms CPU time & not need to unroll data from adc_buf[x][x] in getADCAvg()

#undef HID_ENABLED


uint8_t dig[NUM_DIGITAL];   //digital inputs. sudo/analogue inputs
uint8_t adc[NUM_ANALOG][2]; // [x][0]This holds the ADC Channel number. // This is calculated in sys_early_setup()
                            // [x][1]This holds the pointer of the ADC number position into the adc_buf[x][x] array (Now redundent).
                            // This is calculated in sys_early_setup()

uint8_t out[NUM_OUTPUT];    //digital output configuration details

#define Num_Samples 32U
uint32_t NumADCSamples = Num_Samples;
#define Buffer_Size NUM_ANALOG * Num_Samples
uint16_t Num_Samples_To_Take = Buffer_Size;   // 9-Analogue inputs * Num_Samples = 288
#define Num_Buffers 4U

volatile uint16_t bufn,obufn;   // Buffer pointers used in DMA
uint16_t adc_buf[Num_Buffers][Buffer_Size];   // 4 buffers of 288 samples used to store scanned samples (Acts like a FIFO buffer)

//uint16_t adc_values[NUM_ANALOG * 2];
uint16_t adc_out_vals[NUM_ANALOG];  //holds the summed values of the analogue inputs

uint32_t Enabled_Analogue_Pins = 0; // Sum of ADC input numbers to load into ADC_CHER
#ifdef ADC_User_Seq
    uint32_t SEQR1 = 0;
    uint32_t SEQR2 = 0;
#endif

//the ADC values fluctuate a lot so smoothing is required.
uint32_t adc_buffer[NUM_ANALOG][Num_Samples];
//uint8_t adc_pointer[NUM_ANALOG]; //pointer to next position to use

ADC_COMP adc_comp[NUM_ANALOG]; //holds ADC offset or gain


/*
When the ADC reads in the programmed # of readings it will do two things:
1. It loads the next buffer and buffer size into current buffer and size
2. It sends this interrupt
This interrupt then loads the "next" fields (ADC_RNPR & ADC_RNCR) with the proper values. This is
done with a nine position buffer. In this way the ADC is constantly sampling
and this happens virtually for free. It all happens in the background with
minimal CPU overhead.
*/
void ADC_Handler()      // move DMA pointers to next buffer
{
/*
26.1 Description
The Peripheral DMA Controller (PDC) transfers data between on-chip serial peripherals and the on- and/or off-chip
memories. The link between the PDC and a serial peripheral is operated by the AHB to APB bridge.
The user interface of each PDC channel is integrated into the user interface of the peripheral it serves. The user
interface of mono directional channels (receive only or transmit only), contains two 32-bit memory pointers and two
16-bit counters, one set (pointer, counter) for current transfer and one set (pointer, counter) for next transfer. The
bi-directional channel user interface contains four 32-bit memory pointers and four 16-bit counters. Each set
(pointer, counter) is used by current transmit, next transmit, current receive and next receive.
Using the PDC removes processor overhead by reducing its intervention during the transfer. This significantly
reduces the number of clock cycles required for a data transfer, which improves microcontroller performance.
!!!To launch a transfer, the peripheral triggers its associated PDC channels by using transmit and receive signals.
When the programmed data is transferred, an end of transfer interrupt is generated by the peripheral itself.
*/

/*
26.4.1 Configuration
The PDC channel user interface enables the user to configure and control data transfers for each channel. The
user interface of each PDC channel is integrated into the associated peripheral user interface.
The user interface of a serial peripheral, whether it is full or half duplex, contains four 32-bit pointers (RPR, RNPR,
TPR, TNPR) and four 16-bit counter registers (RCR, RNCR, TCR, TNCR). However, the transmit and receive
parts of each type are programmed differently: the transmit and receive parts of a full duplex peripheral can be
programmed at the same time, whereas only one part (transmit or receive) of a half duplex peripheral can be
programmed at a time.
32-bit pointers define the access location in memory for current and next transfer, whether it is for read (transmit)
or write (receive). 16-bit counters define the size of current and next transfers. It is possible, at any moment, to
read the number of transfers left for each channel.
!!!The PDC has dedicated status registers which indicate if the transfer is enabled or disabled for each channel. The
status for each channel is located in the associated peripheral status register. Transfers can be enabled and/or
disabled by setting TXTEN/TXTDIS and RXTEN/RXTDIS in the peripheral�s Transfer Control Register.
!!!At the end of a transfer, the PDC channel sends status flags to its associated peripheral. These flags are visible in
the peripheral status register (ENDRX, ENDTX, RXBUFF, and TXBUFE). Refer to Section 26.4.3 and to the
associated peripheral user interface.
*/

/*
26.4.2 Memory Pointers
Each full duplex peripheral is connected to the PDC by a receive channel and a transmit channel. Both channels
have 32-bit memory pointers that point respectively to a receive area and to a transmit area in on- and/or off-chip
memory.
Each half duplex peripheral is connected to the PDC by a bidirectional channel. This channel has two 32-bit
memory pointers, one for current transfer and the other for next transfer. These pointers point to transmit or
receive data depending on the operating mode of the peripheral.
Depending on the type of transfer (byte, half-word or word), the memory pointer is incremented respectively by 1,
2 or 4 bytes.
If a memory pointer address changes in the middle of a transfer, the PDC channel continues operating using the
new address.
*/

/*
26.4.3 Transfer Counters
Each channel has two 16-bit counters, one for current transfer and the other one for next transfer. These counters
define the size of data to be transferred by the channel. The current transfer counter is decremented first as the
data addressed by current memory pointer starts to be transferred. When the current transfer counter reaches
zero, the channel checks its next transfer counter. If the value of next counter is zero, the channel stops
transferring data and sets the appropriate flag. But if the next counter value is greater then zero, the values of the
next pointer/next counter are copied into the current pointer/current counter and the channel resumes the transfer
whereas next pointer/next counter get zero/zero as values. At the end of this transfer the PDC channel sets the
appropriate flags in the Peripheral Status Register.
The following list gives an overview of how status register flags behave depending on the counters� values:
. ENDRX flag is set when the PERIPH_RCR register reaches zero.
. RXBUFF flag is set when both PERIPH_RCR and PERIPH_RNCR reach zero.
. ENDTX flag is set when the PERIPH_TCR register reaches zero.
. TXBUFE flag is set when both PERIPH_TCR and PERIPH_TNCR reach zero.
These status flags are described in the Peripheral Status Register.
*/

/*
26.4.5 PDC Flags and Peripheral Status Register
This flag is set when PERIPH_RCR register reaches zero and the last data has been transferred to memory.
It is reset by writing a non zero value in PERIPH_RCR or PERIPH_RNCR.
*/

    uint32_t f = ADC->ADC_ISR;
    if(f & (3 << 27)){ //receive counter end of buffer

/*      // debugging purposes only enabling this will seriously delay the Interupt service routine
        SerialUSB.print("\nADC_ISR ");
        SerialUSB.print(f);
        SerialUSB.print("\nBuffer ");
        SerialUSB.print(obufn);
        SerialUSB.print("\n");
        for(uint8_t j = 0; j < 2; j++){
            for(uint8_t i = (NUM_ANALOG * j); i < ((NUM_ANALOG * j) + NUM_ANALOG); i++){
                SerialUSB.print(adc_buf[obufn][i]);
                SerialUSB.print(" ");
            }
            SerialUSB.print("\n");
        }
*/

        getADCAvg();

        obufn = bufn;
        bufn = (bufn + 1) & 3;
        ADC->ADC_RNPR = (uint32_t) adc_buf[bufn];
        ADC->ADC_RNCR = Num_Samples_To_Take;
    }
}

//forces the digital I/O ports to a safe state. This is called very early in initialization.
void sys_early_setup(){
    uint8_t i;


    dig[0] = 0;
    dig[1] = 1;
    dig[2] = 2;
    dig[3] = 3;
    dig[4] = 4;
    dig[5] = 5;

    adc[0][0] = Ana_In1;    // Analogue Input 1 (CH12)
    adc[0][1] = 7;  // pointer to position in adc_buf[x][adc[0][1]] Now Obselete if using ADC User sequence registers
    adc[1][0] = Ana_In2;    // Analogue Input 2 (CH11)
    adc[1][1] = 6;  // Now Obselete because we are ordering the ADC inputs in SEQR1 & SEQR2 in order of appeareance in this list top to bottom
    adc[2][0] = Ana_In3;    // Analogue Input 3 (CH0)
    adc[2][1] = 0;  // ditto as above Now Obselete
    adc[3][0] = Ana_In4;    // Analogue Input 4 (CH2)
    adc[3][1] = 2;  // ditto as above Now Obselete
    adc[4][0] = Ana_In5;    // Analogue Input 5 (CH1)
    adc[4][1] = 1;  // ditto as above Now Obselete
    adc[5][0] = Ana_In6;    // Analogue Input 6 (CH7)
    adc[5][1] = 4;  // ditto as above Now Obselete
    adc[6][0] = Ana_In7;    // Vehicle Volts (CH3)
    adc[6][1] = 3;  // ditto as above Now Obselete
    adc[7][0] = Ana_In8;    // M2 12V line Current draw AMPS (CH10)
    adc[7][1] = 5;  // ditto as above Now Obselete
    adc[8][0] = Ana_In9;    // M2 CPU Temperature (CH15)
    adc[8][1] = 8;  // ditto as above Now Obselete

    out[0] = Dig_Out1;
    out[1] = Dig_Out2;
    out[2] = Dig_Out3;
    out[3] = Dig_Out4;
    out[4] = Dig_Out5;
    out[5] = Dig_Out6;

    Enabled_Analogue_Pins = 0;
#ifdef ADC_User_Seq
    SEQR1 = SEQR2 = 0;  // Requires ADC_MR(USEQ) field is set to �1�.
    for(i = 0; i < NUM_ANALOG; i++){    // Setup the Analogue User Sequence ADC Scanning order 
        Enabled_Analogue_Pins += 1 << i;    // Analogue pins enabled 0-8 "0x01FF" used in setupFastADC()
        if(i <= 7){
            SEQR1 += adc[i][0] << (i * 4);  // Sequence order of the first 8 analogue inputs "0xA37120BC" used in setupFastADC()
        } else{
            SEQR2 += adc[i][0] << ((i - 8) * 4);  // Sequence order of the second 8 analogue inputs "0x0F" used in setupFastADC()
        }
    }
#else
    for(i = 0; i < NUM_ANALOG; i++){    // Setup the Analogue ADC Scanning order
        Enabled_Analogue_Pins += 1 << adc[i][0];
    }
#endif

#ifdef _M2IO
    // Using M2_12VIO Input pins have been setup when we called "M2IO.Init_12VIO();" in setup
#else
    // we are not using M2_12IO therfore set up the input pins

        // this is not required as we are using analogue inputs as sudo inputs
    //for (i = 0; i < NUM_DIGITAL; i++) pinMode(dig[i], INPUT); // Setup the Input pins

    for (i = 0; i < NUM_OUTPUT; i++) {  // Setup the Output pins
        if (out[i] != 255) {
            pinMode(out[i], OUTPUT);
            (i <= 2) ? digitalWrite(out[i], LOW) : digitalWrite(out[i], HIGH);
        }
    }
#endif
}

/*
Initialize DMA driven ADC and read in gain/offset for each channel
*/
void setup_sys_io()
{
    uint8_t i;

    setupFastADC();

    for (i = 0; i < NUM_ANALOG; i++) {
        for(uint8_t j = 0; j < NumADCSamples; j++){
            adc_buffer[i][j] = 0;
        }
        //adc_pointer[i] = 0;
        //adc_values[i] = 0;
        adc_out_vals[i] = 0;
    }
}

/*
Setup the system to continuously read the proper ADC channels and use DMA to place the results into RAM
Testing to find a good batch of settings for how fast to do ADC readings. The relevant areas:
1. In the adc_init call it is possible to use something other than ADC_FREQ_MAX to slow down the ADC clock
2. ADC_MR has a clock divisor, start up time, settling time, tracking time, and transfer time. These can be adjusted
*/
void setupFastADC(){
    pmc_enable_periph_clk(ID_ADC);
    adc_init(ADC, SystemCoreClock, ADC_FREQ_MIN, ADC_STARTUP_FAST); //just about to change a bunch of these parameters with the next command

#ifdef ADC_User_Seq
    ADC->ADC_SEQR1 = SEQR1; // User sequence ADC scan order first 8 ADC inputs
    ADC->ADC_SEQR2 = SEQR2; // User sequence ADC scan order 9th ADC input
    ADC->ADC_CHER = Enabled_Analogue_Pins; // These are the # of analogue pins we are reading from
#else
    ADC->ADC_CHER = Enabled_Analogue_Pins; //enable Analogue pins as configured in adc[x][0]
#endif

/*
The ADC uses the ADC Clock to perform conversions. Converting a single analog value to a 12-bit digital data
requires Tracking Clock cycles as defined in the field TRACKTIM of the �ADC Mode Register� on page 1333 and
Transfer Clock cycles as defined in the field TRANSFER of the same register. The ADC Clock frequency is
selected in the PRESCAL field of the Mode Register (ADC_MR). The tracking phase starts during the conversion
of the previous channel. If the tracking time is longer than the conversion time, the tracking phase is extended to
the end of the previous conversion.
*/

/*
The MCLK is 12MHz on our boards. The ADC can only run 1MHz so the prescaler must be at least 12x.
The ADC should take Tracking+Transfer for each read when it is set to switch channels with each read

Example:
(settling time clks + tranfer time clks) 5+7  = 12 clocks per read 1M / 12 = 83333 reads per second. For newer boards there are 4 channels interleaved
so, for each channel, the readings are (1000000/83333)*4channels = 48uS apart. 64 of these readings are averaged together for a total of 3ms
worth of ADC in each average. This is then averaged with the current value in the ADC buffer that is used for output.

If, for instance, someone wanted to average over 6ms instead then the prescaler could be set to 24x instead.
*/
    ADC->ADC_MR = (1 << 7) //free running
        + (63 << 8) //12x MCLK divider ((This value + 1) * 2) = divisor 0=min 255=max (original value (5 << 8)) !!!(Working value (63 << 8))
        + (1 << 16) //8 periods start up time (0=0clks, 1=8clks, 2=16clks, 3=24, 4=64, 5=80, 6=96, etc) (original value (1 << 16))
        + (1 << 20) //settling time (0=3clks, 1=5clks, 2=9clks, 3=17clks) (original value (1 << 20))
        + (4 << 24) //tracking time (Value + 1) clocks 0=min 15=max (original value (4 << 24))
        + (2 << 28) //transfer time ((Value * 2) + 3) 0=min 3=max clocks (original value (2 << 28))
#ifdef ADC_User_Seq
        + (1 << 31);//USEQ = 1  User Sequence Mode: The sequence respects what is defined in ADC_SEQR1 and ADC_SEQR2 registers.
#else
        ;// terminate the above function
#endif

/*
43.6.10 ADC Timings
    Each ADC has its own minimal Startup Time that is programmed through the field STARTUP in the Mode Register,
    ADC_MR.
    A minimal Tracking Time is necessary for the ADC to guarantee the best converted final value between two
    channel selections.This time has to be programmed through the TRACKTIM bit field in the Mode Register,
    ADC_MR.
    When the gain, offset or differential input parameters of the analog cell change between two channels, the analog
    cell may need a specific settling time before starting the tracking phase.In that case, the controller automatically
    waits during the settling time defined in the �ADC Mode Register�.Obviously, if the ANACH option is not set, this
    time is unused.
    Warning: No input buffer amplifier to isolate the source is included in the ADC.This must be taken into
    consideration to program a precise value in the TRACKTIM field.See the product ADC Characteristics section.
*/                    
//43. Analog-to-Digital Converter (ADC)
//33.8.7 Using the Peripheral DMA Controller (PDC)
    obufn = 0;
    bufn = 1;
    NVIC_EnableIRQ(ADC_IRQn);
    ADC->ADC_RPR = (uint32_t) adc_buf[obufn];   // DMA buffer. current memory pointer
    ADC->ADC_RCR = Num_Samples_To_Take; //# of samples to take
    ADC->ADC_RNPR = (uint32_t) adc_buf[bufn]; // next DMA buffer. next memory pointer
    ADC->ADC_RNCR = Num_Samples_To_Take; //# of samples to take
    ADC->ADC_PTCR = 1; //enable dma mode

    ADC->ADC_IDR = ~(1 << 27); //dont disable the ADC interrupt for (rx end) disable all other
    ADC->ADC_IER = 1 << 27; //do enable the ADC interrupt
    ADC->ADC_CR = 2; //start ADC conversions

    Logger::debug("Fast ADC Mode Enabled");
}

/*
Take the arithmetic average of the readings in the buffer for each channel. This smooths out the ADC readings
*/
void getADCAvg()
{
    uint32_t sum = 0;
    uint16_t Temp_Sum = 0;
    uint16_t Buffer_Pointer = obufn;    // Copy the obufn pointer, it could change while we process the buffer

#ifdef ADC_User_Seq
    for(uint8_t j = 0; j < Num_Samples; j++){
        for(uint8_t k = 0; k < NUM_ANALOG; k++){
            adc_buffer[k][j] = adc_buf[Buffer_Pointer][k];
        }
    }
#else
    for(uint8_t i = 0; i < Num_Samples; i++){   // Unpack the ADC readbuffer to adc_buffer[x][i] for averaging
            //this is a somewhat unrolled for loop with no incrementer. it's odd but it works
        for(uint8_t j = 0; j < NUM_ANALOG;){
            adc_buffer[2][i] = adc_buf[Buffer_Pointer][j];  // 0
            adc_buffer[4][i] = adc_buf[Buffer_Pointer][++j];// 1
            adc_buffer[3][i] = adc_buf[Buffer_Pointer][++j];// 2
            adc_buffer[6][i] = adc_buf[Buffer_Pointer][++j];// 3
            adc_buffer[5][i] = adc_buf[Buffer_Pointer][++j];// 4
            adc_buffer[7][i] = adc_buf[Buffer_Pointer][++j];// 5
            adc_buffer[1][i] = adc_buf[Buffer_Pointer][++j];// 6
            adc_buffer[0][i] = adc_buf[Buffer_Pointer][++j];// 7
            adc_buffer[8][i] = adc_buf[Buffer_Pointer][++j];// 8
            ++j;
        }
    }
#endif

    for(uint8_t i = 0; i < NUM_ANALOG; i++){
        for(uint8_t j = 0; j < Num_Samples; j++){
            sum += adc_buffer[i][j];
        }
        Temp_Sum = sum / Num_Samples;
//        adc_out_vals[i] = Temp_Sum;
        adc_out_vals[i] = (adc_out_vals[i] + Temp_Sum) / 2;
        sum = 0;
    }
}

/*
get value of one of the 9 analog inputs 0->(NUM_ANALOG - 1)
Uses a special buffer which has smoothed and corrected ADC values. This call is very fast
because the actual work is done via DMA and then a separate polled step.
*/
uint16_t getAnalog(uint8_t which)
{

    if (which >= NUM_ANALOG) which = 0;
    if (adc[which][0] > 15) which = 0;

    return adc_out_vals[which];
}

/*
get value of one of the sudo 6 digital/Analogue inputs 0->(NUM_DIGITAL - 1)
*/
boolean getDigital(uint8_t which)
{
    if((which >= NUM_OUTPUT) || (dig[which] == 255)){
        return(false);
    }
    return((adc_out_vals[which] > 200) ? true : false);
}

//set output high or not
void setOutput(uint8_t which, boolean active)
{
    if((which >= NUM_OUTPUT) || (out[which] == 255)){
        return;
    }
#ifdef _M2IO
    (active) ? M2IO.Setpin_12VIO(which + 1, 1) : M2IO.Setpin_12VIO(which + 1, 0);
#else
    if(active){
        (which <= 2) ? digitalWrite(out[which], HIGH) : digitalWrite(out[which], LOW);
    }else{
        (which <= 2) ? digitalWrite(out[which], LOW) : digitalWrite(out[which], HIGH);
    }
#endif
}

//get current value of output state (high?)
boolean getOutput(uint8_t which)
{
    if((which >= NUM_OUTPUT) || (out[which] == 255)){
        return false;
    }
#ifdef _M2IO
    return false;   // not implemented yet
#else
    return digitalRead(out[which]);
#endif
}

void setLED(uint8_t which, boolean hi){
    if(which == 255){
        return;
    }
    if(hi){
        digitalWrite(which, HIGH);
    } else{
        digitalWrite(which, LOW);
    }
}
