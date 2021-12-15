/*
 * File:   main.c
 * Author: Syed Tahmid Mahbub
 *
 */

/* SPI pins configuration:
 * Using SPI1       (defined in HardwareProfile.h)
 *  SDO:    RB13    (TRIS defined in HardwareProfile.h and PPS defined in SD-SPI.c)
 *          physical pin 24
 *  SDI:    RB11    (TRIS defined in HardwareProfile.h and PPS defined in SD-SPI.c)
 *          physical pin 22
 *  SCK:    RB14    (TRIS defined in HardwareProfie.h)
 *          physical pin 25
 *  CS:     RB2     (defined in HardwareProfile.h)
 *          physical pin 6
 *  CD:     RB7     (defined in HardwareProfile.h)
 *          physical pin 16
 *          For now I'm not using this - overriden in code
 *  WE:     RB8     (defined in HardwareProfile.h)
 *          physical pin 17
 *          For now I'm not using this - overriden in code
 */

/* microSD pins configuration (SPI mode):
 * 1:   RESERVED (NA)
 * 2:   CS      CHIP SELECT
 * 3:   SDI     SERDIAL DATA IN
 * 4:   VDD     VDD SUPPLY (+3.3V)
 * 5:   SCK     SERIAL CLOCK
 * 6:   VSS     VSS SUPPLY (GND)
 * 7:   SDO     SERIAL DATA OUT
 * 8:   RESERVED (NA)
 */

#include <stdio.h>
#include "FSIO.h"
#include "plib.h"
#include <GenericTypeDefs.h>

// Configuration bits settings
#pragma config FNOSC = FRCPLL, POSCMOD = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20
#pragma config FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF, JTAGEN = OFF, FSOSCEN = OFF

#define SYS_CLK         40000000
#define PB_CLK          SYS_CLK

#define SS              LATAbits.LATA4      // physical pin 12
#define dirSS           TRISAbits.TRISA4

// useful VT100 escape sequences for PuTTY/GTKterm
#define clearPutty()    printf( "\x1b[2J")
#define homePutty()     printf( "\x1b[H")

#define AC_ZERO         2048    // since Dac resolution is 12-bits

#define BAUDRATE        115200  // for serial comm

#define stackSize       128     // cyclic buffer/stack

#define allGood             0
#define incorrectHeaderSize 1
#define riff                2
#define wav                 3
#define fmt                 4
#define notPCM              5

UINT8 receiveBuffer[100];
char txtBuffer[100];

volatile UINT16 LSTACK[stackSize];
volatile UINT16 RSTACK[stackSize];
volatile UINT32 BOS;
volatile UINT32 TOS;

volatile UINT32 msCounter = 0;
volatile UINT32 randomvar = 0;
volatile UINT32 CSlength = 0;
volatile UINT32 j = 0;
volatile UINT32 TIC=0, TOC=0;

volatile UINT32 bufferCounter = 0;
volatile UINT32 intCounter = 0;

void initDAC(void);
void setupUART(void);
void infiniteBlink(void);
volatile UINT32 intCounter = 0;

void initDAC(void);
void setupAudioPWM(void);
void setupSystemClock(void);
void getFilename(char * buffer);
void configureHardware(UINT32 sampleRate);
inline void writeDac(UINT16 dacVal);
UINT8 getWavHeader(FSFILE * pointer);
void getParameters(UINT8 * bitsPerSample, UINT8 * numberOfChannels,
                        UINT32 * dataSize, UINT32 * sampleRate,
                        UINT32 * blockAlign);
void zeroDacs(void);

void __ISR(_TIMER_3_VECTOR, ipl3) T3Int (void) {
//  millisecond counter
//  timer 3 is setup such that the period is 1ms
//  a system clock of sorts
    msCounter++;
    mT3ClearIntFlag();
}

void initDAC(void) {
    dirSS = 0;
    SS = 1;

    PPSOutput(2, RPB5, SDO2);   // physical pin 14
    SpiChnOpen(2, SPI_OPEN_MSTEN | SPI_OPEN_MODE16 | SPI_OPEN_ON |
            SPI_OPEN_DISSDI | SPI_OPEN_CKE_REV, 2);
    // Clock at 20MHz
}

void setupUART(void) {
    PPSInput (2, U2RX, RPA1); //Assign U2RX to pin RPA1 -- Physical pin 3 on 28 PDIPS
    PPSOutput(4, RPB10, U2TX); //Assign U2TX to pin RPB10 -- Physical pin 21 on 28 PDIP

    // white on adafruit 954 is RX in - so connect to pin 21
    // green on adafruit 954 is TX out - so connect to pin 3

    UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART2, PB_CLK, BAUDRATE);
    UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
}

void infiniteBlink(void) {
    UINT32 i;
    while (1) {
        LATAINV = 1;
        for (i = 0; i < 5000000; i++);
    }
}

void setupAudioPWM(void) {
    T2CON = 0x0; // TMR2 off, prescaler 1:1
    mT2SetIntPriority(2);
}

void setupSystemClock(void) {
    T3CON = 0x8000; // t3 on, prescaler 1:1
    PR3 = 40000 - 1; // 1ms
    mT3IntEnable(1);
    mT3SetIntPriority(3);
}

void getFilename(char * buffer) {
#define maxChars        20
#define _BACKSPACE_     8

    char c = 32;    // 32 = space
    INT8 bufferCounter = 0;

    while (DataRdyUART2() == 0);
    c = getcUART2();

    while ((c!= 10) & (c!= 13)) {

        putcUART2(c);
        while (DataRdyUART2() == 0);

        *(buffer + bufferCounter) = c;

        if (c == _BACKSPACE_) {
            if (bufferCounter > 1) {
                bufferCounter-=2;
            }
            else {
                bufferCounter = 0;
            }
        }

        if (++bufferCounter == maxChars)
            break;

        c = getcUART2();
    }

    printf("\n\r");

    *(buffer + bufferCounter) = '\0';   // terminate string
}

void configureHardware(UINT32 sampleRate) {
    PR2 = (PB_CLK/sampleRate) - 1;
    mT2IntEnable(1);
}

void main(void) {
    FSFILE * pointer;
//    FSFILE * pointer2;
//    char path[30];
//    char count = 30;
    SearchRec rec;
    UINT8 attributes = ATTR_MASK;   // file can have any attributes

    // audio stuff (WAV file)
    UINT8 bitsPerSample;
    UINT32 sampleRate;
    UINT8 numberOfChannels;
    UINT32 dataSize;
    UINT8 blockAlign;

    UINT8 audioStream[stackSize*4];
    UINT16 audioByte;
    UINT16 lc;
    UINT16 retBytes;
    UINT16 unsign_audio;

    SYSTEMConfigPerformance(SYS_CLK);
    INTEnableSystemMultiVectoredInt();

    // clear cyclic buffer
    BOS = 0;
    TOS = 0;
    for (j=0; j<stackSize; j++) {
        LSTACK[j] = 0;
        RSTACK[j] = 0;
    }

    setupUART();
    setupAudioPWM();
    setupSystemClock();
    initDAC();
    zeroDacs();

    ANSELA = 0; ANSELB = 0;                 // Disable analog inputs
    CM1CON = 0; CM2CON = 0; CM3CON = 0;     // Disable analog comparators
    TRISACLR = 1;                           // RA0 for LED

    clearPutty();
    homePutty();
    printf("Waiting for button to be pressed...\n\r");
    LATASET = 1;
    while ( (getcUART2() != 's') );
    printf("Got required character!\n\r");
    printf("\n\rProceeding...\n\r");

    printf("#################################\n\r");
    printf("#      PIC32 AUDIO PLAYER       #\n\r");
    printf("#################################\n\r");
    printf("#      SYED TAHMID MAHBUB       #\n\r");
    printf("#################################\n\r");

    printf("\n\rLooking to detect media...\n\r");
    while (!MDD_MediaDetect());
    printf("Found media!\n\r");

    printf("\n\rInitializing library...\n\r");
    // Initialize the library
    while (!FSInit());
    printf("Initialized library!\n\r");

    printf("\n\rShowing all WAV files in root directory:\n\r");
    if (FindFirst("*.WAV", attributes, &rec) == 0) { // file found
        printf("%s\t%u KB\n\r", rec.filename, rec.filesize/1000);
        while (FindNext(&rec) == 0) { // more files found
            printf("%s\t%u KB\n\r", rec.filename, rec.filesize/1000);
        }
    }

    // GET FILE NAME TO PLAY
    j = 0;
    while (j == 0) {
        printf("\n\rSelect which file to play\n\r");
        getFilename(&txtBuffer);
        if (FindFirst(txtBuffer, attributes, &rec)) {
            printf("Invalid file! Try again!\n\r");
        }
        else {
            j = 1;
        }
    }

    // HAVE FILE NAME - SET TO PLAY

    pointer = FSfopen(txtBuffer,"r");
    if (pointer != NULL) {

        printf("Opened \"%s\"\n\r\n\r",txtBuffer);

        if (getWavHeader(pointer) == allGood) {
            
            getParameters(&bitsPerSample, &numberOfChannels, &dataSize,
                            &sampleRate, &blockAlign);
            
            configureHardware(sampleRate);

            printf("\n\r");

            printf("Bits per sample = %d\n\r", bitsPerSample);
            printf("Number of Channels = %d\n\r", numberOfChannels);
            printf("Sample Rate = %d\n\r", sampleRate);
            printf("Data file size = %u\n\r", dataSize);
            printf("Block align = %u\n\r", blockAlign);

            bufferCounter = 0;
            if (bitsPerSample < 8)
                bitsPerSample = 8;
            else if (bitsPerSample < 16)
                bitsPerSample = 16;

            TIC = msCounter;

            while (bufferCounter < dataSize) {

                retBytes = FSfread(audioStream, 1,
                                stackSize*blockAlign, pointer);

                for (lc = 0; lc < retBytes; lc += blockAlign) {
                    if (bitsPerSample == 16) {
                        audioByte = (audioStream[lc+1] << 4) |
                            (audioStream[lc] >> 4);
                        if (audioByte & 0x0800) {
                            unsign_audio = ~(audioByte - 1);
                            audioByte = AC_ZERO - unsign_audio;
                        }
                        else {
                            audioByte = AC_ZERO + audioByte;
                        }
                        LSTACK[BOS] = 0x3000 | audioByte;

                        if (numberOfChannels == 2) {
                            audioByte = (audioStream[lc+3] << 4) |
                                (audioStream[lc+2] >> 4);
                            if (audioByte & 0x0800) {
                                unsign_audio = ~(audioByte - 1);
                                audioByte = AC_ZERO - unsign_audio;
                            }
                            else {
                                audioByte = AC_ZERO + audioByte;
                            }
                        }
                        RSTACK[BOS] = 0xB000 | audioByte;
                    }
                    else {
                        audioByte = audioStream[lc] << 4;
                        LSTACK[BOS] = 0x3000 | audioByte;
                        if (numberOfChannels == 2) {
                            audioByte = audioStream[lc+1] << 4;
                        }

                        RSTACK[BOS] = 0xB000 | audioByte;
                    }

                    if (++BOS == stackSize) BOS = 0;

                    if (bitsPerSample == 16) {
                        while (BOS == (TOS-2));
                    }
                    else {
                        while (BOS == (TOS-2)) {
// for some reason, not putting a delay here makes the 8-bit part not work (?)
                            UINT32 temp = msCounter;
                            while ((msCounter - temp) < 2);
                        }
                    }

                }   // for (lc ... )...

                bufferCounter += retBytes;
                T2CONSET = 0x8000;
            }   // while (bufferCounter < dataSize) ...

            FSfclose(pointer);
        } // if (pointer != NULL), ie if read audio file correctly
    }

    T2CON = 0;
    TMR2 = 0;
    mT2IntEnable(0);
    mT2ClearIntFlag();

    zeroDacs();

    TOC = msCounter - TIC;

    {
        UINT32 ms = TOC % 1000;
        UINT32 seconds = TOC / 1000;
        UINT32 minutes = seconds / 60;
        seconds = seconds % 60;
        printf("Time elapsed = %02u:%02u.%03u (min:sec:ms)\n\r", minutes, seconds, ms);
    }
    
    printf("\n\r\n\rDone!!!\n\r");
    infiniteBlink();
}

inline void writeDac(UINT16 dacVal) {
    SS = 0;
    while (TxBufFullSPI2());
    WriteSPI2(dacVal);
    while (SPI2STATbits.SPIBUSY);
    SS = 1;
}

void __ISR(_TIMER_2_VECTOR, ipl2) T2int (void) {
    writeDac(LSTACK[TOS]);
    writeDac(RSTACK[TOS]);
    
    if (++TOS == stackSize)
        TOS = 0;

    mT2ClearIntFlag();
}

UINT8 getWavHeader(FSFILE * pointer) {

    if (FSfread(receiveBuffer, 1, 44, pointer) != 44) {
        return incorrectHeaderSize;
    }

    if ( (receiveBuffer[0] != 'R') |
            (receiveBuffer[1] != 'I') |
            (receiveBuffer[2] != 'F') |
            (receiveBuffer[3] != 'F') ) {
        return riff;
    }

    if ( (receiveBuffer[8] != 'W') |
            (receiveBuffer[9] != 'A') |
            (receiveBuffer[10] != 'V') |
            (receiveBuffer[11] != 'E') ) {
        return wav;
    }

    if ( (receiveBuffer[12] != 'f') |
            (receiveBuffer[13] != 'm') |
            (receiveBuffer[14] != 't') ) {
        return fmt;
    }

    if (receiveBuffer[20] != 1) {
        return notPCM;
    }

    return allGood;         // no errors

}

void getParameters(UINT8 * bitsPerSample, UINT8 * numberOfChannels,
                        UINT32 * dataSize, UINT32 * sampleRate,
                        UINT32 * blockAlign) {
    *bitsPerSample = receiveBuffer[34];
    *numberOfChannels = receiveBuffer[22];
    *sampleRate = (receiveBuffer[25] << 8) | receiveBuffer[24];

    *dataSize = (receiveBuffer[43] << 24) |
        (receiveBuffer[42] << 16) |
        (receiveBuffer[41] << 8) |
        receiveBuffer[40];
    
    *blockAlign = receiveBuffer[32];
}

void zeroDacs(void) {
    // Offset about 1.024V (since Dac has 2.048V reference)
    writeDac(0x3000 | AC_ZERO);    // L channel
    writeDac(0xB000 | AC_ZERO);    // R channel
}