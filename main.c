/*
**	SONUS_32_MIDI_player main.c
**
*/

#include <p32xxxx.h>
#include <GenericTypeDefs.h>
#include <utility.h>
#include <SDMMC.h>
#include <plib.h>
#include <peripheral/pps.h>
#include <SONUS32_pin_assignments.h>
#include <AudioDAC.h>

// *****************************************************************************
// *****************************************************************************
// Section: Configuration bits
// *****************************************************************************
// *****************************************************************************

#pragma config POSCMOD = OFF      // primary oscillator conf
#pragma config OSCIOFNC = OFF     // CLKO disconnect from output pin
#pragma config FSOSCEN = OFF      // secondary oscillator disable

// FNOSC FRCPLL selects FRC for input to PLL as well as PLL output for SYSCLK
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPLLODIV = DIV_2  // PLL 40MHz
#pragma config FNOSC = FRCPLL       // oscillator selection
#pragma config FPBDIV = DIV_1     // peripheral bus clock divisor 40MHz

#pragma config FWDTEN = OFF       // watchdog timer
#pragma config FCKSM = CSECME     // clock switching and monitor selection

#pragma config CP = OFF           // code (read and modify) protect
#pragma config BWP = OFF          // boot flash write-protect
#pragma config PWP = OFF          // program flash write-protect

// #pragma config ICESEL = ICS_PGx1  // ice/icd comm channel select
#pragma config JTAGEN = OFF       // JTAG disable
// JTAG port pins are multiplexed with PortA pins RA0, RA1, RA4, and RA5

// *****************************************************************************
// *****************************************************************************
// Section: Definitions
// *****************************************************************************
// *****************************************************************************

#define START_ADDRESS       10000   // start block address
#define N_BLOCKS            10      // number of blocks
#define B_SIZE              512     // sector/data block size

extern unsigned int _excep_code;
extern unsigned int _excep_addr;
extern UINT8        strBuf[bufLen];

volatile int    programStatus;          // error code


// *****************************************************************************
// *****************************************************************************
// Section: System Macros
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Code
// *****************************************************************************
// *****************************************************************************

void main() {
  LBA addr;
  int i, j, r;

  // 1. initializations

  LED_TRIS = 0;
  // TRISBbits.TRISB5 = 0;  // b5 output
  BTN_TRIS = 1;
  // TRISBbits.TRISB7 = 1;  // b7 input

  UART2_RX_PPS_REG = UART2_RX_PPS_ITM;
  // U2RXRbits.U2RXR = 3;   //SET U2RX to RPB11 (Pin 22)
  UART2_TX_PPS_REG = UART2_TX_PPS_ITM;
  // RPB10Rbits.RPB10R = 2; //SET RPB10R (Pin 21) to U2TX

  UARTConfigure(UART2, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART2, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART2, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
  UARTSetDataRate(UART2, FPB, 19200);
  UARTEnable(UART2, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  /*
  CHAN1_PPS_REG = CHAN1_PPS_ITM;
  // RPA0Rbits.RPA0R = 5;  // SET RPA0R (Pin 2) to OC1
  CHAN2_PPS_REG = CHAN2_PPS_ITM;
  // RPA1Rbits.RPA1R = 5;  // SET RPA1R (Pin 3) to OC2
  */

  // enable multi-vector interrupts
  INTEnableSystemMultiVectoredInt();

  initSD();		            // init SD/MMC module

  // 3. wait for the card to be inserted
  while( !getCD());       // check CD switch
  Delayms( 100);          // wait contacts de-bounce
  if ( initMedia()) {       // init card
    // if error code returned
    // clrLCD();
    // putsLCD( "Failed Init");
    snprintf (strBuf, bufLen, "Failed Init\n");
    goto End;
  }
            
  LED = 1;            // SD card in use
  
  if (mount()) {
    // 8k8bit.wav (8000, mono, 8-bit) -> OK
    // 8k16bit.wav (8000, mono, 16-bit) -> OK
    // 11k8bit.wav (11000, mono, 8-bit) -> OK
    // 11k16bit.wav (11000, mono, 16-bit) -> OK
    // swing42.wav -> is compressed
    // swg42MS.wav -> address exception
    // welcome.wav -> lousy sound, but runs OK

    // test.wav -> (44100) OK -- skips a little
    // test1.wav -> is compressed
    // test16a.wav (8000, mono, 16-bit) OK
    // steptest.wav (22050, mono, 16-bit from Python) no sound expected
    // steptst8.wav (8000, mono, 16-bit from Python) OK no sound expected
    // steptst2.wav (22050, mono, 16-bit via Audacity) ??? no sound expected

    
    // alrm1ksa.wav -> OK
    // alarm1.wav -> OK  -- sample rate 22050; 8-bits x 2 channels
    // alrm4wcb.wav -> OK
    // beamen1.wav -> OK
    // commcls.wav -> OK
    // commopen.wav -> OK
    // holoon.wav -> OK
    // intrcom1.wav -> OK
    // cosine.wav (from M5) -> pulsating noise for 30 sec (floating point?)

    r = playWAV("alarm1.wav");
    SendDataBuffer(">", 1);
    while (GetDataBuffer(strBuf, bufLen)) {
      if (!strcmp(strBuf, ".")) {
        break;
      } else {
        r = playWAV(strBuf);

        if (r == ADAC_Success) {
          snprintf (strBuf, bufLen, "Done.\n");
        } else {
          snprintf (strBuf, bufLen, "File crashed!\n");
        }
      }
      SendDataBuffer(">", 1);
    }
  } else {
    snprintf (strBuf, bufLen, "Bad mount\n");
  }
  
  SendDataBuffer (strBuf, strlen(strBuf));

End:
  LED = 0;    // SD card not in use
  // main loop
  PlayPattern (0xaaaaaaaa);

} // main


