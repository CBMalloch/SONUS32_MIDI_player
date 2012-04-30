/*
** SONUS32_pin_assignments.h   M5 SONUS32 project 
** 
** 2012-04-27 v0.4 cbm
**

 * A0  (02) SPI1/I2S SS1 DAC
 * A1  (03) SPI1/I2S SDO1 DAC
 * B0  (04) PICkit 3 programming - MICRO PGD data
 * B1  (05) PICkit 3 programming - MICRO PGC clock
 * B2  (06) card detect SD card
 * B3  (07) card select SD card
 * A2  (09) SPI2 SDO2 SD card
 * A3  (10)
 * B4  (11) write protect SD card
 * A4  (12) SPI2 SDI2 SD card
 * B5  (14) LED
 * B7  (16) input tactile button switch
 * B8  (17) input potentiometer (assigned in main.c)
 * B9  (18)
 * B10 (21) UART2 TX (assigned in main.c)
 * B11 (22) UART2 RX (assigned in main.c)
 * B13 (24) REFOCLK (MCLK) DAC
 * B14 (25) SPI1/I2S SCK1 DAC
 * B15 (26) SPI2 SCK2 SD card
 
 Change history:
  2012-04-27 v0.4 cbm Changed _RA0 to _LATA0, etc.

*/

#define LED                   _LATB5
#define LED_TRIS              _TRISB5

#define BTN                   _RB7
#define BTN_TRIS              _TRISB7

// #define UART2_SFR             RPxx
#define UART2_RX_PPS_REG      U2RXRbits.U2RXR
#define UART2_RX_PPS_ITM      0x03
// #define UART2_SDO_TRIS        _TRISxx

#define UART2_TX_PPS_REG      RPB10Rbits.RPB10R
#define UART2_TX_PPS_ITM      0x02


// ************
// SD card pins (SPI2)
// ************

// SDI2 is on pin 12 = RPA4 - PPS as 0010b
#define SDI_PPS_ITM           0x02
// SDO2 is on pin 9 = RPA2
#define SDO_PPS_ITM           0x04
#define SDO_REG               RPA2Rbits.RPA2R
// SCK2 is on pin 26 -- hardwired for SDI2
// Write Protect input is pin 11 = RPB4
//TODO: normalize things like RB4 and _RB4
#define SDWP                  _LATB4
// Card Detect input is pin 6 = RPB2
#define SDCD                  _LATB2
// Card Select output is pin 7 = RPB3
#define SDCS                  _LATB3
#define SDCS_TRIS             _TRISB3

//*  ** For DAC using SPI1:

  // *******************
  // SPI/I2S pins (SPI1)
  // *******************

  // SCK1 : SPI-I2S bit clock output  PIC pin 25 to DAC pin 2
  // hard-wired to pin 25

  // SS1 : SPI-I2S LRCK output       PIC pin 11 to DAC pin 3
  #define I2S_LRCK_SFR        RPA0
  #define I2S_LRCK_PPS_REG    RPA0Rbits.RPA0R
  #define I2S_LRCK_PPS_ITM    0x03
  #define I2S_LRCK_TRIS       _TRISB4

  // SDO1 : SPI-I2S SDO output         PIC pin 3 to DAC pin 1
  #define I2S_SDO_SFR         RPA1
  #define I2S_SDO_PPS_REG     RPA1Rbits.RPA1R
  #define I2S_SDO_PPS_ITM     0x03
  #define I2S_SDO_TRIS        _TRISA1

  // SDI1 : SPI-I2S SDI Input          not used

  // ********************************
  // REFOCLK : Reference Clock Output
  // ********************************
  // REFCLKO: reference clock         PIC pin 24 to DAC pin 4 (MCLK)
  #define REFCLKO_SFR         RPB13
  #define REFCLKO_PPS_REG     RPB13Rbits.RPB13R
  #define REFCLKO_PPS_ITM     0x07
  #define REFCLKO_TRIS        _TRISB13

//*/

/*  ** For PWM output:

  #define CHAN1               _LATA0
  #define CHAN1_PPS_REG       RPA0Rbits.RPA0R
  #define CHAN1_PPS_ITM       0x05
  
  #define CHAN2               _LATA1
  #define CHAN2_PPS_REG       RPA1Rbits.RPA1R
  #define CHAN2_PPS_ITM       0x05
*/
