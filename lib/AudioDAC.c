/*
** AudioDAC.c
**

*/

#include <project_setup.h>
#include <peripheral/pps.h>
#include <math.h>
#include <p32xxxx.h>
#include <plib.h>
#include <stdlib.h>
#include <utility.h>
#include <sdmmc.h>
#include <fileio.h>
#include <AudioDAC.h>

#define B_SIZE  512         // audio buffer size
UINT32 sample_value;

// audio configuration 
typedef struct {
  unsigned char isStereo;          // 0 - mono  1- stereo
  unsigned char skip;              // distance to advance pointer to next sample
  unsigned char bytesPerChannel;   // sample size (8 or 16-bit) in bytes per channel
} AudioCfg;

// chunk IDs
#define  RIFF_DWORD  0x46464952UL
#define  WAVE_DWORD  0x45564157UL
#define  DATA_DWORD  0x61746164UL
#define  FMT_DWORD   0x20746d66UL     
#define  WAV_DWORD   0x00564157UL

typedef struct {
  // data chunk
  unsigned int dlength;       // actual data size
  char    data[4];            // "data"

  // format chunk 
  unsigned short  bits_per_sample_per_channel; // bit per sample
  unsigned short  bytes_per_block;      // bytes per sample
                                        //    (4=16bit stereo)
  unsigned int    bytes_per_sec;        // bytes per second
  unsigned int    blocks_per_sec;       // sample rate in Hz
  unsigned short  channels;             // # of channels
                                        //  (1= mono,2= stereo)
  unsigned short  subtype;              // always 01
  unsigned int    flength;              // size of this block (16)
  char            fmt_[4];              // "fmt_"
  
  char            type[4];              // file type name "WAVE"
  unsigned int    tlength;              // size of encapsulated block
  char            riff[4];              // envelope "RIFF"
} WAVE;

// global definitions
char    ABuffer[ 2][ B_SIZE];           // double data buffer
// "volatile" tells compiler not to optimize - somebody else might change the value
volatile char   CurBuf;                 // index of buffer in use
volatile char   AEmptyFlag;             // buffer needs to be filled

// audio configuration parameters
AudioCfg ACfg;              

// local definitions
unsigned char *BPtr;                    // pointer inside active buffer
short bytesLeft;                       // in units of samples == blocks


void initAudio( void)
{ // configures peripherals for Audio playback 

  // LRCK == LRCLK == sample clock
  I2S_LRCK_TRIS = 0;
  I2S_LRCK_PPS_REG  = I2S_LRCK_PPS_ITM;

  // MOSI == I2S data out
  I2S_SDO_TRIS = 0;
  I2S_SDO_PPS_REG  = I2S_SDO_PPS_ITM;

  // REFCLK == MCLK == master clock
  REFCLKO_TRIS = 0;
  REFCLKO_PPS_REG  = REFCLKO_PPS_ITM;

  programStatus = ADAC_Inited;
    
} // initAudio


void startAudio( int samples_per_second, int count)
{ // begins the audio playback
    
  UINT spi_con1 = 0, spi_con2 = 0;
  // not_TODO: baud_rate should *not* be samples_per_second * channels * bits_per_value??
  UINT baud_rate = samples_per_second * 64;
  // no visible data on what mclk should be, beyond "256, 384, or 512 times LRCLK"
  UINT mclk = samples_per_second * 256;

  // 1. init pointers  and flags
  //  moved to PlayWAV

  // 2. number of actual samples to be played
  bytesLeft = count;

  // FPB is defined to be the frequency of PBCLK (see project_setup.h)
  // samples_per_second = samples per second (equate this to LRCK)
  // master clock (MCLK) will be counted by REFOCLK
  //   freq should be 256 * 2 times the rate of sample clock (LRCK)
  // so RODIV + REFTRIM / 512 is PB ticks per master clock
  // the following calc gives 512 times the desired result
  // getting 5.0 to 5.8 MHz
  //
  int RODIV = floor(FPB / samples_per_second + 0.5);
  int REFTRIM = RODIV & 0x01ff;
  RODIV >>= 9;

  //Configure Reference Clock Output (Master Clock)
  mOSCREFOTRIMSet(REFTRIM);
  OSCREFConfig(OSC_REFOCON_PBCLK,                 //clock output used as REFCLKO source
               OSC_REFOCON_OE | OSC_REFOCON_ON,   //Enable and turn on the REFCLKO
               RODIV);

  //Configure SPI in I2S mode with 32-bit stereo audio.
  /*
      Changes from the example code (2012-04-06 cbm)
   * Using SPI 2
   * 3-wire I2S
   * will LRCLK be connected to FSYNC (not SS)? Or neither?
   * want to use frame sync control to control LRCLK; can we do this?
   *   to do this, we need MCLK / LRCLK 256, 384, or 512
   *      enable FRMEN
   *      FRMSYNC as output
   *      FRMPOL active high
   *      MSSEN slave select off???
   *      FRMSYPW frame sync pulse width is one character wide
   *      FRMCNT 101 (5) generate frame sync pule every 32 characters
   *      SPIFE frame sync pulse edge select bit - 0 - FSP precedes first bit clock
   *
   *
  */
  spi_con1 = SPI_OPEN_FRMEN          |       //Enable framed SPI function
             // SPI_OPEN_FSP_IN         |      // FSP as input
     //TODO: Try inverting FSP to correct right-left backwardsness
             // SPI_OPEN_FSP_HIGH       |       //Frame Sync Pulse is active high
             SPI_OPEN_FSP_WIDE       |       // Frame Sync Pulse is 1 char wide rather than 1 bit wide
             SPI_OPEN_FRM_CNT32      |       //Generate frame sync pulse every 32 data characters (256 bits?)
             SPI_OPEN_MCLKSEL	      |	      //Clock selected is reference clock
             // SPI_OPEN_MODE16	 |	//Data mode: 24b
             SPI_OPEN_MODE32	      |	      //Data mode: 32-bit FIFO
             // SPI_OPEN_SSEN     	 |  	//Enable slave select function
             // SPI_OPEN_CKP_HIGH 	 |  	//Clock polarity Idle High Active Low
             SPI_OPEN_MSTEN          |  	//Master mode enable
             SPI_OPEN_DISSDI         |          //Disable SDI (input pin)
             // SPI_OPEN_TBE_EMPTY      |       //Enable interrupt on TX buffer empty
             SPI_OPEN_TBE_NOT_FULL   |       //Enable interrupt on TX buffer not full
             0;

  spi_con2 = SPI_OPEN2_AUDEN 	      |  	//Enable Audio mode
             SPI_OPEN2_AUDMOD_I2S     |         //Enable I2S mode
             0;

  //Configure and turn on the SPI module.
  // mclk/baud_rate = (SAMPLE_RATE * 256) / (SAMPLE_RATE * 64) = 4
  SpiChnOpenEx(SPI_CHANNEL1, spi_con1, spi_con2, (mclk / baud_rate));

  //Enable SPI1 interrupt.  vector is 31; includes fault, RX done, TX done
  INTSetVectorPriority(INT_SPI_1_VECTOR, INT_PRIORITY_LEVEL_4);
  INTSetVectorSubPriority(INT_SPI_1_VECTOR, INT_SUB_PRIORITY_LEVEL_0);
  INTEnable(INT_SPI1, INT_ENABLED);

  SpiChnPutC(SPI_CHANNEL1, (INT32) 0);	//Dummy write to start the SPI
//  snprintf (strBuf, bufLen, "post SpiChnPutC\n");
//  SendDataBuffer (strBuf, strlen(strBuf));
  programStatus = ADAC_Started;
} // startAudio


void haltAudio( void)
{ // stops playback state machine
  // mT2IntEnable( 0);
  //TODO: turn off LRCLK and/or REFCLK?
  INTEnable(INT_SPI1, INT_DISABLED);
  SpiChnClose(SPI_CHANNEL1);
  programStatus = ADAC_Finished;
  REFOCON = REFOCON & !OSC_REFOCON_ON;
} // halt audio


/* SPI1 ISR */
void __attribute__((vector(_SPI_1_VECTOR), interrupt(ipl4))) SPI1TXInterruptHandler(void)
{	
  static UINT ptrIndex = 0;
  static BOOL isLeftChannel = TRUE;
  #define isRightChannel !isLeftChannel

  // 0. allow interrupt nesting
  asm( "ei");

  /*
   *   unsigned char isStereo;            // 0 - mono  1- stereo
  unsigned char skip;              // advance pointer to next sample
  unsigned char bytesPerChannel;              // sample size (8 or 16-bit) in bytes per channel
   */

  //TODO: Note that *BPtr may have a type; sample_value is UINT32, I think
  //      and we may be little-ending incorrectly...
  //TODO: left channel data goes to channel 1, but that's with LRCLK *HIGH*
  //      when DAC wants LRCLK to be LOW for left channel...
  int offset;
  offset = 0;
  if (isRightChannel && ACfg.isStereo) {
    offset = ACfg.bytesPerChannel;
  }
  // Now *(BPtr + offset) is the low-order byte of the data.
  // sample_value = 0;
  // memcpy((void *) &sample_value, (void *) BPtr + offset, ACfg.bytesPerChannel);
  if (ACfg.bytesPerChannel == 1) {
    sample_value = (UINT32) *((UINT8 *) (BPtr + offset));
    sample_value = sample_value << 22;  //TODO: s.b. 24
  } else {
    // cannot simply sample_value = (UINT32) *((UINT16 *) (BPtr + offset));
    // because it might not be short-aligned -- causing memory error
    sample_value = (UINT32) *((UINT8 *) (BPtr + offset + 1));
    sample_value = sample_value << 8;
    sample_value |= (UINT32) *((UINT8 *) (BPtr + offset));
//    sample_value.b2[0] = (INT16) *((INT16 *) (BPtr + offset));
//    sample_value.b1[1] = *(BPtr + offset);
//    sample_value.b1[0] = *(BPtr + offset + 1);
    sample_value ^= 0x8000;
    sample_value = sample_value << 14;  //TODO: s.b. 16
  }

  if (isRightChannel) {
    // do once per *block*, after right channel
    BPtr += ACfg.skip;
    bytesLeft -= ACfg.skip;
  }
  
  SpiChnPutC(SPI_CHANNEL1, sample_value);
  isLeftChannel = ! isLeftChannel;

  // 3. check if buffer emptied
  if ( bytesLeft == 0) {
    // 3.1 swap buffers
    CurBuf = 1 - CurBuf;

    // 3.2. place pointer on first sample
    BPtr = &ABuffer[CurBuf][0];

    // 3.3 restart counter
    bytesLeft = B_SIZE;

    // 3.4 flag a new buffer needs to be filled
    AEmptyFlag = TRUE;
  }

  // 4. clear interrupt flag and exit
  INTClearFlag(INT_SPI1TX);
}


/*-------------------------------------------------------------
** Wave
** 
** 	Wave File Player 
**
** revision:
** 05/02/03 v1.00 LDJ PIC18
** 07/08/06 v2.00 LDJ porting to PIC24/C30
** 07/14/07 v3.00 LDJ PIC32 porting
*/
int playWAV( char *name)
{
  WAVE    wav;
  MFILE    *f;
  int remaining_byte_count, bytes_read;
  int wi, pos, samples_per_second, PBCLK_cycles_per_sample, last;
  // char s[16];

  // 1. open the file           
  if ( (f = fopenM( name, "r")) == NULL) {
    // failed to open
    programStatus = ADAC_Open_Failed;
    goto EXIT;
  }
  
  // 2. verify it is a RIFF formatted file
  if ( ReadL( f->buffer, 0) != RIFF_DWORD) {
    fcloseM( f);
    programStatus = ADAC_File_Not_RIFF;
    goto EXIT;
  }
  
  // 3. look for the WAVE chunk signature 
  if ( (ReadL( f->buffer, 8)) != WAVE_DWORD) {
    fcloseM( f);
    programStatus = ADAC_Chunk_Not_WAVE;
    goto EXIT;
  }
  
  // 4. look for the chunk containing the wave format data
  if ( ReadL( f->buffer, 12) != FMT_DWORD) {
    fcloseM( f);
    programStatus = ADAC_No_FMT_keyword;
    goto EXIT;
  }
  
  if (ReadW (f->buffer, 20) != 0x0001) {   // s.b. 0x0001, PCM/uncompressed
    fcloseM( f);
    programStatus = ADAC_Is_Compressed;
    goto EXIT;
  }

  // Note: block is one set of samples (including all channels)
  wav.channels                    = ReadW( f->buffer, 22);
  wav.blocks_per_sec              = ReadL( f->buffer, 24);    // sample rate (blocks per sec)
  wav.bytes_per_sec               = ReadL( f->buffer, 28);    // avg bytes per second = samplerate * blockalign
  wav.bytes_per_block             = ReadW( f->buffer, 32);    // aka blockalign
  wav.bits_per_sample_per_channel = ReadW( f->buffer, 34);    // bits per sample usu 8, 16, 24, or 32; per channel
  
  // 5. search for the data chunk
  wi = 20 + ReadW( f->buffer, 16);  // beginning of actual data
  while ( wi < 512)
  {
    if (ReadL( f->buffer, wi) == DATA_DWORD) break;
    wi += 8 + ReadW( f->buffer, wi+4);
  }
  if ( wi >= 512) // could not find in current sector
  {
    fcloseM( f);
    programStatus = ADAC_No_DATA_Word;
    goto EXIT;
  }   

  // 6. find the data size (actual wave content)
  wav.dlength = ReadL( f->buffer, wi+4);
  
  // 7. if sample rate too high, skip
  samples_per_second = wav.bytes_per_sec / wav.bytes_per_block;
  ACfg.skip = wav.bytes_per_block;      // skip to reduce bandwith

  while ( samples_per_second > 48000) {
    samples_per_second >>= 1;  // divide sample rate by two
    ACfg.skip <<= 1;           // multiply skip by two
  }

  // 8. check if sample rate too low
  PBCLK_cycles_per_sample = ( FPB / samples_per_second ) - 1;
  if ( PBCLK_cycles_per_sample > ( 0x00010000)) {     // too big for 16 bits
    fcloseM( f);
    programStatus = ADAC_Period_Too_Long;
    goto EXIT;
  }

  // 9. init the Audio state machine
  CurBuf = 0;
  pos = wi + 8;                   // data begin
  ACfg.isStereo = (wav.channels == 2);
  ACfg.bytesPerChannel  = wav.bits_per_sample_per_channel >> 3;
  
  // 10 # of bytes composing the wav data chunk
  remaining_byte_count = wav.dlength;
  
  // 11. pre-load both buffers

  bytes_read = freadM( ABuffer[0], B_SIZE*2, f);
  remaining_byte_count -= bytes_read;
  BPtr = ABuffer[0] + pos;
  CurBuf = 0;                 // buffer 0 active first
  AEmptyFlag = TRUE;    // the OTHER buffer is yet empty...
      
  // 12. configure Player state machine and start
  initAudio ();
//  snprintf (strBuf, bufLen, "pre SA\n");
//  SendDataBuffer (strBuf, strlen(strBuf));

  startAudio (samples_per_second, bytes_read - pos);
//  snprintf (strBuf, bufLen, "post SA\n");
//  SendDataBuffer (strBuf, strlen(strBuf));
      
  // 13. keep feeding the buffers in the playing loop
  //     as long as entire buffers can be filled
  while (remaining_byte_count > 0) {

    // 13.1 check user input to stop playback
    if ( !BTN ) {            // if any button pressed
      remaining_byte_count = 0;
      programStatus = ADAC_Cancelled;
      break;
    }
    
    // 13.2 check if a buffer needs a refill
    if (AEmptyFlag) {
      // read into the *other* buffer (not ABuffer[CurBuf])
      bytes_read = freadM( ABuffer[1-CurBuf], B_SIZE, f);

      remaining_byte_count -= bytes_read;
      AEmptyFlag = FALSE;            // refilled

      // <<put here additional tasks>>
      // putsLCD("\n");          //  on the second line
      // sprintf( s, "%dKB", (wav.dlength - remaining_byte_count) / 1024);
      // putsLCD( s);            // byte count
    }
  } // while wav data available

  // 14. pad the rest of the buffer 
  last = ABuffer[1-CurBuf][bytes_read-1];
  while( bytes_read<B_SIZE) ABuffer[1-CurBuf][bytes_read++] = last;
  AEmptyFlag = FALSE;         // refilled

  // 15.finish the last buffer
  AEmptyFlag = FALSE;
  while (!AEmptyFlag);
  
  // 16. stop playback 
  haltAudio();

  // 17. close the file 
  fcloseM( f);

  // 18. return with success
  if (programStatus != ADAC_Cancelled) programStatus = ADAC_Success;

EXIT:
  return programStatus;

} // playWAV
         


