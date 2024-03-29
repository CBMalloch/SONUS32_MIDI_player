/*
**	RWTest.c
**
*/

#include <p32xxxx.h>
#include <SDMMC.h>

#define START_ADDRESS       10000   // start block address
#define N_BLOCKS            10      // number of blocks
#define B_SIZE              512     // sector/data block size

char    data[ B_SIZE];
char  buffer[ B_SIZE];

main( void)
{
    LBA addr;
    int i, j, r;

    // 1. initializations
    initSD();		            // init SD/MMC module

    // 2. fill the buffer with pattern
    for( i=0; i<B_SIZE; i++)
        data[i]= i;
 
    // 3. wait for the card to be inserted
    while( !getCD());       // check CD switch
    Delayms( 100);          // wait contacts de-bounce
    if ( initMedia())       // init card
    {  // if error code returned
        clrLCD();
        putsLCD( "Failed Init");
        goto End;
    }
            
    // 4. fill 16 groups of N_BLOCK sectors with data 
    LED = 1;            // SD card in use
    clrLCD();
    putsLCD( "Writing\n");
    addr = START_ADDRESS;
    for( j=0; j<16; j++)
    {
        for( i=0; i<N_BLOCKS; i++)
        {
            if (!writeSECTOR( addr+i*j, data))
            {  // writing failed
                putsLCD( "Failed to Write");
                goto End;
            }
        } // i
        putLCD( 0xff);
    } // j
    
    // 5. verify the contents of each sector written
    clrLCD();
    putsLCD( "Verifying\n");    
    addr = START_ADDRESS;
    for( j=0; j<16; j++)
    {
        for( i=0; i<N_BLOCKS; i++)
        {   // read back one block at a time
            if (!readSECTOR( addr+i*j, buffer))
            {   // reading failed
                putsLCD( "Failed to Read");
                goto End;
            }

            // verify each block content
            if ( memcmp( data, buffer, B_SIZE))
            {   // mismatch
                putsLCD( "Failed to Match");
                goto End;
            }
        } // i
        putLCD(0xff);
    } // j

    // 7. indicate successful execution
    clrLCD();
    putsLCD( "   Success!");

End:    
    LED = 0;    // SD card not in use
    // main loop
    while( 1);

} // main

