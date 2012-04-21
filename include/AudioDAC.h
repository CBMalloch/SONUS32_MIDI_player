/*
** AudioDAC.h
*/

// AudioDAC status codes

#define ADAC_Success         0x00010000
#define ADAC_Cancelled       0x000100f0
#define ADAC_Finished        0x000100ff

#define ADAC_Inited          0x00010001
#define ADAC_Started         0x00010002
#define ADAC_Open_Failed     0x00010011
#define ADAC_File_Not_RIFF   0x00010012
#define ADAC_Chunk_Not_WAVE  0x00010013
#define ADAC_No_FMT_keyword  0x00010014
#define ADAC_Is_Compressed   0x00010015
#define ADAC_No_DATA_Word    0x00010016
#define ADAC_Period_Too_Long 0x00010017

int playWAV (char *name);
