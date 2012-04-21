/*
** AudioPWM.h
*/

// AudioPWM error codes

#define APWM_Inited          -1;
#define APWM_Started          0;
#define APWM_Halted         999;
#define APWM_Open_Failed     11;
#define APWM_File_Not_RIFF   12;
#define APWM_Chunk_Not_WAVE  13;
#define APWM_No_FMT_keyword  14;
#define APWM_Is_Compressed   15;
#define APWM_No_DATA_Word    16;
#define APWM_Period_Too_Long 17;

int playWAV( char *name);
