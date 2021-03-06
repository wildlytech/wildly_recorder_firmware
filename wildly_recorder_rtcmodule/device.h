/*************************************************************************
 * CHANGE OPERATING MODE VALUES TO CONFIGURE DEVICE IN DIFFERENT MODES
 * OPERATING_MODE   1 (RECORDER MODE)
 * OPERATING_MODE   2 (CONFIGURE DEVICE)
*****************************************************************************/

#define OPERATING_MODE    1

//Enable record only mode to use device only as recorder

//#define DEBUG             1                           //uncomment this to enable serial print AT command output

#ifdef DEBUG //TODO: Print filename
#define DEBUG_PRINT(x)  \
Serial.print(x); \
Serial.print(" : Line: "); \
Serial.print(__LINE__);   \
Serial.println ("");
#else
#define DEBUG_PRINT(x) do {} while (0)
#endif

//************defines*****************//
#define BAUD             115200
#define NEW_BAUDRATE     921600 
#define AUDIO_MEMORY     600

#define BYTES_IN_HEADER   256
#define BYTES_WAVHEADER   36
#define HEADER_SIZE       264
//***********pin_configuration**********//
#define GSM_RTS           33
#define GSM_CTS           34
#define BUTTON            4
#define LED_REC           36 // RECLED used to indicate initialization
#define LED_UPL           37 // SDLED used for upload indication
#define LED_NETSTAT       38 // NETSTATLED
#define ERR_LED           39 // ERR LED used to indicate Error  
#define PWR_PIN           20 //  PWR pin of GSM
#define NTWRK_STAT        24 //  Network status pin of GSM
#define MODE_SEL_SWITCH   32 //This slider switch is interfaced externally

#define REC_TIME          10 //  Length of recorded wav file


//**********************DEVICE ACTIVE TIME WINDOW*********//
/*Device will work from T1 to T2 window and T3 to T4 window, 
  rest of the time it will be in sleep mode*/
#define T1_WINDOW         0
#define T2_WINDOW         1
#define T3_WINDOW         22
#define T4_WINDOW         24
//***********MIC constants**********//
#define SAMPLE_RATE       16000
#define MIC_GAIN          15

#define FILE_NAME         "A"                 //Add single char for filename
#define FOLDER            "DEV"
