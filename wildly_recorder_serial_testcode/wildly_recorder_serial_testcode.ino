#include "device.h"        //Header file to change device configuration parameters
#include <Wire.h>          //I2C library
#include <TimeLib.h>       //Time library
#include "RTClib.h"        //RealTimeClock Library for DS1307 and DS3231
#include "SdFs.h"          //Sdcard library
#include "record_queue.h"  //Audio queue header file
#include "input_i2s.h"     //I2S input header file
#include "mixer.h"         //Amplification header file
#include "filter_biquad.h" //Filter header file
#include "MAX17043.h"      //Fuel gauge header file
#include <TeensyID.h>      //Device ID header file
#include <EEPROM.h>        //EEPROM library header file
#include <Snooze.h>        //Sleep mode header file

//******************************Time difference structure****************************//
struct TIME {
  int seconds;
  int minutes;
  int hours;
};
struct TIME startTime, stopTime, diff;

//*******************************Sleep mode objects***********************************//
SnoozeAlarm  alarm1;
SnoozeBlock config_teensy36_t1(alarm1);

elapsedMillis startrecord;

//********************Device RESET MACROS***************************************************//
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

/************************************************************************************
                                  SDCARD  SETTINGS (NEW LIBRARY)                      
************************************************************************************/
// SD_FAT_TYPE = 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI)
#endif  // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 1
SdFat sd;
File frec;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile frec,ftx;
#else  // SD_FAT_TYPE
//#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE
//************************************************************************************//

/*Change the value of recording flag to enable 4k,8k,16k write
  1 : 4K writes
  2 : 8K writes
  3 : 16K writes
  4 : 12K writes
  comment RECORDING_FLAG to select 512bytes writes
*/
#define RECORDING_FLAG  5

#if RECORDING_FLAG == 1
byte buffer[4096] = {0};
#elif RECORDING_FLAG == 2
byte buffer[8192] = {0};
#elif RECORDING_FLAG == 3
byte buffer[16384] = {0};
#elif RECORDING_FLAG == 4
byte buffer[12288] = {0};
#else
byte buffer[512] = {0};
#endif

//*******************************RTC function*****************************************//

RTC_DS1307 RTC;

long int syncProvider()
{
  return RTC.now().unixtime();
}

//***************Audio library objects and connections*********//
AudioInputI2S            i2s1;
AudioAmplifier           amp1;
AudioFilterBiquad        biquad1;
AudioRecordQueue         queue1;
AudioConnection          patchCord1(i2s1, 0, amp1, 0);
AudioConnection          patchCord2(amp1, biquad1);
AudioConnection          patchCord3(biquad1, queue1);

//********** recording_flags **********************************//
bool recordInProgress = false;
bool recording_flag = false;
bool recording_started = false;
bool first_file_flag = false;
bool recording_mode = false;

//********** recording_constants **********************************//

#define SUB_CHUNK_1_SIZE  16
#define AUDIO_FORMAT      1     //WAV FILE
#define NUM_CHANNELS      1     //MONO:1  STEREO:2
#define BITS_PER_SAMPLE   16    
#define BITS_PER_BYTE     8
#define BYTE_RATE         New_samplerate*NUM_CHANNELS*(BITS_PER_SAMPLE/BITS_PER_BYTE) // SAMPLE_RATE x channels x (BITS_PER_SAMPLE / 8)
#define BLOCK_ALIGN       NUM_CHANNELS*(BITS_PER_SAMPLE/BITS_PER_BYTE)

//********** recording_variables **********************************//
unsigned long ChunkSize = 0L;
unsigned long Subchunk2Size = 0L;
unsigned long recByteSaved = 0L;
unsigned long NumSamples = 0L;
byte byte1, byte2, byte3, byte4;
char const* Firm_rev = "Ver1.0";
unsigned long sizeof_icmt_chunk = 0;
unsigned long length_icmt_comment = 200;
char icmt_comment[200]={0};
byte Fdate = 0, Fmonth = 0, Fhour = 0, Fmin = 0, Fsec = 0;
uint16_t Fyear = 0;
char filename[50] = {0};
char folder_buffer[30] = {0};
uint32_t filenumber = 0;
uint16_t folder_variable = 0;
char folder_files[30] = {0};
int previous_day = 0;
static char header[264] = {0};
byte prev_min = 0,prev_sec = 0;

uint8_t serial[4];
uint32_t device_id_number = 0;
uint32_t freeClusterCount = 0;
float freeKB = 0;
int read_button_status = 0;
uint32_t dedicated_bytes = 0;

long previous_millis = 0;
long rec_interval = 0;
byte T1_WINDOW = 0,T2_WINDOW = 0,T3_WINDOW = 0,T4_WINDOW = 0;
//*************************EEPROM variables***********************************//

unsigned int eeprom_addr = 0,eeprom_var=0,eeprom_value=0;

char parameter_set_buffer[50] = {0};
byte parameter_set_counter = 0;
byte random_variable = 0;
char random_buffer[50]={0};
byte New_micgain = 0,New_rectime = 0;
uint32_t New_samplerate = 0;
//********** recording_functions **********************************//
void startRecording(void);
void stopRecording(void);
void recording(void);
void writeWavHeader(void);
void continueRecording(void);
void dateTime(uint16_t* date, uint16_t* time);
void writeOutHeader(void);
void Set_Icmt_comment(void);
char * wavHeader(void);
void read_EEPROM(void);

void differenceBetweenTimePeriod(struct TIME start, struct TIME stop, struct TIME *diff);

//*************************************Function to calculate time difference**********************************/

void differenceBetweenTimePeriod(struct TIME start, struct TIME stop, struct TIME *diff) 
{
  if (stop.seconds > start.seconds) 
  {
    --start.minutes;
    start.seconds += 60;
  }
  diff->seconds = start.seconds - stop.seconds;
  if (stop.minutes > start.minutes) 
  {
    --start.hours;
    start.minutes += 60;
  }
  diff->minutes = start.minutes - stop.minutes;
  diff->hours = start.hours - stop.hours;
}
//***************************MAIN CODE************************************
void setup()
{
  Serial.begin(BAUD);                      //initialize UART 
  Wire.begin();                            //initialize I2C
  RTC.begin();                             //initialize RTC

  //Update RTC time with machine time
  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (now.unixtime() < compiled.unixtime()) {
  DEBUG_PRINT(("RTC is older than compile time! Updating"));
  // following line sets the RTC to the date & time this sketch was compiled    
  RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  
  pinMode_setup();                         //initialize I/O pins
  FuelGauge.begin();                       //initialize fuel gauge

  read_EEPROM();                           //read configuration data from EEPROM

  /*Read teensy serial ID*/
  teensySN(serial);
  device_id_number = teensyUsbSN();
  DEBUG_PRINT(("USB Serialnumber: %u \n",device_id_number));

  /*Set device time*/
  setTime(Fhour, Fmin, Fsec, Fdate, Fmonth,Fyear);
  /*Check SDcard connection*/
  SDcard_check();         
  /*Initialize microphone and audio queue*/
  microphone_initialization();
  
  first_file_flag = true;

  prev_sec = second();
  DEBUG_PRINT(("Previous second"));
  DEBUG_PRINT((prev_sec));
  prev_min = minute();
  DEBUG_PRINT(("Previous minute"));
  DEBUG_PRINT((prev_sec));
  previous_day = day();  
  DEBUG_PRINT(("Previous day"));
  DEBUG_PRINT((previous_day));
  dedicated_bytes = (New_samplerate * 2 * New_rectime);
  DEBUG_PRINT(("START!"));
  DEBUG_PRINT(("dedicated_bytes!")); 
  DEBUG_PRINT((dedicated_bytes));

  previous_millis = millis();

  DEBUG_PRINT(("hour"));
  DEBUG_PRINT((hour()));

  rec_interval = (New_rectime * 1000);
  DEBUG_PRINT(("rec interval")); 
  DEBUG_PRINT((rec_interval));  

  recording_mode = true;
}


//*********************************Firmware Loop*******************************************//
void loop()
{
     /*Read data from terminal*/
     while(Serial.available() > 0) 
     {
       parameter_set_buffer[parameter_set_counter] = Serial.read();
       parameter_set_counter++;
     }
     /*Pass (SMF Space) to read Sample rate*/
     if(strstr(parameter_set_buffer,"SMF "))
     {
       eeprom_value = EEPROM.read(1);
       Serial.println(eeprom_value);   
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);  
     }
     /*Pass (SMF,samplerate) to set sample rate of device */
     else if(strstr(parameter_set_buffer,"SMF,"))
     {
       random_variable = Find_FirstCharacter(parameter_set_buffer, ',');
       memcpy(random_buffer,&parameter_set_buffer[random_variable+1],2);
       eeprom_value = atoi(random_buffer);
       EEPROM.write(1,eeprom_value);
       Serial.printf("SMF:%d\n",eeprom_value);
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);
     }
     /*Pass (MCG SPACE) to read Mic gain data*/
     else if(strstr(parameter_set_buffer,"MCG "))
     {
       eeprom_value = EEPROM.read(2);
       Serial.println((eeprom_value));
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);
     }
     /*Pass (MCG,micgain) to set micgain value*/
     else if(strstr(parameter_set_buffer,"MCG,"))
     {
       random_variable = Find_FirstCharacter(parameter_set_buffer, ',');
       memcpy(random_buffer,&parameter_set_buffer[random_variable+1],2);
       eeprom_value = atoi(random_buffer);
       EEPROM.write(2,eeprom_value);
       Serial.printf("MCG:%d\n",eeprom_value);
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);
     }
     /*Pass (RCT SPACE) to read Recording time in seconds*/
     else if(strstr(parameter_set_buffer,"RCT "))
     {
       eeprom_value = EEPROM.read(3);
       Serial.println(eeprom_value);
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);
     }
     /*Pass (RCT,time) to set recording time*/
     else if(strstr(parameter_set_buffer,"RCT,"))
     {
       random_variable = Find_FirstCharacter(parameter_set_buffer, ',');
       memcpy(random_buffer,&parameter_set_buffer[random_variable+1],2);
       eeprom_value = atoi(random_buffer);
       EEPROM.write(3,eeprom_value);
       Serial.printf("RCT:%d\n",eeprom_value);   
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);   
     }
     /*Pass (T:T1_WINDOW,T2_WINDOW,T3_WINDOW,T4_WINDOW) to set recording time window */
     else if(strstr(parameter_set_buffer,"T:"))
     {
       memcpy(random_buffer,&parameter_set_buffer[2],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(4,eeprom_value);       
       memcpy(random_buffer,&parameter_set_buffer[5],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(5,eeprom_value);
       memcpy(random_buffer,&parameter_set_buffer[8],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(6,eeprom_value);
       memcpy(random_buffer,&parameter_set_buffer[11],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(7,eeprom_value);
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50); 
       memset(random_buffer, '\0', 50);         
     }
     /*Pass (T#) to read recording window values*/
     else if(strstr(parameter_set_buffer,"T#"))
     {
       eeprom_value = EEPROM.read(4);
       Serial.printf("%02d",eeprom_value);
       Serial.print(",");
       eeprom_value = EEPROM.read(5);
       Serial.printf("%02d",eeprom_value);
       Serial.print(",");
       eeprom_value = EEPROM.read(6);
       Serial.printf("%02d",eeprom_value);
       Serial.print(",");
       eeprom_value = EEPROM.read(7);
       Serial.printf("%02d",eeprom_value);
       Serial.println();
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);  
     }
     /*Pass (BATT) to read Battery values */
     else if(strstr(parameter_set_buffer,"BATT"))
     {      
       Serial.printf("VOLT: %.1fV, Pct:%.1f%", FuelGauge.voltage(), FuelGauge.percent());
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);  
     }
     /*Pass ("TIME:12:14:55") to set device time manually*/
     else if(strstr(parameter_set_buffer,"TIME:"))
     {
       memcpy(random_buffer,&parameter_set_buffer[5],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(8,eeprom_value);
       memcpy(random_buffer,&parameter_set_buffer[8],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(9,eeprom_value);
       memcpy(random_buffer,&parameter_set_buffer[11],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(10,eeprom_value);
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50); 
       memset(random_buffer, '\0', 50);
     }
     /*Pass TIME# to read time*/
     else if(strstr(parameter_set_buffer,"TIME#"))
     {
       eeprom_value = EEPROM.read(8);
       Serial.printf("%02d",eeprom_value);
       Serial.print(":");
       eeprom_value = EEPROM.read(9);
       Serial.printf("%02d",eeprom_value);
       Serial.print(":");
       eeprom_value = EEPROM.read(10);
       Serial.printf("%02d",eeprom_value);
       Serial.println();
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);       
     }
     /*Pass ("DATE:12/11/20") to set date of the device*/
     else if(strstr(parameter_set_buffer,"DATE:"))
     {
       memcpy(random_buffer,&parameter_set_buffer[5],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(11,eeprom_value);
       memcpy(random_buffer,&parameter_set_buffer[8],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(12,eeprom_value);
       memcpy(random_buffer,&parameter_set_buffer[11],2);
       DEBUG_PRINT((random_buffer));
       eeprom_value = atoi(random_buffer);
       EEPROM.write(13,eeprom_value);
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50); 
       memset(random_buffer, '\0', 50);
     }
     /*Pass DATE# to read date*/
     else if(strstr(parameter_set_buffer,"DATE#"))
     {
       eeprom_value = EEPROM.read(11);
       Serial.printf("%02d",eeprom_value);
       Serial.print(":");
       eeprom_value = EEPROM.read(12);
       Serial.printf("%02d",eeprom_value);
       Serial.print(":");
       eeprom_value = EEPROM.read(13);
       Serial.printf("%02d",eeprom_value);
       Serial.println();
       parameter_set_counter = 0;
       memset(parameter_set_buffer, '\0', 50);      
     }
     /*Pass RST to reset device manually*/
     else if(strstr(parameter_set_buffer,"RST"))
     {
        CPU_RESTART
     }
     /*Pass REC to start recording*/
     else if(strstr(parameter_set_buffer,"REC"))
     {
      while(1)
      {
        recording(); 
      }
     }
}

//Uncomment this to enable queue profiling
/*
void loop()
{ 
  static uint32_t t0=0, t2=0, tx, ty;
  tx=micros(),
  recording();
  ty=micros()-tx;
  if((ty)>t2) t2=ty;
  if(tx-t0>1000000)
  {
    Serial.printf(" %6d %3d\n",t2, AudioMemoryUsageMax());
    t2=0;
    AudioMemoryUsageMaxReset();
    t0=tx;
  }
}
*/

void read_EEPROM(void)
{

  New_samplerate = EEPROM.read(1);
  New_micgain = EEPROM.read(2);
  New_rectime = EEPROM.read(3);
  T1_WINDOW = EEPROM.read(4);
  T2_WINDOW = EEPROM.read(5);
  T3_WINDOW = EEPROM.read(6); 
  T4_WINDOW = EEPROM.read(7);

  DEBUG_PRINT(("T1_WINDOW"));
  DEBUG_PRINT((T1_WINDOW));
  DEBUG_PRINT(("T2_WINDOW"));
  DEBUG_PRINT((T2_WINDOW));
  DEBUG_PRINT(("T3_WINDOW"));
  DEBUG_PRINT((T3_WINDOW));
  DEBUG_PRINT(("T4_WINDOW"));
  DEBUG_PRINT((T4_WINDOW));

  Fhour = EEPROM.read(8);
  Fmin = EEPROM.read(9);
  Fsec = EEPROM.read(10);
  Fdate = EEPROM.read(11);
  Fmonth = EEPROM.read(12);
  Fyear = EEPROM.read(13);
  Fyear = (2000+Fyear);

  DEBUG_PRINT(("Fhour"));
  DEBUG_PRINT((Fhour));
  DEBUG_PRINT(("Fmin"));
  DEBUG_PRINT((Fmin));
  DEBUG_PRINT(("Fsec"));
  DEBUG_PRINT((Fsec));
  DEBUG_PRINT(("Fdate"));
  DEBUG_PRINT((Fdate));  
  DEBUG_PRINT(("Fmonth"));
  DEBUG_PRINT((Fmonth));
  DEBUG_PRINT(("Fyear"));
  DEBUG_PRINT((Fyear));  
    
  if(New_samplerate == 44)
  {
    New_samplerate = 44100; 
  }
  else if((New_samplerate == 8) || (New_samplerate == 16) ||(New_samplerate == 32) || (New_samplerate == 48))
  {
    DEBUG_PRINT(("CUSTOM SAMPLE RATE"));
    New_samplerate = New_samplerate*1000;
  }
  else
  {
    DEBUG_PRINT(("DEFAULT SAMPLE RATE"));
    New_samplerate = SAMPLE_RATE;    
  }

  if(New_rectime > 60)
  {
    DEBUG_PRINT(("DEFAULT REC TIME"));
    New_rectime = REC_TIME;        
  }
  else
  {
    DEBUG_PRINT(("CUSTOM REC TIME"));
    DEBUG_PRINT((New_rectime));
  }

  if(New_micgain < 30)
  {
    DEBUG_PRINT(("CUSTOM MIC GAIN"));
  }
  else
  {
    DEBUG_PRINT(("DEFAULT MIC GAIN"));  
    New_micgain = MIC_GAIN;
  }
  
  DEBUG_PRINT((New_samplerate));
  DEBUG_PRINT((New_micgain));
  DEBUG_PRINT((New_rectime));  

  
}
//*****************************************************************************************************************/
//Set wave file header with Operator, Device ID, Batt voltage and %
void Set_Icmt_comment(void)
{
  snprintf(icmt_comment, sizeof(icmt_comment), "Operator:%s, DeviceID:%s%lu, Batt: %.1fV, Pct:%.1f, Signal: %s-%s, Firm_rev: %s, Timestamp:%d/%02d/%02d-%02d:%02d:%02d, Latitude:%s, Longitude:%s,Clock:%d ", "NA",FILE_NAME,device_id_number, FuelGauge.voltage(), FuelGauge.percent(),"NA","NA",Firm_rev,year(),month(),day(),hour(),minute(),second(),"NA","NA",F_CPU);
  DEBUG_PRINT((icmt_comment));
}

//*****************************************************************************************************************/

//initialize microphone and sdcard
void microphone_initialization()
{
    AudioMemory(MEMORY_VALUE);                    //initialize audio memory          
    amp1.gain(New_micgain);                       //Set mic gain
    biquad1.setHighpass(0, 200, 0.707);           //200 is cutoff frequency
    setI2SFreq(New_samplerate);                   //Set sample rate
    queue1.begin();                               //start audio queue
}

//*****************************************Function to check SDcard connection and initialization*******************************************************/

void SDcard_check(void)
{
  if (!(sd.begin(SdioConfig(FIFO_SDIO))))
  {
    DEBUG_PRINT(("SD Error!"));
    // stop here, but print a message repetitively
    digitalWrite(LED_UPL,HIGH); // SD LED set
    while (1)
    {
      DEBUG_PRINT(("Unable to access the SD card"));
      digitalWrite(ERR_LED,HIGH); // ERRLED set
      delay(500);
      digitalWrite(ERR_LED,LOW); // ERRLED set
      delay(500);
    }
  }
  previous_day = day();
  FsDateTime::callback = dateTime;    //callback function to set time on recorded files
}
//*************************Pin initialisation********************************************
void pinMode_setup()
{
  pinMode(BUTTON,INPUT_PULLUP);
  pinMode(MODE_SEL_SWITCH,INPUT);
  pinMode(EXTRASWITCH,INPUT);
  pinMode(LED_REC, OUTPUT);
  pinMode(ERR_LED,OUTPUT);
  pinMode(LED_UPL,OUTPUT);
  pinMode(GSM_RTS, OUTPUT);
  pinMode(GSM_CTS, INPUT);
  pinMode(PWR_PIN, OUTPUT);
  pinMode(NTWRK_STAT, INPUT);
  pinMode(LED_NETSTAT,OUTPUT);
}

//*********************************Start Recording******************************************************

void startRecording(void)
{
  previous_millis = millis();
  /*Create a directory if day has changed*/
  if ((day() > previous_day))
  {
    DEBUG_PRINT(("New DAY"));
    previous_day = day();
    snprintf(folder_files, sizeof(folder_files), "%s%d%02d%02d_%02d", FOLDER, year(), month(), day(), hour());
    if (!sd.mkdir(folder_files))
    {
      DEBUG_PRINT(("Create Folder1 failed"));
    }
  }
  /*Create a directory if day has changed, end of the month*/
  else if ((previous_day >= 28) && (day() == 1))
  {
    DEBUG_PRINT(("New Month"));
    previous_day = day();
    snprintf(folder_files, sizeof(folder_files), "%s%d%02d%02d_%02d", FOLDER, year(), month(), day(), hour());
    if (!sd.mkdir(folder_files))
    {
      DEBUG_PRINT(("Create Folder1 failed"));
    }
  }
  /*If first file, create a directory*/
  else if (first_file_flag)
  {
    DEBUG_PRINT(("First file"));
    first_file_flag = false;
    snprintf(folder_files, sizeof(folder_files), "%s%d%02d%02d_%02d", FOLDER, year(), month(), day(), hour());
    if (!sd.mkdir(folder_files))
    {
      DEBUG_PRINT(("Create Folder1 failed"));
    }
  }
  recordInProgress = true;
  digitalWrite(LED_UPL, LOW);
  DEBUG_PRINT(("startRecording"));
  DEBUG_PRINT(("new filename"));
  Fhour = hour(); Fmin = minute(); Fsec = second(); Fdate = day(); Fmonth = month(); Fyear = year();
  snprintf(filename, sizeof(filename), "%s/%s%d%02d%02d_%02d%02d%02d.wav", folder_files, FILE_NAME, Fyear, Fmonth, Fdate, Fhour, Fmin, Fsec);
  DEBUG_PRINT((filename));
  digitalWrite(LED_REC, HIGH);
  Set_Icmt_comment();

  // Get the count of free clusters in the volume.
  freeClusterCount = sd.freeClusterCount();
  DEBUG_PRINT(("Free clusters: "));
  DEBUG_PRINT((freeClusterCount));

  // Calculate free space in KB.
  freeKB = 0.512 * freeClusterCount * sd.sectorsPerCluster();
  DEBUG_PRINT(("Free space: "));
  DEBUG_PRINT((freeKB));
  DEBUG_PRINT((" KB (KB = 1000 bytes)"));

  startrecord = 0;

  /*Open file*/
  frec = sd.open(filename, O_CREAT | O_TRUNC | O_RDWR); 
  if (!frec)
  {
    while (1)
    {
      DEBUG_PRINT(("Can't open file!"));
      digitalWrite(ERR_LED, HIGH); // ERRLED set
      delay(500);
      digitalWrite(ERR_LED, LOW); // ERRLED set
      delay(500);
    }
  }

  prev_sec = second();
  prev_min = minute();
  DEBUG_PRINT(("File OPEN!"));
  recByteSaved = 0L;
  recording_started = true;
  memcpy(header, wavHeader(), 264);
  frec.seek(0);
  frec.write(header, HEADER_SIZE);
  DEBUG_PRINT(("OPENED!"));
}


//******************************Continue recording**********************************************

void continueRecording(void)
{
  if(recording_started)
  {

#if RECORDING_FLAG == 1
  if (queue1.available() >= 16)       //accumulate 4K bytes of data in queue
  {
    for(int k=0;k<16;k++)
    {
       uint16_t buff_count = k*256; 
       memcpy(buffer+buff_count, queue1.readBuffer(), 256);
       queue1.freeBuffer();
    }
    frec.write(buffer, 4096);      // write all 4K bytes to the SD card
    recByteSaved += 4096;
  }

#elif RECORDING_FLAG == 2
  if (queue1.available() >= 32)     //accumulate 8K bytes of data in queue
  {
    for(int k=0;k<32;k++)
    {
       uint16_t buff_count = k*256;
       memcpy(buffer+buff_count, queue1.readBuffer(), 256);
       queue1.freeBuffer();
    }
    frec.write(buffer, 8192);      // write all 8K bytes to the SD card
    recByteSaved += 8192;
  }
#elif RECORDING_FLAG == 3
  if (queue1.available() >= 64)     ////accumulate 16K bytes of data in queue
  {
    for(int k=0;k<64;k++)
    {
       uint16_t buff_count = k*256; 
       memcpy(buffer+buff_count, queue1.readBuffer(), 256);
       queue1.freeBuffer();
    }
    frec.write(buffer, 16384);      // write all 16K bytes to the SD card
    recByteSaved += 16384;
  }
#elif RECORDING_FLAG == 4
  if (queue1.available() >= 48)       //accumulate 12K bytes of data in queue
  {
    for(int k=0;k<48;k++)
    {
       uint16_t buff_count = k*256;
       memcpy(buffer+buff_count, queue1.readBuffer(), 256);
       queue1.freeBuffer();
    }
    frec.write(buffer, 12288);      // write all 12K bytes to the SD card
    recByteSaved += 12288;
  }  
#else
  if (queue1.available() >= 2)          //accumulate 512 bytes of data in queue
  {
    memcpy(buffer, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    memcpy(buffer + 256, queue1.readBuffer(), 256);
    queue1.freeBuffer();
    frec.write(buffer, 512);                // write all 512 bytes to the SD card
    recByteSaved += 512;
  }
#endif
  }
}

//***********************************************Stop recording*********************************
void stopRecording(void)
{
  uint16_t eof_rec = 0;
  DEBUG_PRINT(("stopRecording"));

  memcpy(header, wavHeader(), 264);
  frec.seek(0);
  frec.write(header, HEADER_SIZE);
  eof_rec = (Subchunk2Size + HEADER_SIZE);
  frec.seek(eof_rec);
  frec.close();
  digitalWrite(LED_REC, LOW);
  digitalWrite(LED_UPL, HIGH);
  recording_started = false;
  recordInProgress = false;
  recording_flag = true;
  filenumber++;

  /*BlinK lED, if Sdcard space is less than 1MB*/
  if (freeKB < 1000)
  {
    while (1)
    {
      digitalWrite(LED_REC, LOW); // ERRLED set
      digitalWrite(ERR_LED, HIGH); // ERRLED set
      digitalWrite(LED_UPL, LOW); // LED_UPL clear
      delay(500);
      digitalWrite(ERR_LED, LOW); // ERRLED clear
      digitalWrite(LED_UPL, HIGH); // LED_UPL set
      delay(500);
    }
  }
}
//******************************Function to update wav header***************************************************
char * wavHeader(void)
{
  static char wavheader_buffer[264] = {0};

  sizeof_icmt_chunk = (sizeof(icmt_comment)+ 12);
  ChunkSize = recByteSaved + BYTES_IN_HEADER;
  Subchunk2Size = ChunkSize - BYTES_IN_HEADER;
    
  strcpy(wavheader_buffer,"RIFF");
  strcpy(wavheader_buffer+8,"WAVE");
  strcpy(wavheader_buffer+12,"fmt ");
  strcpy(wavheader_buffer+36,"LIST");
  strcpy(wavheader_buffer+44,"INFO");
  strcpy(wavheader_buffer+48,"ICMT");
  strcpy(wavheader_buffer+56,icmt_comment);
  strcpy(wavheader_buffer+256,"data");
  *(int32_t*)(wavheader_buffer+4)=ChunkSize;
  *(int32_t*)(wavheader_buffer+16)= SUB_CHUNK_1_SIZE;// chunk_size
  *(int16_t*)(wavheader_buffer+20)= AUDIO_FORMAT; // PCM 
  *(int16_t*)(wavheader_buffer+22)=NUM_CHANNELS;// numChannels 
  *(int32_t*)(wavheader_buffer+24)= New_samplerate; // sample rate 
  *(int32_t*)(wavheader_buffer+28)= BYTE_RATE; // byte rate
  *(int16_t*)(wavheader_buffer+32)=BLOCK_ALIGN; // block align
  *(int16_t*)(wavheader_buffer+34)=BITS_PER_SAMPLE; // bits per sample 
  *(int32_t*)(wavheader_buffer+40)=sizeof_icmt_chunk;
  *(int32_t*)(wavheader_buffer+52)=length_icmt_comment;   
  *(int32_t*)(wavheader_buffer+260)=Subchunk2Size;

   return wavheader_buffer;
}
//**************************************************************************************************


/* User provided date time callback function.
   See SdFile::dateTimeCallback() for usage.
*/
void dateTime(uint16_t* date, uint16_t* time)
{

  // DateTime now = rtc.now();
  // User gets date and time from GPS or real-time
  // clock in real callback function

  // return date using FAT_DATE macro to format fields
  // *date = FAT_DATE(year, month, day);
  *date = FAT_DATE(year(), month(), day());

  // return time using FAT_TIME macro to format fields
  //  *time = FAT_TIME(hours, minutes, seconds);
  *time = FAT_TIME(hour(), minute(), second());
}
//*****************************************************************************************************************/
//Function to set sample rate
void setI2SFreq(int freq)
{
  typedef struct
  {
    uint8_t mult;
    uint16_t div;
  } tmclk;

  const int numfreqs = 14;
  const int samplefreqs[numfreqs] = { 8000, 11025, 16000, 22050, 32000, 44100, (int)44117.64706 , 48000, 88200, (int)44117.64706 * 2, 96000, 176400, (int)44117.64706 * 4, 192000};

#if (F_PLL==16000000)
  const tmclk clkArr[numfreqs] = {{16, 125}, {148, 839}, {32, 125}, {145, 411}, {64, 125}, {151, 214}, {12, 17}, {96, 125}, {151, 107}, {24, 17}, {192, 125}, {127, 45}, {48, 17}, {255, 83} };
#elif (F_PLL==72000000)
  const tmclk clkArr[numfreqs] = {{32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {128, 1125}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375}, {249, 397}, {32, 51}, {185, 271} };
#elif (F_PLL==96000000)
  const tmclk clkArr[numfreqs] = {{8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {32, 375}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125}, {151, 321}, {8, 17}, {64, 125} };
#elif (F_PLL==120000000)
  const tmclk clkArr[numfreqs] = {{32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {128, 1875}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625}, {178, 473}, {32, 85}, {145, 354} };
#elif (F_PLL==144000000)
  const tmclk clkArr[numfreqs] = {{16, 1125}, {49, 2500}, {32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {4, 51}, {32, 375}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375} };
#elif (F_PLL==168000000)
  const tmclk clkArr[numfreqs] = {{32, 2625}, {21, 1250}, {64, 2625}, {21, 625}, {128, 2625}, {42, 625}, {8, 119}, {64, 875}, {84, 625}, {16, 119}, {128, 875}, {168, 625}, {32, 119}, {189, 646} };
#elif (F_PLL==180000000)
  const tmclk clkArr[numfreqs] = {{46, 4043}, {49, 3125}, {73, 3208}, {98, 3125}, {183, 4021}, {196, 3125}, {16, 255}, {128, 1875}, {107, 853}, {32, 255}, {219, 1604}, {214, 853}, {64, 255}, {219, 802} };
#elif (F_PLL==192000000)
  const tmclk clkArr[numfreqs] = {{4, 375}, {37, 2517}, {8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {1, 17}, {8, 125}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125} };
#elif (F_PLL==216000000)
  const tmclk clkArr[numfreqs] = {{32, 3375}, {49, 3750}, {64, 3375}, {49, 1875}, {128, 3375}, {98, 1875}, {8, 153}, {64, 1125}, {196, 1875}, {16, 153}, {128, 1125}, {226, 1081}, {32, 153}, {147, 646} };
#elif (F_PLL==240000000)
  const tmclk clkArr[numfreqs] = {{16, 1875}, {29, 2466}, {32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {4, 85}, {32, 625}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625} };
#endif

  for (int f = 0; f < numfreqs; f++) {
    if ( freq == samplefreqs[f] ) {
      while (I2S0_MCR & I2S_MCR_DUF);
      I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) | I2S_MDR_DIVIDE((clkArr[f].div - 1));
      return;
    }
  }
}

//****************************************FUnction to record audio file***********************************************************/
void recording(void)
{
  /*Check if current time comes under recording time Window*/
  if ((((hour() >= T1_WINDOW) && (hour() < T2_WINDOW)) || ((hour() >= T3_WINDOW) && (hour() < T4_WINDOW))) && (!recordInProgress))
  {
    /*Start recording if first file or time gap between 2 files is rec_interval*/
    if((((millis() - previous_millis) >= (rec_interval)) && (!recordInProgress)) || first_file_flag)
    {
      startRecording();
    }
  }
  /*Write samples to file if available*/
  else if((queue1.available()>0) && frec)
  {
    continueRecording();
  }
  /*Stop recording after dedicated amount of data written*/
  else if (recordInProgress && (recByteSaved >= dedicated_bytes))
  {
    stopRecording();
  }
  /*Check if device falls between T2-T3 time window*/
  else if (((hour() >= T2_WINDOW) && (hour() < T3_WINDOW)) && (!recordInProgress))
  {
    recording_mode = false;
    DEBUG_PRINT(("stage 3"));
    DEBUG_PRINT(("Time:%d", hour()));
    DEBUG_PRINT(("Sleep!"));
    digitalWrite(LED_REC, LOW);

    startTime.hours = hour();
    startTime.minutes = minute();
    startTime.seconds = second();

    stopTime.hours = T3_WINDOW;
    stopTime.minutes = 0;
    stopTime.seconds = 0;

    differenceBetweenTimePeriod(stopTime, startTime, &diff);
   
    alarm1.setRtcTimer(diff.hours, diff.minutes, diff.seconds);

    /*Disable I2S clock*/
    SIM_SCGC6 &= ~SIM_SCGC6_I2S;   
    /*Goto hibernate mode for calculated time*/                 
    Snooze.hibernate(config_teensy36_t1);
    /*Enable I2S clock after wakeup*/
    SIM_SCGC6 |= SIM_SCGC6_I2S;
    /*Sync time*/
    setSyncProvider(syncProvider);
    prev_sec = second();
    prev_min = minute();
  }
  /*Check if time fall between 12 midnight to T1 window*/
  else if (((hour() >= 0) && (hour() < T1_WINDOW)) && (!recordInProgress))
  {
    recording_mode = false;
    DEBUG_PRINT(("stage 4"));
    DEBUG_PRINT(("Time:%d", hour()));
    DEBUG_PRINT(("Sleep!"));
    digitalWrite(LED_REC, LOW);

    startTime.hours = hour();
    startTime.minutes = minute();
    startTime.seconds = second();

    stopTime.hours = T1_WINDOW;
    stopTime.minutes = 0;
    stopTime.seconds = 0;

    differenceBetweenTimePeriod(stopTime, startTime, &diff);

    alarm1.setRtcTimer(diff.hours, diff.minutes, diff.seconds);

    /*Disable I2S clock*/
    SIM_SCGC6 &= ~SIM_SCGC6_I2S;
    /*Goto hibernate mode for calculated time*/
    Snooze.hibernate(config_teensy36_t1);
    /*Enable I2S clock*/
    SIM_SCGC6 |= SIM_SCGC6_I2S;
    /*Sync time*/
    setSyncProvider(syncProvider);
    prev_sec = second();
    prev_min = minute();

  }
  /*Check if time falls between T4 window and 12 midnight*/
  else if (((hour() >= T4_WINDOW) && (hour() < 24)) && (!recordInProgress))
  {
    recording_mode = false;
    DEBUG_PRINT(("stage 5"));
    DEBUG_PRINT(("Time:%d", hour()));
    DEBUG_PRINT(("Sleep!"));
    digitalWrite(LED_REC, LOW);

    startTime.hours = hour();
    startTime.minutes = minute();
    startTime.seconds = second();

    stopTime.hours = 24;
    stopTime.minutes = 0;
    stopTime.seconds = 0;

    differenceBetweenTimePeriod(stopTime, startTime, &diff);

    alarm1.setRtcTimer(diff.hours, diff.minutes, diff.seconds);

    /*Disable I2S clock*/
    SIM_SCGC6 &= ~SIM_SCGC6_I2S;
    /*Goto hibernate for calculated time*/
    Snooze.hibernate(config_teensy36_t1);
    /*Enable I2S clock*/
    SIM_SCGC6 |= SIM_SCGC6_I2S;
    /*Sync time*/
    setSyncProvider(syncProvider);
    prev_sec = second();
    prev_min = minute();
     
  }
  //Pass RST in terminal to RESET DEVICE
  else if(strstr(parameter_set_buffer,"RST"))
  {
     CPU_RESTART
  }
  /*Pass QSIZE to know amount of queue occupied*/
  else if(strstr(parameter_set_buffer,"QSIZE"))
  {
    parameter_set_counter = 0;
    memset(parameter_set_buffer, '\0', 50);
    Serial.printf("QUEUE:%3d\n", AudioMemoryUsage());
  }
  /*Read data from terminal or serial*/
  while(Serial.available() > 0) 
  {
    parameter_set_buffer[parameter_set_counter] = Serial.read();
    parameter_set_counter++;
  }  
}
//*****************************************************************************************************************/

//Function to find position of particular character in the string(pass string and char to find in argument)
int Find_FirstCharacter(char *str, char ch)
{
  uint16_t i = 0;

  while (*str)
  {
    if (*str == ch)
    {
      return i;
    }
    i++;
    str++;
  }
  return -1;
}
