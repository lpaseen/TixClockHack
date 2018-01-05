/*
   Tix(tm) clock hack, replaced the PIC16F628 with a small add on board that contains an arduino nano and RTC.

   Copyright Peter Sjoberg
   License: GPL3

   V0.03 - made the intensity menu work


   Socket pinouts
   RA2    RA1
   RA3    RA0
   RA4    RA7
   GND    VCC
   RB0    RB7
   RB1    RB6
   RB2    RB5
   RB3    RB4


   Row2  Row1
   Inc   Row0
   AC    Mode
   N/C   Col0
   GND   VCC
   Col8  Col1
   Col7  Col2
   Col6  Col3
   Col5  Col4


  PIC   Arduino pins
   3      2   AC  (to allow interrupt)
  16      3   Mode Button
   2      4   Inc  Button
  15      5   col0
  13      6   col1
  12      7   col2
  11      8   col3
  10     A0   col4
   9     A1   col5
   8     A2   col6
   7     12   col7
   6     13   col8
    (rows on 9/10/11 to allow PWM)
  17      9  row0
  18     10  row1
   1     11  row2

    A3   WWVB input

    A4   I2C SDA
    A5   I2C SCL


   Arduino pins
    2	A/C (to allow interrupt)
    3	Mode Button
    4	Inc Button
    5	col0
    6	col1
    7	col2
    8	col3
    9	row0
   10	row1
   11	row2
   12	col7
   13	col8
   A0	col4
   A1	col5
   A2	col6
   A3   WWVB input
   A4	I2C sda
   A5	I2C sck

*/


// Interrupt routine to update the display now and then
// from https://github.com/PaulStoffregen/TimerOne
#include <TimerOne.h>

#define COLUMNS 9
#define ROWS 3
#define LEDCNT ROWS*COLUMNS
/* from https://electronics.stackexchange.com/questions/60426/dimming-multiplexed-leds
     One thing to keep in mind is that PWM to intensity mapping is not linear, rather
     it is exponential. Using powers of two, the number of cycles the LED needs to on
     for is 2^(intensity) or 2^(intensity)-1. My preference is for the latter, so we'll
     go with that. This means for 4 intensities (intensity levels 0-3) you need to
     divide your PWM period into 7 cycles and your leds should be on for 0, 1, 3,
     and 7 cycles respectively.

   extrapolated from http://www.pyroelectro.com/tutorials/fading_led_pwm/theory2.html
   to have 6 levels (5 plus off) it need to be 10 periods and the levels are
   0,1,2,4,6,10

*/

#define INTLEVELS 5
const uint8_t levels[] = {0, 1, 2, 4, 6,10};
#define CYCLES 10
//const uint8_t levels[] = {0, 3, 6,11,20};
//#define CYCLES 20
//const uint8_t levels[] = {0, 2, 3, 6, 9,14};
//#define CYCLES 14
//const uint8_t levels[] = {0, 2, 3, 6, 9,15};
//#define CYCLES 15
//const uint8_t levels[] = {0, 2, 3, 6, 9,16};
//#define CYCLES 16
//const uint8_t levels[] = {0, 2, 5, 8,12,20};
//#define CYCLES 20

//Display status
volatile boolean LEDBuffer[ROWS][COLUMNS];
volatile boolean LEDAssembly[ROWS][COLUMNS];
volatile uint8_t currentRow = 0;
volatile static uint8_t cycle = 0;

static uint8_t myHour, myMinute, mySecond;
static uint8_t myYear, myMonth, myDay;

#define LEAP_YEAR(Y)     ( (Y>0) && !(Y%4) && ( (Y%100) || !(Y%400) ) )
static  const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0


static boolean VERBOSE=true;

#include <EEPROM.h>
typedef struct {
  boolean mode24; // clock mode 24h or 12h
  uint8_t intensity; // 5 levels of intensity, 0=off
  uint8_t updateInterval; //  1,4,10,60 seconds
  boolean daylight; // whatever daylight saving is in effect or not
  char    DSTabbrev[6];
  uint8_t DSTweek;      //First, Second, Third, Fourth, or Last week of the month
  uint8_t DSTdow;       //day of week, 1=Sun, 2=Mon, ... 7=Sat
  uint8_t DSTmonth;     //1=Jan, 2=Feb, ... 12=Dec
  uint8_t DSThour;      //0-23
  int16_t DSToffset;        //offset from UTC in minutes
  char    STDabbrev[6];
  uint8_t STDweek;      //First, Second, Third, Fourth, or Last week of the month
  uint8_t STDdow;       //day of week, 1=Sun, 2=Mon, ... 7=Sat
  uint8_t STDmonth;     //1=Jan, 2=Feb, ... 12=Dec
  uint8_t STDhour;      //0-23
  int16_t STDoffset;        //offset from UTC in minutes

  unsigned int checksum;
} Settings_t;

volatile Settings_t settings;
volatile Settings_t stored_settings;

#include <TimeLib.h>
#include <Timezone.h> // https://github.com/JChristensen/Timezone

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);


//Timechange rules are to tricky to add to the menu
//leaving them hardcoded here in the software

//If TimeChangeRules are already stored in EEPROM, comment out the three
//lines above and uncomment the line below.
//Timezone myTZ(100);       //assumes rules stored at EEPROM address 100

TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev

/*

  time -> INC -> brightness (5 steps)

  Set the -> MOD -> time (CLK)        -> MOD ->                    10 hours <->   INC
                    |                                                 MOD
                    |                                              1 hours <->   INC
                    |                                                 MOD
                    |                                             10 Minuten <->   INC
                    |                                                 MOD
                    |                                              1 Minuten <->   INC
                    |                                                 MOD Second=0
                    |
                    |
                   INC
             Setting the mode (M)  -> MOD -> static  -> MOD
                   INC                        INC
             Set DCF offset (D- +)          1s change
                   INC                        INC
             DCF Status (DCF)               5s change
                   INC                        INC
             DCF Date / Time (T-D)         10s change


  Tix menu: see http://www.beathis.ch/lgb/tix/tix_manual_internet.pdf

  New menu:
  Similar to original tix clock
  INC -> change brightness in 5 levels

  MOD for clock setting - same
  press 1: flash left two, then show/set hour (setH)
  press 2: flash 10min then show/set 10 mins (set10M)
  press 3: -"- 1 mins (set1M)
  press 4: flash all LEDs, then start clock at sec=0


  ================
  Beginning is similar to original tix clock
  INC -> change brightness in 5 levels

  MOD for clock setting - same
  * press 1: flash left two, then show/set hour (setH)
  * press 2: flash 10min then show/set 10 mins (set10M)
  * press 3: -"- 1 mins (set1M)
  * press 4: flash all LEDs, then start clock at sec=0
  no action in 2 minutes reset back to time

  MOD2, hold "inc" (not mode as the original is) for 2 sec to set interval
  *  left bar shows all 3 leds on (setInt)
  INC now cycles through the interval on the right, showing seconds for updates, 1,4,10,60 seconds (shows 59)
  no action in 10 seconds reset back to time

  New stuff;
  long press "mode"
  *  when all squares light up advanced mode is in effect (mode2)
  release mode (set24)
  * leftmost top LED stays on, right two (minutes) shows 12 or 24 for 12/24h mode
  * inc to toggle
  click mode for daylight saving time (setDL)
  *  leftmost middle LED comes on, two middle fields shows DL, right is all on or all off
  *  "inc" to toggle daylight saving
  click mode for timezone hours (setTZh)
  *  leftmost bottom LED comes on, right lights show "TIZ" for 1sec, then hours like -4 or +5
  *  inc to change
  click mode for timezone minutes (setTZm)
  *  leftmost bottom LED stays on, right lights show "MIN" for 1sec, then minutes as 00/15/30/45 (no +/-)
  *  inc to change
  click mode for date setting - start with year, left shows a "Y" (setY)
   O  XOX (year 10) (year 1)  , this will work until 2069, starts over at 2018
   O  OXO
   O  OXO
    click mode month, left shows a "M" (which looks like H)  (setM)
   O  XOX (month10) (month1)
   O  XXX
   O  XOX
    click mode day, left shows a "D"  (setD)
   O  XXO (day10) (day1)
   O  XOX
   O  XXO
  click mode again to exit
  no action in 10 seconds reset back to time

  X XXX XX XXX
  X XXX XX XXX
  X XXX XX XXX


================================================================
Serial console commands

#every command is changed to UPPERCASE
#time format is somewhat flexible in regards to separators but not order
#any part left out = 0
!time: 13:14:15 # will only set time
!date: 2017-11-12 #  will only set date
#set date and time
!time: 2017-11-12 13:14:15
!date: 2017/11.12 3:1  $ 2017-11-12 03:01:00
#
#timezone is set by defining ST/DT
!DT:EDT:2:Sun:mar:2:-240  # DT = UTC-4h
!ST:EST:1:Sun:nov:2:-300 # ST = UTC-5h
!mode:12h
!mode:24h
!intensity:2  # 5 levels of intensity, 1/2/3/4/5, 0 is off and not allowed here
!Interval:1
!IntervalRange:1,4,10,60 # it must be 4 but the numbers are configurable

*/

enum mode {time, setH, set10M, set1M, mode2, set24, setDL, setTZh, setTZm, setY, setM, setD,mode3,setInt} mode;
static unsigned long back2time; //milliseconds when it will go back to show time

//Arduino pins
#define BMode 3
#define BInc  4
#define AC    2

const uint8_t rowPins[ROWS] = {9, 10, 11};
const uint8_t colPins[COLUMNS] = {5, 6, 7, 8, A0, A1, A2, 12, 13};

#include "OneButton.h"

// connect the buttons
OneButton buttonMode(BMode,true);
OneButton buttonInc(BInc,true);

//RTC
// http://playground.arduino.cc/code/time
// https://www.pjrc.com/teensy/td_libs_DS1307RTC.html
#define DS3231_ADDR 0x68
boolean RTC_present = false;
tmElements_t tm;
#include <Wire.h>
#include <DS1307RTC.h>  // a basic DS1307/DS3231

/****************************************************************/
void printTime() {
  uint8_t currhour, currminute, currsecond;
  uint8_t curryear, currmonth, currday;
  time_t t = now();

  t=myTZ.toLocal(t, &tcr); // compensate for timezone
  
  curryear = year(t);
  currmonth = month(t);
  currday = day(t);
  currhour = hour(t);
  currminute = minute(t);
  currsecond = second(t);

  Serial.print(curryear);
  Serial.print(F("-"));
  if (currmonth < 10) {
    Serial.print(F("0"));
  }
  Serial.print(currmonth);
  Serial.print("-");
  if (currday < 10) {
    Serial.print(F("0"));
  }
  Serial.print(currday);

  if (currhour < 10) {
    Serial.print(F(" 0"));
  } else {
    Serial.print(F(" "));
  }
  Serial.print(currhour);
  Serial.print(F(":"));
  if (currminute < 10) {
    Serial.print(F("0"));
  }
  Serial.print(currminute);
  Serial.print(F(":"));
  if (currsecond < 10) {
    Serial.print(F("0"));
  }
  Serial.println(currsecond);

} // printTime

/****************************************************************/
// This routine is called by interrupt to update the display
// with what is in LEDBuffer

void updateDisplay() {
  digitalWrite(rowPins[currentRow], HIGH); // turn off the row and get ready for next row
  currentRow++;
  if (currentRow == ROWS) {
    currentRow = 0;

    cycle++;
    if (cycle == CYCLES)
      cycle = 0;
  }


  /*
     Round/lev	1	2	3	4	5
     0		ON	ON	ON	ON	ON
     1		off	ON	ON	ON	ON
     2		off	off	ON	ON	ON
     3		off	off	off	ON	ON
     4		off	off	off	off	ON
  */

  //check the LED for each column

  for (uint8_t c = 0; c < COLUMNS; c++) {
    // 
    if (LEDBuffer[currentRow][c] && (levels[settings.intensity] - cycle) > 0) {
      digitalWrite(colPins[c], HIGH);
    } else {
      digitalWrite(colPins[c], LOW);
    }
  }
  // Columns are set, turn on the row
  digitalWrite(rowPins[currentRow], LOW);
} // updateDisplay

/****************************************************************/
void updateDisplayValues() {
  Timer1.stop();
  memcpy((const void *)LEDBuffer,(const void *)LEDAssembly,sizeof(LEDBuffer));
  Timer1.resume();
} // updateDisplayValues

/****************************************************************/
void displayOff(boolean now = true) {
  //Turn off all LEDs
  for (uint8_t r = 0; r < ROWS; r++) {
    for (uint8_t c = 0; c < COLUMNS; c++) {
      LEDAssembly[r][c] = false;
    }
  }
  if (now)
    updateDisplayValues();
} // displayOff

/****************************************************************/
void showTix() {
  displayOff(false);
  //    0 123 45 678
  // 0  0 111 01 101
  // 1  0  1  01 010
  // 2  0  1  01 101
  //
  LEDAssembly[0][1] = true;
  LEDAssembly[0][2] = true;
  LEDAssembly[0][3] = true;
  LEDAssembly[0][5] = true;
  LEDAssembly[0][6] = true;
  LEDAssembly[0][8] = true;

  LEDAssembly[1][2] = true;
  LEDAssembly[1][5] = true;
  LEDAssembly[1][7] = true;

  LEDAssembly[2][2] = true;
  LEDAssembly[2][5] = true;
  LEDAssembly[2][6] = true;
  LEDAssembly[2][8] = true;
  updateDisplayValues();

} // showTix

/****************************************************************/
// turn on some LEDs
// "Rnd" means random spots, otherwise it fills up by rows
void setFrame(uint8_t minCol, uint8_t maxCol, uint8_t LEDs,boolean Rnd=true) {
  uint8_t ledcnt, maxLed, row, col;
  boolean newVal;

  /*
    Serial.print("DEBUG1:");
    Serial.print(minCol);
    Serial.print(":");
    Serial.print(maxCol);
    Serial.print(":");
    Serial.println(LEDs);
    delay(1000);//PSDEBUG
  */

  maxCol++; // "up to but not including"

  maxLed = (maxCol - minCol) * 3;

  // If all are to lit up, turn on all
  // otherwise turn all off to start
  //  with a blank canvas
  if (LEDs == maxLed)
    newVal = true;
  else
    newVal = false;

  //update all LEDs
  for (row = 0; row < 3; row++) {
    for (col = minCol; col < maxCol; col++) {
      LEDAssembly[row][col] = newVal;
    }
  }

  //If we where to turn on all we are now done
  if (LEDs == maxLed)
    return;

  //randomize what leds to turn on
  if (Rnd){
    ledcnt = 0;
    while (ledcnt < LEDs) {
      row = random(0, 3);
      col = random(minCol, maxCol);
      /*
        Serial.print(" DEBUG2:");
        Serial.print(row);
        Serial.print(":");
        Serial.print(col);
        Serial.print(":");
        Serial.print(ledcnt);
        Serial.println();
        delay(100);//PSDEBUG
      */
      // If not already on, turn it on and inc led count
      if (!LEDAssembly[row][col]) {
        LEDAssembly[row][col] = true;
        ledcnt++;
      }
    }
  } else {
    ledcnt = 0;
    row=0;
    col=minCol;
    while (ledcnt < LEDs) {
      LEDAssembly[row][col] = true;
      ledcnt++;
      col++;
      if (col==maxCol){
        col=minCol;
        row++;
      }
    }
  }
} // setFrame

/****************************************************************/
void drawFrame(uint8_t frame, uint8_t digit,boolean Rnd=true,boolean doNow=true) {
  uint8_t ledcnt, row, col;

  if (digit > 9)
    return;
  if (frame == 0) { // 10 hour
    if (digit > 3)
      return;
    setFrame(0, 0, digit, Rnd);
  } else if (frame == 1) { // 1 hour
    setFrame(1, 3, digit, Rnd);
  } else if (frame == 2) { // 10 minute
    if (digit > 6)
      return;
    setFrame(4, 5, digit, Rnd);
  } else if (frame == 3) { // 1 minute
    setFrame(6, 8, digit, Rnd);
  }

  if (doNow)
    updateDisplayValues();
} // drawFrame
/****************************************************************/
void drawTime(uint16_t number,boolean Rnd=true) {
  uint8_t h10, h1, m10, m1;

  h10 = int(number / 1000);
  h1 = int((number % 1000) / 100);
  m10 = int((number % 100) / 10);
  m1 = int(number % 10);

  if (VERBOSE){
    Serial.print("Drawing ");
    Serial.print(number, DEC);
  }
  /*
  Serial.print("=");
  Serial.print(h10, DEC);
  Serial.print(":");
  Serial.print(h1, DEC);
  Serial.print(":");
  Serial.print(m10, DEC);
  Serial.print(":");
  Serial.print(m1, DEC);
  */
  if (VERBOSE){
    Serial.println();
  }

  drawFrame(0, h10, Rnd);
  drawFrame(1, h1, Rnd);
  drawFrame(2, m10, Rnd);
  drawFrame(3, m1, Rnd);

} // drawTime

/****************************************************************/
// show current time
void drawNow(boolean Rnd=true){
  time_t utc_time,local_time;
  uint8_t curr_hour,curr_minute,curr_second;
  
  utc_time = now();
  local_time = myTZ.toLocal(utc_time, &tcr);
  curr_hour = hour(local_time);
  if (!settings.mode24)
    if (curr_hour>12)
      curr_hour-=12;
  curr_minute = minute(local_time);
  curr_second = second(local_time);

  drawTime(curr_hour * 100 + curr_minute,Rnd);
}

/****************************************************************/
//calculate a checksum of a block
// basic function taken from https://www.arduino.cc/en/Tutorial/EEPROMCrc
unsigned int getCsum(void *block, uint8_t size) {
  unsigned char * data;
  unsigned int i, csum;

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  csum = 0;
  data = (unsigned char *) block;

  for (int index = 0 ; index < size  ; ++index) {
    crc = crc_table[(crc ^ data[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (data[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }

#ifdef DEBUGCSUM
  Serial.print(F(" getCsum: "));
  Serial.print(crc, HEX);
  Serial.print(F(" "));
  Serial.print(size, DEC);
  Serial.println();
#endif

  return (int) crc;
} // getCsum

/****************************************************************/
// save the settings to eeprom
void saveSettings() {
  unsigned char *ptr = (unsigned char *)&settings;

  settings.checksum = getCsum((void*) ptr, sizeof(settings) - 2);
  eeprom_update_block((const void*)&settings, 0, sizeof(settings));

} // saveSettings

/****************************************************************/
// read the settings from eeprom
uint8_t readSettings() {

  Settings_t eesettings;
  unsigned char *eeptr = (unsigned char *)&eesettings;

  eeprom_read_block((void*)&eesettings, 0, sizeof(eesettings));

  if (getCsum((void*) eeptr, sizeof(settings) - 2) == eesettings.checksum) { // if valid csum, make it active
    memcpy(&settings, &eesettings, sizeof(eesettings));
    return true;
  } else {
    return false; // csum invalid so don't change old settings
  }
} // readSettings


/****************************************************************/
/****************************************************************/
void setup() {
  uint16_t curr_hour, curr_minute,DoDelay;

  pinMode(BMode, INPUT);
  pinMode(BInc, INPUT);
  for (uint8_t i = 0; i < ROWS; i++) {
    pinMode(rowPins[i], OUTPUT); digitalWrite(rowPins[i], HIGH);
  }
  for (uint8_t i = 0; i < COLUMNS; i++) {
    pinMode(colPins[i], OUTPUT); digitalWrite(colPins[i], LOW);
  }

  //  pinMode(A4,INPUT_PULLUP);
  //  pinMode(A5,INPUT_PULLUP);

  randomSeed((analogRead(0) + 1) * (analogRead(1) + 1) * (analogRead(2) + 1) * (analogRead(3) + 1) * (analogRead(4) + 1) * (analogRead(5) + 1)); // some may be 0/1023 but hopefully the rest creates some entropy by being a little random.

  Serial.begin(38400);
  Serial.println(F("Tix clock v0.03"));

  // read settings from eeprom - if there
  if (!readSettings()) {
    settings.mode24 = true; // clock mode 24h or 12h
    settings.intensity = INTLEVELS; // 5 levels of intensity, 0=off
    settings.updateInterval = 4; //  1,4,10,60 seconds
    settings.daylight = false; // daylight saving on/off
    strcpy(settings.DSTabbrev,"EDT");
    settings.DSTweek=Second;      //First, Second, Third, Fourth, or Last week of the month
    settings.DSTdow=Sun;          //day of week, 1=Sun, 2=Mon, ... 7=Sat
    settings.DSTmonth=Mar;        //1=Jan, 2=Feb, ... 12=Dec
    settings.DSThour=2;           //0-23
    settings.DSToffset=-240;      //offset from UTC in minutes
    strcpy(settings.STDabbrev,"EST");
    settings.STDweek=First;
    settings.STDdow=Sun;
    settings.STDmonth=Nov;
    settings.STDhour=2;
    settings.STDoffset=-300;
    saveSettings();
  }
  memcpy(&stored_settings,&settings,sizeof(settings));

//US Eastern Time Zone (New York, Detroit)
//TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
//TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);



  if (RTC.chipPresent()) {
    Serial.println(F("No rtc found"));
    RTC_present = false; // no clock
    setTime(12, 0, 0, 1, 1, 2018);
  } else {
    Serial.println(F("Found RTC, using it to set time"));
    RTC_present = true; // clock found
    setSyncProvider(RTC.get); // the function to get the time from the RTC
    setSyncInterval(30);

    Serial.print(F("In setup, time is: "));
    printTime();
    Serial.println();
  }

  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");
 
  // Setup timer interrupt to update the LED
  // to get 100hz all LEDs need to refresh every 10ms
  //   3 rows means 10/3=3.33
  // to get 150hz all LEDs need to refresh every 6.666ms
  //   3 rows means 6.666/3=2.22
  // to get 200hz all LEDs need to refresh every 5ms
  //   3 rows means 5/3=1.33
  // Now add to that the handling of brightness with PWM
  // After some experiments it sees like 0.5ms is good
  Timer1.initialize(500);
  Timer1.attachInterrupt(updateDisplay);

  //Show a "spash screen"
  DoDelay=1000;
  drawTime(3969); delay(DoDelay);
  while (digitalRead(BMode) == LOW || digitalRead(BInc)== LOW){};
  drawTime(3868); DoDelay-=200; delay(DoDelay);
  drawTime(3767); DoDelay-=175; delay(DoDelay);
  drawTime(3666); DoDelay-=150; delay(DoDelay);
  drawTime(3555); DoDelay-=125; delay(DoDelay);
  drawTime(3444); DoDelay-=100; delay(DoDelay);
  drawTime(3333); DoDelay-= 75; delay(DoDelay);
  drawTime(2222); DoDelay-= 50; delay(DoDelay);
  drawTime(1111); DoDelay-= 25; delay(DoDelay);
  Serial.print(F("Final delay="));Serial.println(DoDelay);
  displayOff(true);   delay(500);

  mode = time;
  //  showTix();
  //  delay(2000);
  drawNow();
  //  displayOff(true);
  
  //  curr_hour = hour();
  //  curr_minute = minute();

  //  drawTime(curr_hour * 100 + curr_minute);

// link the doubleclick function to be called on a doubleclick event.   
  buttonMode.attachClick(modeClick);
  buttonMode.attachDoubleClick(modeDoubleClick);
  buttonMode.attachLongPressStart(modePressStart);
  buttonMode.attachDuringLongPress(modeDuringPress);
  buttonMode.attachLongPressStop(modePressStop);
  buttonInc.attachClick(incClick);
  buttonInc.attachLongPressStart(incPressStart);
  buttonInc.attachDuringLongPress(incDuringPress);
  buttonInc.attachLongPressStop(incPressStop);  
} // setup

/****************************************************************/
// action when 
void modeClick(){
  time_t newTime;
  tmElements_t newTm;

  Serial.print(F("in mode Click - "));
  back2time=millis()+10000; // reset in 10 seconds
  //enum mode {time, setH, set10M, set1M, mode2, set24, setDL, setTZh, setTZm, setY, setM, setD,setInt} mode;
  if (mode==time){
    //flash left two, then show/set hour (setH)
    // Start 2 minute timer
    drawTime(3900);
    delay(500);
    back2time=millis()+120000; // reset in 2 minutes 
    mode=setH;
    Serial.println(F("Set hour"));
  }else if (mode==setH){
    //flash 10min then show/set 10 mins (set10M)
    drawTime(30);
    delay(500);
    back2time=millis()+120000; // reset in 2 minutes 
    mode=set10M;
    Serial.println(F("Set 10 Minute"));
  }else if (mode==set10M){
    //flash 1min then show/set 1 mins (set1M)
    back2time=millis()+120000; // reset in 2 minutes 
    drawTime(9);
    delay(500);
    mode=set1M;
    Serial.println(F("Set 1 minute"));
  }else if (mode==set1M){
    //TODO -> VERIFY
    //start clock at sec=0 then flash all LEDs
    newTime=now();
    breakTime(newTime, newTm);
    newTm.Second=0;
    newTime=makeTime(newTm);
    setTime(newTime);
    RTC.set(now());
    drawTime(3969);
    delay(500);
    mode=time;
    drawNow();
    Serial.println(F("back to show time"));
    Serial.print(F(", time is: "));
    printTime();
  }else if (mode==set24){
    //set daylight saving on/off
    mode=setDL;
    Serial.println(F("set DL"));
  }else if (mode==setDL){
    // set timezone hours
    mode=setTZh;
    Serial.println(F("set TZh"));
  }else if (mode==setTZh){
    // set timezone minutes
    mode=setTZm;
    Serial.println(F("set TZm"));
  }else if (mode==setTZm){
    //set clock year
    mode=setY;
    Serial.println(F("set Year"));
  }else if (mode==setY){
    //set clock month
    mode=setM;
    Serial.println(F("set Month"));
  }else if (mode==setM){
    //set clock day
    mode=setD;
    Serial.println(F("set Day"));
  }else if (mode==setD){
    //return to time
    mode=time;
    Serial.println(F("Date set, show time "));
  }else if (mode==setInt){
    //return to time
    mode=time;
    Serial.println(F("Interval set, show time "));
  }
} // modeClick

/****************************************************************/
void modeDoubleClick(){
  Serial.println(F("in mode DoubleClick"));
}

/****************************************************************/
void modePressStart(){
  Serial.println(F("in mode PressStart"));
  /*
  New stuff;
  long press "mode"
  *  when all squares light up advanced mode is in effect (mode2)
  release mode (set24)
  * leftmost top LED stays on, right two (minutes) shows 12 or 24 for 12/24h mode
  * inc to toggle
  click mode for daylight saving time (setDL)
  *  leftmost middle LED comes on, two middle fields shows DL, right is all on or all off
  *  "inc" to toggle daylight saving
  click mode for timezone hours (setTZh)
  *  leftmost bottom LED comes on, right lights show "TIZ" for 1sec, then hours like -4 or +5
  *  inc to change
  click mode for timezone minutes (setTZm)
  *  leftmost bottom LED stays on, right lights show "MIN" for 1sec, then minutes as 00/15/30/45 (no +/-)
  *  inc to change
  click mode for date setting - start with year, left shows a "Y" (setY)
   O  XOX (year 10) (year 1)  , this will work until 2069, starts over at 2018
   O  OXO
   O  OXO
    click mode month, left shows a "M" (which looks like H)  (setM)
   O  XOX (month10) (month1)
   O  XXX
   O  XOX
    click mode day
   O  XXO (day10) (day1), left shows a "D"  (setD)
   O  XOX
   O  XXO
  click mode again to exit
  no action in 10 seconds reset back to time
*/
}

/****************************************************************/
void modeDuringPress(){
  static long lastsecond;
  if (int(millis()/1000) != lastsecond){
    Serial.println(F("in mode DuringPress"));
    lastsecond=int(millis()/1000);
  }
}

/****************************************************************/
void modePressStop(){
  Serial.println(F("in mode PressStop"));
  mode=set24;
  back2time=millis()+10000; // reset in 10 seconds
  Serial.println(F("set 12/24h"));
}

void incClick(){
  tmElements_t newTm;
  time_t newTime;
  uint8_t monthLength;

  Serial.print(F("in inc Click - "));
  if (mode==time){
    if (settings.intensity==1)
      settings.intensity=INTLEVELS;
    else
      settings.intensity--;
    Serial.print(F("intensity level changed to "));
    Serial.print(settings.intensity);
    Serial.print(F(" or "));
    Serial.println(levels[settings.intensity]);
  }else{
    back2time=millis()+10000; // reset in 10 seconds
    if (mode==setH){
      back2time=millis()+120000; // reset in 2 minutes
      Serial.print(F("inc hours, "));
      Serial.print(hour());
      //      Serial.print(F(" "));Serial.print(now());Serial.print(F(" "));Serial.print(now()-(23UL*3600UL));Serial.print(F(" "));;Serial.print((23UL*3600UL),DEC);Serial.print(F(" "));
      Serial.print(F(" => "));
      if (hour() == 23){
        newTime=now()-(23UL*3600UL);
      }else{
        newTime=now()+3600UL;
      }
      setTime(newTime);
      RTC.set(now());
      //      Serial.print(newTime);Serial.print(F(" "));Serial.print(now());Serial.print(F(" "));
      Serial.println(hour());
    }else if (mode==set10M){
      back2time=millis()+120000; // reset in 2 minutes 
      Serial.print(F("inc 10 minutes"));
      Serial.print(minute());
      Serial.print(F(" => "));
      if ((minute()/10)%10 == 9){
        newTime=now()-(9*600);
      }else{
        newTime=now()+600;
      }
      setTime(newTime);
      RTC.set(now());
      Serial.println(minute());
    }else if (mode==set1M){
      Serial.print(F("inc 1 minute"));
      Serial.print(minute());
      Serial.print(F(" => "));
      if (minute()%10 == 9){
        newTime=now()-(9*60);
      }else{
        newTime=now()+60;
      }
      setTime(newTime);
      RTC.set(now());
      Serial.println(minute());
    }else if (mode==set24){
      //set 12/24h mode
      if (settings.mode24){
        settings.mode24=false;
        Serial.println(F(" 12h mode"));
      }else{
        settings.mode24=true;
        Serial.println(F(" 24h mode"));
      }
    }else if (mode==setDL){
      //set daylight saving on/off
      if (settings.daylight){
        settings.daylight=false;
        Serial.println(F(" daylight off"));
      }else{
        settings.daylight=true;
        Serial.println(F(" daylight on"));
      }
    }else if (mode==setTZh){
      //TODO
      // set timezone hours
      Serial.println(F(" set TZ hours"));
    }else if (mode==setTZm){
      //TODO
      // set timezone minutes
      Serial.println(F(" set TZ minutes"));
    }else if (mode==setY){
      //set clock year
      Serial.print(F(" set clock year, "));
      Serial.print(year());
      Serial.print(F(" => "));
      newTime=now();
      breakTime(newTime, newTm);
      if (newTm.Year <2018 || newTm.Year >2032){ // start over at 2032, when we get closer to that it can be increased up to 2069
	newTm.Year=2018;
      }else{
	newTm.Year++;
      }
      newTime=makeTime(newTm);
      setTime(newTime);
      RTC.set(now());
      Serial.println(year());
    }else if (mode==setM){
      //set clock month
      Serial.print(F(" set clock month, "));
      Serial.print(month());
      Serial.print(F(" => "));
      newTime=now();
      breakTime(newTime, newTm);
      if (newTm.Month <1 || newTm.Month >=12){
	newTm.Month=1;
      }else{
	newTm.Month++;
      }
      newTime=makeTime(newTm);
      setTime(newTime);
      RTC.set(now());
      Serial.println(month());
    }else if (mode==setD){
      //set clock day
      Serial.print(F(" set clock day, "));
      Serial.print(day());
      Serial.print(F(" => "));
      newTime=now();
      breakTime(newTime, newTm);
      if (newTm.Day <1 || newTm.Day >=31){
	newTm.Day=1;
      }else{
	newTm.Day++;
      }
      //TODO -> VERIFY
      if (newTm.Month==2) { // february
	if (LEAP_YEAR(newTm.Year)) {
	  monthLength=29;
	} else {
	  monthLength=28;
	}
      }else{
	monthLength=monthDays[newTm.Month-1];
      }
      if (newTm.Day>monthLength){
	newTm.Day=1;
      }

      newTime=makeTime(newTm);
      setTime(newTime);
      RTC.set(now());
      Serial.println(day());
    }else if (mode==setInt){
      // INC now cycles through the interval on the right, showing seconds for updates, 1,4,10,60 seconds (shows 59)
      Serial.print(F(" update interval set to "));
      if (settings.updateInterval==1){
        settings.updateInterval=4;
      }else if (settings.updateInterval==4){
        settings.updateInterval=10;
      }else if (settings.updateInterval==10){
        settings.updateInterval=60;
      }else {
        settings.updateInterval=1;
      }
      Serial.println(settings.updateInterval);
    }
    // no action in 10 seconds reset back to time
  } // swith (mode) {
} // incClick

void incPressStart(){
  Serial.println(F("in inc PressStart"));
  mode=mode3;
  drawTime(3000);
  // TODO
  // MOD2, hold "inc" (not mode as the original is) for 2 sec to set interval
  // *  left bar shows all 3 leds on (setInt)
  // INC now cycles through the interval on the right, showing seconds for updates, 1,4,10,60 seconds (shows 59)
  // no action in 10 seconds reset back to time
}

void incDuringPress(){
  static long lastsecond;
  if (int(millis()/1000) != lastsecond){
    Serial.println(F("in inc DuringPress"));
    lastsecond=int(millis()/1000);
  }
}

void incPressStop(){
  Serial.println(F("in inc PressStop"));
  // MOD2, hold "inc" (not mode as the original is) for 2 sec to set interval
  // *  left bar shows all 3 leds on (setInt)
  back2time=millis()+10000; // reset in 10 seconds
  mode=setInt;
}


void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long lastSecond=0,currSecond=0;
  time_t newTime;
  tmElements_t newTm;

  // keep watching the push button:
  buttonMode.tick();
  buttonInc.tick();
  
  if (mode != time){
    if (back2time<millis()){
      mode=time;
      Serial.println(F("timeout - back to show time"));
      VERBOSE=true;
      drawNow();
    }else{
      VERBOSE=false;
    }
  }

  currSecond=millis()/1000;
  
  if (mode == time) {
    //Time changed?
    if (currSecond != lastSecond) {
      if ((millis() - lastUpdate) > settings.updateInterval * 1000) {
        lastSecond=currSecond;
	if (mode == set1M) {
	  //if we set the time - Keep the seconds at 0
	  newTime=now();
	  breakTime(newTime, newTm);
	  newTm.Second=0;
	  newTime=makeTime(newTm);
	  setTime(newTime);
	  RTC.set(now());
	}
        /*
          Serial.print("DEBUG1:");
          Serial.print(curr_hour);
          Serial.print(":");
          Serial.print(curr_minute);
          Serial.print(":");
          Serial.println(curr_hour*100+curr_minute);
          printTime();
          Serial.println();
        */
        lastUpdate = millis();
        drawNow();
        Serial.print(F("Intensity: "));
        Serial.print(settings.intensity);
        Serial.print(F(" or "));
        Serial.print(levels[settings.intensity]);
        Serial.print(F(" cycles out of "));
        Serial.print(CYCLES);
        Serial.print(F(", time is: "));
        printTime();
        Serial.println();
        if (memcmp(&settings, &stored_settings, sizeof(settings)) != 0){
          Serial.println(F("stored settings differes from current, saving them to eeprom"));
          saveSettings();
          memcpy(&stored_settings,&settings,sizeof(settings));
        }
      } // if time to update
    } // if new second
  } else if (mode == setH) {
    //show only hour
    //    drawTime(curr_hour * 100,false);
    drawNow(false);
  } else if (mode == set10M) {
    //    drawTime(curr_minute,false);
    drawNow(false);
  } else if (mode == set1M) {
    //slowly blink 1m
    //    drawTime(curr_minute,false);
    drawNow(false);
  } else if (mode == mode2) {
    //long press "mode", going to MOD2, all on
    drawTime(3969,false);
  } else if (mode == mode3) {
    //long press "inc", going to INTerval, left 3 on
    drawTime(3000,false);
  } else if (mode == set24) {
    //leftmost top,  12/24
    if (settings.mode24){
      drawTime(1024,false);
    }else{
      drawTime(1012,false);
    }
  } else if (mode == setDL) {
    //leftmost middle LED on, two middle fields shows DL, right is all on or all off
    displayOff(false);
    LEDAssembly[0][0] = false;
    LEDAssembly[1][0] = true;
    LEDAssembly[2][0] = false;
    // D
    LEDAssembly[0][1] = true;
    LEDAssembly[1][1] = true;
    LEDAssembly[2][1] = true;
    LEDAssembly[0][2] = true;
    LEDAssembly[1][2] = false;
    LEDAssembly[2][2] = true;
    LEDAssembly[0][3] = false;
    LEDAssembly[1][3] = true;
    LEDAssembly[2][3] = false;
    // L
    LEDAssembly[0][4] = true;
    LEDAssembly[1][4] = true;
    LEDAssembly[2][4] = true;
    LEDAssembly[0][5] = false;
    LEDAssembly[1][5] = false;
    LEDAssembly[2][5] = true;
        
    if (settings.daylight){
      drawFrame(3,9);  
    }else{
      drawFrame(3,0);
    }
    updateDisplayValues();
  } else if (mode == setTZh) {
    //TODO
    //leftmost bottom LED comes on, right lights show "TIZ" for 1sec, then hours like -4 or +5
    displayOff(false);
    LEDAssembly[0][0] = false;
    LEDAssembly[1][0] = false;
    LEDAssembly[2][0] = true;
    updateDisplayValues();
  } else if (mode == setTZm) {
    //TODO
    //leftmost bottom LED stays on, right lights show "MIN" for 1sec, then minutes as 00/15/30/45 (no +/-)
    LEDAssembly[1][0] = true;
  } else if (mode == setY) {
    //TODO -> VERIFY
    //left shows a "Y" (setY)
    // O  XOX (year 10) (year 1)  , this will work until 2069, starts over at 2018
    // O  OXO
    // O  OXO
    displayOff(false);
    LEDAssembly[0][0] = false;
    LEDAssembly[1][0] = false;
    LEDAssembly[2][0] = false;

    LEDAssembly[0][1] = true;
    LEDAssembly[1][1] = false;
    LEDAssembly[2][1] = true;
    LEDAssembly[0][2] = false;
    LEDAssembly[1][2] = true;
    LEDAssembly[2][2] = false;
    LEDAssembly[0][3] = false;
    LEDAssembly[1][3] = true;
    LEDAssembly[2][3] = false;

    drawFrame(2,(year()%100)/10,false,false);
    drawFrame(3,year()%10,false);
  } else if (mode == setM) {
    //TODO -> VERIFY
    //left shows a "M" (which looks like H)  (setM)
    // O  XOX (month10) (month1)
    // O  XXX
    // O  XOX
    displayOff(false);
    LEDAssembly[0][0] = false;
    LEDAssembly[1][0] = false;
    LEDAssembly[2][0] = false;

    LEDAssembly[0][1] = true;
    LEDAssembly[1][1] = false;
    LEDAssembly[2][1] = true;
    LEDAssembly[0][2] = true;
    LEDAssembly[1][2] = true;
    LEDAssembly[2][2] = true;
    LEDAssembly[0][3] = true;
    LEDAssembly[1][3] = false;
    LEDAssembly[2][3] = true;

    drawFrame(2,(month()%100)/10,false,false);
    drawFrame(3,month()%10,false);
  } else if (mode == setD) {
    //TODO -> VERIFY
    //left shows a "D"  (setD)
    // O  XXO (day10) (day1)
    // O  XOX
    // O  XXO
    displayOff(false);
    LEDAssembly[0][0] = false;
    LEDAssembly[1][0] = false;
    LEDAssembly[2][0] = false;

    LEDAssembly[0][1] = true;
    LEDAssembly[1][1] = true;
    LEDAssembly[2][1] = false;
    LEDAssembly[0][2] = true;
    LEDAssembly[1][2] = false;
    LEDAssembly[2][2] = true;
    LEDAssembly[0][3] = true;
    LEDAssembly[1][3] = true;
    LEDAssembly[2][3] = false;

    drawFrame(2,(day()%100)/10,false,false);
    drawFrame(3,day()%10,false);
  } else if (mode == setInt) {
    //left bar shows all 3 leds on
    //show the interval on the right, showing seconds for updates, 1/4/10/60
    drawTime(3000+settings.updateInterval,false);
  }
}
