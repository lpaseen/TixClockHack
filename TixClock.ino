/*
   Tix(tm) clock hack, replaced the PIC16F628 with a small add on board that contains an arduino nano and RTC.

   Copyright Peter Sjoberg
   V0.00


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
   to have 6 levels (5 plus off) it need to be 10 periodes and the levels are
   0,1,2,4,6,10

*/

#define CYCLES 10
#define INTLEVELS 5
const uint8_t levels[] = {0, 1, 2, 4, 6, 10};

//Display status
volatile boolean LEDBuffer[ROWS][COLUMNS];
volatile boolean LEDAssembly[ROWS][COLUMNS];
volatile uint8_t currentRow = 0;

static uint8_t myHour, myMinute, mySecond;
static uint8_t myYear, myMonth, myDay;
static uint8_t cycle = 0;
static uint8_t intens=INTLEVELS;

#include <EEPROM.h>
typedef struct {
  boolean mode24; // clock mode 24h or 12h
  uint8_t intensity; // 5 levels of intensity, 0=off
  uint8_t updateInterval; // 1/5/30/60 seconds
  unsigned int checksum;
} Settings_t;

volatile Settings_t settings;

#include <TimeLib.h>
#include <Timezone.h> //https://github.com/JChristensen/Timezone

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

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
  press 1: hour (setH)
  press 2: 10 mins (set10M)
  press 3: 1 mins (set1M)
  press 4: flash all LEDs, then start clock at sec=0

  MOD 2 sec for interval
  left bar shows all 3 leds on (setInt)
  INC now cycles through the interval on the right, showing seconds for updates, 1/5/30/60 seconds

  ================
  maybe change the above two ones
  long press "mode"
    when all squares light up advanced mode is in effect
  release mode
   leftmost top LED stays on, right two (minutes) shows 12 or 24 for 12/24h mode
   inc to toggle
  click mode for daylight saving time
    leftmost middle LED comes on, two middle fields shows DL, right is all on or all off
    "inc" to toggle daylight saving
  click mode for timezone hours
    leftmost bottom LED comes on, right lights show "TIZ" for 1sec, then hours like -4 or +5
    inc to change
  click mode for timezone minutes
    leftmost bottom LED stays on, right lights show "MIN" for 1sec, then minutes as 00/15/30/45 (no +/-)
    inc to change
  click mode again to exit


  X XXX XX XXX
  X XXX XX XXX
  X XXX XX XXX


*/

enum mode {time, setH, set10M, set1M, setInt, set24, TZ} mode;

//Arduino pins
#define BMenu 3
#define BInc  4
#define AC    2

const uint8_t rowPins[ROWS] = {9, 10, 11};
const uint8_t colPins[COLUMNS] = {5, 6, 7, 8, A0, A1, A2, 12, 13};

#include "OneButton.h"

// connect the buttons
OneButton buttonMenu(BMenu,true);
OneButton buttonInc(BInc,true);


//RTC
#define DS3231_ADDR 0x68
boolean RTC_present = false;
#include <Wire.h>
#include <DS1307RTC.h>  // a basic DS1307/DS3231

/****************************************************************/
void printTime() {
  uint8_t currhour, currminute, currsecond;
  uint8_t curryear, currmonth, currday;
  time_t t = now();

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
  }

  cycle++;
  if (cycle == INTLEVELS)
    cycle = 0;

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
    if (LEDBuffer[currentRow][c] && (settings.intensity - cycle) > 0) {
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
  // should use memcpy
  for (uint8_t r = 0; r < ROWS; r++) {
    for (uint8_t c = 0; c < COLUMNS; c++) {
      LEDBuffer[r][c] = LEDAssembly[r][c];
    }
  }
  Timer1.resume();
  //  interrupts();

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
  displayOff();
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
void setFrame(uint8_t minCol, uint8_t maxCol, uint8_t LEDs) {
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

  if (LEDs == maxLed)
    newVal = true;
  else
    newVal = false;

  for (row = 0; row < 3; row++) {
    for (col = minCol; col < maxCol; col++) {
      LEDAssembly[row][col] = newVal;
    }
  }
  if (LEDs == maxLed)
    return;

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
    if (!LEDAssembly[row][col]) {
      LEDAssembly[row][col] = true;
      ledcnt++;
    }
  }
} // setFrame

/****************************************************************/
void drawFrame(uint8_t frame, uint8_t digit) {
  uint8_t ledcnt, row, col;

  if (digit > 9)
    return;
  if (frame == 0) { // 10 hour
    if (digit > 3)
      return;
    setFrame(0, 0, digit);
  } else if (frame == 1) { // 1 hour
    setFrame(1, 3, digit);
  } else if (frame == 2) { // 10 minute
    if (digit > 6)
      return;
    setFrame(4, 5, digit);
  } else if (frame == 3) { // 1 minute
    setFrame(6, 8, digit);
  }

  updateDisplayValues();
} // drawFrame
/****************************************************************/
void drawTime(uint16_t number) {
  uint8_t h10, h1, m10, m1;

  h10 = int(number / 1000);
  h1 = int((number % 1000) / 100);
  m10 = int((number % 100) / 10);
  m1 = int(number % 10);

  Serial.print(number, DEC);
  Serial.print("=");
  Serial.print(h10, DEC);
  Serial.print(":");
  Serial.print(h1, DEC);
  Serial.print(":");
  Serial.print(m10, DEC);
  Serial.print(":");
  Serial.print(m1, DEC);
  Serial.println();

  drawFrame(0, h10);
  drawFrame(1, h1);
  drawFrame(2, m10);
  drawFrame(3, m1);


} // drawTime

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
  uint16_t curr_hour, curr_minute;

  pinMode(BMenu, INPUT);
  pinMode(BInc, INPUT);
  for (uint8_t i = 0; i < ROWS; i++) {
    pinMode(rowPins[i], OUTPUT); digitalWrite(rowPins[i], HIGH);
  }
  for (uint8_t i = 0; i < COLUMNS; i++) {
    pinMode(colPins[i], OUTPUT); digitalWrite(colPins[i], LOW);
  }

  randomSeed((analogRead(0) + 1) * (analogRead(1) + 1) * (analogRead(2) + 1) * (analogRead(3) + 1) * (analogRead(4) + 1) * (analogRead(5) + 1)); // some may be 0/1023 but hopefully the rest creates some entropy by being a little random.

  Serial.begin(38400);
  Serial.println(F("Tix clock v0.02"));

  // read settings from eeprom - if there
  if (!readSettings()) {
    settings.mode24 = true; // clock mode 24h or 12h
    settings.intensity = INTLEVELS; // 5 levels of intensity, 0=off
    settings.updateInterval = 5; // 1/5/30/60 seconds
    saveSettings();
  }

  //  pinMode(A4,INPUT_PULLUP);
  //  pinMode(A5,INPUT_PULLUP);

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

  drawTime(3969); delay(500); //while (1){};
  drawTime(3868); delay(400);
  drawTime(3767); delay(300);
  drawTime(3666); delay(200);
  drawTime(3555); delay(150);
  drawTime(3444); delay(100);
  drawTime(3333); delay(80);
  drawTime(2222); delay(60);
  drawTime(1111); delay(40);
  displayOff();  delay(100);
  //  drawTime(2359);delay(5000);
  //  drawTime(1959);delay(3000);
  //  drawTime(539);delay(3000);

  if (digitalRead(BMenu) == LOW) {
    mode = set24;
  } else {
    mode = time;
    showTix();
    delay(4000);
  }
  displayOff();
  
  curr_hour = hour();
  curr_minute = minute();

  drawTime(curr_hour * 100 + curr_minute);

// link the doubleclick function to be called on a doubleclick event.   
  buttonMenu.attachClick(menuClick);
  buttonMenu.attachDoubleClick(menuDoubleClick);
  buttonMenu.attachLongPressStart(menuPressStart);
  buttonMenu.attachDuringLongPress(menuDuringPress);
  buttonMenu.attachLongPressStop(menuPressStop);
  buttonInc.attachClick(incClick);
  
} // setup

void menuClick(){
  Serial.println(F("in menu Click"));
}

void menuDoubleClick(){
  Serial.println(F("in menu DoubleClick"));
}

void menuPressStart(){
  Serial.println(F("in menu PressStart"));
}

void menuDuringPress(){
  static long lastsecond;
  if (int(millis()/1000) != lastsecond){
    Serial.println(F("in menu DuringPress"));
    lastsecond=int(millis()/1000);
  }
}

void menuPressStop(){
  Serial.println(F("in menu PressStop"));
}

void incClick(){
  Serial.println(F("in inc Click"));
}


void loop() {
  static uint8_t last_hour = -1, last_minute = -1, last_second = -1;
  static uint8_t curr_hour = -1, curr_minute = -1, curr_second = -1;
  static unsigned long lastUpdate = 0;
  time_t utc_time,local_time;

  // keep watching the push button:
  buttonMenu.tick();
  buttonInc.tick();
  
  utc_time = now();

  local_time = myTZ.toLocal(utc_time, &tcr);
  curr_hour = hour(local_time);
  if (!settings.mode24)
    if (curr_hour>12)
      curr_hour-=12;
  curr_minute = minute(local_time);
  curr_second = second(local_time);

  if (mode == time) {
    //Time changed?
    if (curr_second != last_second) {
      if ((millis() - lastUpdate) > settings.updateInterval * 1000) {
        last_hour = curr_hour;
        last_minute = curr_minute;
        last_second = curr_second;
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
        /*
        intens++;
        if (intens>INTLEVELS)
          intens=1;
        */
        settings.intensity=levels[intens];
        lastUpdate = millis();
        drawTime(curr_hour * 100 + curr_minute);
        Serial.print(F("Intensity: "));
        Serial.print(intens);
        Serial.print(F("/"));
        Serial.print(settings.intensity);
        Serial.print(F(", time is: "));
        printTime();
        Serial.println();
      } // if timye to update
    } // if new second
  } else if (mode = setH) {
  } else if (mode = set10M) {
  } else if (mode = set1M) {
  } else if (mode = setInt) {
    /*
      if (BInt released){
      intens++;
      if (intens>INTLEVELS)
        intens=0;
      settings.intensity=levels[intens];
      }
    */
  } else if (mode = set24) {
  }
}
