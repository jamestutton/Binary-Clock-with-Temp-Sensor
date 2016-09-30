#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include <RtcDS1307.h>
#include <RtcDS3231.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define eBRIGHTNESS  0


#define MODE_SHOW_TIME 1
#define MODE_SHOW_TEMP 2
#define MODE_SHOW_DATE 3
#define MODE_SHOW_YEAR 4
#define MODE_SET_SEC 5
#define MODE_SET_MIN 6
#define MODE_SET_HOUR 7
#define MODE_SET_DAY 8
#define MODE_SET_MONTH 9
#define MODE_SET_YEAR 10
#define MODE_SET_BRIGHTNESS 11
#define MODE_LED_TEST 12
#define LAST_MODE 12

#define MODEBUTTON 7
#define SETBUTTON 6


#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

const uint8_t mRun = 0; //Index for second RTC setting
const uint8_t mSecond= 1; //Index for second RTC setting
const uint8_t mMinute = 2; //Index for minute RTC setting
const uint8_t mHour = 3; //Index for hour RTC setting
const uint8_t mDimming = 4; //Index for hour RTC setting

byte mode = MODE_SHOW_TIME; //Mode for time and date setting
//
const int fSecPin = 0;
const int lSecPin = 5;

const int fMinPin = 6;
const int lMinPin = 11;

const int fHourPin = 12;
const int lHourPin = 15;

const int fYearPin = 0;
const int lYearPin = 11;

const int DimSteps = 16;
int Dim = 8;



int pulse;

unsigned long lastflash = 0;

unsigned long btn_last = 0;
unsigned long btn_timeout = 30000; //30 Seconds

const int flashtime = 500;
boolean flashstate = true;

RtcDateTime lTime;



//RtcDS1307 Rtc;
RtcDS3231 Rtc;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);


void AddMinute (int8_t);
void AddHour(int8_t);
void AddDay(int8_t);
void AddMonth(int8_t);
void AddYear(int8_t);

void setup() {
  // put your setup code here, to run once:
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

sensors.begin();
sensors.setResolution(TEMP_9_BIT);

  //Button Setup
  pinMode(MODEBUTTON, INPUT);
  pinMode(SETBUTTON, INPUT);
  
  Serial.begin(115200);
  Serial.print("compiled: ");
    Serial.print(__DATE__);
    Serial.println(__TIME__);

    //--------RTC SETUP ------------
    Rtc.Begin();


   RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);


   if ((digitalRead(SETBUTTON)==HIGH) && (digitalRead(MODEBUTTON)==HIGH)) {
      Rtc.SetDateTime(compiled);
      Dim = 8;
   }
     
    if (!Rtc.IsDateTimeValid()) 
    {
        // Common Cuases:
        //    1) first time you ran and the device wasn't running yet
        //    2) the battery on the device is low or even missing

        Serial.println("RTC lost confidence in the DateTime!");

        // following line sets the RTC to the date & time this sketch was compiled
        // it will also reset the valid flag internally unless the Rtc device is
        // having an issue

       
    }

    if (!Rtc.GetIsRunning())
    {
        Serial.println("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    RtcDateTime now = Rtc.GetDateTime();
    if (now < compiled) 
    {
        Serial.println("RTC is older than compile time!  (Updating DateTime)");
        Rtc.SetDateTime(compiled);
    }
    else if (now > compiled) 
    {
        Serial.println("RTC is newer than compile time. (this is expected)");
    }
    else if (now == compiled) 
    {
        Serial.println("RTC is the same as compile time! (not expected but all is fine)");
    }

    // never assume the Rtc was last configured by you, so
    // just clear them to your needed state
    //Rtc.SetSquareWavePin(DS1307SquareWaveOut_Low);

         Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 
    LEDTest();
    Dim = EEPROM.read(eBRIGHTNESS);
    
}

void showclock() {

  RtcDateTime cTime = Rtc.GetDateTime();
  if (cTime != lTime or mode > MODE_SHOW_TIME) {
          lTime = cTime;
          printDateTime(cTime);
          
          if (mode==MODE_SET_HOUR && flashstate) {
            showHour(0);

          }else{
            showHour(cTime.Hour());

          }
          
          if (mode==MODE_SET_MIN && flashstate) {
            showMin(0);

          }else{
            showMin(cTime.Minute());

          }
          
          if (mode==MODE_SET_SEC && flashstate) {
            showSec(0);
          }else{
            showSec(cTime.Second());
          }
          
          
  }
}

void showmode() {
  //Show the Mode
  if (flashstate) {
    showHour(0);
  }else {
    showHour(mode);
  }
  
}
void showdate() {

  showmode();
  RtcDateTime cTime = Rtc.GetDateTime();
    if (mode==MODE_SET_MONTH && flashstate) {
      showMin(0);
    }else{
      showMin(cTime.Month());
    }
    
    if (mode==MODE_SET_DAY && flashstate) {
      showSec(0); 
    }else{
      showSec(cTime.Day());
    }
  
}

void showyear() {
  showmode();
  RtcDateTime cTime = Rtc.GetDateTime();
  //Show the Year
  if (mode==MODE_SET_YEAR && flashstate) {
    showYear(0);
  }else{
    showYear(cTime.Year());
  }
}

void showtemp() {
  sensors.requestTemperatures(); // Send the command to get temperatures
  float tempC = sensors.getTempCByIndex(0);

 Serial.print("Temp=");
 Serial.println(tempC);
 
  boolean negativeTemp = false;
  
  if (tempC<0) {
    tempC = -tempC;
    negativeTemp = true;
  }
  
  unsigned int WholeDegC= tempC; //Get the Whole Degress C
  unsigned int DecDegC = (tempC - WholeDegC) * 10;
  

  
  showmode();
  showMin(WholeDegC);
  showSec(DecDegC);

  //Use PWN Pin 6 To Show Deg Degrees
  if (negativeTemp && flashstate) {
    pwm.setPWM(5, 0, Brightness() );
  }else{
    pwm.setPWM(5, 0, 0 );
  }
}

void loop() {
  if ((millis() - lastflash) > flashtime){
    lastflash = millis();
    flashstate = !flashstate;
  }
  if ((millis() - btn_last) > btn_timeout){
    mode=1;
  }


 
               
  if (!Rtc.IsDateTimeValid()) 
    {
        // Common Cuases:
        //    1) the battery on the device is low or even missing and the power line was disconnected
        Serial.println("RTC lost confidence in the DateTime!");
    }
    switch(mode){
      case MODE_SHOW_TIME:
      case MODE_SET_HOUR:
      case MODE_SET_MIN:
      case MODE_SET_SEC:
        showclock();
        break;
      case MODE_SHOW_DATE:
      case MODE_SET_MONTH:
      case MODE_SET_DAY:
        showdate();
        break;
      case MODE_SHOW_YEAR:
      case MODE_SET_YEAR:
        showyear();
        break;
      case MODE_SHOW_TEMP:
        btn_last =millis();  //Dont Timeout of Temp Mode
        showtemp();
        break;
      case MODE_LED_TEST:
        LEDTest();
        break;
      
      case MODE_SET_BRIGHTNESS:
        showMin(63);
          showSec(63);
          showmode();
        break;
      default:
        {
          showMin(0);
          showSec(0);
          showmode();
          break;
        }
    }
    
   if(digitalRead(MODEBUTTON)==HIGH){ //Read setting mode button
    delay(25); //25ms debounce time
    if(digitalRead(MODEBUTTON)==HIGH){ //Activate setting mode change after 100ms button press
      mode = mode + 1; //Increment the time setting mode on each button press
      btn_last =millis();
      
      if(mode > LAST_MODE) {
        mode = 1;
      }
      Serial.print("Mode:");
      Serial.println(mode);
      while (digitalRead(MODEBUTTON)==HIGH) {
        delay(200);
      }
      
    }
 }

 
if(digitalRead(SETBUTTON)==HIGH){ //Read setting mode button
    delay(25); //25ms debounce time
    if(digitalRead(SETBUTTON)==HIGH){ //Activate setting mode change after 100ms button press
      btn_last =millis();
      if(mode == MODE_SHOW_TIME){
        
      }
      else if(mode == MODE_SHOW_DATE){
        
      }
      else if(mode == MODE_SHOW_TEMP){
        
      }
      else if(mode == MODE_SET_SEC){
        ResetSeconds();
      }
      else if(mode == MODE_SET_MIN){
        AddMinute(1);
      }
      else if(mode == MODE_SET_HOUR){
        AddHour(1);
      }
      else if(mode == MODE_SET_DAY){
          AddDay(1);        
      }
      else if(mode == MODE_SET_MONTH){
          AddMonth(1);
      }
      else if(mode == MODE_SET_YEAR){
          AddYear(1);
      }
      else if(mode == MODE_SET_BRIGHTNESS){
        AdjustBrightness();
      }
      
      
        delay(500);
      
    }
 }

      
}

void ResetSeconds () {
      RtcDateTime cTime = Rtc.GetDateTime();
      RtcDateTime newClock= RtcDateTime(cTime.Year(),cTime.Month(),cTime.Day(),cTime.Hour(),cTime.Minute(),0);
      Rtc.SetDateTime(newClock);
      showSec(63);
      
}

void AddMinute (int8_t Step = 1) {
      RtcDateTime cTime = Rtc.GetDateTime();
      RtcDateTime newClock= RtcDateTime(cTime.Year(),cTime.Month(),cTime.Day(),cTime.Hour(),cTime.Minute()+Step,cTime.Second());
      Rtc.SetDateTime(newClock);
      
}

void AddHour (int8_t Step = 1) {
      RtcDateTime cTime = Rtc.GetDateTime();
      RtcDateTime newClock= RtcDateTime(cTime.Year(),cTime.Month(),cTime.Day(),cTime.Hour()+Step,cTime.Minute(),cTime.Second());
      Rtc.SetDateTime(newClock);
      
}

void AddMonth (int8_t Step = 1) {
      RtcDateTime cTime = Rtc.GetDateTime();
      RtcDateTime newClock= RtcDateTime(cTime.Year(),cTime.Month()+Step,cTime.Day(),cTime.Hour(),cTime.Minute(),cTime.Second());
      Rtc.SetDateTime(newClock);
      
}

void AddDay (int8_t Step = 1) {
      RtcDateTime cTime = Rtc.GetDateTime();
      RtcDateTime newClock= RtcDateTime(cTime.Year(),cTime.Month(),cTime.Day()+Step,cTime.Hour(),cTime.Minute(),cTime.Second());
      Rtc.SetDateTime(newClock);
      
}

void AddYear (int8_t Step = 1) {
      RtcDateTime cTime = Rtc.GetDateTime();
      RtcDateTime newClock= RtcDateTime(cTime.Year()+Step,cTime.Month(),cTime.Day(),cTime.Hour(),cTime.Minute(),cTime.Second());
      Rtc.SetDateTime(newClock);
      
}



void showYear(int cYear) {
  Serial.print("Year:");
  Serial.print(cYear);
  Serial.print(":");
  for (int pwnnum = lYearPin; pwnnum >= fYearPin; pwnnum--) {
    int Digit = (1<<pwnnum-fYearPin);
    if ((Digit &  cYear) == Digit) {
      pulse = Brightness();
      Serial.print(1);
    }else{
      pulse = 0;
      Serial.print(0);
    }

    pwm.setPWM(pwnnum, 0, pulse );
  }
  Serial.println();
 }

void showSec(int cSec) {

  
  
  
  //Serial.print(" s");
  for (int pwnnum = lSecPin; pwnnum >= fSecPin; pwnnum--) {
    
    uint8_t shiftBit = pwnnum-fSecPin;
    uint8_t Digit = (1<<shiftBit);
    if ((Digit &  cSec) == Digit) {
      pulse = Brightness();
      //Serial.print(1);
    }else{
      pulse = 0;
     // Serial.print(0);
    }
    pwm.setPWM(pwnnum, 0, pulse );
  }
  
  
 }


void showMin(int cMinute) {

  //Serial.print(" m");

  for (int pwnnum = lMinPin; pwnnum >= fMinPin; pwnnum--) {
    uint8_t Digit = (1<<pwnnum-fMinPin);
    if ((Digit &  cMinute) == Digit) {
      pulse = Brightness();
      //Serial.print(1);
    }else{
      pulse = 0;
      //Serial.print(0);
    }

    pwm.setPWM(pwnnum, 0, pulse );
  }
 
}

void showHour(int cHour) {
  //Serial.print(" h");
  if (cHour > 12)  cHour = cHour - 12;
  
  for (int  pwnnum = lHourPin; pwnnum >= fHourPin; pwnnum-- ) {
    uint8_t Digit = (1<<pwnnum-fHourPin);
    if ((Digit &  cHour) == Digit) {
     pulse = Brightness();
      //Serial.print(1);
    }else{
      pulse = 0;
      //Serial.print(0);
    }

    pwm.setPWM(pwnnum, 0, pulse );
  }

  //Serial.println();
  //delay(1000);
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.println(datestring);
}

void LEDTest() {
  for (int  pwnnum = 15; pwnnum >= 0; pwnnum-- ) {
    pwm.setPWM(pwnnum, 0, 1024 );
    delay(300);
  }
  delay(1000);
  for (int  pwnnum = 15; pwnnum >= 0; pwnnum-- ) {
    pwm.setPWM(pwnnum, 0, 0 );
    delay(300);
  }
  delay(1000);
  
}

void AdjustBrightness() {
    Dim++;
    if (Dim >= 16) Dim = 0;
    EEPROM.write(eBRIGHTNESS, Dim);
    showHour(Dim);
    showMin(63);
    showSec(63);
    delay(500);
}

int Brightness() {
  int iBright =  Dim*(4096/DimSteps);
  if (iBright <= 0) iBright = 1;
  if (iBright > 4095) iBright = 4095;
  return iBright;
  
}


double GetInternalTemp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (t);
}

