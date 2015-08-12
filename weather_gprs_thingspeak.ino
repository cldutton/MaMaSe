// tph connected on I2C and a rainfall sensor connected on A1
// Portions of this code were developed by the Sodaq team

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <Sodaq_DS3231.h>
#include <RTCTimer.h>
#include <SPI.h>
#include <SD.h>
#include <GPRSbee.h>
#include <Sodaq_PcInt.h>
#include <Sodaq_TPH.h>

//Network constants
#define APN "safaricom"
#define APN_USERNAME "saf"
#define APN_PASSWORD "data"

//Seperators
#define FIRST_SEP "?"
#define OTHER_SEP "&"
#define LABEL_DATA_SEP "="

//battery constants
#define ADC_AREF 3.3
#define BATVOLTPIN A6
#define BATVOLT_R1 4.7
#define BATVOLT_R2 10

//Data labels, cannot change for ThingSpeak
#define LABEL1 "field1"
#define LABEL2 "field2"
#define LABEL3 "field3"
#define LABEL4 "field4"
#define LABEL5 "field5"

//SpeakThings constants
#define URL "api.thingspeak.com/update"
#define WRITE_API_KEY "XXXXXXXXXXXXXXX" //Change to your channel's key

volatile float rainValue = 0; // inches

RTCTimer timer;           // Instantiate the timer

bool hz_flag;

// soil moisture probes
int calculatedvalue;
int calculatedvaluetwo;
  
const int chipSelect = 11;

const uint8_t dataPin  =  5;
const uint8_t clockPin =  4;

File dataFile;

void setup()
{
  wdt_disable();
  
  Serial.begin(57600);
  Serial1.begin(57600);
  Wire.begin();
  rtc.begin();
  tph.begin();
  
  pinMode(4,INPUT);
  digitalWrite(4,HIGH);

  pinMode(GROVEPWR, OUTPUT);

  pinMode(A2, INPUT); // soil 1
  pinMode(A3, INPUT); // soil 2
  
  //Intialise the GPRSbee
  gprsbee.init(Serial1, BEECTS, BEEDTR);

  //uncomment this line to debug the GPRSbee with the serial monitor
  //gprsbee.setDiag(Serial);
  
  //This is required for the Switched Power method
  gprsbee.setPowerSwitchedOnOff(true); 

  // Make sure the GPRSbee is switched off
  doCheckGPRSoff(0);

  // Do the work every number of seconds
  timer.every(1800 , takedata);

  // Instruct the RTCTimer how to get the current timestamp
  timer.setNowCallback(getNow);
  
  Serial.print("Initializing SD card...");
  pinMode(11, OUTPUT);
  
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");
  
   pinMode(A1, INPUT_PULLUP);
   PcInt::attachInterrupt(A1, rainInterrupt);
  
  setupWatchdog();

  interrupts();
  
  rainValue = 0;
  
  Serial.println("Setup complete");
  
  wdt_reset();
  gprsbee.on();    

  FiveSecondDelay();
  FiveSecondDelay();
  FiveSecondDelay(); 

  Serial1.print(F("AT+CMGF=1\r"));                     //line for sms mode
  delay(100);
  Serial1.println(F("AT + CMGS = \"+XXXXXXXXXXXXX\""));          //my number for testing
  delay(100);
  Serial1.print(F("Sodaq Mbili Soil Moisture Station has reinitialized"));
  Serial.println(F("Sodaq Mbili Soil Moisture Station has reinitialized"));
  Serial1.println((char)26);                        //the ASCII code of the ctrl+z is 26, which closes the sms
  wdt_reset();
  FiveSecondDelay();
  wdt_reset();  
  Serial1.println();
                          
  wdt_reset();
  gprsbee.off();  
}

void loop()
{
  if (hz_flag) {
    hz_flag = false;
    wdt_reset();
    WDTCSR |= _BV(WDIE);
  }
  
  timer.update();

  systemSleep();
}

void takedata(uint32_t ts)
{
  wdt_reset();
  
  FiveSecondDelay();
    
  DateTime now = rtc.now(); //get the current date-time  
   
  float temp = tph.readTemperature();

  float bmp_temp = tph.readTemperatureBMP();

  float sht_temp = tph.readTemperatureSHT();

  float hum = tph.readHumidity();

  int32_t pres = tph.readPressure();
  
  int mv = getRealBatteryVoltage() * 1000.0;

  wdt_reset();

  String url;
  
  wdt_reset();
  gprsbee.off();
  wdt_reset();
    
  delay(50);
    
  gprsbee.on(); 
  
  url += String("api.thingspeak.com/update");
  url += String(FIRST_SEP) + String("key");
  url += String(LABEL_DATA_SEP) + String(WRITE_API_KEY);
   
  url += String(OTHER_SEP) + String(LABEL1);
  url += String(LABEL_DATA_SEP) + String(rainValue);
  
  url += String(OTHER_SEP) + String(LABEL2);
  url += String(LABEL_DATA_SEP) + String(temp);
  
  url += String(OTHER_SEP) + String(LABEL3);
  url += String(LABEL_DATA_SEP) + String(hum);
    
  url += String(OTHER_SEP) + String(LABEL4);
  url += String(LABEL_DATA_SEP) + String(pres);
  
  url += String(OTHER_SEP) + String(LABEL5);
  url += String(LABEL_DATA_SEP) + String(mv);
  
  
  url += String("&created_at=") + String(now.year(), DEC) + String("/") + String(now.month(), DEC) + String("/") + String(now.date(), DEC) + String("T") + String(now.hour(), DEC) + String(":") + String(now.minute(), DEC) + String(":") + String(now.second(), DEC) + String("&timezone=Africa/Nairobi");
  
  Serial.println(url);
  
  FiveSecondDelay();
  FiveSecondDelay();
  FiveSecondDelay();
  
    char result[20] = "";
  gprsbee.doHTTPGET(APN, APN_USERNAME, APN_PASSWORD, url.c_str(), result, sizeof(result));
  
  Serial.println("Received: " + String(result));
  
  wdt_reset();
  FiveSecondDelay();
  wdt_reset();
  
  gprsbee.off();
  
   dataFile = SD.open("datalog.txt", FILE_WRITE);
   if (! dataFile) {
    Serial.println(F("error opening datalog.txt"));
    // Wait forever since we cant write data
    }
   
  dataFile.print(now.year(), DEC);
  dataFile.print('/');
  dataFile.print(now.month(), DEC);
  dataFile.print('/');
  dataFile.print(now.date(), DEC);
  dataFile.print(',');
  dataFile.print(now.hour(), DEC);
  dataFile.print(':');
  dataFile.print(now.minute(), DEC);
  dataFile.print(':');
  dataFile.print(now.second(), DEC);
  dataFile.print(',');
  dataFile.print(rainValue);
  dataFile.print(',');
  dataFile.print(temp);
  dataFile.print(',');
  dataFile.print(hum);
  dataFile.print(',');
  dataFile.print(pres);
  dataFile.print(',');
  dataFile.println(mv);
  
  dataFile.flush();
  
  wdt_reset();
  
  rainValue = 0;
  
  delay(500);
}

void FiveSecondDelay()
{
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);
  wdt_reset();
  delay(500);  
  wdt_reset();
}

/*
 * Return the current timestamp
 *
 * This is a general purpose wrapper function to get the current timestamp.
 * It can also be used for timer.setNowCallback
 */
uint32_t getNow()
{
  return rtc.now().getEpoch();
}

//######### watchdog and system sleep #############
void systemSleep()
{
  ADCSRA &= ~_BV(ADEN);         // ADC disabled

  /*
  * Possible sleep modes are (see sleep.h):
  #define SLEEP_MODE_IDLE         (0)
  #define SLEEP_MODE_ADC          _BV(SM0)
  #define SLEEP_MODE_PWR_DOWN     _BV(SM1)
  #define SLEEP_MODE_PWR_SAVE     (_BV(SM0) | _BV(SM1))
  #define SLEEP_MODE_STANDBY      (_BV(SM1) | _BV(SM2))
  #define SLEEP_MODE_EXT_STANDBY  (_BV(SM0) | _BV(SM1) | _BV(SM2))
  */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  /*
   * This code is from the documentation in avr/sleep.h
   */
  cli();
  // Only go to sleep if there was no watchdog interrupt.
  if (!hz_flag)
  {
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
  }
  sei();

  ADCSRA |= _BV(ADEN);          // ADC enabled
}
// The watchdog timer is used to make timed interrupts
// This is a modified version of wdt_enable() from avr/wdt.h
// The avr/wdt.h version only sets WDE (not WDIE for interrupts)
#if 1
// Both WDE and WDIE are set!!
// Note from the doc: "Executing the corresponding interrupt
// vector will clear WDIE and WDIF automatically by hardware
// (the Watchdog goes to System Reset Mode)
#define my_wdt_enable(value)   \
__asm__ __volatile__ (  \
    "in __tmp_reg__,__SREG__" "\n\t"    \
    "cli" "\n\t"    \
    "wdr" "\n\t"    \
    "sts %0,%1" "\n\t"  \
    "out __SREG__,__tmp_reg__" "\n\t"   \
    "sts %0,%2" "\n\t" \
    : /* no outputs */  \
    : "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
      "r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
      "r" ((uint8_t) (((value & 0x08) ? _WD_PS3_MASK : 0x00) | \
          _BV(WDE) | _BV(WDIE) | (value & 0x07)) ) \
    : "r0"  \
)
#else
// Only WDIE is set!!
#define my_wdt_enable(value)   \
__asm__ __volatile__ (  \
    "in __tmp_reg__,__SREG__" "\n\t"    \
    "cli" "\n\t"    \
    "wdr" "\n\t"    \
    "sts %0,%1" "\n\t"  \
    "out __SREG__,__tmp_reg__" "\n\t"   \
    "sts %0,%2" "\n\t" \
    : /* no outputs */  \
    : "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
      "r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
      "r" ((uint8_t) (((value & 0x08) ? _WD_PS3_MASK : 0x00) | \
          _BV(WDIE) | (value & 0x07)) ) \
    : "r0"  \
)
#endif

void setupWatchdog()
{
  my_wdt_enable(WDTO_1S);
}

//################ interrupt ################
ISR(WDT_vect)
{
  hz_flag = true;
}

float getRealBatteryVoltage()
{
   uint16_t batteryVoltage = analogRead(BATVOLTPIN);

   return (ADC_AREF / 1023.0) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * batteryVoltage; 
}

void doCheckGPRSoff(uint32_t now)
{
  // Maybe we shouldn't care about WDT. If it does not switch
  // off properly it could be a good thing to reset the system.
  // TODO We need to verify this somehow.
  gprsbee.off();
  for (uint8_t i = 0; i < 10; ++i) {
    delay(50);
    if (gprsbee.off()) {
      break;
    }
  }
}
void rainInterrupt()
{
    rainValue += 0.1;  // everytime it moves
}
