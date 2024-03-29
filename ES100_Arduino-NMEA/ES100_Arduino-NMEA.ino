
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// es100_host_arduino.c
//
// Xtendwave ES100 Example Host MCU Code for Arduino - version 0002
//
// Copyright 2013 Xtendwave
//
// THIS SOFTWARE IS PROVIDED TO YOU "AS IS," AND WE MAKE NO EXPRESS OR IMPLIED
// WARRANTIES WHATSOEVER WITH RESPECT TO ITS FUNCTIONALITY, OPERABILITY, OR USE,
// INCLUDING, WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, OR INFRINGEMENT. WE EXPRESSLY DISCLAIM ANY
// LIABILITY WHATSOEVER FOR ANY DIRECT, INDIRECT, CONSEQUENTIAL, INCIDENTAL OR
// SPECIAL DAMAGES, REGARDLESS OF THE FORM OF ACTION OR LEGAL THEORY UNDER
// WHICH THE LIABILITY MAY BE ASSERTED, EVEN IF ADVISED OF THE POSSIBILITY OR
// LIKELIHOOD OF SUCH DAMAGES.
//
// This software contains example C code that shows how an Arduino host
// microcontroller can control an ES100 device. The software uses the
// Serial.print() function to display the time.
//
// 4 pins are used. 1 digital output pin controls EN and 1 digital input pin
// monitors IRQ. Two pins, SCL and SDA, connect to the ES100 Serial interface.
//
// This software was testing using Arduino Software version 1.5.2 and an Arduino Due.
//
// User-Supplied Functions
//
//   mcu_init
//   mcu_gpio_set_high
//   mcu_gpio_set_low
//   mcu_gpio_read
//   mcu_timer_read
//   mcu_timer_wait_us
//
// General I2C Functions
//
//   i2c_write
//   i2c_read
//
// ES100 Functions
//
//   es100_write_register
//   es100_read_register
//   es100_enable
//   es100_disable
//   es100_start_rx
//   es100_wait_for_irq
//   es100_get_irq_status
//   es100_read_time
//
// Other
//
//   setup
//   loop
//
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#include <Wire.h>
#include <Rtc_Pcf8563.h>
#define PCF8563address 0x51

//init the real time clock
Rtc_Pcf8563 rtc;
#define CENTURY  2000 

//assuming this is programmed with the USB port 
#ifdef ARDUINO_SAMD_ZERO
#define Serial                      SerialUSB
#endif

#define PPS_DURATION 200 //200ms

#define INIT_TIMER_COUNT 6
#define RESET_TIMER2 TCNT2 = INIT_TIMER_COUNT
int tickCounter = 0;
int ppsCounter = 0;


//------------------------------------------------------------------------------
// MCU constants - USER TO MODIFY
//------------------------------------------------------------------------------

// GPIO pins
#define GPIO_EN      17  //TODO: figure out pins 
#define GPIO_IRQ     15  
#define GPIO_PPS     2    
#define GPIO_CLK     3   


//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------

// I2C slave address
#define ES100_SLAVE_ADDR 0x32

// enable delay time
//#define ENABLE_DELAY_US 1500
#define ENABLE_DELAY_US 100000       // 100ms: module version 1 only

// I2C clock high/low time
#define SCL_TIME_US 2

// ES100 API register addresses
#define ES100_CONTROL0_REG       0x00
#define ES100_CONTROL1_REG       0x01
#define ES100_IRQ_STATUS_REG     0x02
#define ES100_STATUS0_REG        0x03
#define ES100_YEAR_REG           0x04
#define ES100_MONTH_REG          0x05
#define ES100_DAY_REG            0x06
#define ES100_HOUR_REG           0x07
#define ES100_MINUTE_REG         0x08
#define ES100_SECOND_REG         0x09
#define ES100_NEXT_DST_MONTH_REG 0x0A
#define ES100_NEXT_DST_DAY_REG   0x0B
#define ES100_NEXT_DST_HOUR_REG  0x0C

//------------------------------------------------------------------------------
// array to store date and time
//------------------------------------------------------------------------------

#define DT_STATUS         0
#define DT_YEAR           1
#define DT_MONTH          2
#define DT_DAY            3
#define DT_HOUR           4
#define DT_MINUTE         5
#define DT_SECOND         6
#define DT_NEXT_DST_MONTH 7
#define DT_NEXT_DST_DAY   8
#define DT_NEXT_DST_HOUR  9

#define DT_LENGTH        10

//------------------------------------------------------------------------------
// mcu functions - USER TO MODIFY
//------------------------------------------------------------------------------

void mcu_init(void)
{
  // optional code to initialize GPIO, timer and other MCU functions
  pinMode(GPIO_EN, OUTPUT);
  pinMode(GPIO_PPS, OUTPUT);
  pinMode(GPIO_IRQ, INPUT);
  Wire.begin();
  Serial.begin(9600);
}

void mcu_gpio_set_high(int pin)
{
  // set pin to a logic one
  digitalWrite(pin, HIGH);
}

void mcu_gpio_set_low(int pin)
{
  // set pin to a logic zero
  digitalWrite(pin, LOW);
}

int mcu_gpio_read(int pin)
{
  // read current state of input pin
  return(digitalRead(pin));
}

unsigned long mcu_timer_read(void)
{
  // read timer value and return as integer
  return(millis());
}

void mcu_timer_wait_us(int count)
{
  // wait 1 microsecond or greater for each count
  delayMicroseconds(count);
}

//------------------------------------------------------------------------------
// write I2C data - USER TO MODIFY
//------------------------------------------------------------------------------

void i2c_write(uint8_t slave_addr, uint8_t num_bytes, uint8_t *ptr)
{

  int i;

  Wire.beginTransmission(slave_addr);

  for (i=0; i<num_bytes; i++)
  {
    Wire.write(ptr[i]);
  }

  Wire.endTransmission();

}

//------------------------------------------------------------------------------
// read I2C data - USER TO MODIFY
//------------------------------------------------------------------------------

void i2c_read(uint8_t slave_addr, uint8_t num_bytes, uint8_t *ptr)
{

  int i;
  const uint8_t stop_flag = 1;

  Wire.requestFrom(slave_addr, num_bytes, stop_flag);
  for(i=0; (i<num_bytes && Wire.available()); i++)
  {
    ptr[i] = Wire.read();
  }
}

//------------------------------------------------------------------------------
// write data to an ES100 API register
//------------------------------------------------------------------------------

void es100_write_register(uint8_t addr, uint8_t data)
{
  uint8_t es100_write_array[2];

  es100_write_array[0]= addr;
  es100_write_array[1]= data;
  i2c_write(ES100_SLAVE_ADDR, 0x2, es100_write_array);
}

//------------------------------------------------------------------------------
// read data from an ES100 API register
//------------------------------------------------------------------------------

uint8_t es100_read_register(uint8_t addr)
{
  uint8_t   data;

  i2c_write(ES100_SLAVE_ADDR, 0x1, &addr);
  i2c_read(ES100_SLAVE_ADDR, 0x1, &data);

  return(data);
}

//------------------------------------------------------------------------------
// enable ES100
//------------------------------------------------------------------------------

void es100_enable(void)
{
  mcu_gpio_set_high(GPIO_EN);
}

//------------------------------------------------------------------------------
// disable ES100
//------------------------------------------------------------------------------

void es100_disable(void)
{
  mcu_gpio_set_low(GPIO_EN);
}

//------------------------------------------------------------------------------
// start reception
//------------------------------------------------------------------------------

void es100_start_rx()
{
  es100_write_register(ES100_CONTROL0_REG, 0x01);

  // perform read of control register for i2c debug only
  es100_read_register(ES100_CONTROL0_REG);
}

//------------------------------------------------------------------------------
// wait for falling edge of IRQ
//------------------------------------------------------------------------------

void es100_wait_for_irq()
{
  while(mcu_gpio_read(GPIO_IRQ));   // wait until IRQ is low
}

//------------------------------------------------------------------------------
// read IRQ status register
//------------------------------------------------------------------------------

uint8_t es100_get_irq_status()
{
  //Read IRQ status register
  return(es100_read_register(ES100_IRQ_STATUS_REG));
}

//------------------------------------------------------------------------------
// read date and time from API registers
//------------------------------------------------------------------------------

void es100_read_time(int dt_array[])
{
  dt_array[DT_STATUS]         = es100_read_register(ES100_STATUS0_REG);
  dt_array[DT_YEAR]           = es100_read_register(ES100_YEAR_REG);
  dt_array[DT_MONTH]          = es100_read_register(ES100_MONTH_REG);
  dt_array[DT_DAY]            = es100_read_register(ES100_DAY_REG);
  dt_array[DT_HOUR]           = es100_read_register(ES100_HOUR_REG);
  dt_array[DT_MINUTE]         = es100_read_register(ES100_MINUTE_REG);
  dt_array[DT_SECOND]         = es100_read_register(ES100_SECOND_REG);
  dt_array[DT_NEXT_DST_MONTH] = es100_read_register(ES100_NEXT_DST_MONTH_REG);
  dt_array[DT_NEXT_DST_DAY]   = es100_read_register(ES100_NEXT_DST_DAY_REG);
  dt_array[DT_NEXT_DST_HOUR]  = es100_read_register(ES100_NEXT_DST_HOUR_REG);
}

//------------------------------------------------------------------------------
// top level function to receive time from WWVB
//------------------------------------------------------------------------------

/*===================================== Project code starts here =====================*/
int         dt_array[DT_LENGTH];
unsigned long lastSync = 0;
volatile bool irqReady = false;
unsigned long  current_timer_value;
bool es100_receiving = false; 
bool clockStarted = false;
// Time variables
volatile byte ss   = 0;                        // seconds
volatile byte mm   = 0;                        // minutes
volatile byte hh   = 0;                        // UTC hours
volatile byte day  = 0;                       // day
volatile word doy  = 0;                       // day of year
volatile byte mon  = 0;                       // month
volatile word year = 0;                      // year
volatile byte dst  = 0;                       // daylight saving time flag
volatile byte lyr  = 0;                       // leap year flag
byte previousSecond = 0;
unsigned long previousMills = 0;
bool doTick = false;

void ClockInt() {
   ss++;
    if (ss == 60) {       // increment minute
      ss = 0;
      mm++;
      if (mm == 60) {     // increment hour
        mm = 0;
        hh++;
        if (hh == 24) {   // increment day
          hh = 0;
          doy++;
          if (doy == (366 + lyr)) { // incr year
            doy = 1;
            year++;
          }
        }
      }
    }
    tickCounter = 0;
   //Serial.println(ss, DEC);
   doTick = true;

}

void Clock(unsigned long ms) {
  tickCounter += ms;       // increment millisecond
   if (tickCounter >= 1000) {
    tickCounter = tickCounter % 1000;
  } 
}

// convert BCD to decimal numbers
byte bcdToDec(byte val) {
  return ((val/16*10) + (val%16));
}


void int0handler() {
  //triggered on falling edge of IRQ
  irqReady = true;
}

bool es100_process_irq() {
    current_timer_value = mcu_timer_read();

    // read interrupt status
    int irq_status = es100_get_irq_status();
    irqReady = false;

    if (irq_status == 0x01) {
       es100_read_time(dt_array);
       return true;
    }
    Serial.println("received");
    Serial.print("IRQ status = 0x");
    Serial.println(irq_status, HEX);
  return false;
}

void serialDumpTime(void) {
   bool displayNMEA = false;
   char timeString[16];
   char dateString[12];
   char tempString[8];
    if (year == 0) {
    } else {
      Serial.print("Time: ");
      if (!displayNMEA) {                   // display either UTC or local time
        Serial.println(rtc.formatTime()); //now dump RTC time
        sprintf(timeString, "%02d:%02d:%02d.%04i UTC ", hh, mm, ss, tickCounter);
        Serial.println(timeString);
        Serial.println("---");
      }  else {

      }

    }
}
void setup()
{
  // initialize MCU
  mcu_init();
  
  // initialize gpios
//  rtc.initClock();
//  rtc.setDate(16, 3, 11, 20, 20);
//  rtc.setTime(11, 0, 0);
  rtc.setSquareWave(SQW_1HZ);
  
  pinMode(GPIO_CLK, INPUT);
  pinMode(GPIO_PPS, OUTPUT);


  digitalWrite(GPIO_PPS, LOW);
  Serial.println("enabling es100");
  es100_enable();

  mcu_timer_wait_us(ENABLE_DELAY_US);

  attachInterrupt(digitalPinToInterrupt(GPIO_IRQ), int0handler, FALLING);
  ss=rtc.getSecond();
  mm=rtc.getMinute();
  hh=rtc.getHour();
  attachInterrupt(digitalPinToInterrupt(GPIO_CLK), ClockInt, FALLING);
  Serial.println(rtc.formatTime());
}

void loop() {
  unsigned long clk = 0;
  
  //advance clock:
  clk = millis();
  if(clk != previousMills) {
        Clock(clk - previousMills);
        previousMills = clk;
  }
  if(doTick && clockStarted) {
    ppsCounter = clk; 
    digitalWrite(GPIO_PPS, HIGH);
  }
  if(!doTick && (clk + PPS_DURATION) > ppsCounter) {
    digitalWrite(GPIO_PPS, LOW);
  }
  unsigned long duration;
  // receive time from WWVB
  if(!es100_receiving) {
     Serial.println("start reception");
     es100_start_rx();
     es100_receiving = true;
  }
  if(irqReady && es100_process_irq() ) {
     //Gotten the time from WWVB lets update everything
    tickCounter=0;       //restart ms counter
    ss = bcdToDec(dt_array[DT_SECOND]);
    mm = bcdToDec(dt_array[DT_MINUTE]);
    hh = bcdToDec(dt_array[DT_HOUR]);
    mon = bcdToDec(dt_array[DT_MONTH]);
    day = bcdToDec(dt_array[DT_DAY]);
    year = CENTURY + bcdToDec(dt_array[DT_YEAR]);
    rtc.setDate(day, 0, mon, 20, (year - CENTURY)); //set RTC date - Not bothering about DoW
    rtc.setTime(hh, mm, ss);                        //set RTC time 
    es100_receiving = false;
    if(!clockStarted || mm % 5 == 0) {
      //restart 1HZ square wave if we weren't synced before 
      rtc.clearSquareWave();
      rtc.setSquareWave(SQW_1HZ);
    }
    ss=rtc.getSecond();        //read RTC seconds back in 
    clockStarted=true;
    Serial.println("successful reception");

  }
  if (ss != previousSecond) {    // upon seconds change
    previousSecond = ss;         // store previous second
    if(es100_receiving == false)
    serialDumpTime();            // print internal time to serial
  }
 
  if (doTick) {
    doTick = false;
  }
  

}
