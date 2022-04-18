#include <SPI.h>         

#define wwvbRxPin      2

char str[20];
int nStart,nEnd,nWid;
int nLastDecode;
int nBytes[6];
int nIndex, nMask;
int nMode;
int hour, minute, day, year, second;
int nPinState;
int keyVal, oldKey, curKey;
bool edgeIgnore=false;

void setup()
{
  pinMode(wwvbRxPin,INPUT);    // clock receiver input
      
  for(int i =0; i < 5; i++)
  {
    nBytes[i] = 0;
  }
  nIndex = 0;
  nMode = 0;
  nMask = 0x80;
  nStart = nEnd = millis();
  nPinState = digitalRead(wwvbRxPin);
  hour = minute = second = day = 0;
  year = 2000;
  
  Serial.begin(9600);
  while(!Serial);
  Serial.println("--------------START-----------");
}


void loop()
{

    readclock();
}

void readclock(void)
{
int nRead;
  nRead = digitalRead(wwvbRxPin);
  if(nRead != nPinState)
  {
    nPinState = nRead;
    
    // we get here on every transition of the input signal (edge)
    if(nRead == HIGH)
    {
      // when sig goes high, just save the systenm clock
     nStart = millis();

      if(!edgeIgnore) {
              second++;
              if(second == 60) {
                     second = 0;
              }
        //      if(second < 10) 
        //          Serial.print('0');
        //      Serial.print(second);
      }
    }
    else if(nRead == LOW)
    {
      // when sig goes low, do all the work
      // this means we have a new bit from the receiver
      nEnd = millis();
      // nWid is our measurement of the pulse width in ms.
      // this can vary a bit, so we dice it up into three
      // very generous windows.
      nWid = nEnd - nStart;

      if(nWid < 100) {
        edgeIgnore=true;
        return;
      }
      edgeIgnore=false;
      
      if(nWid > 900)
      Serial.print("Width: ");
      if(nWid > 900)
        Serial.println(nWid);
      if(nWid > 725)  // > 725ms = sync pulse
      {
        if(nLastDecode == 2)
        {
          dsync();
          nLastDecode = 3;
         }
         else
         {
           sync();
           nLastDecode = 2;
         }
       }
       else if(nWid < 275)
       {
         its0();
         nLastDecode = 0;
       }
       else
       {
         its1();
         nLastDecode = 1;
       }
     }
  }
}


void its1(void)
{
  nBytes[nIndex] |= nMask;
  nMask = nMask >> 1;
//Serial.println("1");
}

void its0(void)
{
  nMask = nMask >> 1;
//  Serial.println("0");
}

void sync(void)
{
  nMask = 0x100;
  nIndex++;
//  Serial.println("S");
}

void dsync(void)
{
  second = 0;
  nMask = 0x80;
  nIndex = 0;
  showit();
  for(int i =0; i < 5; i++)
  {
    nBytes[i] = 0;
  }
  nMode = 1;
  Serial.println("D");
  
}

void showit(void)
{
  int i;
      if(nMode == 1)
      {
        Decode();
        Serial.print("UTC: ");
         if(hour <= 9) 
            Serial.print("0");
        Serial.print(hour);
        Serial.print(":");
        if(minute <= 9) 
            Serial.print("0");   // do I have to do everything?
        Serial.print(minute);
        Serial.print(":");
        Serial.println(second);
        Serial.print("DAY: ");
        Serial.println(day);
        Serial.print(" YR: ");
        Serial.println(year);
      }
      else
      {
        Serial.println("In sync");
      }
}


void Decode(void)
{
  minute = nBytes[0] & 0x0f;
  minute += ((nBytes[0] >> 5) & 0x07) * 10;

  hour = nBytes[1] & 0x0f;
  hour += ((nBytes[1] >> 5) & 0x07) * 10;

  //  WWVB sends the time AFTER the time mark, so we need to add a minute
  //  for the correct time.
  minute++;
  if(minute >= 60)
  {
    minute = 0;
    hour++;
    if(hour >= 24)
    {
      hour = 0;
    }
  }
  
  day = (nBytes[3] >> 5) & 0x0f;
  day += (nBytes[2] & 0x0f) * 10;
  day += ((nBytes[2] >> 5) & 0x03) * 100;

  year = (nBytes[5] >> 5) & 0x0f;
  year += ((nBytes[4]) & 0x0f) * 10;
}


// send an NTP request to the time server at the given address 
