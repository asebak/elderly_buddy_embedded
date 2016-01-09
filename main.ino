#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
//Name: Elderly Buddy Embedded System Code
//Functions: Sleeps, Serial Communication, Temperature, Heart Rate, Battery Life 
//Refrences: www.pulsesensor.com
#define batteryLevel A2
#define tempPin A4
#define heartPin A7
#define timeCycle 20
#define KelvinConversion 273.15
#define TempPulseThresholdold 25   
#define TemperatureSAmplifiedSignallingIntervalRateList 5
#define Beta 3950
#define Thermistor 10000    
#define voltage 3.3
#define BaudRate 57600
#define PWMpins 0x02
#define PreScaler 0x05
#define InterruptRate 0X7C
#define InterruptPins 0x02
#define DelayRate 3000
#define WatchDogTimer1 B00011000
#define WatchDogTimer2 B01100001
struct ElderlyBuddy{
float R1;//R1 Connected to battery
float R2;// R2 Connected to GND of MCU
float Voutput;
float Vinput;
float SAmplifiedSignallingRate;     
float HeartTimer;  
int HeartIntervalRateList;                
int ProcessingSignal;             
int IntervalTime;           
int IntervalRateListArray[10];
int Pulse;
int Timer;  
int PulseThreshold;             
int AmplifiedSignal;                   
int TimerSleep;  
bool IntervalRateList;    
bool BeatFound;  
bool SampleOne; 
bool SampleTwo; 
};
ElderlyBuddy *HW;
void setup()
{
 Serial.begin(BaudRate);
 analogReference(EXTERNAL);
 TCCR2A = PWMpins; 
 TCCR2B = PreScaler;
 OCR2A = InterruptRate; 
 TIMSK2 = InterruptPins;
 HW->IntervalRateList=HW->BeatFound=false;
 HW->SampleOne=HW->SampleTwo =true;    
 HW->SAmplifiedSignallingRate=HW->HeartTimer=HW->TimerSleep=0;
 HW->Pulse=HW->Timer=HW->PulseThreshold=512;           
 HW->AmplifiedSignal = 100; 
 HW->IntervalTime=600; 
 sei();  
}

void loop()
{

 if (HW->TimerSleep == timeCycle) {  //if true MCU awakens
    Serial.println(getBatteryLevel());
     delay(DelayRate);
    Serial.println(getHardwareTemperature());
      delay(DelayRate);
    Serial.println(getHeartRate());
      delay(DelayRate);
     HW->TimerSleep = 0;//reset timer, 
 } 
  
 else
HW->TimerSleep++;      //Watching Enabled,Sleeping Enabled           
SleepMicrocontroller();
}





void SleepMicrocontroller()
{
  //Setting Watchdog Register
  wdt_reset();        
  cli();
  MCUSR = 0;
  WDTCSR |= WatchDogTimer1;
  WDTCSR = WatchDogTimer2;   
  sei();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();                                           
  sleep_mode(); 
  sleep_disable();                     
}


float getBatteryLevel()//voltage divider 3.3kohm with 12 kohm
{
  HW->Voutput= (analogRead(batteryLevel) * voltage) / 1024;
  HW->Vinput = (HW->Voutput) / (HW->R2/(HW->R1+HW->R2));
  return HW->Vinput;
}

float getHardwareTemperature()
{
   
int sAmplifiedSignalles[TemperatureSAmplifiedSignallingIntervalRateList];
  uint8_t i;
  float average;
 
  // take N sAmplifiedSignalles in a row, with a slight delay
  for (i=0; i< 5; i++) {
   sAmplifiedSignalles[i] = analogRead(tempPin);
   delay(10);
  }
 
  // average all the sAmplifiedSignalles out
  average = 0;
  for (i=0; i< TemperatureSAmplifiedSignallingIntervalRateList; i++) {
     average += sAmplifiedSignalles[i];
  }
  average /= TemperatureSAmplifiedSignallingIntervalRateList;
 
  // convert the value to Thermistor
  average = 1023 / average - 1;
  average = Thermistor / average;
 
  float steinhart;
  steinhart = average / Thermistor;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= Beta;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TempPulseThresholdold + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;       
  return steinhart;
}

float getHeartRate()
{
  if(HW->BeatFound)
  {
   HW->BeatFound=false;
   return HW->HeartIntervalRateList;
  }
  
}


ISR (WDT_vect) {
  cli();
  wdt_disable();
  sei();
}



ISR(TIMER2_COMPA_vect){         
    HW->ProcessingSignal = analogRead(heartPin);              // read the IntervalRateList Sensor 
    HW->SAmplifiedSignallingRate += 2;                         // keep track of the time in mS with this variable
    int N = HW->SAmplifiedSignallingRate - HW->HeartTimer;       // monitor the time since the last beat to avoid noise

//  find the peak and trough of the IntervalRateList wave
    if(HW->ProcessingSignal < HW->PulseThreshold && N > (HW->IntervalTime/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IntervalTime
        if (HW->ProcessingSignal < HW->Timer){                        // T is the trough
            HW->Timer = HW->ProcessingSignal;                         // keep track of lowest point in IntervalRateList wave 
         }
       }
      
    if(HW->ProcessingSignal > HW->PulseThreshold && HW->ProcessingSignal > HW->Pulse){          // PulseThreshold condition helps avoid noise
       HW->Pulse = HW->ProcessingSignal;                             // P is the peak
       }                                        // keep track of highest point in IntervalRateList wave
    
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // ProcessingSignal surges up in value every time there is a IntervalRateList
if (N > 250){                                   // avoid high frequency noise
  if ( (HW->ProcessingSignal > HW->PulseThreshold) && (HW->IntervalRateList == false) && (N > (HW->IntervalTime/5)*3) ){        
    HW->IntervalRateList = true;                               // set the IntervalRateList flag when we think there is a IntervalRateList              // turn on pin 13 LED
    HW->IntervalTime = HW->SAmplifiedSignallingRate - HW->HeartTimer;         // measure time between beats in mS
    HW->HeartTimer = HW->SAmplifiedSignallingRate;               // keep track of time for next IntervalRateList
         
         if(HW->SampleOne){                         // if it's the first time we found a beat, if SampleOne == TRUE
             HW->SampleOne = false;                 // clear SampleOne flag
             return;                            // IntervalTime value is unreliable so discard it
            }   
         if(HW->SampleTwo){                        // if this is the second beat, if SampleTwo == TRUE
            HW->SampleTwo = false;                 // clear SampleTwo flag
               for(int i=0; i<=9; i++){         // seed the running total to get a realisitic HeartIntervalRateList at startup
                   HW-> IntervalRateListArray[i] = HW->IntervalTime;                      
                    }
            }
          
    // keep a running total of the last 10 IntervalTime values
    word runningTotal = 0;                   // clear the runningTotal variable    

    for(int i=0; i<=8; i++){                // shift data in the IntervalRateList array
          HW->IntervalRateListArray[i] = HW->IntervalRateListArray[i+1];              // and drop the oldest IntervalTime value 
          runningTotal += HW->IntervalRateListArray[i];          // add up the 9 oldest IntervalTime values
        }
        
    HW->IntervalRateListArray[9] = HW->IntervalTime;                          // add the latest IntervalTime to the IntervalRateList array
    runningTotal += HW->IntervalRateListArray[9];                // add the latest IntervalTime to runningTotal
    runningTotal /= 10;                     // average the last 10 IntervalTime values 
    HW->HeartIntervalRateList = 60000/runningTotal;               // how many beats can fit into a minute? that's HeartIntervalRateList!
    HW->BeatFound = true;                              // set Quantified Self flag 
    // BeatFound FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
}

  if (HW->ProcessingSignal < HW->PulseThreshold && HW->IntervalRateList == true){     // when the values are going down, the beat is over
      HW->IntervalRateList = false;                         // reset the IntervalRateList flag so we can do it again
      HW->AmplifiedSignal = HW->Pulse - HW->Timer;                           // get AmplifiedSignallitude of the IntervalRateList wave
      HW->PulseThreshold = HW->AmplifiedSignal/2 + HW->Timer;                    // set PulseThreshold at 50% of the AmplifiedSignallitude
      HW->Pulse = HW->PulseThreshold;                            // reset these for next time
      HW->Timer = HW->PulseThreshold;
     }
  
  if (N > 2500){                             // if 2.5 seconds go by without a beat
      HW->PulseThreshold = 512;                          // set PulseThreshold default
      HW->Pulse = 512;                               // set P default
     HW-> Timer = 512;                               // set T default
      HW->HeartTimer = HW->SAmplifiedSignallingRate;          // bring the HeartTimer up to date        
      HW->SampleOne = true;                      // set these to avoid noise
      HW->SampleTwo = true;                     // when we get the heartbeat back
     }

  }// end isr


