//////////////////////////////////////////////////////////////////
//©2011 bildr
//Released under the MIT License - Please reuse change and share
//Simple code to output a PWM sine wave signal on pin 9
//////////////////////////////////////////////////////////////////

#define lightPin 9
#define potPin A1

#define smoothSteps 50 // Smooth steps (50 index's == 2.5 second smoothing with 50ms sampleWindow)
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)

float voltArr[smoothSteps]; 
int voltArrIndex = 0;

unsigned int sample;

boolean fading;

void setup(){
  pinMode(lightPin, OUTPUT);  
  Serial.begin(9600);
}

void loop(){
  fading = false;
  unsigned long startMillis= millis();  // Start of sample window
  
  //Mic listener 1 variables
  unsigned int peakToPeak = 0;   // peak-to-peak level
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  
  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    //Mic 1
     sample = analogRead(0);
     if (sample < 1024)  // toss out spurious readings
     {
        if (sample > signalMax)
        {
           signalMax = sample;  // save just the max levels
        }
        else if (sample < signalMin)
        {
           signalMin = sample;  // save just the min levels
        }
     }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  double volts = (peakToPeak * 3.3) / 1024;  // convert to volts

  double smoothedVolt = getSmoothedVolt(volts);
  Serial.println(smoothedVolt);
  
  if (volts > smoothedVolt + 0.1) {
    fading = true; 
  }
  
  if (fading) {
    int potVal = 25;
    for(int i = 0; i<180; i++){
      //convert 0-360 angle to radian (needed for sin function)
      float rad = DEG_TO_RAD * i;
  
      //calculate sin of angle as number between 0 and 255
      int sinOut = constrain((sin(rad) * 255), 0, 255); 
      analogWrite(lightPin, sinOut);
  
      delay(potVal);
    }
    analogWrite(lightPin, 0);
    fading = false;
  }
}

double getSmoothedVolt(double thisVolt) {
   //Set smoothing values
   voltArr[voltArrIndex] = thisVolt;
   voltArrIndex++;
   if (voltArrIndex == smoothSteps) {
     voltArrIndex = 0; 
   }

   //Get smoothing values
   double totalVolts = 0.0;
   for (int i=0; i<smoothSteps; i++) {
      totalVolts += voltArr[i];
   }
   return totalVolts/smoothSteps;
}
