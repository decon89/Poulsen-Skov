//Settings (Disse må justeres!)
double maxPower = 10.0; //Fade område, afgør hvor lang tid det skal tage at nå max lys intensitet - jo højere destro længere tid - Default 10.0;
double maxVolts = 0.5; //Sensitivitet - jo lavere detro mere sensitiv (max er 3.0) - Default 0.5
double ambientNoise = 0.0; //En fælles variable der styrer hvor meget ambient støj der er i området (vind, biler, etc) - 0.0 på kontoret, burde også være 0.0 i realiteten, men nu kan den styres just in case.

//Idle settings
double idleThreshold = 30.0; //Bestemmer hvor høj intensitet idle fadet skal nå - jo højere destro stærkere - Default 30.0 : 255 er max
double idleSpeed = 1.7; //Bestemmer hvor hurtigt idle-fadet skal være - jo højere destro hurtigere - Default 1.7
double idlePause = 5.0; //Bestemmer pausen mellem idle-fades - Default 5.0
double idleTimeout = 3.0; //Bestemmer hvor mange sekunders stilhed der skal være, før devicet går i idle tilstand. 

//Special settings
double decayRate = 1.0; //Bestemmer hvor hurtigt lyset skal fade ud - jo højere destro hurtigere - Default 1.0

//Variables (ikke pil ved dem her)
double noiseFloorMic = 0.08; //Skal kalibreres on-location. Bestemmer hvor cut-off punktet skal være for den ambiente støj på både mics og i området. - skal kun reagere over dette niveau.
#define lightPin 9
#define smoothSteps 10 // Smooth steps (50 index's == 2.5 second smoothing with 50ms sampleWindow)
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
float voltArr[smoothSteps];  
int voltArrIndex = 0;
double power = 0.0;
boolean up = true;
double minVolts = 0.02;
double resistance = 0.06;
boolean idle = true;
double currentIdleBrightness = 0.0;
double currentIdleIndex = 0.0;
boolean upIdleDirection = true;
unsigned long triggerTime;

unsigned int sample; 

void setup(){
  pinMode(lightPin, OUTPUT);  
  Serial.begin(9600);
}

void loop(){
  unsigned long startMillis= millis();  // Start of sample window
  
  //Mic listener 1 variables
  unsigned int peakToPeak = 0;   // peak-to-peak level
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  
  //Mic listener 2 variables
  unsigned int peakToPeak2 = 0;   // peak-to-peak level
  unsigned int signalMax2 = 0;
  unsigned int signalMin2 = 1024;
  
  //Mic listener 3 variables
  unsigned int peakToPeak3 = 0;   // peak-to-peak level
  unsigned int signalMax3 = 0;
  unsigned int signalMin3 = 1024;
  
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
     //Mic 2
     sample = analogRead(1); 
     if (sample < 1024)  // toss out spurious readings
     {
        if (sample > signalMax2)
        {
           signalMax2 = sample;  // save just the max levels
        }
        else if (sample < signalMin2)
        {
           signalMin2 = sample;  // save just the min levels
        }
     } 
     //Mic 3
     sample = analogRead(2); 
     if (sample < 1024)  // toss out spurious readings
     {
        if (sample > signalMax3)
        {
           signalMax3 = sample;  // save just the max levels
        }
        else if (sample < signalMin3)
        {
           signalMin3 = sample;  // save just the min levels
        }
     }
  }
  
  //Mic 1
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  double volts = (peakToPeak * 3.3) / 1024;  // convert to volts
  volts -= (noiseFloorMic + ambientNoise);
  volts = constrain(volts,0.0,maxVolts);
  
  //Mic 2
  peakToPeak2 = signalMax2 - signalMin2;  // max - min = peak-peak amplitude
  double volts2 = (peakToPeak2 * 3.3) / 1024;
  volts2 -= (noiseFloorMic + ambientNoise);
  volts2 = constrain(volts2,0.0,maxVolts);
  
  //Mic 3
  peakToPeak3 = signalMax3 - signalMin3;  // max - min = peak-peak amplitude
  double volts3 = (peakToPeak3 * 3.3) / 1024;  // convert to volts
  volts3 -= (noiseFloorMic + ambientNoise);
  volts3 = constrain(volts3,0.0,maxVolts);
  
  //Add all voltages together and limit to maxVolts
  double allVolts = volts + volts2 + volts3;
  allVolts = constrain(allVolts, 0.0, maxVolts);
  Serial.println(allVolts);

  //Add power
  power += allVolts;
   
  //Calculate resistance
  double smoothedPower = getSmoothedVolt(power);
  double thisResist = resistance;
  
  //Determine direction
  if (power < smoothedPower * 0.9) {
    up = false;
  }
  if (power > smoothedPower) {
    up = true; 
  }
  
  //If decaying multiply resistance
  if (!up) {
    thisResist *= decayRate;
  }
  
  //Add resustance
  power -= thisResist; 
  power = constrain(power, 0.0, maxPower);
  
  //Set resistance for next cycle
  resistance = maxVolts * ((1.0/maxPower) * power);
  resistance = constrain(resistance, 0.02, maxVolts - 0.1);
  
  //Calculate brightness
  float fraction = 255.0/maxPower;
  double brightness = smoothedPower * fraction;
  
  //Idle?  
  if (brightness < 1 && idle == false && startMillis > triggerTime + (idleTimeout * 1000)) {
    currentIdleBrightness = brightness;
    //currentIdleIndex = (currentIdleBrightness / idleThreshold) * 70.0;
    currentIdleIndex = -idlePause;
    upIdleDirection = true;
    idle = true;
  }
  
  if (idle == true) {
    if (brightness > currentIdleBrightness + 3) {
      idle = false; 
    } else {
        double doIndex = constrain(currentIdleIndex, 0.0, 70.0);
        //convert 0-360 angle to radian (needed for sin function)
        float rad = DEG_TO_RAD * doIndex;
        //calculate sin of angle as number between 0 and threshold
        brightness = constrain((sin(rad) * idleThreshold), 0, idleThreshold); 
        if (upIdleDirection == true) {
          currentIdleIndex += idleSpeed;
          if (currentIdleIndex >= 70.0) {
            upIdleDirection = false;
          }
        } else {
          currentIdleIndex -= idleSpeed;
          if (currentIdleIndex <= -idlePause) {
            upIdleDirection = true;
          }
        }
    }
  }
  
  //Set brightness
  brightness = constrain(brightness, 0.0, 255.0);
  if (idle == true) {
    currentIdleBrightness = brightness; 
  } else if (brightness > 4) {
    //The device is not quiet, reset the idle timer
    triggerTime = startMillis; 
  } else {
    brightness = 0;
  }
  //Serial.println(brightness);
  analogWrite(lightPin, 255 - brightness);
  
 
 
  
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
