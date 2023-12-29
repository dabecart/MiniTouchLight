#include <Arduino.h>
#include <CapacitiveSensor.h>

#define MAX_ARRAY_COUNT 10
#define SENSING_TIMEOUT 20 // ms
// The value for the HIGH signal may vary on the long term, so the HIGH threshold is updated
// by using the sensorMax value times this define value.
#define HIGH_SENSOR_MULTIPLE 2 

#define CALIBRATION_INTERVAL 1000 // ms
#define CALIBRATION_DURATION 2000 // ms
// If in this period of time the calibration couldn't happen, force a recalibration.
#define FORCE_CALIBRATION_TIME 10000 // ms
#define LONG_CLICK_DURATION 1500 // ms
#define RELEASE_DURATION 400 // ms
#define AUTO_TURN_OFF_TIMEOUT 60000

long lastSwitchTime = 0;
long lastCalibrationTime = 0;
long lastReleasingTime = 0;
bool clicking = false, releasing = false;

#define MAX_LED_MODES 3
#define LED_STATIC  0
#define LED_SINE_FAST 1
#define LED_SINE_SLOW  2
long nextAnimationTime = 0;
uint8_t ledMode = LED_STATIC;
uint8_t animationIndex = 0;
bool ledOn = false;
bool changeModeNow = false;

long sensorValue = 0;
long sensorMax = 0; // Sensor low value (while not pressed).
long sensorHighValue = 1000;

uint8_t sensorIndex = 0;
uint32_t *sensorArray = new uint32_t[MAX_ARRAY_COUNT];

const int ledPin = 0;
CapacitiveSensor capSensor = CapacitiveSensor(4, 2);

long measureSensor(){
  long read = capSensor.capacitiveSensor(5)*1000;
  
  // Reset the sensor (it seems to get rid of some minor noise)
  digitalWrite(4, LOW);

  // Serial.print(">raw:");
  // Serial.println(read);

  if(sensorMax != 0 && read > 10*sensorMax) read = 10*sensorMax; // Reduce noise spikes.
  sensorValue = read; // Maybe it could be pre-filtered a bit more?

  // Store in array.
  sensorArray[sensorIndex++] = sensorValue;

  static bool enoughPoints = false;
  if(enoughPoints){
    sensorIndex %= MAX_ARRAY_COUNT;
  
    // Calculate the average inside the circular buffer.
    double sum = 0;
    int pointer = sensorIndex;

    for (int i = 0; i < MAX_ARRAY_COUNT; ++i) {
        sum += sensorArray[pointer];

        pointer++;
        pointer %= MAX_ARRAY_COUNT;
    }
    sensorValue = sum/MAX_ARRAY_COUNT;
  }else{
    enoughPoints = sensorIndex==MAX_ARRAY_COUNT;
  }
  delay(SENSING_TIMEOUT);
  return sensorValue;
}

void calibrateSensor(){
  // Serial.println("Calibrating...");
  long startTime = millis();
  sensorMax = 0;
  while(millis()-startTime < 5000){
    measureSensor();
    if(sensorValue > sensorMax){
      sensorMax = sensorValue;
    }
  }
  if(sensorMax==0) sensorMax = 1;
  sensorHighValue = HIGH_SENSOR_MULTIPLE*sensorMax;
}

void setup() {
  // Serial.begin(9600);
  digitalWrite(ledPin, LOW);
  pinMode(ledPin, OUTPUT);

  for(int i = 0; i < MAX_ARRAY_COUNT; i++) sensorArray[i] = -1;

  calibrateSensor();

  // Show mini animation that everything is ready.
  for(uint8_t i = 0; i < 3; i++){
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
  digitalWrite(ledPin, HIGH);
  delay(600);
  digitalWrite(ledPin, LOW);
}

void loop() {
  measureSensor();
  long timeNow = millis();

  // Serial.print(">sensor:");
  // Serial.println(sensorValue);

  // Serial.print(">max:");
  // Serial.println(sensorMax);

  // Serial.print(">status:");
  // Serial.println(ledOn);

  // Serial.print(sensorValue);
  // Serial.print(',');
  // Serial.print(sensorMax);
  // Serial.print(',');
  // Serial.print(sensorSlope);
  // Serial.print(',');
  // Serial.println(digitalRead(ledPin));

  static bool calibrating = false;
  static long newSensorMax = 0;
  if(calibrating || (timeNow - lastCalibrationTime)>=FORCE_CALIBRATION_TIME){
    if(sensorValue > newSensorMax){
      newSensorMax = sensorValue;
    }
    
    if((timeNow-lastCalibrationTime) > CALIBRATION_DURATION){
      if(newSensorMax > 1.2*sensorMax){
        sensorMax = 1.2*sensorMax;
      }else{
        sensorMax = newSensorMax;
      }
      if(sensorMax==0) sensorMax = 1;
      sensorHighValue = HIGH_SENSOR_MULTIPLE*sensorMax;
    
      calibrating = false;
      lastCalibrationTime = timeNow;
      // Serial.println("Calibrated");
    }
  }

  // If the button is being clicked...
  if(clicking){
    // When keeping pressed, it disables the calibration process.
    calibrating = false;
    
    // When the sensor value crosses the low threshold (sensorMax) for a certain time, it may be considered that
    // the button has been released. As the input is really noisy, it has to detect that threshold crossing for a 
    // fet milliseconds.
    if(sensorValue < sensorHighValue){
      if(releasing){
        if((timeNow-lastReleasingTime) >= RELEASE_DURATION){
          clicking = false;
          releasing = false;
          lastSwitchTime = timeNow;
          // Serial.println("Released");
        }
      }else{  // Starting the releasing...
        // Serial.println("Releasing...");
        releasing = true;
        lastReleasingTime = timeNow;
      }
    }else{ // A higher value has been detected, so must be a noisy signal what it had previously detected.
      releasing = false;
    }
  }

  // When changing modes it plays a little animation to signal the change :).
  if(changeModeNow){
    // To signal mode change.
    if(timeNow > nextAnimationTime){
      switch (animationIndex) {
      case 0:
        digitalWrite(ledPin, LOW);
        nextAnimationTime = timeNow + 700;
        break;

      case 1:
        digitalWrite(ledPin, HIGH);
        nextAnimationTime = timeNow + 700;
        break;
      
      case 2:
        digitalWrite(ledPin, LOW);
        nextAnimationTime = timeNow + 700;
        break;

      case 3:
        digitalWrite(ledPin, HIGH);
        nextAnimationTime = timeNow + 1500;
        break;

      case 4:{
        ledMode++;
        ledMode %= MAX_LED_MODES;
        // Serial.print("New mode: ");
        // Serial.println(ledMode);

        changeModeNow = false;

        // Long press should not turn off the device.
        ledOn = true;
      }
      }
      animationIndex++;
    }
  }

  if((timeNow-lastSwitchTime) >= LONG_CLICK_DURATION) {
    if(sensorValue > sensorHighValue){
      if(clicking){
        // Serial.println("Long click!");
        changeModeNow = true;
        nextAnimationTime = timeNow;
        animationIndex = 0;
      }else{
        // Serial.println("Click"); // Initial click
        ledOn = !ledOn;
        digitalWrite(ledPin, ledOn);

        clicking = true;
      }

      lastSwitchTime = timeNow;

      // Disable the calibration while pushing.
      calibrating = false;
    }
  }
  
  if(ledOn && (timeNow-lastSwitchTime)>=AUTO_TURN_OFF_TIMEOUT){
    // Serial.println("Off");
    ledOn = false;
    digitalWrite(ledPin, LOW);
  }
  
  if(!calibrating && (timeNow-lastCalibrationTime)>=CALIBRATION_INTERVAL){
    calibrating = true;
    // Restart substitutes.
    newSensorMax = 0;
    // Serial.println("Calibrating...");
    lastCalibrationTime = timeNow;
  }

  if(!ledOn || changeModeNow) return;

  switch (ledMode){
  case LED_SINE_FAST:{
    uint8_t output = 127*(1.0+sin(TWO_PI*timeNow/2000.0));
    analogWrite(ledPin, output);
    // Serial.print(">sin:");
    // Serial.println(output);
    break;
  }
  
  case LED_SINE_SLOW:{
    uint8_t output = 127*(1.0+sin(TWO_PI*timeNow/10000.0));
    analogWrite(ledPin, output);
    break;
  }

  case LED_STATIC:
  default:
    digitalWrite(ledPin, HIGH);
  break;
  }

}