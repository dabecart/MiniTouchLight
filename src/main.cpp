#include <Arduino.h>
#include <CapacitiveSensor.h>

#define MAX_ARRAY_COUNT 10
#define SENSING_TIMEOUT 20 // ms
#define SLOPE_ADD_POINT_TIMEOUT 1 // * SENSING_TIMEOUT

#define CALIBRATION_INTERVAL 5000 // ms
#define CALIBRATION_DURATION 2000 // ms

#define AUTO_TURN_OFF_TIMEOUT 60000

#define LOWER_SENSOR_MAX_SLOPE 100

CapacitiveSensor capSensor = CapacitiveSensor(4, 2);

const int ledPin = 9;

long lastSwitch = 0;
long lastCalibration = 0;
bool clicking = false;

bool ledOn = false;
#define MAX_LED_MODES 3
typedef enum{
  STATIC = 0, SINE_FAST = 1, SINE_SLOW = 2
}LedMode;
LedMode ledMode = STATIC;
bool changeModeNow = false;

long sensorValue = 0;
long sensorMax = 0, sensorMaxSlope = 0;
double alpha = 0.005;

double sensorSlope = 0;
uint8_t sensorIndex = 0;
uint32_t sensorArray[MAX_ARRAY_COUNT];

long measureSensor(){
  // Counts the numbers of times this function has been called. When it reaches the threshold 
  // SLOPE_ADD_POINT_TIMEOUT it calculates the slope of the curve.
  static uint8_t inputToArrayCount = 0;
  long read = capSensor.capacitiveSensor(5)*1000;
  
  // Reset the sensor (it seems to get rid of some minor noise)
  digitalWrite(4, LOW);

  Serial.print(">raw:");
  Serial.println(read);

  if(sensorMax != 0 && read > 10*sensorMax) read = 10*sensorMax;
  if(sensorValue == 0) sensorValue = read;
  // LP Filter. 
  sensorValue = read*alpha + sensorValue*(1.0-alpha);
  // sensorValue = read; // TEMPORARY!

  inputToArrayCount++;
  if(inputToArrayCount == SLOPE_ADD_POINT_TIMEOUT){
    inputToArrayCount = 0;
    sensorArray[sensorIndex++] = sensorValue;

    static bool enoughPoints = false;
    if(enoughPoints){
      sensorIndex %= MAX_ARRAY_COUNT;
    
      // Calculate regression slope.
      double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x_squared = 0;
      int pointer = sensorIndex;

      for (int i = 0; i < MAX_ARRAY_COUNT; ++i) {
          double x = i*SENSING_TIMEOUT*SLOPE_ADD_POINT_TIMEOUT/1000.0;
          sum_x += x;
          sum_y += sensorArray[pointer];
          sum_xy += x * sensorArray[pointer];
          sum_x_squared += x * x;

          pointer++;
          pointer %= MAX_ARRAY_COUNT;
      }

      sensorSlope = (MAX_ARRAY_COUNT * sum_xy - sum_x * sum_y) / (MAX_ARRAY_COUNT * sum_x_squared - sum_x * sum_x);
      // sensorValue = sum_y/MAX_ARRAY_COUNT;
    }else{
      enoughPoints = sensorIndex==MAX_ARRAY_COUNT;
    }

  }
  delay(SENSING_TIMEOUT);
  return sensorValue;
}

void calibrateSensor(){
  Serial.println("Calibrating...");
  long startTime = millis();
  sensorMax = 0;
  while(millis()-startTime < 5000){
    measureSensor();
    if(sensorValue > sensorMax){
      sensorMax = sensorValue;
    }
    if(sensorSlope > sensorMaxSlope){
      sensorMaxSlope = sensorSlope;
    }
    delay(20); // 20 more to compensate
  }

  if(sensorMaxSlope < LOWER_SENSOR_MAX_SLOPE){
    sensorMaxSlope = LOWER_SENSOR_MAX_SLOPE;
  }
}

void (* resetFunc) (void) = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);

  for(int i = 0; i < MAX_ARRAY_COUNT; i++) sensorArray[i] = -1;

  calibrateSensor();
}

void loop() {
  measureSensor();
  long timeNow = millis();

  Serial.print(">sensor:");
  Serial.println(sensorValue);

  Serial.print(">max:");
  Serial.println(sensorMax);

  Serial.print(">max_slope:");
  Serial.println(sensorMaxSlope);

  Serial.print(">slope:");
  Serial.println(sensorSlope);
  
  Serial.print(">status:");
  Serial.println(ledOn);

  // Serial.print(sensorValue);
  // Serial.print(',');
  // Serial.print(sensorMax);
  // Serial.print(',');
  // Serial.print(sensorSlope);
  // Serial.print(',');
  // Serial.println(digitalRead(ledPin));

  static bool calibrating = false;
  static long newSensorMax = 0, newSensorMaxSlope = 0;
  if(calibrating){
    if(sensorValue > newSensorMax){
      newSensorMax = sensorValue;
    }
    if(sensorSlope > newSensorMaxSlope){
      newSensorMaxSlope = sensorSlope;
    }
    
    if((timeNow-lastCalibration) > CALIBRATION_DURATION){
      if(newSensorMax > 1.2*sensorMax){
        sensorMax = 1.2*sensorMax;
      }else{
        sensorMax = newSensorMax;
      }

      if(newSensorMaxSlope > 4*sensorMaxSlope){
        sensorMaxSlope = 4*sensorMaxSlope;
      }else{
        sensorMaxSlope = newSensorMaxSlope;
      }
      
      if(sensorMaxSlope < LOWER_SENSOR_MAX_SLOPE){
        sensorMaxSlope = LOWER_SENSOR_MAX_SLOPE;
      }

      calibrating = false;
      Serial.println("Calibrated");
    }
  }

  // When pressed, the slope tends to be this big!
  if(sensorSlope > sensorMaxSlope*4 || sensorSlope < -sensorMaxSlope*4){ // TODO
    calibrating = false;
    lastCalibration = timeNow;
  }

  // When the derivative crosses zero means a peak has been reached (button released).
  if(clicking && sensorSlope<0){
    Serial.println("Released");
    clicking = false;
    lastSwitch = timeNow;
  }

  if(changeModeNow){
    static uint8_t animationIndex = 0;
    static long nextAnimationTime = 0;

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
        int pos = (int) ledMode;
        pos++;
        pos %= MAX_LED_MODES;
        ledMode = (LedMode) pos;
        Serial.print("New mode: ");
        Serial.println(pos);

        animationIndex = 0xFF;
        changeModeNow = false;
      }
      }
      animationIndex++;
    }
  }

  if((timeNow-lastSwitch) >= 3000) {
    // Inital click
    if(!clicking && sensorSlope > 6*sensorMaxSlope){
      Serial.println("Click");
      ledOn = !ledOn;
      if(!ledOn) digitalWrite(ledPin, LOW);

      // Disable the calibration while pushing.
      lastCalibration = timeNow+3000;
      calibrating = false;
      lastSwitch = timeNow;
      clicking = true;
    }
    
    //Long click
    if(clicking && sensorValue > 2*sensorMax){
      changeModeNow = true;
      lastSwitch = timeNow;
    }
  }
  
  if(ledOn && (timeNow-lastSwitch)>=AUTO_TURN_OFF_TIMEOUT){
    Serial.println("Off");
    ledOn = false;
    digitalWrite(ledPin, LOW);
  }
  
  if((timeNow-lastCalibration)>=CALIBRATION_INTERVAL){
    calibrating = true;
    // Restart substitutes.
    newSensorMax = 0;
    newSensorMaxSlope = 0;
    Serial.println("Calibrating...");
    lastCalibration = timeNow;
  }

  if(!ledOn || changeModeNow) return;

  switch (ledMode){
  case SINE_FAST:{
    uint8_t output = 127*(1.0+sin(TWO_PI*timeNow/2000.0));
    analogWrite(ledPin, output);
    // Serial.print(">sin:");
    // Serial.println(output);
    break;
  }
  
  case SINE_SLOW:{
    uint8_t output = 127*(1.0+sin(TWO_PI*timeNow/10000.0));
    analogWrite(ledPin, output);
    break;
  }

  case STATIC:
  default:
    digitalWrite(ledPin, HIGH);
  break;
  }

  if(timeNow > 86400000) resetFunc(); // Reset Arduino every day
}