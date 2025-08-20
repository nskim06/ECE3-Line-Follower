#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, indices 0..7

// pin initialization
const int left_nslp_pin  = 31; 
const int right_nslp_pin = 11;

const int left_dir_pin   = 29;
const int right_dir_pin  = 30;

const int left_pwm_pin   = 40;
const int right_pwm_pin  = 39;

const int LED_RF         = 41;

//calibrations
const int   minArr[8]    = { 432, 367, 322, 390, 482, 390, 413, 459 };
const float maxArr[8]    = {2067,1901,1983,1445,2018,1915,1897,2041};
int baseWeights[8]       = {-15, -14, -12, -8,  8, 12, 14, 15};
int leftBiasWeights[8]   = {-15, -14, -12, -8, 33, 37, 39, 40};
int rightBiasWeights[8]  = {-24, -23, -21, -17, 8, 12, 14, 15};

// PID and error variables
float errorVal       = 0.0;
float errorValPrev   = 0.0;
float Kp             = 0.045;
float Kd             = 0.3;

// Motor speeds
int baseSpd          = 35;
int leftSpd          = baseSpd;
int rightSpd         = baseSpd;

// Other settings
bool turnedAround = false;

///////////////////////////////////
void setup() {
    pinMode(left_nslp_pin, OUTPUT);
    pinMode(left_dir_pin,   OUTPUT);
    pinMode(left_pwm_pin,   OUTPUT);

    pinMode(right_nslp_pin, OUTPUT);
    pinMode(right_dir_pin,  OUTPUT);
    pinMode(right_pwm_pin,  OUTPUT);

    // Start both drivers “awake” with forward direction
    digitalWrite(left_dir_pin,  LOW);
    digitalWrite(left_nslp_pin,  HIGH);
    digitalWrite(right_dir_pin, LOW);
    digitalWrite(right_nslp_pin, HIGH);

    pinMode(LED_RF, OUTPUT);

    ECE3_Init();
    Serial.begin(9600);
    delay(2000);
}

bool isWideSplit(uint16_t sensorValues[]) {
    // Look for any i, j with j >= i+2 where both > 1000
    for (int i = 0; i < 8; i++) {
        if (sensorValues[i] <= 1000) continue;
        // Only check j starting at i+2 (non-consecutive)
        for (int j = i + 2; j < 8; j++) {
            if (sensorValues[j] > 1000) {
                return true;
            }
        }
    }
    return false;
}

void loop() {

  // put your main code here, to run repeatedly: 

  // read raw sensor values
  ECE3_read_IR(sensorValues);


  // implement crosspiece checking below

  static bool prevCrosspiece = false; // track previous state
  static int crosspieceCount = 0;     // count successive detections
  bool currentCrosspiece = true;
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] != 2500) {
      currentCrosspiece = false;
      break;
    }
  }

  if (currentCrosspiece) {
    if (prevCrosspiece) 
    {
      crosspieceCount++;
      if (crosspieceCount >= 2 && turnedAround) 
      {
        digitalWrite(left_nslp_pin,  LOW);
        digitalWrite(right_nslp_pin, LOW);
        return;
      }
      else if (crosspieceCount >= 2 && !turnedAround)
      {
        digitalWrite(left_dir_pin,  LOW);   
        analogWrite(left_pwm_pin,   60);    
        digitalWrite(right_dir_pin, HIGH); 
        analogWrite(right_pwm_pin,  60);

        delay(800);  // ←roughly 180 degree turn

        // Stop pivoting and restore “both forward”
        analogWrite(left_pwm_pin,   0);
        analogWrite(right_pwm_pin,  0);
        digitalWrite(left_dir_pin,  LOW);
        digitalWrite(right_dir_pin, LOW);

        // Reset any split/crosspiece state so the second lap works cleanly
        errorValPrev = 0;        // zero out D-term so you don’t get a big jump
        leftSpd = baseSpd;
        rightSpd = baseSpd;
        turnedAround = true;

        return;  // skip PD for this iteration; next loop will re-read sensors
      }
    } 
    else
    {
        crosspieceCount = 1; //start count
    }
    prevCrosspiece = true;
  } 
  else 
  {
    crosspieceCount = 0;
    prevCrosspiece = false;
  }

  //-------------------------------------//
  //split detection

  bool wideSplit = isWideSplit(sensorValues);

   int weights[8];

  if (wideSplit) {
    if (!turnedAround) {
      // 第一次走左边
      for (int i = 0; i < 8; i++)
      {
        weights[i] = leftBiasWeights[i];
      }
    } 
    else if (turnedAround)
    {
      // 第二次走右边
      for (int i = 0; i < 8; i++)
      {
        weights[i] = rightBiasWeights[i];
      }

    }
  }
  else
  {
    for (int i = 0; i < 8; i++)
    {
      weights[i] = baseWeights[i];
    }
  }

  // ---------------------------//
  // calculating error value //

  float errArr[8] = {0.0};
  float finalValue = 0.0;
  int temp;

  for (int i = 0; i < 8; i++)
  {
    //Serial.println(sensorValues[i]);
    temp = sensorValues[i] - minArr[i];
    if (temp < 0)
      errArr[i] = 0;
    else
      errArr[i] = temp;

    //Serial.println(temp[i]);

    errArr[i] = errArr[i]*(1000.0/maxArr[i]);
    finalValue += errArr[i] * weights[i];

  }

  errorVal = finalValue / 8;

  //----------------------------//
  if (errorVal < -1900)
  {
      
      digitalWrite(right_dir_pin, HIGH);
      analogWrite(right_pwm_pin, 80);

      digitalWrite(left_dir_pin, LOW);
      analogWrite(left_pwm_pin, 80);

      errorValPrev = errorVal;
      return;
  }
  else if (errorVal > 1900)
  {
      digitalWrite(left_dir_pin, HIGH);
      analogWrite(left_pwm_pin, 80);
      
      digitalWrite(right_dir_pin, LOW);
      analogWrite(right_pwm_pin, 80);

      errorValPrev = errorVal;
      return;
  }
  else
  {
      digitalWrite(right_dir_pin, LOW);
      digitalWrite(left_dir_pin, LOW);
  }

  //------------------------------------//

  int P = -Kp * errorVal; 
  int D = -Kd * (errorVal - errorValPrev);
  int speedChange = P + D;
  
  leftSpd = constrain(baseSpd + speedChange, 0, 60);
  rightSpd = constrain(baseSpd - speedChange, 0, 60);
                                                                                                                                                                                                                                                                                                                                                                                                                
  analogWrite(left_pwm_pin, leftSpd);
  analogWrite(right_pwm_pin, rightSpd);


  errorValPrev = errorVal;

  //----------------------------------------------//
} 
