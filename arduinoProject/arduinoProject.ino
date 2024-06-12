




/*
// Project 1: Blinking LED

void setup() {
  pinMode(6, OUTPUT);
}

void loop() {
  digitalWrite(6, HIGH);
  delay(1000);
  digitalWrite(6, LOW);
  delay(1000);
}



// Project 2: Button Press

const int buttonPin = 2;
const int ledPin = 6;

int buttonState = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
}

void loop() {
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}



// Project 3: Potentiometer

const int potPin = A0;
const int ledPin = 5;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  int potValue = analogRead(potPin);
  int brightness = map(potValue, 0, 1023, 0, 255);

  analogWrite(ledPin, brightness);
}


// Project 4: Multiple LEDs


const int ledPins[] = {2, 3, 4, 5, 6, 7};

void setup() {
  for (int i = 0; i < 6; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}

void loop() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(ledPins[i], HIGH);
    delay(500);
    digitalWrite(ledPins[i], LOW);
  }
}

// Project 5: Servo Motor Control

#include <Servo.h>

Servo myservo;

const int potPin = A0;

void setup() {
  myservo.attach(9);
}

void loop() {
  int potValue = analogRead(potPin);
  int angle = map(potValue, 0, 1023, 0, 180);

  myservo.write(angle);
  delay(15);
}

// Project 6: RGB LED

const int redPin = 3;
const int greenPin = 5;
const int bluePin = 6;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  setColor(255, 0, 0);
  delay(1000);
  setColor(0, 255, 0);
  delay(1000);
  setColor(0, 0, 255);
  delay(1000);
}

void setColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}



// Project 7: Temperature Sensor

const int sensorPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(sensorPin);
  float voltage = sensorValue * 3.3 / 1024.0;
  float temperatureC = (voltage - 0.5) * 100.0;

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" C");

  delay(1000);
}


// Project 8: Digital Thermometer with LCD



#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>

const int sensorPin = A0;
Adafruit_LiquidCrystal lcd(0);

void setup() {
  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  lcd.print("Temp:");

  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(sensorPin);
  float voltage = sensorValue * 3.3 / 1024.0;
  float temperatureC = (voltage - 0.5) * 100.0;

  lcd.setCursor(0, 1);
  lcd.print(temperatureC);
  lcd.print(" C");

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" C");

  delay(1000);
}

// Project 11: Crystal Ball

#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>

Adafruit_LiquidCrystal lcd(7, 4, 5, 1, 3, 0);

const int switchPin = 6; 
int switchState = 0;     
int prevSwitchState = 0; 
int reply;               

void setup() {
  pinMode(switchPin, INPUT); 

  lcd.begin(16, 2);           
  lcd.setBacklight(HIGH);     
  lcd.print("Ask the");       
  lcd.setCursor(0, 1);
  lcd.print("Crystal Ball!");
  
  Serial.begin(9600);         
}

void loop() {
  switchState = digitalRead(switchPin); 

  if (switchState != prevSwitchState) { 
    if (switchState == LOW) {           
      reply = random(8);               

      lcd.clear();                      
      lcd.setCursor(0, 0);              
      lcd.print("The ball says:");

      lcd.setCursor(0, 1);              
      switch (reply) {                  
        case 0:
          lcd.print("Yes");
          Serial.println("The answer is Yes!");
          break;
        case 1:
          lcd.print("Most likely");
          Serial.println("The answer is Most likely!");
          break;
        case 2:
          lcd.print("Certainly");
          Serial.println("The answer is Certainly!");
          break;
        case 3:
          lcd.print("Outlook good");
          Serial.println("The answer is Outlook good!");
          break;
        case 4:
          lcd.print("Unsure");
          Serial.println("The answer is Unsure!");
          break;
        case 5:
          lcd.print("Ask again");
          Serial.println("The answer is Ask again!");
          break;
        case 6:
          lcd.print("Doubtful");
          Serial.println("The answer is Doubtful!");
          break;
        case 7:
          lcd.print("No");
          Serial.println("The answer is No!");
          break;
      }
    }
  }
  prevSwitchState = switchState;  
}

*/


#include <Adafruit_ZeroTimer.h>
#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>
#include <math.h>


// Timer configuration
Adafruit_ZeroTimer timer = Adafruit_ZeroTimer(3);

// Timetick variable
volatile uint32_t timetick = 0;

// LED pin
const int ledPin = 6; // Onboard LED

// DAC configuration
const int dacPin = A0; // AOUT pin

// Sinewave parameters
const int sineTableSize = 1024;
float sineTable[sineTableSize];

// Function to increment timetick
void TC3_Handler(void) {
  TcCount16* TC = (TcCount16*) TC3;
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    timetick++;
  }
}


void setup() {

  // Initialize serial for debugging
  Serial.begin(9600);

  // Initialize LED pin
  pinMode(ledPin, OUTPUT);

  // Initialize DAC pin
  analogWriteResolution(10); // Set resolution to 10-bit

  for (int i = 0; i < sineTableSize; i++) {
    sineTable[i] = (sin(2 * PI * i / sineTableSize) + 1) * 512; // Scale to 0-1023 range
  }

  // Set up timer for 1 ms tick
  timer.configure(TC_CLOCK_PRESCALER_DIV256, TC_COUNTER_SIZE_16BIT, TC_WAVE_GENERATION_MATCH_FREQ);
  timer.setCompare(0, (SystemCoreClock / 256) / 1000); // 1 ms tick
  timer.enable(true);
  timer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, TC3_Handler);
}

void loop() {
  static uint32_t lastTick = 0;
  static int sineIndex = 0;

  // Flash LED every 500 ms
  if (timetick - lastTick >= 500) {
    digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED
    lastTick = timetick;
  }

  // Output sinewave at 50 Hz
  analogWrite(dacPin, (int)sineTable[sineIndex]);
  sineIndex = (sineIndex + 1) % sineTableSize;
  delay(1000 / 50 / sineTableSize); // Delay for 50 Hz sine wave generation
}



