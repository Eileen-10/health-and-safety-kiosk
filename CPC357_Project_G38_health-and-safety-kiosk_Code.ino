#include <ESP32Servo.h> 
#include "DHT.h"
#define DHTTYPE DHT11

const int irPin = 4;          // IR sensor (Maker: Pin 4, Left Maker Port) 
const int ledPinG = 5;         // Green LED (Maker: Pin 5)
const int ledPinY = 9;        // Yellow LED (Maker: Pin 9) 
const int ledPinR = 10;       // Red LED (Maker: Pin 10) 
const int servoPin = 39;      // Servo (Maker: Pin 39)
const int buttonPin = 40;     // Push button (Maker: Pin 40)
const int dht11Pin = 41;      // DHT11 sensor (Maker: Pin 41, Right Maker Port)
const int mqPin = A2;         // MH MQ Sensor (Middle Maker Port) 

bool irDetected = false;      // Tracks IR sensor state
bool buttonPressed = false;   // Tracks button state
int airQuality = 0;           // Air quality reading
float temperature = 0.0;     // Temperature value
float humidity = 0.0;        // Humidity value

Servo myServo;  // Create a Servo object
DHT dht(dht11Pin, DHTTYPE);

void setup() {   
  Serial.begin(9600); // Initiate serial communication    
  
  // Set pin mode   
  pinMode(irPin, INPUT);   
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinY, OUTPUT);
  pinMode(ledPinR, OUTPUT); 
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(mqPin, INPUT);

  myServo.attach(servoPin); // Attach the servo to the servo pin
  
  // Set initial states
  digitalWrite(ledPinG, LOW);
  digitalWrite(ledPinY, LOW);
  digitalWrite(ledPinR, LOW);
  myServo.write(0);  // Initialize servo to idle position (0 degrees)

  Serial.println("System initialized...");
  Serial.println("Gas sensor warming up!");
  delay(20000);  // allow the MQ-2 to warm up

  dht.begin();  // Initialize DHT sensor
} 

void loop() { 
  // Read sensor values
  irDetected = (digitalRead(irPin) == LOW);  // LOW means object detected
  buttonPressed = (digitalRead(buttonPin) == LOW);  // LOW means button pressed
  airQuality = analogRead(mqPin);  // Read air quality value
  
  delay(5000);
  temperature = dht.readTemperature();  // Read temperature
  humidity = dht.readHumidity();       // Read humidity

  // Check whether the given floating-point number argument is a NaN value   
  if (isnan(temperature) || isnan(humidity)) {     
    Serial.println("Failed to read from the DHT sensor");     
    return;   
  } 

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("Air Quality Value: ");
  Serial.println(airQuality);

  // LED Control for Air Pollution Levels
  if (airQuality < 1400) { // Low pollution
    digitalWrite(ledPinG, HIGH);  // Green light
    digitalWrite(ledPinY, LOW);
    digitalWrite(ledPinR, LOW);
    Serial.println("Air Quality: Clean (Low Pollution)");
  } else if (airQuality >= 1400 && airQuality < 2500) { // Moderate pollution
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinY, HIGH);  // Yellow light
    digitalWrite(ledPinR, LOW);
    Serial.println("Air Quality: Moderate Pollution");
  } else {  // High pollution
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinY, LOW);
    digitalWrite(ledPinR, HIGH);  // Red light
    Serial.println("Air Quality: High Pollution");
  }
  
  // Servo Control (Mask Dispenser)
  // If moderate/high polution, detected object (human) & button pressed
  if ((airQuality >= 1400) && (irDetected && buttonPressed)) {  
    myServo.write(90);            // Move servo to active position
    Serial.println("Dispensing mask...");
    delay(1000);                  // Hold position for 1 second
  } else {
    myServo.write(0);            // Move servo to idle position
  }
  
  delay(2000);
} 


