#include "VOneMqttClient.h"
#include <ESP32Servo.h> 
#include "DHT.h"
#define DHTTYPE DHT11

// Define device ID
const char* InfraredSensor = "7d7db9fb-e622-4d13-960b-9de71873da24";  //Replace with the deviceID of YOUR infrared sensor
const char* ServoMotor = "c565a9ed-57f5-4032-bda9-01274a7b8545";      //Replace this with YOUR deviceID for the servo
const char* MQ2sensor = "b17689a8-6f25-4c8d-8888-552203a528e7";       //Replace this with YOUR deviceID for the MQ2 sensor
const char* DHT11Sensor = "ad2efb66-75ef-48aa-b17c-3fe4d725d47c";     //Replace this with YOUR deviceID for the DHT11 sensor
const char* DigitalInput = "69125c80-e8b8-42c2-81b8-718b86ad71de";    //Replace with the deviceID of YOUR push button
const char* LEDLight1 = "05718b95-249e-410c-9adc-aae478125d7e";  //Replace this with YOUR deviceID for the first LED
const char* LEDLight2 = "cf0292bc-84b9-45e3-b1b2-f9c3394273c6";  //Replace this with YOUR deviceID for the second LED
const char* LEDLight3 = "cae948a5-d7d8-46b8-a2e5-4ba5ed29b184";  //Replace this with YOUR deviceID for the third LED

const int irPin = 4;          // IR sensor (Maker: Pin 4, Left Maker Port) 
const int ledPinG = 5;        // Green LED (Maker: Pin 5)
const int ledPinY = 9;        // Yellow LED (Maker: Pin 9) 
const int ledPinR = 10;       // Red LED (Maker: Pin 10) 
const int servoPin = 39;      // Servo (Maker: Pin 39)
const int buttonPin = 40;     // Push button (Maker: Pin 40)
const int dht11Pin = 41;      // DHT11 sensor (Maker: Pin 41, Right Maker Port)
const int mqPin = A2;         // MH MQ Sensor (Middle Maker Port) 

bool irDetected = false;      // Tracks IR sensor state
bool buttonPressed = false;   // Tracks button state
float airQuality = 0;           // Air quality reading
float temperature = 0.0;      // Temperature value
float humidity = 0.0;         // Humidity value

Servo myServo;  // Create a Servo object
DHT dht(dht11Pin, DHTTYPE);


// Create an insance of VOneMqttClient
VOneMqttClient voneClient;

// Last message time
unsigned long lastMsgTime = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// void setup() {   
//   //Serial.begin(9600); // Initiate serial communication    
  
//   // Set pin mode   
//   pinMode(irPin, INPUT);   
//   pinMode(ledPinG, OUTPUT);
//   pinMode(ledPinY, OUTPUT);
//   pinMode(ledPinR, OUTPUT); 
//   pinMode(buttonPin, INPUT_PULLUP);
//   pinMode(mqPin, INPUT);

//   myServo.attach(servoPin); // Attach the servo to the servo pin
  
//   // Set initial states
//   digitalWrite(ledPinG, LOW);
//   digitalWrite(ledPinY, LOW);
//   digitalWrite(ledPinR, LOW);
//   myServo.write(0);  // Initialize servo to idle position (0 degrees)

//   Serial.println("System initialized...");
//   Serial.println("Gas sensor warming up!");
//   delay(20000);  // allow the MQ-2 to warm up

//   dht.begin();  // Initialize DHT sensor
//   setup_wifi();
//   voneClient.setup();
// } 

// void loop() { 
//   // Reconnect if disconnected
//   if (!voneClient.connected()) {
//     voneClient.reconnect();
//   }
//   voneClient.loop();

//   // Read sensor values
//   irDetected = (digitalRead(irPin) == LOW);  // LOW means object detected
//   buttonPressed = (digitalRead(buttonPin) == LOW);  // LOW means button pressed
//   airQuality = analogRead(mqPin);  // Read air quality value
//   temperature = dht.readTemperature();  // Read temperature
//   humidity = dht.readHumidity();       // Read humidity

//   // Check whether the given floating-point number argument is a NaN value   
//   if (isnan(temperature) || isnan(humidity)) {     
//     //Serial.println("Failed to read from the DHT sensor");     
//     return;   
//   } 

//   Serial.print("Temperature: ");
//   Serial.print(temperature);
//   Serial.print(" °C, Humidity: ");
//   Serial.print(humidity);
//   Serial.println(" %");
//   Serial.print("Air Quality Value: ");
//   Serial.println(airQuality);

//   // Publish data to V-One
//   unsigned long currentTime = millis();
//   if (currentTime - lastMsgTime > INTERVAL) {
//     lastMsgTime = currentTime;

//     // Publish gas sensor data
//     voneClient.publishTelemetryData(MQ2sensor, "Gas detector", airQuality);

//     // Publish temperature and humidity data
//     JSONVar dhtPayload;
//     dhtPayload["Humidity"] = humidity;
//     dhtPayload["Temperature"] = temperature;
//     voneClient.publishTelemetryData(DHT11Sensor, dhtPayload);

//     // Publish IR sensor state
//     voneClient.publishTelemetryData(InfraredSensor, "IR Detected", irDetected);

//     // Publish button state
//     voneClient.publishTelemetryData(DigitalInput, "Button Pressed", buttonPressed);
//   }

//   // LED Control for Air Pollution Levels
//   if (airQuality < 1400) { // Low pollution
//     digitalWrite(ledPinG, HIGH);  // Green light
//     digitalWrite(ledPinY, LOW);
//     digitalWrite(ledPinR, LOW);
//     Serial.println("Air Quality: Clean (Low Pollution)");
//   } else if (airQuality >= 1400 && airQuality < 2500) { // Moderate pollution
//     digitalWrite(ledPinG, LOW);
//     digitalWrite(ledPinY, HIGH);  // Yellow light
//     digitalWrite(ledPinR, LOW);
//     Serial.println("Air Quality: Moderate Pollution");
//   } else {  // High pollution
//     digitalWrite(ledPinG, LOW);
//     digitalWrite(ledPinY, LOW);
//     digitalWrite(ledPinR, HIGH);  // Red light
//     Serial.println("Air Quality: High Pollution");
//   }
  
//   // Servo Control (Mask Dispenser)
//   // If moderate/high polution, detected object (human) & button pressed
//   if ((airQuality >= 1400) && (irDetected && buttonPressed)) {  
//     myServo.write(90);            // Move servo to active position
//     Serial.println("Dispensing mask...");
//     delay(1000);                  // Hold position for 1 second
//   } else {
//     myServo.write(0);            // Move servo to idle position
//   }
  
//   delay(5000);
// } 

void triggerActuator_callback(const char* actuatorDeviceId, const char* actuatorCommand) {
  Serial.print("Received callback: ");
  Serial.print(actuatorDeviceId);
  Serial.print(" : ");
  Serial.println(actuatorCommand);

  String errorMsg = "";
  JSONVar commandObject = JSON.parse(actuatorCommand);
  JSONVar keys = commandObject.keys();

  if (String(actuatorDeviceId) == ServoMotor) {
    String key = "";
    JSONVar commandValue = "";
    for (int i = 0; i < keys.length(); i++) {
      key = (const char*)keys[i];
      commandValue = commandObject[keys[i]];
    }
    Serial.print("Key : ");
    Serial.println(key.c_str());
    Serial.print("value : ");
    Serial.println(commandValue);

    int angle = (int)commandValue;
    myServo.write(angle);
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), true);  //publish actuator status
  } else {
    Serial.print(" No actuator found : ");
    Serial.println(actuatorDeviceId);
    errorMsg = "No actuator found";
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), false);  //publish actuator status
  }
  
  // if (String(actuatorDeviceId) == LEDLight1 || String(actuatorDeviceId) == LEDLight2 || String(actuatorDeviceId) == LEDLight3) {
  //   bool commandValue = commandObject["state"];
  //   int pin = (String(actuatorDeviceId) == LEDLight1) ? ledPinG : (String(actuatorDeviceId) == LEDLight2) ? ledPinY : ledPinR;
  //   digitalWrite(pin, commandValue);
  //   voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), true);
  // } else {
  //   errorMsg = "No matching actuator found.";
  //   voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), false);
  // }
}

void setup() {
  //Serial.begin(115200);
  setup_wifi();
  voneClient.setup();
  voneClient.registerActuatorCallback(triggerActuator_callback);
  Serial.println("Gas sensor warming up!");
  delay(20000);  // allow the MQ-2 to warm up

  pinMode(irPin, INPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinY, OUTPUT);
  pinMode(ledPinR, OUTPUT);
  pinMode(buttonPin, INPUT);

  dht.begin();
  myServo.attach(servoPin);
  myServo.write(0);
}

void loop() {
  if (airQuality < 1400) { // Low pollution
    digitalWrite(ledPinG, HIGH);  // Green light
    digitalWrite(ledPinY, LOW);
    digitalWrite(ledPinR, LOW);
    //Serial.println("Air Quality: Clean (Low Pollution)");
  } else if (airQuality >= 1400 && airQuality < 2500) { // Moderate pollution
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinY, HIGH);  // Yellow light
    digitalWrite(ledPinR, LOW);
    //Serial.println("Air Quality: Moderate Pollution");
  } else {  // High pollution
    digitalWrite(ledPinG, LOW);
    digitalWrite(ledPinY, LOW);
    digitalWrite(ledPinR, HIGH);  // Red light
    //Serial.println("Air Quality: High Pollution");
  }
  
  if (!voneClient.connected()) {
    voneClient.reconnect();
    String errorMsg = "Sensor Fail";
    voneClient.publishDeviceStatusEvent(InfraredSensor, true);
    voneClient.publishDeviceStatusEvent(MQ2sensor, true);
    voneClient.publishDeviceStatusEvent(DHT11Sensor, true);
    voneClient.publishDeviceStatusEvent(DigitalInput, true);
  }
  voneClient.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastMsgTime > INTERVAL) {
    lastMsgTime = currentMillis;

    //Publish MQ2 data
    airQuality = analogRead(mqPin);
    voneClient.publishTelemetryData(MQ2sensor, "Gas detector", airQuality);

    //Publish DHT data
    float humidity = dht.readHumidity();
    int temperature = dht.readTemperature();

    JSONVar payloadObject;
    payloadObject["Humidity"] = humidity;
    payloadObject["Temperature"] = temperature;
    voneClient.publishTelemetryData(DHT11Sensor, payloadObject);

    //Publish Infrared data
    int InfraredVal = !digitalRead(irPin);
    voneClient.publishTelemetryData(InfraredSensor, "Obstacle", InfraredVal);

    //Publish Button data
    int buttonPressed = digitalRead(buttonPin) == LOW ? 1 : 0;  //***BUGS!!!!!!!!!!!!!
    voneClient.publishTelemetryData(DigitalInput, "Button1", buttonPressed);
  }
}



