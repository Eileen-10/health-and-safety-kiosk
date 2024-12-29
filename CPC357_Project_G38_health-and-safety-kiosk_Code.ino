#include "VOneMqttClient.h"
#include <ESP32Servo.h> 
#include "DHT.h"
#define DHTTYPE DHT11

// Define device ID
const char* InfraredSensor = "7d7db9fb-e622-4d13-960b-9de71873da24";  //Replace with the deviceID of YOUR infrared sensor
const char* ServoMotor = "c565a9ed-57f5-4032-bda9-01274a7b8545";      //Replace this with YOUR deviceID for the servo
const char* MQ2sensor = "b17689a8-6f25-4c8d-8888-552203a528e7";       //Replace this with YOUR deviceID for the MQ2 sensor
const char* DHT11Sensor = "ad2efb66-75ef-48aa-b17c-3fe4d725d47c";     //Replace this with YOUR deviceID for the DHT11 sensor
const char* LEDLight1 = "05718b95-249e-410c-9adc-aae478125d7e";       //Replace this with YOUR deviceID for the first LED
const char* LEDLight2 = "cf0292bc-84b9-45e3-b1b2-f9c3394273c6";       //Replace this with YOUR deviceID for the second LED
const char* LEDLight3 = "cae948a5-d7d8-46b8-a2e5-4ba5ed29b184";       //Replace this with YOUR deviceID for the third LED

const int irPin = 4;          // IR sensor (Maker: Pin 4, Left Maker Port) 
const int ledPinG = 5;        // Green LED (Maker: Pin 5)
const int ledPinY = 9;        // Yellow LED (Maker: Pin 9) 
const int ledPinR = 10;       // Red LED (Maker: Pin 10) 
const int servoPin = 39;      // Servo (Maker: Pin 39)
const int dht11Pin = 42;      // DHT11 sensor (Maker: Pin 42, Right Maker Port)
const int mqPin = A2;         // MH MQ Sensor (Middle Maker Port) 

bool irDetected = false;      // Tracks IR sensor state
float airQuality = 0;         // Air quality reading
float temperature = 0.0;      // Temperature value
float humidity = 0.0;         // Humidity value

Servo myServo;  // Create a Servo object
DHT dht(dht11Pin, DHTTYPE);

VOneMqttClient voneClient;  // Create an insance of VOneMqttClient
unsigned long lastMsgTime = 0;  // Last message time


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

void triggerActuator_callback(const char* actuatorDeviceId, const char* actuatorCommand) {
  Serial.print("Received callback: ");
  Serial.print(actuatorDeviceId);
  Serial.print(" : ");
  Serial.println(actuatorCommand);

  String errorMsg = "";
  JSONVar commandObject = JSON.parse(actuatorCommand);
  JSONVar keys = commandObject.keys();

  // Servo
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
  
  // Green LED
  if (String(actuatorDeviceId) == LEDLight1) {
    String key = "";
    bool commandValue = "";
    for (int i = 0; i < keys.length(); i++) {
      key = (const char*)keys[i];
      commandValue = (bool)commandObject[keys[i]];
      Serial.print("Key : ");
      Serial.println(key.c_str());
      Serial.print("value : ");
      Serial.println(commandValue);
    }

    if (commandValue == true) {
      Serial.println("LED ON");
      digitalWrite(ledPinG, true);
    } else {
      Serial.println("LED OFF");
      digitalWrite(ledPinG, false);
    }
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, true);
  }

  // Yellow LED
  if (String(actuatorDeviceId) == LEDLight2) {
    String key = "";
    bool commandValue = "";
    for (int i = 0; i < keys.length(); i++) {
      key = (const char*)keys[i];
      commandValue = (bool)commandObject[keys[i]];
      Serial.print("Key : ");
      Serial.println(key.c_str());
      Serial.print("value : ");
      Serial.println(commandValue);
    }

    if (commandValue == true) {
      Serial.println("LED ON");
      digitalWrite(ledPinY, true);
    } else {
      Serial.println("LED OFF");
      digitalWrite(ledPinY, false);
    }
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, true);
  }
  
  // Red LED
  if (String(actuatorDeviceId) == LEDLight3) {
    String key = "";
    bool commandValue = "";
    for (int i = 0; i < keys.length(); i++) {
      key = (const char*)keys[i];
      commandValue = (bool)commandObject[keys[i]];
      Serial.print("Key : ");
      Serial.println(key.c_str());
      Serial.print("value : ");
      Serial.println(commandValue);
    }

    if (commandValue == true) {
      Serial.println("LED ON");
      digitalWrite(ledPinR, true);
    } else {
      Serial.println("LED OFF");
      digitalWrite(ledPinR, false);
    }
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, true);
  }
}

void setup() {
  setup_wifi();
  voneClient.setup();
  voneClient.registerActuatorCallback(triggerActuator_callback);
  Serial.println("Gas sensor warming up!");
  delay(20000);  // allow the MQ-2 to warm up

  pinMode(irPin, INPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinY, OUTPUT);
  pinMode(ledPinR, OUTPUT);

  dht.begin();
  myServo.attach(servoPin);
  myServo.write(0); // Initialize servo to idle position (0 degrees)

  // Set initial states
  digitalWrite(ledPinG, LOW);
  digitalWrite(ledPinY, LOW);
  digitalWrite(ledPinR, LOW);

}

void loop() {
  if (!voneClient.connected()) {
    voneClient.reconnect();
    String errorMsg = "Sensor Fail";
    voneClient.publishDeviceStatusEvent(InfraredSensor, true);
    voneClient.publishDeviceStatusEvent(MQ2sensor, true);
    voneClient.publishDeviceStatusEvent(DHT11Sensor, true);
  }
  voneClient.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastMsgTime > INTERVAL) {
    lastMsgTime = currentMillis;

    //Publish MQ2 data
    airQuality = analogRead(mqPin);             // Read air quality
    // voneClient.publishTelemetryData(MQ2sensor, "Gas detector", airQuality);
    Serial.print("Air Quality Value: ");
    Serial.println(airQuality);

    //Publish DHT data
    float humidity = dht.readHumidity();     // Read humidity
    int temperature = dht.readTemperature(); // Read temperature

    // JSONVar payloadObject;
    // payloadObject["Humidity"] = humidity;
    // payloadObject["Temperature"] = temperature;
    // voneClient.publishTelemetryData(DHT11Sensor, payloadObject);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");  

    //Publish Infrared data
    bool irDetected = (digitalRead(irPin) == LOW);       // Read object detected
    // voneClient.publishTelemetryData(InfraredSensor, "Obstacle", irDetected);
    if (irDetected){Serial.println("Object Detected!");}


    // LED Control for Air Pollution Levels
    if (airQuality < 100) {         // Low pollution
      digitalWrite(ledPinG, HIGH);  // Green light
    } else if (airQuality >= 100 && airQuality < 2500) {  // Moderate pollution
      digitalWrite(ledPinY, HIGH);                        // Yellow light
    } else {                        // High pollution
      digitalWrite(ledPinR, HIGH);  // Red light
    }

    // Servo Control (Mask Dispenser)
    // If moderate/high polution, detected object (human) & button pressed
    if ((airQuality >= 100) && irDetected) {  
      Serial.println("Dispensing mask...");
      myServo.write(90);            // Move servo to active position
      // [***Adjust on how the servo dispense the mask***]
      delay(1000);                  // Hold position for 1 second
    } else {
      myServo.write(0);             // Move servo to idle position
    }
  }
}
