#include "VOneMqttClient.h"
#include <ESP32Servo.h> 
#include "SCD30.h"

// SCD30
#if defined(ARDUINO_ARCH_AVR)
    #pragma message("Defined architecture for ARDUINO_ARCH_AVR.")
    #define SERIAL Serial
#elif defined(ARDUINO_ARCH_SAM)
    #pragma message("Defined architecture for ARDUINO_ARCH_SAM.")
    #ifdef SEEED_XIAO_M0
        #define SERIAL Serial
    #elif defined(ARDUINO_SAMD_VARIANT_COMPLIANCE)
        #define SERIAL SerialUSB
    #else
        #define SERIAL Serial
    #endif
#elif defined(ARDUINO_ARCH_SAMD)
    #pragma message("Defined architecture for ARDUINO_ARCH_SAMD.")
    #ifdef SEEED_XIAO_M0
        #define SERIAL Serial
    #elif defined(ARDUINO_SAMD_VARIANT_COMPLIANCE)
        #define SERIAL SerialUSB
    #else
        #define SERIAL Serial
    #endif
#elif defined(ARDUINO_ARCH_STM32F4)
    #pragma message("Defined architecture for ARDUINO_ARCH_STM32F4.")
    #ifdef SEEED_XIAO_M0
        #define SERIAL Serial
    #elif defined(ARDUINO_SAMD_VARIANT_COMPLIANCE)
        #define SERIAL SerialUSB
    #else
        #define SERIAL Serial
    #endif
#else
    #pragma message("Not found any architecture.")
    #define SERIAL Serial
#endif

// Define device ID
const char* InfraredSensor = "69384d46-405f-4ef0-b7e3-067e58f14e12";  //Replace with the deviceID of YOUR infrared sensor
const char* ServoMotor = "c565a9ed-57f5-4032-bda9-01274a7b8545";      //Replace this with YOUR deviceID for the servo
const char* SCDsensor = "b8a91767-9af5-43ae-8500-12d083b992a4";       //Replace this with YOUR deviceID for the MQ2 sensor
const char* LEDLight1 = "05718b95-249e-410c-9adc-aae478125d7e";       //Replace this with YOUR deviceID for the first LED
const char* LEDLight2 = "cf0292bc-84b9-45e3-b1b2-f9c3394273c6";       //Replace this with YOUR deviceID for the second LED
const char* LEDLight3 = "cae948a5-d7d8-46b8-a2e5-4ba5ed29b184";       //Replace this with YOUR deviceID for the third LED

// Define pin number
const int irPin = 4;          // IR sensor (Maker: Pin 4, Left Maker Port) 
const int ledPinG = 5;        // Green LED (Maker: Pin 5)
const int ledPinY = 9;        // Yellow LED (Maker: Pin 9) 
const int ledPinR = 10;       // Red LED (Maker: Pin 10) 
const int servoPin = 39;      // Servo (Maker: Pin 39)
// const int SCDPin = 42;         // SCD30 Sensor

// Define data variables
bool irDetected = false;        // Tracks IR sensor state
float SCDresult[3] = {0};       // Store data from SCD30 sensor
float CO2Concentration = 0;     // CO2 concentration
float temperature = 0.0;        // Temperature value
float humidity = 0.0;           // Humidity value
volatile int maskDispensed = 0; // Tracks number of times the lid opens
float moderatePollutionThres = 1000;  // Moderate air pollution threshold
float highPollutionThres = 5000;      // High air pollution threshold

Servo myServo;  // Create a Servo object

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

void setup() {
  setup_wifi();
  voneClient.setup();
  Wire.begin();
  SERIAL.begin(115200);
  scd30.initialize();

  pinMode(irPin, INPUT);
  pinMode(ledPinG, OUTPUT);
  pinMode(ledPinY, OUTPUT);
  pinMode(ledPinR, OUTPUT);

  myServo.write(175); // Initialize servo to idle position (close lid)
  myServo.attach(servoPin);

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
    voneClient.publishDeviceStatusEvent(SCDsensor, true);
  }
  voneClient.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastMsgTime > INTERVAL) {
    lastMsgTime = currentMillis;

    //Publish SCD data
    if (scd30.isAvailable()) {
        scd30.getCarbonDioxideConcentration(SCDresult); 

        SERIAL.print("Carbon Dioxide Concentration: ");
        SERIAL.print(SCDresult[0]); 
        SERIAL.println(" ppm");
        CO2Concentration = SCDresult[0];  // Store CO2 Concentration

        SERIAL.print("Temperature: ");
        SERIAL.print(SCDresult[1]); 
        SERIAL.println(" ℃");
        temperature = SCDresult[1];       // Store Temperature

        SERIAL.print("Humidity: "); 
        SERIAL.print(SCDresult[2]);
        SERIAL.println(" %");
        humidity = SCDresult[2];          // Store Humidity
        SERIAL.println(" ");

        JSONVar payloadObject;
        payloadObject["CO2 Concentration"] = CO2Concentration;
        payloadObject["Temperature"] = temperature;
        payloadObject["Humidity"] = humidity;
        voneClient.publishTelemetryData(SCDsensor, payloadObject);
    }
    delay(2000);

    // LED Control for Air Pollution Levels
    if (SCDresult[0] < moderatePollutionThres) {  // Low pollution
      digitalWrite(ledPinG, HIGH);  // Green light
      digitalWrite(ledPinY, LOW);
      digitalWrite(ledPinR, LOW);
    } else if (SCDresult[0] >= moderatePollutionThres && SCDresult[0] < highPollutionThres) {  // Moderate pollution
      digitalWrite(ledPinG, LOW);
      digitalWrite(ledPinY, HIGH);  // Yellow light
      digitalWrite(ledPinR, LOW);
    } else {  // High pollution
      digitalWrite(ledPinG, LOW);
      digitalWrite(ledPinY, LOW);
      digitalWrite(ledPinR, HIGH);  // Red light
    }

    // Publish Infrared data
    // Servo Control (Mask Dispenser)
    // If moderate/high polution & detected object (hand)
    irDetected = (digitalRead(irPin) == LOW);  // Read object detected
    
    if (irDetected && (SCDresult[0] >= moderatePollutionThres)){ 
      Serial.println("Opening mask container lid...");
      myServo.write(115);           // Move servo to active position (open lid)
      delay(1000);                  // Hold position for 1 second
    
      maskDispensed++;  // Accumulate number of time user dispensed mask
      Serial.print("Mask Dispensed: ");
      Serial.println(maskDispensed);
      voneClient.publishTelemetryData(InfraredSensor, "Obstacle", maskDispensed);   

    } else {
        myServo.write(175);         // Move servo to idle position (close lid)
    }

    delay(2000);
  }
}
