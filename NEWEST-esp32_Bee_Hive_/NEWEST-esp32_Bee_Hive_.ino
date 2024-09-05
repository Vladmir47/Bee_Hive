#define TINY_GSM_MODEM_SIM800 // Define GSM modem model
#define TINY_GSM_RX_BUFFER 1024

#include <Wire.h>
#include <TinyGsmClient.h>
#include <DHT.h>

// Define sensor pins
#define DHTPIN 4           // Pin for DHT11 sensor (digital input)
#define DHTTYPE DHT11       // DHT11 sensor type
#define LDRPIN 34           // Pin for LDR (analog input, use an available ADC pin)
#define SDA_PIN 21          // SDA pin for ENS160
#define SCL_PIN 22          // SCL pin for ENS160

// Define ENS160 I2C address (check your sensor's datasheet)
#define ENS160_ADDR 0x53

// Linear actuator control pins
#define IN1_PIN 33  // IN1 for L298N to control actuator direction
#define IN2_PIN 25  // IN2 for L298N
#define ENA_PIN 18  // Enable pin for speed control (optional)

// Thresholds
const int co2Threshold = 1000000;  // CO2 level threshold in ppm
const int ldrThreshold = 100000;   // LDR threshold for night detection (adjust as needed)

// Define the APN and credentials
const char apn[] = "halotel internet"; // APN for Halotel Tanzania
const char gprsUser[] = "";            // GPRS User (leave empty if not needed)
const char gprsPass[] = "";            // GPRS Password (leave empty if not needed)

// SIM card PIN (leave empty if not defined)
const char simPIN[] = "";

// ThingSpeak server details
const char server[] = "api.thingspeak.com";
const int port = 80;
String apiKeyValue = "F2UFOOKOC44JGHDC";

// TTGO T-Call pins
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26

// Set serial for debug console
#define SerialMon Serial
#define SerialAT Serial1

// Configure TinyGSM library
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

// Initialize sensors
DHT dht(DHTPIN, DHTTYPE);

unsigned long previousMillis = 0; // Stores last time data was sent
const long interval = 60000; // Interval at which to send data (1 minute)

void setup() {
  SerialMon.begin(115200);
  SerialMon.println("Serial Monitor is starting...");

  // Set modem pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module
  SerialMon.println("Initializing modem...");
  modem.restart();

  // Unlock SIM card if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3) {
    modem.simUnlock(simPIN);
  }

  // Connect to GPRS network
  SerialMon.print("Connecting to APN: ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    return;  // Exit setup if GPRS connection fails
  } else {
    SerialMon.println(" OK");
  }

  // Initialize DHT11 sensor
  dht.begin();

  // Initialize ENS160 sensor with defined SDA and SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(ENS160_ADDR);
  if (Wire.endTransmission() != 0) {
    SerialMon.println("ENS160 sensor not found.");
    while (1);  // Halt execution if sensor is not found
  }
  SerialMon.println("ENS160 sensor found.");

  // Initialize linear actuator control pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);

  // Initially stop the actuator
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(ENA_PIN, 255);  // Full speed
}

// Function to move the actuator forward
void moveForward(int speed) {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
}

// Function to move the actuator backward
void moveBackward(int speed) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
}

// Function to stop the actuator
void stopActuator() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to send data
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    SerialMon.println("Loop started...");

    // Read sensors
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int ldrValue = analogRead(LDRPIN);  // Read LDR value

    // Read ENS160 sensor data
    Wire.beginTransmission(ENS160_ADDR);
    Wire.requestFrom(ENS160_ADDR, 4);  // Request 4 bytes
    if (Wire.available() == 4) {
      uint16_t co2 = Wire.read() | (Wire.read() << 8);
      uint16_t tvoc = Wire.read() | (Wire.read() << 8);

      // Print sensor data
      SerialMon.print("CO2: ");
      SerialMon.print(co2);
      SerialMon.print(" ppm, TVOC: ");
      SerialMon.print(tvoc);
      SerialMon.println(" ppb");

      SerialMon.print("Temperature: ");
      SerialMon.print(temperature);
      SerialMon.println("°C");

      SerialMon.print("Humidity: ");
      SerialMon.print(humidity);
      SerialMon.println("%");

      SerialMon.print("LDR Value: ");
      SerialMon.println(ldrValue);

      // Control linear actuator based on CO2 and light levels
      if (co2 < co2Threshold && ldrValue < ldrThreshold) {
        Serial.println("Moving forward...");
        moveForward(204);  // Move forward at 80% speed
        delay(20000);

        Serial.println("Stopping...");
        stopActuator();
        delay(2000);

        Serial.println("Moving backward...");
        moveBackward(153);  // Move backward at 60% speed
        delay(20000);

        Serial.println("Stopping...");
        stopActuator();
        delay(2000);
      } else {
        SerialMon.println("Conditions not met. Stopping actuator.");
        stopActuator();
      }

      // Prepare HTTP POST request data
      String httpRequestData = "api_key=" + apiKeyValue
                               + "&field1=" + String(temperature)
                               + "&field2=" + String(humidity)
                               + "&field3=" + String(ldrValue)
                               + "&field4=" + String(co2)
                               + "&field5=" + String(tvoc);

      // Send HTTP POST request to ThingSpeak
      if (client.connect(server, port)) {
        SerialMon.println("Connected to ThingSpeak, sending data...");
        client.print(String("POST /update HTTP/1.1\r\n"));
        client.print(String("Host: ") + server + "\r\n");
        client.println("Connection: close");
        client.println("Content-Type: application/x-www-form-urlencoded");
        client.print("Content-Length: ");
        client.println(httpRequestData.length());
        client.println();
        client.println(httpRequestData);

        // Read server response
        unsigned long timeout = millis();
        while (client.connected() && millis() - timeout < 10000L) {
          while (client.available()) {
            char c = client.read();
            SerialMon.print(c);  // Print the response to Serial Monitor
          }
        }

        // Close connection
        client.stop();
        SerialMon.println("Data sent and server disconnected");
      } else {
        SerialMon.println("Failed to connect to ThingSpeak.");
      }
    } else {
      SerialMon.println("Failed to read from ENS160 sensor.");
    }
  }
}
