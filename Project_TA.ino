#define BLYNK_TEMPLATE_ID "TMPL6VKDy8ahI"
#define BLYNK_TEMPLATE_NAME "ESP Controlling"

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <BlynkSimpleEsp32.h>
#include <RBDdimmer.h>

// Pin Definitions
#define DHTPIN 4        
#define DHTTYPE DHT22   
#define OUTPUT_PIN 19 
#define ZERO_CROSS_PIN 18 
#define RELAY1_PIN 12 
#define RELAY2_PIN 13 

// ThingSpeak Settings
const char* server = "api.thingspeak.com";  
const char* apiKey = "6J7TOKY9ERZK815L";    

// Blynk Credentials
char auth[] = "nj5zwe_KMwcXelF5-EtQOCRqiAFww549";
char ssid[] = "UNILA-PGN-22";
char pass[] = "";

// Global Variables
int outVal = 0; 
int dim_val = 0;

bool relayState1 = false;
bool relayState2 = false;
bool manualControl1 = false; 
bool manualControl2 = false; 
bool manualDimmerControl = false; 

float temperature; 
float humidity;    
float kalibrasi;
float kalibrasi2;

// Initialize Components
LiquidCrystal_I2C lcd(0x27, 16, 2);  
DHT dht(DHTPIN, DHTTYPE);
WiFiClient client;
dimmerLamp dimmer(OUTPUT_PIN, ZERO_CROSS_PIN); 

// Blynk Control Handlers
BLYNK_WRITE(V0) {
    outVal = param.asInt();
    dim_val = map(outVal, 0, 1023, 0, 100);
    dimmer.setPower(dim_val);
    Blynk.virtualWrite(V1, dim_val);
    manualDimmerControl = true;
}

BLYNK_WRITE(V2) {
    int relay1Value = param.asInt();
    digitalWrite(RELAY1_PIN, relay1Value ? HIGH : LOW);
    relayState1 = relay1Value;
    manualControl1 = true;
}

BLYNK_WRITE(V3) {
    int relay2Value = param.asInt();
    digitalWrite(RELAY2_PIN, relay2Value ? HIGH : LOW);
    relayState2 = relay2Value;
    manualControl2 = true;
}

void setup() {
    Serial.begin(115200);
    lcd.init();
    lcd.backlight();
    dht.begin();

    Serial.println("DHT22 Sensor Test"); 

    dimmer.begin(NORMAL_MODE, ON);

    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);

    // Ensure Relays Start Off
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);

    // Connect to Wi-Fi
    Serial.println();
    Serial.print("Menghubungkan ke ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nWiFi Terhubung");

    // Connect to Blynk
    Blynk.config(auth);
    if (Blynk.connect()) {
        Serial.println("Blynk Terhubung");
    } else {
        Serial.println("Blynk Gagal Terhubung");
    }

    // Create tasks for Blynk and sensor reading
    xTaskCreate(blynkTask, "BlynkTask", 4096, NULL, 1, NULL);
    xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 1, NULL);
}

void loop() {
    // Main loop left empty
}

void blynkTask(void *pvParameters) {
    while (true) {
        Blynk.run();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void sensorTask(void *pvParameters) {
    while (true) {
        bacaSensorDanKirim();
        vTaskDelay(20000 / portTICK_PERIOD_MS); // 20-second interval for sensor reading
    }
}

void bacaSensorDanKirim() {
    // Sensor readings and calibration
    temperature = dht.readTemperature();
    kalibrasi = 0.9849 * temperature + 0.4584;
    humidity = dht.readHumidity();
    kalibrasi2 = 0.9728 * humidity + 1.366;

    // Check for failed sensor readings
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Failed to read from DHT sensor!");
        lcd.setCursor(0, 0);
        lcd.print("Error: DHT22");
        lcd.setCursor(0, 1);
        lcd.print("Gagal dibaca");
        return;
    }

    // Display readings on LCD
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.printf("Temp: %.2f C", kalibrasi);
    lcd.setCursor(0, 1); 
    lcd.printf("Humi: %.2f %%", kalibrasi2);

    // Print readings to Serial
    Serial.printf("Suhu: %.2f °C Kelembaban: %.2f %%\n", temperature, humidity);
    Serial.printf("Suhu (kalibrasi): %.2f °C Kelembaban (kalibrasi): %.2f %%\n", kalibrasi, kalibrasi2);

    // Control relays based on humidity, if not manually controlled
    if (!manualControl1) {
        if (kalibrasi2 < 65.0) {
            digitalWrite(RELAY1_PIN, HIGH);  // Turn on Relay 1
            relayState1 = true;
        } else if (kalibrasi2 > 70.0) {
            digitalWrite(RELAY1_PIN, LOW);   // Turn off Relay 1
            relayState1 = false;
        }
    }

    if (!manualControl2) {
        if (kalibrasi2 < 65.0) {
            digitalWrite(RELAY2_PIN, HIGH);  // Turn on Relay 2
            relayState2 = true;
        } else if (kalibrasi2 > 70.0) {
            digitalWrite(RELAY2_PIN, LOW);   // Turn off Relay 2
            relayState2 = false;
        }
    }

    // Reset manual control flags if automatic control has been engaged
    if (!manualControl1 && !manualControl2) {
        manualControl1 = false;
        manualControl2 = false;
    }
    
    // Control dimmer based on temperature, if not manually controlled
    if (!manualDimmerControl) {
        if (kalibrasi < 27.0) {
            dimmer.setPower(50);
        } else if (kalibrasi > 29.0) {
            dimmer.setPower(0);
        } else {
            dimmer.setPower(0);
        }
    }

    manualDimmerControl = false; // Reset manual control after automatic control
}
