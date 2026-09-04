#include "Arduino.h"
#include "WiFi.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"
#include <HTTPClient.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <ESPmDNS.h>
#include <BLE2902.h>

// LORA & WIFI CONFIG
#define RF_FREQUENCY                                915000000
#define TX_OUTPUT_POWER                             20
#define LORA_BANDWIDTH                              0
#define LORA_SPREADING_FACTOR                       10
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            0
#define BUFFER_SIZE                                 128

const char* ssid = "SurisNET";
const char* password = "NETdidge123";

// CONFIGURACIÓN HOME ASSISTANT
const char* ha_token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJiZTBjMGUzY2VhZWM0ZDMwYjA1NjY3ZDA1MWIwYzNmYSIsImlhdCI6MTc3NDQ5MjM0MCwiZXhwIjoyMDg5ODUyMzQwfQ.mi2Yvb9pMuU2KlYZbzBkBQn1FX1OJWoKMLY16nV9NTQ";
const char* ha_server = "http://homeassistant.local:8123/api/states/device_tracker."; 

// BLE Config
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "be152a26-9821-4a34-a63e-60d540217030"
BLECharacteristic *pCharacteristic;
bool ble_connected = false;

// GLOBAL VARIABLES
SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
static RadioEvents_t RadioEvents;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
bool message_received = false;
bool command_pending = false;

// Last known data from tracker
String last_lat = "0.0";
String last_lng = "0.0";
String tracker_batt = "0";
String last_state = "BOOT";
String last_payload = "Waiting...";
String tracker_name = "Unknown";
bool tracker_pos_valid = false;
int tracker_sleep_duration = 30;
uint32_t tracker_sleep_start_time = 0;
uint32_t last_message_time = 0; // Tiempo del último mensaje recibido

// HELPER FUNCTIONS
class MyBLECallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { ble_connected = true; }
    void onDisconnect(BLEServer* pServer) { ble_connected = false; BLEDevice::startAdvertising(); }
};

void ble_init() {
    BLEDevice::init("Gateway-Repetidor"); 
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyBLECallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEDevice::startAdvertising();
    Serial.println("Bluetooth iniciado como: Gateway-Repetidor");
}

void update_ble_gateway(String payload) {
    if (ble_connected) {
        pCharacteristic->setValue(payload.c_str());
        pCharacteristic->notify();
        Serial.printf("BLE Enviado: %s\n", payload.c_str());
    } else {
        Serial.println("BLE no enviado: Sin conexión de celular");
    }
}

void VextON(void) { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }
void VextOFF(void) { pinMode(Vext, OUTPUT); digitalWrite(Vext, HIGH); }

int getBatteryPercent() {
    float voltage = (analogRead(1) / 4095.0) * 3.3 * 4.9;
    int percent = (int)((voltage - 3.3) / (4.2 - 3.3) * 100.0);
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return percent;
}

void parseJsonData(String payload) {
    int nameIdx = payload.indexOf("\"name\":\"");
    if (nameIdx != -1) tracker_name = payload.substring(nameIdx + 8, payload.indexOf("\"", nameIdx + 8));

    int latIdx = payload.indexOf("\"lat\":");
    if (latIdx != -1) {
        last_lat = payload.substring(latIdx + 6, payload.indexOf(",", latIdx));
        tracker_pos_valid = (last_lat.toFloat() != 0.0);
    } else {
        tracker_pos_valid = false;
    }

    int lngIdx = payload.indexOf("\"lng\":");
    if (lngIdx != -1) last_lng = payload.substring(lngIdx + 6, payload.indexOf(",", lngIdx));

    int battIdx = payload.indexOf("\"batt\":");
    if (battIdx != -1) tracker_batt = payload.substring(battIdx + 7, payload.indexOf(",", battIdx));

    int stateIdx = payload.indexOf("\"state\":\"");
    if (stateIdx != -1) last_state = payload.substring(stateIdx + 9, payload.indexOf("\"", stateIdx + 9));
    
    int sleepIdx = payload.indexOf("\"sleep_t\":");
    if (sleepIdx != -1) tracker_sleep_duration = payload.substring(sleepIdx + 10, payload.indexOf("}", sleepIdx)).toInt();
}

void forwardData() {
    if(WiFi.status() == WL_CONNECTED && tracker_pos_valid){
      HTTPClient http;
      
      String sanitized_name = tracker_name;
      sanitized_name.toLowerCase();
      sanitized_name.replace("-", "_");
      sanitized_name.replace(" ", "_");

      String url = String(ha_server) + sanitized_name;
      http.begin(url);
      
      http.addHeader("Authorization", "Bearer " + String(ha_token));
      http.addHeader("Content-Type", "application/json");

      String ha_payload = "{";
      ha_payload += "\"state\": \"not_home\","; 
      ha_payload += "\"attributes\": {";
      ha_payload += "\"latitude\": " + last_lat + ",";
      ha_payload += "\"longitude\": " + last_lng + ",";
      ha_payload += "\"battery_level\": " + tracker_batt + ",";
      ha_payload += "\"friendly_name\": \"" + tracker_name + "\","; 
      ha_payload += "\"source_type\": \"gps\",";
      ha_payload += "\"gps_accuracy\": 10";
      ha_payload += "}}";

      int httpResponseCode = http.POST(ha_payload);
      Serial.printf("HA Response code: %d\n", httpResponseCode);
      http.end();
    }
}

// LORA CALLBACKS
void OnTxDone(void) { Radio.Rx(0); Serial.println("TX done"); }
void OnTxTimeout(void) { Radio.Rx(0); Serial.println("TX Timeout"); }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    Serial.printf("Received: %s (RSSI: %d)\n", rxpacket, rssi);
    message_received = true;
}

void IRAM_ATTR isr() { command_pending = true; }

void lora_init(void) {
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxDone = OnRxDone;
	Radio.Init(&RadioEvents);
	Radio.SetChannel(RF_FREQUENCY);
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    pinMode(0, INPUT_PULLUP);
    attachInterrupt(0, isr, FALLING);
    VextON();
    delay(100);
    factory_display.init();
    factory_display.setFont(ArialMT_Plain_10);
    
    WiFi.begin(ssid, password);
    if (!MDNS.begin("esp32-gateway")) {
        Serial.println("Error configurando mDNS");
    }

    ble_init(); 
    lora_init();
    Radio.Rx(0);
}

// --- MAIN LOOP ---
void loop() {
    Radio.IrqProcess();

    if (message_received) {
        String payload = String(rxpacket);
        last_payload = payload;
        last_message_time = millis(); 
        parseJsonData(payload);
        forwardData(); 
        update_ble_gateway(payload); 
        message_received = false;
        if (last_state == "SLEEPING") {
            tracker_sleep_start_time = millis() - 4000;
        }
    }

    if (command_pending) {
        sprintf(txpacket, "ACTIVE");
        Radio.Send((uint8_t *)txpacket, strlen(txpacket));
        command_pending = false;
    }

    factory_display.clear();
    String wifiStr = (WiFi.status() == WL_CONNECTED) ? "WiFi: OK" : "WiFi: --";
    String btStr = ble_connected ? "BT: OK" : "BT: --";
    factory_display.setTextAlignment(TEXT_ALIGN_LEFT);
    factory_display.drawString(0, 0, wifiStr);
    factory_display.setTextAlignment(TEXT_ALIGN_RIGHT);
    factory_display.drawString(128, 0, btStr);
    factory_display.drawLine(0, 13, 128, 13);
    
    bool is_tracker_sleeping = (last_state == "SLEEPING");
    bool countdown_active = is_tracker_sleeping && (millis() - tracker_sleep_start_time < (uint32_t)tracker_sleep_duration * 1000);

    if (countdown_active) {
        int elapsed_sec = (millis() - tracker_sleep_start_time) / 1000;
        int remaining_sec = tracker_sleep_duration - elapsed_sec;
        if (remaining_sec < 0) remaining_sec = 0;
        
        factory_display.setTextAlignment(TEXT_ALIGN_CENTER);
        factory_display.drawString(64, 20, tracker_name + " wakes in:");
        factory_display.setFont(ArialMT_Plain_24);
        factory_display.drawString(64, 35, String(remaining_sec) + "s");
        factory_display.setFont(ArialMT_Plain_10);
    } else {
        factory_display.setTextAlignment(TEXT_ALIGN_LEFT);
        factory_display.drawString(0, 16, "Tracker: " + tracker_name);
        factory_display.drawString(0, 30, "GPS: " + String(tracker_pos_valid ? "Valid" : "No Fix"));
        factory_display.drawString(0, 44, "State: " + last_state);
        
        if (last_message_time > 0) {
            uint32_t seconds = (millis() - last_message_time) / 1000;
            factory_display.setTextAlignment(TEXT_ALIGN_RIGHT);
            factory_display.drawString(127, 54, "Last: " + String(seconds) + "s ago");
        }
    }
    
    factory_display.display();
    delay(300);
}
