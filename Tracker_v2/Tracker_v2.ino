#include "Arduino.h"
#include "WiFi.h"
#include "LoRaWan_APP.h"
#include <Wire.h>  
#include "HT_st7735.h"
#include "HT_TinyGPS++.h"
#include "USB.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE Config
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "be152a26-9821-4a34-a63e-60d540217030"
BLECharacteristic *pCharacteristic;
bool ble_connected = false;

HT_st7735 st7735;
TinyGPSPlus gps;
#define VGNSS_CTRL Vext
#define VBAT_READ 1
#define ADC_CTRL  2

// LoRa config
#define RF_FREQUENCY                                915000000
#define TX_OUTPUT_POWER                             20
#define LORA_BANDWIDTH                              0
#define LORA_SPREADING_FACTOR                       10
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 128

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);

typedef enum {
    STATE_BOOT_DISPLAY,
    STATE_GOING_TO_SLEEP,
    STATE_SLEEP_3M,
    STATE_WAKE_TX_RX,
    STATE_ACTIVE_MODE
} StateMachine_t;

RTC_DATA_ATTR StateMachine_t current_state = STATE_BOOT_DISPLAY;
RTC_DATA_ATTR float last_valid_lat = 0.0;
RTC_DATA_ATTR float last_valid_lng = 0.0;
RTC_DATA_ATTR bool command_pending = false;

// Variables
bool lora_rx_success = false;
uint32_t state_start_time = 0;
uint32_t last_tx_time = 0;
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
const char* trackerName = "Tracker-1";
void IRAM_ATTR isr() { command_pending = true; }

class MyBLECallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { ble_connected = true; }
    void onDisconnect(BLEServer* pServer) { ble_connected = false; BLEDevice::startAdvertising(); }
};

void ble_init() {
    BLEDevice::init("Tracker-1");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyBLECallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEDevice::startAdvertising();
}

void update_ble_data() {
    if (ble_connected) {
        char ble_payload[128];
        sprintf(ble_payload, "{\"lat\":%.4f,\"lng\":%.4f,\"batt\":%d}", last_valid_lat, last_valid_lng, getBatteryPercent());
        pCharacteristic->setValue(ble_payload);
        pCharacteristic->notify();
    }
}

void VextON(void) { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }
void VextOFF(void) { pinMode(Vext, OUTPUT); digitalWrite(Vext, HIGH); }
void clearScreen() { st7735.st7735_fill_screen(ST7735_BLACK); }
void showScreen(int x, int y, const String& str_data, int pad_len = 20) { 
    String padded = str_data;
    while(padded.length() < pad_len) padded += " ";
    st7735.st7735_write_str(x, y, padded, Font_7x10, ST7735_CYAN, ST7735_BLACK); 
}

void lora_init(void) {
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.RxTimeout = OnRxTimeout;
	Radio.Init(&RadioEvents);
	Radio.SetChannel(RF_FREQUENCY);
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

void enter_deepsleep(uint32_t sleep_sec) {
	Radio.Sleep();
	SPI.end();
	VextOFF();
	pinMode(RADIO_DIO_1, ANALOG);
	pinMode(RADIO_NSS, ANALOG);
	pinMode(RADIO_RESET, ANALOG);
	pinMode(RADIO_BUSY, ANALOG);
	pinMode(LORA_CLK, ANALOG);
	pinMode(LORA_MISO, ANALOG);
	pinMode(LORA_MOSI, ANALOG);
	esp_sleep_enable_timer_wakeup((uint64_t)sleep_sec * 1000000ULL);
	esp_deep_sleep_start();
} 

int getBatteryPercent() {
    pinMode(ADC_CTRL, OUTPUT);
    digitalWrite(ADC_CTRL, HIGH);   // probar HIGH
    delay(5);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    int raw = analogRead(VBAT_READ);

    // divisor típico usado en Heltec/Meshtastic para Tracker
    float voltage = (raw / 4095.0f) * 3.3f * 4.9f;
    int percent = (int)((voltage - 3.3f) / (4.2f - 3.3f) * 100.0f);
    //Serial.printf("BAT raw=%d voltage=%.2fV percent=%d\n", raw, percent);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    return percent;

    return 0;
}

void send_lora_packet() {
    if (gps.location.isValid() && gps.location.lat() != 0.0) {
        last_valid_lat = gps.location.lat();
        last_valid_lng = gps.location.lng();
    }
    uint16_t batt = getBatteryPercent();
    const char* state_str = "STANDBY";
    char temp_payload[BUFFER_SIZE];

    if (current_state == STATE_BOOT_DISPLAY) {
        state_str = "BOOT";
        sprintf(temp_payload, "{\"name\":\"%s\",\"lat\":%.4f,\"lng\":%.4f,\"batt\":%d,\"state\":\"%s\"}", trackerName, last_valid_lat, last_valid_lng, batt, state_str);
    } else if (current_state == STATE_GOING_TO_SLEEP) {
        state_str = "SLEEPING";
        int sleep_duration_sec = 30;
        sprintf(temp_payload, "{\"name\":\"%s\",\"lat\":%.4f,\"lng\":%.4f,\"batt\":%d,\"state\":\"%s\",\"sleep_t\":%d}", trackerName, last_valid_lat, last_valid_lng, batt, state_str, sleep_duration_sec);
    } else if (current_state == STATE_WAKE_TX_RX) {
        state_str = "AWAKE";
        sprintf(temp_payload, "{\"name\":\"%s\",\"lat\":%.4f,\"lng\":%.4f,\"batt\":%d,\"state\":\"%s\"}", trackerName, last_valid_lat, last_valid_lng, batt, state_str);
    } else if (current_state == STATE_ACTIVE_MODE) {
        state_str = "ACTIVE";
        sprintf(temp_payload, "{\"name\":\"%s\",\"lat\":%.4f,\"lng\":%.4f,\"batt\":%d,\"state\":\"%s\"}", trackerName, last_valid_lat, last_valid_lng, batt, state_str);
    }
    strcpy(txpacket, temp_payload);
    Radio.Send((uint8_t *)txpacket, strlen(txpacket));
    last_tx_time = millis();
}

void gps_start(void)
{
    Serial1.begin(115200, SERIAL_8N1, 33, 34);
    Serial.println("gps_start");
}

void setup() {
	Serial.begin(115200);
	Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        if (current_state == STATE_SLEEP_3M) current_state = STATE_WAKE_TX_RX;
    } else {
        current_state = STATE_BOOT_DISPLAY;
    }

    if (current_state == STATE_BOOT_DISPLAY || current_state == STATE_ACTIVE_MODE) {
        VextON();
        delay(10);
        st7735.st7735_init();
        clearScreen();
        gps_start();
    }
    // boton
    pinMode(0, INPUT_PULLUP);
    attachInterrupt(0, isr, FALLING);
	lora_init();
    state_start_time = millis();
    lora_rx_success = false;
    if (current_state == STATE_WAKE_TX_RX) {
        VextON();
        delay(500);
        gps_start();
    }
}

void loop() {
    if(command_pending) {
        current_state = STATE_ACTIVE_MODE;
        command_pending = false;
        VextON();
        delay(10);
        st7735.st7735_init();
        clearScreen();
        gps_start();
        send_lora_packet();
    }

    while (Serial1.available() > 0) {
        char c = Serial1.read();
        gps.encode(c);
        if (gps.location.isUpdated()) {
            last_valid_lat = gps.location.lat();
            last_valid_lng = gps.location.lng();
            update_ble_data();
        }
    }

    Radio.IrqProcess();

    switch(current_state) {
        case STATE_BOOT_DISPLAY:
            if (millis() - state_start_time < 3600000) {
                showScreen(50, 5, "STANDBY", 10);
                showScreen(120, 5, String(getBatteryPercent()) + "%", 5);
                showScreen(0, 25, "Lat: " + String(gps.location.lat(), 4));
                showScreen(0, 40, "Lng: " + String(gps.location.lng(), 4));
                showScreen(0, 55, "Sats: " + String(gps.satellites.value()));
                showScreen(0, 70,
                    "Time: " +
                    String(gps.time.hour()) + ":" +
                    String(gps.time.minute()) + ":" +
                    String(gps.time.second()),
                    20
                );
                if (millis() - last_tx_time > 10000) send_lora_packet();
                delay(100);
            } else {
                current_state = STATE_GOING_TO_SLEEP;
            }
            break;
        case STATE_GOING_TO_SLEEP:
            send_lora_packet();
            showScreen(50, 5, "TO SLEEP", 10);
            delay(1500);
            current_state = STATE_SLEEP_3M;
            break;
        case STATE_SLEEP_3M:
            if (Serial) { // Check if USB serial is connected
                // SIMULATED SLEEP
                Serial.println("USB connected, using simulated sleep...");
                Radio.Rx(0);
                uint32_t sleep_start = millis();
                while(millis() - sleep_start < 30000) {
                    Radio.IrqProcess();
                    if(command_pending || lora_rx_success) break;
                    
                    int remaining = 30 - ((millis() - sleep_start) / 1000);
                    showScreen(50, 5, "SIM SLEEP", 10);
                    showScreen(120, 5, String(getBatteryPercent()) + "%", 5);
                    showScreen(0, 25, "Wake in: " + String(remaining) + "s      ", 20);
                    delay(500);
                }

                if (command_pending || lora_rx_success) {
                    current_state = STATE_ACTIVE_MODE;
                    command_pending = false;
                    lora_rx_success = false;
                } else {
                    current_state = STATE_WAKE_TX_RX;
                    state_start_time = millis();
                    lora_rx_success = false;
                }
                send_lora_packet();
                
            } else {
                // REAL DEEP SLEEP
                Serial.println("On battery, using real deep sleep...");
                showScreen(50, 5, "DEEP SLEEP", 10);
                delay(1000);
                enter_deepsleep(30);
            }
            break;
        case STATE_WAKE_TX_RX:
            if (millis() - state_start_time > 10000) {
                if (!lora_rx_success) {
                    current_state = STATE_GOING_TO_SLEEP;
                }
            } else {
                if (lora_rx_success) {
                    current_state = STATE_ACTIVE_MODE;
                    st7735.st7735_init();
                    clearScreen();
                }
            }
            break;
        case STATE_ACTIVE_MODE:
            if(command_pending) command_pending = false;
            showScreen(50, 5, "ACTIVE", 10);
            showScreen(120, 5, String(getBatteryPercent()) + "%", 5);
            showScreen(0, 25, "Lat: " + String(gps.location.lat(), 4));
            showScreen(0, 40, "Lng: " + String(gps.location.lng(), 4));
            showScreen(0, 55, "Sats: " + String(gps.satellites.value()));
            if (millis() - last_tx_time > 1000) send_lora_packet();
            delay(100);
            break;
    }
}

void OnTxDone(void) { Serial.println("TX done"); Radio.Rx(0); }
void OnTxTimeout(void) { Radio.Sleep(); Serial.println("TX Timeout"); }
void OnRxTimeout(void) { Radio.Sleep(); Serial.println("RX Timeout"); }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    Serial.printf("Received: %s (RSSI: %d)\\n", rxpacket, rssi);
    if (strstr(rxpacket, "ACTIVE") != NULL) lora_rx_success = true;
}
