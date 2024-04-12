# 1 "C:\\Users\\priit\\AppData\\Local\\Temp\\tmpg8slyu40"
#include <Arduino.h>
# 1 "C:/xampp7.4/htdocs/Battery-Emulator/Software/Software.ino"



#include <Arduino.h>
#include <Preferences.h>
#include "HardwareSerial.h"
#include "USER_SETTINGS.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "src/battery/BATTERIES.h"
#include "src/charger/CHARGERS.h"
#include "src/devboard/config.h"
#include "src/devboard/utils/events.h"
#include "src/inverter/INVERTERS.h"
#include "src/lib/adafruit-Adafruit_NeoPixel/Adafruit_NeoPixel.h"
#include "src/lib/bblanchon-ArduinoJson/ArduinoJson.h"
#include "src/lib/eModbus-eModbus/Logging.h"
#include "src/lib/eModbus-eModbus/ModbusServerRTU.h"
#include "src/lib/eModbus-eModbus/scripts/mbServerFCs.h"
#include "src/lib/miwagner-ESP32-Arduino-CAN/CAN_config.h"
#include "src/lib/miwagner-ESP32-Arduino-CAN/ESP32CAN.h"

#ifdef WEBSERVER
#include <ESPmDNS.h>
#include "src/devboard/webserver/webserver.h"
#endif

Preferences settings;

const char* version_number = "5.7.0";


uint16_t intervalUpdateValues = INTERVAL_5_S;
unsigned long previousMillis10ms = 50;
unsigned long previousMillisUpdateVal = 0;


CAN_device_t CAN_cfg;
const int rx_queue_size = 10;

#ifdef DUAL_CAN
#include "src/lib/pierremolinaro-acan2515/ACAN2515.h"
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL;
ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);
static ACAN2515_Buffer16 gBuffer;
#endif
#ifdef CAN_FD
#include "src/lib/pierremolinaro-ACAN2517FD/ACAN2517FD.h"
ACAN2517FD canfd(MCP2517_CS, SPI, MCP2517_INT);
#endif


#if defined(BYD_MODBUS) || defined(LUNA2000_MODBUS)
#define MB_RTU_NUM_VALUES 30000
uint16_t mbPV[MB_RTU_NUM_VALUES];

ModbusServerRTU MBserver(Serial2, 2000);
#endif


uint32_t system_capacity_Wh = BATTERY_WH_MAX;
uint32_t system_remaining_capacity_Wh = BATTERY_WH_MAX;
int16_t system_temperature_max_dC = 0;
int16_t system_temperature_min_dC = 0;
int32_t system_active_power_W = 0;
int16_t system_battery_current_dA = 0;
uint16_t system_battery_voltage_dV = 3700;
uint16_t system_max_design_voltage_dV = 5000;
uint16_t system_min_design_voltage_dV = 2500;
uint16_t system_scaled_SOC_pptt = 5000;
uint16_t system_real_SOC_pptt = 5000;
uint16_t system_SOH_pptt = 9900;
uint32_t system_max_discharge_power_W = 0;
uint32_t system_max_charge_power_W = 4312;
uint16_t system_cell_max_voltage_mV = 3700;
uint16_t system_cell_min_voltage_mV = 3700;
uint16_t system_cellvoltages_mV[MAX_AMOUNT_CELLS];
uint8_t system_bms_status = ACTIVE;
uint8_t system_number_of_cells = 0;
bool system_LFP_Chemistry = false;


volatile float charger_setpoint_HV_VDC = 0.0f;
volatile float charger_setpoint_HV_IDC = 0.0f;
volatile float charger_setpoint_HV_IDC_END = 0.0f;
bool charger_HV_enabled = false;
bool charger_aux12V_enabled = false;


float charger_stat_HVcur = 0;
float charger_stat_HVvol = 0;
float charger_stat_ACcur = 0;
float charger_stat_ACvol = 0;
float charger_stat_LVcur = 0;
float charger_stat_LVvol = 0;


Adafruit_NeoPixel pixels(1, WS2812_PIN, NEO_GRB + NEO_KHZ800);
static uint8_t brightness = 0;
static bool rampUp = true;
const uint8_t maxBrightness = 100;
uint8_t LEDcolor = GREEN;


#ifdef CONTACTOR_CONTROL
enum State { DISCONNECTED, PRECHARGE, NEGATIVE, POSITIVE, PRECHARGE_OFF, COMPLETED, SHUTDOWN_REQUESTED };
State contactorStatus = DISCONNECTED;

#define MAX_ALLOWED_FAULT_TICKS 500
#define PRECHARGE_TIME_MS 160
#define NEGATIVE_CONTACTOR_TIME_MS 1000
#define POSITIVE_CONTACTOR_TIME_MS 2000
#ifdef PWM_CONTACTOR_CONTROL
#define PWM_Freq 20000
#define PWM_Res 10
#define PWM_Hold_Duty 250
#define POSITIVE_PWM_Ch 0
#define NEGATIVE_PWM_Ch 1
#endif
unsigned long prechargeStartTime = 0;
unsigned long negativeStartTime = 0;
unsigned long timeSpentInFaultedMode = 0;
#endif
bool batteryAllowsContactorClosing = false;
bool inverterAllowsContactorClosing = true;

TaskHandle_t mainLoopTask;
void setup();
void loop();
void mainLoop(void* pvParameters);
void init_mDNS();
void init_serial();
void init_stored_settings();
void init_CAN();
void init_LED();
void init_contactors();
void init_modbus();
void inform_user_on_inverter();
void init_battery();
void receive_canfd();
void receive_can();
void send_can();
void receive_can2();
void send_can2();
void handle_LED_state();
void handle_contactors();
void update_SOC();
void update_values();
void runSerialDataLink();
void init_serialDataLink();
void storeSettings();
#line 132 "C:/xampp7.4/htdocs/Battery-Emulator/Software/Software.ino"
void setup() {
  init_serial();

  init_stored_settings();

#ifdef WEBSERVER
  init_webserver();

  init_mDNS();
#endif

  init_events();

  init_CAN();

  init_LED();

  init_contactors();

  init_modbus();

  init_serialDataLink();

  inform_user_on_inverter();

  init_battery();


  pinMode(0, INPUT_PULLUP);

  esp_task_wdt_deinit();

  xTaskCreatePinnedToCore((TaskFunction_t)&mainLoop, "mainLoop", 4096, NULL, 8, &mainLoopTask, 1);
}


void loop() {
  ;
}

void mainLoop(void* pvParameters) {
  while (true) {

#ifdef WEBSERVER

    wifi_monitor();
    ElegantOTA.loop();
#ifdef MQTT
    mqtt_loop();
#endif
#endif


    receive_can();
#ifdef CAN_FD
    receive_canfd();
#endif
#ifdef DUAL_CAN
    receive_can2();
#endif
#if defined(SERIAL_LINK_RECEIVER) || defined(SERIAL_LINK_TRANSMITTER)
    runSerialDataLink();
#endif


    if (millis() - previousMillis10ms >= INTERVAL_10_MS) {
      previousMillis10ms = millis();
      handle_LED_state();
#ifdef CONTACTOR_CONTROL
      handle_contactors();
#endif
    }

    if (millis() - previousMillisUpdateVal >= intervalUpdateValues)
    {
      previousMillisUpdateVal = millis();
      update_SOC();
      update_values();
      if (DUMMY_EVENT_ENABLED) {
        set_event(EVENT_DUMMY_ERROR, (uint8_t)millis());
      }
    }


    send_can();
#ifdef DUAL_CAN
    send_can2();
#endif
    run_event_handling();

    delay(1);
  }
}

#ifdef WEBSERVER

void init_mDNS() {



  String mac = WiFi.macAddress();
  String mdnsHost = "batteryemulator" + mac.substring(mac.length() - 2);


  if (!MDNS.begin(mdnsHost)) {
#ifdef DEBUG_VIA_USB
    Serial.println("Error setting up MDNS responder!");
#endif
  } else {

    MDNS.addService("battery_emulator", "tcp", 80);
  }
}
#endif


void init_serial() {

  Serial.begin(115200);
  while (!Serial) {}
#ifdef DEBUG_VIA_USB
  Serial.println("__ OK __");
#endif
}

void init_stored_settings() {
  settings.begin("batterySettings", false);

#ifndef LOAD_SAVED_SETTINGS_ON_BOOT
  settings.clear();
#endif

  static uint32_t temp = 0;
  temp = settings.getUInt("BATTERY_WH_MAX", false);
  if (temp != 0) {
    BATTERY_WH_MAX = temp;
  }
  temp = settings.getUInt("MAXPERCENTAGE", false);
  if (temp != 0) {
    MAXPERCENTAGE = temp;
  }
  temp = settings.getUInt("MINPERCENTAGE", false);
  if (temp != 0) {
    MINPERCENTAGE = temp;
  }
  temp = settings.getUInt("MAXCHARGEAMP", false);
  if (temp != 0) {
    MAXCHARGEAMP = temp;
  }
  temp = settings.getUInt("MAXDISCHARGEAMP", false);
  if (temp != 0) {
    MAXDISCHARGEAMP = temp;
    temp = settings.getBool("USE_SCALED_SOC", false);
    USE_SCALED_SOC = temp;
  }

  settings.end();
}

void init_CAN() {

  pinMode(CAN_SE_PIN, OUTPUT);
  digitalWrite(CAN_SE_PIN, LOW);
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_27;
  CAN_cfg.rx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));

  ESP32Can.CANInit();

#ifdef DUAL_CAN
#ifdef DEBUG_VIA_USB
  Serial.println("Dual CAN Bus (ESP32+MCP2515) selected");
#endif
  gBuffer.initWithSize(25);
  SPI.begin(MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI);
  ACAN2515Settings settings(QUARTZ_FREQUENCY, 500UL * 1000UL);
  settings.mRequestedMode = ACAN2515Settings::NormalMode;
  can.begin(settings, [] { can.isr(); });
#endif

#ifdef CAN_FD
#ifdef DEBUG_VIA_USB
  Serial.println("CAN FD add-on (ESP32+MCP2517) selected");
#endif
  SPI.begin(MCP2517_SCK, MCP2517_SDO, MCP2517_SDI);
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_40MHz, 500 * 1000,
                              DataBitRateFactor::x4);
  settings.mRequestedMode = ACAN2517FDSettings::NormalFD;
  const uint32_t errorCode = canfd.begin(settings, [] { canfd.isr(); });
  if (errorCode == 0) {
#ifdef DEBUG_VIA_USB
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);
    Serial.print("Arbitration Phase segment 1: ");
    Serial.println(settings.mArbitrationPhaseSegment1);
    Serial.print("Arbitration Phase segment 2: ");
    Serial.println(settings.mArbitrationPhaseSegment2);
    Serial.print("Arbitration SJW:");
    Serial.println(settings.mArbitrationSJW);
    Serial.print("Actual Arbitration Bit Rate: ");
    Serial.print(settings.actualArbitrationBitRate());
    Serial.println(" bit/s");
    Serial.print("Exact Arbitration Bit Rate ? ");
    Serial.println(settings.exactArbitrationBitRate() ? "yes" : "no");
    Serial.print("Arbitration Sample point: ");
    Serial.print(settings.arbitrationSamplePointFromBitStart());
    Serial.println("%");
#endif
  } else {
#ifdef DEBUG_VIA_USB
    Serial.print("CAN-FD Configuration error 0x");
    Serial.println(errorCode, HEX);
#endif
    set_event(EVENT_CANFD_INIT_FAILURE, (uint8_t)errorCode);
  }
#endif
}

void init_LED() {

  pixels.begin();
}

void init_contactors() {

#ifdef CONTACTOR_CONTROL
  pinMode(POSITIVE_CONTACTOR_PIN, OUTPUT);
  digitalWrite(POSITIVE_CONTACTOR_PIN, LOW);
  pinMode(NEGATIVE_CONTACTOR_PIN, OUTPUT);
  digitalWrite(NEGATIVE_CONTACTOR_PIN, LOW);
#ifdef PWM_CONTACTOR_CONTROL
  ledcSetup(POSITIVE_PWM_Ch, PWM_Freq, PWM_Res);
  ledcSetup(NEGATIVE_PWM_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(POSITIVE_CONTACTOR_PIN, POSITIVE_PWM_Ch);
  ledcAttachPin(NEGATIVE_CONTACTOR_PIN, NEGATIVE_PWM_Ch);
  ledcWrite(POSITIVE_PWM_Ch, 0);
  ledcWrite(NEGATIVE_PWM_Ch, 0);
#endif
  pinMode(PRECHARGE_PIN, OUTPUT);
  digitalWrite(PRECHARGE_PIN, LOW);
#endif
}

void init_modbus() {
#if defined(BYD_MODBUS) || defined(LUNA2000_MODBUS)

  pinMode(RS485_EN_PIN, OUTPUT);
  digitalWrite(RS485_EN_PIN, HIGH);
  pinMode(RS485_SE_PIN, OUTPUT);
  digitalWrite(RS485_SE_PIN, HIGH);
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, HIGH);

#ifdef BYD_MODBUS

  handle_static_data_modbus_byd();
#endif


  RTUutils::prepareHardwareSerial(Serial2);
  Serial2.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  MBserver.registerWorker(MBTCP_ID, READ_HOLD_REGISTER, &FC03);
  MBserver.registerWorker(MBTCP_ID, WRITE_HOLD_REGISTER, &FC06);
  MBserver.registerWorker(MBTCP_ID, WRITE_MULT_REGISTERS, &FC16);
  MBserver.registerWorker(MBTCP_ID, R_W_MULT_REGISTERS, &FC23);

  MBserver.begin(Serial2, 0);
#endif
}

void inform_user_on_inverter() {

#ifdef BYD_CAN
#ifdef DEBUG_VIA_USB
  Serial.println("BYD CAN protocol selected");
#endif
#endif
#ifdef BYD_MODBUS
#ifdef DEBUG_VIA_USB
  Serial.println("BYD Modbus RTU protocol selected");
#endif
#endif
#ifdef LUNA2000_MODBUS
#ifdef DEBUG_VIA_USB
  Serial.println("Luna2000 Modbus RTU protocol selected");
#endif
#endif
#ifdef PYLON_CAN
#ifdef DEBUG_VIA_USB
  Serial.println("PYLON CAN protocol selected");
#endif
#endif
#ifdef SMA_CAN
#ifdef DEBUG_VIA_USB
  Serial.println("SMA CAN protocol selected");
#endif
#endif
#ifdef SMA_TRIPOWER_CAN
#ifdef DEBUG_VIA_USB
  Serial.println("SMA Tripower CAN protocol selected");
#endif
#endif
#ifdef SOFAR_CAN
#ifdef DEBUG_VIA_USB
  Serial.println("SOFAR CAN protocol selected");
#endif
#endif
#ifdef SOLAX_CAN
  inverterAllowsContactorClosing = false;
  intervalUpdateValues = 800;
#ifdef DEBUG_VIA_USB
  Serial.println("SOLAX CAN protocol selected");
#endif
#endif
}

void init_battery() {

  setup_battery();

#ifndef BATTERY_SELECTED
#error No battery selected! Choose one from the USER_SETTINGS.h file
#endif
}

#ifdef CAN_FD

void receive_canfd() {
  CANFDMessage frame;
  while (canfd.available()) {
    canfd.receive(frame);
    if(frame.id == 0x7EC){
      WebSerial.println("received Frame 7ec");
    }
    receive_canfd_battery(frame);
  }
}
#endif

void receive_can() {

  CAN_frame_t rx_frame;
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
    if (rx_frame.FIR.B.FF == CAN_frame_std) {

#ifndef SERIAL_LINK_RECEIVER
      receive_can_battery(rx_frame);
#endif

#ifdef BYD_CAN
      receive_can_byd(rx_frame);
#endif
#ifdef SMA_CAN
      receive_can_sma(rx_frame);
#endif
#ifdef SMA_TRIPOWER_CAN
      receive_can_sma_tripower(rx_frame);
#endif

#ifdef CHEVYVOLT_CHARGER
      receive_can_chevyvolt_charger(rx_frame);
#endif
#ifdef NISSANLEAF_CHARGER
      receive_can_nissanleaf_charger(rx_frame);
#endif
    } else {
#ifdef PYLON_CAN
      receive_can_pylon(rx_frame);
#endif
#ifdef SOFAR_CAN
      receive_can_sofar(rx_frame);
#endif
#ifdef SOLAX_CAN
      receive_can_solax(rx_frame);
#endif
    }
  }
}

void send_can() {


#ifdef BYD_CAN
  send_can_byd();
#endif
#ifdef SMA_CAN
  send_can_sma();
#endif
#ifdef SMA_TRIPOWER_CAN
  send_can_sma_tripower();
#endif
#ifdef SOFAR_CAN
  send_can_sofar();
#endif

  send_can_battery();

#ifdef CHEVYVOLT_CHARGER
  send_can_chevyvolt_charger();
#endif
#ifdef NISSANLEAF_CHARGER
  send_can_nissanleaf_charger();
#endif
}

#ifdef DUAL_CAN
void receive_can2() {

  CAN_frame_t rx_frame2;
  CANMessage MCP2515Frame;

  if (can.available()) {
    can.receive(MCP2515Frame);

    rx_frame2.MsgID = MCP2515Frame.id;
    rx_frame2.FIR.B.FF = MCP2515Frame.ext ? CAN_frame_ext : CAN_frame_std;
    rx_frame2.FIR.B.RTR = MCP2515Frame.rtr ? CAN_RTR : CAN_no_RTR;
    rx_frame2.FIR.B.DLC = MCP2515Frame.len;
    for (uint8_t i = 0; i < MCP2515Frame.len; i++) {
      rx_frame2.data.u8[i] = MCP2515Frame.data[i];
    }

    if (rx_frame2.FIR.B.FF == CAN_frame_std) {
#ifdef BYD_CAN
      receive_can_byd(rx_frame2);
#endif
    } else {
#ifdef PYLON_CAN
      receive_can_pylon(rx_frame2);
#endif
#ifdef SOLAX_CAN
      receive_can_solax(rx_frame2);
#endif
    }
  }
}

void send_can2() {


#ifdef BYD_CAN
  send_can_byd();
#endif
}
#endif

void handle_LED_state() {

  if (rampUp && brightness < maxBrightness) {
    brightness++;
  } else if (rampUp && brightness == maxBrightness) {
    rampUp = false;
  } else if (!rampUp && brightness > 0) {
    brightness--;
  } else if (!rampUp && brightness == 0) {
    rampUp = true;
  }

  switch (get_event_level()) {
    case EVENT_LEVEL_INFO:
      LEDcolor = GREEN;
      pixels.setPixelColor(0, pixels.Color(0, brightness, 0));
      break;
    case EVENT_LEVEL_WARNING:
      LEDcolor = YELLOW;
      pixels.setPixelColor(0, pixels.Color(brightness, brightness, 0));
      break;
    case EVENT_LEVEL_DEBUG:
    case EVENT_LEVEL_UPDATE:
      LEDcolor = BLUE;
      pixels.setPixelColor(0, pixels.Color(0, 0, brightness));
      break;
    case EVENT_LEVEL_ERROR:
      LEDcolor = RED;
      pixels.setPixelColor(0, pixels.Color(150, 0, 0));
      break;
    default:
      break;
  }


  if (digitalRead(0) == LOW) {
    pixels.setPixelColor(0, pixels.Color(brightness, abs((100 - brightness)), abs((50 - brightness))));
  }

  pixels.show();
}

#ifdef CONTACTOR_CONTROL
void handle_contactors() {

  if (system_bms_status == FAULT) {
    timeSpentInFaultedMode++;
  } else {
    timeSpentInFaultedMode = 0;
  }

  if (timeSpentInFaultedMode > MAX_ALLOWED_FAULT_TICKS) {
    contactorStatus = SHUTDOWN_REQUESTED;
  }
  if (contactorStatus == SHUTDOWN_REQUESTED) {
    digitalWrite(PRECHARGE_PIN, LOW);
    digitalWrite(NEGATIVE_CONTACTOR_PIN, LOW);
    digitalWrite(POSITIVE_CONTACTOR_PIN, LOW);
    return;
  }


  if (contactorStatus == DISCONNECTED) {
    digitalWrite(PRECHARGE_PIN, LOW);
#ifdef PWM_CONTACTOR_CONTROL
    ledcWrite(POSITIVE_PWM_Ch, 0);
    ledcWrite(NEGATIVE_PWM_Ch, 0);
#endif

    if (batteryAllowsContactorClosing && inverterAllowsContactorClosing) {
      contactorStatus = PRECHARGE;
    }
  }


  if (contactorStatus == COMPLETED) {
    if (!inverterAllowsContactorClosing)
      contactorStatus = DISCONNECTED;

    return;
  }

  unsigned long currentTime = millis();

  switch (contactorStatus) {
    case PRECHARGE:
      digitalWrite(PRECHARGE_PIN, HIGH);
      prechargeStartTime = currentTime;
      contactorStatus = NEGATIVE;
      break;

    case NEGATIVE:
      if (currentTime - prechargeStartTime >= PRECHARGE_TIME_MS) {
        digitalWrite(NEGATIVE_CONTACTOR_PIN, HIGH);
#ifdef PWM_CONTACTOR_CONTROL
        ledcWrite(NEGATIVE_PWM_Ch, 1023);
#endif
        negativeStartTime = currentTime;
        contactorStatus = POSITIVE;
      }
      break;

    case POSITIVE:
      if (currentTime - negativeStartTime >= NEGATIVE_CONTACTOR_TIME_MS) {
        digitalWrite(POSITIVE_CONTACTOR_PIN, HIGH);
#ifdef PWM_CONTACTOR_CONTROL
        ledcWrite(POSITIVE_PWM_Ch, 1023);
#endif
        contactorStatus = PRECHARGE_OFF;
      }
      break;

    case PRECHARGE_OFF:
      if (currentTime - negativeStartTime >= POSITIVE_CONTACTOR_TIME_MS) {
        digitalWrite(PRECHARGE_PIN, LOW);
#ifdef PWM_CONTACTOR_CONTROL
        ledcWrite(NEGATIVE_PWM_Ch, PWM_Hold_Duty);
        ledcWrite(POSITIVE_PWM_Ch, PWM_Hold_Duty);
#endif
        contactorStatus = COMPLETED;
      }
      break;
    default:
      break;
  }
}
#endif

void update_SOC() {
  if (USE_SCALED_SOC) {
    static int16_t CalculatedSOC = 0;
    CalculatedSOC = system_real_SOC_pptt;
    CalculatedSOC = (10000) * (CalculatedSOC - (MINPERCENTAGE * 10)) / (MAXPERCENTAGE * 10 - MINPERCENTAGE * 10);
    if (CalculatedSOC < 0) {
      CalculatedSOC = 0;
    }
    if (CalculatedSOC > 10000) {
      CalculatedSOC = 10000;
    }
    system_scaled_SOC_pptt = CalculatedSOC;
  } else {
    system_scaled_SOC_pptt = system_real_SOC_pptt;
  }
}

void update_values() {

  update_values_battery();

#ifdef BYD_CAN
  update_values_can_byd();
#endif
#ifdef BYD_MODBUS
  update_modbus_registers_byd();
#endif
#ifdef LUNA2000_MODBUS
  update_modbus_registers_luna2000();
#endif
#ifdef PYLON_CAN
  update_values_can_pylon();
#endif
#ifdef SMA_CAN
  update_values_can_sma();
#endif
#ifdef SMA_TRIPOWER_CAN
  update_values_can_sma_tripower();
#endif
#ifdef SOFAR_CAN
  update_values_can_sofar();
#endif
#ifdef SOLAX_CAN
  update_values_can_solax();
#endif
}

#if defined(SERIAL_LINK_RECEIVER) || defined(SERIAL_LINK_TRANSMITTER)
void runSerialDataLink() {
  static unsigned long updateTime = 0;
  unsigned long currentMillis = millis();

  if ((currentMillis - updateTime) > 1) {
    updateTime = currentMillis;
#ifdef SERIAL_LINK_RECEIVER
    manageSerialLinkReceiver();
#endif
#ifdef SERIAL_LINK_TRANSMITTER
    manageSerialLinkTransmitter();
#endif
  }
}
#endif

void init_serialDataLink() {
#if defined(SERIAL_LINK_RECEIVER) || defined(SERIAL_LINK_TRANSMITTER)
  Serial2.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
#endif
}

void storeSettings() {
  settings.begin("batterySettings", false);
  settings.putUInt("BATTERY_WH_MAX", BATTERY_WH_MAX);
  settings.putUInt("MAXPERCENTAGE", MAXPERCENTAGE);
  settings.putUInt("MINPERCENTAGE", MINPERCENTAGE);
  settings.putUInt("MAXCHARGEAMP", MAXCHARGEAMP);
  settings.putUInt("MAXDISCHARGEAMP", MAXDISCHARGEAMP);
  settings.putBool("USE_SCALED_SOC", USE_SCALED_SOC);

  settings.end();
}