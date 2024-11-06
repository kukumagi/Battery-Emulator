#include "../include.h"
#ifdef BYD_CAN
#include "../datalayer/datalayer.h"
#include "BYD-CAN.h"

/* Do not change code below unless you are sure what you are doing */
static unsigned long previousMillis2s = 0;   // will store last time a 2s CAN Message was send
static unsigned long previousMillis10s = 0;  // will store last time a 10s CAN Message was send
static unsigned long previousMillis60s = 0;  // will store last time a 60s CAN Message was send
static uint8_t char1_151 = 0;
static uint8_t char2_151 = 0;
static uint8_t char3_151 = 0;
static uint8_t char4_151 = 0;
static uint8_t char5_151 = 0;
static uint8_t char6_151 = 0;
static uint8_t char7_151 = 0;

CAN_frame BYD_250 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x250,
                     .data = {FW_MAJOR_VERSION, FW_MINOR_VERSION, 0x00, 0x66, (uint8_t)((BATTERY_WH_MAX / 100) >> 8),
                              (uint8_t)(BATTERY_WH_MAX / 100), 0x02,
                              0x09}};  //0-1 FW version , Capacity kWh byte4&5 (example 24kWh = 240)
CAN_frame BYD_290 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x290,
                     .data = {0x06, 0x37, 0x10, 0xD9, 0x00, 0x00, 0x00, 0x00}};
CAN_frame BYD_2D0 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x2D0,
                     .data = {0x00, 0x42, 0x59, 0x44, 0x00, 0x00, 0x00, 0x00}};  //BYD
CAN_frame BYD_3D0_0 = {.FD = false,
                       .ext_ID = false,
                       .DLC = 8,
                       .ID = 0x3D0,
                       .data = {0x00, 0x42, 0x61, 0x74, 0x74, 0x65, 0x72, 0x79}};  //Battery
CAN_frame BYD_3D0_1 = {.FD = false,
                       .ext_ID = false,
                       .DLC = 8,
                       .ID = 0x3D0,
                       .data = {0x01, 0x2D, 0x42, 0x6F, 0x78, 0x20, 0x50, 0x72}};  //-Box Pr
CAN_frame BYD_3D0_2 = {.FD = false,
                       .ext_ID = false,
                       .DLC = 8,
                       .ID = 0x3D0,
                       .data = {0x02, 0x65, 0x6D, 0x69, 0x75, 0x6D, 0x20, 0x48}};  //emium H
CAN_frame BYD_3D0_3 = {.FD = false,
                       .ext_ID = false,
                       .DLC = 8,
                       .ID = 0x3D0,
                       .data = {0x03, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00}};  //VS
//Actual content messages
CAN_frame BYD_110 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x110,
                     .data = {0x01, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame BYD_150 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x150,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00}};
CAN_frame BYD_190 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x190,
                     .data = {0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}};
CAN_frame BYD_1D0 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x1D0,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x08}};
CAN_frame BYD_210 = {.FD = false,
                     .ext_ID = false,
                     .DLC = 8,
                     .ID = 0x210,
                     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static uint16_t discharge_current = 0;
static uint16_t charge_current = 0;
static int16_t temperature_average = 0;
static uint16_t inverter_voltage = 0;
static uint16_t inverter_SOC = 0;
static uint16_t remaining_capacity_ah = 0;
static uint16_t fully_charged_capacity_ah = 0;
static long inverter_timestamp = 0;
static bool initialDataSent = 0;

void update_values_can_inverter() {  //This function maps all the values fetched from battery CAN to the correct CAN messages

  /* Calculate allowed charge/discharge currents*/
  if (datalayer.battery.status.voltage_dV > 10) {  // Only update value when we have voltage available to avoid div0
    charge_current =
        ((datalayer.battery.status.max_charge_power_W * 10) /
         datalayer.battery.status.voltage_dV);  //Charge power in W , max volt in V+1decimal (P=UI, solve for I)
    //The above calculation results in (30 000*10)/3700=81A
    charge_current = (charge_current * 10);  //Value needs a decimal before getting sent to inverter (81.0A)

    discharge_current =
        ((datalayer.battery.status.max_discharge_power_W * 10) /
         datalayer.battery.status.voltage_dV);  //Charge power in W , max volt in V+1decimal (P=UI, solve for I)
    //The above calculation results in (30 000*10)/3700=81A
    discharge_current = (discharge_current * 10);  //Value needs a decimal before getting sent to inverter (81.0A)
  }
  /* Restrict values from user settings if needed*/
  if (charge_current > datalayer.battery.info.max_charge_amp_dA) {
    charge_current =
        datalayer.battery.info
            .max_charge_amp_dA;  //Cap the value to the max allowed Amp. Some inverters cannot handle large values.
  }
  if (discharge_current > datalayer.battery.info.max_discharge_amp_dA) {
    discharge_current =
        datalayer.battery.info
            .max_discharge_amp_dA;  //Cap the value to the max allowed Amp. Some inverters cannot handle large values.
  }

  /* Calculate temperature */
  temperature_average =
      ((datalayer.battery.status.temperature_max_dC + datalayer.battery.status.temperature_min_dC) / 2);

  /* Calculate capacity, Amp hours(Ah) = Watt hours (Wh) / Voltage (V)*/
  if (datalayer.battery.status.voltage_dV > 10) {  // Only update value when we have voltage available to avoid div0
    remaining_capacity_ah =
        ((datalayer.battery.status.reported_remaining_capacity_Wh / datalayer.battery.status.voltage_dV) * 100);
    fully_charged_capacity_ah =
        ((datalayer.battery.info.total_capacity_Wh / datalayer.battery.status.voltage_dV) * 100);
  }

  //Map values to CAN messages
  //Maxvoltage (eg 400.0V = 4000 , 16bits long)
  BYD_110.data.u8[0] = (datalayer.battery.info.max_design_voltage_dV >> 8);
  BYD_110.data.u8[1] = (datalayer.battery.info.max_design_voltage_dV & 0x00FF);
  //Minvoltage (eg 300.0V = 3000 , 16bits long)
  BYD_110.data.u8[2] = (datalayer.battery.info.min_design_voltage_dV >> 8);
  BYD_110.data.u8[3] = (datalayer.battery.info.min_design_voltage_dV & 0x00FF);
  //Maximum discharge power allowed (Unit: A+1)
  BYD_110.data.u8[4] = (discharge_current >> 8);
  BYD_110.data.u8[5] = (discharge_current & 0x00FF);
  //Maximum charge power allowed (Unit: A+1)
  BYD_110.data.u8[6] = (charge_current >> 8);
  BYD_110.data.u8[7] = (charge_current & 0x00FF);

  //SOC (100.00%)
  BYD_150.data.u8[0] = (datalayer.battery.status.reported_soc >> 8);
  BYD_150.data.u8[1] = (datalayer.battery.status.reported_soc & 0x00FF);
  //StateOfHealth (100.00%)
  BYD_150.data.u8[2] = (datalayer.battery.status.soh_pptt >> 8);
  BYD_150.data.u8[3] = (datalayer.battery.status.soh_pptt & 0x00FF);
  //Remaining capacity (Ah+1)
  BYD_150.data.u8[4] = (remaining_capacity_ah >> 8);
  BYD_150.data.u8[5] = (remaining_capacity_ah & 0x00FF);
  //Fully charged capacity (Ah+1)
  BYD_150.data.u8[6] = (fully_charged_capacity_ah >> 8);
  BYD_150.data.u8[7] = (fully_charged_capacity_ah & 0x00FF);

  //Voltage (ex 370.0)
  BYD_1D0.data.u8[0] = (datalayer.battery.status.voltage_dV >> 8);
  BYD_1D0.data.u8[1] = (datalayer.battery.status.voltage_dV & 0x00FF);
  //Current (ex 81.0A)
  BYD_1D0.data.u8[2] = (datalayer.battery.status.current_dA >> 8);
  BYD_1D0.data.u8[3] = (datalayer.battery.status.current_dA & 0x00FF);
  //Temperature average
  BYD_1D0.data.u8[4] = (temperature_average >> 8);
  BYD_1D0.data.u8[5] = (temperature_average & 0x00FF);

  //Temperature max
  BYD_210.data.u8[0] = (datalayer.battery.status.temperature_max_dC >> 8);
  BYD_210.data.u8[1] = (datalayer.battery.status.temperature_max_dC & 0x00FF);
  //Temperature min
  BYD_210.data.u8[2] = (datalayer.battery.status.temperature_min_dC >> 8);
  BYD_210.data.u8[3] = (datalayer.battery.status.temperature_min_dC & 0x00FF);

#ifdef DEBUG_VIA_USB
  if (char1_151 != 0) {
    Serial.print("Detected inverter: ");
    Serial.print((char)char1_151);
    Serial.print((char)char2_151);
    Serial.print((char)char3_151);
    Serial.print((char)char4_151);
    Serial.print((char)char5_151);
    Serial.print((char)char6_151);
    Serial.println((char)char7_151);
  }
#endif
}

void receive_can_inverter(CAN_frame rx_frame) {
  switch (rx_frame.ID) {
    case 0x151:  //Message originating from BYD HVS compatible inverter. Reply with CAN identifier!
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      if (rx_frame.data.u8[0] & 0x01) {  //Battery requests identification
        send_intial_data();
      } else {  // We can identify what inverter type we are connected to
        char1_151 = rx_frame.data.u8[1];
        char2_151 = rx_frame.data.u8[2];
        char3_151 = rx_frame.data.u8[3];
        char4_151 = rx_frame.data.u8[4];
        char5_151 = rx_frame.data.u8[5];
        char6_151 = rx_frame.data.u8[6];
        char7_151 = rx_frame.data.u8[7];
      }
      break;
    case 0x091:
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      inverter_voltage = ((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]) * 0.1;
      break;
    case 0x0D1:
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      inverter_SOC = ((rx_frame.data.u8[1] << 8) | rx_frame.data.u8[0]) * 0.1;
      break;
    case 0x111:
      datalayer.system.status.CAN_inverter_still_alive = CAN_STILL_ALIVE;
      inverter_timestamp = ((rx_frame.data.u8[3] << 24) | (rx_frame.data.u8[2] << 16) | (rx_frame.data.u8[1] << 8) |
                            rx_frame.data.u8[0]);
      break;
    default:
      break;
  }
}

void send_can_inverter() {
  unsigned long currentMillis = millis();
  // Send initial CAN data once on bootup
  if (!initialDataSent) {
    send_intial_data();
    initialDataSent = 1;
  }

  // Send 2s CAN Message
  if (currentMillis - previousMillis2s >= INTERVAL_2_S) {
    previousMillis2s = currentMillis;

    transmit_can(&BYD_110, can_config.inverter);
  }
  // Send 10s CAN Message
  if (currentMillis - previousMillis10s >= INTERVAL_10_S) {
    previousMillis10s = currentMillis;

    transmit_can(&BYD_150, can_config.inverter);
    transmit_can(&BYD_1D0, can_config.inverter);
    transmit_can(&BYD_210, can_config.inverter);
  }
  //Send 60s message
  if (currentMillis - previousMillis60s >= INTERVAL_60_S) {
    previousMillis60s = currentMillis;

    transmit_can(&BYD_190, can_config.inverter);
  }
}

void send_intial_data() {
  transmit_can(&BYD_250, can_config.inverter);
  transmit_can(&BYD_290, can_config.inverter);
  transmit_can(&BYD_2D0, can_config.inverter);
  transmit_can(&BYD_3D0_0, can_config.inverter);
  transmit_can(&BYD_3D0_1, can_config.inverter);
  transmit_can(&BYD_3D0_2, can_config.inverter);
  transmit_can(&BYD_3D0_3, can_config.inverter);
}
#endif
