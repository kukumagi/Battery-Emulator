#include "BATTERIES.h"
#ifdef KIA_HYUNDAI_E_GMP_BATTERY
#include "../devboard/utils/events.h"
#include "../lib/miwagner-ESP32-Arduino-CAN/CAN_config.h"
#include "../lib/miwagner-ESP32-Arduino-CAN/ESP32CAN.h"
#include "../lib/perremolinaro-ACAN2517/ACAN2517FD.h"
#include "KIA-HYUNDAI-E-GMP-BATTERY.h"

/* Do not change code below unless you are sure what you are doing */
static unsigned long previousMillis100 = 0;   // will store last time a 100ms CAN Message was send
static unsigned long previousMillis10ms = 0;  // will store last time a 10s CAN Message was send
static const int interval100 = 100;           // interval (ms) at which send CAN Messages
static const int interval10ms = 10;           // interval (ms) at which send CAN Messages
static uint8_t CANstillAlive = 12;            //counter for checking if CAN is still alive

#define MAX_CELL_VOLTAGE 4250   //Battery is put into emergency stop if one cell goes over this value
#define MIN_CELL_VOLTAGE 2950   //Battery is put into emergency stop if one cell goes below this value
#define MAX_CELL_DEVIATION 150  //LED turns yellow on the board if mv delta exceeds this value

static uint16_t soc_calculated = 0;
static uint16_t SOC_BMS = 0;
static uint16_t SOC_Display = 0;
static uint16_t batterySOH = 1000;
static uint8_t waterleakageSensor = 164;
static int16_t leadAcidBatteryVoltage = 0;
static uint16_t CellVoltMax_mV = 3700;
static uint8_t CellVmaxNo = 0;
static uint16_t CellVoltMin_mV = 3700;
static uint8_t CellVminNo = 0;
static uint16_t cell_deviation_mV = 0;
static int16_t allowedDischargePower = 0;
static int16_t allowedChargePower = 0;
static uint16_t batteryVoltage = 0;
static int16_t batteryAmps = 0;
static int16_t powerWatt = 0;
static int16_t temperatureMax = 0;
static int16_t temperatureMin = 0;
static int8_t temperature_water_inlet = 0;
static uint8_t batteryManagementMode = 0;
static uint8_t BMS_ign = 0;
static int16_t poll_data_pid = 0;
static int8_t heatertemp = 0;
static uint8_t batteryRelay = 0;
static int8_t powerRelayTemperature = 0;
static uint16_t inverterVoltageFrameHigh = 0;
static uint16_t inverterVoltage = 0;
static uint8_t startedUp = false;
static uint8_t counter_200 = 0;

CAN_frame_t KIA_HYUNDAI_200 = {.FIR = {.B =
                                           {
                                               .DLC = 8,
                                               .FF = CAN_frame_std,
                                           }},
                               .MsgID = 0x200,
                               //.data = {0x00, 0x00, 0x00, 0x04, 0x00, 0x50, 0xD0, 0x00}}; //Initial value
                               .data = {0x00, 0x80, 0xD8, 0x04, 0x00, 0x17, 0xD0, 0x00}};  //Mid log value
CAN_frame_t KIA_HYUNDAI_523 = {.FIR = {.B =
                                           {
                                               .DLC = 8,
                                               .FF = CAN_frame_std,
                                           }},
                               .MsgID = 0x523,
                               //.data = {0x00, 0x38, 0x28, 0x28, 0x28, 0x28, 0x00, 0x01}}; //Initial value
                               .data = {0x08, 0x38, 0x36, 0x36, 0x33, 0x34, 0x00, 0x01}};  //Mid log value
CAN_frame_t KIA_HYUNDAI_524 = {.FIR = {.B =
                                           {
                                               .DLC = 8,
                                               .FF = CAN_frame_std,
                                           }},
                               .MsgID = 0x524,
                               .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};  //Initial value

//553 Needed frame 200ms
CAN_frame_t KIA64_553 = {.FIR = {.B =
                                     {
                                         .DLC = 8,
                                         .FF = CAN_frame_std,
                                     }},
                         .MsgID = 0x553,
                         .data = {0x04, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00}};
//57F Needed frame 100ms
CAN_frame_t KIA64_57F = {.FIR = {.B =
                                     {
                                         .DLC = 8,
                                         .FF = CAN_frame_std,
                                     }},
                         .MsgID = 0x57F,
                         .data = {0x80, 0x0A, 0x72, 0x00, 0x00, 0x00, 0x00, 0x72}};
//Needed frame 100ms
CAN_frame_t KIA64_2A1 = {.FIR = {.B =
                                     {
                                         .DLC = 8,
                                         .FF = CAN_frame_std,
                                     }},
                         .MsgID = 0x2A1,
                         .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

CAN_frame_t EGMP_7E4 = {.FIR =
  {.B =
    {
        .DLC = 8,
        .FF = CAN_frame_std,
    }
  },
  .MsgID = 0x7E4,
  .data = {0x03, 0x22, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00}
};  //Poll PID 03 22 01 01 


CAN_frame_t KIA64_7E4_ack = {
    .FIR = {.B =
                {
                    .DLC = 8,
                    .FF = CAN_frame_std,
                }},
    .MsgID = 0x7E4,
    .data = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};  //Ack frame, correct PID is returned

void update_values_battery() {  //This function maps all the values fetched via CAN to the correct parameters used for modbus

  system_real_SOC_pptt = (SOC_Display * 10);  //increase SOC range from 0-100.0 -> 100.00

  system_SOH_pptt = (batterySOH * 10);  //Increase decimals from 100.0% -> 100.00%

  system_battery_voltage_dV = batteryVoltage;  //value is *10 (3700 = 370.0)

  system_battery_current_dA = batteryAmps;  //value is *10 (150 = 15.0)

  system_capacity_Wh = BATTERY_WH_MAX;

  system_remaining_capacity_Wh = static_cast<int>((static_cast<double>(system_real_SOC_pptt) / 10000) * BATTERY_WH_MAX);

  //system_max_charge_power_W = (uint16_t)allowedChargePower * 10;  //From kW*100 to Watts
  //The allowed charge power is not available. We estimate this value
  if (system_scaled_SOC_pptt == 10000) {  // When scaled SOC is 100%, set allowed charge power to 0
    system_max_charge_power_W = 0;
  } else {  // No limits, max charging power allowed
    system_max_charge_power_W = MAXCHARGEPOWERALLOWED;
  }
  //system_max_discharge_power_W = (uint16_t)allowedDischargePower * 10;  //From kW*100 to Watts
  if (system_scaled_SOC_pptt < 100) {  // When scaled SOC is <1%, set allowed charge power to 0
    system_max_discharge_power_W = 0;
  } else {  // No limits, max charging power allowed
    system_max_discharge_power_W = MAXDISCHARGEPOWERALLOWED;
  }

  powerWatt = ((batteryVoltage * batteryAmps) / 100);

  system_active_power_W = powerWatt;  //Power in watts, Negative = charging batt

  system_temperature_min_dC = (int8_t)temperatureMin * 10;  //Increase decimals, 17C -> 17.0C

  system_temperature_max_dC = (int8_t)temperatureMax * 10;  //Increase decimals, 18C -> 18.0C

  system_cell_max_voltage_mV = CellVoltMax_mV;

  system_cell_min_voltage_mV = CellVoltMin_mV;

  /* Check if the BMS is still sending CAN messages. If we go 60s without messages we raise an error*/
  if (!CANstillAlive) {
    set_event(EVENT_CAN_RX_FAILURE, 0);
  } else {
    CANstillAlive--;
    clear_event(EVENT_CAN_RX_FAILURE);
  }

  if (waterleakageSensor == 0) {
    set_event(EVENT_WATER_INGRESS, 0);
  }

  if (leadAcidBatteryVoltage < 110) {
    set_event(EVENT_12V_LOW, leadAcidBatteryVoltage);
  }

  // Check if cell voltages are within allowed range
  cell_deviation_mV = (system_cell_max_voltage_mV - system_cell_min_voltage_mV);

  if (CellVoltMax_mV >= MAX_CELL_VOLTAGE) {
    set_event(EVENT_CELL_OVER_VOLTAGE, 0);
  }
  if (CellVoltMin_mV <= MIN_CELL_VOLTAGE) {
    set_event(EVENT_CELL_UNDER_VOLTAGE, 0);
  }
  if (cell_deviation_mV > MAX_CELL_DEVIATION) {
    set_event(EVENT_CELL_DEVIATION_HIGH, 0);
  } else {
    clear_event(EVENT_CELL_DEVIATION_HIGH);
  }

  if (system_bms_status == FAULT) {  //Incase we enter a critical fault state, zero out the allowed limits
    system_max_charge_power_W = 0;
    system_max_discharge_power_W = 0;
  }

  /* Safeties verified. Perform USB serial printout if configured to do so */

#ifdef DEBUG_VIA_USB
  Serial.println();  //sepatator
  Serial.println("Values from battery: ");
  Serial.print("SOC BMS: ");
  Serial.print((uint16_t)SOC_BMS / 10.0, 1);
  Serial.print("%  |  SOC Display: ");
  Serial.print((uint16_t)SOC_Display / 10.0, 1);
  Serial.print("%  |  SOH ");
  Serial.print((uint16_t)batterySOH / 10.0, 1);
  Serial.println("%");
  Serial.print((int16_t)batteryAmps / 10.0, 1);
  Serial.print(" Amps  |  ");
  Serial.print((uint16_t)batteryVoltage / 10.0, 1);
  Serial.print(" Volts  |  ");
  Serial.print((int16_t)system_active_power_W);
  Serial.println(" Watts");
  Serial.print("Allowed Charge ");
  Serial.print((uint16_t)allowedChargePower * 10);
  Serial.print(" W  |  Allowed Discharge ");
  Serial.print((uint16_t)allowedDischargePower * 10);
  Serial.println(" W");
  Serial.print("MaxCellVolt ");
  Serial.print(CellVoltMax_mV);
  Serial.print(" mV  No  ");
  Serial.print(CellVmaxNo);
  Serial.print("  |  MinCellVolt ");
  Serial.print(CellVoltMin_mV);
  Serial.print(" mV  No  ");
  Serial.println(CellVminNo);
  Serial.print("TempHi ");
  Serial.print((int16_t)temperatureMax);
  Serial.print("°C  TempLo ");
  Serial.print((int16_t)temperatureMin);
  Serial.print("°C  WaterInlet ");
  Serial.print((int8_t)temperature_water_inlet);
  Serial.print("°C  PowerRelay ");
  Serial.print((int8_t)powerRelayTemperature * 2);
  Serial.println("°C");
  Serial.print("Aux12volt: ");
  Serial.print((int16_t)leadAcidBatteryVoltage / 10.0, 1);
  Serial.println("V  |  ");
  Serial.print("BmsManagementMode ");
  Serial.print((uint8_t)batteryManagementMode, BIN);
  if (bitRead((uint8_t)BMS_ign, 2) == 1) {
    Serial.print("  |  BmsIgnition ON");
  } else {
    Serial.print("  |  BmsIgnition OFF");
  }

  if (bitRead((uint8_t)batteryRelay, 0) == 1) {
    Serial.print("  |  PowerRelay ON");
  } else {
    Serial.print("  |  PowerRelay OFF");
  }
  Serial.print("  |  Inverter ");
  Serial.print(inverterVoltage);
  Serial.println(" Volts");
#endif
}
void CAN_FD_WriteFrame(CAN_frame_t* tx_frame) {
  CANFDMessage frame;
  frame.id = tx_frame->MsgID;
  frame.ext = tx_frame->FIR.B.FF;
  frame.len = tx_frame->FIR.B.DLC;
  for (uint8_t i = 0; i < frame.len; i++) {
    frame.data[i] = tx_frame->data.u8[i];
  }
  const bool ok = canFD.tryToSend(frame);
    // Serial.println ("Send frame");

  // printFrame(frame);
  if (ok) {
    // Serial.println ("Send ok") ;
  }else{
    Serial.println ("Send failure");
  }

}
void printFrame(CANFDMessage rx_frame) {
  int i = 0;
  Serial.print(rx_frame.id,HEX);
  Serial.print(" ");
  for(i = 0;i < rx_frame.len; i++) {
    Serial.print(rx_frame.data[i],HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
}
void set_cell_voltages(CANFDMessage rx_frame, int start, int length, int startCell) {
  for (size_t i = 0; i < length; i++)
  {
    system_cellvoltages_mV[startCell + i] = (rx_frame.data[start + i] / 50);
  }
  
}
void receive_canFD_battery(CANFDMessage rx_frame) {
    // id (inMessage.id),  // Frame identifier
//   ext (inMessage.ext), // false -> base frame, true -> extended frame
//   type (inMessage.rtr ? CAN_REMOTE : CAN_DATA),
//   idx (inMessage.idx),  // This field is used by the driver
//   len (inMessage.len), // Length of data (0 ... 64)
//   data () {
//     data64 [0] = inMessage.data64 ;
//   }
  // printFrame(rx_frame);
  switch (rx_frame.id) {
    // case 0x4DE:
    //   break;
    // case 0x542:                               //BMS SOC
    //   CANstillAlive = 12;                     //We use this message to verify that BMS is still alive
    //   SOC_Display = rx_frame.data[0] * 5;  //100% = 200 ( 200 * 5 = 1000 )
    //   break;
    // case 0x594:
    //   SOC_BMS = rx_frame.data[5] * 5;  //100% = 200 ( 200 * 5 = 1000 )
    //   break;
    // case 0x595:
    //   batteryVoltage = (rx_frame.data[7] << 8) + rx_frame.data[6];
    //   batteryAmps = (rx_frame.data[5] << 8) + rx_frame.data[4];
    //   if (counter_200 > 3) {
    //     KIA_HYUNDAI_524.data.u8[0] = (uint8_t)(batteryVoltage / 10);
    //     KIA_HYUNDAI_524.data.u8[1] = (uint8_t)((batteryVoltage / 10) >> 8);
    //   }  //VCU measured voltage sent back to bms
    //   break;
    // case 0x596:
    //   leadAcidBatteryVoltage = rx_frame.data[1];  //12v Battery Volts
    //   temperatureMin = rx_frame.data[6];          //Lowest temp in battery
    //   temperatureMax = rx_frame.data[7];          //Highest temp in battery
    //   break;
    // case 0x598:
    //   break;
    // case 0x5D5:
    //   waterleakageSensor = rx_frame.data[3];  //Water sensor inside pack, value 164 is no water --> 0 is short
    //   powerRelayTemperature = rx_frame.data[7];
    //   break;
    // case 0x5D8:
    //   startedUp = 1;

    //   //PID data is polled after last message sent from battery:
    //   if (poll_data_pid >= 10) {  //polling one of ten PIDs at 100ms, resolution = 1s
    //     poll_data_pid = 0;
    //   }
    //   poll_data_pid++;
    //   if (poll_data_pid == 1) {
    //     CAN_FD_WriteFrame(&KIA64_7E4_id1);
    //   } else if (poll_data_pid == 2) {
    //     CAN_FD_WriteFrame(&KIA64_7E4_id2);
    //   } else if (poll_data_pid == 3) {
    //     CAN_FD_WriteFrame(&KIA64_7E4_id3);
    //   } else if (poll_data_pid == 4) {
    //     CAN_FD_WriteFrame(&KIA64_7E4_id4);
    //   } else if (poll_data_pid == 5) {
    //     CAN_FD_WriteFrame(&KIA64_7E4_id5);
    //   } else if (poll_data_pid == 6) {
    //     CAN_FD_WriteFrame(&KIA64_7E4_id6);
    //   } else if (poll_data_pid == 7) {
    //   } else if (poll_data_pid == 8) {
    //   } else if (poll_data_pid == 9) {
    //   } else if (poll_data_pid == 10) {
    //   }
    //   break;
    case 0x7EC:
      printFrame(rx_frame);
      switch (rx_frame.data[0]) {
        case 0x10:  //"PID Header"
          // Serial.println ("Send ack");
          poll_data_pid = rx_frame.data[4];
          // if (rx_frame.data[4] == poll_data_pid) {
            CAN_FD_WriteFrame(&KIA64_7E4_ack);  //Send ack to BMS if the same frame is sent as polled
          // }
          break;
        case 0x21:  //First frame in PID group
          if (poll_data_pid == 1) {
            allowedChargePower = ((rx_frame.data[3] << 8) + rx_frame.data[4]);
            allowedDischargePower = ((rx_frame.data[5] << 8) + rx_frame.data[6]);
            batteryRelay = rx_frame.data[7];
          } else if (poll_data_pid == 2) {
            // set cell voltages data, start bite, data length from start, start cell
            set_cell_voltages(rx_frame, 2, 6, 0);
          } else if (poll_data_pid == 3) {
            set_cell_voltages(rx_frame, 2, 6, 32);
          } else if (poll_data_pid == 4) {
            set_cell_voltages(rx_frame, 2, 6, 64);
          } else if (poll_data_pid == 0x0A) {
            set_cell_voltages(rx_frame, 2, 6, 96);
          } else if (poll_data_pid == 0x0B) {
            set_cell_voltages(rx_frame, 2, 6, 128);
          } else if (poll_data_pid == 0x0C) {
            set_cell_voltages(rx_frame, 2, 6, 160);
          }
          break;
        case 0x22:  //Second datarow in PID group
          if (poll_data_pid == 2) {
            set_cell_voltages(rx_frame, 1, 7, 6);
          } else if (poll_data_pid == 3) {
            set_cell_voltages(rx_frame, 1, 7, 38);
          } else if (poll_data_pid == 4) {
            set_cell_voltages(rx_frame, 1, 7, 70);
          } else if (poll_data_pid == 0x0A) {
            set_cell_voltages(rx_frame, 1, 7, 102);
          } else if (poll_data_pid == 0x0B) {
            set_cell_voltages(rx_frame, 1, 7, 134);
          } else if (poll_data_pid == 0x0C) {
            set_cell_voltages(rx_frame, 1, 7, 166);
          } else if (poll_data_pid == 6) {
            batteryManagementMode = rx_frame.data[5];
          }
          break;
        case 0x23:  //Third datarow in PID group
          if (poll_data_pid == 1) {
            temperature_water_inlet = rx_frame.data[6];
            CellVoltMax_mV = (rx_frame.data[7] * 20);  //(volts *50) *20 =mV
          } else if (poll_data_pid == 2) {
            set_cell_voltages(rx_frame, 1, 7, 13);
          } else if (poll_data_pid == 3) {
            set_cell_voltages(rx_frame, 1, 7, 45);
          } else if (poll_data_pid == 4) {
            set_cell_voltages(rx_frame, 1, 7, 77);
          } else if (poll_data_pid == 0x0A) {
            set_cell_voltages(rx_frame, 1, 7, 109);
          } else if (poll_data_pid == 0x0B) {
            set_cell_voltages(rx_frame, 1, 7, 141);
          } else if (poll_data_pid == 0x0C) {
            set_cell_voltages(rx_frame, 1, 7, 173);
          } else if (poll_data_pid == 5) {
            heatertemp = rx_frame.data[7];
          }
          break;
        case 0x24:  //Fourth datarow in PID group
          if (poll_data_pid == 1) {
            CellVmaxNo = rx_frame.data[1];
            CellVminNo = rx_frame.data[3];
            CellVoltMin_mV = (rx_frame.data[2] * 20);  //(volts *50) *20 =mV
          } else if (poll_data_pid == 2) {
            set_cell_voltages(rx_frame, 1, 7, 20);
          } else if (poll_data_pid == 3) {
            set_cell_voltages(rx_frame, 1, 7, 52);
          } else if (poll_data_pid == 4) {
            set_cell_voltages(rx_frame, 1, 7, 84);
          } else if (poll_data_pid == 0x0A) {
            set_cell_voltages(rx_frame, 1, 7, 116);
          } else if (poll_data_pid == 0x0B) {
            set_cell_voltages(rx_frame, 1, 7, 148);
          } else if (poll_data_pid == 0x0C) {
            set_cell_voltages(rx_frame, 1, 7, 180);
          } else if (poll_data_pid == 5) {
            batterySOH = ((rx_frame.data[2] << 8) + rx_frame.data[3]);
          }
          break;
        case 0x25:  //Fifth datarow in PID group
          if (poll_data_pid == 2) {
            set_cell_voltages(rx_frame, 1, 5, 27);
          } else if (poll_data_pid == 3) {
            set_cell_voltages(rx_frame, 1, 5, 59);
          } else if (poll_data_pid == 4) {
            set_cell_voltages(rx_frame, 1, 5, 91);
          } else if (poll_data_pid == 0x0A) {
            set_cell_voltages(rx_frame, 1, 5, 123);
          } else if (poll_data_pid == 0x0B) {
            set_cell_voltages(rx_frame, 1, 5, 155);
          } else if (poll_data_pid == 0x0C) {
            set_cell_voltages(rx_frame, 1, 5, 187);
          } else if (poll_data_pid == 5) {
            system_number_of_cells = 98;
          }
          break;
        case 0x26:  //Sixth datarow in PID group
          break;
        case 0x27:  //Seventh datarow in PID group
          if (poll_data_pid == 1) {
            BMS_ign = rx_frame.data[6];
            inverterVoltageFrameHigh = rx_frame.data[7];
          }
          break;
        case 0x28:  //Eighth datarow in PID group
          if (poll_data_pid == 1) {
            inverterVoltage = (inverterVoltageFrameHigh << 8) + rx_frame.data[1];
          }
          break;
      }
      break;
    default:
      break;
  }
}


static uint8_t KIA_7E4_COUNTER = 0x01;
void send_can_battery() {
  unsigned long currentMillis = millis();
  //Send 100ms message
  if (currentMillis - previousMillis100 >= 500) {
    previousMillis100 = currentMillis;
  // Serial.println("send_can_battery");

    // CAN_FD_WriteFrame(&KIA64_553);
    // CAN_FD_WriteFrame(&KIA64_57F);
    // CAN_FD_WriteFrame(&KIA64_2A1);

      EGMP_7E4.data.u8[3] = KIA_7E4_COUNTER;
      CAN_FD_WriteFrame(&EGMP_7E4);

      KIA_7E4_COUNTER++;
      if (KIA_7E4_COUNTER > 0x0D) { // gets up to 0x010C before repeating
          KIA_7E4_COUNTER = 0x01;
      }
  }
  // Send 10ms CAN Message
  // if (currentMillis - previousMillis10ms >= interval10ms) {
  //   previousMillis10ms = currentMillis;
    

    // switch (counter_200) {
    //   case 0:
    //     KIA_HYUNDAI_200.data.u8[5] = 0x17;
    //     ++counter_200;
    //     break;
    //   case 1:
    //     KIA_HYUNDAI_200.data.u8[5] = 0x57;
    //     ++counter_200;
    //     break;
    //   case 2:
    //     KIA_HYUNDAI_200.data.u8[5] = 0x97;
    //     ++counter_200;
    //     break;
    //   case 3:
    //     KIA_HYUNDAI_200.data.u8[5] = 0xD7;
    //     if (startedUp) {
    //       ++counter_200;
    //     } else {
    //       counter_200 = 0;
    //     }
    //     break;
    //   case 4:
    //     KIA_HYUNDAI_200.data.u8[3] = 0x10;
    //     KIA_HYUNDAI_200.data.u8[5] = 0xFF;
    //     ++counter_200;
    //     break;
    //   case 5:
    //     KIA_HYUNDAI_200.data.u8[5] = 0x3B;
    //     ++counter_200;
    //     break;
    //   case 6:
    //     KIA_HYUNDAI_200.data.u8[5] = 0x7B;
    //     ++counter_200;
    //     break;
    //   case 7:
    //     KIA_HYUNDAI_200.data.u8[5] = 0xBB;
    //     ++counter_200;
    //     break;
    //   case 8:
    //     KIA_HYUNDAI_200.data.u8[5] = 0xFB;
    //     counter_200 = 5;
    //     break;
    // }

    // CAN_FD_WriteFrame(&KIA_HYUNDAI_200);

    // CAN_FD_WriteFrame(&KIA_HYUNDAI_523);

    // CAN_FD_WriteFrame(&KIA_HYUNDAI_524);
  // }
}

void setup_battery(void) {  // Performs one time setup at startup
  Serial.println("Kia / Hyundai E-GMP battery selected");

  system_max_design_voltage_dV = 7915;  // 791.0V, over this, charging is not possible (goes into forced discharge)
  system_min_design_voltage_dV = 6074;  // 607.0V under this, discharging further is disabled
}

#endif
