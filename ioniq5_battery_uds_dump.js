// Script to dump the battery health/state using the SavvyCAN scripting interface
// based on https://github.com/maciek16c/hyundai-santa-fe-phev-battery/blob/main/uds.js
// and data from https://docs.google.com/spreadsheets/d/1eT2R8hmsD1hC__9LtnkZ3eDjLcdib9JR-3Myc97jy8M/edit#gid=246048743
// and https://github.com/Esprit1st/Hyundai-Ioniq-5-Torque-Pro-PIDs


var enable_uds =1;
var received =1;
var counter = 0;
var id = 0x101;
const cellVoltages = Array(192).fill(0)

function int16(v) {
    return (v << 16) >>16;
}

const c2f = (c) => (c * 9/5) + 32

function setup ()
{
    host.log("Hyundai/Kia e-GMP Battery data dump");
    host.setTickInterval(10);
    uds.setFilter(0x7EC, 0x7FF, 0)
    //can.setFilter(0x7EC, 0x7FF, 0); // uncomment to activate gotCANFrame
    host.addParameter("enable_uds")
}

function gotCANFrame (bus, id, len, data) {
	//host.log("Got CAN frame 0x" + id.toString(16) + " with len " + len + " :"
	//	+ " " + data[0].toString(16) + " " + data[1].toString(16)
	//	+ " " + data[2].toString(16) + " " + data[3].toString(16)
	//	+ " " + data[4].toString(16) + " " + data[5].toString(16)
	//	+ " " + data[6].toString(16) + " " + data[7].toString(16));
	host.log("Got CAN frame 0x" + id.toString(16) + " with len " + len + " :"
		+ " " + data[0].toString(16) + " " + data[1].toString(10)
		+ " " + data[2].toString(10) + " " + data[3].toString(10)
		+ " " + data[4].toString(10) + " " + data[5].toString(10)
		+ " " + data[6].toString(10) + " " + data[7].toString(10));
}

const dump_101 = (data) => {
    var soc = data[7]/2; // why not +3 ??
    var max_regen = (data[8]*256+data[9])/100;
    var max_power = (data[10]*256+data[11])/100;
    var contactor_11 = data[12];    // ??
    var battery_current = int16((data[13]*256+data[14]))/10;
    var battery_voltage = (data[15]*256+data[16])/10;
    var battery_max_temp = data[17];
    var battery_min_temp = data[18];
    var battery_temp1 = data[19];
    var battery_temp2 = data[20];
    var battery_temp3 = data[21];
    var battery_temp4 = data[22];
    var battery_inlet_temp = data[25];  // ?? shows 260.6F (127c) with 41F ambient
    var max_cell_voltage = data[26]/50;
    var max_cell_voltage_nr = data[27];
    var min_cell_voltage = data[28]/50;
    var min_cell_voltage_nr = data[29];
    var battery_fan_mod = data[30]
    var battery_fan_speed = data[31]
    var lv_battery_voltage = data[32]/10;
    var cumulative_charge_current = (data[33]*(2^24)+data[34]*(2^16)+data[35]*(2^8)+data[36])/10;
    var cumulative_discharge_current = (data[37]*(2^24)+data[38]*(2^16)+data[39]*(2^8)+data[40])/10;
    var cumulative_energy_charged = (data[41]*(2^24)+data[42]*(2^16)+data[43]*(2^8)+data[44])/10;
    var cumulative_energy_discharged = (data[45]*(2^24)+data[46]*(2^16)+data[47]*(2^8)+data[48])/10;
    var op_time = data[49]*(2^24)+data[50]*(2^16)+data[51]*(2^8)+data[52];
    var ignition = data[53];
    var inv_cap_voltage = data[54]*(2^8)+data[55] // ??
    var rpm_1 = data[56]*(2^8)+data[57];
    var rpm_2 = data[58]*(2^8)+data[59]
    var isolation_resistance = data[60]*(2^8)+data[61]

    host.log("Soc: " + soc.toString(10) + "% Lv: " + lv_battery_voltage.toString(10) + "V Hv: " + battery_voltage.toString(10) + "V"
        + " Current: " + battery_current.toString(10) + "A Max regen: " +  max_regen.toString(10) + "Kw"
        +" Max pwr: " +  max_power.toString(10) + "Kw Ign: "+ ignition.toString(2) 
        + "  inv_v: " + inv_cap_voltage.toString(10) + " iso_r: " + isolation_resistance.toString(10) + "kOhm");  
    
    host.log("  min/max cell voltage: " + min_cell_voltage.toString(10) + "/" + max_cell_voltage.toString(10) + "V ("
        + min_cell_voltage_nr + "/" + max_cell_voltage_nr + ")"
        + " cum energy charge/discharge " + cumulative_energy_charged.toString(10) + "/" + cumulative_energy_discharged.toString(10) + "Ah"
        + " cum current charge/discharge " + cumulative_charge_current.toString(10) + "/" + cumulative_discharge_current.toString(10) + "Ah");
    host.log("  op time: " + op_time.toString(10) 
        + " min_temp: " + c2f(battery_min_temp) + "F max_temp: " + c2f(battery_max_temp) + "F"
        + " inlet_temp: " + c2f(battery_inlet_temp) + "F"
        + " temp1: " + c2f(battery_temp1) + "F temp2: " + c2f(battery_temp2) + "F" 
        + " temp3: " + c2f(battery_temp3) + "F temp4: " + c2f(battery_temp4) + "F")
    host.log("  Relay: " + contactor_11.toString(2) 
        + " RPM 1: " + rpm_1.toString(10) + " RPM 2: " + rpm_2.toString(10)
        + " batt fan mod: " + battery_fan_mod + " batt fan speed: " + battery_fan_speed)

    // seems to run out of time in one cycle to dump these too so things get interleaved
    //logCellVoltages(1, 192)
}

const save_cell_voltages = (data, dataOffset, cellOffset, cnt) => {
    for (let i=0; i < cnt; i++) {
        const c = data[dataOffset + i]/50
        cellVoltages[cellOffset + i] = c
    }
}

const logCellVoltages = (start, end) => {
    //host.log(cellVoltages)
    host.log('cell voltages (v):\n')
    let out = ''
    for (let i=start-1; i < end; i++) { // 180 on 72.6kWh
        const o = i + 1
        out += o.toString().padStart(3, '0') + ": " + cellVoltages[i].toString().padEnd(4, '0') + "    "
        if ( o%8 == 0 ) {
            host.log(out)
            out = ''
        }
    }
   // host.log("cell voltages (v):\n" + out)
}

/*
    Battery options:
    58 kWh: 24 modules (288 cells) 144 cell voltages
    72.6 kWh: 30 modules (360 cells) it's 360/2 = 180 since each module has 2 parallel
    77.4 kWh: 32 modules (384 cells) 192 cell voltages
*/

// contains cell 1-32 voltages starting at data[7]
const dump_102 = (data) => {
    save_cell_voltages(data, 7, 0, 32)
    logCellVoltages(1, 32)
}

// contains cell 33-64 voltages starting at data[7]
const dump_103 = (data) => {
    save_cell_voltages(data, 7, 32, 32)
    logCellVoltages(33, 64)
}
// contains cell 65-96 voltages starting at data[7]
const dump_104 = (data) => {
    //const out = dump_cell_voltages(data, 7, 64, 32)
    //host.log("cell voltages (v):\n" + out)
    save_cell_voltages(data, 7, 64, 32)
    logCellVoltages(65, 96)
}
// contains cell 97-128 voltages starting at data[7]
const dump_10A = (data) => {
    //const out = dump_cell_voltages(data, 7, 96, 32)
    //host.log("cell voltages (v):\n" + out)
    save_cell_voltages(data, 7, 96, 32)
    logCellVoltages(97, 128)
}
// contains cell 129-160 voltages starting at data[7]
const dump_10B = (data) => {
    //const out = dump_cell_voltages(data, 7, 128, 32)
    //host.log("cell voltages (v):\n" + out)
    save_cell_voltages(data, 7, 128, 32)
    logCellVoltages(129, 160)
}
// contains cell 161-192 voltages starting at data[7]
const dump_10C = (data) => {
    //const out = dump_cell_voltages(data, 7, 160, 32)
    //host.log("cell voltages (v):\n" + out)
    save_cell_voltages(data, 7, 160, 32)
    logCellVoltages(161,192)
    // does not seem to be able to log that much
    //logCellVoltages(1, 192)
}


const dump_105 = (data) => {
    const temp_x1 = data[15] // ??
    const vdiff = data[23]
    const airbag = data[25]
    const heater_temp1 = data[26]
    //27
    const soh = (data[28]*256 + data[29])/10 // %
    const max_det_cell_no = data[30]
    const min_deterioration = (data[31]*256 + data[32])/10
    const min_det_cell_no = data[33]
    const soc_display = data[34]/2 // %
    const cell97v = data[37]/50 // ??
    const cell98v = data[38]/50 // ??
    const temp_x2 = data[41] // ??
    host.log("vdiff: " + vdiff + "v" + " airbag: " + airbag + " heater temp1: " + heater_temp1)
    host.log("soh: " + soh + "%" + " min det: " + min_deterioration + " min/max det cell: " + min_det_cell_no + "/" + max_det_cell_no)
    host.log("soc disp: " + soc_display + "%" + " cell97v: " + cell97v + "v cell98v: " + cell98v + "v")
    host.log("temp x1: " + c2f(temp_x1) + "F temp x2: " + c2f(temp_x2) + "F")
}
const dump_106 = (data) => {
    const coolant_temp = data[7]
    const temp_x3 = data[9] // ??
    const ac1 = data[11] //30 when AC on, 0a when AC off
    const ac2 = data[15] // 6b/51 when AC ON  00 when AC OFF
    const ac3 = data[16] // "7c when AC ON 00 when AC OFF"
    // BAT.MANAGEMENT MODE
    // bit 0-3
    // LTR 3 11
    // COOL 4 100
    // OFF 6 110 0
    // PTC E 1110
    const batt_manage_mode = data[17]
    const temp_x4 = data[23]
    
    host.log("coolant temp: " + c2f(coolant_temp) + "F" + " temp x3: " + c2f(temp_x3) + "F"  + " temp 4: " + c2f(temp_x4) + "F"
        + " A/C1: 0x " + ac1.toString(16) + " A/C2: 0x " + ac1.toString(16) + " A/C13 0x " + ac1.toString(16)
        + " batt manage mode: 0x" + batt_manage_mode.toString(16))
}

// decoding using https://docs.google.com/spreadsheets/d/1eT2R8hmsD1hC__9LtnkZ3eDjLcdib9JR-3Myc97jy8M/edit#gid=246048743
// and https://github.com/Esprit1st/Hyundai-Ioniq-5-Torque-Pro-PIDs
function gotUDSMessage(bus, id, service, subFunc, len, data)
{
    received =1;
    var data_id = data[1]*256+data[2];
    //host.log("bus=" + bus + " id=" + id + " service=" + service + " subFunc=" + subFunc + " len= " + len + " data_id=0x" + data_id.toString(16))
    if (data_id==0x101) {
        dump_101(data)
    } else if (data_id==0x102) {
        dump_102(data)
    } else if (data_id==0x103) {
        dump_103(data)
    } else if (data_id==0x104) {
        dump_104(data)
    } else if (data_id==0x105) {
        dump_105(data)
    } else if (data_id==0x106) {
        dump_106(data)
    } else if (data_id==0x10A) {
        dump_10A(data)
    } else if (data_id==0x10B) {
        dump_10B(data)
    } else if (data_id==0x10C) {
        dump_10C(data)
    } else {
        // 0x107, 108, 2231
        //host.log("got id " + data_id.toString(16) + " len=" + len + " ??????????");
    }
}

function tick()
{
    if (enable_uds == 1  &&  received ==1) {
        uds.sendUDS(0, 0x7e4, 0x22, 2, id, 0, 0)
        received = 0
        id +=1

        if (id >0x10C) { // gets up to 0x010C before repeating
            id=0x101
        }
    }

    // at 10ms tick intervaal, this triggers the sendUDS every second
    counter +=1;
    if (counter >100) {
        received =1
        counter=0
    }
}