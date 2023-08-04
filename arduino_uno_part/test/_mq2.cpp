// https://www.nn-digital.com/en/blog/2019/11/20/interfacing-mq-mq-2-smoke-gas-sensor-module-using-arduino-to-measure-combustible-gas-concentration/
// https://ai.thestempedia.com/docs/evive-iot-kit/interfacing-mq-2-gas-sensor-with-evive/
// https://circuitdigest.com/microcontroller-projects/interfacing-mq2-gas-sensor-with-arduino
// https://circuitdigest.com/microcontroller-projects/gas-detection-and-ppm-measurement-using-pic-microcontroller-and-mq-gas-sensor
// https://lastminuteengineers.com/mq2-gas-senser-arduino-tutorial/

#include <Arduino.h>

/********************** Hardware Related Macros ************************************/
const uint8_t MQ2_PIN = A1;   // define which analog input channel you are going to use
const uint8_t RL_VALUE = 1;     // define the load resistance on the board, in kilo ohms
const float R0_CLEAN_AIR_FACTOR = 9.83;  // R0_CLEAR_AIR_FACTOR = (Sensor resistance in clean air) / R0, which is derived from the chart in datasheet

/********************** Software Related Macros ************************************/
const uint8_t CALIBARAION_SAMPLE_TIMES = 50;    // define how many samples you are going to take in the calibration phase
const uint16_t CALIBRATION_SAMPLE_INTERVAL = 500;   //define the time interal(in milisecond) between each samples in the cablibration phase
const uint8_t READ_SAMPLE_INTERVAL = 50;    // define how many samples you are going to take in normal operation
const uint8_t READ_SAMPLE_TIMES = 5;    // define the time interal(in milisecond) between each samples in normal operation

/********************** Application Related Macros *********************************/
const uint8_t GAS_LPG = 0;
const uint8_t GAS_CO = 1;
const uint8_t GAS_SMOKE = 2;

/********************** Globals ****************************************************/
// two points are taken from the curve
// with these two points, a line is formed which is "approximately equivalent" to the original curve
// data format:{x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
const float LPG_CURVE[3] = {2.3, 0.21, -0.47};
// data format:{x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
const float CO_CURVE[3] = {2.3, 0.72, -0.34};
// data format:{x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
const float SMOKE_CURVE[3] = {2.3, 0.53, -0.44};

float R0;

/********************** Function Prototypes*****************************************/
float resistance_mq2(void);
float calibrate_mq2(void);
float read_mq2(uint8_t);
uint16_t gas_ppm_percentage_mq2(uint8_t);

/* ******************** resistance_mq2 *********************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
********************************************************************************** */
float resistance_mq2() {
    uint16_t raw_adc = analogRead(MQ2_PIN);
    return (float) (RL_VALUE * (1023 - raw_adc) / raw_adc);
    // return (float) (RL_VALUE * (1023 / raw_adc - 1));
}

/* ******************** calibrate_mq2 **********************************************
Input:   MQ2_PIN - analog channel
Output:  R0 of the sensor, in kohm
Remarks: This function assumes that the sensor is in clean air. It use  
         resistance_mq2 to calculates the sensor resistance in clean air 
         and then divides it with R0_CLEAN_AIR_FACTOR. R0_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
********************************************************************************** */

float calibrate_mq2() {
    float temp = 0;
    float r0_kohm;

    for (uint16_t i = 0; i < CALIBARAION_SAMPLE_TIMES; ++i) {   // take multiple samples
        temp += resistance_mq2();
        delay(CALIBRATION_SAMPLE_INTERVAL);
    }
    temp = temp / CALIBARAION_SAMPLE_TIMES;   // calculate the average value

    r0_kohm = temp / R0_CLEAN_AIR_FACTOR;    // divided by R0_CLEAN_AIR_FACTOR yields the R0 according to the chart in the datasheet 

    return r0_kohm; 
}

/* ******************** read_mq2 ***************************************************
Input:   MQ2_PIN - analog channel
Output:  RS of the sensor
Remarks: This function use resistance_mq2 to caculate the sensor resistenc (RS).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float read_mq2() {
    float rs = 0;

    for (uint16_t i = 0; i < READ_SAMPLE_TIMES; ++i) {
        rs += resistance_mq2();
        delay(READ_SAMPLE_INTERVAL);
    }
    rs = rs / READ_SAMPLE_TIMES;

    return rs;  
}

/* ******************** gas_ppm_percentage_mq2 *************************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/
uint16_t gas_ppm_percentage_mq2(uint8_t gas_id) {
    float rs_ro_ratio = read_mq2() / R0;
    switch (gas_id) {
        case GAS_LPG:
            return (uint16_t) pow(10, (log10(rs_ro_ratio) - LPG_CURVE[1])/LPG_CURVE[2] + LPG_CURVE[0]);
        case GAS_CO:
            return (uint16_t) pow(10, (log10(rs_ro_ratio) - CO_CURVE[1])/CO_CURVE[2] + CO_CURVE[0]);
        case GAS_SMOKE:
            return (uint16_t) pow(10, (log10(rs_ro_ratio) - SMOKE_CURVE[1])/SMOKE_CURVE[2] + SMOKE_CURVE[0]);
        default:
            return 0;
    }
}

void setup() {
    Serial.begin(9600);   // UART setup, baudrate = 9600bps

    Serial.print("Calibrating...\n");                
    R0 = calibrate_mq2(); // make sure the sensor is in clean air when you perform the calibration                    
    Serial.print("Calibration is done.\n"); 
    Serial.print("R0=");
    Serial.print(R0, 8);  // display with 8 decimal point
    Serial.print("kohm\n");
}

void loop() {
    Serial.print("LPG:");
    Serial.print(gas_ppm_percentage_mq2(GAS_LPG));
    Serial.print("ppm    ");

    Serial.print("CO:");
    Serial.print(gas_ppm_percentage_mq2(GAS_CO));
    Serial.print("ppm    ");

    Serial.print("SMOKE:");
    Serial.print(gas_ppm_percentage_mq2(GAS_SMOKE));
    Serial.print("ppm\n");
    
    delay(200);
}

// R0 in kohm
// 1.42624626
// 1.54916334
// 1.47914552
// 1.53407931
// 1.55239067