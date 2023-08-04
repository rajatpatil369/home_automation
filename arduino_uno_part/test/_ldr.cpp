/*
 *  LDR Lux Meter
 *
 *  This sketch calculates the lux from a voltage reading
 *
 *  The hardware connected to Analog Pin 0 should be a voltage divider circuit between an LDR and a
 *  resistor (nominally 5kohm).  The resistor should be connected to ground and the LDR should be
 *  connected to 5V.  The point in between should be connected to Analog Pin 0.
 *  1) Calculate voltage based on reading
 *  2) Calculate resistance of LDR based on voltage
 *  3) Calculate the lux that must be falling on LDR based on the resistance
 *
 *  The lux reading is then output to an Adafruit LCD shield.
 *  Created Dec. 4, 2015
 *  By David Williams
 */

#include <Arduino.h>

// These constants, define values needed for the LDR readings and ADC
#define LDR_PIN A0
#define MAX_ADC_READING 1023
#define ADC_REF_VOLTAGE 5
#define REF_RESISTANCE 5600 // measure this for best results
#define LUX_CALC_SCALAR 12518931
#define LUX_CALC_EXPONENT -1.405

void setup(void) {
    Serial.begin(9600);
    Serial.println(F("Light Sensor Test\n"));
}

void loop(void) {

    uint16_t ldrRawData;
    float resistorVoltage, ldrVoltage;
    float ldrResistance;
    float ldrLux;

    // Perform the analog to digital conversion
    ldrRawData = analogRead(LDR_PIN);

    // RESISTOR VOLTAGE_CONVERSION
    // Convert the raw digital data back to the voltage that was measured on the analog pin
    resistorVoltage = (float) (ldrRawData / MAX_ADC_READING * ADC_REF_VOLTAGE);

    // voltage across the LDR is the 5V supply minus the 5k resistor voltage
    ldrVoltage = ADC_REF_VOLTAGE - resistorVoltage;

    // LDR_RESISTANCE_CONVERSION
    // resistance that the LDR would have for that voltage
    ldrResistance = ldrVoltage / resistorVoltage * REF_RESISTANCE;
    Serial.print("ldrResistance="); Serial.println(ldrResistance);

    float res_vlt_ldr = ldrRawData * (5.0/1023.0);
    float ldr_res = 5600.0 * (5.0/res_vlt_ldr - 1);
    Serial.print("my_resistance="); Serial.println(ldr_res);
    

    // LDR_LUX
    // Change the code below to the proper conversion from ldrResistance to ldrLux
    ldrLux = LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);

    ldrLux = 

    // print out the results
    Serial.print("LDR Raw Data   : ");
    Serial.println(ldrRawData);
    Serial.print("LDR Voltage    : ");
    Serial.print(ldrVoltage);
    Serial.println(" volts");
    Serial.print("LDR Resistance : ");
    Serial.print(ldrResistance);
    Serial.println(" Ohms");
    Serial.print("LDR Illuminance: ");
    Serial.print(ldrLux);
    Serial.println(" lux");
    
    Serial.print("my_lux="); Serial.println(LUX_CALC_SCALAR * pow(ldr_res, LUX_CALC_EXPONENT));

    delay(250);
}
