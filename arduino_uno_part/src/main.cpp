#include <Arduino.h>
#include "dht.h"
#include "NewPing.h"
#include "FastLED.h"
#include <SoftwareSerial.h>

const float ADC_REFERENCE_VOLTAGE = 5.0;                             // reference voltage for ADC
const float ADC_MAX_READING = 1023.0;                                // (2^n - 1)    @n=10-bit
const float ADC_STEP_SIZE = ADC_REFERENCE_VOLTAGE / ADC_MAX_READING; // (Vref_upper - Vref_lower) / (2^n - 1)

const uint8_t PIR_PIN = 4;

const uint8_t LDR_PIN = A0;                     // LDR isn't really a good candidate for measuring light intensity, photodiode would do better
const float LDR_VDR = 5600.0;                   // Voltage Divider Resistance (in ohm)
const float LUX_SCALAR = 8.25 * pow(10, 7.098); // these two constants are subjected to calibration according to the targeted location
const float LUX_EXPONENT = -1.4059;             // LUX_EXPONENT and LUX_THRESHOLD must be acquired by comparing to a standard LUX meter readings and using curve fitting methods

const uint8_t DHT22_PIN = 5;

const uint8_t USS_TRIGGER_PIN = 6;
const uint8_t USS_ECHO_PIN = 7;
const unsigned long USS_MAX_DISTANCE = 140;

const uint8_t MQ2_PIN = A1;
const float MQ2_RS_R0 = 9.83;
const float MQ2_VDR = 1.0; // in kohm
const float MQ2_R0 = 1.50820502;
const float LPG_CURVE_X = 2.3;
const float LPG_CURVE_Y = 0.21;
const float LPG_CURVE_M = -0.47;

const uint8_t IR_PIN = 8;

const uint8_t BUZZER_PIN = 9;

const uint8_t NEOPIX_PIN = 10;
const uint8_t NUMBER_OF_NEOPIX = 1;

const uint8_t DOOR_LOCK_SOLENOID_PIN = 11;
const uint8_t RELAY_SSR_PIN = 12;

const uint8_t STEPPER_IN1 = A2;
const uint8_t STEPPER_IN2 = A3;
const uint8_t STEPPER_IN3 = A4;
const uint8_t STEPPER_IN4 = A5;
// pick even indexed values (0 2 4 6) for wave drive stepping, and odd for full
// iterate from 0 to 7 for anti-clockwise rotation, and 7 to 0 for CW
// it takes 12 cycles of wave/full/half to complete one rotation
// const uint8_t HALF_DRIVE_SEQ[8] = {0b1000, 0b1100, 0b0100, 0b0110, 0b0010, 0b0011, 0b0001, 0b1001};
const uint8_t FULL_DRIVE_SEQ[4] = {0b00110000, 0b00011000, 0b00001100, 0b00100100};

const uint8_t RX_PIN = 2;
const uint8_t TX_PIN = 3;

const bool DEBUG = true;
const bool COM = false;

const float ERR_PER = 0.03;
const float ALT_ERR_PER = 0.09;

const uint16_t DELAY_INTERVAL = 200;

SoftwareSerial com_ch_uno(RX_PIN, TX_PIN); // buffer size is 64bytes
template <typename... T>
void print(bool debug, const char *str, T... args)
{
    int len = snprintf(NULL, 0, str, args...);
    if (len)
    {
        char buff[len + 1];
        snprintf(buff, len + 1, str, args...);
        Serial.println(buff);
        if (!debug)
        {
            com_ch_uno.println(buff);
        }
    }
}
void println(bool debug, const __FlashStringHelper *str)
{
    Serial.println(str);
    if (!debug)
    {
        com_ch_uno.println(str);
    }
}

struct prefrences
{
    bool is_home = true;
    uint16_t ldr_th_min = 52; // needs calibration
    uint16_t ldr_th_max = 106;
    float dht_t_th_min = 26.2;
    float dht_t_th_max = 33.7;
    float dht_h_th = 34.6;
    unsigned long uss_th_min = 20; // water_level
    unsigned long uss_th_max = 125;
    uint16_t mq2_th = 160; // needs calibration
} user_settings;

// TODO: implement class for the same functionality with an ability to access elements by indexing
struct raw_inputs
{
    int pir = 0;
    int ldr = 0;
    float temp = 0.0;
    float humi = 0.0;
    unsigned long uss = 0;
    int mq2 = 0;
    int ir = 0;

    bool operator==(const raw_inputs &other) const
    {
        return pir == other.pir &&
               ldr == other.ldr &&
               temp == other.temp &&
               humi == other.humi &&
               uss == other.uss &&
               mq2 == other.mq2 &&
               ir == other.ir;
    }
    bool operator!=(const raw_inputs &other) const
    {
        return pir != other.pir ||
               ldr != other.ldr ||
               temp != other.temp ||
               humi != other.humi ||
               uss != other.uss ||
               mq2 != other.mq2 ||
               ir != other.ir;
    }
    bool operator%(const raw_inputs &other) const
    { // usage: curr_para % prev_para
        const float ERR_PER = 0.02;
        const int adc_error = (int)(ERR_PER * 1023);                // for range 0 to 1023
        const float temp_error = (float)(ERR_PER * 120);            // for range -40 to 80
        const float humi_error = (float)(ERR_PER * 100);            // for range 0 to 100
        const signed long uss_error = (signed long)(ERR_PER * 398); // for range 2 to 400
        return (pir != other.pir) ||
               (abs(ldr - other.ldr) > adc_error) ||
               (abs(temp - other.temp) > temp_error) ||
               (abs(humi - other.humi) > humi_error) ||
               (abs((signed long)(uss - other.uss)) > uss_error) ||
               (abs(mq2 - other.mq2) > adc_error) ||
               (ir != other.ir);
    }
} curr_para, prev_para;
struct inputs
{
    bool pir_motion;
    uint16_t ldr_lux;
    float dht_temp;
    float dht_humi;
    unsigned long uss_cm;
    uint16_t mq2_lpg_ppm;
    bool ir_position;
} monitored_parameters;

struct outputs
{ // 9 elements in total, now would be a good time to go with OOP
    bool light = false;
    uint8_t blind = 0; // 0=closed, 1=open, 2=halfPosition (4 rotations to completely close in CW, 8 for garage shutter)
    bool ac = false;
    bool fan = false;
    bool water_pump = false;
    bool garage_door = false;
    bool lpg_leak = false;
    bool buzzer = false;
    unsigned long neopix_color = 0xfb0805; // possibility: user can upload a lighting pattern for the neopix led strip
} output_dev_status, manual_i, manual_mode;

dht DHT;
NewPing uss(USS_TRIGGER_PIN, USS_ECHO_PIN, USS_MAX_DISTANCE);
CRGB neo_pixels[NUMBER_OF_NEOPIX];

void blink()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
}

// WARNING: BLOCKING FUNCTION
// TODO: Solution
// 1. vTaskDelay -> Arduino ported RTOS
// 2. timer interrupts
void window_blind(bool open, float r = 1.0, uint16_t d = 3)
{
    /*
        int d = 2;
        for (int i = 0; i < 509; i++){
            // PORTC = (0b11000011 & PORTC) | (0b00100000); delay(d);
            // PORTC = (0b11000011 & PORTC) | (0b00010000); delay(d);
            // PORTC = (0b11000011 & PORTC) | (0b00001000); delay(d);
            // PORTC = (0b11000011 & PORTC) | (0b00000100); delay(d);

            PORTC = (0b11000011 & PORTC) | (0b00110000); delay(d);
            PORTC = (0b11000011 & PORTC) | (0b00011000); delay(d);
            PORTC = (0b11000011 & PORTC) | (0b00001100); delay(d);
            PORTC = (0b11000011 & PORTC) | (0b00100100); delay(d);
        }
    */
    // if (d < 5) return;  // doesn't work below
    int8_t start, stop, step;
    if (open)
    { // implies direction=ACW
        start = 0;
        stop = 3;
        step = 1;
    }
    else
    { // clockwise otherwise
        start = 3;
        stop = 0;
        step = -1;
    }
    uint32_t x = 0, y = (uint32_t)(509.4716 * r); // according to the datasheet: 32 steps per revolution * (32/9 * 22/11 * 26/9 * 31/10) gear ratio / 4 steps = 509.4716049382716 (12 for other)
    for (; x < y; ++x)
    {
        int i;
        for (i = start; i != stop; i += step)
        {
            PORTC = (0b11000011 & PORTC) | FULL_DRIVE_SEQ[i];
            delay(d);
        }
        PORTC = (0b11000011 & PORTC) | FULL_DRIVE_SEQ[i];
        delay(d);
    }
}

uint8_t _i = 0; // only updated in `read_sensors` function
void read_sensors()
{
    curr_para.pir = digitalRead(PIR_PIN);
    curr_para.ldr = analogRead(LDR_PIN);
    if (_i * DELAY_INTERVAL >= 2000)
    {
        uint8_t chk = DHT.read22(DHT22_PIN);
        if (chk != DHTLIB_OK)
        {
            print(DEBUG, "ERROR: Error reading DHT22. Status code: %d", chk);
        }
        _i = 0;
    }
    curr_para.temp = DHT.temperature;
    curr_para.humi = DHT.humidity;
    unsigned long cm = uss.ping_cm();
    if (cm >= 3lu && cm <= USS_MAX_DISTANCE)
    {
        curr_para.uss = cm;
    }
    curr_para.mq2 = analogRead(MQ2_PIN);
    curr_para.ir = digitalRead(IR_PIN);

    if (prev_para != curr_para)
    {
        monitored_parameters.pir_motion = curr_para.pir;

        uint16_t ldr_raw = curr_para.ldr;
        // float res_vlt = map(ldr_val, 0, 1023, 0, ADC_REFERENCE_VOLTAGE);
        float res_vlt_ldr = ldr_raw * ADC_STEP_SIZE;
        float ldr_res = LDR_VDR * (ADC_REFERENCE_VOLTAGE / res_vlt_ldr - 1);
        monitored_parameters.ldr_lux = (uint16_t)(LUX_SCALAR * pow(ldr_res, LUX_EXPONENT));

        monitored_parameters.dht_temp = curr_para.temp;
        monitored_parameters.dht_humi = curr_para.humi;

        monitored_parameters.uss_cm = curr_para.uss;

        uint16_t mq2_raw = curr_para.mq2;
        float mq2_res = (float)(MQ2_VDR * (1023 - mq2_raw) / mq2_raw);
        float rs_ro_ratio = mq2_res / MQ2_R0;
        monitored_parameters.mq2_lpg_ppm = (uint16_t)pow(10, (log10(rs_ro_ratio) - LPG_CURVE_Y) / LPG_CURVE_M + LPG_CURVE_X);

        monitored_parameters.ir_position = curr_para.ir;

        // print(DEBUG, "curr_para = %d %d %.2f %.2f %lu %d %d", curr_para.pir, curr_para.ldr, curr_para.temp, curr_para.humi, curr_para.uss, curr_para.mq2, curr_para.ir);
        // print(DEBUG, "prev_para = %d %d %.2f %.2f %lu %d %d", prev_para.pir, prev_para.ldr, prev_para.temp, prev_para.humi, prev_para.uss, prev_para.mq2, prev_para.ir);
    }

    ++_i;
}

void take_action()
{
    if (user_settings.is_home)
    {
        if (monitored_parameters.pir_motion)
        {
            println(DEBUG, F("INFO: Motion detected."));
            if (monitored_parameters.ldr_lux <= user_settings.ldr_th_min)
            {
                print(DEBUG, "INFO: Ambient light intensity less than set min `%d lx` threshold.", user_settings.ldr_th_min);
                if (!output_dev_status.light && !manual_mode.light)
                {
                    println(DEBUG, F("      Switching ON the light."));
                    digitalWrite(RELAY_SSR_PIN, HIGH);
                    output_dev_status.light = true;
                }
                if (output_dev_status.blind == 0 && !manual_mode.blind)
                {
                    println(DEBUG, F("      Opening window blind till half."));
                    window_blind(false, 0.5);
                    output_dev_status.blind = 2;
                }
            }
            else if (monitored_parameters.ldr_lux >= user_settings.ldr_th_max)
            {
                print(DEBUG, "INFO: Ambient light intensity more than set max `%d lx` threshold.", user_settings.ldr_th_max);
                if (output_dev_status.light && !manual_mode.light)
                {
                    println(DEBUG, F("      Switching OFF the light."));
                    digitalWrite(RELAY_SSR_PIN, LOW);
                    output_dev_status.light = false;
                }
                if (output_dev_status.blind == 1 && !manual_mode.blind)
                {
                    println(DEBUG, F("      Closing window blind till half."));
                    window_blind(true, 0.5);
                    output_dev_status.blind = 2;
                }
            }
            else
            {
                print(DEBUG, "INFO: Ambient light intensity in range min `%d lx` to max `%d lx` threshold.", user_settings.ldr_th_min, user_settings.ldr_th_max);
                if (output_dev_status.light && !manual_mode.light)
                {
                    println(DEBUG, F("      Switching OFF the light."));
                    digitalWrite(RELAY_SSR_PIN, LOW);
                    output_dev_status.light = false;
                }
            }
            // another potential application of LDR is that it could be used to detect whether it is daylight or not, a RTC module can complement this task

            if (monitored_parameters.dht_temp <= user_settings.dht_t_th_min)
            {
                print(DEBUG, "INFO: Ambient temperature less than set min `%.2f °C` threshold.", user_settings.dht_t_th_min);
                if (output_dev_status.fan && !manual_mode.fan)
                {
                    println(DEBUG, F("      Turning OFF the fan."));
                    blink();
                    output_dev_status.fan = false;
                }
            }
            else if (monitored_parameters.dht_temp >= user_settings.dht_t_th_max)
            {
                print(DEBUG, "INFO: Ambient temperature more than set max `%.2f °C` threshold.", user_settings.dht_t_th_max);
                if (!output_dev_status.fan && !manual_mode.fan)
                {
                    println(DEBUG, F("      Turning ON the fan."));
                    blink();
                    output_dev_status.fan = true;
                }
            }
            if (monitored_parameters.dht_humi <= user_settings.dht_h_th)
            {
                print(DEBUG, "INFO: Ambient humidity less than set threshold `%.2f %%`.", user_settings.dht_h_th);
                if (output_dev_status.ac && !manual_mode.ac)
                {
                    println(DEBUG, F("      Deactivating AC."));
                    blink();
                    output_dev_status.ac = false;
                }
            }
            else if (monitored_parameters.dht_humi > user_settings.dht_h_th)
            {
                print(DEBUG, "INFO: Ambient humidity more than set threshold `%.2f %%`.", user_settings.dht_h_th);
                if (!output_dev_status.ac && !manual_mode.ac)
                {
                    println(DEBUG, F("      Activating AC."));
                    blink();
                    output_dev_status.ac = true;
                }
            }
            // `blink();` merely indicates the exceution of an action, such an turning on or off a fan/cooler
            // the action could get as complicated as:
            // 1. setting the dimmness of a dimmable LED bulb or tubelight according the ambient lighting condition using a BT136+MOC3021 driver and a FWR+MCT2E zero cross detector, or more sophisticated trailing edge dimming
            // 2. setting fan/cooler speed according to ambient temperature using capacitive control step type regulating

            // the IR Sensor can be used to determine whether the window blind or garage shutter is fully opened/closed (rotate the stpper in CW/ACW until the IR sensor says "You have reached your destinition.")
            // the Ultrasonic sensor could be useds for the same purpose as well, i.e. keep track of blind/shutter's position
        }
        else
        {
            // more reliable mechanism needs to be in place for detecting that the person has left the room
            println(DEBUG, F("INFO: Shutting down the appliances."));
            if (output_dev_status.light && !manual_mode.light)
            {
                digitalWrite(RELAY_SSR_PIN, LOW);
                output_dev_status.light = false;
            }
            if (output_dev_status.blind != 0 && !manual_mode.blind)
            {
                window_blind(true, 1 / output_dev_status.blind);
                output_dev_status.blind = 0;
            }
            if (output_dev_status.fan && !manual_mode.fan)
            {
                blink();
                output_dev_status.fan = false;
            }
            if (output_dev_status.ac && !manual_mode.ac)
            {
                blink();
                output_dev_status.ac = false;
            }
            if (output_dev_status.neopix_color && !manual_mode.neopix_color)
            {
                neo_pixels[0] = 0x000000;
                FastLED.show();
            }
        }

        if (monitored_parameters.uss_cm >= (USS_MAX_DISTANCE - user_settings.uss_th_min))
        {
            print(DEBUG, "INFO: Tank water level less than set min `%d cm` threshold.", user_settings.uss_th_min);
            if (!output_dev_status.water_pump && !manual_mode.water_pump)
            {
                println(DEBUG, F("      Turning ON the water pump."));
                blink();
                output_dev_status.water_pump = true;
            }
            if (output_dev_status.garage_door && !manual_mode.garage_door)
            {
                println(DEBUG, F("      Outgoing car detected, closing the gates."));
                digitalWrite(DOOR_LOCK_SOLENOID_PIN, HIGH);
                window_blind(true, 2);
                digitalWrite(DOOR_LOCK_SOLENOID_PIN, LOW);
                output_dev_status.garage_door = false;
            }
        }
        else if (monitored_parameters.uss_cm <= (USS_MAX_DISTANCE - user_settings.uss_th_max))
        {
            print(DEBUG, "INFO: Tank water level more than set max `%d cm` threshold.", user_settings.uss_th_max);
            if (output_dev_status.water_pump && !manual_mode.water_pump)
            {
                println(DEBUG, F("      Tank about full, turning off the water pump."));
                output_dev_status.water_pump = false;
            }
            if (!output_dev_status.garage_door && !manual_mode.garage_door)
            {
                println(DEBUG, F("      Incoming car detected, opening the gates."));
                digitalWrite(DOOR_LOCK_SOLENOID_PIN, HIGH);
                window_blind(false, 2);
                digitalWrite(DOOR_LOCK_SOLENOID_PIN, LOW);
                output_dev_status.garage_door = true;
            }
            // the MCU can validate the person (using face recognition, name plate recognition, RFDI tags, LoRa, GPS, etc) and decide to open the garage door using a solenoid lock + stepper operated shutter or keep it closed
        }

        if (output_dev_status.buzzer && !output_dev_status.lpg_leak)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            output_dev_status.buzzer = false;
        }
    }
    else
    {
        if (monitored_parameters.pir_motion)
        {
            println(DEBUG, F("INFO: Motion detected. Intruder alert!!! Sounding the alarm."));
            digitalWrite(BUZZER_PIN, LOW);
            output_dev_status.buzzer = true;
        }
        if (monitored_parameters.uss_cm >= (USS_MAX_DISTANCE - user_settings.uss_th_min) || monitored_parameters.uss_cm <= (USS_MAX_DISTANCE - user_settings.uss_th_max))
        {
            println(DEBUG, F("WARNING: Unusual activity detected."));
            println(DEBUG, F("         Tank water has changed."));
            println(DEBUG, F("         Unexpected activity near the garage door."));
        }
        if (!monitored_parameters.ir_position)
        {
            println(DEBUG, F("INFO: Unusual activity detected: door/window/cabinet/cupboard was opened, sounding the alaram."));
            digitalWrite(BUZZER_PIN, LOW);
            output_dev_status.buzzer = true;
        }
        // notify the owner through GSM message/call, mail/telegram/whatsapp whatever available at disposal
    }
    // additionally, on site finger print authorization can be used for authentication (doors, locks, etc.)

    if (monitored_parameters.mq2_lpg_ppm >= user_settings.mq2_th)
    {
        println(DEBUG, F("INFO: LPG leak detected."));
        if (!output_dev_status.lpg_leak && !manual_mode.lpg_leak)
        {
            println(DEBUG, F("      Activating fire sprinkler system."));
            blink();
            output_dev_status.lpg_leak = true;
        }
        if (!output_dev_status.buzzer && !manual_mode.buzzer)
        {
            println(DEBUG, F("      Sounding the alarm."));
            digitalWrite(BUZZER_PIN, LOW);
            output_dev_status.buzzer = true;
        }
        // notify the authorites and user through GSM/LoRa + GPS (mail, telegram, whatsapp) about the fire incidence, so that a firefigher rescue team could be dispatches to the location
    }
    else
    {
        if (output_dev_status.buzzer && output_dev_status.lpg_leak && !manual_mode.lpg_leak)
        {
            println(DEBUG, F("INFO: LPG leak under control."));
            output_dev_status.lpg_leak = false;
            digitalWrite(BUZZER_PIN, HIGH);
            output_dev_status.buzzer = false;
        }
    }

    if (manual_mode.buzzer)
    {
        digitalWrite(BUZZER_PIN, manual_i.buzzer);
    }
}

void _parse_command()
{
    char c = com_ch_uno.read();
    if (c == '~')
    {
        char change[2];
        com_ch_uno.readBytes(change, 2);
        uint32_t r = com_ch_uno.read();
        uint16_t g = com_ch_uno.read();
        uint8_t b = com_ch_uno.read();

        char mask[2];
        com_ch_uno.readBytes(mask, 2);

        com_ch_uno.readStringUntil('\n');

        manual_i.light = (_BV(7) & change[0]) >> 7;
        manual_i.blind = (_BV(5) & change[0]) >> 5; // | (_BV(6) & change[0]) >> 6
        manual_i.ac = (_BV(4) & change[0]) >> 4;
        manual_i.fan = (_BV(3) & change[0]) >> 3;
        manual_i.water_pump = (_BV(2) & change[0]) >> 2;
        manual_i.garage_door = (_BV(1) & change[0]) >> 1;
        manual_i.lpg_leak = (_BV(0) & change[0]) >> 0;
        manual_i.buzzer = (_BV(7) & change[1]) >> 7; // remaining 7 bits are unused
        manual_i.neopix_color = r << 16 | g << 8 | b;

        print(DEBUG, "INFO: change = %d%d%d%d%d%d%d%d %lu -> digits and NOT bit", manual_i.light, manual_i.blind, manual_i.ac, manual_i.fan, manual_i.water_pump, manual_i.garage_door, manual_i.lpg_leak, manual_i.buzzer, manual_i.neopix_color);

        manual_mode.light = (_BV(7) & mask[0]) >> 7;
        manual_mode.blind = (_BV(6) & mask[0]) >> 6;
        manual_mode.ac = (_BV(5) & mask[0]) >> 5;
        manual_mode.fan = (_BV(4) & mask[0]) >> 4;
        manual_mode.water_pump = (_BV(3) & mask[0]) >> 3;
        manual_mode.garage_door = (_BV(2) & mask[0]) >> 2;
        manual_mode.lpg_leak = (_BV(1) & mask[0]) >> 1;
        manual_mode.buzzer = (_BV(0) & mask[0]) >> 0;
        manual_mode.neopix_color = (_BV(7) & mask[1]) >> 7;

        print(DEBUG, "INFO: mask = %d%d%d%d%d%d%d%d %lu", manual_mode.light, manual_mode.blind, manual_mode.ac, manual_mode.fan, manual_mode.water_pump, manual_mode.garage_door, manual_mode.lpg_leak, manual_mode.buzzer, manual_mode.neopix_color);
    }
    else if (c == '!')
    {
        user_settings.is_home = com_ch_uno.readStringUntil(' ') == "1" ? true : false;
        user_settings.ldr_th_min = com_ch_uno.readStringUntil(' ').toInt();
        user_settings.ldr_th_max = com_ch_uno.readStringUntil(' ').toInt();
        user_settings.dht_t_th_min = com_ch_uno.readStringUntil(' ').toFloat();
        user_settings.dht_t_th_max = com_ch_uno.readStringUntil(' ').toFloat();
        user_settings.dht_h_th = com_ch_uno.readStringUntil(' ').toFloat();
        user_settings.uss_th_min = com_ch_uno.readStringUntil(' ').toInt();
        user_settings.uss_th_max = com_ch_uno.readStringUntil(' ').toInt();
        user_settings.mq2_th = com_ch_uno.readStringUntil('\n').toInt();

        print(DEBUG, "INFO: settings = %d %d %d %.2f %.2f %.2f %d %d %d", user_settings.is_home, user_settings.ldr_th_min, user_settings.ldr_th_max, user_settings.dht_t_th_min, user_settings.dht_t_th_max, user_settings.dht_h_th, user_settings.uss_th_min, user_settings.uss_th_max, user_settings.mq2_th);
    }

    while (com_ch_uno.read() != -1)
        ;
}

void fetch_and_update()
{
    // TODO: chech if all the parameters being send in the app are with in the "RANGE"
    if (com_ch_uno.read() != ':')
    {
        while (com_ch_uno.read() != -1)
            ;
        return;
    }
    _parse_command();

    if (manual_mode.light)
    { // 0
        digitalWrite(RELAY_SSR_PIN, manual_i.light);
        output_dev_status.light = manual_i.light;
    }
    if (manual_mode.blind && output_dev_status.blind != manual_i.blind)
    { // 1
        if (output_dev_status.blind == 0)
        {
            window_blind(false, 1 / manual_i.blind);
        }
        else if (output_dev_status.blind == 1)
        {
            window_blind(true, manual_i.blind == 0 ? 1 : 0.5);
        }
        else
        {
            window_blind(!manual_i.blind, 0.5);
        }
        output_dev_status.blind = manual_i.blind;
    }
    if (manual_mode.ac)
    { // 2
        output_dev_status.ac = manual_i.ac;
    }
    if (manual_mode.fan)
    { // 3
        output_dev_status.fan = manual_i.fan;
    }
    // WARNING: BLOCKING CODE -> solution=interrupt
    if (manual_mode.water_pump && manual_i.water_pump)
    { // 4
        digitalWrite(DOOR_LOCK_SOLENOID_PIN, HIGH);
        while (digitalRead(IR_PIN) == 1)
            delay(100);
        digitalWrite(DOOR_LOCK_SOLENOID_PIN, LOW);
    }
    if (manual_mode.garage_door && output_dev_status.garage_door != manual_i.garage_door)
    { // 5
        digitalWrite(DOOR_LOCK_SOLENOID_PIN, HIGH);
        delay(100);
        window_blind(!manual_i.garage_door, 4);
        delay(100);
        digitalWrite(DOOR_LOCK_SOLENOID_PIN, LOW);
        output_dev_status.garage_door = manual_i.garage_door;
    }
    if (manual_mode.lpg_leak)
    { // 6
        digitalWrite(BUZZER_PIN, !manual_i.lpg_leak);
        output_dev_status.buzzer = manual_i.lpg_leak;
        output_dev_status.lpg_leak = manual_i.lpg_leak;
    }
    if (manual_mode.buzzer)
    { // 7
        digitalWrite(BUZZER_PIN, !manual_mode.buzzer);
        output_dev_status.buzzer = manual_i.buzzer;
    }
    if (manual_mode.neopix_color)
    { // 8
        neo_pixels[0] = manual_i.neopix_color;
        FastLED.show();
        output_dev_status.neopix_color = manual_i.neopix_color;
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    // use this if external voltage reference is used for the ADC
    // also make sure that you DO NOT apply more than voltage at `Aref` as this WILL damage the ADC
    // analogReference(EXTERNAL);

    pinMode(PIR_PIN, INPUT);
    pinMode(IR_PIN, INPUT);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, HIGH);

    FastLED.addLeds<WS2812, NEOPIX_PIN, GRB>(neo_pixels, NUMBER_OF_NEOPIX);

    pinMode(DOOR_LOCK_SOLENOID_PIN, OUTPUT);
    pinMode(RELAY_SSR_PIN, OUTPUT);

    DDRC |= 0b00111100;  // configuring A2 to A5 as output
    PORTC |= 0b00000000; // setting initial values to zero

    Serial.begin(9600);
    com_ch_uno.begin(9600);

    println(DEBUG, F("INFO: Giving some time for sensors to stabilize."));
    delay(2000); // delay of 20-30s would suffice
    println(DEBUG, F("INFO: DONE. Ready."));
}

void loop()
{
    read_sensors();

    take_action();

    if (curr_para % prev_para)
    {
        // char buff[36];
        // sprintf(buff, ":%d %d %.2f %.2f %d %d %d", monitored_parameters.pir_motion, monitored_parameters.ldr_lux, monitored_parameters.dht_temp, monitored_parameters.dht_humi, monitored_parameters.uss_cm, monitored_parameters.mq2_lpg_ppm, monitored_parameters.ir_position);
        // com_ch_uno.println(buff);
        print(DEBUG, ":%d %d %.2f %.2f %lu %d %d", monitored_parameters.pir_motion, monitored_parameters.ldr_lux, monitored_parameters.dht_temp, monitored_parameters.dht_humi, monitored_parameters.uss_cm, monitored_parameters.mq2_lpg_ppm, monitored_parameters.ir_position);

        prev_para = curr_para;
    }

    fetch_and_update();

    blink(); // helps in detecting blocking code

    delay(DELAY_INTERVAL);
}
