#ifndef MISC_H
#define MISC_H 

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <stdio.h>
#include <math.h>

/* ADC configuration*/
#define DEFAULT_VREF  1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64 // Multisampling
#define sampleSize 64
#define weight 0.25

/*GPIO MUX SELECTION*/
#define pinA 5
#define pinB 17
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<pinA) | (1ULL<<pinB))

extern const float b_const , a_const;

static adc_channel_t channel = ADC_CHANNEL_0;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
esp_adc_cal_characteristics_t *adc_chars;

double adc_read(int addr,esp_adc_cal_characteristics_t *adc_chars);
void mux_selector_config();
void adc_config();
void print_char_val_type(esp_adc_cal_value_t val_type);
void check_efuse(void);

#endif