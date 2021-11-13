#include <misc.h>

/* Check calibration data table on ESP32*/
void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

/*Print Vref by efuse or default*/
void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void adc_config(){

	//Check if Two Point or Vref are burned into eFuse
	check_efuse();
	//Configure ADC

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL_0, atten);		
  adc1_config_channel_atten(ADC_CHANNEL_3, atten);	
  adc1_config_channel_atten(ADC_CHANNEL_4, atten);			
  adc1_config_channel_atten(ADC_CHANNEL_6, atten);		
  adc1_config_channel_atten(ADC_CHANNEL_7, atten);		

  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);
}

double adc_read(int addr,esp_adc_cal_characteristics_t *adc_chars){
	uint32_t adc_reading = 0;
    float force=0;
    switch(addr){
      case 0:
      channel = ADC_CHANNEL_0; 
      break;
      case 1:
      channel = ADC_CHANNEL_3;
      break;
      case 2:
      channel = ADC_CHANNEL_4;
      break;
      case 3:
      channel = ADC_CHANNEL_6;
      break;
      case 4:
      channel = ADC_CHANNEL_7;
      break;
  }
  /*for (int i = sampleSize-1 ;i>=0;i--){

      adc_reading += adc1_get_raw((adc1_channel_t)channel)*pow((1-weight),i);

  }
  adc_reading = adc_reading*weight;*/
  adc_reading = adc1_get_raw((adc1_channel_t)channel);
  //printf("read adc: %d\t",adc_reading );

		//Convert adc_reading to voltage in mV
  double voltage = 3300-((int16_t) esp_adc_cal_raw_to_voltage(adc_reading, adc_chars)-142);
  //printf("voltage: %f\t",voltage );

  force = 0.00006*voltage*voltage+0.2555*voltage+17.04;
  //printf("force: %f\n",force );

  return force;	
}

void mux_selector_config(){
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);
}
