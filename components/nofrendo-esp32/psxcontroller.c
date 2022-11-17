// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "psxcontroller.h"
#include "sdkconfig.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define bit_joypad1_select 0
#define bit_joypad1_start  3
#define bit_joypad1_up     4
#define bit_joypad1_right  5
#define bit_joypad1_down   6
#define bit_joypad1_left   7
#define bit_soft_reset     12
#define bit_joypad1_a      13
#define bit_joypad1_b      14
#define bit_hard_reset     15


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   1          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t channel_d4  = ADC_CHANNEL_6;     //方向键 GPIO34
static const adc_channel_t channel_147 = ADC_CHANNEL_0;		//方向键 GPIO34
static const adc_channel_t channel_258 = ADC_CHANNEL_3;		//方向键 GPIO34
static const adc_channel_t channel_369 = ADC_CHANNEL_4;		//方向键 GPIO34

static const adc_atten_t atten = ADC_ATTEN_DB_11;

static void check_efuse()
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

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void adc_key_init()
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(channel_d4, atten);//上下左右
	adc1_config_channel_atten(channel_147, atten);//147*
	adc1_config_channel_atten(channel_258, atten);//2580
	adc1_config_channel_atten(channel_369, atten);//369#
 
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    // while (1) //ADC Key Test
	// {
    //     uint32_t adc_reading = 0;
    //     //Multisampling
    //     for (int i = 0; i < NO_OF_SAMPLES; i++) 
	// 	{
    //         adc_reading += adc1_get_raw((adc1_channel_t)channel_369);
    //     }
    //     adc_reading /= NO_OF_SAMPLES;
    //     //Convert adc_reading to voltage in mV
    //     uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    //     printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }
}



int psxReadInput() 
{
	uint16_t retval = 0;
	uint32_t adc_reading = 0;

	if(gpio_get_level(0) == 0) retval |=  1<<bit_joypad1_start;
	if(gpio_get_level(22) == 0) retval |=  1<<bit_joypad1_select;
	if(gpio_get_level(26) == 0) retval |=  1<< bit_soft_reset;

	//147
	for (int i = 0; i < NO_OF_SAMPLES; i++) 
	{
		adc_reading += adc1_get_raw((adc1_channel_t)channel_147);
	}
	adc_reading /= NO_OF_SAMPLES;

	if(adc_reading < 100) retval |=  1<<bit_joypad1_a;
	else if(adc_reading < 1000) retval |=  1<<bit_joypad1_right;
	else if(adc_reading < 2400) retval |=  1<<bit_joypad1_b;

	//258
	adc_reading = 0;
	for (int i = 0; i < NO_OF_SAMPLES; i++) 
	{
		adc_reading += adc1_get_raw((adc1_channel_t)channel_258);
	}
	adc_reading /= NO_OF_SAMPLES;

	if(adc_reading < 100) retval |=  1<<bit_joypad1_down;
	else if(adc_reading < 1000) retval |=  1<<bit_joypad1_start;
	else if(adc_reading < 2400) retval |=  1<<bit_joypad1_up;

	//369
	adc_reading = 0;
	for (int i = 0; i < NO_OF_SAMPLES; i++) 
	{
		adc_reading += adc1_get_raw((adc1_channel_t)channel_369);
	}
	adc_reading /= NO_OF_SAMPLES;

	if(adc_reading < 100) retval |=  1<<bit_joypad1_a;
	else if(adc_reading < 1000) retval |=  1<<bit_joypad1_left;
	else if(adc_reading < 2400) retval |=  1<<bit_joypad1_b;

	return (int)retval;
}

void psxcontrollerInit() 
{
	static int is_inited = 0;
	if(is_inited) 
	{
		printf("PSX controller Inited! Not again.\n");
		return;
	}


	printf("Init PSX controller.\n");
	
	gpio_set_direction(0, GPIO_MODE_INPUT); //start
	gpio_pullup_en(0);

	gpio_set_direction(22, GPIO_MODE_INPUT); //select
	gpio_pullup_en(22);

	gpio_set_direction(26, GPIO_MODE_INPUT); //select
	gpio_pullup_en(26);

	adc_key_init();

	is_inited = 1;
}


