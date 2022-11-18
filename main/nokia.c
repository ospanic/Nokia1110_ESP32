/*  
该文件主要实现Nokia开机动画的播放，以及桌面图片显示、游戏选择
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/timer.h"
#include "esp_log.h"
#include "driver/adc.h"

/*  开机声音就是按照时间序列控制DAC产生对应的波形，系列数据是定时器的一些参数
*   在定时器中断里面，更新DAC数据，数据存储在nokia_pcm中
*/

#define WITH_RELOAD            1
#define TIMER_INTR_US          22                                    // Execution time of each ISR interval in micro-seconds
#define TIMER_DIVIDER          24
#define SEC_TO_MICRO_SEC(x)    ((x) / 1000 / 1000)                  // Convert second to micro-second
#define UNUSED_PARAM           __attribute__((unused))              // A const period parameter which equals 2 * pai, used to calculate raw dac output value.
#define TIMER_TICKS            (TIMER_BASE_CLK / TIMER_DIVIDER)     // TIMER_BASE_CLK = APB_CLK = 80MHz
#define ALARM_VAL_US           SEC_TO_MICRO_SEC(TIMER_INTR_US * TIMER_TICKS)     // Alarm value in micro-seconds

#define DAC_CHAN               DAC_CHANNEL_1             // DAC_CHANNEL_1 (GPIO25) by default

static const char *TAG = "Nokia";

//开机声音
extern const int8_t nokia_pcm_start[] asm("_binary_nokia_pcm_start");
extern const int8_t nokia_pcm_end[]   asm("_binary_nokia_pcm_end");

//开机动画图像
extern const int8_t nokia_pic00_start[] asm("_binary_00_bin_start");
extern const int8_t nokia_pic01_start[] asm("_binary_01_bin_start");
extern const int8_t nokia_pic02_start[] asm("_binary_02_bin_start");
extern const int8_t nokia_pic03_start[] asm("_binary_03_bin_start");
extern const int8_t nokia_pic04_start[] asm("_binary_04_bin_start");
extern const int8_t nokia_pic05_start[] asm("_binary_05_bin_start");
extern const int8_t nokia_pic06_start[] asm("_binary_06_bin_start");
extern const int8_t nokia_pic07_start[] asm("_binary_07_bin_start");
extern const int8_t nokia_pic08_start[] asm("_binary_08_bin_start");
extern const int8_t nokia_pic09_start[] asm("_binary_09_bin_start");
extern const int8_t nokia_pic10_start[] asm("_binary_10_bin_start");
extern const int8_t nokia_pic11_start[] asm("_binary_11_bin_start");
extern const int8_t nokia_pic12_start[] asm("_binary_12_bin_start");
extern const int8_t nokia_pic13_start[] asm("_binary_13_bin_start");
extern const int8_t nokia_pic14_start[] asm("_binary_14_bin_start");
extern const int8_t nokia_pic15_start[] asm("_binary_15_bin_start");
extern const int8_t nokia_pic16_start[] asm("_binary_16_bin_start");
extern const int8_t nokia_pic17_start[] asm("_binary_17_bin_start");
extern const int8_t nokia_pic18_start[] asm("_binary_18_bin_start");
extern const int8_t nokia_pic19_start[] asm("_binary_19_bin_start");
extern const int8_t nokia_pic20_start[] asm("_binary_20_bin_start");
extern const int8_t nokia_pic21_start[] asm("_binary_21_bin_start");
extern const int8_t nokia_pic22_start[] asm("_binary_22_bin_start");

//桌面图像
extern const int8_t nokia_desktop_start[] asm("_binary_desktop_bin_start");

//讯息有内鬼图像
extern const int8_t nokia_msg_start[] asm("_binary_msg_bin_start");

//游戏背景
extern const int8_t nokia_game_start[] asm("_binary_game_bin_start");

//游戏图标
extern const int8_t nokia_game1_start[] asm("_binary_game1_bin_start");
extern const int8_t nokia_game2_start[] asm("_binary_game2_bin_start");
extern const int8_t nokia_game3_start[] asm("_binary_game3_bin_start");
extern const int8_t nokia_game4_start[] asm("_binary_game4_bin_start");
extern const int8_t nokia_game5_start[] asm("_binary_game5_bin_start");
extern const int8_t nokia_game6_start[] asm("_binary_game6_bin_start");

//游戏图标(选中)
extern const int8_t nokia_game1s_start[] asm("_binary_game1s_bin_start");
extern const int8_t nokia_game2s_start[] asm("_binary_game2s_bin_start");
extern const int8_t nokia_game3s_start[] asm("_binary_game3s_bin_start");
extern const int8_t nokia_game4s_start[] asm("_binary_game4s_bin_start");
extern const int8_t nokia_game5s_start[] asm("_binary_game5s_bin_start");
extern const int8_t nokia_game6s_start[] asm("_binary_game6s_bin_start");

//下列变量用来记录Nokia开机声音的长度以及播放位置
static int g_index = 0, sound_len = 0;

/* Timer interrupt service routine */
static void IRAM_ATTR timer0_ISR(void *ptr)
{
    TIMERG0.int_clr_timers.t0 = 1;//清理中断
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;//开中断
    g_index++;
    if (g_index > sound_len) 
    {
        TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_DIS;//开中断
        timer_pause(TIMER_GROUP_0, TIMER_0);
        return;
    }

    dac_output_voltage(DAC_CHAN, *(nokia_pcm_start + g_index) + 128);
}

/* Timer group0 TIMER_0 initialization */
static void example_timer_init(int timer_idx, bool auto_reload)
{
    esp_err_t ret;
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .intr_type = TIMER_INTR_LEVEL,
        .auto_reload = auto_reload,
    };

    ret = timer_init(TIMER_GROUP_0, timer_idx, &config);
    ESP_ERROR_CHECK(ret);
    ret = timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
    ESP_ERROR_CHECK(ret);
    ret = timer_set_alarm_value(TIMER_GROUP_0, timer_idx, ALARM_VAL_US);
    ESP_ERROR_CHECK(ret);
    ret = timer_enable_intr(TIMER_GROUP_0, timer_idx);
    ESP_ERROR_CHECK(ret);
    /* Register an ISR handler */
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer0_ISR, (void *)timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

//下列函数在spi_lcd.c中
extern void st7789_init(void);
extern void st7789_draw_pic(const uint16_t xs, const uint16_t ys, const uint16_t width, const uint16_t height, const uint8_t data[]);
extern void st7789_clear(const uint16_t color);

//按键初始化，该函数在psxcontroller.c中
void psxcontrollerInit(void);

//游戏序号，该变量在main.c 中，用来决定进行入哪个游戏
extern int game_num;

//显示游戏图标的函数
void game_icon_dis(int icon_num, int normal)
{
    if(normal == 0)//显示未选中图标
    {
        if(icon_num == 1) st7789_draw_pic(68,30,64,64,(const uint8_t *)nokia_game1_start);
        else if(icon_num == 2) st7789_draw_pic(138,30,64,64,(const uint8_t *)nokia_game2_start);
        else if(icon_num == 3) st7789_draw_pic(208,30,64,64,(const uint8_t *)nokia_game3_start);
        else if(icon_num == 4) st7789_draw_pic(68,105,64,64,(const uint8_t *)nokia_game4_start);
        else if(icon_num == 5) st7789_draw_pic(138,105,64,64,(const uint8_t *)nokia_game5_start);
        else if(icon_num == 6) st7789_draw_pic(208,105,64,64,(const uint8_t *)nokia_game6_start);
    }
    else if(normal == 1)//显示选中图标
    {
        if(icon_num == 1) st7789_draw_pic(68,30,64,64,(const uint8_t *)nokia_game1s_start);
        else if(icon_num == 2) st7789_draw_pic(138,30,64,64,(const uint8_t *)nokia_game2s_start);
        else if(icon_num == 3) st7789_draw_pic(208,30,64,64,(const uint8_t *)nokia_game3s_start);
        else if(icon_num == 4) st7789_draw_pic(68,105,64,64,(const uint8_t *)nokia_game4s_start);
        else if(icon_num == 5) st7789_draw_pic(138,105,64,64,(const uint8_t *)nokia_game5s_start);
        else if(icon_num == 6) st7789_draw_pic(208,105,64,64,(const uint8_t *)nokia_game6s_start);
    }
}

//Nokia开机画面及声音
void nokia_start(void)
{
    esp_err_t ret;

    //计算音频长度
    sound_len = nokia_pcm_end - nokia_pcm_start;

    st7789_init(); //初始化显示屏
	st7789_clear(0xFFFF);//清屏

    example_timer_init(TIMER_0, WITH_RELOAD); //启动开机音乐播放
    ret = dac_output_enable(DAC_CHAN);
    ESP_ERROR_CHECK(ret);

    //初始化按键
    psxcontrollerInit();

    printf("Nokia Starting ... ...\r\n");
	
    //播放开机动画
    st7789_draw_pic(55,50,32*1,58,(const uint8_t *)nokia_pic00_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*2,93,(const uint8_t *)nokia_pic01_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*2,103,(const uint8_t *)nokia_pic02_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50+23,32*7,100,(const uint8_t *)nokia_pic03_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50+23,32*7,100,(const uint8_t *)nokia_pic04_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50+23,32*7,86,(const uint8_t *)nokia_pic05_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50+23,32*7,87,(const uint8_t *)nokia_pic06_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50+23,32*7,85,(const uint8_t *)nokia_pic07_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50+18,32*7,90,(const uint8_t *)nokia_pic08_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50+9,32*7,98,(const uint8_t *)nokia_pic09_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50+4,32*7,99,(const uint8_t *)nokia_pic10_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,105,(const uint8_t *)nokia_pic11_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,105,(const uint8_t *)nokia_pic12_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,105,(const uint8_t *)nokia_pic13_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,100,(const uint8_t *)nokia_pic14_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,100,(const uint8_t *)nokia_pic15_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,100,(const uint8_t *)nokia_pic16_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,118,(const uint8_t *)nokia_pic17_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,118,(const uint8_t *)nokia_pic18_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,118,(const uint8_t *)nokia_pic19_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,118,(const uint8_t *)nokia_pic20_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,118,(const uint8_t *)nokia_pic21_start);
    vTaskDelay(100);
    st7789_draw_pic(55,50,32*7,118,(const uint8_t *)nokia_pic22_start);
    vTaskDelay(2000);

    st7789_draw_pic(40,10,32*8,220,(const uint8_t *)nokia_desktop_start);

    ret = dac_output_disable(DAC_CHAN);

    //记录当前处于那个页面，0桌面，1讯息，2游戏选择
    int page_index = 0;

    while(1) //桌面、讯息及游戏选择页面切换
    {
        if(gpio_get_level(0) == 0) //右键
        {
            while(gpio_get_level(0) == 0){vTaskDelay(10);}
            if(page_index ==0)//桌面进入游戏界面
            {
                page_index = 2;
                st7789_draw_pic(40,10,32*8,220,(const uint8_t *)nokia_game_start);

                game_icon_dis(1, game_num == 1?1:0);
                game_icon_dis(2, game_num == 2?1:0);
                game_icon_dis(3, game_num == 3?1:0);
                game_icon_dis(4, game_num == 4?1:0);
                game_icon_dis(5, game_num == 5?1:0);
                game_icon_dis(6, game_num == 6?1:0);           
            }
            else //其他界面返回桌面
            {
                page_index = 0;
                st7789_draw_pic(40,10,32*8,220,(const uint8_t *)nokia_desktop_start);
            }
        }

        if(gpio_get_level(22) == 0) //左键
        {
            while(gpio_get_level(22) == 0){vTaskDelay(10);}
            if(page_index ==0)//桌面进入讯息界面
            {
                page_index = 1;
                st7789_draw_pic(40,10,32*8,220,(const uint8_t *)nokia_msg_start);
            }
            if(page_index ==2)//游戏界面确认选择游戏
            {
                page_index = 0;
                break;
            }
        }

        if(gpio_get_level(26) == 0) //挂机键
        {
            if(page_index != 0)
            {
                page_index = 0;
                st7789_draw_pic(40,10,32*8,220,(const uint8_t *)nokia_desktop_start);   
            }
        }

        if(page_index == 2)//游戏页面选择游戏
        {
            uint32_t adc_reading = 0;
            for (int i = 0; i < 1; i++) 
            {
                adc_reading += adc1_get_raw(ADC_CHANNEL_6); //方向键
            }
            adc_reading /= 1;

            if(adc_reading < 100)//上
            {
                game_icon_dis(game_num,0);
                if(game_num > 3) game_num -= 3;
                game_icon_dis(game_num,1);
                vTaskDelay(200);
            }
            else if(adc_reading < 1000)//左
            {
                game_icon_dis(game_num,0);
                if((game_num != 1) &&(game_num != 4))  game_num -= 1;
                game_icon_dis(game_num,1);
                vTaskDelay(200);
            }
            else if(adc_reading < 2400)//下
            {
                game_icon_dis(game_num,0);
                if(game_num < 4) game_num += 3;
                game_icon_dis(game_num,1);
                vTaskDelay(200);
            }
            else if(adc_reading < 3300)//右
            {
                game_icon_dis(game_num,0);
                if((game_num != 3) && (game_num != 6))  game_num += 1;
                game_icon_dis(game_num,1);
                vTaskDelay(200);
            }
            //printf("Raw: %d\n", adc_reading);
        }
    }
}
