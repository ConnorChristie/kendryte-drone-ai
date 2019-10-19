#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "board_config.h"
#include "bsp.h"
#include "dvp.h"
#include "fpioa.h"
#include "gpiohs.h"
#include "image_process.h"
#include "kpu.h"
#include "ov2640.h"
#include "plic.h"
#include "region_layer.h"
#include "sysctl.h"
#include "uarths.h"
#include "utils.h"
#include "w25qxx.h"
#include "sdcard.h"
#include "ff.h"
#include "rgb2bmp.h"
#include "multiwii.h"
#include "serial.h"
#include "sbus.h"

#define INCBIN_STYLE INCBIN_STYLE_SNAKE
#define INCBIN_PREFIX
#include "incbin.h"

volatile uint8_t g_ram_mux = 0;
volatile uint8_t g_ai_done_flag;
volatile uint8_t g_dvp_finish_flag;

uint32_t g_lcd_gram0[38400] __attribute__((aligned(64)));
uint32_t g_lcd_gram1[38400] __attribute__((aligned(64)));
uint8_t g_ai_buf[320 * 240 *3] __attribute__((aligned(128)));

kpu_model_context_t face_detect_task;
static region_layer_t face_detect_rl;
static obj_info_t face_detect_info;

#if LOAD_KMODEL_FROM_FLASH
    uint8_t model_data[KMODEL_SIZE];
#else
    INCBIN(model, KMODEL_FILE_NAME);
#endif

static int ai_done(void *ctx)
{
    g_ai_done_flag = 1;
    return 0;
}

static int dvp_irq(void *ctx)
{
    if (dvp_get_interrupt(DVP_STS_FRAME_FINISH))
    {
        dvp_set_display_addr(g_ram_mux ? (uint32_t)g_lcd_gram0 : (uint32_t)g_lcd_gram1);

        dvp_clear_interrupt(DVP_STS_FRAME_FINISH);
        g_dvp_finish_flag = 1;
    }
    else
    {
        if (g_dvp_finish_flag == 0)
            dvp_start_convert();
        dvp_clear_interrupt(DVP_STS_FRAME_START);
    }
    return 0;
}

static void io_mux_init(void)
{
    /* SD card */
    fpioa_set_function(27, FUNC_SPI1_SCLK);
    fpioa_set_function(28, FUNC_SPI1_D0);
    fpioa_set_function(26, FUNC_SPI1_D1);
    fpioa_set_function(29, FUNC_GPIOHS7);

    /* Init DVP IO map and function settings */
    fpioa_set_function(42, FUNC_CMOS_RST);
    fpioa_set_function(44, FUNC_CMOS_PWDN);
    fpioa_set_function(46, FUNC_CMOS_XCLK);
    fpioa_set_function(43, FUNC_CMOS_VSYNC);
    fpioa_set_function(45, FUNC_CMOS_HREF);
    fpioa_set_function(47, FUNC_CMOS_PCLK);
    fpioa_set_function(41, FUNC_SCCB_SCLK);
    fpioa_set_function(40, FUNC_SCCB_SDA);

    sysctl_set_spi0_dvp_data(1);

    /* Serial connection to Drone */
    fpioa_set_function(24, FUNC_UART1_TX);
    fpioa_set_function(25, FUNC_UART1_RX);

    /* SBUS connection */
    fpioa_set_function(9, FUNC_UART2_RX);
    fpioa_set_function(10, FUNC_UART2_TX);

    fpioa_io_config_t cfg;
    fpioa_get_io(9, &cfg);

    cfg.di_inv = 1;
    fpioa_set_io(9, &cfg);
}

static void io_set_power(void)
{
    /* Set dvp and spi pin to 1.8V */
    sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
}

static void draw_edge(uint32_t *gram, obj_info_t *obj_info, uint32_t index, uint16_t color)
{
    uint32_t data = ((uint32_t)color << 16) | (uint32_t)color;
    uint32_t *addr1, *addr2, *addr3, *addr4, x1, y1, x2, y2;

    x1 = obj_info->obj[index].x1;
    y1 = obj_info->obj[index].y1;
    x2 = obj_info->obj[index].x2;
    y2 = obj_info->obj[index].y2;

    if(x1 <= 0)
        x1 = 1;
    if(x2 >= 319)
        x2 = 318;
    if(y1 <= 0)
        y1 = 1;
    if(y2 >= 239)
        y2 = 238;

    addr1 = gram + (320 * y1 + x1) / 2;
    addr2 = gram + (320 * y1 + x2 - 8) / 2;
    addr3 = gram + (320 * (y2 - 1) + x1) / 2;
    addr4 = gram + (320 * (y2 - 1) + x2 - 8) / 2;
    for(uint32_t i = 0; i < 4; i++)
    {
        *addr1 = data;
        *(addr1 + 160) = data;
        *addr2 = data;
        *(addr2 + 160) = data;
        *addr3 = data;
        *(addr3 + 160) = data;
        *addr4 = data;
        *(addr4 + 160) = data;
        addr1++;
        addr2++;
        addr3++;
        addr4++;
    }
    addr1 = gram + (320 * y1 + x1) / 2;
    addr2 = gram + (320 * y1 + x2 - 2) / 2;
    addr3 = gram + (320 * (y2 - 8) + x1) / 2;
    addr4 = gram + (320 * (y2 - 8) + x2 - 2) / 2;
    for(uint32_t i = 0; i < 8; i++)
    {
        *addr1 = data;
        *addr2 = data;
        *addr3 = data;
        *addr4 = data;
        addr1 += 160;
        addr2 += 160;
        addr3 += 160;
        addr4 += 160;
    }
}

void print_bytes(char* data, size_t length)
{
    // Print out raw bytes
    for (unsigned int i = 0; i < length; i++)
    {
        // char x = (data[i] & 0x0F) << 4 | (data[i] & 0xF0) >> 4;
        printf("%02X ", (unsigned char)data[i]);
    }

    printf("\n");
}

int main(void)
{
    FATFS sdcard_fs;

    /* Set CPU and dvp clk */
    sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_OUTPUT_FREQ);
    sysctl_pll_set_freq(SYSCTL_PLL1, PLL1_OUTPUT_FREQ);
    sysctl_clock_enable(SYSCTL_CLOCK_AI);

    uarths_init();
    io_mux_init();
    io_set_power();
    plic_init();

#if LOAD_KMODEL_FROM_FLASH
    /* flash init */
    printf("flash init\n");
    w25qxx_init(3, 0);
    w25qxx_enable_quad_mode();
    w25qxx_read_data(KMODEL_FLASH_ADDRESS, model_data, KMODEL_SIZE, W25QXX_QUAD_FAST);
#endif

    /* DVP init */
    printf("DVP init\n");
    dvp_init(8);
    dvp_set_xclk_rate(24000000);
    dvp_enable_burst();
    dvp_set_output_enable(DVP_OUTPUT_AI, 1);
    dvp_set_output_enable(DVP_OUTPUT_DISPLAY, 1);
    dvp_set_image_format(DVP_CFG_RGB_FORMAT);
    dvp_set_image_size(320, 240);
    ov2640_init();

    dvp_set_ai_addr((uint32_t)g_ai_buf, (uint32_t)(g_ai_buf + 320 * 240), (uint32_t)(g_ai_buf + 320 * 240 * 2));
    dvp_set_display_addr((uint32_t)g_lcd_gram0);
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
    dvp_disable_auto();

    /* DVP interrupt config */
    printf("DVP interrupt config\n");
    plic_set_priority(IRQN_DVP_INTERRUPT, 1);
    plic_irq_register(IRQN_DVP_INTERRUPT, dvp_irq, NULL);
    plic_irq_enable(IRQN_DVP_INTERRUPT);

    /* enable global interrupt */
    sysctl_enable_irq();

    g_ram_mux = 0;
    g_dvp_finish_flag = 0;
    dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);

    /* Init face detect model */
    if (kpu_load_kmodel(&face_detect_task, model_data) != 0)
    {
        printf("Model init error\n");
        return -1;
    }

#if SAVE_TO_SDCARD
    /* SD card init */
    if (sd_init())
    {
        printf("Fail to init SD card\n");
        return -1;
    }

    /* mount file system to SD card */
    if (f_mount(&sdcard_fs, _T("0:"), 1))
    {
        printf("Fail to mount file system\n");
        return -1;
    }
#endif

    Serial *msp = new Serial(UART_DEVICE_1, 115200, UART_BITWIDTH_8BIT, UART_STOP_1, UART_PARITY_NONE);
    SBUS::init(UART_DEVICE_2);

    face_detect_rl.anchor_number = ANCHOR_NUM;
    face_detect_rl.anchor = anchor;
    face_detect_rl.threshold = 0.7;
    face_detect_rl.nms_value = 0.3;
#if AI_MODEL == 0
    region_layer_init(&face_detect_rl, 20, 15, 30, 320, 240);
#elif AI_MODEL == 1
    region_layer_init(&face_detect_rl, 10, 7, 125, 320, 240);
#endif

    /* system start */
    printf("System start\n");
    while (1)
    {
        while (g_dvp_finish_flag == 0);

        /* Run face detect */
        kpu_run_kmodel(&face_detect_task, g_ai_buf, DMAC_CHANNEL5, ai_done, NULL);
        while (!g_ai_done_flag);
        g_ai_done_flag = 0;

        float *output;
        size_t output_size;
        kpu_get_output(&face_detect_task, 0, (uint8_t **)&output, &output_size);

        face_detect_rl.input = output;
        region_layer_run(&face_detect_rl, &face_detect_info);

        /* Run key point detect */
        printk("Detected %u faces\n", face_detect_info.obj_number);
        for (uint32_t face_cnt = 0; face_cnt < face_detect_info.obj_number; face_cnt++)
        {
            draw_edge((uint32_t *)(g_ram_mux ? g_lcd_gram0 : g_lcd_gram1), &face_detect_info, face_cnt, 0xF800);
        }

        g_ram_mux ^= 0x01;
        g_dvp_finish_flag = 0;

#if SAVE_TO_SDCARD
        if (face_detect_info.obj_number > 0)
        {
            rgb565tobmp(g_lcd_gram, 320, 240, _T("0:photo.bmp"));
        }
#endif

        Msp::DroneReceiver params = {
            .roll        = 1300,
            .pitch       = 1300,
            .throttle    = (uint16_t)(1200),
            .yaw         = 1300,
            .flight_mode = 1000, // DISABLE = Horizon mode
            .aux_2       = 1300,
            .arm_mode    = 1300
        };
        Msp::send_command<Msp::DroneReceiver>(msp, Msp::MspCommand::SET_RAW_RC, &params);
    }
}
