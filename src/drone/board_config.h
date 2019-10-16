#ifndef _BOARD_CONFIG_
#define _BOARD_CONFIG_

#define SAVE_TO_SDCARD 0

// 0 = face-detect
// 1 = yolo
#define AI_MODEL 0
#define LOAD_KMODEL_FROM_FLASH 1

#if AI_MODEL == 0 // face-detect
    #define KMODEL_SIZE (380 * 1024)
    #define KMODEL_FILE_NAME "../../models/yolo.kmodel"
    #define KMODEL_FLASH_ADDRESS 0x0300000

    #define ANCHOR_NUM 5
    static float anchor[ANCHOR_NUM * 2] = {1.889, 2.5245, 2.9465, 3.94056, 3.99987, 5.3658, 5.155437, 6.92275, 6.718375, 9.01025};
#elif AI_MODEL == 1 // yolo
    #define KMODEL_SIZE 1351976
    #define KMODEL_FILE_NAME "../../models/detect.kmodel"
    #define KMODEL_FLASH_ADDRESS 0x0300000

    #define ANCHOR_NUM 5
    static float anchor[ANCHOR_NUM * 2] = {1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52};
#endif

#define PLL0_OUTPUT_FREQ 800000000UL
#define PLL1_OUTPUT_FREQ 400000000UL

#endif
