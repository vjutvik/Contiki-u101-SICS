#ifndef STM32_ID_H
#define STM32_ID_H

/**
 * \file  Support for "Device electronic signature" in STM32
 *        devices. This includes device ID and flash size. 
 *        MCU device id and revision also ended up here.
 */



/* From ST AN2606 */
#define STM32_DEV_ID_MEDIUM_DENSITY            0x410
#define STM32_DEV_ID_STM32F2XX_DEVICES         0x411
#define STM32_DEV_ID_LOW_DENSITY               0x412
#define STM32_DEV_ID_STM32F4XX_DEVICES         0x413
#define STM32_DEV_ID_HIGH_DENSITY              0x414
#define STM32_DEV_ID_MEDIUM_DENSITY_ULP        0x416
#define STM32_DEV_ID_CONNECTIVITY_LINE         0x418
#define STM32_DEV_ID_MEDIUM_DENSITY_VALUE_LINE 0x420
#define STM32_DEV_ID_HIGH_DENSITY_VALUE_LINE   0x428
#define STM32_DEV_ID_XL_DENSITY                0x430

/**

*/
#define stm32_id_flash_size_kb stm32_id_arch_flash_size_kb
void stm32_id_arch_flash_size_kb(uint16_t *dest);

/**

*/
#define stm32_id_chipinfo stm32_id_arch_chipinfo
void stm32_id_arch_chipinfo(uint16_t *device, uint16_t *revision);

/**

*/
#define stm32_id_uid96 stm32_id_arch_uid96
void stm32_id_arch_uid96(uint16_t *dest);



#endif

