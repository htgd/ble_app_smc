/**@file    gnt_board.c
* @brief   	设备引脚分配头文件
* @details  在 gnt_config.h 中配置硬件版本 BOARD_GNT_VERSION 宏定义控制不同版本引脚分配
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-09-20
* @version 	V1.0
* @copyright	Copyright (c) 2018-2020  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* 硬件平台:nRF52832_QFAA \n
* SDK版本：nRF5_SDK_15.0.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/09/20  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*/

#ifndef GNT_BOARD_H
#define GNT_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gnt_config.h"
#include "nrf_gpio.h"

#if (BOARD_GNT_VERSION == 1)

// LEDs definitions for GNT_BOARD
#define LEDS_NUMBER     1

#define LED_1           12   //PCA10040:17     GNT:12

#define LEDS_ACTIVE_STATE   1

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1}

#define BSP_LED_0       LED_1


// 按键或IO输入中断
#define BUTTONS_NUMBER  4

#define BUTTON_1        11   //PCA10040:13     GNT:11
#define LIS_INT1        17
#define LIS_INT2        18
#define USBVIN_CHECK    7 

#define BUTTON_PULL     NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE    1

#define BUTTONS_LIST { BUTTON_1, LIS_INT1, LIS_INT2, USBVIN_CHECK}

#define BSP_BUTTON_0    BUTTON_1
#define BSP_BUTTON_1    LIS_INT1
#define BSP_BUTTON_2    LIS_INT2
#define BSP_BUTTON_3    USBVIN_CHECK


// 串口
#define RX_PIN_NUMBER   3    //8
#define TX_PIN_NUMBER   2    //6
#define CTS_PIN_NUMBER  4
#define RTS_PIN_NUMBER  5

// I2C
#define TWI_SCL_M       20       //!< Master SCL pin.
#define TWI_SDA_M       19       //!< Master SDA pin.

// GNT端口使用定义（输出或ADC）
#define NB_PWRON        27
#define NB_WKUP         26
#define NB_RESET        25
#define ADC_BAT_EN      6

#define BATTERY_ADC_CH      NRF_SAADC_INPUT_AIN3


#elif (BOARD_GNT_VERSION == 2)

// LEDs definitions for GNT_BOARD
#define LEDS_NUMBER     1

#define LED_1           26   //PCA10040:17     GNT:12

#define LEDS_ACTIVE_STATE   1

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1}

#define BSP_LED_0       LED_1


// 按键或IO输入中断
#define BUTTONS_NUMBER  4

#define BUTTON_1        25   //PCA10040:13     GNT:11
#define LIS_INT1        17
#define LIS_INT2        18
#define USBVIN_CHECK    11 

#define BUTTON_PULL     NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE    1

#define BUTTONS_LIST { BUTTON_1, LIS_INT1, LIS_INT2, USBVIN_CHECK}

#define BSP_BUTTON_0    BUTTON_1
#define BSP_BUTTON_1    LIS_INT1
#define BSP_BUTTON_2    LIS_INT2
#define BSP_BUTTON_3    USBVIN_CHECK


// 串口
#define RX_PIN_NUMBER   3    //8
#define TX_PIN_NUMBER   2    //6
#define CTS_PIN_NUMBER  4
#define RTS_PIN_NUMBER  5

// I2C
#define TWI_SCL_M       20       //!< Master SCL pin.
#define TWI_SDA_M       19       //!< Master SDA pin.

// GNT端口使用定义（输出或ADC）
#define NB_PWRON        8
#define NB_WKUP         7
#define NB_RESET        4
#define ADC_BAT_EN      6

#define BATTERY_ADC_CH      NRF_SAADC_INPUT_AIN3


#elif (BOARD_GNT_VERSION == 3)

// LEDs definitions for GNT_BOARD
#define LEDS_NUMBER     1

#define LED_1           29

#define LEDS_ACTIVE_STATE   1

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1}

#define BSP_LED_0       LED_1


// 按键或IO输入中断
#define BUTTONS_NUMBER  4

#define BUTTON_1        26  //PCA10040:13     GNT:11
#define LIS_INT1        11
#define LIS_INT2        12
#define USBVIN_CHECK    4

#define BUTTON_PULL     NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE    1

#define BUTTONS_LIST { BUTTON_1, LIS_INT1, LIS_INT2, USBVIN_CHECK}

#define BSP_BUTTON_0    BUTTON_1
#define BSP_BUTTON_1    LIS_INT1
#define BSP_BUTTON_2    LIS_INT2
#define BSP_BUTTON_3    USBVIN_CHECK


// 串口
#define RX_PIN_NUMBER   2
#define TX_PIN_NUMBER   3 
#define CTS_PIN_NUMBER  13
#define RTS_PIN_NUMBER  14

// I2C
#define TWI_SCL_M       8           //!< Master SCL pin.
#define TWI_SDA_M       7           //!< Master SDA pin.

// GNT端口使用定义（输出或ADC）
#define NB_PWRON        6
#define NB_RESET        5
#define ADC_BAT_EN      27
#define GPS_LNA_EN      13          //跳线版（11），更新跳线版程序是修改LIS_INT1为其他引脚

#define BATTERY_ADC_CH      NRF_SAADC_INPUT_AIN4

#endif



#ifdef __cplusplus
}
#endif

#endif // GNT_BOARD_H


