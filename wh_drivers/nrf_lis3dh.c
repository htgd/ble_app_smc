/**@file    nrf_lis3dh.c
* @brief   	基于nRF的lis3DH驱动
* @details  
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-08-17
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  江苏亨通光网科技有限公司
**********************************************************************************
* @attention
* 硬件平台:nRF52832_QFAA \n
* SDK版本：nRF5_SDK_15.0.0
* @par 修改日志:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/17  <td>1.0      <td>wanghuan  <td>创建初始版本
* </table>
*/

#include "nrf_lis3dh.h"
#if (GNT_LIS3DH_EN == 1)

void lis3dh_twi_enable(void);
void lis3dh_twi_disable(void);
void lis3dh_power_off(void);
void lis3dh_ResetInt1Latch(uint8_t err_try);


// TWI操作使用（三轴加速传感器）*****************************************************************************************************

/* Master Configuration */
#define MASTER_TWI_INST     0       //!< TWI interface used as a master accessing EEPROM memory.

#define LIS3DH_ADDR         0x19 					///< 从机地址
#define TWI_SLAVE_WRITE_MAX_BYTES           50      ///< 从机单次可写入最大长度

static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);	///< twi主机句柄定义

/**@brief   I2C主机初始化
* @return 	Status [NRFX_ERROR_ , NRFX_SUCCESS]
*/
static ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_MID,
       .clear_bus_init     = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

    if (NRFX_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }

    return ret;
}

/**@brief IIC写操作
* @param[in] addr	 i2c寻址地址
* @param[in] *pdata  要写入的数据
* @param[in] size    要写入的数据的长度
* @return 	Status [MEMS_ERROR , MEMS_SUCCESS]
*/
static status_t lis3dh_i2c_write(uint8_t addr, uint8_t const * pdata, uint8_t size)
{
    uint8_t buffer[size+1]; // Addr + data
    
    memcpy(buffer, &addr, 1);   //复制寻址地址（1个字节）
    memcpy(buffer + 1, pdata, size);
    if(NRFX_SUCCESS == nrf_drv_twi_tx(&m_twi_master, LIS3DH_ADDR, buffer, size + 1, false))
    {
        return MEMS_SUCCESS;
    }
    else
        return MEMS_ERROR;
}

/**@brief I2C读操作
* @param[in] addr	 i2c寻址地址
* @param[out] *pdata 读出的数据
* @param[in] size    要读出的数据的长度
* @return 	Status [MEMS_ERROR , MEMS_SUCCESS]
*/
static status_t lis3dh_i2c_read(uint8_t addr, uint8_t * pdata, uint8_t size)
{
    if(NRFX_SUCCESS == nrf_drv_twi_tx(&m_twi_master, LIS3DH_ADDR, &addr, 1, true))  //发送寻址地址
    {
        if(NRFX_SUCCESS == nrf_drv_twi_rx(&m_twi_master, LIS3DH_ADDR, pdata, size)) //i2c读取
        {
            return MEMS_SUCCESS;
        }
        else
            return MEMS_ERROR;
    }
    else
        return MEMS_ERROR;
}

static uint8_t nrf_twi_enabled;         ///< i2c外设是否使能 0:关闭，1：使能
static uint8_t lis3dh_twi_registed;     ///< lis3dh的i2c操作函数是否注册



/**@brief 开启I2C外设
*/
void lis3dh_twi_enable(void)
{
    if(nrf_twi_enabled == 0)
        twi_master_init();
    nrf_twi_enabled = 1;
//    nrf_drv_twi_enable(&m_twi_master);
}

/**@brief 关闭I2C外设
*/
void lis3dh_twi_disable(void)
{
    if(nrf_twi_enabled == 1)
        nrf_drv_twi_uninit(&m_twi_master);
    nrf_twi_enabled = 0;
//    nrf_drv_twi_disable(&m_twi_master);
}

/**@brief lis3dh进入掉电模式
*/
void lis3dh_power_off(void)
{
    lis3dh_twi_enable();
    if(lis3dh_twi_registed == 0)
    {
        lis3dh_regist_init(lis3dh_i2c_write, lis3dh_i2c_read);
        lis3dh_twi_registed = 1;
    }
    LIS3DH_SetMode(LIS3DH_POWER_DOWN);
    lis3dh_twi_disable();
}

/**@brief lis3dh清中断锁存
*/
void lis3dh_ResetInt1Latch(uint8_t err_try)
{
    uint8_t rec;
    lis3dh_twi_enable();
    do{
        rec = LIS3DH_ResetInt1Latch();
    }while((rec != MEMS_SUCCESS)&&(err_try--));
    lis3dh_twi_disable();
    
    if(rec != MEMS_SUCCESS)
    {
        lis3dh_twi_enable();
        do{
            rec = LIS3DH_ResetInt1Latch();
        }while((rec != MEMS_SUCCESS)&&(err_try--));
        lis3dh_twi_disable();
    }
}

#if 0
/**@brief lis3dh读取辅助ADC温度
*/
int8_t lis3dh_GetTemp(uint8_t err_try)
{
    uint8_t rec;
    int8_t temp;
    lis3dh_twi_enable();
    do{
        rec = LIS3DH_GetTempRaw(&temp);
        NRF_LOG_INFO("Temperature:%d.", temp);
    }while((rec != MEMS_SUCCESS)&&(err_try--));
    lis3dh_twi_disable();
    
    return temp;
}

/**@brief lis3dh驱动注册和内部温度传感使能
*/
void lis3dh_reg_temp(void)
{
    uint8_t tmp;
    lis3dh_twi_enable();
    
    lis3dh_regist_init(lis3dh_i2c_write, lis3dh_i2c_read);  //注册I2C读写函数
    
    LIS3DH_GetWHO_AM_I(&tmp);
    NRF_LOG_INFO("WHO_AM_I: 0X%02x", tmp);
    
    LIS3DH_SetMode(LIS3DH_LOW_POWER);   // 设置工作电源模式
    LIS3DH_SetODR(LIS3DH_ODR_1Hz);      // 设置速率,打开设备
    LIS3DH_SetTemperature(MEMS_ENABLE); //使能内部温度传感
    LIS3DH_SetADCAux(MEMS_ENABLE);      //使能辅助ADC
    LIS3DH_SetBDU(MEMS_ENABLE);         //Block data update
    
    lis3dh_twi_disable();
}
#endif

/**@brief lis3dh初始化 \n
* lis3dh运动检测中断配置
*/
void lis3dh_init(void)
{
    uint8_t tmp;
    
    lis3dh_twi_enable();
    if(lis3dh_twi_registed == 0)
    {
        lis3dh_regist_init(lis3dh_i2c_write, lis3dh_i2c_read);
        lis3dh_twi_registed = 1;
    }
    
    LIS3DH_GetWHO_AM_I(&tmp);
    NRF_LOG_INFO("WHO_AM_I: 0X%02x", tmp);
    
    // CTRL_REG1 设置采样频率为10Hz，低功耗模式，开启x,y,z轴检测
    LIS3DH_SetODR(LIS3DH_ODR_10Hz);      // 设置速率,打开设备
    LIS3DH_SetMode(LIS3DH_LOW_POWER);    // 设置工作电源模式
    LIS3DH_SetAxis(LIS3DH_X_ENABLE|LIS3DH_Y_ENABLE|LIS3DH_Z_ENABLE);// 启用三轴
    
    // CTRL_REG2 正常模式，数据从内部滤波器到输出寄存器，高通滤波器使能到中断1
    LIS3DH_HPFAOI1Enable(MEMS_ENABLE);          //滤波器应用与INT1
    LIS3DH_SetFilterDataSel(MEMS_ENABLE);       //滤波器应用与输出寄存器
//    LIS3DH_SetHPFMode(LIS3DH_HPM_AUTORESET_INT);
//    LIS3DH_SetHPFCutOFF(LIS3DH_HPFCF_1);        //设置高通滤波器截止频率
    LIS3DH_GetReference(&tmp);                  //第一次虚读reference寄存器，强制将高通滤波器的值加载到当前加速度值,从这时开始比较了
    
    // CTRL_REG4 设置能测量的重力加速度 FS = 2 g
    LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
    
    // INT1_ON_PIN_INT2
//    LIS3DH_Int1LatchEnable(MEMS_ENABLE);    // 中断锁存 MEMS_DISABLE
    LIS3DH_Int1LatchEnable(MEMS_DISABLE);   
    LIS3DH_SetInt1Threshold(10);            //设置中断阈值 16*16mg
    LIS3DH_SetInt1Duration(1);              //中断持续时间设置为100ms，(设置中断锁存，所以中断不会消失，直到手动清除中断)
    LIS3DH_SetIntMode(LIS3DH_INT_MODE_OR);  // 静止时进行运动检测
    LIS3DH_SetIntConfiguration(LIS3DH_INT1_XHIE_ENABLE|LIS3DH_INT1_YHIE_ENABLE|LIS3DH_INT1_ZHIE_ENABLE);    //int1中断源配置
    
    LIS3DH_SetInt2Pin(LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE);   //使能引脚2中断
//    LIS3DH_SetInt2Pin(LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE);   //使能引脚1中断
//    LIS3DH_ResetInt1Latch();    //清中断
    
    lis3dh_twi_disable();
}



#if 0
// lis3dh加速度传感器初始化
void lis3dh_init(void)
{
    uint8_t tmp;
    
    twi_master_init();
    nrf_twi_enabled = 1;    //twi外设使能
    lis3dh_regist_init(lis3dh_i2c_write, lis3dh_i2c_read);
#if 1
    LIS3DH_GetWHO_AM_I(&tmp);
    NRF_LOG_INFO("WHO_AM_I: 0X%02x", tmp);
    
    // CTRL_REG1 设置采样频率为10Hz，低功耗模式，开启x,y,z轴检测
    LIS3DH_SetODR(LIS3DH_ODR_10Hz);      // 设置速率,打开设备
    LIS3DH_SetMode(LIS3DH_LOW_POWER);    // 设置工作电源模式
    LIS3DH_SetAxis(LIS3DH_X_ENABLE|LIS3DH_Y_ENABLE|LIS3DH_Z_ENABLE);// 启用三轴

//    LIS3DH_SetBDU(MEMS_ENABLE);   //Block data update
    
    // CTRL_REG2 正常模式，数据从内部滤波器到输出寄存器，高通滤波器使能到中断1
    LIS3DH_HPFAOI1Enable(MEMS_ENABLE);          //滤波器应用与INT1
    LIS3DH_SetFilterDataSel(MEMS_ENABLE);       //滤波器应用与输出寄存器
//    LIS3DH_SetHPFMode(LIS3DH_HPM_AUTORESET_INT);
//    LIS3DH_SetHPFCutOFF(LIS3DH_HPFCF_1);        //设置高通滤波器截止频率
    LIS3DH_GetReference(&tmp);                  //第一次虚读reference寄存器，强制将高通滤波器的值加载到当前加速度值,从这时开始比较了
    
    // CTRL_REG4 设置能测量的重力加速度 FS = 2 g
    LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
    
    // INT1 引脚上的中断发生器1使能
//    LIS3DH_SetInt1Pin(LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE); 
//    LIS3DH_SetInt2Pin(LIS3DH_I2_INT1_ON_PIN_INT2_DISABLE);
//    
//    LIS3DH_Int1LatchEnable(MEMS_ENABLE);// 中断锁存
//    LIS3DH_SetInt1Threshold(10);        //设置中断阈值
//    LIS3DH_SetInt1Duration(0);          //中断持续时间设置为0，因为已经设置中断锁存，所以中断不会消失，直到手动清除中断
//    LIS3DH_SetIntMode(LIS3DH_INT_MODE_OR);  // 静止时进行运动检测
//    LIS3DH_SetIntConfiguration(LIS3DH_INT1_XHIE_ENABLE|LIS3DH_INT1_YHIE_ENABLE|LIS3DH_INT1_ZHIE_ENABLE);    //int1中断源配置
    
    // INT2
    LIS3DH_Int1LatchEnable(MEMS_ENABLE);    // 中断锁存 MEMS_DISABLE
    LIS3DH_SetInt1Threshold(16);            //设置中断阈值 16*16mg
    LIS3DH_SetInt1Duration(0);              //中断持续时间设置为20ms，因为已经设置中断锁存，所以中断不会消失，直到手动清除中断
    LIS3DH_SetIntMode(LIS3DH_INT_MODE_OR);  // 静止时进行运动检测
    LIS3DH_SetIntConfiguration(LIS3DH_INT1_XHIE_ENABLE|LIS3DH_INT1_YHIE_ENABLE|LIS3DH_INT1_ZHIE_ENABLE);    //int1中断源配置
    
    LIS3DH_SetInt2Pin(LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE);   //使能引脚2中断
    LIS3DH_ResetInt1Latch();    //清中断

    // 进入Power-down mode
//    LIS3DH_SetMode(LIS3DH_POWER_DOWN);
#endif

#if 0
    lis3dh_i2c_read(LIS3DH_WHO_AM_I, &tmp, 1);
    NRF_LOG_INFO("WHO_AM_I: 0X%02x", tmp);
    
    tmp = 0x3F;
    lis3dh_i2c_write(LIS3DH_CTRL_REG1, &tmp, 1);//CTRL_REG1  设置采样频率为25Hz，低功耗模式，开启x,y,z轴检测
    tmp = 0x09;
    lis3dh_i2c_write(LIS3DH_CTRL_REG2, &tmp, 1);//CTRL_REG2 正常模式，数据从内部滤波器到输出寄存器，高通滤波器使能到中断1
    tmp = 0x40;
    lis3dh_i2c_write(LIS3DH_CTRL_REG3, &tmp, 1);//CTRL_REG3 AOI中断1使能
    tmp = 0x00;
    lis3dh_i2c_write(LIS3DH_CTRL_REG4, &tmp, 1);//CTRL_REG4 分辨率+/-2g 每位~16mg
    tmp = 0x08;
    lis3dh_i2c_write(LIS3DH_CTRL_REG5, &tmp, 1);//CTRL_REG5 中断锁存
    tmp = 0x10;
    lis3dh_i2c_write(LIS3DH_INT1_THS, &tmp, 1);//INT1_TH  =  16*16mg  设置中断阈值
    tmp = 0x00;
    lis3dh_i2c_write(LIS3DH_INT1_DURATION, &tmp, 1);//INT_DURATION 中断持续时间设置为0，因为已经设置中断锁存，所以中断不会消失，直到手动清除中断
    
    lis3dh_i2c_read(0x26, &tmp, 1);//reference  第一次虚读reference寄存器，强制将高通滤波器的值加载到当前加速度值,从这时开始比较了
    tmp = 0x2A;
    lis3dh_i2c_write(LIS3DH_INT1_CFG, &tmp, 1);//INT1_CFG 使能中断，中断模式是中断事件的‘OR’组合，这里只使能了x,y,z轴的高事件中断
    
    AxesRaw_t data;
    response = LIS3DH_GetAccAxesRaw(&data);
    if(response==1)
    {
        NRF_LOG_INFO("X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
    }
#endif
}

#endif



#if 0
#define GNT_BUTTONS_NUMBER  1
#define LIS_INT1    17

#define LIS_INT1_LONG_TIME_MS   1000

// 使用定时器
APP_TIMER_DEF(gnt_button_timer_id);   //IO按键、输入唤醒用定时器


static gnt_event_callback_t   gnt_registered_callback = NULL; //应用板级事件回调注册函数
static void gnt_button_event_handler(uint8_t pin_no, uint8_t button_action);
static void gnt_button_timer_handler(void * p_context);


// IO唤醒输入配置信息列表（引脚、状态、上拉/下拉、按键回调处理）
// 根据配置添加，对应IODM_BUTTONS_NUMBER记得改变
static const app_button_cfg_t gnt_buttons[GNT_BUTTONS_NUMBER] =
{
    {LIS_INT1, false, GPIO_PIN_CNF_PULL_Pulldown, gnt_button_event_handler},
    
};


void gnt_gpio_config(uint32_t ticks_per_100ms, gnt_event_callback_t callback)
{
    uint32_t err_code = NRF_SUCCESS;
    
    gnt_registered_callback = callback;
    
    // 按键配置初始化（GPIOTE）
    err_code = app_button_init((app_button_cfg_t *)gnt_buttons, GNT_BUTTONS_NUMBER, APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);
    // 按键中断使能
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
    
    // 创建定时器，用于按键操作
    app_timer_create(&gnt_button_timer_id, APP_TIMER_MODE_SINGLE_SHOT, gnt_button_timer_handler);
}


// 按键定时器超时回调函数（用于按键长按操作）
static void gnt_button_timer_handler(void * p_context)
{
    gnt_button_event_handler(*(uint8_t *)p_context, BSP_BUTTON_ACTION_LONG_PUSH);
}

// 按键事件处理函数
// pin_no:引脚号  button_action：引脚动作
static void gnt_button_event_handler(uint8_t pin_no, uint8_t button_action)
{
//    uint32_t err_code;
    gnt_event_t event  = IODM_EVENT_NOTHING;
    
    switch(button_action)
    {
        case BSP_BUTTON_ACTION_PUSH:
        {
            if(pin_no == LIS_INT1)
            {
                NRF_LOG_INFO("LIS_INT1 push event");
                app_timer_start(gnt_button_timer_id, APP_TIMER_TICKS(LIS_INT1_LONG_TIME_MS), (void*)&pin_no);
            }
        }break;
        case BSP_BUTTON_ACTION_RELEASE:
        {
            if(pin_no == LIS_INT1)
            {
                app_timer_stop(gnt_button_timer_id);
            }
        }break;
        case BSP_BUTTON_ACTION_LONG_PUSH:
        {
            if(pin_no == LIS_INT1)
            {
                event = IODM_EVENT_LISINT1;
            }
        }break;
    }
    
    if ((event != IODM_EVENT_NOTHING) && (gnt_registered_callback != NULL))
    {
        gnt_registered_callback(event);
    }
    
}

#endif


#endif  // #if (GNT_LIS3DH_EN == 1)


/******************* (C) COPYRIGHT 2018 HTGD *****END OF FILE****/



