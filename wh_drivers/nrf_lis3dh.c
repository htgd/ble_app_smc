/**@file    nrf_lis3dh.c
* @brief   	����nRF��lis3DH����
* @details  
* @author  	wanghuan  any question please send mail to 371463817@qq.com
* @date    	2018-08-17
* @version 	V1.0
* @copyright	Copyright (c) 2016-2018  ���պ�ͨ�����Ƽ����޹�˾
**********************************************************************************
* @attention
* Ӳ��ƽ̨:nRF52832_QFAA \n
* SDK�汾��nRF5_SDK_15.0.0
* @par �޸���־:
* <table>
* <tr><th>Date        <th>Version  <th>Author    <th>Description
* <tr><td>2018/08/17  <td>1.0      <td>wanghuan  <td>������ʼ�汾
* </table>
*/

#include "nrf_lis3dh.h"
#if (GNT_LIS3DH_EN == 1)

void lis3dh_twi_enable(void);
void lis3dh_twi_disable(void);
void lis3dh_power_off(void);
void lis3dh_ResetInt1Latch(uint8_t err_try);


// TWI����ʹ�ã�������ٴ�������*****************************************************************************************************

/* Master Configuration */
#define MASTER_TWI_INST     0       //!< TWI interface used as a master accessing EEPROM memory.

#define LIS3DH_ADDR         0x19 					///< �ӻ���ַ
#define TWI_SLAVE_WRITE_MAX_BYTES           50      ///< �ӻ����ο�д����󳤶�

static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);	///< twi�����������

/**@brief   I2C������ʼ��
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

/**@brief IICд����
* @param[in] addr	 i2cѰַ��ַ
* @param[in] *pdata  Ҫд�������
* @param[in] size    Ҫд������ݵĳ���
* @return 	Status [MEMS_ERROR , MEMS_SUCCESS]
*/
static status_t lis3dh_i2c_write(uint8_t addr, uint8_t const * pdata, uint8_t size)
{
    uint8_t buffer[size+1]; // Addr + data
    
    memcpy(buffer, &addr, 1);   //����Ѱַ��ַ��1���ֽڣ�
    memcpy(buffer + 1, pdata, size);
    if(NRFX_SUCCESS == nrf_drv_twi_tx(&m_twi_master, LIS3DH_ADDR, buffer, size + 1, false))
    {
        return MEMS_SUCCESS;
    }
    else
        return MEMS_ERROR;
}

/**@brief I2C������
* @param[in] addr	 i2cѰַ��ַ
* @param[out] *pdata ����������
* @param[in] size    Ҫ���������ݵĳ���
* @return 	Status [MEMS_ERROR , MEMS_SUCCESS]
*/
static status_t lis3dh_i2c_read(uint8_t addr, uint8_t * pdata, uint8_t size)
{
    if(NRFX_SUCCESS == nrf_drv_twi_tx(&m_twi_master, LIS3DH_ADDR, &addr, 1, true))  //����Ѱַ��ַ
    {
        if(NRFX_SUCCESS == nrf_drv_twi_rx(&m_twi_master, LIS3DH_ADDR, pdata, size)) //i2c��ȡ
        {
            return MEMS_SUCCESS;
        }
        else
            return MEMS_ERROR;
    }
    else
        return MEMS_ERROR;
}

static uint8_t nrf_twi_enabled;         ///< i2c�����Ƿ�ʹ�� 0:�رգ�1��ʹ��
static uint8_t lis3dh_twi_registed;     ///< lis3dh��i2c���������Ƿ�ע��



/**@brief ����I2C����
*/
void lis3dh_twi_enable(void)
{
    if(nrf_twi_enabled == 0)
        twi_master_init();
    nrf_twi_enabled = 1;
//    nrf_drv_twi_enable(&m_twi_master);
}

/**@brief �ر�I2C����
*/
void lis3dh_twi_disable(void)
{
    if(nrf_twi_enabled == 1)
        nrf_drv_twi_uninit(&m_twi_master);
    nrf_twi_enabled = 0;
//    nrf_drv_twi_disable(&m_twi_master);
}

/**@brief lis3dh�������ģʽ
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

/**@brief lis3dh���ж�����
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
/**@brief lis3dh��ȡ����ADC�¶�
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

/**@brief lis3dh����ע����ڲ��¶ȴ���ʹ��
*/
void lis3dh_reg_temp(void)
{
    uint8_t tmp;
    lis3dh_twi_enable();
    
    lis3dh_regist_init(lis3dh_i2c_write, lis3dh_i2c_read);  //ע��I2C��д����
    
    LIS3DH_GetWHO_AM_I(&tmp);
    NRF_LOG_INFO("WHO_AM_I: 0X%02x", tmp);
    
    LIS3DH_SetMode(LIS3DH_LOW_POWER);   // ���ù�����Դģʽ
    LIS3DH_SetODR(LIS3DH_ODR_1Hz);      // ��������,���豸
    LIS3DH_SetTemperature(MEMS_ENABLE); //ʹ���ڲ��¶ȴ���
    LIS3DH_SetADCAux(MEMS_ENABLE);      //ʹ�ܸ���ADC
    LIS3DH_SetBDU(MEMS_ENABLE);         //Block data update
    
    lis3dh_twi_disable();
}
#endif

/**@brief lis3dh��ʼ�� \n
* lis3dh�˶�����ж�����
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
    
    // CTRL_REG1 ���ò���Ƶ��Ϊ10Hz���͹���ģʽ������x,y,z����
    LIS3DH_SetODR(LIS3DH_ODR_10Hz);      // ��������,���豸
    LIS3DH_SetMode(LIS3DH_LOW_POWER);    // ���ù�����Դģʽ
    LIS3DH_SetAxis(LIS3DH_X_ENABLE|LIS3DH_Y_ENABLE|LIS3DH_Z_ENABLE);// ��������
    
    // CTRL_REG2 ����ģʽ�����ݴ��ڲ��˲���������Ĵ�������ͨ�˲���ʹ�ܵ��ж�1
    LIS3DH_HPFAOI1Enable(MEMS_ENABLE);          //�˲���Ӧ����INT1
    LIS3DH_SetFilterDataSel(MEMS_ENABLE);       //�˲���Ӧ��������Ĵ���
//    LIS3DH_SetHPFMode(LIS3DH_HPM_AUTORESET_INT);
//    LIS3DH_SetHPFCutOFF(LIS3DH_HPFCF_1);        //���ø�ͨ�˲�����ֹƵ��
    LIS3DH_GetReference(&tmp);                  //��һ�����reference�Ĵ�����ǿ�ƽ���ͨ�˲�����ֵ���ص���ǰ���ٶ�ֵ,����ʱ��ʼ�Ƚ���
    
    // CTRL_REG4 �����ܲ������������ٶ� FS = 2 g
    LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
    
    // INT1_ON_PIN_INT2
//    LIS3DH_Int1LatchEnable(MEMS_ENABLE);    // �ж����� MEMS_DISABLE
    LIS3DH_Int1LatchEnable(MEMS_DISABLE);   
    LIS3DH_SetInt1Threshold(10);            //�����ж���ֵ 16*16mg
    LIS3DH_SetInt1Duration(1);              //�жϳ���ʱ������Ϊ100ms��(�����ж����棬�����жϲ�����ʧ��ֱ���ֶ�����ж�)
    LIS3DH_SetIntMode(LIS3DH_INT_MODE_OR);  // ��ֹʱ�����˶����
    LIS3DH_SetIntConfiguration(LIS3DH_INT1_XHIE_ENABLE|LIS3DH_INT1_YHIE_ENABLE|LIS3DH_INT1_ZHIE_ENABLE);    //int1�ж�Դ����
    
    LIS3DH_SetInt2Pin(LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE);   //ʹ������2�ж�
//    LIS3DH_SetInt2Pin(LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE);   //ʹ������1�ж�
//    LIS3DH_ResetInt1Latch();    //���ж�
    
    lis3dh_twi_disable();
}



#if 0
// lis3dh���ٶȴ�������ʼ��
void lis3dh_init(void)
{
    uint8_t tmp;
    
    twi_master_init();
    nrf_twi_enabled = 1;    //twi����ʹ��
    lis3dh_regist_init(lis3dh_i2c_write, lis3dh_i2c_read);
#if 1
    LIS3DH_GetWHO_AM_I(&tmp);
    NRF_LOG_INFO("WHO_AM_I: 0X%02x", tmp);
    
    // CTRL_REG1 ���ò���Ƶ��Ϊ10Hz���͹���ģʽ������x,y,z����
    LIS3DH_SetODR(LIS3DH_ODR_10Hz);      // ��������,���豸
    LIS3DH_SetMode(LIS3DH_LOW_POWER);    // ���ù�����Դģʽ
    LIS3DH_SetAxis(LIS3DH_X_ENABLE|LIS3DH_Y_ENABLE|LIS3DH_Z_ENABLE);// ��������

//    LIS3DH_SetBDU(MEMS_ENABLE);   //Block data update
    
    // CTRL_REG2 ����ģʽ�����ݴ��ڲ��˲���������Ĵ�������ͨ�˲���ʹ�ܵ��ж�1
    LIS3DH_HPFAOI1Enable(MEMS_ENABLE);          //�˲���Ӧ����INT1
    LIS3DH_SetFilterDataSel(MEMS_ENABLE);       //�˲���Ӧ��������Ĵ���
//    LIS3DH_SetHPFMode(LIS3DH_HPM_AUTORESET_INT);
//    LIS3DH_SetHPFCutOFF(LIS3DH_HPFCF_1);        //���ø�ͨ�˲�����ֹƵ��
    LIS3DH_GetReference(&tmp);                  //��һ�����reference�Ĵ�����ǿ�ƽ���ͨ�˲�����ֵ���ص���ǰ���ٶ�ֵ,����ʱ��ʼ�Ƚ���
    
    // CTRL_REG4 �����ܲ������������ٶ� FS = 2 g
    LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
    
    // INT1 �����ϵ��жϷ�����1ʹ��
//    LIS3DH_SetInt1Pin(LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE); 
//    LIS3DH_SetInt2Pin(LIS3DH_I2_INT1_ON_PIN_INT2_DISABLE);
//    
//    LIS3DH_Int1LatchEnable(MEMS_ENABLE);// �ж�����
//    LIS3DH_SetInt1Threshold(10);        //�����ж���ֵ
//    LIS3DH_SetInt1Duration(0);          //�жϳ���ʱ������Ϊ0����Ϊ�Ѿ������ж����棬�����жϲ�����ʧ��ֱ���ֶ�����ж�
//    LIS3DH_SetIntMode(LIS3DH_INT_MODE_OR);  // ��ֹʱ�����˶����
//    LIS3DH_SetIntConfiguration(LIS3DH_INT1_XHIE_ENABLE|LIS3DH_INT1_YHIE_ENABLE|LIS3DH_INT1_ZHIE_ENABLE);    //int1�ж�Դ����
    
    // INT2
    LIS3DH_Int1LatchEnable(MEMS_ENABLE);    // �ж����� MEMS_DISABLE
    LIS3DH_SetInt1Threshold(16);            //�����ж���ֵ 16*16mg
    LIS3DH_SetInt1Duration(0);              //�жϳ���ʱ������Ϊ20ms����Ϊ�Ѿ������ж����棬�����жϲ�����ʧ��ֱ���ֶ�����ж�
    LIS3DH_SetIntMode(LIS3DH_INT_MODE_OR);  // ��ֹʱ�����˶����
    LIS3DH_SetIntConfiguration(LIS3DH_INT1_XHIE_ENABLE|LIS3DH_INT1_YHIE_ENABLE|LIS3DH_INT1_ZHIE_ENABLE);    //int1�ж�Դ����
    
    LIS3DH_SetInt2Pin(LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE);   //ʹ������2�ж�
    LIS3DH_ResetInt1Latch();    //���ж�

    // ����Power-down mode
//    LIS3DH_SetMode(LIS3DH_POWER_DOWN);
#endif

#if 0
    lis3dh_i2c_read(LIS3DH_WHO_AM_I, &tmp, 1);
    NRF_LOG_INFO("WHO_AM_I: 0X%02x", tmp);
    
    tmp = 0x3F;
    lis3dh_i2c_write(LIS3DH_CTRL_REG1, &tmp, 1);//CTRL_REG1  ���ò���Ƶ��Ϊ25Hz���͹���ģʽ������x,y,z����
    tmp = 0x09;
    lis3dh_i2c_write(LIS3DH_CTRL_REG2, &tmp, 1);//CTRL_REG2 ����ģʽ�����ݴ��ڲ��˲���������Ĵ�������ͨ�˲���ʹ�ܵ��ж�1
    tmp = 0x40;
    lis3dh_i2c_write(LIS3DH_CTRL_REG3, &tmp, 1);//CTRL_REG3 AOI�ж�1ʹ��
    tmp = 0x00;
    lis3dh_i2c_write(LIS3DH_CTRL_REG4, &tmp, 1);//CTRL_REG4 �ֱ���+/-2g ÿλ~16mg
    tmp = 0x08;
    lis3dh_i2c_write(LIS3DH_CTRL_REG5, &tmp, 1);//CTRL_REG5 �ж�����
    tmp = 0x10;
    lis3dh_i2c_write(LIS3DH_INT1_THS, &tmp, 1);//INT1_TH  =  16*16mg  �����ж���ֵ
    tmp = 0x00;
    lis3dh_i2c_write(LIS3DH_INT1_DURATION, &tmp, 1);//INT_DURATION �жϳ���ʱ������Ϊ0����Ϊ�Ѿ������ж����棬�����жϲ�����ʧ��ֱ���ֶ�����ж�
    
    lis3dh_i2c_read(0x26, &tmp, 1);//reference  ��һ�����reference�Ĵ�����ǿ�ƽ���ͨ�˲�����ֵ���ص���ǰ���ٶ�ֵ,����ʱ��ʼ�Ƚ���
    tmp = 0x2A;
    lis3dh_i2c_write(LIS3DH_INT1_CFG, &tmp, 1);//INT1_CFG ʹ���жϣ��ж�ģʽ���ж��¼��ġ�OR����ϣ�����ֻʹ����x,y,z��ĸ��¼��ж�
    
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

// ʹ�ö�ʱ��
APP_TIMER_DEF(gnt_button_timer_id);   //IO���������뻽���ö�ʱ��


static gnt_event_callback_t   gnt_registered_callback = NULL; //Ӧ�ð弶�¼��ص�ע�ắ��
static void gnt_button_event_handler(uint8_t pin_no, uint8_t button_action);
static void gnt_button_timer_handler(void * p_context);


// IO��������������Ϣ�б����š�״̬������/�����������ص�����
// ����������ӣ���ӦIODM_BUTTONS_NUMBER�ǵøı�
static const app_button_cfg_t gnt_buttons[GNT_BUTTONS_NUMBER] =
{
    {LIS_INT1, false, GPIO_PIN_CNF_PULL_Pulldown, gnt_button_event_handler},
    
};


void gnt_gpio_config(uint32_t ticks_per_100ms, gnt_event_callback_t callback)
{
    uint32_t err_code = NRF_SUCCESS;
    
    gnt_registered_callback = callback;
    
    // �������ó�ʼ����GPIOTE��
    err_code = app_button_init((app_button_cfg_t *)gnt_buttons, GNT_BUTTONS_NUMBER, APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);
    // �����ж�ʹ��
    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
    
    // ������ʱ�������ڰ�������
    app_timer_create(&gnt_button_timer_id, APP_TIMER_MODE_SINGLE_SHOT, gnt_button_timer_handler);
}


// ������ʱ����ʱ�ص����������ڰ�������������
static void gnt_button_timer_handler(void * p_context)
{
    gnt_button_event_handler(*(uint8_t *)p_context, BSP_BUTTON_ACTION_LONG_PUSH);
}

// �����¼�������
// pin_no:���ź�  button_action�����Ŷ���
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



