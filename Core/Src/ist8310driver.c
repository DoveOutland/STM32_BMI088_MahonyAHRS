#include "ist8310driver.h"
#include "ist8310driver_middleware.h"

#define MAG_SEN 0.3f //raw int16 data change to uT unit. 原始整型数据变成 单位ut
#define IST8310_WHO_AM_I 0x00       //ist8310 "who am I " 
#define IST8310_WHO_AM_I_VALUE 0x10 //device ID
#define IST8310_WRITE_REG_NUM 4 

//the first column:the registers of IST8310. 第一列:IST8310的寄存器
//the second column: the value to be writed to the registers.第二列:需要写入的寄存器值
//the third column: return error value.第三列:返回的错误码
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] ={
        {0x0B, 0x08, 0x01},     //enalbe interrupt  and low pin polarity.开启中断，并且设置低电平
        {0x41, 0x09, 0x02},     //average 2 times.平均采样两次
        {0x42, 0xC0, 0x03},     //must be 0xC0. 必须是0xC0
        {0x0A, 0x0B, 0x04}};    //200Hz output rate.200Hz输出频率

/**
  * @brief          初始化IST8310
  * @param[in]      none
  * @retval         error value
  */
uint8_t ist8310_init(void)
{
    uint8_t res = 0;
    uint8_t writeNum = 0;

    ist8310_RST_L();
    ist8310_delay_ms(50);
    ist8310_RST_H();
    ist8310_delay_ms(50);

    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }

    //set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_us(150);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        ist8310_delay_us(150);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    return IST8310_NO_ERROR;
}

/**
  * @brief          如果已经通过I2C的DMA方式读取到了从STAT1到DATAZL的数据，可以使用这个函数进行处理
  * @param[in]      status_buf:数据指针,从STAT1(0x02) 寄存器到 DATAZL(0x08)寄存器 
  * @param[out]     ist8310_real_data:ist8310的数据结构
  * @retval         none
  */
void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{

    if (status_buf[0] & 0x01)
    {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

/**
  * @brief          通过读取磁场数据
  * @param[out]     磁场数组
  * @retval         none
  */
void ist8310_read_mag(fp32 mag[3])
{
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    //read the "DATAXL" register (0x03)
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}
