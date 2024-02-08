#include "ist8310driver_middleWare.h"
#include "main.h"
#include "bsp_delay.h"
extern I2C_HandleTypeDef hi2c3;


/**
  * @brief          读取IST8310的一个字节通过I2C
  * @param[in]      寄存器地址
  * @retval         寄存器值
  */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,&res,1,10);
    return res;
}

/**
  * @brief          通过I2C写入一个字节到IST8310的寄存器中
  * @param[in]      寄存器地址
  * @param[in]      写入值
  */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,&data,1,10);
}

/**
  * @brief          读取IST8310的多个字节通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
  */
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,buf,len,10);
}

/**
  * @brief          写入多个字节到IST8310的寄存器通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
  */
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,data,len,10);
}

/**
  * @brief          延时x毫秒
  */
void ist8310_delay_ms(uint16_t ms)
{
    delay_ms(ms);
}

/**
  * @brief          延时x微秒
  */
void ist8310_delay_us(uint16_t us)
{
    delay_us(us);
}

/**
  * @brief          设置RSTN引脚为1
  */
void ist8310_RST_H(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
}

/**
  * @brief          设置RSTN引脚为0
  */
extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
}
