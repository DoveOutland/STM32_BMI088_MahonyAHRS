#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H
#include "struct_typedef.h"

#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
  uint8_t status;
  fp32 mag[3];
} ist8310_real_data_t;

/**
  * @brief          初始化IST8310
  * @retval         error value
  */ 
extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data);
extern void ist8310_read_mag(fp32 mag[3]);

#endif
