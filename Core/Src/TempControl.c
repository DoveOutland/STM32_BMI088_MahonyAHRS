#include "main.h"
#include "pid.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "TempControl.h"
//测试使用 
//extern pid_type_def imu_temp_pid;
//extern const fp32 imu_temp_PID[3] ;
//extern fp32 gyro[3], accel[3], temp;
//extern fp32 mag[3];
//测试使用 

void imu_pwm_set(uint16_t pwm)
{
    TIM10->CCR1 = (pwm);
}
void imu_temp_control_task(void const* argument)
{
//    HAL_Delay(50);
//    BMI088_read(gyro, accel, &temp);
//    uint16_t tempPWM;
//    PID_calc(&imu_temp_pid, temp, 45.0f);
//    if(imu_temp_pid.out < 0.0f)
//    {
//        imu_temp_pid.out = 0.0f;
//    }
//    tempPWM = (uint16_t)imu_temp_pid.out;
//    imu_pwm_set(tempPWM);

}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if(GPIO_Pin == INT1_ACCEL_Pin)
//    {
//		
//    }
//    else if(GPIO_Pin == INT1_GRYO_Pin)
//    {

//    }
//	else if(GPIO_Pin == IST8310_DRDY_Pin)
//    {

//        
//    }
//}








