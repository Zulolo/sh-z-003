#include "SIM7600.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  Sim80x_RxCallBack();
}

void  Sim80x_UserInit(void)
{
  //GPRS_ConnectToNetwork("mcinet","","",false);
  //GPRS_HttpGet("www.google.com");  
}
