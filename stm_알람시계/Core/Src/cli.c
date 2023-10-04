//문제4번
#if 0
#include "main.h"
extern uint8_t data;
extern short int flag;
extern short int flag2;
extern UART_HandleTypeDef huart3;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)		//문제4번
{
	  if(huart->Instance == huart3.Instance)
	  {

		  if(data=='2'&&flag==0)
		  {
			  HAL_GPIO_WritePin(GPIOB,LD2_Pin, GPIO_PIN_SET);
			  flag=1;
		  }

		  else if(data=='2'&&flag==1)
		  {
		  	  HAL_GPIO_WritePin(GPIOB,LD2_Pin, GPIO_PIN_RESET);
		  	  flag=0;
		  }

		  else if(data=='3'&&flag2==0)
		  {
		  	  HAL_GPIO_WritePin(GPIOB,LD3_Pin, GPIO_PIN_SET);
		  	  flag2=1;
		  }

		  else if(data=='3'&&flag2==1)
		  {
		  	  HAL_GPIO_WritePin(GPIOB,LD3_Pin, GPIO_PIN_RESET);
		  	  flag2=0;
		  }

		  else if(data=='q')
		  {
			  HAL_GPIO_WritePin(GPIOB,LD2_Pin, GPIO_PIN_RESET);
		  	  HAL_GPIO_WritePin(GPIOB,LD3_Pin, GPIO_PIN_RESET);
		  	  flag=2;
			  flag2=2;
		  	  printf("Thank You~~\r\n");
		  }

		  HAL_UART_Receive_IT(&huart3, &data,1);
	  }
}
#endif
