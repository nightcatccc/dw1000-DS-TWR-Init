#include "UART.h"

void UART_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_InitTypeDef USART1GPIO;
	USART1GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
	USART1GPIO.GPIO_Pin = GPIO_Pin_9;
	USART1GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &USART1GPIO);

	USART1GPIO.GPIO_Mode = GPIO_Mode_IPU;
	USART1GPIO.GPIO_Pin = GPIO_Pin_10;
	USART1GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &USART1GPIO);

	USART_InitTypeDef USART1Init;
	USART1Init.USART_BaudRate = 115200;
	USART1Init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART1Init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART1Init.USART_Parity = USART_Parity_No;
	USART1Init.USART_StopBits = USART_StopBits_1;
	USART1Init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART1Init);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef NVICInitStructure;
	NVICInitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVICInitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVICInitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVICInitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVICInitStructure);

	USART_Cmd(USART1, ENABLE);

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//	USART1GPIO.GPIO_Mode = GPIO_Mode_AF_PP;
//	USART1GPIO.GPIO_Pin = GPIO_Pin_2;
//	USART1GPIO.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &USART1GPIO);

//	USART1GPIO.GPIO_Mode = GPIO_Mode_IPU;
//	USART1GPIO.GPIO_Pin = GPIO_Pin_3;
//	USART1GPIO.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &USART1GPIO);

//	USART_InitTypeDef USART2Init;
//	USART2Init.USART_BaudRate = 115200;
//	USART2Init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART2Init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//	USART2Init.USART_Parity = USART_Parity_No;
//	USART2Init.USART_StopBits = USART_StopBits_1;
//	USART2Init.USART_WordLength = USART_WordLength_8b;
//	USART_Init(USART2, &USART2Init);
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

//	NVICInitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVICInitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVICInitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVICInitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVICInitStructure);

//	USART_Cmd(USART2, ENABLE);
}

void UART_SendData(USART_TypeDef *USARTx, uint8_t ch)
{
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
	USART_SendData(USARTx, ch);
}

void UART_SendStr(USART_TypeDef *USARTx, uint8_t *str)
{
	while (*str != 0)
	{
		UART_SendData(USARTx, *str);
		if (*str == '\n')
			break;
		str++;
	}
}

void USART1_IRQHandler(void)
{
	uint8_t a;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		a = USART_ReceiveData(USART1);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

