/*
This file is part of VP-Digi.

VP-Digi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

VP-Digi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with VP-Digi.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "drivers/uart.h"
#include "drivers/systick.h"
#include "terminal.h"
#include "ax25.h"
#include "common.h"
#include <string.h>

#include "digipeater.h"

Uart Uart1, Uart2, UartUsb;

//uint8_t Uart_txKiss(uint8_t *buf, uint16_t len)
//{
//	if(len < 10) //frame is too small
//	{
//		return 1;
//	}
//
//	uint16_t framebegin = 0;
//	uint8_t framestatus = 0; //0 - frame not started, 1 - frame start found, 2 - in a frame, 3 - frame end found
//
//	for(uint16_t i = 0; i < len; i++)
//	{
//		if(*(buf + i) == 0xc0) //found KISS frame delimiter
//		{
//			if((i > 2) && (framestatus == 2)) //we are already in frame, this is the ending marker
//			{
//				framestatus = 3;
//				ax25.frameXmit[ax25.xmitIdx++] = 0xFF; //write frame separator
//				Digi_storeDeDupeFromXmitBuf(framebegin); //store duplicate protection hash
//		        if((FRAMEBUFLEN - ax25.xmitIdx) < (len - i + 2)) //there might be next frame in input buffer, but if there is no space for it, drop it
//		        	break;
//			}
//		}
//		else if((*(buf + i) == 0x00) && (*(buf + i - 1) == 0xC0) && ((framestatus == 0) || (framestatus == 3))) //found frame delimiter, modem number (0x00) and we are not in a frame yet or preceding frame has been processed
//		{
//			framestatus = 1; //copy next frame
//			framebegin = ax25.xmitIdx;
//		}
//		else if((framestatus == 1) || (framestatus == 2)) //we are in a frame
//		{
//			ax25.frameXmit[ax25.xmitIdx++] = *(buf + i); //copy data
//			framestatus = 2;
//		}
//	}
//
//	return 0;
//}

static void handleInterrupt(Uart *port)
{
	if(port->port->SR & USART_SR_RXNE) //byte received
	{
		port->port->SR &= ~USART_SR_RXNE;
		port->rxBuffer[port->rxBufferHead++] = port->port->DR; //store it
		port->rxBufferHead %= UART_BUFFER_SIZE;

//			if(port->port == USART1) //handle special functions and characters
//				term_handleSpecial(TERM_UART1);
//			else if(port->port == USART2)
//				term_handleSpecial(TERM_UART2);

		if(port->mode == MODE_KISS)
			port->kissTimer = ticks + (5000 / SYSTICK_INTERVAL); //set timeout to 5s in KISS mode
	}
	if(port->port->SR & USART_SR_IDLE) //line is idle, end of data reception
	{
		port->port->DR; //reset idle flag by dummy read
		if(port->rxBufferHead != 0)
		{
			if((port->rxBuffer[0] == 0xC0) && (port->rxBuffer[port->rxBufferHead - 1] == 0xC0)) //data starts with 0xc0 and ends with 0xc0 - this is a KISS frame
			{
				port->rxType = DATA_KISS;
				port->kissTimer = 0;
			}
			else if(((port->rxBuffer[port->rxBufferHead - 1] == '\r') || (port->rxBuffer[port->rxBufferHead - 1] == '\n'))) //data ends with \r or \n, process as data
			{
				port->rxType = DATA_TERM;
				port->kissTimer = 0;
			}
		}
	}
	if(port->port->SR & USART_SR_TXE) //TX buffer empty
	{
		if((port->txBufferHead != port->txBufferTail) || port->txBufferFull) //if there is anything to transmit
		{
			port->port->DR = port->txBuffer[port->txBufferTail++]; //push it to the refister
			port->txBufferTail %= UART_BUFFER_SIZE;
			port->txBufferFull = 0;
		}
		else //nothing more to be transmitted
		{
			port->port->CR1 &= ~USART_CR1_TXEIE;
		}
	}

	if((port->kissTimer > 0) && (ticks >= port->kissTimer)) //KISS timer timeout
	{
		port->kissTimer = 0;
		port->rxBufferHead = 0;
		memset(port->rxBuffer, 0, sizeof(port->rxBuffer));
	}
}

void USART1_IRQHandler(void) __attribute__ ((interrupt));
void USART1_IRQHandler(void)
{
	handleInterrupt(&Uart1);
}

void USART2_IRQHandler(void) __attribute__ ((interrupt));
void USART2_IRQHandler(void)
{
	handleInterrupt(&Uart2);
}


void UartSendByte(Uart *port, uint8_t data)
{
	if(!port->enabled)
		return;

	if(port->isUsb)
	{
		CDC_Transmit_FS(&data, 1);
	}
	else
	{
		while(port->txBufferFull)
			;
		port->txBuffer[port->txBufferHead++] = data;
		port->txBufferHead %= UART_BUFFER_SIZE;
		if(port->txBufferHead == port->txBufferTail)
			port->txBufferFull = 1;
		if(0 == (port->port->CR1 & USART_CR1_TXEIE))
			port->port->CR1 |= USART_CR1_TXEIE;
	}
}


void UartSendString(Uart *port, void *data, uint16_t len)
{
	if(0 == len)
		len = strlen((char*)data);

	for(uint16_t i = 0; i < len; i++)
	{
		UartSendByte(port, ((uint8_t*)data)[i]);
	}
}


static unsigned int findHighestPosition(unsigned int n)
{
    unsigned int i = 1;
    while((i * 10) <= n)
        i *= 10;

    return i;
}

void UartSendNumber(Uart *port, int32_t n)
{
	if(n < 0)
		UartSendByte(port, '-');
	n = abs(n);
    unsigned int position = findHighestPosition(n);
    while(position)
    {
        unsigned int number = n / position;
        UartSendByte(port, (number + 48));
        n -= (number * position);
        position /= 10;
    }
}

void UartInit(Uart *port, USART_TypeDef *uart, uint32_t baud)
{
	port->port = uart;
	port->baudrate = baud;
	port->rxType = DATA_NOTHING;
	port->rxBufferHead = 0;
	port->txBufferHead = 0;
	port->txBufferTail = 0;
	port->txBufferFull = 0;
	port->mode = MODE_KISS;
	port->enabled = 0;
	port->kissTimer = 0;
	memset(port->rxBuffer, 0, sizeof(port->rxBuffer));
	memset(port->txBuffer, 0, sizeof(port->txBuffer));
}


void UartConfig(Uart *port, uint8_t state)
{
	if(port->port == USART1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		GPIOA->CRH |= GPIO_CRH_MODE9_1;
		GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
		GPIOA->CRH |= GPIO_CRH_CNF9_1;
		GPIOA->CRH |= GPIO_CRH_CNF10_0;
		GPIOA->CRH &= ~GPIO_CRH_CNF10_1;

		USART1->BRR = (SystemCoreClock / (port->baudrate));

		if(state)
			USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
		else
			USART1->CR1 &= (~USART_CR1_RXNEIE) & (~USART_CR1_TE) & (~USART_CR1_RE) &  (~USART_CR1_UE) & (~USART_CR1_IDLEIE);

		NVIC_SetPriority(USART1_IRQn, 2);
		if(state)
			NVIC_EnableIRQ(USART1_IRQn);
		else
			NVIC_DisableIRQ(USART1_IRQn);

		port->enabled = state > 0;
		port->isUsb = 0;
	}
	else if(port->port == USART2)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		GPIOA->CRL |= GPIO_CRL_MODE2_1;
		GPIOA->CRL &= ~GPIO_CRL_CNF2_0;
		GPIOA->CRL |= GPIO_CRL_CNF2_1;
		GPIOA->CRL |= GPIO_CRL_CNF3_0;
		GPIOA->CRL &= ~GPIO_CRL_CNF3_1;

		USART2->BRR = (SystemCoreClock / (port->baudrate * 2));
		if(state)
			USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE;
		else
			USART2->CR1 &= (~USART_CR1_RXNEIE) & (~USART_CR1_TE) & (~USART_CR1_RE) &  (~USART_CR1_UE) & (~USART_CR1_IDLEIE);

		NVIC_SetPriority(USART2_IRQn, 2);
		if(state)
			NVIC_EnableIRQ(USART2_IRQn);
		else
			NVIC_DisableIRQ(USART2_IRQn);

		port->enabled = state > 0;
		port->isUsb = 0;
	}
	else
	{
		port->isUsb = 1;
		port->enabled = state > 0;
	}

}


void UartClearRx(Uart *port)
{
	port->rxBufferHead = 0;
	port->rxType = DATA_NOTHING;
}

void UartHandleKissTimeout(Uart *port)
{
	if((port->kissTimer > 0) && (ticks >= port->kissTimer)) //KISS timer timeout
	{
		port->kissTimer = 0;
		port->rxBufferHead = 0;
		memset(port->rxBuffer, 0, sizeof(port->rxBuffer));
	}
}
