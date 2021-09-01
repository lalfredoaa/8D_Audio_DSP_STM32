
#include "stm32f4xx.h"
#include "arm_math.h"
#include "coefficients.h"

#define baudrate 1600000
#define BUFFERSIZE 256
#define NUMTAPS 128

#define angle 130
#define coefficient 128*angle/5
/*
 * 0 angle, &left[0]
 * 5 angle, &left[128]
 * 10 angle, &left[256]
 * 15 angle, &left[384]
 * 20 angle, &left[512]
 * 25 angle, &left[640]
 *
 */


uint8_t bufferIn[BUFFERSIZE];

uint8_t bufferOutLeft[BUFFERSIZE];
uint8_t bufferOutRight[BUFFERSIZE];
uint8_t bufferOut[2*BUFFERSIZE];

q15_t q15In[BUFFERSIZE/2];

q15_t q15OutLeft[BUFFERSIZE/2];
q15_t q15OutRight[BUFFERSIZE/2];

float32_t preFiltered[BUFFERSIZE/2];

float32_t FilteredLeft[BUFFERSIZE/2];
float32_t FilteredRight[BUFFERSIZE/2];

float32_t leftStates[BUFFERSIZE+NUMTAPS-1];
float32_t rightStates[BUFFERSIZE+NUMTAPS-1];
arm_fir_instance_f32 LeftFIR;
arm_fir_instance_f32 RightFIR;
uint32_t buffer_size = BUFFERSIZE;
uint16_t num_taps = NUMTAPS;


int sending = 0;

/**
   * @brief  Initialization function for the floating-point FIR filter.
   * @param[in,out] *S points to an instance of the floating-point FIR filter structure.
   * @param[in] 	numTaps  Number of filter coefficients in the filter.
   * @param[in] 	*pCoeffs points to the filter coefficients.
   * @param[in] 	*pState points to the state buffer.
   * @param[in] 	blockSize number of samples that are processed at a time.
   * @return    	none.
   *
  void arm_fir_init_f32(
  arm_fir_instance_f32 * S,
  uint16_t numTaps,
  float32_t * pCoeffs,
  float32_t * pState,
  uint32_t blockSize);**/


int main(void)
{
	arm_fir_init_f32((float32_t *)&LeftFIR, (uint16_t)num_taps, (float32_t *)&left[coefficient], (float32_t *)&leftStates, (uint32_t)buffer_size);
	arm_fir_init_f32((float32_t *)&RightFIR, (uint16_t)num_taps, (float32_t *)&right[coefficient], (float32_t *)&rightStates, (uint32_t)buffer_size);

	RCC_Configuration();
	NVIC_Configuration();
	GPIO_init();
	UART_init();

	DMA_Rx_Configuration();
	DMA_Tx_Configuration();

	/*while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
	USART_SendData(USART1, '*');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
	USART_SendData(USART1, '*');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
	USART_SendData(USART1, '*');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
	USART_SendData(USART1, '\n');
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
	USART_SendData(USART1, '\r');*/



	for(;;){
		/*while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
		USART_SendData(USART1, 'A');*/
	}

}


void RCC_Configuration(void)
{
/* USART1 clock enable */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
/* GPIOA clock enable */
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
/* DMA1 clock enable */
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
}


void GPIO_init(void){
	GPIO_InitTypeDef 	GPIO_InitStruct;
	/**
	* Tell pins PA9 and PA10 which alternating function you will use
	* @important Make sure, these lines are before pins configuration!
	*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	// Initialize pins as alternating function
	//GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	  GPIO_Init(GPIOA, &GPIO_InitStruct);
	  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	  GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void UART_init(void){
	USART_InitTypeDef USART_InitStruct;
	/**
	 * Set Baudrate to value you pass to function
	 * Disable Hardware Flow control
	 * Set Mode To TX and RX, so USART will work in full-duplex mode
	 * Disable parity bit
	 * Set 1 stop bit
	 * Set Data bits to 8
	 *
	 * Initialize USART1
	 * Activate USART1
	 */
	  //USART_OverSampling8Cmd(USART1, ENABLE);

	USART_InitStruct.USART_BaudRate = baudrate;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStruct);
	//USART_Cmd(USART1, ENABLE);

	/**
	 * Enable RX interrupt
	 */
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	  USART_Cmd(USART1, ENABLE);

}


void DMA_Rx_Configuration(void)
{
DMA_InitTypeDef DMA_InitStructure;
DMA_DeInit(DMA2_Stream5);
DMA_InitStructure.DMA_Channel = DMA_Channel_4;
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Receive

DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)bufferIn;
//DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(bufferIn);
DMA_InitStructure.DMA_BufferSize = BUFFERSIZE;


DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
DMA_InitStructure.DMA_Priority = DMA_Priority_High;
DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
DMA_Init(DMA2_Stream5, &DMA_InitStructure);
/* Enable the USART Rx DMA request */
USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
/* Enable DMA Stream Half Transfer and Transfer Complete interrupt */
DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
//DMA_ITConfig(DMA2_Stream5, DMA_IT_HT, ENABLE);
/* Enable the DMA RX Stream */
DMA_Cmd(DMA2_Stream5, ENABLE);
}


void DMA_Tx_Configuration(void)
{
DMA_InitTypeDef DMA_InitStructure;
DMA_DeInit(DMA2_Stream7);
DMA_InitStructure.DMA_Channel = DMA_Channel_4;
DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // Receive

DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)bufferOut;
//DMA_InitStructure.DMA_BufferSize = (uint16_t)sizeof(bufferIn);
DMA_InitStructure.DMA_BufferSize = 2*BUFFERSIZE;


DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
DMA_InitStructure.DMA_Priority = DMA_Priority_High;
DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
DMA_Init(DMA2_Stream7, &DMA_InitStructure);
/* Enable the USART Rx DMA request */
USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
/* Enable DMA Stream Half Transfer and Transfer Complete interrupt */
DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
//DMA_ITConfig(DMA2_Stream5, DMA_IT_HT, ENABLE);
/* Enable the DMA RX Stream */
//DMA_Cmd(DMA2_Stream7, ENABLE);
}


void DMA2_Stream5_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
	{
	/* Clear DMA Stream Transfer Complete interrupt pending bit */
	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);

		//group pairs in q15 format
		for(int i=0; i<BUFFERSIZE/2;i++){
			q15In[i]=(bufferIn[2*i]<<8)+bufferIn[2*i+1];
			//q15In[i]=(bufferIn[2*i+1]<<8)+bufferIn[2*i];
		}

		//q15 to float
		arm_q15_to_float ((q15_t*)&q15In, (float32_t *)&preFiltered, buffer_size/2);

		//filter
		arm_fir_f32(&LeftFIR,  (float32_t *)&preFiltered, (float32_t *)&FilteredLeft , buffer_size/2);
		arm_fir_f32(&RightFIR,  (float32_t *)&preFiltered, (float32_t *)&FilteredRight , buffer_size/2);

		//float to q15
		arm_float_to_q15 ((float32_t *)&FilteredLeft, (q15_t*)&q15OutLeft, buffer_size/2);
		arm_float_to_q15 ((float32_t *)&FilteredRight, (q15_t*)&q15OutRight, buffer_size/2);

		//separate pairs from q15 format
		/*for(int j=0; j<BUFFERSIZE/2;j++){
			bufferOut[2*j]=(q15OutLeft[j]>>8);
			bufferOut[2*j+1]=q15OutLeft[j]&0xFF;
		}*/
		/*for(int j=0; j<BUFFERSIZE/2;j++){
			bufferOut[2*j]=(q15OutRight[j]>>8);
			bufferOut[2*j+1]=q15OutRight[j]&0xFF;
		}*/



		for(int x=0; x<BUFFERSIZE/2; x++){
			bufferOut[2*x]=(q15OutLeft[x]>>8);
			bufferOut[2*x+1]=(q15OutLeft[x]&0xFF);
		}
		for(int y=0; y<BUFFERSIZE/2; y++){
			bufferOut[BUFFERSIZE+2*y]=(q15OutRight[y]>>8);
			bufferOut[BUFFERSIZE+2*y+1]=(q15OutRight[y]&0xFF);
		}

		//sending = 0;


		/*for(int i=0; i<BUFFERSIZE;i++){
				bufferOut[i]=bufferIn[i];
			}*/


		/*while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
		USART_SendData(USART1, '#');
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
		USART_SendData(USART1, 's');
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
		USART_SendData(USART1, 't');
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty*/

		DMA_Cmd(DMA2_Stream7, ENABLE);
	}

}


void DMA2_Stream7_IRQHandler(void)
{
	DMA_Cmd(DMA2_Stream7, DISABLE);

	/* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))
	{
		DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
		//while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
		//USART_SendData(USART1, 10);
		//while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty

		/*if(sending == 0){
			for(int y=0; y<BUFFERSIZE/2; y++){
				bufferOut[2*y]=(q15OutRight[y]>>8);
				bufferOut[2*y+1]=(q15OutRight[y]&0xFF);
			}
			sending=1;
			DMA_Cmd(DMA2_Stream7, ENABLE);
		}
		if(sending==1){
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
			USART_SendData(USART1, 10);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET); // Wait for Empty
		}*/
	}

}


void NVIC_Configuration(void)
{
NVIC_InitTypeDef NVIC_InitStructure;
/* Configure the Priority Group to 2 bits */
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
/* Enable the USART1 RX DMA Interrupt */
NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream5_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

/* Configure the Priority Group to 2 bits */
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
/* Enable the USART1 RX DMA Interrupt */
NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);


}


