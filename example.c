#include "lora.h"

void example_RCC_config(void)
{
	__HAL_RCC_SPI4_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

void example_GPIO_Init(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct = {0};


	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(RFM95_NSS_GPIO_Port, RFM95_NSS_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin : PF10 */
	  GPIO_InitStruct.Pin = RFM95_DIO0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(RFM95_DIO0_GPIO_Port, &GPIO_InitStruct);

	  /*SPI_PINS*/
	  /*Configure GPIO pin : PE11 */
	  GPIO_InitStruct.Pin = RFM95_NSS_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(RFM95_NSS_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = (RFM95_SPI_SCK_Pin | RFM95_SPI_MISO_Pin | RFM95_SPI_MOSI_Pin);
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
	  HAL_GPIO_Init(RFM95_SPI_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB10 for reset , it will be not use*/
	  GPIO_InitStruct.Pin = RFM95_NRST_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(RFM95_NRST_GPIO_Port, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


}

void example_SPI_Init(void)
{

	hspi4.Instance = SPI4;
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 10;
	HAL_SPI_Init(&hspi4);
}

void example_init(void)
{
	example_RCC_config();
	example_GPIO_Init();
	example_SPI_Init();

	lora_handle_t lora;
	lora.spi_handle = &hspi4;
	lora.nss_port = RFM95_NSS_GPIO_Port;
	lora.nss_pin = RFM95_NSS_Pin;
	lora.nrst_port = RFM95_NRST_GPIO_Port;
	lora.nrst_pin = RFM95_NRST_Pin;
	lora.irq_port = RFM95_DIO0_GPIO_Port;
	lora.irq_pin = RFM95_DIO0_Pin;
	lora_init(&lora);

}

void example_sender(void)
{
	uint8_t res;
	char buf1[32];
	res = lora_begin(868E6);
	uint32_t counter = 0;
	if(res == 1) {
		printf("RFM96 init failed\n");
	}
	else {
		printf("RFM96 init success\n");
		while(1)
		{
			lora_begin_packet(false);
			sprintf(buf1,"Packet Nr %ld from STM32x:", counter);
			lora_write_string(buf1);
			lora_write_char('\n');
			lora_write_string("Hello World\n");
			lora_end_packet(false);
			counter += 1;
			HAL_Delay(3000);
		}
	}
}

void example_receiver(void)
{
	uint8_t res;
	res = lora_begin(868E6);
	if(res == 1) {
		printf("RFM96 init failed\n");
	}
	else
	{
		printf("RFM96 init success\n");

		lora_on_receive(on_packet_receiving);

		//put the lora radio in receive mode
		lora_receive(DEFAULT_RECEIVE_SIZE);

		while(1)
		{
			;
		}
	}
}

void on_packet_receiving(int size)
{
	char buff[2];
	if(size > 0) {
		printf("Packet is there :)\n");
	}
	for(int i = 0; i < size; i++)
	{
		sprintf(buff, "%c", lora_read());
		printf(buff);
	}
}
