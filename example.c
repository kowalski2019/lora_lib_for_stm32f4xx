#include "lora.h"

void example_init(void)
{

	lora_handle_t lora;
	lora.spi_handle = &hspi_handle_struct;
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
