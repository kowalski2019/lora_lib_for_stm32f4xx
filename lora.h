/**
  ******************************************************************************
  * @file    	lora.h
  * @author		Claude Stephane M. Kouame
  * @version 	V1.0
  * @date		Oct 2, 2021
  * @brief  	Module for using the RFM95 or SX1276.
  ******************************************************************************
*/

#ifndef LORA_LORA_RFM96_SX1276_LORA_H_
#define LORA_LORA_RFM96_SX1276_LORA_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>


#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1
#define DEFAULT_RECEIVE_SIZE	0

/**
 * Structure defining a handle describing an RFM96(W) transceiver.
 */
typedef struct {

	/**
	 * The handle to the SPI bus for the device.
	 */
	SPI_HandleTypeDef *spi_handle;

	/**
	 * The port of the NSS pin.
	 */
	GPIO_TypeDef *nss_port;

	/**
	 * The NSS pin.
	 */
	uint16_t nss_pin;

	/**
	 * The port of the RST pin.
	 */
	GPIO_TypeDef *nrst_port;

	/**
	 * The RST pin.
	 */
	uint16_t nrst_pin;

	/**
	 * The port of the IRQ / DIO0 pin.
	 */
	GPIO_TypeDef *irq_port;

	/**
	 * The IRQ / DIO0 pin.
	 */
	uint16_t irq_pin;

	/**
	 * frequency
	 */
	long frequency;

	/**
	 * packet index
	 */

	uint64_t packet_index;

	/**
	 * implicit header
	 */
	uint8_t implicit_header_mode;

	/**
	 * on_receive pointer
	 */
	void (*_on_receive)(int);

	/**
	 * on_tx_done
	 */
	void (*_on_tx_done)(void);

	/* LoRaWAN component*/

	/**
	 * The device address for the LoraWAN
	 */
	uint8_t device_address[4];

	/**
	 * The network session key for ABP activation with the LoraWAN
	 */
	uint8_t network_session_key[16];

	/**
	 * The application session key for ABP activation with the LoraWAN
	 */
	uint8_t application_session_key[16];

} lora_handle_t;


/* Public functions (prototypes) */
uint8_t lora_init(lora_handle_t * lora_ha);
uint8_t lora_begin(long frequency);
uint8_t lora_begin_packet(int implicit_header);
uint8_t lora_end_packet(bool async);
bool lora_is_transmitting(void);
int lora_parse_packet(int size);
int lora_packet_rssi(void);
float lora_packet_snr(void);
long lora_packet_frequency_error(void);
int lora_rssi(void);
uint8_t lora_write_char(char c);
uint8_t lora_write_string(char *s);
size_t lora_write(uint8_t byte);
size_t lora_write1(const uint8_t *buffer, size_t size);
int lora_available(void);
uint8_t lora_read(void);
int lora_peek(void);
void lora_on_receive(void(*callback)(int));
void lora_on_tx_done(void(*callback)());
void lora_flush(void);
void lora_receive(int size);
void lora_idle(void);
void lora_sleep(void);
void lora_set_tx_power(int level, int output_pin);
void lora_set_frequency(long frequency);
int lora_get_spreading_factor(void);
void lora_set_spreading_factor(int sf);
long lora_get_signal_bandwidth(void);
void lora_set_signal_bandwidth(long sbw);
void lora_set_ldo_flag(void);
void lora_set_coding_rate4(int denominator);
void lora_set_preamble_length(long length);
void lora_set_sync_word(int sw);
void lora_enable_crc(void);
void lora_disable_crc(void);
void lora_enable_invert_IQ(void);
void lora_disable_invert_IQ(void);
void lora_set_OCP(uint8_t mA);
void lora_set_Gain(uint8_t gain);
uint8_t lora_lora_random(void);
void lora_dump_registers(char *buffer);
void lora_explicit_header_mode(void);
void lora_implicit_header_mode(void);
void lora_handle_dio0_rise(void);
uint8_t lora_read_register(uint8_t address);
void lora_write_register(uint8_t address, uint8_t value);
uint8_t lora_single_transfer(uint8_t address, uint8_t value, uint8_t mode);
void lora_set_spi(SPI_HandleTypeDef hspi);
void lora_set_pins(uint16_t nss,uint16_t rst, uint16_t dio0 );

extern lora_handle_t lora_handle;

#endif /* LORA_LORA_RFM96_SX1276_LORA_H_ */
