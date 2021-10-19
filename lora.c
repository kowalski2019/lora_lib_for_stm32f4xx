/**
  ******************************************************************************
  * @file    	lora.c
  * @author		Claude Stephane M. Kouame
  * @version 	V1.0
  * @date		Oct 2, 2021
  * @brief  	Module for using the RFM95 or SX1276.
  ******************************************************************************
*/

#include <lora/lora_rfm96_sx1276/lora.h>

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

#define LORA_READ 	0
#define LORA_WRITE	1

//#define WRITE_BIT(x, n, b)	((x) | ((b) << (n)))

lora_handle_t lora_handle;

uint8_t lora_init(lora_handle_t * lora_h)
{
	lora_handle = *lora_h;
	lora_handle._on_receive = NULL;
	lora_handle._on_tx_done = NULL;
	lora_handle.implicit_header_mode = false;
	lora_handle.frequency = 0;
	lora_handle.packet_index = 0;

	return 0;
}

uint8_t lora_begin(long frequency)
{
	// setup pins
	// init here the GPIO Pin for NSS and RESET

	// set NSS high
	HAL_GPIO_WritePin(lora_handle.nss_port, lora_handle.nss_pin, GPIO_PIN_SET);

	// perform reset
	HAL_GPIO_WritePin(lora_handle.nrst_port, lora_handle.nrst_pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(lora_handle.nrst_port, lora_handle.nrst_pin, GPIO_PIN_SET);
	HAL_Delay(10);

	// init here the SPI handle structure

	// check version
	uint8_t version = lora_read_register(REG_VERSION);
	if(version != 0x12 )
		return 1;

	// put in sleep mode
	lora_sleep();

	// set frequency
	lora_set_frequency(frequency);

	// set base addresses
	lora_write_register(REG_FIFO_TX_BASE_ADDR, 0);
	lora_write_register(REG_FIFO_RX_BASE_ADDR, 0);

	// set LNA bosst
	lora_write_register(REG_LNA, lora_read_register(REG_LNA) | 0x03);

	// set auto AGC
	lora_write_register(REG_MODEM_CONFIG_3, 0x04);

	// set output power to 17 dBm
	lora_set_tx_power(17, PA_OUTPUT_PA_BOOST_PIN);

	// put in standby mode
	lora_idle();

	return 0;
}

uint8_t lora_begin_packet(int implicit_header)
{
	if(lora_is_transmitting()) {
		return 1;
	}

	// put in standby mode
	lora_idle();

	if(implicit_header) {
		lora_implicit_header_mode();
	} else {
		lora_explicit_header_mode();
	}

	// reset FIFO address and payload length
	lora_write_register(REG_FIFO_ADDR_PTR, 0);
	lora_write_register(REG_PAYLOAD_LENGTH, 0);

	return 0;
}

uint8_t lora_end_packet(bool async)
{
	if((async) && (lora_handle._on_tx_done))
		lora_write_register(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE

	// put in TX mode
	lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

	if(!async)
	{
		while((lora_read_register(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
			HAL_Delay(1000);
		}
		// clear IRQ's
		lora_write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}

	return 0;
}

bool lora_is_transmitting(void)
{
	if((lora_read_register(REG_OP_MODE) & MODE_TX) == MODE_TX) {
		return true;
	}
	if(lora_read_register(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
		// clear IRQ's
		lora_write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}
	return false;
}

int lora_parse_packet(int size)
{

	int packet_length = 0;
	int irq_flags = lora_read_register(REG_IRQ_FLAGS);

	  if (size > 0) {
		  lora_implicit_header_mode();

	    lora_write_register(REG_PAYLOAD_LENGTH, size & 0xff);
	  } else {
		  lora_explicit_header_mode();
	  }

	  // clear IRQ's
	  lora_write_register(REG_IRQ_FLAGS, irq_flags);

	  if ((irq_flags & IRQ_RX_DONE_MASK) && (irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		  // received a packet
		  lora_handle.packet_index = 0;

	    // read packet length
	    if (lora_handle.implicit_header_mode) {
	    	packet_length = lora_read_register(REG_PAYLOAD_LENGTH);
	    } else {
	    	packet_length = lora_read_register(REG_RX_NB_BYTES);
	    }

	    // set FIFO address to current RX address
	    lora_write_register(REG_FIFO_ADDR_PTR, lora_read_register(REG_FIFO_RX_CURRENT_ADDR));

	    // put in standby mode
	    lora_idle();
	  } else if ( lora_read_register(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
	    // not currently in RX mode

	    // reset FIFO address
		  lora_write_register(REG_FIFO_ADDR_PTR, 0);

	    // put in single RX mode
		  lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	  }

	  return packet_length;
}

int lora_packet_rssi(void)
{
	return (lora_read_register(REG_PKT_RSSI_VALUE) - (lora_handle.frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));

}

float lora_packet_snr(void)
{
	return ((int8_t)lora_read_register(REG_PKT_SNR_VALUE)) * 0.25;
}

long lora_packet_frequency_error(void)
{
	int32_t freq_error = 0;
	freq_error = (uint32_t)(lora_read_register(REG_FREQ_ERROR_MSB) & 0b111);
	freq_error <<= 8L;
	freq_error += (uint32_t)(lora_read_register(REG_FREQ_ERROR_MID));
	freq_error <<= 8L;
	freq_error += (uint32_t)(lora_read_register(REG_FREQ_ERROR_LSB));

	if (lora_read_register(REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
		freq_error -= 524288; // B1000'0000'0000'0000'0000
		}

	const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
	const float fError = (((long)freq_error * (1L << 24)) / fXtal) * (lora_get_signal_bandwidth() / 500000.0f); // p. 37

	return (long)(fError);
}

int lora_rssi(void)
{
	return (lora_read_register(REG_RSSI_VALUE) - (lora_handle.frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}


uint8_t lora_write_string(char *s)
{
	int i = 0;
	while( (i < sizeof(s)) || (s[i] != '\0'))
	{
		lora_write_char(*(s + i));
		i += 1;
	}
	return 0;
}

uint8_t lora_write_char(char c)
{
	return lora_write((uint8_t)c);
}

size_t lora_write(uint8_t byte)
{
	return lora_write1(&byte, sizeof(byte));
}

size_t lora_write1(const uint8_t *buffer, size_t size)
{
	int current_length = lora_read_register(REG_PAYLOAD_LENGTH);

	// check size
	if ((current_length + size) > MAX_PKT_LENGTH) {
		size = MAX_PKT_LENGTH - current_length;
	}

	// write data
	for (size_t i = 0; i < size; i++) {
		lora_write_register(REG_FIFO, buffer[i]);
	}

	// update length
	lora_write_register(REG_PAYLOAD_LENGTH, current_length + size);

	return size;
}

int lora_available(void)
{
	return (lora_read_register(REG_RX_NB_BYTES) - lora_handle.packet_index);
}

uint8_t lora_read(void)
{
	if (!lora_available()) {
		return -1;
	}

	lora_handle.packet_index += 1;

	return lora_read_register(REG_FIFO);
}

int lora_peek(void)
{
	if(lora_available()) {
		return -1;
	}

	// store current FIFO address
	int current_address = lora_read_register(REG_FIFO_ADDR_PTR);

	 // read
	uint8_t b = lora_read_register(REG_FIFO);

	// restore FIFO address
	lora_write_register(REG_FIFO_ADDR_PTR, current_address);

	return b;
}


// on_receive for interrupt
void lora_on_receive(void(*callback)(int))
{
	lora_handle._on_receive = callback;
}
// on_tx_done for interrupt
void lora_on_tx_done(void(*callback)())
{
	lora_handle._on_tx_done = callback;
}

void lora_receive(int size)
{
	lora_write_register(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE
	if(size > 0) {
		lora_implicit_header_mode();
		lora_write_register(REG_PAYLOAD_LENGTH, size & 0xff);
	} else {
		lora_explicit_header_mode();
	}

	lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}


void lora_idle(void)
{
	lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void lora_sleep(void)
{
	lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}


void lora_set_tx_power(int level, int output_pin)
{
	if(PA_OUTPUT_RFO_PIN == output_pin)
	{
		// RFO
		if(level < 0) {
			level = 0;
		}else if (level > 14){
			level = 14;
		}
		lora_write_register(REG_PA_CONFIG, 0x70 | level);
	}
	else
	{
		// PA BOOST
		if(level > 17){
			if(level > 20) {
				level = 20;
			}

			// subtract 3 from level, so 18 - 20 maps to 15 - 17
			level -= 3;

			// High Power +29 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
			lora_write_register(REG_PA_DAC, 0x87);
			lora_set_OCP(140);
		}
		else
		{
			if(level < 2){
				level = 2;
			}
			// Default value PA_HF/LF or +17dBm
			lora_write_register(REG_PA_DAC, 0x84);
			lora_set_OCP(100);
		}
		lora_write_register(REG_PA_CONFIG, PA_BOOST | (level - 3));
	}
}

void lora_set_frequency(long frequency)
{
	lora_handle.frequency = frequency;

	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	lora_write_register(REG_FRF_MSB, (uint8_t)(frf >> 16));
	lora_write_register(REG_FRF_MID, (uint8_t)(frf >> 8));
	lora_write_register(REG_FRF_LSB, (uint8_t)(frf >> 0));
}


int lora_get_spreading_factor(void)
{
	return lora_read_register(REG_MODEM_CONFIG_2) >> 4;
}

void lora_set_spreading_factor(int sf)
{
	if (sf < 6) {
	    sf = 6;
	  } else if (sf > 12) {
	    sf = 12;
	  }

	  if (sf == 6) {
		  lora_write_register(REG_DETECTION_OPTIMIZE, 0xc5);
		  lora_write_register(REG_DETECTION_THRESHOLD, 0x0c);
	  } else {
		  lora_write_register(REG_DETECTION_OPTIMIZE, 0xc3);
		  lora_write_register(REG_DETECTION_THRESHOLD, 0x0a);
	  }

	  lora_write_register(REG_MODEM_CONFIG_2, (lora_read_register(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
	  lora_set_ldo_flag();
}

long lora_get_signal_bandwidth(void)
{
	uint8_t bw = (lora_read_register(REG_MODEM_CONFIG_1) >> 4);

	  switch (bw) {
	    case 0: return 7.8E3;
	    case 1: return 10.4E3;
	    case 2: return 15.6E3;
	    case 3: return 20.8E3;
	    case 4: return 31.25E3;
	    case 5: return 41.7E3;
	    case 6: return 62.5E3;
	    case 7: return 125E3;
	    case 8: return 250E3;
	    case 9: return 500E3;
	  }

	  return -1;
}

void lora_set_signal_bandwidth(long sbw)
{
	int bw;

	  if (sbw <= 7.8E3) {
	    bw = 0;
	  } else if (sbw <= 10.4E3) {
	    bw = 1;
	  } else if (sbw <= 15.6E3) {
	    bw = 2;
	  } else if (sbw <= 20.8E3) {
	    bw = 3;
	  } else if (sbw <= 31.25E3) {
	    bw = 4;
	  } else if (sbw <= 41.7E3) {
	    bw = 5;
	  } else if (sbw <= 62.5E3) {
	    bw = 6;
	  } else if (sbw <= 125E3) {
	    bw = 7;
	  } else if (sbw <= 250E3) {
	    bw = 8;
	  } else /*if (sbw <= 250E3)*/ {
	    bw = 9;
	  }

	  lora_write_register(REG_MODEM_CONFIG_1, (lora_read_register(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
	  lora_set_ldo_flag();
}


void lora_set_ldo_flag(void)
{
	long symbol_duration = 1000 / (lora_get_signal_bandwidth() / (1L << lora_get_spreading_factor()));

	// Section 4.1.1.6
	bool ldoOn = symbol_duration > 16;

	uint8_t config3 = lora_read_register(REG_MODEM_CONFIG_3);
	//WRITE_BIT(config3, 3, ldoOn);
	config3 |= (ldoOn << 3);
	lora_write_register(REG_MODEM_CONFIG_3, config3);
}

void lora_set_coding_rate4(int denominator)
{
	if(denominator < 5) {
		denominator = 5;
	} else if (denominator > 8) {
		denominator = 8;
	}

	int cr = denominator - 4;

	lora_write_register(REG_MODEM_CONFIG_1, (lora_read_register(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void lora_set_preamble_length(long length)
{
	lora_write_register(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
	lora_write_register(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void lora_set_sync_word(int sw)
{
	lora_write_register(REG_SYNC_WORD, sw);
}

void lora_enable_crc(void)
{
	lora_write_register(REG_MODEM_CONFIG_2, lora_read_register(REG_MODEM_CONFIG_2) | 0x04);
}

void lora_disable_crc(void)
{
	lora_write_register(REG_MODEM_CONFIG_2, lora_read_register(REG_MODEM_CONFIG_2) & 0xfb);
}

void lora_enable_invert_IQ(void)
{
	lora_write_register(REG_INVERTIQ, 0x66);
	lora_write_register(REG_INVERTIQ2, 0x19);
}

void lora_disable_invert_IQ(void)
{
	lora_write_register(REG_INVERTIQ, 0x27);
	lora_write_register(REG_INVERTIQ2, 0x1d);
}


void lora_set_OCP(uint8_t mA)
{
	uint8_t ocpTrim = 27;
	if(mA <= 120) {
		ocpTrim = (mA - 45) / 5;
	} else if(mA <= 240) {
		ocpTrim = (mA + 30) / 10;
	}
	lora_write_register(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void lora_set_Gain(uint8_t gain)
{
	// check allowed range
	if(gain > 6) {
		gain = 6;
	}
	// set to standby
	lora_idle();

	// set gain
	if(gain == 0) {
		// if gain = 0, enable AGC
		lora_write_register(REG_MODEM_CONFIG_3, 0x04);
	} else {
		// disable AGC
		lora_write_register(REG_MODEM_CONFIG_3, 0x00);

		// clear Gain and set LNA boost
		lora_write_register(REG_LNA, 0x03);

		// set gain
		lora_write_register(REG_LNA, lora_read_register(REG_LNA) | (gain << 5));
	}
}


uint8_t lora_random(void)
{
	return lora_read_register(REG_RSSI_WIDEBAND);
}

void lora_dump_registers(char *buffer)
{
	for (int i = 0; i < 128; i++) {
		sprintf(buffer, "0x%x: 0x%x",i, lora_read_register(i));
		//serial_port_println(buffer);
	}
}

void lora_explicit_header_mode(void)
{
	lora_handle.implicit_header_mode = 0;

	lora_write_register(REG_MODEM_CONFIG_1, lora_read_register(REG_MODEM_CONFIG_1) & 0xfe);
}

void lora_implicit_header_mode(void)
{
	lora_handle.implicit_header_mode = 1;

	lora_write_register(REG_MODEM_CONFIG_1, lora_read_register(REG_MODEM_CONFIG_1) & 0x01);
}


/**
 * when an interrupt come at DIO0
 */
void lora_handle_dio0_rise(void)
{
	int irq_flags = lora_read_register(REG_IRQ_FLAGS);

	// clear IRQ'S
	lora_write_register(REG_IRQ_FLAGS, irq_flags);

	if((irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		if((irq_flags & IRQ_RX_DONE_MASK) != 0) {
			// received a packet
			lora_handle.packet_index = 0;

			// read packet length
			int packet_length = lora_handle.implicit_header_mode ? lora_read_register(REG_PAYLOAD_LENGTH) : lora_read_register(REG_RX_NB_BYTES);

			// set FIFO address to current RX address
			lora_write_register(REG_FIFO_ADDR_PTR, lora_read_register(REG_FIFO_RX_CURRENT_ADDR));

			if(lora_handle._on_receive) {
				lora_handle._on_receive(packet_length);
			}
		} else if ((irq_flags & IRQ_TX_DONE_MASK) != 0) {
			if (lora_handle._on_tx_done) {
				lora_handle._on_tx_done();
			}
		}
	}
}



uint8_t lora_read_register(uint8_t address)
{
	return lora_single_transfer(address & 0x7f, 0x00, LORA_READ);
}

void lora_write_register(uint8_t address, uint8_t value)
{
	lora_single_transfer(address | 0x80, value, LORA_WRITE);
}

uint8_t lora_single_transfer(uint8_t address, uint8_t value, uint8_t mode)
{
	uint8_t res[1];
	HAL_GPIO_WritePin(lora_handle.nss_port, lora_handle.nss_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(lora_handle.spi_handle,(uint8_t *)&address, 1, 100);
	if(mode == LORA_WRITE)
	{
		HAL_SPI_Transmit(lora_handle.spi_handle,(uint8_t *)&value, 1, 100);
	}
	else
	{
		HAL_SPI_Receive(lora_handle.spi_handle, res, 1, 100);
	}
	HAL_GPIO_WritePin(lora_handle.nss_port, lora_handle.nss_pin, GPIO_PIN_SET);

	return *res;
}




