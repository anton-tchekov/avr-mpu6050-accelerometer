#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>

#define TWI_FREQ          100000L
#define I2C_BUFFER_LENGTH     32
#define I2C_READY              0
#define I2C_MASTER_RX          1
#define I2C_MASTER_TX          2
#define I2C_SLAVE_RX           3
#define I2C_SLAVE_TX           4

static volatile uint8_t state;
static volatile uint8_t error;
static uint8_t slave_rw;

static uint8_t master_buf[I2C_BUFFER_LENGTH];
static volatile uint8_t master_buf_idx;
static uint8_t master_buf_len;

static uint8_t rx_buf[I2C_BUFFER_LENGTH];
static uint8_t rx_buf_idx = 0;
static uint8_t rx_buf_len = 0;

static uint8_t tx_addr = 0;
static uint8_t tx_buf[I2C_BUFFER_LENGTH];
static uint8_t tx_buf_idx = 0;
static uint8_t tx_buf_len = 0;

static void i2c_stop(void);
static void i2c_release_bus(void);
static uint8_t i2c_read_from
	(uint8_t address, uint8_t *data, uint8_t length);

static uint8_t i2c_write_to
	(uint8_t address, uint8_t *data, uint8_t length, uint8_t wait);

static void i2c_reply_ack(void);
static void i2c_reply_nack(void);

static void i2c_init(void);
static void i2c_begin_transmission(uint8_t address);
static uint8_t i2c_end_transmission(void);
static uint8_t i2c_request_from(uint8_t address, uint8_t count);
static void i2c_write(uint8_t data);
static uint8_t i2c_read(void);
static uint8_t i2c_available(void);

static void i2c_init(void)
{
	rx_buf_idx = 0;
	rx_buf_len = 0;
	tx_buf_idx = 0;
	tx_buf_len = 0;

	/* initialize state */
	state = I2C_READY;

	/* internal pullups on i2c pins */
	PORTC |= (1 << 4);
	PORTC |= (1 << 5);

	/* initialize i2c prescaler and bitrate */
	TWSR &= ~TWPS0;
	TWSR &= ~TWPS1;
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;

	/* enable i2c module and interrupt */
	TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}

static void i2c_begin_transmission(uint8_t address)
{
	tx_addr = address;
	tx_buf_idx = 0;
	tx_buf_len = 0;
}

static uint8_t i2c_end_transmission(void)
{
	int8_t ret = i2c_write_to(tx_addr, tx_buf, tx_buf_len, 1);
	tx_buf_len = 0;
	tx_buf_idx = 0;
	return ret;
}

static uint8_t i2c_request_from(uint8_t address, uint8_t count)
{
	uint8_t read;
	if(count > I2C_BUFFER_LENGTH)
	{
		count = I2C_BUFFER_LENGTH;
	}

	read = i2c_read_from(address, rx_buf, count);
	rx_buf_idx = 0;
	rx_buf_len = read;
	return read;
}

static void i2c_write(uint8_t data)
{
	if(tx_buf_len >= I2C_BUFFER_LENGTH) { return; }
	tx_buf[tx_buf_idx++] = data;
	tx_buf_len = tx_buf_idx;
}

static uint8_t i2c_read(void)
{
	return (rx_buf_idx < rx_buf_len)
		? rx_buf[rx_buf_idx++] : '\0';
}

static uint8_t i2c_available(void)
{
	return rx_buf_len - rx_buf_idx;
}

static void i2c_stop(void)
{
	TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT) |
		(1 << TWSTO);

	while(TWCR & _BV(TWSTO)) ;
	state = I2C_READY;
}

static void i2c_release_bus(void)
{
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
	state = I2C_READY;
}

static uint8_t i2c_read_from
	(uint8_t address, uint8_t *data, uint8_t length)
{
	uint8_t i;

	if(I2C_BUFFER_LENGTH < length)
	{
		return 0;
	}

	while(I2C_READY != state) ;
	state = I2C_MASTER_RX;
	error = 0xFF;
	master_buf_idx = 0;
	master_buf_len = length - 1;
	slave_rw = TW_READ;
	slave_rw |= address << 1;
	TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT) |
		(1 << TWSTA);

	while(state == I2C_MASTER_RX) ;

	if(master_buf_idx < length)
	{
		length = master_buf_idx;
	}

	for(i = 0; i < length; ++i)
	{
		data[i] = master_buf[i];
	}

	return length;
}

static uint8_t i2c_write_to
	(uint8_t address, uint8_t *data, uint8_t length, uint8_t wait)
{
	uint8_t i;

	if(I2C_BUFFER_LENGTH < length)
	{
		return 1;
	}

	while(I2C_READY != state) ;
	state = I2C_MASTER_TX;
	error = 0xFF;
	master_buf_idx = 0;
	master_buf_len = length;
	for(i = 0; i < length; ++i)
	{
		master_buf[i] = data[i];
	}

	slave_rw = TW_WRITE;
	slave_rw |= address << 1;
	TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT) |
		(1 << TWSTA);

	while(wait && (I2C_MASTER_TX == state)) ;

	if(error == 0xFF)
	{
		/* success */
		return 0;
	}
	else if(error == TW_MT_SLA_NACK)
	{
		/* error: address sent, NACK received */
		return 2;
	}
	else if(error == TW_MT_DATA_NACK)
	{
		/* error: data sent, NACK received */
		return 3;
	}
	else
	{
		/* other error */
		return 4;
	}
}

static void i2c_reply_ack(void)
{
	TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
}

static void i2c_reply_nack(void)
{
	TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
}

SIGNAL(TWI_vect)
{
	switch(TW_STATUS)
	{
		/* all master */
		case TW_START:
			/* sent start condition */

		case TW_REP_START:
			/* sent repeated start condition */
			TWDR = slave_rw;
			i2c_reply_ack();
			break;

		/* master transmitter */
		case TW_MT_SLA_ACK:
			/* slave receiver acknowledged address */

		case TW_MT_DATA_ACK:
			/* slave receiver acknowledged data */
			if(master_buf_idx < master_buf_len)
			{
				TWDR = master_buf[master_buf_idx++];
				i2c_reply_ack();
			}
			else
			{
				i2c_stop();
			}
			break;

		case TW_MT_SLA_NACK:
			/* address sent, NACK received */
			error = TW_MT_SLA_NACK;
			i2c_stop();
			break;

		case TW_MT_DATA_NACK:
			/* data sent, NACK received */
			error = TW_MT_DATA_NACK;
			i2c_stop();
			break;

		case TW_MT_ARB_LOST:
			/* bus arbitration lost */
			error = TW_MT_ARB_LOST;
			i2c_release_bus();
			break;

		/* master receiver */
		case TW_MR_DATA_ACK:
			/* data received, ACK sent */
			master_buf[master_buf_idx++] = TWDR;

		case TW_MR_SLA_ACK:
			/* address sent, ACK received */
			if(master_buf_idx < master_buf_len)
			{
				i2c_reply_ack();
			}
			else
			{
				i2c_reply_nack();
			}
			break;

		case TW_MR_DATA_NACK:
			/* data received, NACK sent */
			master_buf[master_buf_idx++] = TWDR;

		case TW_MR_SLA_NACK:
			/* address sent, NACK received */
			i2c_stop();
			break;

		/* all */
		case TW_NO_INFO:
			/* no state information */
			break;

		case TW_BUS_ERROR:
			/* bus error, illegal stop/start */
			error = TW_BUS_ERROR;
			i2c_stop();
			break;
	}
}

