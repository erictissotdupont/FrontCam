
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

#include "ArduCAM.h"

/*
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
*/

#include "ov2640_regs.h"
#include "ov5642_regs.h"


regtype *P_CS;
regsize B_CS;
byte m_fmt = BMP;
byte sensor_model;
byte sensor_addr;

spi_device_handle_t CAM_spi;

#define sleep_ms(t) vTaskDelay( t / portTICK_PERIOD_MS )

static const char *TAG = "arduCAM";

bool CAM_start()
{ 
  // Enable power
  CAM_resume( );

  // Set register bank to "SENSOR"
  if( !CAM_wrSensorReg8_8(0xff, 0x01)) return false;
  sleep_ms(30);

  if( CAM_rdSensorReg8_8( 0x1C, NULL) != 0x7F ) return false;
  if( CAM_rdSensorReg8_8( 0x1D, NULL) != 0xA2 ) return false; 
  if( CAM_rdSensorReg8_8( 0x0A, NULL) != 0x26 ) return false;
  if( CAM_rdSensorReg8_8( 0x0B, NULL) != 0x42 ) return false;         

  CAM_wrSensorReg8_8(0x12, 0x80);
  sleep_ms(100);
  
  if (m_fmt == JPEG)
  {
    CAM_wrSensorRegs8_8(OV2640_JPEG_INIT);
    CAM_wrSensorRegs8_8(OV2640_YUV422);
    CAM_wrSensorRegs8_8(OV2640_JPEG);
    CAM_wrSensorReg8_8(0xff, 0x01);
    CAM_wrSensorReg8_8(0x15, 0x00);
    CAM_wrSensorRegs8_8(OV2640_320x240_JPEG);
  }
  else
  {
    CAM_wrSensorRegs8_8(OV2640_QVGA);
  }
  
  ESP_LOGI(TAG,"ArduCAM version is 0x%02X", CAM_read_reg(0x40));

  ESP_LOGI(TAG,"GPIO direction is 0x%02X", CAM_read_reg(0x05));
  ESP_LOGI(TAG,"GPIO value is     0x%02X", CAM_read_reg(0x06));
  
  return true;
}

bool CAM_standby( )
{
  CAM_write_reg( 0x06, 0x12 );
  //CAM_set_bit( 0x03, 0x40 );
  return true;
}

bool CAM_resume( )
{
  //CAM_clear_bit( 0x03, 0x40 );
  CAM_write_reg( 0x05, 0x00 );
  CAM_write_reg( 0x06, 0x15 );
  return true;
}

void CAM_CS_HIGH(void)
{
	 //sbi(P_CS, B_CS);	
}
void CAM_CS_LOW(void)
{
	 //cbi(P_CS, B_CS);	
}

void CAM_flush_fifo(void)
{
  // Reg 0x04 = 0x31
	CAM_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK | FIFO_RDPTR_RST_MASK | FIFO_WRPTR_RST_MASK);
}

void CAM_start_capture(void)
{
  CAM_write_reg(ARDUCHIP_FRAMES, 0 ); // One frame
  
  // Reg 0x04 = 0x02
	CAM_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}


void CAM_clear_fifo_flag(void )
{
  
	CAM_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}


uint8_t CAM_read_fifo(void)
{
	uint8_t data;
	data = CAM_bus_read(SINGLE_FIFO_READ);
	return data;
}


uint8_t CAM_read_reg(uint8_t addr)
{
  esp_err_t ret;
  spi_transaction_t t;
  uint8_t cmd[2];
  
  cmd[0] = addr & 0x7F;
  cmd[1] = 0;
  
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length = 8*2 + 1;                   //Command is 8 bits
  t.tx_buffer = &cmd;               //The data is the cmd itself
  t.user = (void*)0;                //D/C needs to be set to 0
  t.flags = SPI_TRANS_USE_RXDATA;
  ret = spi_device_polling_transmit(CAM_spi, &t);  //Transmit!
  assert( ret==ESP_OK );            //Should have had no issues.
  
  /*
  memset(&t, 0, sizeof(t));
  t.length=8*1;
  t.flags = SPI_TRANS_USE_RXDATA;
  t.user = (void*)1;
  ret = spi_device_polling_transmit(CAM_spi, &t);
  assert( ret == ESP_OK );
  */
  /*
  ESP_LOGI(TAG, "Got %02X %02X", 
   ((uint8_t*)t.rx_data)[0],
   ((uint8_t*)t.rx_data)[1] );
  */
  
  return ((uint8_t*)t.rx_data)[1];
  
/*  
  uint8_t value = 0;
	addr = addr& 0x7f;
 	//cbi(P_CS, B_CS);
	spi_write_blocking(SPI_PORT, &addr, 1);
  spi_read_blocking(SPI_PORT, 0, &value, 1);
  //sbi(P_CS, B_CS);
	return value;
*/
}

void CAM_write_reg(uint8_t addr, uint8_t data)
{
  esp_err_t ret;
  spi_transaction_t t;
  uint8_t cmd[2];
  
  cmd[0] = addr | 0x80;
  cmd[1] = data;
  
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8*2 + 1;                //Command is 8 bits
  t.tx_buffer=&cmd;               //The data is the cmd itself
  t.user=(void*)0;                //D/C needs to be set to 0
  ret=spi_device_polling_transmit(CAM_spi, &t);  //Transmit!
  
  assert( ret == ESP_OK );
  
  sleep_ms(10);
  
  /*
    uint8_t buf[2];
    buf[0] = addr|WRITE_BIT ;  // remove read bit as this is a write
    buf[1] = data;
    //cbi(P_CS, B_CS);
    spi_write_blocking(SPI_PORT, buf, 2);
    //sbi(P_CS, B_CS);
    sleep_ms(1);
    */
}


uint32_t CAM_read_fifo_length(void)
{
	uint32_t len1,len2,len3,length=0;
	len1 = CAM_read_reg(FIFO_SIZE1);
  len2 = CAM_read_reg(FIFO_SIZE2);
  len3 = CAM_read_reg(FIFO_SIZE3) & 0x7f;
  length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return length;	
}

void CAM_set_fifo_burst( uint8_t* image, uint32_t size )
{
  esp_err_t ret;
  spi_transaction_t t;
  uint8_t txData = BURST_FIFO_READ;
  
  memset(&t, 0, sizeof(t));
  t.length=8 * size + 1;
  //t.flags = SPI_TRANS_USE_RXDATA;
  t.tx_buffer=&txData;
  t.rx_buffer=image;
  t.user = (void*)1;
  ret = spi_device_polling_transmit(CAM_spi, &t);
  assert( ret == ESP_OK );
  
  //uint8_t value;
  //spi_read_blocking(SPI_PORT, BURST_FIFO_READ, &value, 1);	
}

//Set corresponding bit  
void CAM_set_bit(uint8_t addr, uint8_t bit)
{
	uint8_t temp;
	temp = CAM_read_reg(addr);
	CAM_write_reg(addr, temp | bit);
}


//Clear corresponding bit 
void CAM_clear_bit(uint8_t addr, uint8_t bit)
{
	uint8_t temp;
	temp = CAM_read_reg(addr);
	CAM_write_reg(addr, temp & (~bit));
}


//Get corresponding bit status
uint8_t CAM_get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = CAM_read_reg(addr);
  temp = temp & bit;
  return temp;
}

uint8_t CAM_bus_write(int address,int value)
{	
	//cbi(P_CS, B_CS);
		//SPI.transfer(address);
		//SPI.transfer(value);
	//sbi(P_CS, B_CS);
	return 1;
}


uint8_t CAM_bus_read(int address)
{
	uint8_t value = 0;
	//cbi(P_CS, B_CS);
	//	  SPI.transfer(address);
//		 value = SPI.transfer(0x00);
	//sbi(P_CS, B_CS);
	return value;
}

	// Write 8 bit values to 8 bit register address
int CAM_wrSensorRegs8_8(const struct sensor_reg reglist[])
{
	  uint16_t reg_addr = 0;
	  uint16_t reg_val = 0;
	  const struct sensor_reg *next = reglist;
	  while ((reg_addr != 0xff) || (reg_val != 0xff))
	  {
	    reg_addr = next->reg;
	    reg_val = next->val;
	    CAM_wrSensorReg8_8(reg_addr, reg_val);
	    next++;
	  }
	return 1;
}

	// Write 16 bit values to 8 bit register address
int CAM_wrSensorRegs8_16(const struct sensor_reg reglist[])
{
  /*
	  unsigned int reg_addr, reg_val;
	  const struct sensor_reg *next = reglist;
	  while ((reg_addr != 0xff) | (reg_val != 0xffff))
	  {

	     reg_addr = next->reg;
	     reg_val = next->val;
	    CAM_wrSensorReg8_16(reg_addr, reg_val);
	    next++;
	  }
    */
	return 1;
}
// Write 8 bit values to 16 bit register address
int CAM_wrSensorRegs16_8(const struct sensor_reg reglist[])
{
	  unsigned int reg_addr = 0;
	  unsigned char reg_val = 0;
	  const struct sensor_reg *next = reglist;
	  while ((reg_addr != 0xffff) | (reg_val != 0xff))
	  {
	     reg_addr = next->reg;
	     reg_val = next->val;
	    CAM_wrSensorReg16_8(reg_addr, reg_val);
	    next++;
	  }
	return 1;
}

// Read/write 8 bit value to/from 16 bit register address
byte CAM_wrSensorReg16_8(int regID, int regDat)
{
  esp_err_t ret;
    uint8_t buf[3]={0};
    buf[0]=(regID >> 8)&0xff;
    buf[1]=(regID)&0xff;
    buf[2]=regDat;
    
    // TODO : Figure out what to do about the STOP
    
  ret = i2c_master_write_to_device(I2C_PORT, sensor_addr, buf, 3, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
  if( ret != ESP_OK )
  {
    ESP_LOGE(TAG, "I2C write 0x%04X = 0x%02X failed : %d", regID, regDat, ret );
  }
  else
  {
    // ESP_LOGI(TAG, "I2C write 0x%04X = 0x%02X", regID, regDat );
  }
    
    //i2c_write_blocking(I2C_PORT, sensor_addr, buf,  3, true );
		sleep_ms(10);
	  return 1;
}

// Read/write 8 bit value to/from 8 bit register address	
byte CAM_wrSensorReg8_8(int regID, int regDat)
{
  esp_err_t ret;
  uint8_t buf[2];
  
  buf[0] = regID;
  buf[1] = regDat;
    
    // TODO : Figure out what to do about the STOP bit
    
  ret = i2c_master_write_to_device(I2C_PORT, sensor_addr, buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
  if( ret != ESP_OK )
  {
    ESP_LOGE(TAG, "I2C write 0x%02X = 0x%02X failed : %d", regID, regDat, ret );
    return 0;
  }

  //ESP_LOGI(TAG, "I2C write 0x%02X = 0x%02X", regID, regDat );
  return 1;
}

byte CAM_rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{	
  esp_err_t ret;
  uint8_t val = 0;
  
#if 0
  ret = i2c_master_write_read_device( 
    I2C_PORT, 
    sensor_addr, 
    &regID, 1, 
    &val, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
#else
  i2c_cmd_handle_t h = i2c_cmd_link_create( );
  
  i2c_master_start( h );
  i2c_master_write_byte( h, sensor_addr << 1, true );
  i2c_master_write_byte( h, regID, true );
  //i2c_master_stop( h );
  i2c_master_start( h );
  i2c_master_write_byte( h, (sensor_addr << 1) + 1, true );
  i2c_master_read_byte( h, &val, true );
  i2c_master_stop( h );
  
  ret = i2c_master_cmd_begin( I2C_PORT, h, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
  i2c_cmd_link_delete( h );
#endif
    
  if( ret != ESP_OK )
  {
    ESP_LOGE(TAG, "I2C read 0x%2X failed : %d", regID, ret );
  }
  else
  {
    if( regDat ) *regDat = val;
    ESP_LOGI(TAG, "I2C read 0x%02X = 0x%02X", regID, val );
  }
  
  //i2c_write_blocking(I2C_PORT, sensor_addr, &regID, 1, true );
  //i2c_read_blocking(I2C_PORT, sensor_addr, regDat,  1, false );
  return val;
}

byte CAM_rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
  esp_err_t ret;
	uint8_t buffer[2]={0};
	buffer[0]=(regID>>8)&0xff;
	buffer[1]=regID&0xff;
  
  ret = i2c_master_write_read_device(I2C_PORT, sensor_addr, buffer, 2, regDat, 1, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
  if( ret != ESP_OK )
  {
    ESP_LOGE(TAG, "I2C read 0x%4X failed : %d", regID, ret );
  }
  else
  {
    ESP_LOGI(TAG, "I2C read 0x%04X = 0x%02X", regID, *regDat );
  }
  
	//i2c_write_blocking(I2C_PORT, sensor_addr, buffer, 2, true );
	//i2c_read_blocking(I2C_PORT, sensor_addr, regDat,  1, false );
	return 1;
}

	void CAM_OV2640_set_Special_effects(uint8_t Special_effect)
	{
// #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))	
		switch(Special_effect)
		{
			case Antique:
	
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x18);
				CAM_wrSensorReg8_8(0x7c, 0x05);
				CAM_wrSensorReg8_8(0x7d, 0x40);
				CAM_wrSensorReg8_8(0x7d, 0xa6);
			break;
			case Bluish:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x18);
				CAM_wrSensorReg8_8(0x7c, 0x05);
				CAM_wrSensorReg8_8(0x7d, 0xa0);
				CAM_wrSensorReg8_8(0x7d, 0x40);
			break;
			case Greenish:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x18);
				CAM_wrSensorReg8_8(0x7c, 0x05);
				CAM_wrSensorReg8_8(0x7d, 0x40);
				CAM_wrSensorReg8_8(0x7d, 0x40);
			break;
			case Reddish:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x18);
				CAM_wrSensorReg8_8(0x7c, 0x05);
				CAM_wrSensorReg8_8(0x7d, 0x40);
				CAM_wrSensorReg8_8(0x7d, 0xc0);
			break;
			case BW:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x18);
				CAM_wrSensorReg8_8(0x7c, 0x05);
				CAM_wrSensorReg8_8(0x7d, 0x80);
				CAM_wrSensorReg8_8(0x7d, 0x80);
			break;
			case Negative:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x40);
				CAM_wrSensorReg8_8(0x7c, 0x05);
				CAM_wrSensorReg8_8(0x7d, 0x80);
				CAM_wrSensorReg8_8(0x7d, 0x80);
			break;
			case BWnegative:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x58);
				CAM_wrSensorReg8_8(0x7c, 0x05);
				CAM_wrSensorReg8_8(0x7d, 0x80);
			  CAM_wrSensorReg8_8(0x7d, 0x80);
	
			break;
			case Normal:
		
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x05);
				CAM_wrSensorReg8_8(0x7d, 0x80);
				CAM_wrSensorReg8_8(0x7d, 0x80);
			
			break;
					
		}
	// #endif
	}


	void CAM_OV2640_set_Contrast(uint8_t Contrast)
	{
//  #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))	
		switch(Contrast)
		{
			case Contrast2:
		
			CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x07);
				CAM_wrSensorReg8_8(0x7d, 0x20);
				CAM_wrSensorReg8_8(0x7d, 0x28);
				CAM_wrSensorReg8_8(0x7d, 0x0c);
				CAM_wrSensorReg8_8(0x7d, 0x06);
			break;
			case Contrast1:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x07);
				CAM_wrSensorReg8_8(0x7d, 0x20);
				CAM_wrSensorReg8_8(0x7d, 0x24);
				CAM_wrSensorReg8_8(0x7d, 0x16);
				CAM_wrSensorReg8_8(0x7d, 0x06); 
			break;
			case Contrast0:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x07);
				CAM_wrSensorReg8_8(0x7d, 0x20);
				CAM_wrSensorReg8_8(0x7d, 0x20);
				CAM_wrSensorReg8_8(0x7d, 0x20);
				CAM_wrSensorReg8_8(0x7d, 0x06); 
			break;
			case Contrast_1:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x07);
				CAM_wrSensorReg8_8(0x7d, 0x20);
				CAM_wrSensorReg8_8(0x7d, 0x20);
				CAM_wrSensorReg8_8(0x7d, 0x2a);
		  CAM_wrSensorReg8_8(0x7d, 0x06);	
			break;
			case Contrast_2:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x07);
				CAM_wrSensorReg8_8(0x7d, 0x20);
				CAM_wrSensorReg8_8(0x7d, 0x18);
				CAM_wrSensorReg8_8(0x7d, 0x34);
				CAM_wrSensorReg8_8(0x7d, 0x06);
			break;
		}
// #endif		
	}

	void CAM_OV2640_set_Brightness(uint8_t Brightness)
	{
	// #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))
		switch(Brightness)
		{
			case Brightness2:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x09);
				CAM_wrSensorReg8_8(0x7d, 0x40);
				CAM_wrSensorReg8_8(0x7d, 0x00);
			break;
			case Brightness1:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x09);
				CAM_wrSensorReg8_8(0x7d, 0x30);
				CAM_wrSensorReg8_8(0x7d, 0x00);
			break;	
			case Brightness0:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x09);
				CAM_wrSensorReg8_8(0x7d, 0x20);
				CAM_wrSensorReg8_8(0x7d, 0x00);
			break;
			case Brightness_1:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x09);
				CAM_wrSensorReg8_8(0x7d, 0x10);
				CAM_wrSensorReg8_8(0x7d, 0x00);
			break;
			case Brightness_2:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x04);
				CAM_wrSensorReg8_8(0x7c, 0x09);
				CAM_wrSensorReg8_8(0x7d, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x00);
			break;	
		}
// #endif	
			
	}

	void CAM_OV2640_set_Color_Saturation(uint8_t Color_Saturation)
	{
	// #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))
		switch(Color_Saturation)
		{
			case Saturation2:
			
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x02);
				CAM_wrSensorReg8_8(0x7c, 0x03);
				CAM_wrSensorReg8_8(0x7d, 0x68);
				CAM_wrSensorReg8_8(0x7d, 0x68);
			break;
			case Saturation1:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x02);
				CAM_wrSensorReg8_8(0x7c, 0x03);
				CAM_wrSensorReg8_8(0x7d, 0x58);
				CAM_wrSensorReg8_8(0x7d, 0x58);
			break;
			case Saturation0:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x02);
				CAM_wrSensorReg8_8(0x7c, 0x03);
				CAM_wrSensorReg8_8(0x7d, 0x48);
				CAM_wrSensorReg8_8(0x7d, 0x48);
			break;
			case Saturation_1:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x02);
				CAM_wrSensorReg8_8(0x7c, 0x03);
				CAM_wrSensorReg8_8(0x7d, 0x38);
				CAM_wrSensorReg8_8(0x7d, 0x38);
			break;
			case Saturation_2:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0x7c, 0x00);
				CAM_wrSensorReg8_8(0x7d, 0x02);
				CAM_wrSensorReg8_8(0x7c, 0x03);
				CAM_wrSensorReg8_8(0x7d, 0x28);
				CAM_wrSensorReg8_8(0x7d, 0x28);
			break;	
		}
// #endif	
	}

void CAM_OV2640_set_Light_Mode(uint8_t Light_Mode)
	{
//  #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))
		 switch(Light_Mode)
		 {
			
			  case Auto:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0xc7, 0x00); //AWB on
			  break;
			  case Sunny:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0xc7, 0x40); //AWB off
			  CAM_wrSensorReg8_8(0xcc, 0x5e);
				CAM_wrSensorReg8_8(0xcd, 0x41);
				CAM_wrSensorReg8_8(0xce, 0x54);
			  break;
			  case Cloudy:
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0xc7, 0x40); //AWB off
				CAM_wrSensorReg8_8(0xcc, 0x65);
				CAM_wrSensorReg8_8(0xcd, 0x41);
				CAM_wrSensorReg8_8(0xce, 0x4f);  
			  break;
			  case Office:
			  CAM_wrSensorReg8_8(0xff, 0x00);
			  CAM_wrSensorReg8_8(0xc7, 0x40); //AWB off
			  CAM_wrSensorReg8_8(0xcc, 0x52);
			  CAM_wrSensorReg8_8(0xcd, 0x41);
			  CAM_wrSensorReg8_8(0xce, 0x66);
			  break;
			  case Home:
			  CAM_wrSensorReg8_8(0xff, 0x00);
			  CAM_wrSensorReg8_8(0xc7, 0x40); //AWB off
			  CAM_wrSensorReg8_8(0xcc, 0x42);
			  CAM_wrSensorReg8_8(0xcd, 0x3f);
			  CAM_wrSensorReg8_8(0xce, 0x71);
			  break;
			  default :
				CAM_wrSensorReg8_8(0xff, 0x00);
				CAM_wrSensorReg8_8(0xc7, 0x00); //AWB on
			  break; 
		 }	
// #endif
	}


void CAM_OV2640_set_JPEG_size(uint8_t size)
{
// #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))
	switch(size)
	{
		case OV2640_160x120:
			CAM_wrSensorRegs8_8(OV2640_160x120_JPEG);
			break;
		case OV2640_176x144:
			CAM_wrSensorRegs8_8(OV2640_176x144_JPEG);
			break;
		case OV2640_320x240:
			CAM_wrSensorRegs8_8(OV2640_320x240_JPEG);
			break;
		case OV2640_352x288:
	  	CAM_wrSensorRegs8_8(OV2640_352x288_JPEG);
			break;
		case OV2640_640x480:
			CAM_wrSensorRegs8_8(OV2640_640x480_JPEG);
			break;
		case OV2640_800x600:
			CAM_wrSensorRegs8_8(OV2640_800x600_JPEG);
			break;
		case OV2640_1024x768:
			CAM_wrSensorRegs8_8(OV2640_1024x768_JPEG);
			break;
		case OV2640_1280x1024:
			CAM_wrSensorRegs8_8(OV2640_1280x1024_JPEG);
			break;
		case OV2640_1600x1200:
			CAM_wrSensorRegs8_8(OV2640_1600x1200_JPEG);
			break;
		default:
			CAM_wrSensorRegs8_8(OV2640_320x240_JPEG);
			break;
	}
//#endif
}


void CAM_set_format(byte fmt)
{
  if (fmt == BMP)
    m_fmt = BMP;
  else if(fmt == RAW)
    m_fmt = RAW;
  else
    m_fmt = JPEG;
}
unsigned char usart_symbol=0;
unsigned char usart_Command = 0;
// RX interrupt handler
void on_uart_rx() {
  /*
    while (uart_is_readable(UART_ID)) {
       usart_Command = uart_getc(UART_ID);
       usart_symbol=1;
    }
  */
}

void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
  // Do nothing
}

void CAM_init(byte model)
{
	sensor_model = model;
	switch (sensor_model)
	{
    case OV2640:
      	sensor_addr = 0x30;
    break;
     case OV5642:
      	sensor_addr = 0x3C;
    break;
		default:
      sensor_model = OV7670;
      sensor_addr = 0x42;
		break;
	}	
  
  esp_err_t ret;

  int i2c_master_port = I2C_MASTER_NUM;

  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };

  i2c_param_config(i2c_master_port, &conf);

  ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
  ESP_ERROR_CHECK(ret);
    
  spi_bus_config_t buscfg={
    .miso_io_num=PIN_NUM_MISO,
    .mosi_io_num=PIN_NUM_MOSI,
    .sclk_io_num=PIN_NUM_CLK,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=16*320*2+8
  };
  spi_device_interface_config_t devcfg={
    .clock_speed_hz=1*1000*1000,           //Clock out at 4 MHz
    .mode=0,                                //SPI mode 0
    .spics_io_num=PIN_NUM_CS,               //CS pin
    .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
  };
  
  //Initialize the SPI bus
  ret=spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  //Attach the LCD to the SPI bus
  ret=spi_bus_add_device(LCD_HOST, &devcfg, &CAM_spi);
  ESP_ERROR_CHECK(ret);
  
  ESP_LOGI(TAG, "Initalized" );

/*
    // This example will use I2C0 on GPIO4 (SDA) and GPIO5 (SCL)
  i2c_init(I2C_PORT, 100 * 1000);
  gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(PIN_SDA);
  gpio_pull_up(PIN_SCL);
  // Make the I2C pins available to picotool
  bi_decl( bi_2pins_with_func(PIN_SDA, PIN_SCL, GPIO_FUNC_I2C));
    // This example will use SPI0 at 0.5MHz.
  spi_init(SPI_PORT, 4 * 1000*1000);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
*/
}


void CAM_OV5642_set_JPEG_size(uint8_t size)
{
//#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)
  //uint8_t reg_val;

  switch (size)
  {
    case OV5642_320x240:
      CAM_wrSensorRegs16_8(ov5642_320x240);
      break;
    case OV5642_640x480:
      CAM_wrSensorRegs16_8(ov5642_640x480);
      break;
    case OV5642_1024x768:
      CAM_wrSensorRegs16_8(ov5642_1024x768);
      break;
    case OV5642_1280x960:
      CAM_wrSensorRegs16_8(ov5642_1280x960);
      break;
    case OV5642_1600x1200:
      CAM_wrSensorRegs16_8(ov5642_1600x1200);
      break;
    case OV5642_2048x1536:
      CAM_wrSensorRegs16_8(ov5642_2048x1536);
      break;
    case OV5642_2592x1944:
      CAM_wrSensorRegs16_8(ov5642_2592x1944);
      break;
    default:
      CAM_wrSensorRegs16_8(ov5642_320x240);
      break;
  }
//#endif
}

void CAM_OV5642_set_Light_Mode(uint8_t Light_Mode)
{
//#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)
		switch(Light_Mode)
		{
		
			case Advanced_AWB:
			CAM_wrSensorReg16_8(0x3406 ,0x0 );
			CAM_wrSensorReg16_8(0x5192 ,0x04);
			CAM_wrSensorReg16_8(0x5191 ,0xf8);
			CAM_wrSensorReg16_8(0x518d ,0x26);
			CAM_wrSensorReg16_8(0x518f ,0x42);
			CAM_wrSensorReg16_8(0x518e ,0x2b);
			CAM_wrSensorReg16_8(0x5190 ,0x42);
			CAM_wrSensorReg16_8(0x518b ,0xd0);
			CAM_wrSensorReg16_8(0x518c ,0xbd);
			CAM_wrSensorReg16_8(0x5187 ,0x18);
			CAM_wrSensorReg16_8(0x5188 ,0x18);
			CAM_wrSensorReg16_8(0x5189 ,0x56);
			CAM_wrSensorReg16_8(0x518a ,0x5c);
			CAM_wrSensorReg16_8(0x5186 ,0x1c);
			CAM_wrSensorReg16_8(0x5181 ,0x50);
			CAM_wrSensorReg16_8(0x5184 ,0x20);
			CAM_wrSensorReg16_8(0x5182 ,0x11);
			CAM_wrSensorReg16_8(0x5183 ,0x0 );	
			break;
			case Simple_AWB:
			CAM_wrSensorReg16_8(0x3406 ,0x00);
			CAM_wrSensorReg16_8(0x5183 ,0x80);
			CAM_wrSensorReg16_8(0x5191 ,0xff);
			CAM_wrSensorReg16_8(0x5192 ,0x00);
			break;
			case Manual_day:
			CAM_wrSensorReg16_8(0x3406 ,0x1 );
			CAM_wrSensorReg16_8(0x3400 ,0x7 );
			CAM_wrSensorReg16_8(0x3401 ,0x32);
			CAM_wrSensorReg16_8(0x3402 ,0x4 );
			CAM_wrSensorReg16_8(0x3403 ,0x0 );
			CAM_wrSensorReg16_8(0x3404 ,0x5 );
			CAM_wrSensorReg16_8(0x3405 ,0x36);
			break;
			case Manual_A:
			CAM_wrSensorReg16_8(0x3406 ,0x1 );
			CAM_wrSensorReg16_8(0x3400 ,0x4 );
			CAM_wrSensorReg16_8(0x3401 ,0x88);
			CAM_wrSensorReg16_8(0x3402 ,0x4 );
			CAM_wrSensorReg16_8(0x3403 ,0x0 );
			CAM_wrSensorReg16_8(0x3404 ,0x8 );
			CAM_wrSensorReg16_8(0x3405 ,0xb6);
			break;
			case Manual_cwf:
			CAM_wrSensorReg16_8(0x3406 ,0x1 );
			CAM_wrSensorReg16_8(0x3400 ,0x6 );
			CAM_wrSensorReg16_8(0x3401 ,0x13);
			CAM_wrSensorReg16_8(0x3402 ,0x4 );
			CAM_wrSensorReg16_8(0x3403 ,0x0 );
			CAM_wrSensorReg16_8(0x3404 ,0x7 );
			CAM_wrSensorReg16_8(0x3405 ,0xe2);
			break;
			case Manual_cloudy:
			CAM_wrSensorReg16_8(0x3406 ,0x1 );
			CAM_wrSensorReg16_8(0x3400 ,0x7 );
			CAM_wrSensorReg16_8(0x3401 ,0x88);
			CAM_wrSensorReg16_8(0x3402 ,0x4 );
			CAM_wrSensorReg16_8(0x3403 ,0x0 );
			CAM_wrSensorReg16_8(0x3404 ,0x5 );
			CAM_wrSensorReg16_8(0x3405 ,0x0);
			break;
			default :
			break; 
		}	
//#endif
}


void CAM_OV5642_set_Color_Saturation(uint8_t Color_Saturation)
{
//#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)
	
		switch(Color_Saturation)
		{
			case Saturation4:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5583 ,0x80);
				CAM_wrSensorReg16_8(0x5584 ,0x80);
				CAM_wrSensorReg16_8(0x5580 ,0x02);
			break;
			case Saturation3:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5583 ,0x70);
				CAM_wrSensorReg16_8(0x5584 ,0x70);
				CAM_wrSensorReg16_8(0x5580 ,0x02);
			break;
			case Saturation2:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5583 ,0x60);
				CAM_wrSensorReg16_8(0x5584 ,0x60);
				CAM_wrSensorReg16_8(0x5580 ,0x02);
			break;
			case Saturation1:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5583 ,0x50);
				CAM_wrSensorReg16_8(0x5584 ,0x50);
				CAM_wrSensorReg16_8(0x5580 ,0x02);
			break;
			case Saturation0:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5583 ,0x40);
				CAM_wrSensorReg16_8(0x5584 ,0x40);
				CAM_wrSensorReg16_8(0x5580 ,0x02);
			break;		
			case Saturation_1:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5583 ,0x30);
				CAM_wrSensorReg16_8(0x5584 ,0x30);
				CAM_wrSensorReg16_8(0x5580 ,0x02);
			break;
				case Saturation_2:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5583 ,0x20);
				CAM_wrSensorReg16_8(0x5584 ,0x20);
				CAM_wrSensorReg16_8(0x5580 ,0x02);
			break;
				case Saturation_3:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5583 ,0x10);
				CAM_wrSensorReg16_8(0x5584 ,0x10);
				CAM_wrSensorReg16_8(0x5580 ,0x02);
			break;
				case Saturation_4:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5583 ,0x00);
				CAM_wrSensorReg16_8(0x5584 ,0x00);
				CAM_wrSensorReg16_8(0x5580 ,0x02);
			break;
		}
//#endif	
}


void CAM_OV5642_set_Brightness(uint8_t Brightness)
{
//	#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)
	
		switch(Brightness)
		{
			case Brightness4:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5589 ,0x40);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Brightness3:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5589 ,0x30);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;	
			case Brightness2:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5589 ,0x20);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Brightness1:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5589 ,0x10);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Brightness0:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5589 ,0x00);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;	
			case Brightness_1:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5589 ,0x10);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x558a ,0x08);
			break;	
			case Brightness_2:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5589 ,0x20);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x558a ,0x08);
			break;	
			case Brightness_3:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5589 ,0x30);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x558a ,0x08);
			break;	
			case Brightness_4:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5589 ,0x40);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x558a ,0x08);
			break;	
		}
//#endif	
			
}


void CAM_OV5642_set_Contrast(uint8_t Contrast)
{
//#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
		switch(Contrast)
		{
			case Contrast4:
			CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x5587 ,0x30);
				CAM_wrSensorReg16_8(0x5588 ,0x30);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast3:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x5587 ,0x2c);
				CAM_wrSensorReg16_8(0x5588 ,0x2c);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast2:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x5587 ,0x28);
				CAM_wrSensorReg16_8(0x5588 ,0x28);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast1:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x5587 ,0x24);
				CAM_wrSensorReg16_8(0x5588 ,0x24);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast0:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x5587 ,0x20);
				CAM_wrSensorReg16_8(0x5588 ,0x20);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast_1:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x5587 ,0x1C);
				CAM_wrSensorReg16_8(0x5588 ,0x1C);
				CAM_wrSensorReg16_8(0x558a ,0x1C);
			break;
			case Contrast_2:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x5587 ,0x18);
				CAM_wrSensorReg16_8(0x5588 ,0x18);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast_3:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x5587 ,0x14);
				CAM_wrSensorReg16_8(0x5588 ,0x14);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast_4:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x04);
				CAM_wrSensorReg16_8(0x5587 ,0x10);
				CAM_wrSensorReg16_8(0x5588 ,0x10);
				CAM_wrSensorReg16_8(0x558a ,0x00);
			break;
		}
//#endif		
}


void CAM_OV5642_set_hue(uint8_t degree)
{
//	#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
		switch(degree)
		{
			case degree_180:
			CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x80);
				CAM_wrSensorReg16_8(0x5582 ,0x00);
				CAM_wrSensorReg16_8(0x558a ,0x32);
			break;
			case degree_150:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x6f);
				CAM_wrSensorReg16_8(0x5582 ,0x40);
				CAM_wrSensorReg16_8(0x558a ,0x32);
			break;
			case degree_120:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x40);
				CAM_wrSensorReg16_8(0x5582 ,0x6f);
				CAM_wrSensorReg16_8(0x558a ,0x32);
			break;
			case degree_90:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x00);
				CAM_wrSensorReg16_8(0x5582 ,0x80);
				CAM_wrSensorReg16_8(0x558a ,0x02);
			break;
			case degree_60:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x40);
				CAM_wrSensorReg16_8(0x5582 ,0x6f);
				CAM_wrSensorReg16_8(0x558a ,0x02);
			break;
			case degree_30:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x6f);
				CAM_wrSensorReg16_8(0x5582 ,0x40);
				CAM_wrSensorReg16_8(0x558a ,0x02);
			break;
			case degree_0:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x80);
				CAM_wrSensorReg16_8(0x5582 ,0x00);
				CAM_wrSensorReg16_8(0x558a ,0x01);
			break;
			case degree30:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x6f);
				CAM_wrSensorReg16_8(0x5582 ,0x40);
				CAM_wrSensorReg16_8(0x558a ,0x01);
			break;
			case degree60:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x40);
				CAM_wrSensorReg16_8(0x5582 ,0x6f);
				CAM_wrSensorReg16_8(0x558a ,0x01);
			break;
			case degree90:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x00);
				CAM_wrSensorReg16_8(0x5582 ,0x80);
				CAM_wrSensorReg16_8(0x558a ,0x31);
			break;
			case degree120:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x40);
				CAM_wrSensorReg16_8(0x5582 ,0x6f);
				CAM_wrSensorReg16_8(0x558a ,0x31);
			break;
			case degree150:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x01);
				CAM_wrSensorReg16_8(0x5581 ,0x6f);
				CAM_wrSensorReg16_8(0x5582 ,0x40);
				CAM_wrSensorReg16_8(0x558a ,0x31);
			break;
		}
//#endif	
		
}


void CAM_OV5642_set_Special_effects(uint8_t Special_effect)
{
//	#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
		switch(Special_effect)
		{
			case Bluish:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x18);
				CAM_wrSensorReg16_8(0x5585 ,0xa0);
				CAM_wrSensorReg16_8(0x5586 ,0x40);
			break;
			case Greenish:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x18);
				CAM_wrSensorReg16_8(0x5585 ,0x60);
				CAM_wrSensorReg16_8(0x5586 ,0x60);
			break;
			case Reddish:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x18);
				CAM_wrSensorReg16_8(0x5585 ,0x80);
				CAM_wrSensorReg16_8(0x5586 ,0xc0);
			break;
			case BW:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x18);
				CAM_wrSensorReg16_8(0x5585 ,0x80);
				CAM_wrSensorReg16_8(0x5586 ,0x80);
			break;
			case Negative:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x40);
			break;
			
				case Sepia:
				CAM_wrSensorReg16_8(0x5001 ,0xff);
				CAM_wrSensorReg16_8(0x5580 ,0x18);
				CAM_wrSensorReg16_8(0x5585 ,0x40);
				CAM_wrSensorReg16_8(0x5586 ,0xa0);
			break;
			case Normal:
				CAM_wrSensorReg16_8(0x5001 ,0x7f);
				CAM_wrSensorReg16_8(0x5580 ,0x00);		
			break;		
		}
//	#endif
}


void CAM_OV5642_set_Exposure_level(uint8_t level)
{
//	#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
		switch(level)
		{
			case Exposure_17_EV:
			  CAM_wrSensorReg16_8(0x3a0f ,0x10);
				CAM_wrSensorReg16_8(0x3a10 ,0x08);
				CAM_wrSensorReg16_8(0x3a1b ,0x10);
				CAM_wrSensorReg16_8(0x3a1e ,0x08);
				CAM_wrSensorReg16_8(0x3a11 ,0x20);
				CAM_wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_13_EV:
				CAM_wrSensorReg16_8(0x3a0f ,0x18);
				CAM_wrSensorReg16_8(0x3a10 ,0x10);
				CAM_wrSensorReg16_8(0x3a1b ,0x18);
				CAM_wrSensorReg16_8(0x3a1e ,0x10);
				CAM_wrSensorReg16_8(0x3a11 ,0x30);
				CAM_wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_10_EV:
				CAM_wrSensorReg16_8(0x3a0f ,0x20);
				CAM_wrSensorReg16_8(0x3a10 ,0x18);
				CAM_wrSensorReg16_8(0x3a11 ,0x41);
				CAM_wrSensorReg16_8(0x3a1b ,0x20);
				CAM_wrSensorReg16_8(0x3a1e ,0x18);
				CAM_wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_07_EV:
				CAM_wrSensorReg16_8(0x3a0f ,0x28);
				CAM_wrSensorReg16_8(0x3a10 ,0x20);
				CAM_wrSensorReg16_8(0x3a11 ,0x51);
				CAM_wrSensorReg16_8(0x3a1b ,0x28);
				CAM_wrSensorReg16_8(0x3a1e ,0x20);
				CAM_wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_03_EV:
				CAM_wrSensorReg16_8(0x3a0f ,0x30);
				CAM_wrSensorReg16_8(0x3a10 ,0x28);
				CAM_wrSensorReg16_8(0x3a11 ,0x61);
				CAM_wrSensorReg16_8(0x3a1b ,0x30);
				CAM_wrSensorReg16_8(0x3a1e ,0x28);
				CAM_wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_default:
				CAM_wrSensorReg16_8(0x3a0f ,0x38);
				CAM_wrSensorReg16_8(0x3a10 ,0x30);
				CAM_wrSensorReg16_8(0x3a11 ,0x61);
				CAM_wrSensorReg16_8(0x3a1b ,0x38);
				CAM_wrSensorReg16_8(0x3a1e ,0x30);
				CAM_wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure03_EV:
				CAM_wrSensorReg16_8(0x3a0f ,0x40);
				CAM_wrSensorReg16_8(0x3a10 ,0x38);
				CAM_wrSensorReg16_8(0x3a11 ,0x71);
				CAM_wrSensorReg16_8(0x3a1b ,0x40);
				CAM_wrSensorReg16_8(0x3a1e ,0x38);
				CAM_wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure07_EV:
				CAM_wrSensorReg16_8(0x3a0f ,0x48);
				CAM_wrSensorReg16_8(0x3a10 ,0x40);
				CAM_wrSensorReg16_8(0x3a11 ,0x80);
				CAM_wrSensorReg16_8(0x3a1b ,0x48);
				CAM_wrSensorReg16_8(0x3a1e ,0x40);
				CAM_wrSensorReg16_8(0x3a1f ,0x20);
			break;
			case Exposure10_EV:
				CAM_wrSensorReg16_8(0x3a0f ,0x50);
				CAM_wrSensorReg16_8(0x3a10 ,0x48);
				CAM_wrSensorReg16_8(0x3a11 ,0x90);
				CAM_wrSensorReg16_8(0x3a1b ,0x50);
				CAM_wrSensorReg16_8(0x3a1e ,0x48);
				CAM_wrSensorReg16_8(0x3a1f ,0x20);
			break;
			case Exposure13_EV:
				CAM_wrSensorReg16_8(0x3a0f ,0x58);
				CAM_wrSensorReg16_8(0x3a10 ,0x50);
				CAM_wrSensorReg16_8(0x3a11 ,0x91);
				CAM_wrSensorReg16_8(0x3a1b ,0x58);
				CAM_wrSensorReg16_8(0x3a1e ,0x50);
				CAM_wrSensorReg16_8(0x3a1f ,0x20);
			break;
			case Exposure17_EV:
				CAM_wrSensorReg16_8(0x3a0f ,0x60);
				CAM_wrSensorReg16_8(0x3a10 ,0x58);
				CAM_wrSensorReg16_8(0x3a11 ,0xa0);
				CAM_wrSensorReg16_8(0x3a1b ,0x60);
				CAM_wrSensorReg16_8(0x3a1e ,0x58);
				CAM_wrSensorReg16_8(0x3a1f ,0x20);
			break;
		}
//#endif	
}


void CAM_OV5642_set_Sharpness(uint8_t Sharpness)
{
//	#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
		switch(Sharpness)
		{
			case Auto_Sharpness_default:
			CAM_wrSensorReg16_8(0x530A ,0x00);
				CAM_wrSensorReg16_8(0x530c ,0x0 );
				CAM_wrSensorReg16_8(0x530d ,0xc );
				CAM_wrSensorReg16_8(0x5312 ,0x40);
			break;
			case Auto_Sharpness1:
				CAM_wrSensorReg16_8(0x530A ,0x00);
				CAM_wrSensorReg16_8(0x530c ,0x4 );
				CAM_wrSensorReg16_8(0x530d ,0x18);
				CAM_wrSensorReg16_8(0x5312 ,0x20);
			break;
			case Auto_Sharpness2:
				CAM_wrSensorReg16_8(0x530A ,0x00);
				CAM_wrSensorReg16_8(0x530c ,0x8 );
				CAM_wrSensorReg16_8(0x530d ,0x30);
				CAM_wrSensorReg16_8(0x5312 ,0x10);
			break;
			case Manual_Sharpnessoff:
				CAM_wrSensorReg16_8(0x530A ,0x08);
				CAM_wrSensorReg16_8(0x531e ,0x00);
				CAM_wrSensorReg16_8(0x531f ,0x00);
			break;
			case Manual_Sharpness1:
				CAM_wrSensorReg16_8(0x530A ,0x08);
				CAM_wrSensorReg16_8(0x531e ,0x04);
				CAM_wrSensorReg16_8(0x531f ,0x04);
			break;
			case Manual_Sharpness2:
				CAM_wrSensorReg16_8(0x530A ,0x08);
				CAM_wrSensorReg16_8(0x531e ,0x08);
				CAM_wrSensorReg16_8(0x531f ,0x08);
			break;
			case Manual_Sharpness3:
				CAM_wrSensorReg16_8(0x530A ,0x08);
				CAM_wrSensorReg16_8(0x531e ,0x0c);
				CAM_wrSensorReg16_8(0x531f ,0x0c);
			break;
			case Manual_Sharpness4:
				CAM_wrSensorReg16_8(0x530A ,0x08);
				CAM_wrSensorReg16_8(0x531e ,0x0f);
				CAM_wrSensorReg16_8(0x531f ,0x0f);
			break;
			case Manual_Sharpness5:
				CAM_wrSensorReg16_8(0x530A ,0x08);
				CAM_wrSensorReg16_8(0x531e ,0x1f);
				CAM_wrSensorReg16_8(0x531f ,0x1f);
			break;
		}
//#endif
}


void CAM_OV5642_set_Mirror_Flip(uint8_t Mirror_Flip)
{
//	#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
			 uint8_t reg_val;
	switch(Mirror_Flip)
		{
			case MIRROR:
				CAM_rdSensorReg16_8(0x3818,&reg_val);
				reg_val = reg_val|0x00;
				reg_val = reg_val&0x9F;
			CAM_wrSensorReg16_8(0x3818 ,reg_val);
			CAM_rdSensorReg16_8(0x3621,&reg_val);
				reg_val = reg_val|0x20;
				CAM_wrSensorReg16_8(0x3621, reg_val );
			
			break;
			case FLIP:
				CAM_rdSensorReg16_8(0x3818,&reg_val);
				reg_val = reg_val|0x20;
				reg_val = reg_val&0xbF;
			CAM_wrSensorReg16_8(0x3818 ,reg_val);
			CAM_rdSensorReg16_8(0x3621,&reg_val);
				reg_val = reg_val|0x20;
				CAM_wrSensorReg16_8(0x3621, reg_val );
			break;
			case MIRROR_FLIP:
			 CAM_rdSensorReg16_8(0x3818,&reg_val);
				reg_val = reg_val|0x60;
				reg_val = reg_val&0xFF;
			CAM_wrSensorReg16_8(0x3818 ,reg_val);
			CAM_rdSensorReg16_8(0x3621,&reg_val);
				reg_val = reg_val&0xdf;
				CAM_wrSensorReg16_8(0x3621, reg_val );
			break;
			case Normal:
				  CAM_rdSensorReg16_8(0x3818,&reg_val);
				reg_val = reg_val|0x40;
				reg_val = reg_val&0xdF;
			CAM_wrSensorReg16_8(0x3818 ,reg_val);
			CAM_rdSensorReg16_8(0x3621,&reg_val);
				reg_val = reg_val&0xdf;
				CAM_wrSensorReg16_8(0x3621, reg_val );
			break;
		}
//	#endif
}


void CAM_OV5642_set_Compress_quality(uint8_t quality)
{
//#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
	switch(quality)
		{
			case high_quality:
				CAM_wrSensorReg16_8(0x4407, 0x02);
				break;
			case default_quality:
				CAM_wrSensorReg16_8(0x4407, 0x04);
				break;
			case low_quality:
				CAM_wrSensorReg16_8(0x4407, 0x08);
				break;
		}
//#endif
}


void CAM_OV5642_Test_Pattern(uint8_t Pattern)
{
//	#if defined(OV5642_CAM) || defined(OV5642_CAM_BIT_ROTATION_FIXED)|| defined(OV5642_MINI_5MP) || defined (OV5642_MINI_5MP_PLUS)	
	  switch(Pattern)
		{
			case Color_bar:
				CAM_wrSensorReg16_8(0x503d , 0x80);
				CAM_wrSensorReg16_8(0x503e, 0x00);
				break;
			case Color_square:
				CAM_wrSensorReg16_8(0x503d , 0x85);
				CAM_wrSensorReg16_8(0x503e, 0x12);
				break;
			case BW_square:
				CAM_wrSensorReg16_8(0x503d , 0x85);
				CAM_wrSensorReg16_8(0x503e, 0x1a);
				break;
			case DLI:
				CAM_wrSensorReg16_8(0x4741 , 0x4);
				break;
		}
//#endif
}