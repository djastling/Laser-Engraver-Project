#include "i2c-lcd.h"
#include <stdio.h>

#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit
#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD

uint8_t backlight_state = 1;

void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
 uint8_t data = nibble << D4_BIT;
 data |= rs << RS_BIT;
 data |= backlight_state << BL_BIT; // Include backlight state in data
 data |= 1 << EN_BIT;
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
 HAL_Delay(1);
 data &= ~(1 << EN_BIT);
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
}

HAL_StatusTypeDef lcd_write_byte(uint8_t byte){
	char data_upper, data_lower;
	uint8_t data[4];
	data_upper = (byte&0xf0);
	data_lower = ((byte<<4)&0xf0);
	data[0] = data_upper|0x0D;  //en=1, rs=0 -> bxxxx1101
	data[1] = data_upper|0x09;  //en=0, rs=0 -> bxxxx1001
	data[2] = data_lower|0x0D;  //en=1, rs=0 -> bxxxx1101
	data[3] = data_lower|0x09;  //en=0, rs=0 -> bxxxx1001
	return HAL_I2C_Master_Transmit (&hi2c1, I2C_ADDR << 1, (uint8_t *) data, 4, 100);
}
void lcd_send_cmd(uint8_t cmd) {
 uint8_t upper_nibble = cmd >> 4;
 uint8_t lower_nibble = cmd & 0x0F;
 lcd_write_nibble(upper_nibble, 0);
 lcd_write_nibble(lower_nibble, 0);
 if (cmd == 0x01 || cmd == 0x02) {
 HAL_Delay(2);
 }
}
void lcd_send_data(uint8_t data) {
 uint8_t upper_nibble = data >> 4;
 uint8_t lower_nibble = data & 0x0F;
 lcd_write_nibble(upper_nibble, 1);
 lcd_write_nibble(lower_nibble, 1);
}

void lcd_init() {
 HAL_Delay(50);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(5);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(1);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(1);
 lcd_write_nibble(0x02, 0);
 lcd_send_cmd(0x28);
 lcd_send_cmd(0x0C);
 lcd_send_cmd(0x06);
 lcd_send_cmd(0x01);
 HAL_Delay(2);
}
void lcd_write_string(char *str) {
	 while (*str) {
		 if (*str == '\n' || *str == '\r'){
			 str++;
		 }
		 else{
			 if (lcd_write_byte((uint8_t)*str++) == HAL_ERROR){
				 printf("Failed to Transmit over I2C\n\r");
				 return;		// Return Without Continuing, if unable to transmit
			 }
		 }
	 }
	 return;
}
void lcd_set_cursor(uint8_t row, uint8_t column) {
 uint8_t address;
 switch (row) {
 case 0:
 address = 0x00;
 break;
 case 1:
 address = 0x40;
 break;
 default:
 address = 0x00;
 }
 address += column;
 lcd_send_cmd(0x80 | address);
}
void lcd_clear(void) {
lcd_send_cmd(0x01);
 HAL_Delay(2);
}
void lcd_backlight(uint8_t state) {
 if (state) {
 backlight_state = 1;
 } else {
 backlight_state = 0;
 }
}
