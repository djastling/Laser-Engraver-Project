#ifndef I2C_LCD_H_
#define I2C_LCD_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void lcd_write_nibble(uint8_t nibble, uint8_t rs);

void lcd_send_cmd(uint8_t cmd);

void lcd_send_data(uint8_t data);

void lcd_init();

void lcd_write_string(char *str);

void lcd_set_cursor(uint8_t row, uint8_t column);

void lcd_clear(void);

void lcd_backlight(uint8_t state);

#endif
