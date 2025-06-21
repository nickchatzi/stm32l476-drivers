#ifndef LCD_H
#define LCD_H

#include "gpio_driver.h"
#include "spi_driver.h"
#include "stm32l476.h"

uint8_t current_page;
uint8_t current_column;

void LCD_Select(void);
void LCD_Unselect(void);
void lcd_reset(void);
void lcd_send_command(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init();
void lcd_set_page(uint8_t page);
void lcd_set_column(uint8_t column);
void lcd_write_char(char c);
void lcd_write_char_big2x(char c);
void lcd_write_char_big3x(char c);
void lcd_set_black_background(void);
extern const uint8_t font5x8[][5];

#endif