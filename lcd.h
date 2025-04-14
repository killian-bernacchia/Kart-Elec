#if !defined(LCD_H)
#define LCD_H

#include <LiquidCrystal_I2C.h>
#include <stdint.h>

class Lcd : public LiquidCrystal_I2C {
public:
  Lcd(uint8_t address, int columns, int rows);
  void init();
  void printBar(float ratio);
};

#endif // LCD_H
