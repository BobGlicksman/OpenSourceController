/* http://www.b2cqshop.com Example Software Sketch
LCD Display Blue: I2C/TWI Interface
B2cqshop@gmail.com
*/

#include <Wire.h> 

#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27,16,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display


void setup()

{

  delay(500);

  lcd.init();                      // initialize the lcd 

 

  // Print a message to the LCD.

  lcd.backlight();

  lcd.setCursor(0, 0);

  delay(100);

  lcd.setCursor(2, 0);

  lcd.print("Welcome ");

  lcd.setCursor(0, 1);

  lcd.print("WWW.B2CQSHOP.COM");

  lcd.setCursor(2, 2);

  lcd.print("Hello, world ");

  lcd.setCursor(0, 3);

  lcd.print("Good for Arduino ");

}


void loop()

{

}
