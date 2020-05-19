#include <Wire.h>

#define I2CLCD_ADDR 0x3e  
byte lcd_contrast = 35;  

void i2clcd_cmd(byte x) {
  Wire.beginTransmission(I2CLCD_ADDR);
  Wire.write((uint8_t)0b00000000); // CO = 0,RS = 0
  Wire.write(x);
  Wire.endTransmission();
}
   
void i2clcd_clear() {
  i2clcd_cmd(0b00000001);
}
 
void i2clcd_contdata(byte x) {
  Wire.write(0b11000000); // CO = 1, RS = 1
  Wire.write(x);
}
 
void i2clcd_lastdata(byte x) {
  Wire.write(0b01000000); // CO = 0, RS = 1
  Wire.write(x);
}
 
void i2clcd_printStr(const char *s) {
  Wire.beginTransmission(I2CLCD_ADDR);
  while (*s) {
    if (*(s + 1)) {
      i2clcd_contdata(*s);
    } else {
      i2clcd_lastdata(*s);
    }
    s++;
  }
  Wire.endTransmission();
}
 
void i2clcd_setCursor(byte x, byte y) {
  i2clcd_cmd(0x80 | (y * 0x40 + x));
}
 
void i2clcd_printInt(int num) {
    char int2str[10];
    String str(num);    
    str.getBytes((unsigned char*)int2str, 10);
    i2clcd_printStr(int2str);
}

void i2clcd_printfloat(float num) {
    char float2str[10];
    String str1((int)num);
    String str2((int)((num-(int)num)*100));
    str1 += '.' + str2;
    str1.getBytes((unsigned char*)(float2str), 10);
    float2str[str1.length()] = '\0';
    i2clcd_printStr(float2str);
}

void i2clcd_begin() {
  Wire.begin();
  i2clcd_cmd(0b00111000); // function set
  i2clcd_cmd(0b00111001); // function set
  i2clcd_cmd(0b00000100); // EntryModeSet
  i2clcd_cmd(0b00010100); // interval osc
  i2clcd_cmd(0b01110000 | (lcd_contrast & 0xF)); // lcd_contrast Low
  i2clcd_cmd(0b01011100 | ((lcd_contrast >> 4) & 0x3)); // contast High/icon/power
  i2clcd_cmd(0b01101100); // follower control
  delay(200);
  i2clcd_cmd(0b00111000); // function set
  i2clcd_cmd(0b00001100); // Display On
  i2clcd_cmd(0b00000001); // Clear Display
  delay(2);
}
