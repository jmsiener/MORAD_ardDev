#include <Arduino.h>
#include <MR_SPI.h>
//#include <driver/spi_master.h>
#include <SPI.h>


MR_SPI::MR_SPI() {
  
}



void IRAM_ATTR MR_SPI::CVout(int outChannel, unsigned outValue) {
  // CVout - send a 12 bit value to the DAC
  // there are 2 DACs, each with 2 channels of output
  // channels 0,1 on first chip, 2,3 on 2nd chip
  // channel = channel number 0-3
  // value - 12 bit DAC value to write
  //if (outChannel &2) gpio_set_level(dac1_cs,0);
  //else gpio_set_level(dac0_cs, 0);
  if (outChannel &2) digitalWrite(DAC1_CS, LOW); // assert correct chip select
  else digitalWrite(DAC0_CS, LOW);  
  byte data = outValue >> 8;  // get to 8 bits of DAC value
  data = data & B00001111; // mask off 4 msbs which are control
  if (outChannel &1) data = data | B10110000; //2nd DAC channel, 2.048v mode
  else data = data | B00110000; //1st DAC channel, 2.048v mode
  xSemaphoreTake(SPIhandle, portMAX_DELAY);
  SPI.transfer(data); // send  control bits and 4 MSBs of DAC value
  data = outValue;
  SPI.transfer(data);  // send low 8 bits of DAC value
  xSemaphoreGive(SPIhandle);
  if (outChannel &2) digitalWrite(DAC1_CS, HIGH); // assert correct chip select
  else digitalWrite(DAC0_CS, HIGH);  
  //if (outChannel &2) gpio_set_level(dac1_cs,1);
  //else gpio_set_level(dac0_cs, 1);
}

unsigned short IRAM_ATTR MR_SPI::CVin(int inChannel) {
  // CVin - read MCP3204 12 bit ADC 
  // input voltage of -5v = 0, +5V in = 4095
  // channel 0-3
  uint8_t cmd=0x6 | inChannel >>2; // start bit, single ended mode
  uint16_t cmd2 = (inChannel << 14); // high 2 bits of channel number
  //gpio_set_level(adc0_cs,0);
  //noInterrupts();
  digitalWrite(ADC_CS, LOW);  
  xSemaphoreTake(SPIhandle, portMAX_DELAY);
  SPI.transfer(cmd);
  int inValue =SPI.transfer16(cmd2) & 0xfff;
  xSemaphoreGive(SPIhandle);
  //gpio_set_level(adc0_cs,1);
  digitalWrite(ADC_CS, HIGH); 
  //interrupts();
  inValue=-((inValue-4096)+1); // invert the result
  return inValue;
}

unsigned MIDInote_to_DACvalue(byte notenumber, int cvnumber) {
  // convert MIDI note number to a DAC value
  // conversion is DAC specific because each one has a slightly different scale and offset
  return (unsigned)(notenumber-LOWEST_NOTE)*(float)NOTE_STEP*(float)1/*cvout[cvnumber].scale*/;
}

void MR_SPI::init() {
  Serial.println("Initializing the SPI bus");
  pinMode(DAC0_CS, OUTPUT);
  pinMode(DAC1_CS, OUTPUT);
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(DAC0_CS, HIGH);
  digitalWrite(DAC1_CS, HIGH);
  digitalWrite(ADC_CS, HIGH);
  
  SPI.begin(SCLK,MISO,MOSI,DAC0_CS); // we actually use a CS pin for each DAC
  SPI.setBitOrder(MSBFIRST);
  SPI.setFrequency(1000000); // ADC max clock 2 MHz; settling on 1 MHz
  SPIhandle = xSemaphoreCreateMutex();


  Serial.println("DONE");
}