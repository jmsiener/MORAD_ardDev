#ifndef MR_DAC_H
#define MR_DAC_H

#include <Arduino.h>
//#include <driver/spi_master.h>

//SPI Bus
#define MISO 12
#define MOSI 13
#define SCLK 14

// DAC chip select pins
#define DAC0_CS 33 
#define DAC1_CS 32

// DAC uses 2.048v vref and is amplified 3x ie 6.144V full scale 
// the DACs/amps are not perfect and really should be calibrated in software
// 6v is 6 octaves so each note step is 4000/(6*12)=55.5555 counts. if we use 56 there is some error but its OK for a test
#define DAC_BITS 12
#define DAC_RANGE 4096
#define NOTE_STEP 55.5555
#define LOWEST_NOTE 24 // make C1 the lowest MIDI note ie 0V output
#define HIGHEST_NOTE 96 // highest MIDI note C7 ie 6V output
#define NUM_CHANNELS 4

// ADC chip select pin
#define ADC_CS 15



//static spi_device_handle_t spi = NULL;

class MR_SPI {
	public:
        MR_SPI(); 
        void init();
        unsigned short IRAM_ATTR CVin(int inChannel);
        void IRAM_ATTR CVout(int outChannel, unsigned outValue);
        unsigned MIDInote_to_DACvalue(byte notenumber, int cvnumber);
        void SampleCV(void * pvParameters);
        
        //volatile int CV0 = 0;
        //volatile int CV1 = 0;
        //volatile int CV2 = 0;
        //volatile int CV3 = 0;

        //~MR_SPI();

    //protected:
        //unsigned MIDInote_to_DACvalue(byte notenumber, int cvnumber);
    private:
        uint8_t outChannel;
        uint8_t inChannel;
        uint16_t outValue;
        uint16_t inValue;   
        SemaphoreHandle_t SPIhandle = NULL;
        
};

#endif  // MOTIVATION_RADIO_DAC_H_ 

//NULL