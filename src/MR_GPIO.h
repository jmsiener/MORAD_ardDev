#ifndef MR_GPIO_H
#define MR_GPIO_H

#include <Arduino.h>
//#include <MR_UI.h>

#define     ENC_SW  19
#ifndef CONFIG_SWAP_ENCODER_PINS
    #define ENC_A   5
    #define ENC_B   18
#else 
    #define ENC_B   5
    #define ENC_A   18
#endif

// GATE output pins
#define GATEout_0 25
#define GATEout_1 26
#define GATEout_2 4
#define GATEout_3 27

// GATE input pins 
#define GATEin_0 36
#define GATEin_1 39
#define GATEin_2 34
#define GATEin_3 35

// encoder pins
#define ENC_A_t   GPIO_NUM_5
#define ENC_B_t   GPIO_NUM_18
#define ENC_SW_t  GPIO_NUM_19

#define ESP_INTR_FLAG_DEFAULT 0

class MR_GPIO {
    public:
        MR_GPIO();
        void Init();
        static void IRAM_ATTR gpio_isr_handler_encoder1(void* arg);
        void IRAM_ATTR gpio_isr_handler_encoder1_btn1(void* arg);
        void GATEout(int outChannel, bool outBool);
        void GATEin(int inChannel, bool inBool);
        //xQueueHandle ui_event_queue = NULL;
        //~MR_GPIO();
    protected:
        static uint32_t check;
    private:
        uint32_t pin;
        uint32_t arg;
        int32_t pv;
        uint32_t a;
        uint32_t b;
        uint8_t outChannel;
        uint8_t inChannel;
        bool outValue;
        bool inValue;
        
        
};

#endif //MR_GPIO_H 