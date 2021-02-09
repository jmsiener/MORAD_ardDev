#include <Arduino.h>
//#include <freertos/FreeRTOS.h>
//#include <freertos/task.h>
//#include <freertos/queue.h>
//#include <MORAD_io.h>
#include <MR_GPIO.h>
//#include <MR_UI.h>
//#include <MR_UIevents.h>

//static xQueueHandle ui_event_queue = NULL;

MR_GPIO::MR_GPIO() {
    /*
    Serial.println("Doing pinMode stuff");
    pinMode(GATEout_0, OUTPUT);
    pinMode(GATEout_1, OUTPUT);
    pinMode(GATEout_2, OUTPUT);
    pinMode(GATEout_3, OUTPUT);
    
    pinMode(GATEin_0, INPUT);
    pinMode(GATEin_0, INPUT);
    pinMode(GATEin_0, INPUT);
    pinMode(GATEin_0, INPUT);

    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    pinMode(ENC_SW, INPUT);
    */
}

void MR_GPIO::Init() {
    
    Serial.println("Initializing the GPIO");
    Serial.println("Doing pinMode stuff");
    pinMode(GATEout_0, OUTPUT);
    pinMode(GATEout_1, OUTPUT);
    pinMode(GATEout_2, OUTPUT);
    pinMode(GATEout_3, OUTPUT);
    
    pinMode(GATEin_0, INPUT);
    pinMode(GATEin_1, INPUT);
    pinMode(GATEin_2, INPUT);
    pinMode(GATEin_3, INPUT);

    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    pinMode(ENC_SW, INPUT);
    
    Serial.println("Attach Interrupts");
    //attachInterrupt(ENC_A, gpio_isr_handler_encoder1, RISING);
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //gpio_isr_handler_add(ENC_A_t, gpio_isr_handler_encoder1, (void*) ENC_A_t);
    //gpio_isr_handler_add(ENC_B_t, gpio_isr_handler_encoder1, (void*) ENC_B_t);
    //attachInterrupt(ENC_B, ISR, Mode);
    //attachInterrupt(ENC_SW, ISR, Mode);
    Serial.println("DONE");
}

void IRAM_ATTR MR_GPIO::gpio_isr_handler_encoder1(void* arg) {
    //ui_ev_ts_t ev;
    //MR_UI mr_ui;
    uint32_t pin = (uint32_t) arg;
    uint32_t pv = (uint32_t) digitalRead(pin);
    uint32_t a = digitalRead(ENC_A);
    uint32_t b = digitalRead(ENC_B);
    static uint32_t check = 0;
    // de bounce
    if(pv) return;
    if(pin == ENC_B && a){
        check = ENC_B;
        return;
    }
    if(pin == ENC_A && b){
        check = ENC_A;
        return;
    }
    // real event
    if(check == ENC_A && !a && !b){
        //ev.event = EV_ENC1_BWD;
        //xQueueSendFromISR(ui_event_queue, &ev, NULL);
    }
    if(check == ENC_B && !a && !b){
        //ev.event = EV_ENC1_FWD;
        //xQueueSendFromISR(ui_event_queue, &ev, NULL);
    }
    check = 0;
    return;
}

void IRAM_ATTR MR_GPIO::gpio_isr_handler_encoder1_btn1(void* arg) {
    //ui_ev_ts_t ev;
    //static ui_ev_t state = EV_ENC1_BT_UP;
    //uint32_t pv = digitalRead(ENC_SW);
    // de-bounce
    //if(pv == 0 && state == EV_ENC1_BT_DWN) return;
    //if(pv == 1 && state == EV_ENC1_BT_UP) return;
    // real event
    //if(pv == 1 && state == EV_ENC1_BT_DWN) ev.event = EV_ENC1_BT_UP;
    //if(pv == 0 && state == EV_ENC1_BT_UP) ev.event = EV_ENC1_BT_DWN;
    //state = ev.event;
    //xQueueSendFromISR(ui_event_queue, &ev, NULL);
}

void IRAM_ATTR MR_GPIO::GATEout(int outChannel, bool outValue) {
  switch (outChannel) {
    case 0:
       digitalWrite(GATEout_0,outValue); // 
       break;
    case 1:
       digitalWrite(GATEout_1,outValue); // 
       break;
    case 2:
       digitalWrite(GATEout_2,outValue); // 
       break;
    case 3:
       digitalWrite(GATEout_3,outValue); // 
       break;
  }
}

void IRAM_ATTR MR_GPIO::GATEin(int inChannel, bool inBool) {
    
}

