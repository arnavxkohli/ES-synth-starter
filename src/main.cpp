#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <STM32FreeRTOS.h>
#include <cstddef>
#include <string>
#include <cstring>
#include "Knob.h"
#include <ES_CAN.h>

//Constants
const uint32_t interval = 100; //Display update interval


//Pin definitions
//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

const uint32_t stepSizes [] = {51149155, 54077542, 57396381, 60715219, 64424509,
68133799, 72233540, 76528508, 81018701, 85899345, 90975216, 96441538};

const char* notes[] = {"C", "C#", "D", "D#", "E", "F",
                       "F#", "G", "G#", "A", "A#", "B"};

volatile uint32_t currentStepSize = 0;

struct {
  std::bitset<32> inputs;
  const char* notePlayed = "Play a note...";
  SemaphoreHandle_t mutex;
  uint8_t RX_Message[8]={0};
} sysState;

Knob Knob3;
QueueHandle_t msgInQ;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

std::bitset<4> readCols(){
  std::bitset<4> result;

  result[0] = !digitalRead(C0_PIN);
  result[1] = !digitalRead(C1_PIN);
  result[2] = !digitalRead(C2_PIN);
  result[3] = !digitalRead(C3_PIN);

  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  std::bitset<3> rowBits = std::bitset<3>(rowIdx);

  digitalWrite(RA0_PIN, rowBits[0]);
  digitalWrite(RA1_PIN, rowBits[1]);
  digitalWrite(RA2_PIN, rowBits[2]);

  digitalWrite(REN_PIN, HIGH);
}

void sampleISR(){
  static uint32_t phaseAcc = 0;
  uint32_t localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  uint8_t localRotation = Knob3.getRotationISR();

  phaseAcc += localCurrentStepSize;

  int32_t Vout = (phaseAcc >> 24) - 128;

  // localRotation used for volume before or after the 128?
  Vout = Vout >> (8 - localRotation);

  analogWrite(OUTR_PIN, Vout + 128);
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  std::bitset<32> previousInputs;
  uint8_t TX_Message[8] = {0};
  TX_Message[1] = 4; // Octave to be changed later - try auto assigning this.

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    uint32_t localCurrentStepSize = 0;

    /*
    You can potentially define a local inputs variable, which would carry the state of the keyboard matrix and also be used to play the note. The lock can then be acquired at the end of this function, just to assign sysState.inputs the value of your local array using memcpy or std::copy(). The semaphore is not used yet, because inputs is never read by the displayUpdate thread.
    */

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    std::bitset<32> localInputs = sysState.inputs;
    const char* localNotePlayed = sysState.notePlayed;
    xSemaphoreGive(sysState.mutex);

    for(int row = 0; row < 4; row++){
      setRow(row);
      delayMicroseconds(3);
      for (int bit = 0; bit < 4; ++bit) {
          localInputs[row * 4 + bit] = readCols()[bit];
      }
    }

    Knob3.updateRotation(std::to_string(localInputs[13]) + std::to_string(localInputs[12]));

    for(int i = 0; i < 12; i++){
      if(localInputs[i]){
        localCurrentStepSize = stepSizes[i];
        localNotePlayed = notes[i];
        TX_Message[2] = i;
        if(!previousInputs[i]){
          TX_Message[0] = 'P';
        }
      }
      else if (previousInputs[i]) {
        TX_Message[0] = 'R';
      }
    }

    CAN_TX(0x123, TX_Message);

    previousInputs = localInputs;

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = localInputs;
    sysState.notePlayed = localNotePlayed;
    xSemaphoreGive(sysState.mutex);

    // ------- UNCOMMENT THIS LATER -------
    //__atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    // ------- UNCOMMENT THIS LATER -------
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t ID;

  while(1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    //Update display
    u8g2.clearBuffer();         // clear the internal memory

    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2, 10, "Music Synth");  // write something to the internal memory
    u8g2.setCursor(2, 20);

    u8g2.print(Knob3.getRotation());

    // xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    // u8g2.drawStr(2, 30, sysState.notePlayed);
    // xSemaphoreGive(sysState.mutex);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.setCursor(66,30);
    u8g2.print((char) sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    u8g2.print(sysState.RX_Message[2]);
    xSemaphoreGive(sysState.mutex);

    u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void* pvParameters){
  uint8_t local_RX[8];
  uint32_t localCurrentStepSize = 0;

  while(1){
    xQueueReceive(msgInQ, local_RX, portMAX_DELAY);
    if(local_RX[0] == 'R'){
      localCurrentStepSize = 0;
    } else if(local_RX[0] == 'P') {
      localCurrentStepSize = (stepSizes[local_RX[2]] << local_RX[1]) >> 4;
    }

    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy(sysState.RX_Message, local_RX, sizeof(local_RX));
    xSemaphoreGive(sysState.mutex);
  }
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);

  msgInQ = xQueueCreate(36,8);

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_Start();

  TaskHandle_t scanKeysHandle = NULL;
  TaskHandle_t displayUpdateHandle = NULL;
  TaskHandle_t decodeTaskHandle = NULL;

  xTaskCreate(
    displayUpdateTask, /* Function that implements the task */
    "displayUpdate", /* Text name for the task */
    256, /* Stack size in words, not bytes */
    NULL, /* Parameter passed into the task */
    2, /* Task priority */
    &displayUpdateHandle /* Pointer to store the task handle */
  );

  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &scanKeysHandle /* Pointer to store the task handle */
  );

  xTaskCreate(
    decodeTask,
    "decoreTask",
    64,
    NULL,
    1,
    &decodeTaskHandle
  );

  sysState.mutex = xSemaphoreCreateMutex();

  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
}