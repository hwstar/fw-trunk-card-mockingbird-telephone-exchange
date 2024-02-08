#include "common.h"
#include "enm.h"


#define TAG "main"

uint8_t I2c_address = 0x30;
HardwareTimer TickTimer(TIM4);
ENM::ENM Enm;
LOGGING::Logging Log;

/*
* 5 Millisecond Tick Timer Callback
*/

void TickTimer_Callback() {
  // We are of type Slic (for now)
  Enm.service();
}

/*
* I2C Request Callback
*/

void i2c_request() {
  Enm.i2c_request();
}

/*
* I2C receive callback
*/

void i2c_receive(int howMany) {
  Enm.i2c_receive(howMany);
}

void setup() {
  // Use external 8MHz crystal
  enableClock(HSE_CLOCK);

  // Timer initialization
  TickTimer.setMode(1, TIMER_DISABLED);
  TickTimer.setPrescaleFactor(72); // 72 MHz clock
  TickTimer.setOverflow(5000); // 5 Millisecond interrupts
  TickTimer.attachInterrupt(TickTimer_Callback);
  TickTimer.resume();

  // Serial port
  Serial.setRx(PB7);
  Serial.setTx(PB6);
  Serial.begin(115200);
  
  // Configuration strapping pins
  pinMode(CFG0N, INPUT_PULLUP);
  pinMode(CFG1N, INPUT_PULLUP);
  pinMode(CFG2N, INPUT_PULLUP);

  // I2C address pins
  pinMode(SW1N, INPUT_PULLUP);
  pinMode(SW2N, INPUT_PULLUP);
  pinMode(SW3N, INPUT_PULLUP);

  // Test points
  pinMode(TP401, OUTPUT);
  pinMode(TP402, OUTPUT);
  pinMode(TP403, OUTPUT);

  // Test LED
  pinMode(LEDN_TEST, OUTPUT);
  digitalWrite(LEDN_TEST, HIGH); // LED off


  // We are of type SLIC for now.
  Enm.setup();

  // Determine I2c address
  I2c_address |= !digitalRead(SW1N);
  I2c_address |= ((!digitalRead(SW2N)) << 1);
  I2c_address |= ((!digitalRead(SW3N)) << 2);

  // Initialize I2C
  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.begin(I2c_address);

  

  Wire.onRequest(i2c_request);
  Wire.onReceive(i2c_receive);
  
}




void loop() 
{
  Log.loop();
  Enm.loop();
}
