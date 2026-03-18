#include "driver/pulse_cnt.h"
#include <math.h>
#include <SPI.h>

SPIClass AMT_spi(HSPI);

// -------------------- SPI AMT203S-V --------------------
#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS1  10
// Control
const float kp = 1023.0/360.0*5.0;
float setPoint = 0.0;
float setPointVel = 50.0;
float anguloMotor = 0.0;
float velocidadMotor = 0.0;
float u_motor = 0.0;
// -------------------- ENCODER --------------------
const int pinA = 15;
const int pinB = 2;

pcnt_unit_handle_t pcnt_unit = NULL;
pcnt_channel_handle_t pcnt_chan = NULL;

const int CPR = 1980;   // x2

// -------------------- PWM --------------------
const int pwmFreq = 20000;
const int pwmResolution = 10;
const int pwmPin1 = 18;//Aumenta angulo
const int pwmPin2 = 17;//Disminuye angulo

// -------------------- TIMER --------------------
hw_timer_t *timer = NULL;
const float Ts = 0.02;

volatile long posNow = 0;
volatile bool hayDato = false;

void IRAM_ATTR onTimer() {
  int count = 0;
  pcnt_unit_get_count(pcnt_unit, &count);
  //posNow = count;
  float dtheta = (count*360.0)/CPR;
  anguloMotor += dtheta;
  if(anguloMotor>360.0){
    anguloMotor = anguloMotor - 360.0;
  }else if(anguloMotor<0){
    anguloMotor = 360.0+anguloMotor;
  }
  velocidadMotor = (count * 60.0f) / (CPR * Ts);
  pcnt_unit_clear_count(pcnt_unit);
  //*
  float errorPos = setPoint-anguloMotor;
  if(errorPos>180.0){
    errorPos -= 360.0;
  }else if(errorPos<-180.0){
    errorPos += 360.0;
  }
  float errorVel = setPointVel-velocidadMotor;
  //u_motor = kp*(errorPos);
  if(velocidadMotor < setPointVel){
    u_motor += 10;
  }else if(velocidadMotor > setPointVel){
    u_motor -= 10;
  }
  if(u_motor>0){
    if(u_motor>1023.0){
      u_motor = 1023.0;
    }
    ledcWrite(pwmPin2, 0);
    delayMicroseconds(5); 
    ledcWrite(pwmPin1, (int)u_motor);
  }else if(u_motor<0){
    u_motor = -u_motor;
    if(u_motor>1023.0){
      u_motor = 1023.0;
    }
    ledcWrite(pwmPin1, 0);
    delayMicroseconds(5);
    ledcWrite(pwmPin2, (int)u_motor);
  }else{
    ledcWrite(pwmPin1, 0);
    ledcWrite(pwmPin2, 0);
  }
  //*/
  if((errorVel<2)&&(errorVel>-2)){
    hayDato = false;
  }else{
    hayDato = true;
  }
}

void ejecutarPrueba() {
  timerStop(timer);
  timerWrite(timer, 0);
  pcnt_unit_clear_count(pcnt_unit);
  hayDato = false;

  Serial.print("[");

  ledcWrite(pwmPin1, 700);
  timerStart(timer);

  for (int i = 0; i < 50; i++) {
    while (!hayDato);
    hayDato = false;

    long counts = posNow;
    float rpm = (counts * 60.0f) / (CPR * Ts);

    Serial.print(rpm, 4);

    if (i < 49) {
      Serial.print(",");
    }
  }

  Serial.println("];");

  ledcWrite(pwmPin1, 0);
  timerStop(timer);
}

// -------------------- AMT203S-V --------------------
uint8_t SPI_T(uint8_t msg)
{
    uint8_t resp;

    AMT_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    digitalWrite(PIN_CS1, LOW);
    resp = AMT_spi.transfer(msg);
    delayMicroseconds(5);
    digitalWrite(PIN_CS1, HIGH);

    delayMicroseconds(100);

    AMT_spi.endTransaction();

    return resp;
}

uint16_t readAMT203()
{
    uint8_t recieved = 0xA5;
    uint8_t temp[2];
    uint16_t ABSposition = 0;

    SPI_T(0x10);

    recieved = SPI_T(0x00);

    while(recieved != 0x10)
    {
        recieved = SPI_T(0x00);
    }

    temp[0] = SPI_T(0x00);
    temp[1] = SPI_T(0x00);

    temp[0] &= ~0xF0;

    ABSposition = temp[0] << 8;
    ABSposition += temp[1];

    return ABSposition;
}

float readAngleDeg()
{
    uint16_t pos = readAMT203();
    return (pos * 360.0f) / 4096.0f;
}

void setup() {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  ledcAttach(pwmPin1, pwmFreq, pwmResolution);
  ledcWrite(pwmPin1, 0);
  ledcAttach(pwmPin2, pwmFreq, pwmResolution);
  ledcWrite(pwmPin2, 0);

  Serial.begin(115200);
  delay(1000);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 20000, true, 0);

  pcnt_unit_config_t unit_config = {
    .low_limit = -32768,
    .high_limit = 32767,
  };

  esp_err_t err = pcnt_new_unit(&unit_config, &pcnt_unit);
  if (err != ESP_OK) {
    Serial.println("Error creando PCNT unit");
    while (true);
  }

  pcnt_chan_config_t chan_config = {
    .edge_gpio_num = pinA,
    .level_gpio_num = pinB,
  };

  err = pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);
  if (err != ESP_OK) {
    Serial.println("Error creando PCNT channel");
    while (true);
  }

  err = pcnt_channel_set_edge_action(
    pcnt_chan,
    PCNT_CHANNEL_EDGE_ACTION_INCREASE,
    PCNT_CHANNEL_EDGE_ACTION_DECREASE
  );
  if (err != ESP_OK) {
    Serial.println("Error en edge_action");
    while (true);
  }

  err = pcnt_channel_set_level_action(
    pcnt_chan,
    PCNT_CHANNEL_LEVEL_ACTION_KEEP,
    PCNT_CHANNEL_LEVEL_ACTION_INVERSE
  );
  if (err != ESP_OK) {
    Serial.println("Error en level_action");
    while (true);
  }

  err = pcnt_unit_enable(pcnt_unit);
  if (err != ESP_OK) {
    Serial.println("Error habilitando PCNT");
    while (true);
  }

  err = pcnt_unit_clear_count(pcnt_unit);
  if (err != ESP_OK) {
    Serial.println("Error limpiando PCNT");
    while (true);
  }

  err = pcnt_unit_start(pcnt_unit);
  if (err != ESP_OK) {
    Serial.println("Error iniciando PCNT");
    while (true);
  }

  // SPI
  AMT_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS1);

  pinMode(PIN_CS1, OUTPUT);
  digitalWrite(PIN_CS1, HIGH);

  timerWrite(timer, 0);
  timerStart(timer);

  //ledcWrite(pwmPin2, 300);
  anguloMotor = readAngleDeg();
  setPoint = anguloMotor + 180.0;
  if(setPoint>360.0){
    setPoint = setPoint - 360.0;
  }else if(setPoint<0){
    setPoint = 360.0+setPoint;
  }
  Serial.println("Sistema listo");
}

void loop() {
  if(hayDato){
    /*
    Serial.print("Ang abs: ");
    Serial.print(readAngleDeg(), 4);
    Serial.print(" | Ang puls: ");
    Serial.println(anguloMotor, 4);
    */
    Serial.print("Setpoint: ");
    Serial.print(setPointVel, 4);
    Serial.print(" RPM | Vel: ");
    Serial.println(velocidadMotor, 4);
    hayDato = false;
  }
  /*
  delay(100);
  long counts = posNow;
  float rpm = (counts * 60.0f) / (CPR * Ts);
  float angleDeg = readAngleDeg();
  Serial.print("RPM: ");
  Serial.print(rpm, 4);
  Serial.print(" | Angulo abs: ");
  Serial.println(angleDeg, 4);
  */
}