#include "driver/pulse_cnt.h"
// -------------------- ENCODER --------------------
const int pinA = 15;   // canal A del encoder
const int pinB = 2;   // canal B del encoder
pcnt_unit_handle_t pcnt_unit = NULL;
pcnt_channel_handle_t pcnt_chan = NULL;
// Parámetros del PWM
const int pwmFreq = 20000;      // 20 kHz
const int pwmResolution = 10;   // 10 bits → 0 a 1023
const int pwmPin = 18;           // Pin PWM

// Timer
hw_timer_t *timer = NULL;

long posNow = 0;

void IRAM_ATTR onTimer() {
  int count = 0;
  pcnt_unit_get_count(pcnt_unit, &count);
  posNow = count;
  pcnt_unit_clear_count(pcnt_unit);
}

void setup() {
  //Configuracion de pines
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  //PWM
  ledcAttach(pwmPin, pwmFreq, pwmResolution);
  ledcWrite(pwmPin, 700);   // 50% duty

  // Timer
  timer = timerBegin(1000000);        // timer a 1 MHz (1 tick = 1 µs)
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 100000, true, 0); // 100000 µs = 100 ms

  //Modulo Serial
  Serial.begin(115200);

  //Modulo CNT
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
    .edge_gpio_num = pinA,   // A genera los eventos de conteo
    .level_gpio_num = pinB,  // B define el sentido
  };
  err = pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);
  if (err != ESP_OK) {
    Serial.println("Error creando PCNT channel");
    while (true);
  }
  // Contar solo flanco de subida de A
  err = pcnt_channel_set_edge_action(
    pcnt_chan,
    PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // subida de A
    PCNT_CHANNEL_EDGE_ACTION_HOLD       // bajada de A
  );
  if (err != ESP_OK) {
    Serial.println("Error en edge_action");
    while (true);
  }
  // B decide si se mantiene o se invierte el signo
  err = pcnt_channel_set_level_action(
    pcnt_chan,
    PCNT_CHANNEL_LEVEL_ACTION_KEEP,     // si B = 0  -> +1
    PCNT_CHANNEL_LEVEL_ACTION_INVERSE   // si B = 1  -> -1
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
}

void loop() {
  delay(100);
  Serial.print("Posicion: ");
  Serial.println(posNow);
}
