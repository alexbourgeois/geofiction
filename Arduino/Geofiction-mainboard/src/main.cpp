#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>



float dataRate = 60.0;
float _previousDataTime = 0;

/*ABSOLUTE ENCODER*/
Adafruit_ADS1015 ads;

int ADS_READY_PIN = 23;

float YAxisValue = 0.0f;
float XAxisValue = 0.0f;


// TABLE RELATIVE ENCODER
int TABLE_ENCODER_A_PIN = 32;
int TABLE_ENCODER_B_PIN = 33;

//Encoder variables
byte encoderState = 0;
byte lastEncoderState = 0;
int direction = 0;
long pulseCount = 0;
const int countsPerRevolution = 1600; // 400 PPR x 4 transitions en quadrature

// Table RPM computation
unsigned long lastTime = 0;
float rpm = 0.0;
const unsigned long samplingInterval = 125; // Intervalle d'échantillonnage vitesse en ms

bool mustADSReadData = false;

void IRAM_ATTR OnAdsNewData() {
  mustADSReadData = true;
}

void IRAM_ATTR OnTableEncoderData() {
  encoderState = (digitalRead(TABLE_ENCODER_A_PIN) << 1) | digitalRead(TABLE_ENCODER_B_PIN);
  
  // Déterminer la direction en fonction de la séquence
  switch (lastEncoderState) {
    case 0: // 00
      if (encoderState == 2) direction = 1;
      if (encoderState == 1) direction = -1;
      break;
    case 1: // 01
      if (encoderState == 0) direction = 1;
      if (encoderState == 3) direction = -1;
      break;
    case 2: // 10
      if (encoderState == 3) direction = 1;
      if (encoderState == 0) direction = -1;
      break;
    case 3: // 11
      if (encoderState == 1) direction = 1;
      if (encoderState == 2) direction = -1;
      break;
  }

  pulseCount += direction;
  lastEncoderState = encoderState;
}

void ReadADSData() {
  XAxisValue = (ads.readADC_SingleEnded(0) * 0.125) / 1.0;
  YAxisValue = (ads.readADC_SingleEnded(1) * 0.125) / 1.0;

  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);

  mustADSReadData = false;
}

void ComputeTableSpeed() {

  // Calcul du RPM à intervalles réguliers
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  if (deltaTime >= samplingInterval) {
    float revolutions = pulseCount / (float)countsPerRevolution;
    float minutes = deltaTime / 60000.0; // Conversion des ms en minutes
    rpm = revolutions / minutes;

    // Réinitialisation du compteur et du temps
    pulseCount = 0;
    lastTime = currentTime;
  }
}


void SendData() {
  Serial.print(">Table:");
  Serial.println(rpm);
  
  Serial.print(">YAxis:");
  Serial.println(YAxisValue);
  
  Serial.print(">XAxis:");
  Serial.println(XAxisValue);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000UL);

  if (!ads.begin(0x48)) {
    
    while (1) {
      Serial.println("Failed to initialize ADS.");
      delay(1500);
    }
  }

  //ADS1115
  pinMode(ADS_READY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ADS_READY_PIN), OnAdsNewData, FALLING);

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_475SPS); //RATE_ADS1115_860SPS
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);

  //TABLE ENCODER
  pinMode(TABLE_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(TABLE_ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(TABLE_ENCODER_A_PIN), OnTableEncoderData, RISING);
  //attachInterrupt(digitalPinToInterrupt(TABLE_ENCODER_B_PIN), OnTableEncoderData, RISING);
  lastEncoderState = (digitalRead(TABLE_ENCODER_A_PIN) << 1) | digitalRead(TABLE_ENCODER_B_PIN);
}

void loop() {

  ComputeTableSpeed();

  if(mustADSReadData) {
    ReadADSData();
    mustADSReadData = false;
  }

  if (millis() - _previousDataTime >= ((1.0 / dataRate) * 1000)) {
    _previousDataTime = millis();

    SendData();
  }
}
