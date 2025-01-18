#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>

#define SCL_PIN 9 //On C3 mini, On old board SCL=22
#define SDA_PIN 8 //On C3 mini. On old board SDA=21

/*IO*/
int LED_PIN = 3;
int BTN_PIN = 10;

/* DATA OUTPUT*/
float dataRate = 60.0;
float _previousDataTime = 0;

/*ABSOLUTE ENCODER*/
Adafruit_ADS1015 ads;

int ADS_READY_PIN = 2;

float YAxisValue = 0.0f;
float XAxisValue = 0.0f;


// TABLE RELATIVE ENCODER
int TABLE_ENCODER_A_PIN = 0;
int TABLE_ENCODER_B_PIN = 1;

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

volatile bool mustADSReadData = false;

void IRAM_ATTR OnAdsNewData() {
  mustADSReadData = true;
}

  static uint8_t lastA = 0;
  static uint8_t lastB = 0;

volatile bool checkTableEncoder;

void IRAM_ATTR OnTAbleEncoderAData() {
  checkTableEncoder = true;
}

void ReadADSData() {
  XAxisValue = (ads.readADC_SingleEnded(0) * 0.125) / 1.0;
  YAxisValue = (ads.readADC_SingleEnded(1) * 0.125) / 1.0;

  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);

  mustADSReadData = false;
}

void ComputeTableSpeed() {
  if( checkTableEncoder) {
  // Lecture de l'état actuel de l'encodeur
    encoderState = (digitalRead(TABLE_ENCODER_A_PIN) << 1) | digitalRead(TABLE_ENCODER_B_PIN);

    // Si l'état a changé
    if (encoderState != lastEncoderState) {
      int encoded = (lastEncoderState << 2) | encoderState;

      switch (encoded) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
          direction = 1; // Sens horaire
          pulseCount++;
          break;
        case 0b0010:
        case 0b0100:
        case 0b1101:
        case 0b1011:
          direction = -1; // Sens antihoraire
          pulseCount++;
          break;
        default:
          // Transition invalide, ignorer
          break;
      }
      lastEncoderState = encoderState;
    }
    checkTableEncoder = false;
  }

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
  Serial.print(">TableSpeed:");
  Serial.println(rpm);

  Serial.print(">TableDirection:");
  Serial.println(direction);
  
  Serial.print(">YAxis:");
  Serial.println(YAxisValue);
  
  Serial.print(">XAxis:");
  Serial.println(XAxisValue);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("[GF] Initializing ...");
  Serial.println("[I2C] SDA : " + String(SDA_PIN));
  Serial.println("[I2C] SCL : " + String(SCL_PIN));

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000UL);

  Serial.println("[IO] LED : " + String(LED_PIN));
  Serial.println("[IO] BTN : " + String(BTN_PIN));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(BTN_PIN, INPUT);

  if (!ads.begin(0x48)) {
    
    while (1) {
      Serial.println("[GF] Failed to initialize ADS.");
      digitalWrite(LED_PIN, HIGH);
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }

  //ADS1115
  pinMode(ADS_READY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ADS_READY_PIN), OnAdsNewData, FALLING);

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_475SPS); //RATE_ADS1115_860SPS
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);

  //TABLE ENCODER
  pinMode(TABLE_ENCODER_A_PIN, INPUT);
  pinMode(TABLE_ENCODER_B_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(TABLE_ENCODER_A_PIN), OnTAbleEncoderAData, RISING);
  //attachInterrupt(digitalPinToInterrupt(TABLE_ENCODER_B_PIN), OnTableEncoderData, RISING);

 // lastEncoderState = (digitalRead(TABLE_ENCODER_A_PIN) << 1) | digitalRead(TABLE_ENCODER_B_PIN);

  digitalWrite(LED_PIN, LOW);

  Serial.println("[GF] Done.");
}

void loop() {


  ComputeTableSpeed();
  

  if(digitalRead(BTN_PIN) == LOW) {
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    digitalWrite(LED_PIN, LOW);
  }

  if(mustADSReadData) {
    ReadADSData();
    mustADSReadData = false;
  }

  if (millis() - _previousDataTime >= ((1.0 / dataRate) * 1000)) {
    _previousDataTime = millis();

    SendData();
  }
}
