
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1015 ads;

float dataRate = 50.0;
float _previousDataTime = 0;

// Pin connected to the ALERT/RDY signal for new sample notification.
constexpr int READY_PIN = 23;

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif


volatile bool new_data = false;
void IRAM_ATTR NewDataReadyISR() {
  new_data = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(300000UL);

  if (!ads.begin(0x48)) {
    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }

  pinMode(READY_PIN, INPUT);
  // We get a falling edge every time a new sample is ready.
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);

  ads.setGain(GAIN_FOUR);
  // Start continuous conversions.
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
  //ads.setDataRate(RATE_ADS1115_475SPS);
  //ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);
}

void loop() {

  // If we don't have new data, skip this iteration.
  if (new_data) {

    int16_t results = ads.getLastConversionResults();

    Serial.print("Differential: ");
    Serial.print(results);
    Serial.print("(");
    Serial.print(ads.computeVolts(results));
    Serial.println("V)");

    new_data = false;
  }

  // put your main code here, to run repeatedly:
  if (millis() - _previousDataTime >= ((1.0 / dataRate) * 1000)) {
    _previousDataTime = millis();
    Serial.println((ads.readADC_SingleEnded(0) * 0.1875) / 1.0);
  }
}
