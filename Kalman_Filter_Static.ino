// ARDUINO MEGA/UNO/NANO

#define HCSR04_Trig_Pin   PB1  // HC - SR04 Trigger Pin Port B Bit 1 (Arduino Nano/Uno D9)
#define HCSR04_Echo_Pin   PB2  // HC - SR04 Echo Pin Port B Bit 2 Arduino Nano/Uno D10)
#define Temp_HCSR04       25   // ambient temp 25C

float f_Time_of_Echo    = 0;
float f_Speed_of_Sound  = 0;
float f_Distance_mm     = 0;
uint32_t u32_Prev_Time  = 0;
float f_Q_ProcessNoice  = 0.75;

///////////////////////////////////////////////////////////////////////
const float f_KF_MeasurementError        = 5.0;  // Ölçümün standart sapması / Ölçüm hatası

/* R Measurement Noise Variance (Ölçüm Gürültüsü Varyansı) Sensörün gürültülü olup olmamasını belirtir.
    R büyürse: sensör çok kötüdür -> K küçülür -> filtre modele daha çok güvenir
    R küçülürse: sensör çok iyidir -> K büyür → sensör baskın olur
  Ya da datasheet’ten:
  ±0.5°C hata → R ≈ 0.25
  ±2°C hata → R ≈ 4
  ±0.1°C hata → R ≈ 0.01    */
float f_KF_MeasurementErrorVariance_R    = 0;  // r = sensör gürültüsü   f_KF_MeasurementErrorVariance_R = f_KF_MeasurementError*f_KF_MeasurementError;

/* K, ölçüme mi yoksa modele mi daha çok güvenmen gerektiğini belirler.
   Eğer sensör gürültülü ise (R büyük):
    - K küçük çıkar
    - Modelin tahmini daha baskın olur
   Eğer model belirsiz ise (P büyük):
    - K büyük çıkar
    - Ölçüm daha baskın olur */
float f_KF_KalmanGain                    = 0;  // Kalman Gain (Kazanç)

/* P, ölçüm işlendikten sonra kalan belirsizlik. Ne kadar güvendiğini temsil eder.
  - Küçülürse → “daha eminim”
  - Büyürse → tahmin “daha güvensiz”  */

float f_KF_CurrentEstimatedVariance_P00  = 0;  // p = tahmin belirsizliği, posterior error covariance (hata kovaryans matrisi – ölçüm sonrası)
float f_KF_NextEstimatedVariance_P10     = 0;  // prior covariance (ölçümden önceki belirsizlik) Covariance Predict (P) / tahmin belirsizliği, Tahmin belirsizleştikçe sensörün etkisi artar
float f_KF_CurrentEstimatedValue_X00     = 0;  // Filtrelenmiş tahmin (posterior state estimate)
float f_KF_NextEstimatedValue_X10        = 0;  // predicted state (tahmin – prior) ölçüm gelmeden önceki tahmin
float f_KF_HumanEstimationError          = 100.0;  // ilk tahmin için öngörülen hata


///////////////////////////////////////////////////////////////////////
void setup() {
  // HC-SR04 GPIO INIT
  DDRB &= (~(1 << HCSR04_Echo_Pin));    // 2. bit of portB is false and input for HCSR04_Echo_Pin
  DDRB |= (1 << HCSR04_Trig_Pin);       // 1. bit of portB is true and output for HCSR04_Trig_Pin
  PORTB &= (~(1 << HCSR04_Trig_Pin));      // set HCSR04_Trig_Pin low
  Serial.begin(9600);

  ///////////////////////////////////////////////////////////////////////
  // KF Setup

  f_KF_CurrentEstimatedVariance_P00 = f_KF_HumanEstimationError * f_KF_HumanEstimationError;
  f_KF_NextEstimatedValue_X10       = f_KF_CurrentEstimatedValue_X00;
  f_KF_NextEstimatedVariance_P10    = f_KF_CurrentEstimatedVariance_P00;
  // f_KF_MeasurementErrorVariance_R   = f_KF_MeasurementError * f_KF_MeasurementError;
  // f_KF_CurrentEstimatedValue_X00    = 600.0;
  f_KF_MeasurementErrorVariance_R = Calc_f_KF_MeasurementErrorVariance_R(100);

  ///////////////////////////////////////////////////////////////////////
}

void loop() {

  if (millis() - u32_Prev_Time >= 60) { // getting distance cycle time 60ms
    f_Distance_mm = FB_Get_Distance_mm();

    Serial.print("Distance:");
    Serial.print(f_Distance_mm);
    Serial.print("\t\tKF_Distance:");
    Serial.println(f_KF_CurrentEstimatedValue_X00);

    // UPDATE
    f_KF_KalmanGain = (f_KF_NextEstimatedVariance_P10) / (f_KF_NextEstimatedVariance_P10 + f_KF_MeasurementErrorVariance_R);
    f_KF_CurrentEstimatedValue_X00  = f_KF_NextEstimatedValue_X10 + (f_KF_KalmanGain * (f_Distance_mm - f_KF_NextEstimatedValue_X10));
    f_KF_CurrentEstimatedVariance_P00 = (1 - f_KF_KalmanGain) * f_KF_NextEstimatedVariance_P10;

    // ESTIMATE
    f_KF_NextEstimatedValue_X10  = f_KF_CurrentEstimatedValue_X00;
    f_KF_NextEstimatedVariance_P10  = f_KF_CurrentEstimatedVariance_P00 + f_Q_ProcessNoice;

    u32_Prev_Time = millis();
  }

}

//////////////////////////////////////////////////////////////////////
// GET DISTANCE
float FB_Get_Distance_mm() {
  PORTB |= (1 << HCSR04_Trig_Pin);         // set HCSR04_Trig_Pin high
  delayMicroseconds(10);
  PORTB &= (~(1 << HCSR04_Trig_Pin));      // set HCSR04_Trig_Pin low
  cli();                                  // Turn off global interrupt
  float f_Time_of_Echo = pulseIn(10, HIGH, 23529.4); // max sensor dist ~4m
  sei();                                  // set Global Interrupt Enable
  f_Speed_of_Sound = 331.5 + (0.6 * Temp_HCSR04);
  // Serial.print("Dist:");
  // Serial.println(((f_Time_of_Echo / 2) * f_Speed_of_Sound) / 1000);
  // we suggest to use over 60ms measurement cycle, in order to prevent trigger signal to the echo signal.
  return ((f_Time_of_Echo / 2) * f_Speed_of_Sound) / 1000;
}
//////////////////////////////////////////////////////////////////////




float Calc_f_KF_MeasurementErrorVariance_R(uint8_t u8_sampleSize) {
  float f_Sum_of_Distances = 0;
  float f_Avg_of_Distances = 0;
  float f_Sum_of_Deviation = 0;
  float f_Raw_Distance_Values = 0;
  float f_Arr_Raw_Distance_Values[255];
  uint8_t counter = 0;

  f_Raw_Distance_Values = FB_Get_Distance_mm();

  while (counter < u8_sampleSize) {
    f_Raw_Distance_Values = FB_Get_Distance_mm();
    delay(60);
    f_Arr_Raw_Distance_Values[counter] = f_Raw_Distance_Values;
    f_Sum_of_Distances = f_Sum_of_Distances + f_Raw_Distance_Values;
    /*
        Serial.print("f_Arr_Raw_Distance_Values:");
        Serial.print(f_Arr_Raw_Distance_Values[counter]);
        Serial.print("\t\tf_Sum_of_Distances:");
        Serial.println(f_Sum_of_Distances);
    */
    counter++;
  }
  counter = 0;

  f_Avg_of_Distances = f_Sum_of_Distances / u8_sampleSize;  // okunan tüm değerlerin ortalamasını alıyoruz
  /*
    Serial.print("f_Sum_of_Distances:");
    Serial.println(f_Sum_of_Distances);
    Serial.print("f_Avg_of_Distances:");
    Serial.println(f_Avg_of_Distances);
  */
  while (counter < u8_sampleSize) {
    f_Sum_of_Deviation = f_Sum_of_Deviation + (sq(f_Avg_of_Distances - f_Arr_Raw_Distance_Values[counter]));
    /*
        Serial.print("Sapma:");
        Serial.println((f_Avg_of_Distances - f_Arr_Raw_Distance_Values[counter]));
    */
    counter++;
  }
  counter = 0;
  /*
    Serial.print("f_Sum_of_Deviation:");
    Serial.println(f_Sum_of_Deviation);
    Serial.print("Deviation:");
    Serial.println(f_Sum_of_Deviation / u8_sampleSize);

  */

  f_KF_CurrentEstimatedValue_X00 = f_Avg_of_Distances;
  /*
    Serial.print("f_KF_CurrentEstimatedValue_X00:");
    Serial.println(f_KF_CurrentEstimatedValue_X00);
  */
  return f_Sum_of_Deviation / u8_sampleSize;
}
