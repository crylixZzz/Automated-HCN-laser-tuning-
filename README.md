/*Plasma Density Scanner – REAL
  • Driver :  TB6600  (STEP D8, DIR D9, EN D10  – active-LOW)
  • Sensor :  LTC1968  RMS-to-DC  →  Arduino A0  (0–5 V)
  • Limits :  Left  on D2  (INPUT_PULLUP, LOW = hit)
              Right on D3  (INPUT_PULLUP, LOW = hit)
  • 1/16 micro-stepping  →  0.05625° per ustep
  • Serial 115200 commands  A (auto 2000 deg) / S (self assigned deg) / R (reset home 0 deg) / Y (disable motor)*/

  
#include <Arduino.h>

//setup
const byte STEP_PIN  = 8;
const byte DIR_PIN   = 9;
const byte EN_PIN    = 10;
const byte ADC_PIN   = A0;     // LTC1968 OUT
const byte LIM_LEFT  = 2;      // LOW = left  limit hit
const byte LIM_RIGHT = 3;      // LOW = right limit hit

//motor stuff
const float STEP_DEG   = 0.9f;          // full-step
const int   MICROSTEPS = 16;            // TB6600 set to 1/16
const float uSTEP_DEG  = STEP_DEG / MICROSTEPS;   // 0.05625 °
const float DEG_PER_UM = 280.61f;       // 1 µm ⇆ 280.61 °

//fine parameters
const float PROBE_START  = 10.0f;   // first zoom jump
const byte  PROBE_PASSES = 8;
const float GAIN_MIN_V   = 0.001f;  // 1 mV
const byte  GAIN_NOGAIN  = 3;

//globals
float rngDeg       = 0.0f;      // signed sweep span
long  stepAbs      = 0;         // usteps from home
float crestSum     = 0.0f;      // sum crest (for reset)
bool  driverOn     = true;

//limit interrupt flags
volatile bool limitFlag  = false;
volatile bool limitLeft  = false;   // true = left, false = right

//driver helpers
inline void drvOn()  { if (!driverOn) { digitalWrite(EN_PIN, LOW); driverOn = true; } }
inline void drvOff() { digitalWrite(EN_PIN, HIGH); driverOn = false; }

// 1 microstep + extra limit hits
void ustep(bool cw)
{
  drvOn();
  digitalWrite(DIR_PIN, cw ? HIGH : LOW);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, HIGH); delayMicroseconds(450);
  digitalWrite(STEP_PIN, LOW ); delayMicroseconds(450);
  stepAbs += cw ? 1 : -1;
}

//ADC helper from LTC
inline float readVrms()
{
  int a = analogRead(ADC_PIN);          // 0–1023
  return (a * 5.0f) / 1023.0f;          // V
}

//limit interrupting 
void ISR_left()  { limitFlag = true; limitLeft = true;  }
void ISR_right() { limitFlag = true; limitLeft = false; }

//stop n retreat 2000 deg
void retreatLimit()
{
  Serial.println(limitLeft ?
    F("[LIMIT] Left hit – retreat 2000° CW") :
    F("[LIMIT] Right hit – retreat 2000° CCW"));

  bool cw = limitLeft;                          // left->CW, right->CCW
  long back = lround(2000.0f / uSTEP_DEG);
  for (long i = 0; i < back; ++i) ustep(cw);

  limitFlag = false;            // clear flag
}

//FINETUNE
void fine(float crestDeg)
{
  Serial.println(F("--- Fine Start ---"));
  const float span  = fabs(rngDeg);

  float bestDeg = crestDeg;
  float bestV   = readVrms();
  const float halfV = bestV * 0.5f;

  /* walk left to trough */
  while (!limitFlag)
  {
    ustep(false); bestDeg -= uSTEP_DEG;
    float v = readVrms();
    Serial.print(F("Fine L ")); Serial.print(bestDeg,3);
    Serial.print(F("°  "));     Serial.println(v,4);
    if (v <= halfV || fabs(bestDeg) > span) break;
  }
  if (limitFlag) { retreatLimit(); return; }

  //return to crest 
  long crestIdx = lround(fabs(crestDeg) / uSTEP_DEG);
  while (!limitFlag && stepAbs != crestIdx) ustep(stepAbs < crestIdx);
  if (limitFlag) { retreatLimit(); return; }

  /capture right trough
  float right = crestDeg;
  while (!limitFlag)
  {
    ustep(true);  right += uSTEP_DEG;
    float v = readVrms();
    Serial.print(F("Fine R ")); Serial.print(right,3);
    Serial.print(F("°  "));     Serial.println(v,4);
    if (v <= halfV || fabs(right) > span) break;
  }
  if (limitFlag) { retreatLimit(); return; }

  //zoom search
  float jump = PROBE_START; byte noGain = 0;
  while (noGain < GAIN_NOGAIN && jump > uSTEP_DEG*0.5f && !limitFlag)
  {
    bool gained = false;
    for (int s=-1; s<=1; s+=2)
    {
      float probe = crestDeg + s*jump;
      if (fabs(probe) > span) continue;
      long tgt = lround(fabs(probe) / uSTEP_DEG);
      while (!limitFlag && stepAbs != tgt) ustep(stepAbs < tgt);
      if (limitFlag) { retreatLimit(); return; }
      float v = readVrms();
      Serial.print(F("Fine probe ")); Serial.print(probe,4);
      Serial.print(F("°  "));         Serial.println(v,4);
      if (v > bestV + GAIN_MIN_V) { bestV = v; bestDeg = probe; gained = true; }
    }
    if (gained) noGain = 0; else ++noGain;
    jump *= 0.5f;
  }

  float um = fabs(bestDeg) / DEG_PER_UM;
  Serial.println(F("--- Fine Done ---"));
  Serial.print(F("Crest ")); Serial.print(bestV,4);
  Serial.print(F(" V @ "));  Serial.print(bestDeg,4);
  Serial.print(F("° ("));    Serial.print(um,4); Serial.println(F(" µm)"));

  crestSum += bestDeg;
}

//COARSE SCAN
void coarse()
{
  Serial.println(F("--- Coarse Start ---"));
  const bool cw    = (rngDeg > 0.0f);
  const long total = lround(fabs(rngDeg) / uSTEP_DEG);

  float prevV = -1.0f, peakV = -1e9f, peakDeg = 0.0f;

  for (long i=1; i<=total && !limitFlag; ++i)
  {
    ustep(cw);
    float dMag = i * uSTEP_DEG;
    float dSgn = cw ? dMag : -dMag;
    float vRMS = readVrms();
    if (vRMS > 0.5f && fabs(vRMS - prevV) > 0.001f) {
      Serial.print(F("Deg ")); Serial.print(dSgn,3);
      Serial.print(F("°  "));  Serial.println(vRMS,4);
      prevV = vRMS;
    }
    if (vRMS > peakV) { peakV = vRMS; peakDeg = dSgn; }
  }

  if (limitFlag) { retreatLimit(); return; }

  Serial.print(F("--- Coarse Done ---\nPeak "));
  Serial.print(peakV,4); Serial.print(F(" V @ "));
  Serial.print(peakDeg,3); Serial.println(F("°"));

  //back 2 peak
  float retreat = rngDeg - peakDeg;
  Serial.print(F("Retreat ")); Serial.print(retreat,3); Serial.println(F("°"));

  bool backCW = (retreat < 0.0f);
  long rSteps = lround(fabs(retreat) / uSTEP_DEG);
  for (long i=0; i<rSteps && !limitFlag; ++i) ustep(backCW);

  if (limitFlag) { retreatLimit(); return; }

  fine(peakDeg);
}

//reset home 0 deg
void resetHome()
{
  if (crestSum == 0.0f) { Serial.println(F("Already at home.")); return; }

  Serial.print(F("Reset ")); Serial.print(crestSum,3); Serial.println(F("°"));
  bool backCW = (crestSum < 0.0f);
  long steps  = lround(fabs(crestSum) / uSTEP_DEG);
  for (long i=0; i<steps; ++i) ustep(backCW);

  crestSum = 0.0f; stepAbs = 0;
  Serial.println(F("Home."));
}

//setup + loop
void setup()
{
  /* pins */
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(EN_PIN,   OUTPUT);
  digitalWrite(EN_PIN, LOW);          // coils on
  pinMode(LIM_LEFT,  INPUT_PULLUP);
  pinMode(LIM_RIGHT, INPUT_PULLUP);

  /* limit interrupts */
  attachInterrupt(digitalPinToInterrupt(LIM_LEFT),  ISR_left,  FALLING);
  attachInterrupt(digitalPinToInterrupt(LIM_RIGHT), ISR_right, FALLING);

  Serial.begin(115200);
  Serial.println(F("[Real] Ready – A (2000 deg cw) / S (enter degrees) / R (reset home) / Y (disable motor)"));
}

void loop()
{
  if (limitFlag) { retreatLimit(); return; }    // catch limit even when idle

  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();

  
  if (cmd == "Y") { drvOff(); Serial.println(F("Driver off")); return; }
  if (cmd == "R") { resetHome(); return; }

  //scan cmds
  if (cmd.equalsIgnoreCase("A"))      rngDeg = 2000.0f;
  else if (cmd.equalsIgnoreCase("S")) {
    Serial.println(F("Degrees?")); while (!Serial.available());
    rngDeg = Serial.parseFloat(); while (Serial.available()) Serial.read();
    if (rngDeg == 0.0f) rngDeg = 2000.0f;
  }
  else { Serial.println(F("Bad cmd")); return; }

  drvOn();
  stepAbs  = 0;
  limitFlag = false;
  coarse();
}


