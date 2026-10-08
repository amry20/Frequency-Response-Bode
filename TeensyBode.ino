/*
  TeensyBodeUI.ino  -  Teensy 4.1 impedance analyzer firmware for the Python UI (teensy_bode_ui.py)

  Same hardware and measurement method as TeensyBode.ino:
    Pin 33 (PWM) -> R1 1k -> [C1 1.5n] -> R2 1k -> node A [C2 1.5n] -> RS 1k -> node B -> DUT -> GND
    node A -> R3 4.7k -> pin 15 (+C3 100p)  -> ADC1
    node B -> R4 4.7k -> pin 16 (+C4 100p)  -> ADC2

  Board: Teensy 4.1, USB Type: Serial.  Close the Arduino Serial Monitor/Plotter before
  starting the Python UI (only one program can use the port at a time).

  Text protocol (one command per line, replies are single lines unless noted):
    PING                                   -> OK TEENSYBODE 3
    DDSTICKS <n>                           -> OK DDSTICKS <n> <rate_Hz>   (duty-update period in 24 MHz ticks, default 64 = 375 kHz)
    DITHER <cycles>                        -> OK DITHER <cycles>          (random extra delay per sample, 0-3000 CPU cycles, default 600)
    RS <ohms>                              -> OK RS <ohms>
    MEAS <f> <settle_ms> <meas_ms>         -> M <f> <|Z|> <phase_deg> <Re> <Im> <VppA> <VppB> <dcA> <dcB> <minA> <maxA> <minB> <maxB> <n>
    WAVE <f> <bins>                        -> W <same fields as M>, then <bins> lines "D <A_V> <B_V>", then WEND
    SWEEP <f0> <f1> <step> <settle> <meas> -> P <f> <|Z|> <phase_deg> <Re> <Im>  (one per point), then SEND
                                              (any byte received during a sweep aborts it -> SABORT)
    OPENCAL                                -> CAL <C_pF> <G2>      (DUT disconnected!)
    CLEARCAL                               -> OK CLEARCAL
    IDLE                                   -> OK IDLE   (sets the output to mid-scale, no signal)
  Errors: ERR <text>
*/

#include <ADC.h>
#include <IntervalTimer.h>
#include <math.h>

// ======================= HARDWARE SETTINGS =======================
const int    PWM_PIN = 33;
const int    PIN_A   = 15;          // node A -> ADC1
const int    PIN_B   = 16;          // node B -> ADC2
double       RS      = 1000.0;      // sense resistor (can be changed from the UI)
const double R_ADC   = 4700.0;      // R4
const double C_ADC   = 100e-12;     // C4
const float  SINE_AMPL = 0.45f;     // fraction of full scale (max 0.5)
// ==================================================================

const double VREF = 3.3, ADC_MAX = 4095.0;
const int    PWM_BITS = 7;
const double PWM_FREQ = 1171875.0;
// Duty-cycle update clock: the PIT runs at 24 MHz. 64 ticks = 375 kHz. With the 1,171.875 kHz PWM
// carrier, 1171.875/375 = 25/8, so every DDS/PWM beat product is a multiple of 46.875 kHz -
// none fall inside the 20-40 kHz measurement band (the old 400 kHz rate put one at 28.125 kHz).
uint32_t     ddsTicks = 64;
double       DDS_RATE = 24e6 / 64;
// Random extra delay after each ADC sample (CPU cycles at 600 MHz). Breaks any lock-step between
// the sampling and the DDS interrupt so that aliased spurs average out instead of adding up.
uint32_t     ditherCycles = 600;
const int    MAX_BINS = 200;

// ---------------- types (above all functions for the Arduino prototype generator) ----------------
struct Cpx { double re, im; };
struct Meas {
  Cpx A, B;
  double dcA, dcB;
  int minA, maxA, minB, maxB;
  uint32_t n;
  bool ok;
};

ADC *adc = new ADC();
IntervalTimer ddsTimer;

// ---------------- DDS ----------------
static uint8_t sineTable[1024];
static uint8_t midCode = 63;
volatile uint32_t phaseAcc = 0, phaseInc = 0;
volatile bool outputOn = true;

void ddsISR() {
  phaseAcc += phaseInc;
  analogWrite(PWM_PIN, outputOn ? sineTable[phaseAcc >> 22] : midCode);
}

double setFrequency(double f) {
  uint32_t inc = (uint32_t)llround(f * 4294967296.0 / DDS_RATE);
  noInterrupts();
  phaseInc = inc;
  outputOn = true;
  interrupts();
  return (double)inc * DDS_RATE / 4294967296.0;
}

// (re)start the duty-cycle update timer with an exact number of 24 MHz PIT ticks
void startDDS(uint32_t ticks) {
  double f = (DDS_RATE > 0 && phaseInc) ? (double)phaseInc * DDS_RATE / 4294967296.0 : 30000.0;
  ddsTimer.end();
  ddsTicks = ticks;
  DDS_RATE = 24e6 / ticks;
  setFrequency(f);
  ddsTimer.begin(ddsISR, (float)ticks / 24.0f);   // IntervalTimer loads 24*period-0.5 -> ticks-1 => period = ticks
  ddsTimer.priority(32);
}

// ---------------- complex helpers ----------------
static inline Cpx csub(Cpx a, Cpx b) { return {a.re - b.re, a.im - b.im}; }
static inline Cpx cdiv(Cpx a, Cpx b) {
  double d = b.re*b.re + b.im*b.im;
  return {(a.re*b.re + a.im*b.im)/d, (a.im*b.re - a.re*b.im)/d};
}
static inline Cpx cscale(Cpx a, double s) { return {a.re*s, a.im*s}; }
static inline double cmag(Cpx a) { return sqrt(a.re*a.re + a.im*a.im); }
static inline double cphaseDeg(Cpx a) { return atan2(a.im, a.re) * 180.0 / M_PI; }

// ---------------- acquisition (simultaneous dual-ADC + least-squares sine fit) ----------------
uint32_t chanA = 0, chanB = 0;

static double det3(const double m[3][3]) {
  return m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
       - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
       + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);
}
static bool solve3(const double M[3][3], const double y[3], double x[3]) {
  double d = det3(M);
  if (fabs(d) < 1e-9) return false;
  for (int i = 0; i < 3; i++) {
    double T[3][3];
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) T[r][c] = (c == i) ? y[r] : M[r][c];
    x[i] = det3(T) / d;
  }
  return true;
}

Meas acquire(double f, uint32_t ms, float *binA, float *binB, uint32_t *binN, int nbins) {
  const double TWO_PI_D = 2.0 * M_PI;
  const double wcyc = TWO_PI_D * f / (double)F_CPU_ACTUAL;
  const uint32_t dur = (uint32_t)((uint64_t)ms * F_CPU_ACTUAL / 1000);

  double Scc=0, Sss=0, Scs=0, Sc=0, Ss=0, N=0;
  double SAc=0, SAs=0, SA=0, SBc=0, SBs=0, SB=0;
  Meas m; m.minA = m.minB = 99999; m.maxA = m.maxB = -1; m.ok = false;

  uint32_t t0 = ARM_DWT_CYCCNT;
  uint32_t k = 0;
  while (true) {
    __disable_irq();
    uint32_t t = ARM_DWT_CYCCNT;
    if (k & 1) { ADC2_HC0 = chanB; ADC1_HC0 = chanA; }
    else       { ADC1_HC0 = chanA; ADC2_HC0 = chanB; }
    __enable_irq();

    uint32_t dt = t - t0;
    if (dt > dur) {
      while (!(ADC1_HS & ADC_HS_COCO0) || !(ADC2_HS & ADC_HS_COCO0)) ;
      (void)ADC1_R0; (void)ADC2_R0;
      break;
    }
    double ph = fmod((double)dt * wcyc, TWO_PI_D);
    float c = cosf((float)ph), s = sinf((float)ph);

    while (!(ADC1_HS & ADC_HS_COCO0) || !(ADC2_HS & ADC_HS_COCO0)) ;
    int a = ADC1_R0;
    int b = ADC2_R0;
    if (ditherCycles) {                       // xorshift32 random wait
      static uint32_t rng = 2463534242u;
      rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
      uint32_t w = rng % ditherCycles, tw = ARM_DWT_CYCCNT;
      while (ARM_DWT_CYCCNT - tw < w) ;
    }

    Scc += c*c; Sss += s*s; Scs += c*s; Sc += c; Ss += s; N += 1;
    SAc += a*c; SAs += a*s; SA += a;
    SBc += b*c; SBs += b*s; SB += b;
    if (a < m.minA) m.minA = a;
    if (a > m.maxA) m.maxA = a;
    if (b < m.minB) m.minB = b;
    if (b > m.maxB) m.maxB = b;

    if (nbins > 0) {
      int bi = (int)(ph / TWO_PI_D * nbins);
      if (bi >= nbins) bi = nbins - 1;
      binA[bi] += a; binB[bi] += b; binN[bi]++;
    }
    k++;
  }

  m.n = (uint32_t)N;
  double M[3][3] = {{Scc, Scs, Sc}, {Scs, Sss, Ss}, {Sc, Ss, N}};
  double yA[3] = {SAc, SAs, SA}, yB[3] = {SBc, SBs, SB}, xA[3], xB[3];
  if (N > 10 && solve3(M, yA, xA) && solve3(M, yB, xB)) {
    m.A = {xA[0], -xA[1]};  m.dcA = xA[2];
    m.B = {xB[0], -xB[1]};  m.dcB = xB[2];
    m.ok = true;
  }
  return m;
}

// ---------------- impedance ----------------
bool   openCal = false;
double calC = 0, calG2 = 0;

Cpx Yshunt(double f) {
  double w = 2.0 * M_PI * f;
  if (openCal) return {calG2 * w * w, w * calC};
  Cpx Zb = {R_ADC, -1.0 / (w * C_ADC)};
  return cdiv({1, 0}, Zb);
}
Cpx rawZ(const Meas &m) {
  Cpx I = cscale(csub(m.A, m.B), 1.0 / RS);
  return cdiv(m.B, I);
}
Cpx computeZ(const Meas &m, double f) {
  Cpx Y = csub(cdiv({1, 0}, rawZ(m)), Yshunt(f));
  return cdiv({1, 0}, Y);
}

// ---------------- reply helpers ----------------
void printMeasFields(double f, const Meas &m) {
  const double k = VREF / ADC_MAX;
  Cpx Z = computeZ(m, f);
  Serial.printf("%.3f %.6g %.4f %.6g %.6g %.5f %.5f %.5f %.5f %d %d %d %d %lu\n",
                f, cmag(Z), cphaseDeg(Z), Z.re, Z.im,
                2 * cmag(m.A) * k, 2 * cmag(m.B) * k, m.dcA * k, m.dcB * k,
                m.minA, m.maxA, m.minB, m.maxB, (unsigned long)m.n);
}

// ---------------- commands ----------------
void cmdMeas(double f, uint32_t settle, uint32_t meas) {
  double fa = setFrequency(f);
  delay(settle);
  Meas m = acquire(fa, meas, nullptr, nullptr, nullptr, 0);
  if (!m.ok) { Serial.println("ERR fit failed"); return; }
  Serial.print("M "); printMeasFields(fa, m);
}

void cmdWave(double f, int bins) {
  static float binA[MAX_BINS], binB[MAX_BINS];
  static uint32_t binN[MAX_BINS];
  if (bins < 8) bins = 8;
  if (bins > MAX_BINS) bins = MAX_BINS;
  for (int i = 0; i < bins; i++) { binA[i] = binB[i] = 0; binN[i] = 0; }
  double fa = setFrequency(f);
  delay(80);
  Meas m = acquire(fa, 60, binA, binB, binN, bins);
  if (!m.ok) { Serial.println("ERR fit failed"); return; }
  Serial.print("W "); printMeasFields(fa, m);
  const double k = VREF / ADC_MAX;
  for (int i = 0; i < bins; i++) {
    if (binN[i] == 0) Serial.println("D nan nan");
    else Serial.printf("D %.5f %.5f\n", binA[i] / binN[i] * k, binB[i] / binN[i] * k);
  }
  Serial.println("WEND");
}

void cmdSweep(double f0, double f1, double step, uint32_t settle, uint32_t meas) {
  if (step <= 0 || f1 < f0 || f0 < 100 || f1 > 150000) { Serial.println("ERR bad sweep range"); return; }
  setFrequency(f0);
  delay(200);
  int nmax = (int)((f1 - f0) / step + 1.5);
  for (int i = 0; i < nmax; i++) {
    if (Serial.available()) { Serial.println("SABORT"); return; }
    double fa = setFrequency(f0 + i * step);
    delay(settle);
    Meas m = acquire(fa, meas, nullptr, nullptr, nullptr, 0);
    if (!m.ok) { Serial.printf("P %.3f nan nan nan nan\n", fa); continue; }
    Cpx Z = computeZ(m, fa);
    Serial.printf("P %.3f %.6g %.4f %.6g %.6g\n", fa, cmag(Z), cphaseDeg(Z), Z.re, Z.im);
  }
  Serial.println("SEND");
}

void cmdOpenCal() {
  const double fcal[3] = {25000.0, 30000.0, 35000.0};
  double sumC = 0, sumG2 = 0;
  for (int i = 0; i < 3; i++) {
    double fa = setFrequency(fcal[i]);
    delay(100);
    Meas m = acquire(fa, 300, nullptr, nullptr, nullptr, 0);
    Cpx Y = cdiv({1, 0}, rawZ(m));
    double w = 2.0 * M_PI * fa;
    sumC  += Y.im / w;
    sumG2 += Y.re / (w * w);
  }
  calC = sumC / 3; calG2 = sumG2 / 3; openCal = true;
  Serial.printf("CAL %.3f %.6g\n", calC * 1e12, calG2);
}

void handleLine(char *line) {
  // split into a command word and up to 5 numeric arguments
  char cmd[16] = {0};
  double args[5] = {0, 0, 0, 0, 0};
  int n = 0;
  char *tok = strtok(line, " \t,");
  if (!tok) return;
  strncpy(cmd, tok, sizeof(cmd) - 1);
  n = 1;
  while (n < 6 && (tok = strtok(nullptr, " \t,")) != nullptr) args[(n++) - 1] = strtod(tok, nullptr);
  double a1 = args[0], a2 = args[1], a3 = args[2], a4 = args[3], a5 = args[4];
  for (char *p = cmd; *p; p++) *p = toupper(*p);

  if (!strcmp(cmd, "PING"))          Serial.println("OK TEENSYBODE 3");
  else if (!strcmp(cmd, "DDSTICKS")) {
    if (n >= 2 && a1 >= 40 && a1 <= 400) { startDDS((uint32_t)a1); Serial.printf("OK DDSTICKS %lu %.3f\n", (unsigned long)ddsTicks, DDS_RATE); }
    else Serial.println("ERR DDSTICKS 40-400");
  }
  else if (!strcmp(cmd, "DITHER")) {
    if (n >= 2 && a1 >= 0 && a1 <= 3000) { ditherCycles = (uint32_t)a1; Serial.printf("OK DITHER %lu\n", (unsigned long)ditherCycles); }
    else Serial.println("ERR DITHER 0-3000");
  }
  else if (!strcmp(cmd, "RS")) {
    if (n >= 2 && a1 > 0.1 && a1 < 1e7) { RS = a1; Serial.printf("OK RS %.4f\n", RS); }
    else Serial.println("ERR RS value");
  }
  else if (!strcmp(cmd, "MEAS")) {
    if (n >= 4) cmdMeas(a1, (uint32_t)a2, (uint32_t)a3); else Serial.println("ERR MEAS f settle meas");
  }
  else if (!strcmp(cmd, "WAVE")) {
    if (n >= 3) cmdWave(a1, (int)a2); else Serial.println("ERR WAVE f bins");
  }
  else if (!strcmp(cmd, "SWEEP")) {
    if (n >= 6) cmdSweep(a1, a2, a3, (uint32_t)a4, (uint32_t)a5);
    else Serial.println("ERR SWEEP f0 f1 step settle meas");
  }
  else if (!strcmp(cmd, "OPENCAL"))  cmdOpenCal();
  else if (!strcmp(cmd, "CLEARCAL")) { openCal = false; Serial.println("OK CLEARCAL"); }
  else if (!strcmp(cmd, "IDLE"))     { outputOn = false; Serial.println("OK IDLE"); }
  else if (!strcmp(cmd, "STOP"))     Serial.println("OK STOP");
  else Serial.println("ERR unknown command");
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);

  const double mid = ((1 << PWM_BITS) - 1) / 2.0, amp = SINE_AMPL * ((1 << PWM_BITS) - 1);
  midCode = (uint8_t)lround(mid);
  for (int i = 0; i < 1024; i++)
    sineTable[i] = (uint8_t)lround(mid + amp * sin(2.0 * M_PI * i / 1024.0));

  analogWriteResolution(PWM_BITS);
  analogWriteFrequency(PWM_PIN, PWM_FREQ);
  analogWrite(PWM_PIN, midCode);

  adc->adc0->setAveraging(1);
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  adc->adc1->setAveraging(1);
  adc->adc1->setResolution(12);
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  adc->adc0->analogRead(PIN_A);
  chanA = ADC1_HC0 & 0x1F;
  adc->adc1->analogRead(PIN_B);
  chanB = ADC2_HC0 & 0x1F;

  startDDS(ddsTicks);
  setFrequency(30000.0);
}

void loop() {
  static char buf[96];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}
