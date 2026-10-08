/*
  main.cpp - STM32F407 Impedance Analyzer Firmware (Ported from TeensyBodeUI.ino)

  Hardware Mapping (STM32F407VET6 / Black STM32F407VE):
    Excitation DAC1 (Analog Sine) : PA4  (DAC_OUT1, 12-bit 0-3.3V)
    Excitation PWM  (Carrier Sine): PB6  (TIM4_CH1, 1 MHz carrier, 7-bit)
    Node A (ADC1 Channel 1)       : PA1  (ADC123_IN1)
    Node B (ADC2 Channel 2)       : PA2  (ADC123_IN2)
    Serial communication          : USB CDC (Virtual COM Port via Micro-USB / PA11 DM, PA12 DP)

  Measurement network:
    PA4 (DAC) or PB6 (PWM filter) -> R1 1k -> [C1 1.5n] -> R2 1k -> node A [C2 1.5n] -> RS 1k -> node B -> DUT -> GND
    node A -> R3 4.7k -> PA1 (+C3 100p) -> ADC1
    node B -> R4 4.7k -> PA2 (+C4 100p) -> ADC2

  Text protocol (compatible with teensy_bode_ui.py):
    PING                                   -> OK TEENSYBODE 3
    DDSTICKS <n>                           -> OK DDSTICKS <n> <rate_Hz>
    DITHER <cycles>                        -> OK DITHER <cycles>
    RS <ohms>                              -> OK RS <ohms>
    MEAS <f> <settle_ms> <meas_ms>         -> M <f> <|Z|> <phase_deg> <Re> <Im> <VppA> <VppB> <dcA> <dcB> <minA> <maxA> <minB> <maxB> <n>
    WAVE <f> <bins>                        -> W <same fields as M>, then <bins> lines "D <A_V> <B_V>", then WEND
    SWEEP <f0> <f1> <step> <settle> <meas> -> P <f> <|Z|> <phase_deg> <Re> <Im> (one per point), then SEND
    OPENCAL                                -> CAL <C_pF> <G2>
    CLEARCAL                               -> OK CLEARCAL
    IDLE                                   -> OK IDLE
    STOP                                   -> OK STOP
*/

#include <Arduino.h>
#include <math.h>

#define CommPort Serial
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  // Keep CMSIS SystemCoreClock in sync with the 168 MHz PLL setup above.
  // Without this, SystemCoreClock stays at 16 MHz (HSI default) while the
  // DWT cycle counter runs at 168 MHz, so acquire() would fit the sine at
  // ~10.5x the wrong frequency -> amplitudes ~0 -> 0/0 -> NaN in Z/CAL.
  SystemCoreClockUpdate();
}
// ======================= HARDWARE SETTINGS =======================
#define HEARTBEAT_LED PA_6
const int    PWM_PIN = PB6;         // TIM4_CH1 PWM output pin
const int    DAC_PIN = PA4;         // DAC1 analog output pin
const int    PIN_A   = PA1;         // node A -> ADC1 Channel 1
const int    PIN_B   = PA2;         // node B -> ADC2 Channel 2

double       RS      = 1000.0;     // sense resistor (default 1000 ohm, can be changed via GUI or RS command)
const double R_ADC   = 4700.0;     // R4 series resistor to ADC (4.7k ohm in standard schematic)
const double C_ADC   = 100e-12;    // C4 capacitor to ADC (100 pF)
const float  SINE_AMPL = 0.45f;    // fraction of full scale (max 0.5)
// ==================================================================

const double VREF = 3.3, ADC_MAX = 4095.0;
const int    PWM_BITS = 7;
const double PWM_FREQ = 1000000.0;  // 1.0 MHz PWM carrier frequency

// Duty-cycle update clock: Hardware TIM6 runs at 1.0 MSPS (1 MHz) for smooth sine synthesis.
uint32_t     ddsTicks = 64;
double       DDS_RATE = 1000000.0;

// Random extra delay after each ADC sample (CPU cycles at 168 MHz)
uint32_t     ditherCycles = 200;
const int    MAX_BINS = 200;

// ---------------- types ----------------
struct Cpx { double re, im; };
struct Meas {
  Cpx A, B;
  double dcA, dcB;
  int minA, maxA, minB, maxB;
  uint32_t n;
  bool ok;
};

// ---------------- Hardware DAC DMA Generation ----------------
#define SINE_POINTS 64
static uint16_t dacDmaSine[SINE_POINTS];
static uint16_t midCodeDAC = 2048;
volatile bool outputOn = true;

// Sets excitation frequency using Hardware Timer TIM6 + DMA1 Stream 5 Circular Transfer.
// ZERO CPU interrupt overhead, pure hardware DMA generation.
// Every single sine cycle contains 64 smooth analog voltage points.
double setFrequency(double f) {
  if (f < 1.0) f = 1.0;
  // Timer TIM6 clock is 84 MHz on APB1
  // Desired timer frequency = f * SINE_POINTS
  uint32_t psc = 0;
  uint32_t period = (uint32_t)lround(84000000.0 / (f * (double)SINE_POINTS));
  while (period > 65536) {
    psc++;
    period = (uint32_t)lround((84000000.0 / (psc + 1)) / (f * (double)SINE_POINTS));
  }
  if (period < 2) period = 2; // Maximum timer frequency 42 MHz
  uint32_t arr = period - 1;

  TIM6->PSC = psc;
  TIM6->ARR = arr;
  TIM6->EGR = TIM_EGR_UG; // Immediately load prescaler and ARR shadow registers

  if (!(DMA1_Stream5->CR & DMA_SxCR_EN)) {
    DMA1->HIFCR = DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CFEIF5;
    DMA1_Stream5->NDTR = SINE_POINTS;
    DMA1_Stream5->CR |= DMA_SxCR_EN;
  }
  if (!(TIM6->CR1 & TIM_CR1_CEN)) {
    TIM6->CR1 |= TIM_CR1_CEN;
  }
  outputOn = true;

  double timerClk = 84000000.0 / (double)(psc + 1);
  double fa = timerClk / ((double)(arr + 1) * (double)SINE_POINTS);
  return fa;
}

void stopExcitation() {
  outputOn = false;
  TIM6->CR1 &= ~TIM_CR1_CEN;
  DMA1_Stream5->CR &= ~DMA_SxCR_EN;
  while (DMA1_Stream5->CR & DMA_SxCR_EN);
  DAC->CR &= ~DAC_CR_TEN1; // Disable trigger for direct manual write
  DAC->DHR12R1 = midCodeDAC;
  DAC->CR |= DAC_CR_TEN1;  // Re-enable trigger
}

void startDDS(uint32_t ticks) {
  ddsTicks = ticks;
  setFrequency(30000.0);
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
static double det3(const double m[3][3]) {
  return m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
       - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
       + m[0][2]*(m[1][0]*m[2][1] - m[1][2]*m[2][0]);
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
  const double wcyc = TWO_PI_D * f / (double)SystemCoreClock;
  const uint32_t dur = (uint32_t)((uint64_t)ms * SystemCoreClock / 1000);

  double Scc=0, Sss=0, Scs=0, Sc=0, Ss=0, N=0;
  double SAc=0, SAs=0, SA=0, SBc=0, SBs=0, SB=0;
  Meas m; m.minA = m.minB = 99999; m.maxA = m.maxB = -1; m.ok = false;

  // Clear any residual EOC flags before starting acquisition loop
  (void)ADC1->DR;
  (void)ADC2->DR;
  ADC1->SR = 0;
  ADC2->SR = 0;

  uint32_t t0 = DWT->CYCCNT;
  while (true) {
    uint32_t t = DWT->CYCCNT;
    // Trigger simultaneous conversion on ADC1 & ADC2
    ADC1->CR2 |= ADC_CR2_SWSTART;

    uint32_t dt = t - t0;
    if (dt > dur) {
      while (!(ADC1->SR & ADC_SR_EOC) || !(ADC2->SR & ADC_SR_EOC)) ;
      (void)ADC1->DR; // clear EOC flags
      (void)ADC2->DR;
      break;
    }

    float ph = (float)fmod((double)dt * wcyc, TWO_PI_D);
    float c = cosf(ph), s = sinf(ph);

    // Wait until both ADCs complete conversion (master AND slave)
    while (!(ADC1->SR & ADC_SR_EOC) || !(ADC2->SR & ADC_SR_EOC)) ;

    // Read both ADC1 and ADC2 directly from their respective DR registers.
    // Reading ADC_DR automatically clears the EOC flag in SR.
    // (ADC_CDR is only valid/updated when DMA multi-mode is enabled).
    int a = (int)(ADC1->DR & 0x0FFF);
    int b = (int)(ADC2->DR & 0x0FFF);

    if (ditherCycles) { // xorshift32 random wait
      static uint32_t rng = 2463534242u;
      rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
      uint32_t w = rng % ditherCycles;
      uint32_t tw = DWT->CYCCNT;
      while ((uint32_t)(DWT->CYCCNT - tw) < w) ;
    }

    Scc += (double)c*c; Sss += (double)s*s; Scs += (double)c*s; Sc += c; Ss += s; N += 1;
    SAc += (double)a*c; SAs += (double)a*s; SA += a;
    SBc += (double)b*c; SBs += (double)b*s; SB += b;
    if (a < m.minA) m.minA = a;
    if (a > m.maxA) m.maxA = a;
    if (b < m.minB) m.minB = b;
    if (b > m.maxB) m.maxB = b;

    if (nbins > 0) {
      int bi = (int)(ph / (float)TWO_PI_D * nbins);
      if (bi >= nbins) bi = nbins - 1;
      if (bi >= 0) {
        binA[bi] += a; binB[bi] += b; binN[bi]++;
      }
    }
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
  CommPort.printf("%.3f %.6g %.4f %.6g %.6g %.5f %.5f %.5f %.5f %d %d %d %d %lu\n",
                f, cmag(Z), cphaseDeg(Z), Z.re, Z.im,
                2 * cmag(m.A) * k, 2 * cmag(m.B) * k, m.dcA * k, m.dcB * k,
                m.minA, m.maxA, m.minB, m.maxB, (unsigned long)m.n);
}

// ---------------- commands ----------------
void cmdDiag() {
  // One-shot hardware sanity check: excitation + both ADC channels alive?
  double fa = setFrequency(30000.0);
  delay(100);
  Meas m = acquire(fa, 100, nullptr, nullptr, nullptr, 0);
  const double k = VREF / ADC_MAX;
  CommPort.printf("DIAG f=%.3f ok=%d n=%lu VppA=%.4f VppB=%.4f dcA=%.4f dcB=%.4f "
                  "minA=%d maxA=%d minB=%d maxB=%d clk=%lu dds=%.1f\n",
                  fa, (int)m.ok, (unsigned long)m.n,
                  2 * cmag(m.A) * k, 2 * cmag(m.B) * k, m.dcA * k, m.dcB * k,
                  m.minA, m.maxA, m.minB, m.maxB,
                  (unsigned long)SystemCoreClock, DDS_RATE);
}

void cmdMeas(double f, uint32_t settle, uint32_t meas) {
  double fa = setFrequency(f);
  delay(settle);
  Meas m = acquire(fa, meas, nullptr, nullptr, nullptr, 0);
  if (!m.ok) { CommPort.println("ERR fit failed"); return; }
  CommPort.print("M "); printMeasFields(fa, m);
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
  if (!m.ok) { CommPort.println("ERR fit failed"); return; }
  CommPort.print("W "); printMeasFields(fa, m);
  const double k = VREF / ADC_MAX;
  for (int i = 0; i < bins; i++) {
    if (binN[i] == 0) CommPort.println("D nan nan");
    else CommPort.printf("D %.5f %.5f\n", (double)(binA[i] / binN[i] * k), (double)(binB[i] / binN[i] * k));
  }
  CommPort.println("WEND");
}

void cmdSweep(double f0, double f1, double step, uint32_t settle, uint32_t meas) {
  if (step <= 0 || f1 < f0 || f0 < 100 || f1 > 150000) { CommPort.println("ERR bad sweep range"); return; }
  // Flush any trailing newlines or whitespace from the incoming command buffer
  while (CommPort.available()) {
    char ch = (char)CommPort.peek();
    if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') (void)CommPort.read();
    else break;
  }
  setFrequency(f0);
  delay(200);
  int nmax = (int)((f1 - f0) / step + 1.5);
  for (int i = 0; i < nmax; i++) {
    if (CommPort.available()) { CommPort.println("SABORT"); return; }
    double fa = setFrequency(f0 + i * step);
    digitalToggle(HEARTBEAT_LED);
    delay(settle);
    Meas m = acquire(fa, meas, nullptr, nullptr, nullptr, 0);
    if (!m.ok) { CommPort.printf("P %.3f nan nan nan nan\n", fa); continue; }
    Cpx Z = computeZ(m, fa);
    CommPort.printf("P %.3f %.6g %.4f %.6g %.6g\n", fa, cmag(Z), cphaseDeg(Z), Z.re, Z.im);
  }
  CommPort.println("SEND");
}

void cmdOpenCal() {
  const double fcal[3] = {25000.0, 30000.0, 35000.0};
  double sumC = 0, sumG2 = 0;
  int nok = 0;
  for (int i = 0; i < 3; i++) {
    double fa = setFrequency(fcal[i]);
    delay(100);
    Meas m = acquire(fa, 300, nullptr, nullptr, nullptr, 0);
    if (!m.ok) {
      CommPort.printf("OCDBG %.3f fitfail minA=%d maxA=%d minB=%d maxB=%d n=%lu\n",
                      fa, m.minA, m.maxA, m.minB, m.maxB, (unsigned long)m.n);
      continue;  // skip failed points instead of averaging NaN in
    }
    Cpx Y = cdiv({1, 0}, rawZ(m));
    if (!isfinite(Y.re) || !isfinite(Y.im)) {
      CommPort.printf("OCDBG %.3f badY Amin=%.4f Bmin=%.4f VppA=%.4f VppB=%.4f\n",
                      fa, cmag(csub(m.A, m.B)), cmag(m.B),
                      2 * cmag(m.A) * (VREF / ADC_MAX), 2 * cmag(m.B) * (VREF / ADC_MAX));
      continue;
    }
    double w = 2.0 * M_PI * fa;
    sumC  += Y.im / w;
    sumG2 += Y.re / (w * w);
    nok++;
  }
  if (nok == 0) { CommPort.println("ERR OPENCAL no valid points"); return; }
  calC = sumC / nok; calG2 = sumG2 / nok; openCal = true;
  CommPort.printf("CAL %.3f %.6g\n", calC * 1e12, calG2);
}

void handleLine(char *line) {
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

  if (!strcmp(cmd, "PING"))          CommPort.println("OK TEENSYBODE 3");
  else if (!strcmp(cmd, "DDSTICKS")) {
    if (n >= 2 && a1 >= 40 && a1 <= 400) { startDDS((uint32_t)a1); CommPort.printf("OK DDSTICKS %lu %.3f\n", (unsigned long)ddsTicks, DDS_RATE); }
    else CommPort.println("ERR DDSTICKS 40-400");
  }
  else if (!strcmp(cmd, "DITHER")) {
    if (n >= 2 && a1 >= 0 && a1 <= 3000) { ditherCycles = (uint32_t)a1; CommPort.printf("OK DITHER %lu\n", (unsigned long)ditherCycles); }
    else CommPort.println("ERR DITHER 0-3000");
  }
  else if (!strcmp(cmd, "RS")) {
    if (n >= 2 && a1 > 0.1 && a1 < 1e7) { RS = a1; CommPort.printf("OK RS %.4f\n", RS); }
    else CommPort.println("ERR RS value");
  }
  else if (!strcmp(cmd, "FREQ")) {
    if (n >= 2 && a1 >= 1.0 && a1 <= 150000.0) {
      double fa = setFrequency(a1);
      CommPort.printf("OK FREQ %.3f\n", fa);
    } else CommPort.println("ERR FREQ 1..150000");
  }
  else if (!strcmp(cmd, "MEAS")) {
    if (n >= 4) cmdMeas(a1, (uint32_t)a2, (uint32_t)a3); else CommPort.println("ERR MEAS f settle meas");
  }
  else if (!strcmp(cmd, "WAVE")) {
    if (n >= 3) cmdWave(a1, (int)a2); else CommPort.println("ERR WAVE f bins");
  }
  else if (!strcmp(cmd, "SWEEP")) {
    if (n >= 6) cmdSweep(a1, a2, a3, (uint32_t)a4, (uint32_t)a5);
    else CommPort.println("ERR SWEEP f0 f1 step settle meas");
  }
  else if (!strcmp(cmd, "OPENCAL"))  cmdOpenCal();
  else if (!strcmp(cmd, "DIAG"))     cmdDiag();
  else if (!strcmp(cmd, "CLEARCAL")) { openCal = false; CommPort.println("OK CLEARCAL"); }
  else if (!strcmp(cmd, "IDLE"))     { stopExcitation(); CommPort.println("OK IDLE"); }
  else if (!strcmp(cmd, "STOP"))     { stopExcitation(); CommPort.println("OK STOP"); }
  else CommPort.println("ERR unknown command");
}

// ---------------- Hardware Initialization ----------------
void setupPeripherals() {
  // 0. Setup Heartbeat LED
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW);

  // 1. Enable DWT cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 2. Setup DAC1 on PA4 with Hardware Timer TIM6 TRGO trigger & DMA1 Stream 5 Channel 7
  pinMode(DAC_PIN, INPUT_ANALOG);
  RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  __DSB();

  // TIM6 Basic Timer: MMS = 010 (Update event as TRGO to trigger DAC)
  TIM6->CR1 = 0;
  TIM6->PSC = 0;
  TIM6->ARR = 43;
  TIM6->CR2 = TIM_CR2_MMS_1; // MMS = 010 (TRGO on update)
  TIM6->EGR = TIM_EGR_UG;
  TIM6->SR = 0;

  // DAC1: Channel 1 enabled, Trigger enabled (TSEL1 = 000 for TIM6 TRGO), DMA enabled
  DAC->CR = DAC_CR_TEN1 | DAC_CR_DMAEN1 | DAC_CR_EN1;
  DAC->DHR12R1 = midCodeDAC;

  // DMA1 Stream 5 Channel 7 (DAC1) circular transfer
  DMA1_Stream5->CR = 0;
  while (DMA1_Stream5->CR & DMA_SxCR_EN);
  DMA1->HIFCR = DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CFEIF5;
  DMA1_Stream5->PAR = (uint32_t)&(DAC->DHR12R1);
  DMA1_Stream5->M0AR = (uint32_t)dacDmaSine;
  DMA1_Stream5->NDTR = SINE_POINTS;
  DMA1_Stream5->CR = (7U << DMA_SxCR_CHSEL_Pos)
                   | (1U << DMA_SxCR_MSIZE_Pos)
                   | (1U << DMA_SxCR_PSIZE_Pos)
                   | DMA_SxCR_MINC
                   | DMA_SxCR_CIRC
                   | DMA_SxCR_DIR_0
                   | DMA_SxCR_PL_1;
  DMA1_Stream5->FCR = 0;

  // 3. Optional PWM carrier setup on PB6 (TIM4_CH1)
  pinMode(PWM_PIN, OUTPUT);
  analogWriteResolution(PWM_BITS);
  analogWriteFrequency(PWM_FREQ);
  analogWrite(PWM_PIN, 64);

  // 4. Setup Dual Simultaneous ADC on PA1 (ADC1_IN1) and PA2 (ADC2_IN2)
  pinMode(PIN_A, INPUT_ANALOG);
  pinMode(PIN_B, INPUT_ANALOG);

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;

  ADC->CCR = 0;
  ADC->CCR |= (ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2); // Regular simultaneous mode (0x06)
  ADC->CCR |= ADC_CCR_ADCPRE_0;                     // Prescaler /4 -> 21 MHz ADC clock

  ADC1->CR1 = 0; // 12-bit
  ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SMPR2 = (1 << (1 * 3)); // 15 cycles sampling time
  ADC1->SQR1 = 0;               // 1 conversion
  ADC1->SQR3 = 1;               // SQ1 = Channel 1 (PA1)

  ADC2->CR1 = 0; // 12-bit
  ADC2->CR2 = ADC_CR2_ADON;
  ADC2->SMPR2 = (1 << (2 * 3)); // 15 cycles sampling time
  ADC2->SQR1 = 0;               // 1 conversion
  ADC2->SQR3 = 2;               // SQ1 = Channel 2 (PA2)
}

// ---------------- setup / loop ----------------
void setup() {
  CommPort.begin(115200);

  // Precompute 64-point Sine table for 12-bit DAC1 (PA4):
  const double midDAC = 2047.5, ampDAC = SINE_AMPL * 4095.0;
  midCodeDAC = (uint16_t)lround(midDAC);

  for (int i = 0; i < SINE_POINTS; i++) {
    double s = sin(2.0 * M_PI * (double)i / (double)SINE_POINTS);
    dacDmaSine[i] = (uint16_t)lround(midDAC + ampDAC * s);
  }

  setupPeripherals();
  setFrequency(30000.0);
}

void loop() {
  static uint32_t lastHeartbeat = 0;
  uint32_t now = millis();
  if (now - lastHeartbeat >= 500) {
    lastHeartbeat = now;
    digitalToggle(HEARTBEAT_LED);
  }

  static char buf[96];
  static int len = 0;
  while (CommPort.available()) {
    char c = CommPort.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}
