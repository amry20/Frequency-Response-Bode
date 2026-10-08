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
const double R_ADC   = 1000.0;     // R4 series resistor to ADC (1k ohm)
const double C_ADC   = 10e-12;      // ~10 pF (parasitic pin & PCB capacitance without external C)
const float  SINE_AMPL = 0.45f;    // fraction of full scale (max 0.5)
// ==================================================================

const double VREF = 3.3, ADC_MAX = 4095.0;
const int    PWM_BITS = 7;
const double PWM_FREQ = 1000000.0;  // 1.0 MHz PWM carrier frequency

uint32_t     ddsTicks = 64;

// Random extra delay after each ADC sample (CPU cycles at 168 MHz: 168 cycles = 1.0 us matches Teensy 4.1 timing)
uint32_t     ditherCycles = 168;
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

// ---------------- Pure Hardware Zero-Interrupt DMA Synthesizer (2.0 MSPS Constant Clock) ----------------
// Operates with 100% constant clock rate (TIM6 ARR = 41, 84 MHz / 42 = 2.000000 MSPS exact).
// ZERO CPU interrupts during measurement -> 0% CPU load, ZERO sampling holes, ZERO beat frequencies!
// 60 to 100 points per cycle -> SILKY SMOOTH, PERFECT ANALOG SINE WAVE on oscilloscope!
// Uses dual 8192-point buffers swapped at zero-crossing in ~100 ns.
#define MASTER_LUT_SIZE 4096
#define MASTER_LUT_MASK 4095
static uint16_t masterSine[MASTER_LUT_SIZE];
static uint16_t midCodeDAC = 2048;

#define MAX_DMA_POINTS 8192
static uint16_t dacSineBuf[2][MAX_DMA_POINTS];
static int      activeBuf = 0;
volatile bool   outputOn = true;

const double DDS_RATE = 2000000.0; // 2.000000 MSPS rock-solid constant update rate

double setFrequency(double f) {
  if (f < 10.0) f = 10.0;

  const double f_dac = 2000000.0;
  double best_err = 1e9;
  int best_N = 100, best_M = 1;
  double best_fa = f;

  int max_M = (int)((double)MAX_DMA_POINTS * f / f_dac);
  if (max_M > 128) max_M = 128;
  for (int M = 1; M <= max_M; M++) {
    int N = (int)lround((double)M * f_dac / f);
    if (N >= 64 && N <= MAX_DMA_POINTS) {
      if ((double)N / (double)M >= 32.0) {
        double fa = ((double)M / (double)N) * f_dac;
        double err = fabs(fa - f);
        if (err < best_err) {
          best_err = err;
          best_N = N;
          best_M = M;
          best_fa = fa;
          if (err < 0.05) break;
        }
      }
    }
  }

  // Pre-render waveform into the inactive buffer (~0.1 ms)
  int nextBuf = 1 - activeBuf;
  for (int i = 0; i < best_N; i++) {
    uint32_t phase = ((uint64_t)i * best_M * MASTER_LUT_SIZE) / best_N;
    dacSineBuf[nextBuf][i] = masterSine[phase & MASTER_LUT_MASK];
  }

  if (outputOn && (DMA1_Stream5->CR & DMA_SxCR_EN)) {
    // Wait for the active buffer to reach Transfer Complete (wrap at phase 0, voltage = midDAC)
    uint32_t t0 = DWT->CYCCNT;
    const uint32_t timeout_cycles = (SystemCoreClock / 100); // 10 ms safeguard (8192 samples @ 2 MSPS takes 4.1 ms)
    while (!(DMA1->HISR & DMA_HISR_TCIF5)) {
      if ((uint32_t)(DWT->CYCCNT - t0) > timeout_cycles) break;
    }
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5;

    // Seamlessly swap pointer and reload NDTR in ~100 ns (TIM6 keeps running at 2.0 MSPS)
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN);

    DMA1->HIFCR = DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CFEIF5;
    DMA1_Stream5->M0AR = (uint32_t)dacSineBuf[nextBuf];
    DMA1_Stream5->NDTR = best_N;
    DMA1_Stream5->CR |= DMA_SxCR_EN;
    activeBuf = nextBuf;
  } else {
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN);
    DMA1->HIFCR = DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CFEIF5;
    DMA1_Stream5->M0AR = (uint32_t)dacSineBuf[nextBuf];
    DMA1_Stream5->NDTR = best_N;
    DMA1_Stream5->CR |= DMA_SxCR_EN;
    activeBuf = nextBuf;
    outputOn = true;
  }

  return best_fa;
}

void stopExcitation() {
  outputOn = false;
  DAC->CR &= ~DAC_CR_TEN1;
  DAC->DHR12R1 = midCodeDAC;
}

void startDDS(uint32_t ticks) {
  (void)ticks;
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
  const float binScale = (nbins > 0) ? (float)((double)nbins / TWO_PI_D) : 0.0f;

  double Scc=0, Sss=0, Scs=0, Sc=0, Ss=0, N=0;
  double SAc=0, SAs=0, SA=0, SBc=0, SBs=0, SB=0;
  Meas m; m.minA = m.minB = 99999; m.maxA = m.maxB = -1; m.ok = false;

  (void)ADC1->DR;
  (void)ADC2->DR;
  ADC1->SR = 0;
  ADC2->SR = 0;

  // Adaptive Dither Scaling:
  // Instead of a fixed small delay (168 cycles / 1.0 us), dynamically scale dither
  // to span at least half of the excitation signal period (180 degrees of phase).
  // This guarantees uniform phase sampling across 0..2*pi at any frequency,
  // completely eliminating stroboscopic harmonic locking and point-to-point beating.
  uint32_t dynDither = ditherCycles;
  if (ditherCycles > 0 && f > 10.0) {
    uint32_t halfPeriodCycles = (uint32_t)((double)SystemCoreClock / (2.0 * f));
    if (halfPeriodCycles > 4000) halfPeriodCycles = 4000;
    if (halfPeriodCycles < ditherCycles) halfPeriodCycles = ditherCycles;
    dynDither = halfPeriodCycles;
  }

  uint32_t t0 = DWT->CYCCNT;
  while (true) {
    __disable_irq();
    uint32_t t = DWT->CYCCNT;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    __enable_irq();

    uint32_t dt = t - t0;
    if (dt > dur) {
      while (!(ADC1->SR & ADC_SR_EOC) || !(ADC2->SR & ADC_SR_EOC)) ;
      (void)ADC1->DR;
      (void)ADC2->DR;
      break;
    }

    float ph = (float)fmod((double)dt * wcyc, TWO_PI_D);
    float c = cosf(ph), s = sinf(ph);

    while (!(ADC1->SR & ADC_SR_EOC) || !(ADC2->SR & ADC_SR_EOC)) ;

    int a = (int)(ADC1->DR & 0x0FFF);
    int b = (int)(ADC2->DR & 0x0FFF);

    if (dynDither) {
      static uint32_t rng = 2463534242u;
      rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
      uint32_t w = rng % dynDither;
      uint32_t tw = DWT->CYCCNT;
      while ((uint32_t)(DWT->CYCCNT - tw) < w) ;
    }

    // Hardware FPU single-precision multiplications (1 cycle each on Cortex-M4)
    float fa = (float)a;
    float fb = (float)b;

    Scc += (double)(c * c);
    Sss += (double)(s * s);
    Scs += (double)(c * s);
    Sc  += (double)c;
    Ss  += (double)s;
    N   += 1.0;

    SAc += (double)(fa * c);
    SAs += (double)(fa * s);
    SA  += (double)a;

    SBc += (double)(fb * c);
    SBs += (double)(fb * s);
    SB  += (double)b;

    if (a < m.minA) m.minA = a;
    if (a > m.maxA) m.maxA = a;
    if (b < m.minB) m.minB = b;
    if (b > m.maxB) m.maxB = b;

    if (nbins > 0) {
      int bi = (int)(ph * binScale);
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

// ---------------- Non-Blocking State Machine ----------------
enum AppState {
  STATE_IDLE = 0,
  STATE_SWEEP_SETTLING,
  STATE_SWEEP_MEASURING,
  STATE_MEAS_SETTLING,
  STATE_WAVE_SETTLING,
  STATE_OPENCAL_SETTLING
};

static volatile AppState appState = STATE_IDLE;

struct SweepContext {
  double f0, f1, step;
  uint32_t settle_ms, meas_ms;
  int current_idx;
  int total_points;
  double current_fa;
  uint32_t settle_start;
};
static SweepContext sweepCtx;

struct MeasContext {
  double fa;
  uint32_t meas_ms;
  uint32_t settle_start;
  uint32_t settle_ms;
};
static MeasContext measCtx;

struct WaveContext {
  double fa;
  int bins;
  uint32_t settle_start;
  uint32_t settle_ms;
};
static WaveContext waveCtx;

struct CalContext {
  int step;
  double fa;
  double sumC;
  double sumG2;
  int nok;
  uint32_t settle_start;
};
static CalContext calCtx;

// ---------------- commands ----------------
void cmdDiag() {
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
  measCtx.fa = setFrequency(f);
  measCtx.settle_ms = settle;
  measCtx.meas_ms = meas;
  measCtx.settle_start = millis();
  appState = STATE_MEAS_SETTLING;
}

void cmdWave(double f, int bins) {
  if (bins < 8) bins = 8;
  if (bins > MAX_BINS) bins = MAX_BINS;
  waveCtx.fa = setFrequency(f);
  waveCtx.bins = bins;
  waveCtx.settle_ms = 80;
  waveCtx.settle_start = millis();
  appState = STATE_WAVE_SETTLING;
}

void cmdSweep(double f0, double f1, double step, uint32_t settle, uint32_t meas) {
  if (step <= 0 || f1 < f0 || f0 < 100 || f1 > 150000) { CommPort.println("ERR bad sweep range"); return; }
  // Flush any trailing newlines or whitespace from the incoming command buffer
  while (CommPort.available()) {
    char ch = (char)CommPort.peek();
    if (ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t') (void)CommPort.read();
    else break;
  }
  sweepCtx.f0 = f0;
  sweepCtx.f1 = f1;
  sweepCtx.step = step;
  sweepCtx.settle_ms = settle;
  sweepCtx.meas_ms = meas;
  sweepCtx.current_idx = 0;
  sweepCtx.total_points = (int)((f1 - f0) / step + 1.5);
  sweepCtx.current_fa = setFrequency(f0);
  sweepCtx.settle_start = millis();
  appState = STATE_SWEEP_SETTLING;
}

void cmdOpenCal() {
  const double fcal[3] = {25000.0, 30000.0, 35000.0};
  calCtx.step = 0;
  calCtx.sumC = 0;
  calCtx.sumG2 = 0;
  calCtx.nok = 0;
  calCtx.fa = setFrequency(fcal[0]);
  calCtx.settle_start = millis();
  appState = STATE_OPENCAL_SETTLING;
}

void updateStateMachine() {
  switch (appState) {
    case STATE_IDLE:
      break;

    case STATE_SWEEP_SETTLING: {
      if (millis() - sweepCtx.settle_start >= sweepCtx.settle_ms) {
        appState = STATE_SWEEP_MEASURING;
      }
      break;
    }

    case STATE_SWEEP_MEASURING: {
      double fa = sweepCtx.current_fa;
      Meas m = acquire(fa, sweepCtx.meas_ms, nullptr, nullptr, nullptr, 0);
      if (!m.ok) {
        CommPort.printf("P %.3f nan nan nan nan\n", fa);
      } else {
        Cpx Z = computeZ(m, fa);
        CommPort.printf("P %.3f %.6g %.4f %.6g %.6g\n", fa, cmag(Z), cphaseDeg(Z), Z.re, Z.im);
      }

      sweepCtx.current_idx++;
      if (sweepCtx.current_idx < sweepCtx.total_points) {
        double nextF = sweepCtx.f0 + sweepCtx.current_idx * sweepCtx.step;
        sweepCtx.current_fa = setFrequency(nextF);
        sweepCtx.settle_start = millis();
        appState = STATE_SWEEP_SETTLING;
      } else {
        CommPort.println("SEND");
        appState = STATE_IDLE;
      }
      break;
    }

    case STATE_MEAS_SETTLING: {
      if (millis() - measCtx.settle_start >= measCtx.settle_ms) {
        Meas m = acquire(measCtx.fa, measCtx.meas_ms, nullptr, nullptr, nullptr, 0);
        if (!m.ok) CommPort.println("ERR fit failed");
        else { CommPort.print("M "); printMeasFields(measCtx.fa, m); }
        appState = STATE_IDLE;
      }
      break;
    }

    case STATE_WAVE_SETTLING: {
      if (millis() - waveCtx.settle_start >= waveCtx.settle_ms) {
        static float binA[MAX_BINS], binB[MAX_BINS];
        static uint32_t binN[MAX_BINS];
        for (int i = 0; i < waveCtx.bins; i++) { binA[i] = binB[i] = 0; binN[i] = 0; }
        // 120 ms capture duration gives STM32 (168 MHz) enough iterations to sweep all phase bins
        Meas m = acquire(waveCtx.fa, 120, binA, binB, binN, waveCtx.bins);
        if (!m.ok) CommPort.println("ERR fit failed");
        else {
          CommPort.print("W "); printMeasFields(waveCtx.fa, m);
          const double k = VREF / ADC_MAX;
          for (int i = 0; i < waveCtx.bins; i++) {
            if (binN[i] == 0) {
              // Interpolate empty bin across neighboring populated bins to eliminate waveform gaps
              int prev = (i - 1 + waveCtx.bins) % waveCtx.bins;
              int dist_prev = 1;
              while (binN[prev] == 0 && dist_prev < waveCtx.bins) {
                prev = (prev - 1 + waveCtx.bins) % waveCtx.bins;
                dist_prev++;
              }
              int next = (i + 1) % waveCtx.bins;
              int dist_next = 1;
              while (binN[next] == 0 && dist_next < waveCtx.bins) {
                next = (next + 1) % waveCtx.bins;
                dist_next++;
              }
              if (binN[prev] > 0 && binN[next] > 0) {
                float vA_p = binA[prev] / binN[prev], vA_n = binA[next] / binN[next];
                float vB_p = binB[prev] / binN[prev], vB_n = binB[next] / binN[next];
                float frac = (float)dist_prev / (float)(dist_prev + dist_next);
                float valA = vA_p + frac * (vA_n - vA_p);
                float valB = vB_p + frac * (vB_n - vB_p);
                CommPort.printf("D %.5f %.5f\n", (double)(valA * k), (double)(valB * k));
                continue;
              }
              CommPort.println("D nan nan");
            } else {
              CommPort.printf("D %.5f %.5f\n", (double)(binA[i] / binN[i] * k), (double)(binB[i] / binN[i] * k));
            }
          }
          CommPort.println("WEND");
        }
        appState = STATE_IDLE;
      }
      break;
    }

    case STATE_OPENCAL_SETTLING: {
      if (millis() - calCtx.settle_start >= 100) {
        Meas m = acquire(calCtx.fa, 300, nullptr, nullptr, nullptr, 0);
        if (m.ok) {
          Cpx Y = cdiv({1, 0}, rawZ(m));
          if (isfinite(Y.re) && isfinite(Y.im)) {
            double w = 2.0 * M_PI * calCtx.fa;
            calCtx.sumC  += Y.im / w;
            calCtx.sumG2 += Y.re / (w * w);
            calCtx.nok++;
          }
        }
        calCtx.step++;
        if (calCtx.step < 3) {
          const double fcal[3] = {25000.0, 30000.0, 35000.0};
          calCtx.fa = setFrequency(fcal[calCtx.step]);
          calCtx.settle_start = millis();
        } else {
          if (calCtx.nok == 0) {
            CommPort.println("ERR OPENCAL no valid points");
          } else {
            calC = calCtx.sumC / calCtx.nok;
            calG2 = calCtx.sumG2 / calCtx.nok;
            openCal = true;
            CommPort.printf("CAL %.3f %.6g\n", calC * 1e12, calG2);
          }
          appState = STATE_IDLE;
        }
      }
      break;
    }

    default:
      appState = STATE_IDLE;
      break;
  }
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
  else if (!strcmp(cmd, "IDLE"))     { appState = STATE_IDLE; stopExcitation(); CommPort.println("OK IDLE"); }
  else if (!strcmp(cmd, "STOP"))     { appState = STATE_IDLE; stopExcitation(); CommPort.println("OK STOP"); }
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

  // 2. Setup DAC1 on PA4 with Hardware Timer TIM6 TRGO trigger & DMA1 Stream 5 (2.000 MSPS constant)
  pinMode(DAC_PIN, INPUT_ANALOG);
  RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  __DSB();

  // TIM6 Basic Timer: 84 MHz / 42 = 2.000000 MSPS rock-solid rate, MMS = 010 (TRGO on update event)
  TIM6->CR1 = 0;
  TIM6->PSC = 0;
  TIM6->ARR = 41; // 84 MHz / 42 = 2.000000 MSPS constant (ARR NEVER CHANGES)
  TIM6->CR2 = TIM_CR2_MMS_1; // TRGO on update event
  TIM6->EGR = TIM_EGR_UG;
  TIM6->SR = 0;

  // DAC1: Channel 1 enabled, Trigger enabled (TIM6 TRGO), DMA enabled
  DAC->CR = DAC_CR_TEN1 | DAC_CR_DMAEN1 | DAC_CR_EN1;
  DAC->DHR12R1 = midCodeDAC;

  // DMA1 Stream 5 Channel 7 (DAC1) circular transfer (Pure Hardware, ZERO CPU INTERRUPTS)
  DMA1_Stream5->CR = 0;
  while (DMA1_Stream5->CR & DMA_SxCR_EN);
  DMA1->HIFCR = DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CFEIF5;
  DMA1_Stream5->PAR = (uint32_t)&(DAC->DHR12R1);
  DMA1_Stream5->M0AR = (uint32_t)dacSineBuf[0];
  DMA1_Stream5->NDTR = 100;
  DMA1_Stream5->CR = (7U << DMA_SxCR_CHSEL_Pos)
                   | (1U << DMA_SxCR_MSIZE_Pos)
                   | (1U << DMA_SxCR_PSIZE_Pos)
                   | DMA_SxCR_MINC
                   | DMA_SxCR_CIRC
                   | DMA_SxCR_DIR_0
                   | DMA_SxCR_PL_1;
  DMA1_Stream5->FCR = 0;

  DMA1_Stream5->CR |= DMA_SxCR_EN;
  TIM6->CR1 |= TIM_CR1_CEN;

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
  ADC1->SMPR2 = (2 << (1 * 3)); // 28 cycles sampling time (1.33 us @ 21 MHz: optimal settling for Rin up to 2.5k)
  ADC1->SQR1 = 0;               // 1 conversion
  ADC1->SQR3 = 1;               // SQ1 = Channel 1 (PA1)

  ADC2->CR1 = 0; // 12-bit
  ADC2->CR2 = ADC_CR2_ADON;
  ADC2->SMPR2 = (2 << (2 * 3)); // 28 cycles sampling time (1.33 us @ 21 MHz: optimal settling for Rin up to 2.5k)
  ADC2->SQR1 = 0;               // 1 conversion
  ADC2->SQR3 = 2;               // SQ1 = Channel 2 (PA2)
}

// ---------------- setup / loop ----------------
void setup() {
  CommPort.begin(115200);

  const double midDAC = 2047.5, ampDAC = SINE_AMPL * 4095.0;
  midCodeDAC = (uint16_t)lround(midDAC);

  for (int i = 0; i < MASTER_LUT_SIZE; i++) {
    double s = sin(2.0 * M_PI * (double)i / (double)MASTER_LUT_SIZE);
    masterSine[i] = (uint16_t)lround(midDAC + ampDAC * s);
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

    // If a sweep is active, any incoming byte immediately aborts the sweep!
    if (appState == STATE_SWEEP_SETTLING || appState == STATE_SWEEP_MEASURING) {
      appState = STATE_IDLE;
      CommPort.println("SABORT");
      len = 0;
      while (CommPort.available()) (void)CommPort.read();
      break;
    }

    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  updateStateMachine();
}
