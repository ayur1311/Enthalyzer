#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31865.h>
#include <SensirionI2cSht4x.h>

//
// PINOUT
//
#define FLOW_SENSOR_PIN 17
#define I2C_1_SCL_PIN   22
#define I2C_1_SDA_PIN   21
#define I2C_2_SCL_PIN   26
#define I2C_2_SDA_PIN   25
#define SPI_MOSI_PIN    23      // shared bus (WAGO clip) - both boards
#define SPI_MISO_PIN    19      // shared bus (WAGO clip) - both boards
#define SPI_CLK_PIN     18      // shared bus (WAGO clip) - both boards
#define SPI_CS_PIN      4       // MAX31865 #1 - water OUTLET (unique CS)
#define SPI_CS_PIN_2    5       // MAX31865 #2 - water INLET  (unique CS)

//
// Constant & Function Declarations
//
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

#define RNOMINAL  100.000
#define RREF      430.000

// ─────────────────────────────────────────────────────────────
// RUN-CONFIGURABLE PROCESS PARAMETERS  <-- CHANGE EACH RUN
// ─────────────────────────────────────────────────────────────
const float VVM         = 1.0;    // aeration rate [vessel volumes / minute]
const float V_LIQUID_L  = 0.05;   // reactor liquid volume [L]  <-- SET THIS EVERY RUN

// >>> FIX 1: Stabilization period — number of initial cycles to discard
// so the SHT45 / PT100 sensors settle before efficiency is computed.
const uint8_t STABILIZATION_CYCLES = 3;
uint8_t warmupCounter = 0;   // counts completed measurement cycles at startup

// ─────────────────────────────────────────────────────────────
// SENSOR CALIBRATION  (reference = Gas Outlet SHT45)
// Model:  corrected = A * raw^2 + B * raw + C     (2nd-order)
// Fitted 2026 from 8-pt thermostat sweep (5–40 °C), cal date 19.08
// ─────────────────────────────────────────────────────────────
struct Calib { float A; float B; float C; };

inline float applyCalib(const Calib &k, float raw) {
  return k.A * raw * raw + k.B * raw + k.C;
}

// Coefficients (reference = Gas Outlet SHT45)
const Calib CAL_WATER_IN  = { -0.00001410f, 1.189178f, -5.026825f };
const Calib CAL_WATER_OUT = {  0.00131830f, 1.132807f, -4.080799f };
const Calib CAL_GAS_IN    = {  0.00000000f, 1.002617f, -0.065665f };
const Calib CAL_GAS_OUT   = {  0.00000000f, 1.000000f,  0.000000f }; // reference

// Constant Thermodynamic Boundary Constraints
const float CP_WATER   = 4.184;       // Water specific heat [kJ/kg·°C]
const float CP_AIR     = 1006.0;      // Dry-air specific heat [J/kg·K]
const float R_AIR      = 287.0;       // Dry-air gas constant [J/kg·K]
const float H_FG       = 2501.0;      // Latent heat of vaporisation @0°C [kJ/kg]
const float P_ATM      = 101.325;     // Atmospheric pressure [kPa]

// Heat Exchanger Structural & Flow Geometry Specifications
const float COIL_AREA_A   = 0.013289; // Tube external surface area [m²] (6mm OD, 0.705m Long)
const float COIL_FACTOR_F = 0.95;     // Helical cross-flow correction multiplier

// >>> FIX 2: Valid sensor ranges used to reject bad / disconnected readings.
const float T_MIN_VALID  = -40.0;   // °C
const float T_MAX_VALID  = 125.0;   // °C
const float RH_MIN_VALID = 0.0;     // %
const float RH_MAX_VALID = 100.0;   // %

static char errorMessage[64];
static int16_t error;
const int Measurement_Interval = 10000;

float IN_rH = 0.000;
float IN_T = 0.000;
float OUT_rH = 0.000;
float OUT_T = 0.000;

TwoWire I2C_1 = TwoWire(0);
TwoWire I2C_2 = TwoWire(1);
SensirionI2cSht4x OUTLET_SENSOR = SensirionI2cSht4x();
SensirionI2cSht4x INLET_SENSOR = SensirionI2cSht4x();

// Two MAX31865 boards sharing the SPI bus, each with a unique chip-select pin.
Adafruit_MAX31865 PT100_OUT = Adafruit_MAX31865(SPI_CS_PIN,   SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CLK_PIN);
Adafruit_MAX31865 PT100_IN  = Adafruit_MAX31865(SPI_CS_PIN_2, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CLK_PIN);

volatile uint32_t pulseCount = 0;
const float pulsesPerLiter = 13000.0;
unsigned long lastTime = 0;

// ─────────────────────────────────────────────────────────────
// Flow Sensor Rolling Average (window = 10 samples)
// ─────────────────────────────────────────────────────────────
const uint8_t FLOW_AVG_WINDOW = 10;
float   flowBuffer[FLOW_AVG_WINDOW] = {0.0};
uint8_t flowIndex = 0;
uint8_t flowCount = 0;
float   flowSum   = 0.0;

void IRAM_ATTR pulseCounter() { pulseCount++; }

float flowRollingAverage(float newValue) {
  flowSum -= flowBuffer[flowIndex];
  flowBuffer[flowIndex] = newValue;
  flowSum += newValue;
  flowIndex = (flowIndex + 1) % FLOW_AVG_WINDOW;
  if (flowCount < FLOW_AVG_WINDOW) flowCount++;
  return flowSum / flowCount;
}

// >>> FIX 2 (helpers): simple range validators for temperature and humidity.
inline bool isValidTemp(float T)  { return (T >= T_MIN_VALID)  && (T <= T_MAX_VALID)  && !isnan(T); }
inline bool isValidRH(float RH)   { return (RH >= RH_MIN_VALID) && (RH <= RH_MAX_VALID) && !isnan(RH); }

// ─────────────────────────────────────────────────────────────
// PT100 Helpers: read RAW then apply 2nd-order calibration
// ─────────────────────────────────────────────────────────────
inline float readOutletTemp() {
  float raw = PT100_OUT.temperature(RNOMINAL, RREF);
  return applyCalib(CAL_WATER_OUT, raw);
}
inline float readInletTemp() {
  float raw = PT100_IN.temperature(RNOMINAL, RREF);
  return applyCalib(CAL_WATER_IN, raw);
}

// ─────────────────────────────────────────────────────────────
// Psychrometric helpers
// ─────────────────────────────────────────────────────────────
// Saturation vapour pressure (Tetens) [kPa]
inline float pSatTetens(float T) {
  return 0.61078 * exp((17.27 * T) / (T + 237.3));
}

// Humidity ratio w [kg water / kg dry air] from T [°C] and RH [%]
float humidityRatio(float T, float RH) {
  float pV = (RH / 100.0) * pSatTetens(T);
  return 0.62198 * pV / (P_ATM - pV);
}

// Moist-air enthalpy [kJ/kg dry air] from T [°C] and RH [%]
float calculateEnthalpy(float temperature, float relativeHumidity) {
  float pVapor = (relativeHumidity / 100.0) * pSatTetens(temperature);
  float wLocal = 0.62198 * pVapor / (P_ATM - pVapor);
  return (1.006 * temperature) + wLocal * (H_FG + (1.86 * temperature));
}

//
// SETUP
//
void setup()
{
  Serial.begin(115200);
  I2C_1.begin(I2C_1_SDA_PIN, I2C_1_SCL_PIN);
  I2C_2.begin(I2C_2_SDA_PIN, I2C_2_SCL_PIN);
  SPI.begin(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
  Serial.println();

  OUTLET_SENSOR.begin(I2C_1, SHT45_I2C_ADDR_44);
  uint32_t serialNumber = 0;
  error = OUTLET_SENSOR.serialNumber(serialNumber);
  if (error != NO_ERROR) {
      Serial.println("ERROR: Outlet SHT45-Sensor could not be initialized");
  } else {
      Serial.println("successfully initialized Outlet SHT45-Sensor");
  }

  INLET_SENSOR.begin(I2C_2, SHT45_I2C_ADDR_44);
  error = INLET_SENSOR.serialNumber(serialNumber);
  if (error != NO_ERROR) {
      Serial.println("ERROR: Inlet SHT45-Sensor could not be initialized");
  } else {
      Serial.println("successfully initialized Inlet SHT45-Sensor");
  }

  if (PT100_OUT.begin(MAX31865_4WIRE)) Serial.println("successfully initialized PT100 OUTLET (calibrated)");
  else Serial.println("ERROR: PT100 OUTLET could not be initialized");

  if (PT100_IN.begin(MAX31865_4WIRE)) Serial.println("successfully initialized PT100 INLET (calibrated)");
  else Serial.println("ERROR: PT100 INLET could not be initialized");

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING);

  Serial.println("successfully initialized flow sensor");
  Serial.println("Flow rolling average window: 10 samples");
  Serial.println("Calibration: 2nd-order, reference = Gas Outlet (SHT45)");

  Serial.print("Stabilization period: first ");
  Serial.print(STABILIZATION_CYCLES);
  Serial.println(" measurement cycles will be discarded.");

  Serial.print("Process config: VVM = "); Serial.print(VVM, 2);
  Serial.print(" , V_liquid = "); Serial.print(V_LIQUID_L, 2); Serial.println(" L");

  // ── HEADER (22 columns) — Added Q_water_gain [W] at the end ──
  Serial.println(
    "Runtime [millis]; H2O_Flowrate_raw [L/min]; H2O_Flowrate [L/min]; H2O_Inlet_Temp [°C]; H2O_Outlet_Temp [°C]; "
    "Inlet_rH [%]; Inlet_Temp [°C]; Inlet_Enthalpy [kJ/kg]; "
    "Outlet_rH [%]; Outlet_Temp [°C]; Outlet_Enthalpy [kJ/kg]; LMED [kJ/kg]; "
    "w_in [g/kg]; w_out [g/kg]; dW [g/kg]; DehumEff [%]; "
    "Q_sensible [W]; Q_latent [W]; Q_total [W]; Condensate [g/h]; "
    "U_gas [W/(m²·kJ/kg)]; Q_water_gain [W]"
  );
}

//
// LOOP
//
void loop(){
  if (millis() - lastTime >= Measurement_Interval)
  {
    Serial.print(lastTime);
    lastTime = millis();
    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // ─── Flow: raw + 10-sample rolling average ───
    float litersPerMinuteRaw = (pulses / pulsesPerLiter) * (60000.0 / Measurement_Interval);
    float litersPerMinute    = flowRollingAverage(litersPerMinuteRaw);
    float mWater             = litersPerMinute / 60.0; // kg/s (assume 1 kg/L)

    // ─── Water temperatures (2nd-order calibrated) ───
    float tWaterIn  = readInletTemp();
    float tWaterOut = readOutletTemp();

    Serial.print(" ; "); Serial.print(litersPerMinuteRaw, 3);
    Serial.print(" ; "); Serial.print(litersPerMinute, 3);
    Serial.print(" ; "); Serial.print(tWaterIn, 3);
    Serial.print(" ; "); Serial.print(tWaterOut, 3);
    Serial.print(" ; ");

    // ─── Inlet air (SHT45) — linear calibrated ───
    INLET_SENSOR.measureHighPrecision(IN_T, IN_rH);
    IN_T = applyCalib(CAL_GAS_IN, IN_T);
    float hGasIn = calculateEnthalpy(IN_T, IN_rH);
    Serial.print(IN_rH, 3); Serial.print(" ; ");
    Serial.print(IN_T, 3);  Serial.print(" ; ");
    Serial.print(hGasIn, 3);Serial.print(" ; ");

    // ─── Outlet air (SHT45) — REFERENCE ───
    OUTLET_SENSOR.measureHighPrecision(OUT_T, OUT_rH);
    OUT_T = applyCalib(CAL_GAS_OUT, OUT_T);
    float hGasOut = calculateEnthalpy(OUT_T, OUT_rH);
    Serial.print(OUT_rH, 3); Serial.print(" ; ");
    Serial.print(OUT_T, 3);  Serial.print(" ; ");
    Serial.print(hGasOut, 3);

    // >>> FIX 1 + FIX 2: validity checks
    bool sensorsValid = isValidTemp(IN_T)  && isValidRH(IN_rH) &&
                        isValidTemp(OUT_T) && isValidRH(OUT_rH) &&
                        isValidTemp(tWaterIn) && isValidTemp(tWaterOut);

    // spacer loops updated from 9 to 10 to match the 22 total columns
    if (warmupCounter < STABILIZATION_CYCLES) {
      warmupCounter++;
      Serial.print(" ; STABILIZING");
      for (uint8_t i = 0; i < 10; i++) Serial.print(" ; -");
      Serial.println();
      return;
    }

    if (!sensorsValid) {
      Serial.print(" ; INVALID");
      for (uint8_t i = 0; i < 10; i++) Serial.print(" ; -");
      Serial.println();
      return;
    }

    // ─── Dynamic wall (saturated) state at mean water temp ───
    float tWaterAvg       = (tWaterIn + tWaterOut) / 2.0;
    float hSatWallDynamic = calculateEnthalpy(tWaterAvg, 100.0);

    // ─── LMED Driving Force (signed log-mean) ───
    float deltaH1 = hGasOut - hSatWallDynamic;
    float deltaH2 = hGasIn  - hSatWallDynamic;
    float lmed = 0.0;
    if ((deltaH1 * deltaH2) > 0.0 && fabs(deltaH1 - deltaH2) > 1e-6)
      lmed = (deltaH1 - deltaH2) / log(deltaH1 / deltaH2);
    Serial.print(" ; "); Serial.print(lmed, 3);

    // ═════════════════════════════════════════════════════════
    //  DEHUMIDIFICATION FRAMEWORK  (Steps 3–7)
    // ═════════════════════════════════════════════════════════
    float wIn   = humidityRatio(IN_T,  IN_rH);
    float wOut  = humidityRatio(OUT_T, OUT_rH);
    float wWall = humidityRatio(tWaterAvg, 100.0);
    float dW = (wIn - wOut) * 1000.0;

    float dehumEff = 0.0;
    if ((wIn - wWall) > 1e-6) dehumEff = (wIn - wOut) / (wIn - wWall) * 100.0;
    if (dehumEff < 0.0)   dehumEff = 0.0;
    if (dehumEff > 100.0) dehumEff = 100.0;

    float pDry   = P_ATM - (IN_rH / 100.0) * pSatTetens(IN_T);
    float rhoDry = (pDry * 1000.0) / (R_AIR * (IN_T + 273.15));
    float Vair   = (VVM * V_LIQUID_L) / 1000.0 / 60.0;
    float mDry   = Vair * rhoDry;

    float Q_total    = mDry * (hGasIn - hGasOut) * 1000.0;
    float Q_latent   = mDry * (wIn - wOut) * H_FG * 1000.0;
    float Q_sensible = Q_total - Q_latent;
    float mCond      = mDry * (wIn - wOut) * 1000.0 * 3600.0;

    float overallU = 0.0;
    if (fabs(lmed) > 0.001) overallU = Q_total / (COIL_AREA_A * COIL_FACTOR_F * lmed);

    // ─── WATER-SIDE Heat Gain [W] ───
    // Q = m_dot * Cp * delta_T * 1000 (to convert kW to W)
    float Q_water_gain = mWater * CP_WATER * (tWaterOut - tWaterIn) * 1000.0;

    // ── Stream columns ──
    Serial.print(" ; "); Serial.print(wIn * 1000.0, 3);
    Serial.print(" ; "); Serial.print(wOut * 1000.0, 3);
    Serial.print(" ; "); Serial.print(dW, 3);
    Serial.print(" ; "); Serial.print(dehumEff, 2);
    Serial.print(" ; "); Serial.print(Q_sensible, 3);
    Serial.print(" ; "); Serial.print(Q_latent, 3);
    Serial.print(" ; "); Serial.print(Q_total, 3);
    Serial.print(" ; "); Serial.print(mCond, 3);
    Serial.print(" ; "); Serial.print(overallU, 3);
    Serial.print(" ; "); Serial.print(Q_water_gain, 3); // NEW: Output Q_water_gain

    Serial.println();
  }
}
