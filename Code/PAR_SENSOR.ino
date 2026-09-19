#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_ADS1X15.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <WebServer.h>

#include <Preferences.h>
#include <math.h>

// ---------- Hardware setup ----------

Adafruit_ADS1115 ads;

// 1.3" 128x64 I2C OLED mit SH1106, Hardware-I2C auf GPIO7 (SCL) und GPIO6 (SDA)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0,
  /* reset=*/ U8X8_PIN_NONE,
  /* clock=*/ 7,   // SCL
  /* data=*/  6    // SDA
);


// ---------- Website / ESP32 access point ----------
// Connect your phone/laptop to this WiFi and open: http://192.168.4.1
const char* AP_SSID = "FloraFlux-PAR";
const char* AP_PASS = "12345678";
WebServer server(80);
Preferences prefs;

// ADS1115: GAIN_ONE -> 0.125 mV/LSB
const float LSB_V = 0.000125f;

// Comparator / Reset pins
const int COMP_PIN   = 5;   // Eingang: Komparator
const int RESET_PIN  = 2;   // Ausgang: MAX323 / Schalter

// Button: verbunden mit 3.3V bei Tastendruck
const int BUTTON_PIN = 3;
const unsigned long DEBOUNCE_MS = 50;

// Reset-Pulsdauer
const uint16_t RESET_PULSE_US = 2000;   // 2 ms

// Mindestabstand zwischen gezählten Pulsen (gegen Flattern)
const uint32_t MIN_PULSE_INTERVAL_US = 500;   // simple Entprellung

// ---------- Integrator / PAR parameters ----------

const float V_THR   = 2.5f;      // Komparator-Schwelle [V]
const float V_BIAS  = 0.5f;      // Reset-/Startspannung des Integrators [V]

// 475 capacitor = 4.7 uF, not 470 nF.
// Important: 474 = 470 nF, 475 = 4.7 uF.
const float C_F = 4.7e-6f;       // Feedback-Kondensator [F]

// Fixed 5-second PAR measurement window
// The displayed PAR value is updated only once every 5 seconds.
// This avoids the fast jumping that happened when the code updated after only a few resets.
const uint32_t MEAS_WINDOW_MS       = 5000;   // normal mode: one displayed PPFD value every 5 s

// High-light mode:
// When the measured PPFD is high, the code switches from a fixed-time window
// to a fixed-pulse window. Instead of counting pulses for exactly 5 seconds,
// it waits for a stable number of reset events and measures the elapsed time.
// This reduces jumping at very high PPFD because every result is based on
// roughly the same amount of integrated charge.
const float    HIGH_LIGHT_ENTER_PPFD = 1000.0f;  // switch to 150-pulse mode above this value
const float    HIGH_LIGHT_EXIT_PPFD  = 800.0f;   // switch back to 5-s mode below this value
const uint32_t HIGH_LIGHT_PULSES     = 150;      // target pulse count in strong light
const uint32_t HIGH_LIGHT_MAX_MS     = 25000;    // fallback if light becomes weaker during pulse mode

// Additional display/storage smoothing for strong light:
// In high-light mode the raw measurement is still calculated normally, but
// the displayed/saved PPFD value is a moving average of the last 5 finished
// strong-light measurements. This reduces jumps like 2500 -> 2300 PPFD.
const uint8_t  HIGH_LIGHT_AVG_WINDOW = 5;

const float    MIN_LOWLIGHT_DELTA_V = 0.080f; // zero resets but >=80 mV rise -> estimate low light from ADC
const float    DARK_ZERO_DELTA_V    = 0.020f; // zero resets and <20 mV rise in 5 s -> treat as dark/covered

// Photodiode / Physikalische Konstanten
const double ETA_PD = 0.7;               // mittlere Quanteneffizienz
const double A_PD   = 9e-6;              // Fläche 9 mm^2 -> 9e-6 m^2
const double Q_E    = 1.602176634e-19;   // Elementarladung [C]
const double N_A    = 6.02214076e23;     // Avogadro [1/mol]

// Umrechnungsfaktor: A -> µmol/(m^2*s)
const double PAR_CONV_UMOL = 1e6 / (Q_E * A_PD * ETA_PD * N_A);

// Rough one-point calibration factor.
// Updated for the current high-light test:
// measured value was roughly 2300..2500 PPFD while the reference was 1522 PPFD.
// Using an approximate mean of 2400 PPFD:
// 8.31 * (1522 / 2400) = 5.27.
// This should still be replaced later by a real multi-point calibration curve.
const float PAR_CAL = 5.44f;

// ---------- Globals (ISR & Messung) ----------

volatile uint32_t pulseCountTotal  = 0;   // gesamte Resets seit Start
volatile uint32_t pulseCountWindow = 0;   // Resets im aktuellen Messfenster
volatile uint32_t lastCountMicros  = 0;
// Zeitpunkt, an dem im aktuellen Fenster die Ziel-Pulszahl erreicht wurde.
// Das ist wichtig fuer den Hochlicht-Modus, damit nicht die spaetere loop()-Zeit,
// sondern der echte Zeitpunkt des 150. Pulses fuer die Berechnung verwendet wird.
volatile uint32_t pulseTargetMicros = 0;

// Reset-Steuerung
volatile bool     resetActive     = false;
volatile uint32_t resetEndMicros  = 0;

// Smart storage flags
bool fullStorageSaveRequested = false;
uint8_t lastHalfHourIndexForStorage = 255;

// Anzeige-Werte für langsame Kanäle
float Vout_disp  = 0.0f;
float Vthr_disp  = 0.0f;
float Vbias_disp = 0.0f;

// Zeitsachen
uint32_t lastSlowReadMs   = 0;
const uint32_t SLOW_READ_INTERVAL_MS = 500;   // OLED/I2C langsamer aktualisieren, damit die Messung Vorrang hat
const bool ENABLE_SERIAL_PLOTTER = false;        // true nur fuer Debug; Serial-Ausgabe kann Timing stoeren
const uint32_t SERIAL_PLOT_INTERVAL_MS = 200;    // falls Serial Plotter aktiviert wird
uint32_t lastSerialPlotMs = 0;

uint32_t windowStartMs    = 0;    // Startzeit des aktuellen Messfensters [ms]
uint32_t windowStartMicros = 0;    // Startzeit des aktuellen Messfensters [us] fuer Hochlicht-Modus
float    windowStartVin   = 0.0f; // Integratorspannung am Anfang des Messfensters
bool     windowStartValid = false;

// "Tag" für DLI-Histogramm
uint32_t dayStartMs       = 0;    // Start des aktuellen 24h-Tages

// Button-State
int lastButtonReading     = LOW;
int stableButtonState     = LOW;
unsigned long lastDebounceTime = 0;

// Display-Mode: 0 = PPFD/DLI, 1 = Debug, 2 = Website-Link
int displayMode = 0;

// PAR & DLI
float  parUmol   = 0.0f;   // geglaetteter PAR-Wert [µmol/m^2/s]
float  parRawUmol = 0.0f;  // letzter ungesmoothter PAR-Wert [µmol/m^2/s]
bool   parSmoothingReady = false;
double dliMol    = 0.0;    // DLI [mol/m^2/d]

// DLI pro Stunde (24 Balken)
double dliHour[24] = {0.0};

// Website history storage
// 24h PAR graph: time-weighted 30-minute average PAR values.
// 48 bins = 24 hours * 2 half-hour intervals. This changes only the website/history display,
// not the actual 5-second measurement method.
const uint8_t PAR_24H_BINS = 48;
double parHalfHourWeightedSum[PAR_24H_BINS] = {0.0};
double parHalfHourSeconds[PAR_24H_BINS]     = {0.0};

// 7-day DLI graph: rolling DLI values. Current day is updated live.
double dli7Days[7] = {0.0};
uint8_t current7DayIndex = 0;

// Year calendar: without RTC, this is counted from device boot day.
const uint16_t YEAR_DAYS = 366;
double dliYear[366] = {0.0};
bool   dliYearValid[366] = {false};
uint16_t currentYearDayIndex = 0;

// Real date handling for the website calendar.
// In AP-only mode the ESP32 has no internet time, so the date is set manually
// from the website and then advanced using millis().
bool dateIsSet = false;
bool timeIsSet = false;
uint16_t currentYear = 2026;
uint8_t  currentMonth = 1;
uint8_t  currentDay = 1;
uint8_t  currentHour = 0;
uint8_t  currentMinute = 0;
uint8_t  currentSecond = 0;

// Persistent DLI storage (ESP32 NVS/Preferences).
// Only DLI history/date data is stored. The measurement method itself is unchanged.
const uint32_t STORAGE_SAVE_INTERVAL_MS = 30000UL;  // automatic save every 30 seconds
const bool SAVE_AFTER_EACH_MEASUREMENT = false;     // full history is not written every 5 s; only the latest current DLI is saved immediately
uint32_t lastStorageSaveMs = 0;
bool storageDirty = false;
bool storageLoaded = false;
uint32_t lastManualSaveMs = 0;

uint32_t lastWindowPulses = 0;    // n des letzten Fensters (Debug)
uint32_t lastWindowMs     = 0;    // Länge des letzten Messfensters (Debug)
bool highLightPulseMode  = false; // false = 5-s window, true = 150-pulse window

float highLightAvgBuffer[HIGH_LIGHT_AVG_WINDOW] = {0.0f};
uint8_t highLightAvgCount = 0;
uint8_t highLightAvgIndex = 0;

// Daily PAR statistics for the website.
float parDayMax = 0.0f;
float parDayMin = 0.0f;
bool  parDayMinValid = false;

// Website analysis: manual standard-deviation evaluation.
// Measurement logic is unchanged. The analysis starts only after pressing the website button.
// It starts from the next forced reset/window start, then calculates 5 standard deviations.
// Each standard deviation uses 20 finished 5-s PPFD measurements.
const uint8_t PPFD_STD_WINDOW = 20;
const uint8_t PPFD_STD_RESULTS = 5;
float ppfdStdBuffer[PPFD_STD_WINDOW] = {0.0f};
uint8_t ppfdStdBufferCount = 0;
float ppfdStdValues[PPFD_STD_RESULTS] = {0.0f};
float ppfdStdMeans[PPFD_STD_RESULTS] = {0.0f};
uint8_t ppfdStdCounts[PPFD_STD_RESULTS] = {0};
bool ppfdStdValid[PPFD_STD_RESULTS] = {false};
uint8_t ppfdStdResultCount = 0;
bool ppfdStdPending = false;
bool ppfdStdActive = false;


// ---------- Website HTML ----------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>FloraFlux PPFD-Monitor</title>
  <style>
    :root {
      --bg:#07140f;
      --bg2:#0b1f17;
      --card:rgba(14,31,25,.92);
      --card2:#102820;
      --text:#ecfdf5;
      --muted:#9bb8aa;
      --line:rgba(190,242,100,.18);
      --accent:#84cc16;
      --accent2:#22c55e;
      --warn:#f59e0b;
      --danger:#ef4444;
      --shadow:0 18px 50px rgba(0,0,0,.35);
    }
    * { box-sizing:border-box; }
    body {
      margin:0;
      font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;
      color:var(--text);
      background:
        radial-gradient(circle at top left, rgba(132,204,22,.18), transparent 28rem),
        radial-gradient(circle at top right, rgba(34,197,94,.13), transparent 24rem),
        linear-gradient(180deg, var(--bg), #050b08 70%);
      min-height:100vh;
    }
    .wrap { width:100%; max-width:none; margin:0 auto; padding:18px 28px; }
    .topbar { display:flex; justify-content:space-between; align-items:center; gap:12px; margin-bottom:14px; }
    .brand { display:flex; align-items:center; gap:10px; font-weight:900; letter-spacing:.02em; }
    .logo { width:36px; height:36px; border-radius:12px; display:grid; place-items:center; background:linear-gradient(135deg,var(--accent),var(--accent2)); color:#052e16; box-shadow:0 8px 24px rgba(132,204,22,.28); }
    .pill { color:#d9f99d; border:1px solid var(--line); background:rgba(132,204,22,.08); padding:8px 12px; border-radius:999px; font-size:13px; }
    .hero {
      position:relative; overflow:hidden; text-align:center; padding:34px 18px 30px; border-radius:28px;
      background:linear-gradient(180deg, rgba(22,45,35,.96), rgba(9,22,16,.96));
      border:1px solid var(--line); box-shadow:var(--shadow);
      display:flex; flex-direction:column; align-items:center; justify-content:center; width:100%;
    }
    .hero:before { content:""; position:absolute; inset:auto -20% -55% -20%; height:75%; background:radial-gradient(circle, rgba(132,204,22,.23), transparent 58%); }
    .hero > * { position:relative; }
    .hero .label, .hero .par, .hero .unit { width:100%; text-align:center; }
    .label { color:var(--muted); font-size:14px; letter-spacing:.1em; text-transform:uppercase; font-weight:800; }
    .par { font-size:clamp(48px, 9vw, 92px); font-weight:950; line-height:1; margin:10px 0 6px; text-shadow:0 0 24px rgba(132,204,22,.17); }
    .unit { color:#d9f99d; font-size:18px; font-weight:700; }
    .heroStats { display:flex; flex-direction:column; align-items:center; justify-content:center; gap:10px; margin:18px auto 0; width:100%; max-width:560px; text-align:center; }
    .heroStats .stat { width:100%; max-width:560px; margin-left:auto; margin-right:auto; text-align:center; }
    .stat { min-width:150px; border:1px solid var(--line); background:rgba(255,255,255,.04); border-radius:18px; padding:12px 14px; text-align:center; }
    .stat .k { color:var(--muted); font-size:12px; text-transform:uppercase; letter-spacing:.08em; text-align:center; }
    .stat .v { font-size:22px; font-weight:900; margin-top:3px; text-align:center; width:100%; overflow-wrap:anywhere; }
    .grid { display:grid; grid-template-columns:1fr; gap:16px; margin-top:16px; }
    .two { display:grid; grid-template-columns:1fr 1fr; gap:16px; }
    .card { background:var(--card); border:1px solid var(--line); border-radius:22px; padding:18px; box-shadow:var(--shadow); backdrop-filter:blur(8px); }
    .cardHeader { display:flex; justify-content:space-between; align-items:flex-start; gap:12px; margin-bottom:12px; }
    h2 { margin:0; font-size:20px; }
    .sub { color:var(--muted); font-size:13px; line-height:1.45; }
    canvas { width:100%; height:330px; display:block; }
    canvas.stdCanvas { height:260px; }
    .dateRow, .actionRow { display:flex; gap:10px; flex-wrap:wrap; align-items:center; margin-top:12px; }
    .dateRow.centered { justify-content:center; }
    .actionColumn { display:flex; flex-direction:column; align-items:center; gap:10px; margin-top:16px; }
    .actionColumn button, .actionColumn a.btn { width:min(320px,100%); text-align:center; }
    input, button, a.btn, select { border:1px solid var(--line); border-radius:14px; padding:11px 13px; font:inherit; }
    input { background:#07140f; color:var(--text); color-scheme:dark; }
    button, a.btn { background:linear-gradient(135deg,var(--accent),var(--accent2)); color:#052e16; font-weight:900; cursor:pointer; transition:.18s transform,.18s opacity; text-decoration:none; display:inline-block; }
    button:hover, a.btn:hover { transform:translateY(-1px); }
    button.secondary, a.btn.secondary { background:#0b1f17; color:#d9f99d; }
    button.danger { background:#2a1111; color:#fecaca; border-color:rgba(239,68,68,.35); }
    select { border:1px solid var(--line); border-radius:14px; padding:11px 13px; font:inherit; background:#07140f; color:var(--text); }
    .progressOuter { width:100%; height:18px; border-radius:999px; background:#07140f; border:1px solid var(--line); overflow:hidden; margin:12px 0 8px; }
    .progressInner { height:100%; width:0%; background:linear-gradient(90deg,var(--accent),var(--accent2)); border-radius:999px; }
    .smallCards { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:10px; margin-top:12px; }
    .smallCard { background:rgba(255,255,255,.04); border:1px solid var(--line); border-radius:16px; padding:12px; }
    .smallCard .k { color:var(--muted); font-size:12px; text-transform:uppercase; letter-spacing:.07em; }
    .smallCard .v { font-weight:900; font-size:22px; margin-top:3px; }
    .stdTable { width:100%; border-collapse:collapse; margin-top:14px; overflow:hidden; border-radius:14px; }
    .stdTable th, .stdTable td { border-bottom:1px solid var(--line); padding:10px 8px; text-align:center; font-size:14px; }
    .stdTable th { color:#d9f99d; font-size:12px; text-transform:uppercase; letter-spacing:.06em; background:rgba(132,204,22,.08); }
    .stdStatus { margin-top:10px; color:var(--muted); font-size:13px; text-align:center; }
    .message { min-height:18px; margin-top:8px; color:#d9f99d; font-size:13px; }
    .saveBox { margin:12px auto 0; width:min(420px,100%); text-align:center; border:1px solid var(--line); background:rgba(255,255,255,.04); border-radius:16px; padding:10px 12px; }
    .saveBox .k { color:var(--muted); font-size:12px; text-transform:uppercase; letter-spacing:.07em; }
    .saveBox .v { font-weight:900; font-size:18px; margin-top:2px; }
    .calendar { display:grid; gap:10px; }
    details { background:rgba(5,15,10,.72); border:1px solid var(--line); border-radius:16px; padding:0; overflow:hidden; }
    summary { cursor:pointer; font-weight:900; color:var(--text); padding:14px 16px; list-style:none; display:flex; justify-content:space-between; align-items:center; }
    summary::-webkit-details-marker { display:none; }
    summary:after { content:"▾"; color:#bef264; transition:.2s transform; }
    details:not([open]) summary:after { transform:rotate(-90deg); }
    .days { display:grid; grid-template-columns:repeat(auto-fill,minmax(52px,1fr)); gap:7px; padding:0 12px 12px; }
    .day { border:1px solid rgba(255,255,255,.12); border-radius:13px; padding:8px 4px; text-align:center; font-size:12px; color:white; min-height:48px; background:#102820; }
    .day strong { font-size:13px; }
    .today { outline:2px solid #bef264; box-shadow:0 0 0 4px rgba(190,242,100,.12); }
    .emptyDay { opacity:.35; }
    .legend { display:flex; gap:8px; flex-wrap:wrap; align-items:center; margin-top:10px; color:var(--muted); font-size:12px; }
    .sw { width:28px; height:10px; border-radius:999px; display:inline-block; border:1px solid rgba(255,255,255,.15); }
    .footer { color:var(--muted); font-size:12px; text-align:center; margin:18px 0 6px; }
    @media (max-width:760px) {
      .two { grid-template-columns:1fr; }
      .wrap { padding:12px; width:100%; }
      .hero { width:100%; padding-left:12px; padding-right:12px; align-items:center; text-align:center; }
      .heroStats { max-width:100%; align-items:center; text-align:center; }
      .heroStats .stat { width:100%; max-width:520px; margin-left:auto; margin-right:auto; text-align:center; }
      canvas { height:260px; }
      .stat { flex:1 1 140px; text-align:center; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="topbar">
      <div class="brand"><div class="logo">☘</div><div>FloraFlux PPFD-Monitor</div></div>
    </div>

    <section class="hero">
      <div class="label">Aktueller PPFD-Wert</div>
      <div class="par"><span id="parValue">--</span></div>
      <div class="unit">µmol/m²/s</div>
      <div class="heroStats">
        <div class="stat"><div class="k">DLI heute</div><div class="v"><span id="dliValue">--</span></div></div>
        <div class="stat"><div class="k">Datum / Uhrzeit</div><div class="v" id="dateStatus">--</div></div>
      </div>
    </section>

    <div class="grid">
      <section class="card">
        <div class="cardHeader">
          <div>
            <h2>Datum, Uhrzeit und CSV-Export</h2>
            <div class="sub">DLI-Kalender und der heutige PPFD-Verlauf werden im ESP32-Speicher abgelegt und nach dem Einschalten wieder geladen. Da der ESP32 im Access-Point-Modus keine Internetzeit hat, kann Datum und Uhrzeit hier manuell gesetzt werden. Nach längerer Ausschaltzeit bitte erneut setzen.</div>
          </div>
        </div>
        <div class="dateRow centered">
          <input type="datetime-local" id="dateTimeInput">
          <button onclick="setDateTime()">Datum & Uhrzeit setzen</button>
        </div>
        <div class="sub" style="margin-top:14px; max-width:680px;">
          Die folgenden Buttons dienen zur Verwaltung der gespeicherten Messdaten: manuelles Speichern vor dem Ausschalten, CSV-Export für die Auswertung sowie Zurücksetzen einzelner oder aller DLI-Daten.
        </div>
        <div class="actionColumn">
          <button class="secondary" onclick="saveNow()">Jetzt speichern</button>
          <a class="btn secondary" href="/export/dli.csv" download="flora_flux_dli.csv">CSV exportieren</a>
          <button class="secondary" onclick="resetToday()">Heutigen Tag zurücksetzen</button>
          <button class="danger" onclick="clearStorage()">Alle DLI-Daten löschen</button>
        </div>
        <div class="saveBox"><div class="k">Speicherstatus</div><div class="v" id="saveStatus">--</div></div>
        <div class="message" id="dateMessage"></div>
      </section>

      <section class="card">
        <div class="cardHeader"><div><h2>Pflanzen-DLI-Empfehlung</h2><div class="sub">Wähle eine Pflanze aus, um den heutigen DLI-Wert mit einem typischen Richtbereich zu vergleichen.</div></div></div>
        <div class="dateRow">
          <select id="plantSelect" onchange="savePlantSelection(); updatePlantCard(latestData)"></select>
        </div>
        <div class="smallCards">
          <div class="smallCard"><div class="k">Empfohlener Bereich</div><div class="v" id="plantRange">--</div></div>
          <div class="smallCard"><div class="k">Status heute</div><div class="v" id="plantStatus">--</div></div>
        </div>
        <div class="progressOuter"><div class="progressInner" id="plantProgress"></div></div>
        <div class="sub" id="plantNote">Richtwerte sind allgemeine Empfehlungen und hängen von Sorte, Wachstumsphase und Umgebung ab.</div>
      </section>

      <section class="card">
        <div class="cardHeader"><div><h2>PPFD-Verlauf der letzten 24 Stunden</h2><div class="sub">Mittelwert je 30 Minuten aus den 5-s-Messfenstern.</div></div></div>
        <canvas id="par24"></canvas>
      </section>

      <section class="card">
        <div class="cardHeader"><div><h2>DLI-Verlauf der letzten 7 Tage</h2><div class="sub">Der aktuelle Tag wird live fortgeschrieben.</div></div></div>
        <canvas id="dli7"></canvas>
      </section>

      <section class="card">
        <div class="cardHeader"><div><h2>DLI-Jahreskalender</h2><div class="sub" id="calendarNote">Monate können geöffnet und geschlossen werden. Der geöffnete Monat bleibt auch nach der Aktualisierung offen.</div></div></div>
        <div class="legend"><span>DLI:</span><span class="sw" style="background:#123524"></span><span>niedrig</span><span class="sw" style="background:#1f7a36"></span><span>mittel</span><span class="sw" style="background:#6fbf34"></span><span>hoch</span></div>
        <div id="yearCalendar" class="calendar"></div>
      </section>

      <section class="card">
        <div class="cardHeader"><div><h2>Standardabweichung der PPFD-Messungen</h2><div class="sub">Die Auswertung wird manuell gestartet. Ab dem nächsten Reset/Messfenster werden jeweils 20 fertige 5-s-PPFD-Werte gesammelt. Insgesamt werden 5 Standardabweichungen berechnet und danach automatisch gestoppt.</div></div></div>
        <div class="dateRow centered">
          <button onclick="startStdAnalysis()">Standardabweichung neu starten</button>
        </div>
        <div class="stdStatus" id="stdStatus">Status: wartet auf Start.</div>
        <table class="stdTable">
          <thead><tr><th>Nr.</th><th>Mittelwert</th><th>Standardabweichung</th><th>Messwerte</th></tr></thead>
          <tbody id="stdTableBody">
            <tr><td colspan="4">Noch keine Auswertung gestartet.</td></tr>
          </tbody>
        </table>
      </section>
    </div>
    <div class="footer">FloraFlux · lokaler ESP32-Webserver · http://192.168.4.1</div>
  </div>

<script>
const monthNames = ["Januar","Februar","März","April","Mai","Juni","Juli","August","September","Oktober","November","Dezember"];
let monthDays  = [31,28,31,30,31,30,31,31,30,31,30,31];
let openMonths = {};
let latestData = null;

const plantProfiles = [
  {name:'Salat', min:12, max:17, target:15},
  {name:'Basilikum', min:12, max:17, target:15},
  {name:'Mikrogrün', min:6, max:12, target:9},
  {name:'Erdbeere', min:17, max:20, target:18.5},
  {name:'Tomate', min:20, max:30, target:25},
  {name:'Gurke', min:20, max:30, target:25}
];

function initPlants() {
  const select = document.getElementById('plantSelect');
  if (!select) return;
  select.innerHTML = plantProfiles.map((p,i) => `<option value="${i}">${p.name}</option>`).join('');
  const saved = localStorage.getItem('floraPlantIndex');
  if (saved !== null && plantProfiles[Number(saved)]) select.value = saved;
}

function savePlantSelection() {
  const select = document.getElementById('plantSelect');
  if (select) localStorage.setItem('floraPlantIndex', select.value);
}

function updatePlantCard(data) {
  const select = document.getElementById('plantSelect');
  if (!select || !data) return;
  const p = plantProfiles[Number(select.value)] || plantProfiles[0];
  const dli = Number(data.dli || 0);
  document.getElementById('plantRange').textContent = `${p.min}–${p.max} mol/m²/d`;
  let status = 'zu niedrig';
  if (dli >= p.min && dli <= p.max) status = 'im Bereich';
  else if (dli > p.max) status = 'zu hoch';
  document.getElementById('plantStatus').textContent = status;
  const percent = Math.max(0, Math.min((dli / p.target) * 100, 140));
  document.getElementById('plantProgress').style.width = Math.min(percent, 100).toFixed(0) + '%';
  document.getElementById('plantNote').textContent = `${p.name}: Zielwert ca. ${p.target} mol/m²/d. Aktuell erreicht: ${dli.toFixed(2)} mol/m²/d.`;
}

function niceMax(v) {
  if (!v || v <= 0) return 1;
  const p = Math.pow(10, Math.floor(Math.log10(v)));
  return Math.ceil(v / p) * p;
}

function drawLineChart(canvas, values, unit, labels) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, rect.width * dpr);
  canvas.height = Math.max(1, rect.height * dpr);
  const ctx = canvas.getContext('2d');
  ctx.scale(dpr, dpr);
  const w = rect.width, h = rect.height;
  const pad = {l:52, r:18, t:18, b:34};
  ctx.clearRect(0,0,w,h);
  ctx.font = '12px system-ui';

  // Null-Werte bedeuten: fuer diesen Zeitraum existiert noch kein Messpunkt.
  // Dadurch faellt der PPFD-Verlauf nicht kuenstlich auf 0 ab, sondern endet beim letzten echten Messpunkt.
  const nums = (values || []).map(v => {
    // Wichtig: JSON-null darf nicht mit Number(null) zu 0 werden.
    // Sonst verbindet der Graph den letzten echten PPFD-Punkt mit zukuenftigen Leerwerten bei 0.
    if (v === null || v === undefined || v === '') return null;
    const n = Number(v);
    return Number.isFinite(n) ? n : null;
  });
  const validNums = nums.filter(v => v !== null);
  const maxVal = niceMax(Math.max(...validNums, 1));

  ctx.strokeStyle = 'rgba(190,242,100,.16)';
  ctx.lineWidth = 1;
  ctx.fillStyle = '#9bb8aa';
  ctx.textAlign = 'right';
  for (let i=0;i<=4;i++) {
    const y = pad.t + (h-pad.t-pad.b) * i/4;
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w-pad.r, y); ctx.stroke();
    const label = Math.round(maxVal * (1-i/4));
    ctx.fillText(label, pad.l-8, y+4);
  }

  const xStep = (w-pad.l-pad.r) / Math.max(nums.length-1, 1);
  const yFor = v => pad.t + (h-pad.t-pad.b) * (1 - Math.min(v/maxVal,1));

  ctx.strokeStyle = '#84cc16';
  ctx.lineWidth = 3;
  ctx.beginPath();
  let drawing = false;
  nums.forEach((v,i) => {
    if (v === null) { drawing = false; return; }
    const x = pad.l + i*xStep;
    const y = yFor(v);
    if (!drawing) { ctx.moveTo(x,y); drawing = true; }
    else ctx.lineTo(x,y);
  });
  ctx.stroke();

  ctx.fillStyle = '#ecfdf5';
  nums.forEach((v,i) => {
    if (v === null) return;
    // Bei 48 Halb-Stunden-Bins werden nur volle Stunden und der letzte echte Punkt markiert.
    const lastValidIndex = nums.map(x => x !== null).lastIndexOf(true);
    if (nums.length === 48 && i % 2 && i !== lastValidIndex) return;
    if (nums.length > 48 && i % 2 && i !== lastValidIndex) return;
    const x = pad.l + i*xStep;
    const y = yFor(v);
    ctx.beginPath(); ctx.arc(x,y,3,0,Math.PI*2); ctx.fill();
  });

  ctx.fillStyle = '#9bb8aa';
  ctx.textAlign = 'center';
  if (nums.length === 48) {
    [0,12,24,36,47].forEach(i => {
      const hour = Math.floor(i / 2);
      ctx.fillText((i === 47 ? '23:30' : String(hour).padStart(2,'0') + ':00'), pad.l+i*xStep, h-10);
    });
  } else if (nums.length === 24) {
    [0,6,12,18,23].forEach(i => ctx.fillText(String(i) + ' h', pad.l+i*xStep, h-10));
  } else {
    const l = labels || ['-6','-5','-4','-3','-2','Gestern','Heute'];
    let idxs = [];
    if (nums.length <= 10) {
      nums.forEach((_,i) => idxs.push(i));
    } else {
      idxs = [0, Math.floor((nums.length-1)*0.25), Math.floor((nums.length-1)*0.5), Math.floor((nums.length-1)*0.75), nums.length-1];
    }
    idxs.forEach(i => ctx.fillText(l[i] || '', pad.l+i*xStep, h-10));
  }
  ctx.textAlign = 'left';
  ctx.fillText(unit, pad.l, 12);
}

function colorForDli(v, valid) {
  if (!valid) return '#102018';
  const x = Math.max(0, Math.min(v / 30, 1));
  const r = Math.round(18 + x * 95);
  const g = Math.round(50 + x * 165);
  const b = Math.round(32 + x * 20);
  return `rgb(${r}, ${g}, ${b})`;
}

function rememberOpenMonths() {
  document.querySelectorAll('#yearCalendar details').forEach(det => {
    const m = det.getAttribute('data-month');
    if (m !== null) openMonths[m] = det.open;
  });
}

function renderCalendar(values, validValues, monthDayCounts, currentMonth, todayIndex) {
  rememberOpenMonths();
  if (Array.isArray(monthDayCounts) && monthDayCounts.length === 12) monthDays = monthDayCounts;
  const root = document.getElementById('yearCalendar');
  root.innerHTML = '';
  let dayIndex = 0;
  for (let m=0; m<12; m++) {
    const details = document.createElement('details');
    details.setAttribute('data-month', String(m));
    if (openMonths[m] !== undefined) details.open = !!openMonths[m];
    else details.open = (m + 1) === currentMonth;
    details.addEventListener('toggle', () => { openMonths[m] = details.open; });

    const summary = document.createElement('summary');
    const monthTotal = monthTotalDli(values, validValues, dayIndex, monthDays[m]);
    summary.innerHTML = `<span>${monthNames[m]}</span><span>${monthTotal.toFixed(1)} mol/m²</span>`;
    details.appendChild(summary);

    const days = document.createElement('div');
    days.className = 'days';
    for (let d=1; d<=monthDays[m]; d++) {
      const v = Number(values[dayIndex] || 0);
      const valid = !!(validValues && validValues[dayIndex]);
      const btn = document.createElement('button');
      btn.className = 'day' + (dayIndex === todayIndex ? ' today' : '') + (!valid ? ' emptyDay' : '');
      btn.style.background = colorForDli(v, valid);
      btn.innerHTML = `<strong>${d}</strong><br>${valid ? v.toFixed(1) : '—'}`;
      btn.title = `${monthNames[m]} ${d}: ${valid ? v.toFixed(2) + ' mol/m²/day' : 'kein Wert gespeichert'}`;
      days.appendChild(btn);
      dayIndex++;
    }
    details.appendChild(days);
    root.appendChild(details);
  }
}

function monthTotalDli(values, validValues, start, count) {
  let total = 0;
  for (let i=0; i<count; i++) if (!validValues || validValues[start+i]) total += Number(values[start+i] || 0);
  return total;
}

async function setDateTime() {
  const d = document.getElementById('dateTimeInput').value;
  const msg = document.getElementById('dateMessage');
  if (!d) { msg.textContent = 'Bitte zuerst Datum und Uhrzeit auswählen.'; return; }
  try {
    const r = await fetch('/setdatetime?dt=' + encodeURIComponent(d));
    msg.textContent = await r.text();
    update();
  } catch(e) { msg.textContent = 'Datum/Uhrzeit konnte nicht gesetzt werden.'; }
}

async function saveNow() {
  const msg = document.getElementById('dateMessage');
  try {
    const r = await fetch('/save');
    msg.textContent = await r.text();
    update();
  } catch(e) { msg.textContent = 'Speichern fehlgeschlagen.'; }
}

async function resetToday() {
  if (!confirm('Nur den heutigen DLI-Tageswert zurücksetzen? Gespeicherte ältere Tage bleiben erhalten.')) return;
  const msg = document.getElementById('dateMessage');
  try {
    const r = await fetch('/resettoday');
    msg.textContent = await r.text();
    update();
  } catch(e) { msg.textContent = 'Zurücksetzen fehlgeschlagen.'; }
}

async function clearStorage() {
  if (!confirm('Alle gespeicherten DLI-Werte wirklich löschen?')) return;
  const msg = document.getElementById('dateMessage');
  try {
    const r = await fetch('/clear');
    msg.textContent = await r.text();
    openMonths = {};
    update();
  } catch(e) { msg.textContent = 'Löschen fehlgeschlagen.'; }
}

async function startStdAnalysis() {
  const msg = document.getElementById('dateMessage');
  try {
    const r = await fetch('/startstd');
    msg.textContent = await r.text();
    update();
  } catch(e) { msg.textContent = 'Standardabweichung konnte nicht gestartet werden.'; }
}

function renderStdTable(data) {
  const body = document.getElementById('stdTableBody');
  const status = document.getElementById('stdStatus');
  if (!body || !status) return;

  const values = data.stdValues || [];
  const means = data.stdMeans || [];
  const counts = data.stdCounts || [];
  const valid = data.stdValid || [];
  const active = !!data.stdActive;
  const pending = !!data.stdPending;
  const currentCount = Number(data.stdCurrentCount || 0);
  const resultCount = Number(data.stdResultCount || 0);

  if (pending) status.textContent = 'Status: Start vorgemerkt. Die Auswertung beginnt ab dem nächsten Reset/Messfenster.';
  else if (active) status.textContent = 'Status: läuft. Aktuelle Gruppe: ' + currentCount + ' / 20 Messwerte. Fertige Standardabweichungen: ' + resultCount + ' / 5.';
  else if (resultCount >= 5) status.textContent = 'Status: abgeschlossen. Für eine neue Messreihe den Button erneut drücken.';
  else status.textContent = 'Status: wartet auf Start.';

  let rows = '';
  for (let i=0; i<5; i++) {
    if (valid[i]) {
      rows += '<tr><td>' + (i+1) + '</td><td>' + Number(means[i]).toFixed(1) + ' µmol/m²/s</td><td>' + Number(values[i]).toFixed(1) + ' µmol/m²/s</td><td>' + Number(counts[i] || 20) + '</td></tr>';
    } else {
      rows += '<tr><td>' + (i+1) + '</td><td>—</td><td>—</td><td>—</td></tr>';
    }
  }
  body.innerHTML = rows;
}

async function update() {
  try {
    const r = await fetch('/data');
    const data = await r.json();
    latestData = data;
    document.getElementById('parValue').textContent = Math.round(data.par);
    document.getElementById('dliValue').textContent = Number(data.dli).toFixed(2) + ' mol/m²';
    document.getElementById('dateStatus').textContent = data.dateTimeSet ? data.datetime : (data.dateSet ? data.date : 'nicht gesetzt');
    document.getElementById('saveStatus').textContent = data.storageDirty ? 'offen' : 'gespeichert';
    document.getElementById('calendarNote').textContent = data.dateSet
      ? 'DLI-Werte werden dem realen Kalender zugeordnet. Nach längerer Ausschaltzeit bitte das Datum prüfen.'
      : 'Datum nicht gesetzt: DLI kann gespeichert werden, aber der Kalender ist noch nicht sicher einem echten Datum zugeordnet.';
    drawLineChart(document.getElementById('par24'), data.par24 || [], 'µmol/m²/s');
    drawLineChart(document.getElementById('dli7'), data.dli7 || [], 'mol/m²/day');
    renderStdTable(data);
    renderCalendar(data.year || [], data.yearValid || [], data.monthDays || [], data.month || 1, data.todayIndex || 0);
    updatePlantCard(data);
  } catch(e) { console.log(e); }
}

window.addEventListener('resize', () => { if (latestData) update(); });
initPlants();
update();
setInterval(update, 5000);
</script>
</body>
</html>
)rawliteral";


bool isLeapYear(uint16_t y) {
  if (y % 400 == 0) return true;
  if (y % 100 == 0) return false;
  return (y % 4 == 0);
}

uint8_t daysInMonth(uint16_t y, uint8_t m) {
  static const uint8_t daysNormal[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m < 1 || m > 12) return 31;
  if (m == 2 && isLeapYear(y)) return 29;
  return daysNormal[m - 1];
}

uint16_t daysInYear(uint16_t y) {
  return isLeapYear(y) ? 366 : 365;
}

uint16_t dayOfYearIndex(uint16_t y, uint8_t m, uint8_t d) {
  uint16_t idx = 0;
  for (uint8_t month = 1; month < m; month++) {
    idx += daysInMonth(y, month);
  }
  idx += (uint16_t)(d - 1);
  return idx;
}

void clearYearHistory() {
  for (uint16_t i = 0; i < 366; i++) {
    dliYear[i] = 0.0;
    dliYearValid[i] = false;
  }
}

void clearPPFDStdAnalysis() {
  ppfdStdBufferCount = 0;
  for (uint8_t i = 0; i < PPFD_STD_WINDOW; i++) ppfdStdBuffer[i] = 0.0f;
  for (uint8_t i = 0; i < PPFD_STD_RESULTS; i++) {
    ppfdStdValues[i] = 0.0f;
    ppfdStdMeans[i] = 0.0f;
    ppfdStdCounts[i] = 0;
    ppfdStdValid[i] = false;
  }
  ppfdStdResultCount = 0;
  ppfdStdPending = false;
  ppfdStdActive = false;
}

void requestPPFDStdAnalysisStart() {
  clearPPFDStdAnalysis();
  ppfdStdPending = true;
}

void beginPPFDStdAnalysisAfterReset() {
  ppfdStdPending = false;
  ppfdStdActive = true;
  ppfdStdBufferCount = 0;
}

String twoDigits(uint8_t v) {
  if (v < 10) return "0" + String(v);
  return String(v);
}

String currentDateString() {
  if (!dateIsSet) return "nicht gesetzt";
  return String(currentYear) + "-" + twoDigits(currentMonth) + "-" + twoDigits(currentDay);
}

uint32_t secondsSinceMidnightFromStoredTime() {
  return (uint32_t)currentHour * 3600UL + (uint32_t)currentMinute * 60UL + (uint32_t)currentSecond;
}

void updateCurrentClockFromMillis() {
  if (!timeIsSet) return;
  uint32_t secondsToday = ((uint32_t)(millis() - dayStartMs) / 1000UL) % 86400UL;
  currentHour = secondsToday / 3600UL;
  currentMinute = (secondsToday % 3600UL) / 60UL;
  currentSecond = secondsToday % 60UL;
}

String currentTimeString() {
  if (!timeIsSet) return "nicht gesetzt";
  updateCurrentClockFromMillis();
  return twoDigits(currentHour) + ":" + twoDigits(currentMinute) + ":" + twoDigits(currentSecond);
}

String currentDateTimeString() {
  if (!dateIsSet) return "nicht gesetzt";
  if (!timeIsSet) return currentDateString();
  return currentDateString() + " " + currentTimeString();
}

void alignDayStartToStoredTime(uint32_t nowMs) {
  if (!timeIsSet) return;
  dayStartMs = nowMs - secondsSinceMidnightFromStoredTime() * 1000UL;
}

uint8_t currentHalfHourIndexFromMillis(uint32_t nowMs) {
  const uint32_t HALF_HOUR_MS = 1800000UL;
  uint32_t dayElapsedMs = nowMs - dayStartMs;
  return (uint8_t)((dayElapsedMs / HALF_HOUR_MS) % PAR_24H_BINS);
}

void markStorageDirty() {
  storageDirty = true;
}

void saveCurrentDLIOnly() {
  // Lightweight safety save after each 5-s measurement window.
  // This stores only the current day/date position and the latest DLI value,
  // so the current DLI can reappear after power-off without writing the full history every time.
  if (!prefs.begin("flora", false)) return;
  prefs.putUInt("magic", 0x464C5851); // FLXQ
  updateCurrentClockFromMillis();
  prefs.putBool("dateSet", dateIsSet);
  prefs.putBool("timeSet", timeIsSet);
  prefs.putUShort("year", currentYear);
  prefs.putUChar("month", currentMonth);
  prefs.putUChar("day", currentDay);
  prefs.putUChar("hour", currentHour);
  prefs.putUChar("minute", currentMinute);
  prefs.putUChar("second", currentSecond);
  prefs.putUChar("idx7", current7DayIndex);
  prefs.putUShort("idxYear", currentYearDayIndex);
  prefs.putDouble("dliMol", dliMol);

  // Schnellspeicher für den aktuell laufenden 30-Minuten-PPFD-Bin.
  // Dadurch bleibt auch der heutige PPFD-Verlauf nach einem Neustart erhalten,
  // ohne alle 48 Bins alle 5 Sekunden komplett zu schreiben.
  uint8_t qIdx = currentHalfHourIndexFromMillis(millis());
  prefs.putUChar("qParIdx", qIdx);
  prefs.putDouble("qParSum", parHalfHourWeightedSum[qIdx]);
  prefs.putDouble("qParSec", parHalfHourSeconds[qIdx]);
  prefs.end();
}

void savePersistentData() {
  // Keep the current day mirrored into the stored histories before writing.
  dli7Days[current7DayIndex] = dliMol;
  dliYear[currentYearDayIndex] = dliMol;
  dliYearValid[currentYearDayIndex] = true;

  if (!prefs.begin("flora", false)) return;
  prefs.putUInt("magic", 0x464C5851); // FLXQ
  updateCurrentClockFromMillis();
  prefs.putBool("dateSet", dateIsSet);
  prefs.putBool("timeSet", timeIsSet);
  prefs.putUShort("year", currentYear);
  prefs.putUChar("month", currentMonth);
  prefs.putUChar("day", currentDay);
  prefs.putUChar("hour", currentHour);
  prefs.putUChar("minute", currentMinute);
  prefs.putUChar("second", currentSecond);
  prefs.putUChar("idx7", current7DayIndex);
  prefs.putUShort("idxYear", currentYearDayIndex);
  prefs.putDouble("dliMol", dliMol);
  prefs.putFloat("parMax", parDayMax);
  prefs.putFloat("parMin", parDayMin);
  prefs.putBool("parMinOk", parDayMinValid);
  prefs.putBytes("dli7", dli7Days, sizeof(dli7Days));
  prefs.putBytes("dliYear", dliYear, sizeof(dliYear));
  prefs.putBytes("dliValid", dliYearValid, sizeof(dliYearValid));
  // Only the current day's 30-minute PPFD bins are stored.
  // They are cleared automatically at the next day change.
  prefs.putBytes("par30sum", parHalfHourWeightedSum, sizeof(parHalfHourWeightedSum));
  prefs.putBytes("par30sec", parHalfHourSeconds, sizeof(parHalfHourSeconds));
  prefs.end();

  storageDirty = false;
  lastStorageSaveMs = millis();
  lastManualSaveMs = lastStorageSaveMs;
}

void loadPersistentData() {
  if (!prefs.begin("flora", true)) return;
  uint32_t magic = prefs.getUInt("magic", 0);
  if (magic == 0x464C5851) {
    dateIsSet = prefs.getBool("dateSet", false);
    timeIsSet = prefs.getBool("timeSet", false);
    currentYear = prefs.getUShort("year", currentYear);
    currentMonth = prefs.getUChar("month", currentMonth);
    currentDay = prefs.getUChar("day", currentDay);
    currentHour = prefs.getUChar("hour", currentHour);
    currentMinute = prefs.getUChar("minute", currentMinute);
    currentSecond = prefs.getUChar("second", currentSecond);
    current7DayIndex = prefs.getUChar("idx7", current7DayIndex);
    currentYearDayIndex = prefs.getUShort("idxYear", currentYearDayIndex);
    dliMol = prefs.getDouble("dliMol", dliMol);
    parDayMax = prefs.getFloat("parMax", parDayMax);
    parDayMin = prefs.getFloat("parMin", parDayMin);
    parDayMinValid = prefs.getBool("parMinOk", parDayMinValid);

    if (prefs.getBytesLength("dli7") == sizeof(dli7Days)) prefs.getBytes("dli7", dli7Days, sizeof(dli7Days));
    if (prefs.getBytesLength("dliYear") == sizeof(dliYear)) prefs.getBytes("dliYear", dliYear, sizeof(dliYear));
    if (prefs.getBytesLength("dliValid") == sizeof(dliYearValid)) prefs.getBytes("dliValid", dliYearValid, sizeof(dliYearValid));
    if (prefs.getBytesLength("par30sum") == sizeof(parHalfHourWeightedSum)) prefs.getBytes("par30sum", parHalfHourWeightedSum, sizeof(parHalfHourWeightedSum));
    if (prefs.getBytesLength("par30sec") == sizeof(parHalfHourSeconds)) prefs.getBytes("par30sec", parHalfHourSeconds, sizeof(parHalfHourSeconds));

    // Aktuellen 30-Minuten-Bin aus dem Schnellspeicher wiederherstellen.
    // So geht der heutige PPFD-Verlauf nach Ein-/Ausschalten nicht verloren.
    uint8_t qIdx = prefs.getUChar("qParIdx", 255);
    if (qIdx < PAR_24H_BINS) {
      parHalfHourWeightedSum[qIdx] = prefs.getDouble("qParSum", parHalfHourWeightedSum[qIdx]);
      parHalfHourSeconds[qIdx] = prefs.getDouble("qParSec", parHalfHourSeconds[qIdx]);
    }

    // Safety checks in case stored values are corrupted or from an older sketch.
    if (current7DayIndex > 6) current7DayIndex = 0;
    if (currentYearDayIndex >= YEAR_DAYS) currentYearDayIndex = 0;
    if (currentMonth < 1 || currentMonth > 12) currentMonth = 1;
    if (currentDay < 1 || currentDay > daysInMonth(currentYear, currentMonth)) currentDay = 1;
    if (currentHour > 23) currentHour = 0;
    if (currentMinute > 59) currentMinute = 0;
    if (currentSecond > 59) currentSecond = 0;

    // Den zuletzt schnell gespeicherten aktuellen DLI-Wert wieder in Kalender/7-Tage-Ansicht spiegeln.
    dli7Days[current7DayIndex] = dliMol;
    dliYear[currentYearDayIndex] = dliMol;
    dliYearValid[currentYearDayIndex] = true;

    storageLoaded = true;
    storageDirty = false;
  }
  prefs.end();
}

void clearPersistentData() {
  if (prefs.begin("flora", false)) {
    prefs.clear();
    prefs.end();
  }

  dliMol = 0.0;
  parDayMax = 0.0f;
  parDayMin = 0.0f;
  parDayMinValid = false;
  for (int i = 0; i < 24; i++) dliHour[i] = 0.0;
  for (int i = 0; i < PAR_24H_BINS; i++) {
    parHalfHourWeightedSum[i] = 0.0;
    parHalfHourSeconds[i] = 0.0;
  }
  clearPPFDStdAnalysis();
  for (int i = 0; i < 7; i++) dli7Days[i] = 0.0;
  current7DayIndex = 0;
  clearYearHistory();
  currentYearDayIndex = dateIsSet ? dayOfYearIndex(currentYear, currentMonth, currentDay) : 0;
  dliYear[currentYearDayIndex] = 0.0;
  dliYearValid[currentYearDayIndex] = true;
  storageDirty = false;
  fullStorageSaveRequested = false;
  lastHalfHourIndexForStorage = 255;
  lastStorageSaveMs = millis();
}

bool parseDateString(const String& date, uint16_t& y, uint8_t& m, uint8_t& d) {
  if (date.length() != 10) return false;
  if (date.charAt(4) != '-' || date.charAt(7) != '-') return false;

  y = (uint16_t)date.substring(0, 4).toInt();
  m = (uint8_t)date.substring(5, 7).toInt();
  d = (uint8_t)date.substring(8, 10).toInt();

  if (y < 2020 || y > 2099) return false;
  if (m < 1 || m > 12) return false;
  if (d < 1 || d > daysInMonth(y, m)) return false;
  return true;
}

bool parseDateTimeString(const String& dt, uint16_t& y, uint8_t& m, uint8_t& d, uint8_t& hh, uint8_t& mm, uint8_t& ss) {
  // Expected from HTML datetime-local: YYYY-MM-DDTHH:MM or YYYY-MM-DDTHH:MM:SS
  if (dt.length() < 16) return false;
  if (dt.charAt(4) != '-' || dt.charAt(7) != '-') return false;
  char sep = dt.charAt(10);
  if (sep != 'T' && sep != ' ') return false;
  if (dt.charAt(13) != ':') return false;

  String datePart = dt.substring(0, 10);
  if (!parseDateString(datePart, y, m, d)) return false;

  hh = (uint8_t)dt.substring(11, 13).toInt();
  mm = (uint8_t)dt.substring(14, 16).toInt();
  ss = 0;
  if (dt.length() >= 19 && dt.charAt(16) == ':') ss = (uint8_t)dt.substring(17, 19).toInt();

  if (hh > 23 || mm > 59 || ss > 59) return false;
  return true;
}

void incrementCalendarDate() {
  if (!dateIsSet) return;

  currentDay++;
  if (currentDay > daysInMonth(currentYear, currentMonth)) {
    currentDay = 1;
    currentMonth++;
    if (currentMonth > 12) {
      currentMonth = 1;
      currentYear++;
      clearYearHistory();
    }
  }
}

void advanceDayIfNeeded(uint32_t nowMs) {
  const uint32_t DAY_MS = 24UL * 3600000UL;

  while ((uint32_t)(nowMs - dayStartMs) >= DAY_MS) {
    // Save the finished day.
    dli7Days[current7DayIndex] = dliMol;
    dliYear[currentYearDayIndex] = dliMol;
    dliYearValid[currentYearDayIndex] = true;

    // Move to next day.
    current7DayIndex = (current7DayIndex + 1) % 7;

    if (dateIsSet) {
      incrementCalendarDate();
      currentYearDayIndex = dayOfYearIndex(currentYear, currentMonth, currentDay);
    } else {
      currentYearDayIndex = (currentYearDayIndex + 1) % 365;
    }

    dliMol = 0.0;
    parDayMax = 0.0f;
    parDayMin = 0.0f;
    parDayMinValid = false;
    dli7Days[current7DayIndex] = 0.0;
    dliYear[currentYearDayIndex] = 0.0;
    dliYearValid[currentYearDayIndex] = true;

    for (int i = 0; i < 24; i++) dliHour[i] = 0.0;
    for (int i = 0; i < PAR_24H_BINS; i++) {
      parHalfHourWeightedSum[i] = 0.0;
      parHalfHourSeconds[i] = 0.0;
    }
    lastHalfHourIndexForStorage = 255;
    fullStorageSaveRequested = true;

    dayStartMs += DAY_MS;

    // Save immediately when a day is completed, so daily DLI values survive power loss.
    markStorageDirty();
    savePersistentData();
  }
}

void addFinishedMeasurementToHistory(uint32_t nowMs, double par, double elapsed_s) {
  advanceDayIfNeeded(nowMs);

  if (!parDayMinValid) {
    parDayMin = (float)par;
    parDayMax = (float)par;
    parDayMinValid = true;
  } else {
    if (par > parDayMax) parDayMax = (float)par;
    if (par < parDayMin) parDayMin = (float)par;
  }

  double dliInc = par * elapsed_s / 1e6;
  dliMol += dliInc;

  uint32_t dayElapsedMs = nowMs - dayStartMs;
  const uint32_t HOUR_MS = 3600000UL;
  const uint32_t HALF_HOUR_MS = 1800000UL;
  uint8_t hourIndex = (uint8_t)((dayElapsedMs / HOUR_MS) % 24);
  uint8_t halfHourIndex = (uint8_t)((dayElapsedMs / HALF_HOUR_MS) % PAR_24H_BINS);

  if (lastHalfHourIndexForStorage == 255) {
    lastHalfHourIndexForStorage = halfHourIndex;
  } else if (halfHourIndex != lastHalfHourIndexForStorage) {
    // Beim Wechsel des 30-Minuten-Bins einmal komplett speichern, damit fertige PPFD-Punkte sicher bleiben.
    lastHalfHourIndexForStorage = halfHourIndex;
    fullStorageSaveRequested = true;
  }

  dliHour[hourIndex] += dliInc;
  parHalfHourWeightedSum[halfHourIndex] += par * elapsed_s;
  parHalfHourSeconds[halfHourIndex] += elapsed_s;

  // Keep current day values live on the website.
  dli7Days[current7DayIndex] = dliMol;
  dliYear[currentYearDayIndex] = dliMol;
  dliYearValid[currentYearDayIndex] = true;
  markStorageDirty();
}

void addPPFDToStdAnalysis(float ppfd) {
  if (!ppfdStdActive) return;
  if (ppfdStdResultCount >= PPFD_STD_RESULTS) {
    ppfdStdActive = false;
    return;
  }

  ppfdStdBuffer[ppfdStdBufferCount] = ppfd;
  ppfdStdBufferCount++;

  if (ppfdStdBufferCount >= PPFD_STD_WINDOW) {
    double sum = 0.0;
    for (uint8_t i = 0; i < PPFD_STD_WINDOW; i++) sum += ppfdStdBuffer[i];
    double mean = sum / (double)PPFD_STD_WINDOW;

    double sq = 0.0;
    for (uint8_t i = 0; i < PPFD_STD_WINDOW; i++) {
      double diff = (double)ppfdStdBuffer[i] - mean;
      sq += diff * diff;
    }

    // Sample standard deviation, suitable for evaluation of a finite measurement series.
    float stdValue = (float)sqrt(sq / (double)(PPFD_STD_WINDOW - 1));

    ppfdStdMeans[ppfdStdResultCount] = (float)mean;
    ppfdStdValues[ppfdStdResultCount] = stdValue;
    ppfdStdCounts[ppfdStdResultCount] = PPFD_STD_WINDOW;
    ppfdStdValid[ppfdStdResultCount] = true;
    ppfdStdResultCount++;

    ppfdStdBufferCount = 0;
    for (uint8_t i = 0; i < PPFD_STD_WINDOW; i++) ppfdStdBuffer[i] = 0.0f;

    if (ppfdStdResultCount >= PPFD_STD_RESULTS) {
      ppfdStdActive = false;
    }
  }
}

String buildJsonData() {
  String json;
  json.reserve(16000);

  json += "{";
  json += "\"par\":" + String(parUmol, 1) + ",";
  json += "\"dli\":" + String(dliMol, 3) + ",";
  json += "\"dateSet\":";
  json += (dateIsSet ? "true" : "false");
  json += ",";
  json += "\"date\":\"" + currentDateString() + "\",";
  json += "\"time\":\"" + currentTimeString() + "\",";
  json += "\"datetime\":\"" + currentDateTimeString() + "\",";
  json += "\"timeSet\":";
  json += (timeIsSet ? "true" : "false");
  json += ",";
  json += "\"dateTimeSet\":";
  json += ((dateIsSet && timeIsSet) ? "true" : "false");
  json += ",";
  json += "\"parMax\":" + String(parDayMax, 1) + ",";
  json += "\"parMin\":" + String(parDayMin, 1) + ",";
  json += "\"parMinValid\":";
  json += (parDayMinValid ? "true" : "false");
  json += ",";
  json += "\"todayIndex\":" + String(currentYearDayIndex) + ",";
  json += "\"month\":" + String(currentMonth) + ",";
  json += "\"storageLoaded\":";
  json += (storageLoaded ? "true" : "false");
  json += ",";
  json += "\"storageDirty\":";
  json += (storageDirty ? "true" : "false");
  json += ",";

  json += "\"monthDays\":[";
  for (int m = 1; m <= 12; m++) {
    if (m > 1) json += ",";
    json += String(daysInMonth(currentYear, (uint8_t)m));
  }
  json += "],";

  json += "\"par24\":[";
  for (int i = 0; i < PAR_24H_BINS; i++) {
    if (i) json += ",";
    if (parHalfHourSeconds[i] > 0.0) {
      double avgPar = parHalfHourWeightedSum[i] / parHalfHourSeconds[i];
      json += String(avgPar, 1);
    } else {
      json += "null";
    }
  }
  json += "],";

  json += "\"dli7\":[";
  for (int i = 0; i < 7; i++) {
    if (i) json += ",";
    uint8_t idx = (current7DayIndex + 1 + i) % 7; // oldest -> current
    json += String(dli7Days[idx], 3);
  }
  json += "],";

  json += "\"year\":[";
  for (uint16_t i = 0; i < YEAR_DAYS; i++) {
    if (i) json += ",";
    if (dliYearValid[i] || i == currentYearDayIndex) {
      json += String(dliYear[i], 3);
    } else {
      json += "0";
    }
  }
  json += "],";

  json += "\"yearValid\":[";
  for (uint16_t i = 0; i < YEAR_DAYS; i++) {
    if (i) json += ",";
    json += ((dliYearValid[i] || i == currentYearDayIndex) ? "true" : "false");
  }
  json += "],";

  json += "\"stdPending\":";
  json += (ppfdStdPending ? "true" : "false");
  json += ",";
  json += "\"stdActive\":";
  json += (ppfdStdActive ? "true" : "false");
  json += ",";
  json += "\"stdCurrentCount\":" + String(ppfdStdBufferCount) + ",";
  json += "\"stdResultCount\":" + String(ppfdStdResultCount) + ",";

  json += "\"stdValues\":[";
  for (uint8_t i = 0; i < PPFD_STD_RESULTS; i++) {
    if (i) json += ",";
    if (ppfdStdValid[i]) json += String(ppfdStdValues[i], 2);
    else json += "null";
  }
  json += "],";

  json += "\"stdMeans\":[";
  for (uint8_t i = 0; i < PPFD_STD_RESULTS; i++) {
    if (i) json += ",";
    if (ppfdStdValid[i]) json += String(ppfdStdMeans[i], 2);
    else json += "null";
  }
  json += "],";

  json += "\"stdCounts\":[";
  for (uint8_t i = 0; i < PPFD_STD_RESULTS; i++) {
    if (i) json += ",";
    json += String(ppfdStdValid[i] ? ppfdStdCounts[i] : 0);
  }
  json += "],";

  json += "\"stdValid\":[";
  for (uint8_t i = 0; i < PPFD_STD_RESULTS; i++) {
    if (i) json += ",";
    json += (ppfdStdValid[i] ? "true" : "false");
  }
  json += "]";
  json += "}";
  return json;
}


void applyDateTime(uint16_t y, uint8_t m, uint8_t d, uint8_t hh, uint8_t mm, uint8_t ss) {
  bool hadDate = dateIsSet;
  uint16_t oldYear = currentYear;

  dateIsSet = true;
  timeIsSet = true;
  currentYear = y;
  currentMonth = m;
  currentDay = d;
  currentHour = hh;
  currentMinute = mm;
  currentSecond = ss;

  // Align the 24h DLI cycle with the manually entered clock time.
  alignDayStartToStoredTime(millis());

  // If the date is set for the first time or the year changes, old boot-relative
  // calendar values are cleared to avoid showing them under wrong real dates.
  if (!hadDate || oldYear != currentYear) {
    clearYearHistory();
  }

  currentYearDayIndex = dayOfYearIndex(currentYear, currentMonth, currentDay);
  dliYear[currentYearDayIndex] = dliMol;
  dliYearValid[currentYearDayIndex] = true;
}

void handleSetDateTime() {
  if (!server.hasArg("dt")) {
    server.send(400, "text/plain", "Datum/Uhrzeit fehlt.");
    return;
  }

  uint16_t y;
  uint8_t m, d, hh, mm, ss;
  String dt = server.arg("dt");

  if (!parseDateTimeString(dt, y, m, d, hh, mm, ss)) {
    server.send(400, "text/plain", "Ungueltiges Format. Erwartet: YYYY-MM-DDTHH:MM.");
    return;
  }

  applyDateTime(y, m, d, hh, mm, ss);
  markStorageDirty();
  savePersistentData();

  server.send(200, "text/plain", "Datum und Uhrzeit gesetzt: " + currentDateTimeString() + ".");
}

// Compatibility endpoint: if only a date is supplied, time is set to 00:00.
void handleSetDate() {
  if (!server.hasArg("date")) {
    server.send(400, "text/plain", "Datum fehlt.");
    return;
  }

  uint16_t y;
  uint8_t m, d;
  String date = server.arg("date");
  if (!parseDateString(date, y, m, d)) {
    server.send(400, "text/plain", "Ungueltiges Datum. Format: YYYY-MM-DD.");
    return;
  }

  applyDateTime(y, m, d, 0, 0, 0);
  markStorageDirty();
  savePersistentData();
  server.send(200, "text/plain", "Datum gesetzt: " + currentDateTimeString() + ".");
}

void resetTodayData() {
  dliMol = 0.0;
  parDayMax = 0.0f;
  parDayMin = 0.0f;
  parDayMinValid = false;
  for (int i = 0; i < 24; i++) dliHour[i] = 0.0;
  for (int i = 0; i < PAR_24H_BINS; i++) {
    parHalfHourWeightedSum[i] = 0.0;
    parHalfHourSeconds[i] = 0.0;
  }
  clearPPFDStdAnalysis();
  dli7Days[current7DayIndex] = 0.0;
  dliYear[currentYearDayIndex] = 0.0;
  dliYearValid[currentYearDayIndex] = true;
  fullStorageSaveRequested = true;
  lastHalfHourIndexForStorage = 255;
  markStorageDirty();
}

void handleResetToday() {
  resetTodayData();
  savePersistentData();
  server.send(200, "text/plain", "Heutiger DLI-Tageswert wurde zurueckgesetzt.");
}

String dateForYearIndex(uint16_t idx) {
  if (!dateIsSet) return "Tag " + String(idx + 1);
  uint8_t m = 1;
  uint16_t day = idx + 1;
  while (m <= 12) {
    uint8_t dm = daysInMonth(currentYear, m);
    if (day <= dm) break;
    day -= dm;
    m++;
  }
  return String(currentYear) + "-" + twoDigits(m) + "-" + twoDigits((uint8_t)day);
}

void handleCsvExport() {
  String csv;
  csv.reserve(12000);
  csv += "Datum;Tag_im_Jahr;DLI_mol_m2_d;Gueltig\n";
  uint16_t count = dateIsSet ? daysInYear(currentYear) : 366;
  for (uint16_t i = 0; i < count && i < YEAR_DAYS; i++) {
    bool valid = dliYearValid[i] || i == currentYearDayIndex;
    csv += dateForYearIndex(i);
    csv += ";";
    csv += String(i + 1);
    csv += ";";
    csv += String(valid ? dliYear[i] : 0.0, 3);
    csv += ";";
    csv += (valid ? "1" : "0");
    csv += "\n";
  }
  server.sendHeader("Content-Disposition", "attachment; filename=flora_flux_dli.csv");
  server.send(200, "text/csv; charset=utf-8", csv);
}

void handleStartStdAnalysis() {
  requestPPFDStdAnalysisStart();
  server.send(200, "text/plain", "Standardabweichung startet ab dem naechsten Reset/Messfenster. Alte Werte wurden geloescht.");
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  server.send(200, "application/json", buildJsonData());
}

void handleSaveNow() {
  savePersistentData();
  server.send(200, "text/plain", "DLI-Daten wurden gespeichert.");
}

void handleClearStorage() {
  clearPersistentData();
  savePersistentData();
  server.send(200, "text/plain", "Gespeicherte DLI-Daten wurden geloescht.");
}

void startWebsite() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setdate", handleSetDate);
  server.on("/setdatetime", handleSetDateTime);
  server.on("/save", handleSaveNow);
  server.on("/resettoday", handleResetToday);
  server.on("/export/dli.csv", handleCsvExport);
  server.on("/startstd", handleStartStdAnalysis);
  server.on("/clear", handleClearStorage);
  server.begin();

  Serial.println("Website bereit");
  Serial.print("WiFi: ");
  Serial.println(AP_SSID);
  Serial.print("Passwort: ");
  Serial.println(AP_PASS);
  Serial.println("Oeffne: http://192.168.4.1");
}

// ---------- Measurement window reset helper ----------
// The paper formula assumes that each measurement window starts from the reset/bias voltage.
// Therefore we actively reset the integrator at the beginning of every 5-second window.
// This makes Vres well-defined and avoids over-counting when a window starts in the middle
// of a capacitor ramp.
void forceIntegratorResetForMeasurement() {
  digitalWrite(RESET_PIN, HIGH);
  delayMicroseconds(RESET_PULSE_US);
  digitalWrite(RESET_PIN, LOW);

  resetActive = false;
  resetEndMicros = 0;
}

void serviceResetPulseEnd() {
  // Diese Funktion muss sehr frueh und regelmaessig aufgerufen werden.
  // Dadurch wird der Reset-Puls moeglichst nah an 2 ms beendet, bevor OLED,
  // Website oder Speicherfunktionen den loop() kurz blockieren koennen.
  if (resetActive) {
    uint32_t now = micros();
    if ((int32_t)(now - resetEndMicros) >= 0) {
      digitalWrite(RESET_PIN, LOW);
      resetActive = false;
    }
  }
}

void clearWindowPulseCounter(uint32_t nowUsForDeadTime) {
  noInterrupts();
  pulseCountWindow = 0;
  pulseTargetMicros = 0;
  lastCountMicros = nowUsForDeadTime;
  interrupts();
}

float readIntegratorVoltageA0() {
  int16_t raw = ads.readADC_SingleEnded(0);
  return raw * LSB_V;
}

void startNewMeasurementWindowClean() {
  // Fuer die PPFD-Berechnung soll jedes Messfenster definiert nach einem Reset starten.
  // Danach werden alle Pulse geloescht, die waehrend des erzwungenen Resets entstanden sein koennten.
  forceIntegratorResetForMeasurement();
  clearWindowPulseCounter(micros());

  windowStartMs = millis();
  windowStartMicros = micros();
  windowStartVin = readIntegratorVoltageA0();
  windowStartValid = true;

  // Sicherheit: Falls waehrend der ADC-Startmessung eine Stoerflanke kam, wird sie nicht dem neuen Fenster zugerechnet.
  clearWindowPulseCounter(micros());
}

void resetHighLightAverage() {
  highLightAvgCount = 0;
  highLightAvgIndex = 0;
  for (uint8_t i = 0; i < HIGH_LIGHT_AVG_WINDOW; i++) {
    highLightAvgBuffer[i] = 0.0f;
  }
}

float addHighLightAverage(float value) {
  highLightAvgBuffer[highLightAvgIndex] = value;
  highLightAvgIndex = (highLightAvgIndex + 1) % HIGH_LIGHT_AVG_WINDOW;

  if (highLightAvgCount < HIGH_LIGHT_AVG_WINDOW) {
    highLightAvgCount++;
  }

  double sum = 0.0;
  for (uint8_t i = 0; i < highLightAvgCount; i++) {
    sum += highLightAvgBuffer[i];
  }

  return (float)(sum / (double)highLightAvgCount);
}

// ---------- ISR: Komparator-Flanke ----------
// Bei jeder Komparator-Flanke wird gezählt und der Integrator kurz entladen.

IRAM_ATTR void onComparatorRise() {
  uint32_t now = micros();

  // nur zählen, wenn seit letztem Puls etwas Zeit verging
  if (now - lastCountMicros > MIN_PULSE_INTERVAL_US) {
    lastCountMicros  = now;

    // Reset sofort aktivieren, falls noch nicht aktiv
    if (!resetActive) {
      resetActive    = true;
      resetEndMicros = now + RESET_PULSE_US;
      digitalWrite(RESET_PIN, HIGH);
    }

    pulseCountTotal++;
    pulseCountWindow++;

    // Sobald die Zielpulszahl erreicht ist, den exakten Zeitpunkt merken.
    // Spaeter im loop() wird damit die Zeit bis zum 150. Puls berechnet,
    // nicht die zufaellige Zeit bis der loop() wieder an dieser Stelle ankommt.
    if (pulseCountWindow == HIGH_LIGHT_PULSES && pulseTargetMicros == 0) {
      pulseTargetMicros = now;
    }
  }
}


// ---------- OLED startup logo ----------

void drawCenteredText(const char* text, int y) {
  int w = u8g2.getStrWidth(text);
  int x = (128 - w) / 2;
  if (x < 0) x = 0;
  u8g2.drawStr(x, y, text);
}

void showStartupLogo() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_7x14B_tf);
  drawCenteredText("FloraFlux", 24);

  u8g2.setFont(u8g2_font_6x10_tf);
  drawCenteredText("PPFD & DLI Sensor", 41);
  drawCenteredText("FloraFlux Sensor", 56);

  u8g2.sendBuffer();
  delay(2000);
}

// ---------- Setup ----------

void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C
  Wire.begin(6, 7);
  Wire.setClock(400000);

  u8g2.begin();
  u8g2.setBusClock(400000);
  showStartupLogo();

  if (!ads.begin(0x48)) {
    Serial.println("ADS1115 nicht gefunden (0x48)!");
    while (1) { delay(10); }
  }

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  pinMode(COMP_PIN, INPUT);           // ggf. INPUT_PULLDOWN je nach Schaltung
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);  // Button -> 3V3

  // Start with a clean integrator before the interrupt counter begins.
  forceIntegratorResetForMeasurement();
  clearWindowPulseCounter(micros());

  attachInterrupt(digitalPinToInterrupt(COMP_PIN), onComparatorRise, RISING);

  uint32_t nowMs = millis();
  windowStartMs     = nowMs;
  windowStartMicros = micros();
  windowStartVin    = readIntegratorVoltageA0();
  windowStartValid  = true;
  dayStartMs        = nowMs;
  clearWindowPulseCounter(micros());

  // Load saved DLI/date data from ESP32 flash storage.
  // This does not change the measurement method.
  loadPersistentData();
  if (timeIsSet) alignDayStartToStoredTime(nowMs);
  // The lightweight save stores the latest current-day DLI. Mirror it back into
  // the website histories after boot, so the calendar shows the value again.
  dli7Days[current7DayIndex] = dliMol;
  dliYear[currentYearDayIndex] = dliMol;
  dliYearValid[currentYearDayIndex] = true;
  lastHalfHourIndexForStorage = currentHalfHourIndexFromMillis(nowMs);
  lastStorageSaveMs = nowMs;

  startWebsite();

  Serial.println("System bereit: paper formula, 5s normal window + 150-pulse high-light mode + 5-value high-light average, Vthr=2.5V, Cf=4.7uF, PAR_CAL=7.5, website active, manual date/time setting + CSV export + manual PPFD standard deviation table available");
}

// ---------- Loop ----------

void loop() {
  // 1) Messung hat Prioritaet: Reset-Pulse zuerst beenden.
  // Dadurch wird verhindert, dass OLED, Website oder Speicherzugriffe einen Reset-Puls verlaengern.
  serviceResetPulseEnd();

  uint32_t nowMs = millis();

  // 2) A0 frueh lesen, bevor langsame Aufgaben bearbeitet werden.
  int16_t rawVin = ads.readADC_SingleEnded(0);
  float Vin = rawVin * LSB_V;
  serviceResetPulseEnd();

  if (!windowStartValid) {
    startNewMeasurementWindowClean();
    nowMs = millis();
  }

  // ---- PPFD-Messfenster pruefen ----
  uint32_t elapsedMs = nowMs - windowStartMs;

  uint32_t currentWindowPulses;
  uint32_t targetMicrosSnapshot;
  noInterrupts();
  currentWindowPulses = pulseCountWindow;
  targetMicrosSnapshot = pulseTargetMicros;
  interrupts();

  bool finishMeasurementWindow = false;

  if (highLightPulseMode) {
    // Hochlicht: genauere Zeitmessung ueber den Zeitpunkt des 150. Pulses.
    if (currentWindowPulses >= HIGH_LIGHT_PULSES || elapsedMs >= HIGH_LIGHT_MAX_MS) {
      finishMeasurementWindow = true;
    }
  } else {
    // Normal: 5-s-Fenster. Falls sehr schnell 150 Pulse kommen, sofort auf Pulsfenster auswerten.
    if (elapsedMs >= MEAS_WINDOW_MS || currentWindowPulses >= HIGH_LIGHT_PULSES) {
      finishMeasurementWindow = true;
    }
  }

  if (finishMeasurementWindow) {
    // Alle Werte fuer das beendete Fenster sichern.
    float endVin = Vin;
    float startVinForCalc = windowStartVin;
    uint32_t startMsForCalc = windowStartMs;
    uint32_t startUsForCalc = windowStartMicros;

    uint32_t nWindowRaw;
    uint32_t targetUsForCalc;
    noInterrupts();
    nWindowRaw = pulseCountWindow;
    targetUsForCalc = pulseTargetMicros;
    // Zaehler sofort freigeben/loeschen, damit alte Pulse nicht in das naechste Fenster rutschen.
    pulseCountWindow = 0;
    pulseTargetMicros = 0;
    interrupts();

    // Direkt ein neues sauberes Messfenster starten, bevor Website/OLED/Speicher bearbeitet werden.
    // So koennen langsame Aufgaben das Ende des alten Fensters nicht mehr verfaelschen.
    startNewMeasurementWindowClean();
    serviceResetPulseEnd();

    bool fixedPulseCalculation = (targetUsForCalc != 0 && nWindowRaw >= HIGH_LIGHT_PULSES);
    uint32_t nWindow = nWindowRaw;
    double elapsed_s;

    if (fixedPulseCalculation) {
      // Im Hochlichtbereich wird die echte Zeit bis zum 150. Puls verwendet.
      // Dadurch haengt die Berechnung nicht davon ab, wann loop() zufaellig wieder ausgefuehrt wird.
      uint32_t elapsedUs = (uint32_t)(targetUsForCalc - startUsForCalc);
      elapsed_s = (double)elapsedUs / 1000000.0;
      nWindow = HIGH_LIGHT_PULSES;
    } else {
      elapsed_s = (double)(nowMs - startMsForCalc) / 1000.0;
    }

    if (elapsed_s < 0.001) elapsed_s = 0.001;

    float signedDeltaV = endVin - startVinForCalc;
    float absDeltaV = signedDeltaV;
    if (absDeltaV < 0.0f) absDeltaV = -absDeltaV;

    double equivalentVoltage = 0.0;
    bool darkWindow = false;

    if (nWindow > 0) {
      if (fixedPulseCalculation) {
        // Optimierte Hochlicht-Berechnung:
        // Bei sehr vielen Pulsen ist die ADC-Restspannung am Fensterende nicht mehr die stabile Groesse.
        // Deshalb wird im Pulsmodus nur die Pulsfrequenz benutzt:
        // V_eq = N * (V_thr - V_bias).
        // Das vermeidet, dass kleine ADC-Fehler bei Vstart mit 150 multipliziert werden.
        double vSwingFixed = (double)V_THR - (double)V_BIAS;
        if (vSwingFixed < 0.0) vSwingFixed = 0.0;
        equivalentVoltage = (double)nWindow * vSwingFixed;
      } else {
        // Normaler Bereich: Paper-Formel mit Restspannung am Ende des Messfensters.
        double vSwing = (double)V_THR - (double)startVinForCalc;
        if (vSwing < 0.0) vSwing = 0.0;

        double vRes = (double)endVin - (double)startVinForCalc;
        if (vRes < 0.0) vRes = 0.0;

        equivalentVoltage = (double)nWindow * vSwing + vRes;
      }

    } else if (absDeltaV < DARK_ZERO_DELTA_V) {
      darkWindow = true;
      equivalentVoltage = 0.0;

    } else if (signedDeltaV > 0.0f) {
      // Schwachlicht ohne Reset: Steigung der Integratorspannung verwenden.
      equivalentVoltage = (double)signedDeltaV;

    } else {
      equivalentVoltage = 0.0;
    }

    double iph = (double)C_F * equivalentVoltage / elapsed_s;
    double qpar_raw_umol = iph * PAR_CONV_UMOL;
    double calibratedPar = qpar_raw_umol * (double)PAR_CAL;

    parRawUmol = (float)calibratedPar;

    if (darkWindow) {
      parRawUmol = 0.0f;
      parUmol = 0.0f;
      resetHighLightAverage();
    } else {
      bool strongLightResult = fixedPulseCalculation ||
                               highLightPulseMode ||
                               (parRawUmol >= HIGH_LIGHT_ENTER_PPFD) ||
                               (nWindow >= HIGH_LIGHT_PULSES);

      if (strongLightResult) {
        parUmol = addHighLightAverage(parRawUmol);
      } else {
        resetHighLightAverage();
        parUmol = parRawUmol;
      }
    }

    parSmoothingReady = true;
    lastWindowPulses = nWindow;
    lastWindowMs = (uint32_t)(elapsed_s * 1000.0);

    // Moduswechsel fuer das naechste Fenster mit Rohwert, damit der Mittelwert den Wechsel nicht verzoegert.
    if (highLightPulseMode) {
      if (parRawUmol < HIGH_LIGHT_EXIT_PPFD || nWindow < HIGH_LIGHT_PULSES) {
        highLightPulseMode = false;
      }
    } else {
      if (parRawUmol >= HIGH_LIGHT_ENTER_PPFD || nWindow >= HIGH_LIGHT_PULSES) {
        highLightPulseMode = true;
      }
    }

    // DLI und Verlauf werden erst nach dem sauberen Neustart des Messfensters aktualisiert.
    addFinishedMeasurementToHistory(nowMs, (double)parUmol, elapsed_s);
    addPPFDToStdAnalysis(parUmol);

    // Wichtig fuer stabile Messung:
    // Keine Flash-/NVS-Schreiboperation direkt nach jedem Messfenster.
    // Diese Schreibzugriffe koennen kurz blockieren und dadurch Reset-Timing stoeren.
    // Die Daten werden stattdessen periodisch oder manuell ueber die Webseite gespeichert.
  }

  serviceResetPulseEnd();
  nowMs = millis();

  // 3) Speicher nur dann periodisch schreiben, wenn gerade kein Reset-Puls aktiv ist.
  // Bei Hochlicht wird Flash-Schreiben moeglichst vermieden, damit die Messung Vorrang hat.
  if (storageDirty && !resetActive && !highLightPulseMode &&
      (uint32_t)(nowMs - lastStorageSaveMs) >= STORAGE_SAVE_INTERVAL_MS) {
    savePersistentData();
  }

  serviceResetPulseEnd();

  // 4) Website bedienen. Das passiert bewusst nach der Messlogik.
  server.handleClient();

  serviceResetPulseEnd();

  // ---- Button mit Entprellung ----
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonReading) {
    lastDebounceTime = nowMs;
  }
  if ((nowMs - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;
      if (stableButtonState == HIGH) {
        displayMode = (displayMode + 1) % 3;  // 0 -> 1 -> 2 -> 0
      }
    }
  }
  lastButtonReading = reading;

  // ---- Serial Plotter optional, standardmaessig aus ----
  if (ENABLE_SERIAL_PLOTTER && (uint32_t)(nowMs - lastSerialPlotMs) >= SERIAL_PLOT_INTERVAL_MS) {
    lastSerialPlotMs = nowMs;
    Serial.println(Vin, 4);
  }

  serviceResetPulseEnd();

  // ---- OLED langsamer aktualisieren und Debug-Kanaele nur auf Debug-Seite lesen ----
  if ((uint32_t)(nowMs - lastSlowReadMs) >= SLOW_READ_INTERVAL_MS) {
    lastSlowReadMs = nowMs;

    if (displayMode == 1) {
      int16_t rawVout  = ads.readADC_SingleEnded(1);
      int16_t rawVthr  = ads.readADC_SingleEnded(2);
      int16_t rawVbias = ads.readADC_SingleEnded(3);

      Vout_disp  = rawVout  * LSB_V;
      Vthr_disp  = rawVthr  * LSB_V;
      Vbias_disp = rawVbias * LSB_V;
    }

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);

    if (displayMode == 0) {
      // ---- MODE 0: Clean user screen / Hauptanzeige ----
      char line[40];

      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 9, "PPFD");

      u8g2.setFont(u8g2_font_7x14B_tf);
      snprintf(line, sizeof(line), "%4.0f umol/m2/s", parUmol);
      u8g2.drawStr(0, 26, line);

      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 43, "DLI");

      u8g2.setFont(u8g2_font_7x14B_tf);
      snprintf(line, sizeof(line), "%4.1f mol/m2/d", dliMol);
      u8g2.drawStr(0, 61, line);

    } else if (displayMode == 1) {
      // ---- MODE 1: Debug-Seite ----
      u8g2.drawStr(0, 10, "MODE: Debug");

      char line[40];

      snprintf(line, sizeof(line), "Vin  (A0): %5.3f V", Vin);
      u8g2.drawStr(0, 22, line);

      snprintf(line, sizeof(line), "Vout (A1): %5.3f V", Vout_disp);
      u8g2.drawStr(0, 32, line);

      snprintf(line, sizeof(line), "Vthr (A2): %5.3f V", Vthr_disp);
      u8g2.drawStr(0, 42, line);

      snprintf(line, sizeof(line), "Vbias(A3): %5.3f V", Vbias_disp);
      u8g2.drawStr(0, 52, line);

      snprintf(line, sizeof(line), "n=%lu  PPFD=%6.0f",
               (unsigned long)lastWindowPulses, parUmol);
      u8g2.drawStr(0, 62, line);

    } else {
      // ---- MODE 2: Website-Link ----
      u8g2.setFont(u8g2_font_6x10_tf);
      drawCenteredText("Website", 9);
      drawCenteredText("192.168.4.1", 23);
      drawCenteredText("WLAN: FloraFlux-PAR", 41);
      drawCenteredText("PW: 12345678", 56);
    }

    u8g2.sendBuffer();
  }
}