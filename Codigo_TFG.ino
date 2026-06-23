#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------------- SENSORES ----------------
MAX30105 sensorMAX;
Adafruit_MPU6050 mpu;

// ---------------- PULSO ----------------
long lastBeat = 0;
float bpm = 0;
float bpmProm = 0;
bool dedoDetectado = false;

// ---------------- MPU6050 ----------------
const float UMBRAL_IMPACTO = 2.2;  // impacto fuerte en g
const float UMBRAL_CAIDA_LIBRE = 0.65;  
const float UMBRAL_MOVIMIENTO_GIRO = 2.5; // movimiento tras caida

bool posibleCaida = false;
bool caidaDetectada = false;

unsigned long tiempoPosibleCaida = 0;
unsigned long tiempoCaida = 0;

const unsigned long VENTANA_IMPACTO = 1200;
const unsigned long TIEMPO_MIN_CAIDA = 3000; // ms

float ax = 0;
float ay = 0;
float az = 0;
float aceleracionTotal = 0;

// ---------------- TIEMPOS ----------------
const unsigned long intervaloMPU = 50;
const unsigned long intervaloPantalla = 700;
const unsigned long intervaloMuestraSpo2 = 100;
const unsigned long intervaloCalculoSpo2 = 30000;

unsigned long ultimoMPU = 0;
unsigned long ultimoRefreshPantalla = 0;


// ---------------- OXIGENO ----------------
#define BUFFER_SPO2 100

uint32_t irBuffer[BUFFER_SPO2];
uint32_t redBuffer[BUFFER_SPO2];

int32_t spo2 = 0;
int8_t spo2Valido = 0;
int32_t heartRateSpo2 = 0;
int8_t heartRateValido = 0;

byte indiceSpo2 = 0;
bool bufferSpo2Lleno = false;

unsigned long ultimoCalculoSpo2 = 0;
const unsigned long intervaloSpo2 = 4000;


// ---------------- WIFI / TELEGRAM ----------------
const char* WIFI_SSID = "iPhone de Sergio";
const char* WIFI_PASSWORD = "canamero";

const char* BOT_TOKEN = "8807347909:AAED--ZuKbuv3Iq4276h6R2NAU-O3XJI19g";
const char* CHAT_ID = "979538637";

WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

bool mensajeCaidaEnviado = false;
unsigned long ultimoIntentoTelegram = 0;
const unsigned long intervaloTelegram = 10000;



void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(50000);

  iniciarOLED();
  iniciarMAX30102();
  iniciarMPU6050();

  iniciarWiFi();

  Serial.println("Sistema listo");
}

void loop() {
  leerPulso();
  leerMPU();
  calcularSpo2();
  gestionarTelegram();
  actualizarPantalla();
}

// ---------------- INICIO OLED ----------------

void iniciarOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error OLED");
    while (1);
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(10, 20);
  display.println("Iniciando");
  display.display();

  delay(1500);
}

// ---------------- INICIO MAX30102 ----------------

void iniciarMAX30102() {
  if (!sensorMAX.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30102 no encontrado");

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println("MAX ERROR");
    display.display();

    while (1);
  }

  byte ledBrightness = 0x3F;
  byte sampleAverage = 4;
  byte ledMode = 2;       // Red + IR
  int sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 16384;

  sensorMAX.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  sensorMAX.setPulseAmplitudeRed(0x3F);
  sensorMAX.setPulseAmplitudeIR(0x3F);
  sensorMAX.setPulseAmplitudeGreen(0);

  Serial.println("MAX30102 iniciado");

  sensorMAX.clearFIFO();
}

// ---------------- INICIO MPU6050 ----------------

void iniciarMPU6050() {
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 no encontrado");

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println("MPU ERROR");
    display.display();

    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("MPU6050 iniciado");
}

// ---------------- LECTURA PULSO ----------------

void leerPulso() {
  long irValue = sensorMAX.getIR();

  static unsigned long ultimaMuestraSpo2 = 0;

  if (millis() - ultimaMuestraSpo2 >= intervaloMuestraSpo2 && !caidaDetectada) {
    ultimaMuestraSpo2 = millis();

    long redValue = sensorMAX.getRed();
    guardarMuestraSpo2(irValue, redValue);
  }

  dedoDetectado = irValue >= 50000;

  if (!dedoDetectado) {
    bpm = 0;
    bpmProm = 0;
    lastBeat = 0;
    return;
  }

  if (checkForBeat(irValue)) {
    unsigned long ahora = millis();

    if (lastBeat > 0) {
      unsigned long delta = ahora - lastBeat;
      float bpmCalculado = 60.0 / (delta / 1000.0);

      if (bpmCalculado >= 40 && bpmCalculado <= 160) {
        bpm = bpmCalculado;

        if (bpmProm == 0) {
          bpmProm = bpm;
        } else {
          bpmProm = (0.85 * bpmProm) + (0.15 * bpm);
        }

        Serial.print("IR: ");
        Serial.print(irValue);
        Serial.print(" | BPM: ");
        Serial.print(bpm);
        Serial.print(" | BPM prom: ");
        Serial.println(bpmProm);
      }
    }

    lastBeat = ahora;
  }
}

// ---------------- LECTURA MPU6050 ----------------

void leerMPU() {
  
  if (millis() - ultimoMPU < intervaloMPU) {
    return;
  }

  ultimoMPU = millis();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ax = a.acceleration.x / 9.81;
  ay = a.acceleration.y / 9.81;
  az = a.acceleration.z / 9.81;

  aceleracionTotal = sqrt((ax * ax) + (ay * ay) + (az * az));

  float giroTotal = sqrt(
    (g.gyro.x * g.gyro.x) +
    (g.gyro.y * g.gyro.y) +
    (g.gyro.z * g.gyro.z)
  );

  // 1. Posible caida libre
  if (!caidaDetectada && aceleracionTotal < UMBRAL_CAIDA_LIBRE) {
    posibleCaida = true;
    tiempoPosibleCaida = millis();
  }

  // 2. Impacto despues de posible caida
  if (!caidaDetectada && posibleCaida) {
    if (millis() - tiempoPosibleCaida <= VENTANA_IMPACTO) {
      if (aceleracionTotal > UMBRAL_IMPACTO || giroTotal > UMBRAL_MOVIMIENTO_GIRO) {
        caidaDetectada = true;
        posibleCaida = false;
        tiempoCaida = millis();

        Serial.println("CAIDA detectada");
      }
    } else {
      posibleCaida = false;
    }
  }

  // 3. Impacto directo, por si no se detecta la fase de caida libre
  if (!caidaDetectada && aceleracionTotal > UMBRAL_IMPACTO && giroTotal > 1.5) {
    caidaDetectada = true;
    tiempoCaida = millis();

    Serial.println("CAIDA detectada por impacto directo");
  }

  // 4. Si tras la caida hay movimiento, volvemos a pantalla normal
  if (caidaDetectada && millis() - tiempoCaida > TIEMPO_MIN_CAIDA) {
    if (giroTotal > 1.2 || abs(aceleracionTotal - 1.0) > 0.45) {
      caidaDetectada = false;
      Serial.println("Movimiento tras caida. Estado normal.");
    }
  }
}

// ---------------- OLED ----------------

void actualizarPantalla() {
  if (millis() - ultimoRefreshPantalla < intervaloPantalla) {
    return;
  }

  ultimoRefreshPantalla = millis();

  display.clearDisplay();

  // -------- PANTALLA DE CAIDA --------
  if (caidaDetectada) {
    display.fillScreen(WHITE);
    display.setTextColor(BLACK);

    display.setTextSize(2);
    display.setCursor(18, 12);
    display.println("!CAIDA");

    display.setCursor(10, 34);
    display.println("DETECTADA!");

    display.display();

    display.setTextColor(WHITE); // restaurar color normal
    return;
  }

  display.setTextColor(WHITE);

  // -------- SIN DEDO --------
  if (!dedoDetectado) {
    display.setTextSize(2);
    display.setCursor(10, 20);
    display.println("Pon dedo");

    display.setTextSize(1);
    display.setCursor(0, 55);
    display.print("Acc: ");
    display.print(aceleracionTotal, 2);
    display.print("g");

    display.display();
    return;
  }

  // -------- PANTALLA NORMAL --------

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Frecuencia cardiaca");

  // BPM grande
  display.setTextSize(4);
  display.setCursor(8, 16);

  if (bpmProm > 0) {
    display.print((int)bpmProm);
  } else {
    display.print("--");
  }

  display.setTextSize(2);
  display.setCursor(86, 28);
  display.print("BPM");

  // Linea inferior: SpO2 izquierda, aceleracion derecha
  display.setTextSize(1);

  display.setCursor(0, 56);
  display.print("SpO2:");

  if (spo2Valido && spo2 > 0 && spo2 <= 100) {
    display.print(spo2);
    display.print("%");
  } else {
    display.print("--%");
  }

  display.setCursor(74, 56);
  display.print("Acc:");
  display.print(aceleracionTotal, 1);
  display.print("g");

  display.display();
}

//-----------------------SPo2-------------------------
void guardarMuestraSpo2(long irValue, long redValue) {
  if (irValue < 50000) {
    indiceSpo2 = 0;
    bufferSpo2Lleno = false;
    spo2 = 0;
    spo2Valido = 0;
    return;
  }

  irBuffer[indiceSpo2] = irValue;
  redBuffer[indiceSpo2] = redValue;

  indiceSpo2++;

  if (indiceSpo2 >= BUFFER_SPO2) {
    indiceSpo2 = 0;
    bufferSpo2Lleno = true;
  }
}

void calcularSpo2() {
  if (!bufferSpo2Lleno) {
    return;
  }

  if (millis() - ultimoCalculoSpo2 < intervaloCalculoSpo2) {
  return;
  }

  ultimoCalculoSpo2 = millis();

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    BUFFER_SPO2,
    redBuffer,
    &spo2,
    &spo2Valido,
    &heartRateSpo2,
    &heartRateValido
  );

  Serial.print("SpO2: ");
  if (spo2Valido && spo2 > 0 && spo2 <= 100) {
    Serial.print(spo2);
    Serial.println(" %");
  } else {
    Serial.println("No valido");
  }
}

//-------------------------WiFi--------------------------------------
void iniciarWiFi() {
  Serial.print("Conectando a WiFi");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long inicio = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    secured_client.setInsecure(); // valido para prototipo TFG
  } else {
    Serial.println();
    Serial.println("No se pudo conectar a WiFi");
  }
}


//-----------------------TELEGRAM-------------------------------
void gestionarTelegram() {
  if (!caidaDetectada) {
    mensajeCaidaEnviado = false;
    return;
  }

  if (mensajeCaidaEnviado) {
    return;
  }

  if (millis() - ultimoIntentoTelegram < intervaloTelegram) {
    return;
  }

  ultimoIntentoTelegram = millis();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Reintentando...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    return;
  }

  String mensaje = "ALERTA: caida detectada.\n";
  mensaje += "BPM: ";

  if (bpmProm > 0) {
    mensaje += String((int)bpmProm);
  } else {
    mensaje += "sin dato";
  }

  mensaje += "\nSpO2: ";

  if (spo2Valido && spo2 > 0 && spo2 <= 100) {
    mensaje += String(spo2);
    mensaje += "%";
  } else {
    mensaje += "sin dato";
  }

  mensaje += "\nAceleracion: ";
  mensaje += String(aceleracionTotal, 2);
  mensaje += " g";

  bool enviado = bot.sendMessage(CHAT_ID, mensaje, "");

  if (enviado) {
    mensajeCaidaEnviado = true;
    Serial.println("Mensaje Telegram enviado");
  } else {
    Serial.println("Error enviando mensaje Telegram");
  }
}

