# Proyecto-Factor-de-potencia

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cmath>

#define SDA_PIN 21       // Pin SDA OLED
#define SCL_PIN 22       // Pin SCL OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define N 100            // Número de muestras

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pines de los canales de entrada analógica
const int CUR_IN_PIN = 34; // Canal de corriente
const int VIN_PIN = 35;    // Canal de voltaje

// Variables para muestreo
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool readyf = false;
volatile uint16_t carr[N];
volatile uint16_t varr[N];

void IRAM_ATTR onTimer() {
    static uint16_t i = 0;
    if (i < N && !readyf) {
        carr[i] = analogRead(CUR_IN_PIN);
        varr[i] = analogRead(VIN_PIN);
        i++;
    } else if (!readyf) {
        readyf = true;
        i = 0;
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 167, true);
    timerAlarmEnable(timer);
    Serial.println("Iniciando monitoreo...");
}

void loop() {
    if (readyf) {
        timerAlarmDisable(timer);
        float cfil[N];
        float vfil[N];
        float corriente[N];
        float voltaje[N];
        // Filtrar señales
        filter(carr, cfil, N);
        filter(varr, vfil, N);
        // Filtro de media móvil
        applyMovingAverageFilter(cfil, corriente, N, 5);
        applyMovingAverageFilter(vfil, voltaje, N, 5);
        // Mostrar resultados
        mostrar(corriente, voltaje);
        readyf = false;
        timerAlarmEnable(timer);
    }
}

void filter(const volatile uint16_t uvol[], float result[], int size) {
    float b1 = 1.867, b2 = -0.8752;
    float a1 = -0.004247, a2 = -0.004063;
    result[0] = uvol[0];
    result[1] = uvol[1];
    for (int k = 2; k < size; k++) {
        result[k] = b1 * uvol[k - 1] + b2 * uvol[k - 2] - a1 * result[k - 1] - a2 * result[k - 2];
    }
}

void applyMovingAverageFilter(float *input, float *output, int length, int windowSize) {
    for (int i = 0; i < length; i++) {
        float sum = 0.0;
        int count = 0;
        for (int j = i; j > i - windowSize && j >= 0; j--) {
            sum += input[j];
            count++;
        }
        output[i] = sum / count;
    }
}

std::pair<float, float> encontrarExtremos(float senal[], int tamano) {
    float maximo = senal[0];
    float minimo = senal[0];
    for (int i = 1; i < tamano; ++i) {
        if (senal[i] > maximo) maximo = senal[i];
        if (senal[i] < minimo) minimo = senal[i];
    }
    return {maximo, minimo};
}

float calcularDesfase(float *senal1, float *senal2, int longitud) {
    double maxCorrelacion = -1.0;
    int mejorDesfase = 0;
    for (int desfase = 0; desfase < longitud; ++desfase) {
        double correlacion = 0.0;
        for (int i = 0; i < longitud; ++i) {
            correlacion += senal1[i] * senal2[(i + desfase) % longitud];
        }
        if (correlacion > maxCorrelacion) {
            maxCorrelacion = correlacion;
            mejorDesfase = desfase;
        }
    }
    return mejorDesfase * (360.0 / longitud); // Convertir desfase a grados
}

void mostrar(float corriente[], float voltaje[]) {
    auto extremosV = encontrarExtremos(voltaje, N);
    float v_peak = (((extremosV.first - extremosV.second) / 2.0) * 3.3 / 4095) * 808.12;
    auto extremosI = encontrarExtremos(corriente, N);
    float i_peak = (((extremosI.first - extremosI.second) / 2.0) * 3.3 / 4095) / 0.185;
    float i_rms = i_peak / sqrt(2.0);
    float v_rms = v_peak / sqrt(2.0);
    float desfase = calcularDesfase(voltaje, corriente, N);
    float FP = cos(desfase * PI / 180);  // Convertir desfase a radianes y calcular factor de potencia
    float potAparente = v_rms * i_rms;
    float potReal = potAparente * FP;
    float potReac = sqrt(potAparente * potAparente - potReal * potReal);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("Vpeak=%.2f\nIpeak=%.2f\nVrms=%.2f\nIrms=%.2f\n", v_peak, i_peak, v_rms, i_rms);
    display.printf("P=%.2f\nQ=%.2f\nS=%.2f\nFP=%.2f\nDesf=%.2f", potReal, potReac, potAparente, FP, desfase);
    display.display();
    delay(555);
}
