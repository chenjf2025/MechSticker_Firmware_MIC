#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "arduinoFFT.h"

/* ================== 基础配置 ================== */
#define ADC_PIN         4
#define SAMPLE_RATE     8000
#define FFT_SIZE        128     // 更适合瞬态
#define PUBLISH_INTERVAL 1000

/* ================== WiFi / MQTT ================== */
// Configure these values locally; never commit real network credentials.
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "YOUR_MQTT_HOST";

WiFiClient espClient;
PubSubClient client(espClient);

/* ================== FFT / Buffer ================== */
double vReal[FFT_SIZE];
double vImag[FFT_SIZE];
uint16_t adcBuf[FFT_SIZE];

arduinoFFT FFT(vReal, vImag, FFT_SIZE, SAMPLE_RATE);

/* ================== 声音基线 ================== */
struct Baseline {
    double rms = 0;
    double hf = 0;
    double peak = 0;
    bool ready = false;
    int count = 0;
} baseline;

/* ================== 工业阈值（可调） ================== */
#define KNOCK_FACTOR   4.0
#define VIB_FACTOR     1.8
#define HF_FACTOR      1.6

/* ================== 工具函数 ================== */
void connectMQTT() {
    if (!client.connected()) {
        if (client.connect("ESP32S3_AudioSentry")) {
            client.publish("device/status", "online");

        }
    }
}

/* ================== 核心音频处理 ================== */
void processAudio() {
    /* -------- 1. 采样 -------- */
    uint32_t tick = micros();
    double mean = 0;

    for (int i = 0; i < FFT_SIZE; i++) {
        adcBuf[i] = analogRead(ADC_PIN);
        mean += adcBuf[i];
        while (micros() < tick);
        tick += 125;
    }
    mean /= FFT_SIZE;

    /* -------- 2. 时间域特征 -------- */
    double energy = 0;
    double peak = 0;

    for (int i = 0; i < FFT_SIZE; i++) {
        double v = adcBuf[i] - mean;
        vReal[i] = v;
        vImag[i] = 0;
        energy += v * v;
        if (abs(v) > peak) peak = abs(v);
    }

    double rms = sqrt(energy / FFT_SIZE);

    /* -------- 3. FFT -------- */
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    FFT.ComplexToMagnitude();

    double totalMag = 0;
    double hfMag = 0;
    int hfIdx = (1500 * FFT_SIZE) / SAMPLE_RATE;

    for (int i = 2; i < FFT_SIZE / 2; i++) {
        totalMag += vReal[i];
        if (i >= hfIdx) hfMag += vReal[i];
    }

    double hf_ratio = hfMag / (totalMag + 1e-6);

    /* -------- 4. 建立基线 -------- */
    if (!baseline.ready) {
        baseline.rms += rms;
        baseline.hf += hf_ratio;
        baseline.peak += peak;
        baseline.count++;

        if (baseline.count >= 40) {
            baseline.rms /= baseline.count;
            baseline.hf /= baseline.count;
            baseline.peak /= baseline.count;
            baseline.ready = true;
            Serial.println("✅ Baseline ready");
        }
        return;
    }

    /* -------- 5. 工业判断 -------- */
    bool knock = peak > baseline.peak * KNOCK_FACTOR;
    bool vibration = rms > baseline.rms * VIB_FACTOR;
    bool abnormal_hf = hf_ratio > baseline.hf * HF_FACTOR;

    /* -------- 6. 状态输出 -------- */
    const char* status = "OK";

    if (knock) status = "KNOCK";
    else if (abnormal_hf) status = "NOISE";
    else if (vibration) status = "VIBRATION";

    Serial.printf(
        "RMS:%.0f  PEAK:%.0f  HF:%.3f  => %s\n",
        rms, peak, hf_ratio, status
    );

    /* -------- 7. MQTT 上报 -------- */
    static unsigned long lastPub = 0;
    if (millis() - lastPub > PUBLISH_INTERVAL && client.connected()) {
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "{\"rms\":%.0f,\"peak\":%.0f,\"hf\":%.3f,\"state\":\"%s\"}",
                 rms, peak, hf_ratio, status);
        client.publish("device/audio/state", msg);
        lastPub = millis();
    }
}

/* ================== Arduino 生命周期 ================== */
void setup() {
    Serial.begin(115200);
    delay(1500);

    analogReadResolution(12);
    analogSetPinAttenuation(ADC_PIN, ADC_11db);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");

    client.setServer(mqtt_server, 1883);
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        connectMQTT();
        client.loop();
    }

    processAudio();
    delay(20);
}
