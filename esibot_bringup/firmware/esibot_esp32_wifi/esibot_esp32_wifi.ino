/*
  esibot_esp32_wifi.ino
  =====================
  EsiBot ESP32-CAM firmware — motor control (UART) + MJPEG camera stream (WiFi).

  PIN WIRING (from WIRING_GUIDE):
    GPIO2  → L298N IN1  (left motor A)   ⚠ boot-sensitive: 10kΩ pull-down to GND mandatory
    GPIO4  → L298N IN2  (left motor B)   ⚠ flash LED pin:  10kΩ pull-down to GND mandatory
    GPIO13 → L298N IN3  (right motor A)  ✓ safe
    GPIO14 → L298N IN4  (right motor B)  ✓ safe
    ENA    → 5V hardwired (jumper removed)
    ENB    → 5V hardwired (jumper removed)

  UART TO RASPBERRY PI:
    GPIO1 (U0TXD) → Pi Pin10 (GPIO15/RXD)  — ESP32 sends to Pi
    GPIO3 (U0RXD) → Pi Pin8  (GPIO14/TXD)  — Pi sends to ESP32

  UART PROTOCOL:
    Pi  → ESP32 : "CMD:<v_right>,<v_left>\n"   (m/s, sign = direction)
    ESP32 → Pi  : "BAT:<voltage>\n"            (every 1 second)

  WIFI / CAMERA:
    ESP32-CAM connects to WiFi and serves MJPEG at http://<STATIC_IP>/stream
    The esibot_camera ROS node connects to this URL directly.
    Configure WIFI_SSID, WIFI_PASSWORD and STATIC_IP below.

  SPEED CONTROL:
    ENA/ENB hardwired → ON/OFF only (no PWM speed control in v1).
    Only the sign of v_right/v_left matters: + = forward, - = reverse, ~0 = stop.
*/

#include "esp_camera.h"
#include "esp_http_server.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>

// ═════════════════════════════════════════════════════════════════════════════
//  ▶  CONFIGURATION  — edit these before flashing
// ═════════════════════════════════════════════════════════════════════════════
#define WIFI_SSID      "zitounirania"       // your WiFi network name
#define WIFI_PASSWORD  "zitounirania"   // your WiFi password

// Static IP on the same subnet as the Raspberry Pi (10.191.115.99).
// Change STATIC_IP last octet if .10 is already taken on your network.
static const IPAddress STATIC_IP(10, 191, 115, 10);
static const IPAddress GATEWAY  (10, 191, 115,  1);
static const IPAddress SUBNET   (255, 255, 255,  0);
// ═════════════════════════════════════════════════════════════════════════════

// ── Camera pins (AI-Thinker ESP32-CAM) ───────────────────────────────────────
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// ── Motor pin definitions ─────────────────────────────────────────────────────
#define IN1  2     // Left  motor A  ⚠ boot-sensitive
#define IN2  4     // Left  motor B  ⚠ flash LED
#define IN3  13    // Right motor A
#define IN4  14    // Right motor B

// ── Settings ──────────────────────────────────────────────────────────────────
#define BAUD_RATE       115200
#define CMD_TIMEOUT_MS  500      // stop if no CMD received for this long (ms)
#define DEAD_ZONE       0.01f    // |v| below this = stop
#define WIFI_TIMEOUT_MS 10000    // give up WiFi connect after 10 s

// TODO(hardware): wire a voltage divider to an ADC pin for real battery reading.
#define BATTERY_NOMINAL_V  11.1f

// ── MJPEG multipart constants ─────────────────────────────────────────────────
#define STREAM_BOUNDARY  "frame"
#define STREAM_CONTENT   "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY
#define PART_HEADER      "\r\n--" STREAM_BOUNDARY \
                         "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

// ── Runtime state ─────────────────────────────────────────────────────────────
static unsigned long  lastCmdMillis = 0;
static unsigned long  lastBatMillis = 0;
static httpd_handle_t streamServer  = NULL;
static bool           cameraOk      = false;
static bool           wifiOk        = false;

// forward declarations
void driveMotors(float v_right, float v_left);
void stopAll();


// ═════════════════════════════════════════════════════════════════════════════
//  CAMERA INIT
// ═════════════════════════════════════════════════════════════════════════════

static bool initCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = CAM_PIN_D0;
  cfg.pin_d1       = CAM_PIN_D1;
  cfg.pin_d2       = CAM_PIN_D2;
  cfg.pin_d3       = CAM_PIN_D3;
  cfg.pin_d4       = CAM_PIN_D4;
  cfg.pin_d5       = CAM_PIN_D5;
  cfg.pin_d6       = CAM_PIN_D6;
  cfg.pin_d7       = CAM_PIN_D7;
  cfg.pin_xclk     = CAM_PIN_XCLK;
  cfg.pin_pclk     = CAM_PIN_PCLK;
  cfg.pin_vsync    = CAM_PIN_VSYNC;
  cfg.pin_href     = CAM_PIN_HREF;
  cfg.pin_sda      = CAM_PIN_SIOD;
  cfg.pin_scl      = CAM_PIN_SIOC;
  cfg.pin_pwdn     = CAM_PIN_PWDN;
  cfg.pin_reset    = CAM_PIN_RESET;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;

  // QVGA 320×240 — matches frame_width/frame_height in camera_params.yaml
  if (psramFound()) {
    cfg.frame_size   = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 12;   // 0 = best quality, 63 = worst
    cfg.fb_count     = 2;    // double-buffer → smoother stream
  } else {
    cfg.frame_size   = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 20;
    cfg.fb_count     = 1;
  }

  return esp_camera_init(&cfg) == ESP_OK;
}


// ═════════════════════════════════════════════════════════════════════════════
//  MJPEG STREAM HANDLER
//  Runs inside the esp_http_server FreeRTOS task — does not block loop().
// ═════════════════════════════════════════════════════════════════════════════

static esp_err_t streamHandler(httpd_req_t* req) {
  char         partHdr[64];
  camera_fb_t* fb  = NULL;
  esp_err_t    res = httpd_resp_set_type(req, STREAM_CONTENT);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
      break;
    }

    // Part header with JPEG size
    size_t hlen = snprintf(partHdr, sizeof(partHdr), PART_HEADER, fb->len);
    res = httpd_resp_send_chunk(req, partHdr, hlen);
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    // JPEG payload
    res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    fb = NULL;
    if (res != ESP_OK) break;
  }

  return res;
}

static void startStreamServer() {
  httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
  cfg.server_port       = 80;
  cfg.ctrl_port         = 32768;
  cfg.max_uri_handlers  = 2;

  if (httpd_start(&streamServer, &cfg) != ESP_OK) {
    Serial.println("HTTP: server start failed");
    return;
  }

  static const httpd_uri_t uri = {
    .uri      = "/stream",
    .method   = HTTP_GET,
    .handler  = streamHandler,
    .user_ctx = NULL,
  };
  httpd_register_uri_handler(streamServer, &uri);

  Serial.printf("Stream: http://%s/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("esibot_camera arg: esp32_ip:=%s\n",
                WiFi.localIP().toString().c_str());
}


// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  // Disable brownout detector — WiFi radio peaks at ~300 mA on startup.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Motor pins LOW before anything else (boot-safety for GPIO2 / GPIO4).
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  Serial.begin(BAUD_RATE);
  delay(200);
  Serial.println("EsiBot ESP32-CAM WiFi firmware starting...");

  // ── Camera ──────────────────────────────────────────────────────────────
  cameraOk = initCamera();
  Serial.println(cameraOk ? "Camera: OK" : "Camera: FAILED");

  // ── WiFi — static IP, same subnet as Pi (10.191.115.99) ─────────────────
  WiFi.mode(WIFI_STA);
  WiFi.config(STATIC_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("WiFi: connecting to \"%s\" ...\n", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiOk = true;
    Serial.printf("WiFi: connected  IP=%s\n", WiFi.localIP().toString().c_str());
    if (cameraOk) {
      startStreamServer();
    } else {
      Serial.println("WiFi: connected but camera failed — stream not started");
    }
  } else {
    Serial.println("WiFi: FAILED — motor control still active over UART");
  }

  lastCmdMillis = millis();
  lastBatMillis = millis();
  Serial.println("EsiBot ESP32 firmware ready");
}


// ═════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP  — UART motor control + watchdog + battery report
//  The MJPEG stream runs in its own FreeRTOS task (esp_http_server).
// ═════════════════════════════════════════════════════════════════════════════

void loop() {

  // 1. Read CMD from Pi
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("CMD:")) {
      String payload = line.substring(4);    // "0.300,-0.280"
      int    comma   = payload.indexOf(',');
      if (comma > 0) {
        float v_right = payload.substring(0, comma).toFloat();
        float v_left  = payload.substring(comma + 1).toFloat();
        driveMotors(v_right, v_left);
        lastCmdMillis = millis();
      }
    }
  }

  // 2. Watchdog: stop motors if Pi goes silent
  if (millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
    stopAll();
  }

  // 3. Send battery voltage to Pi every second
  if (millis() - lastBatMillis >= 1000) {
    lastBatMillis = millis();
    Serial.print("BAT:");
    Serial.println(BATTERY_NOMINAL_V);
  }
}


// ═════════════════════════════════════════════════════════════════════════════
//  MOTOR CONTROL
// ═════════════════════════════════════════════════════════════════════════════
/*
  Left motor  → IN1, IN2:   Forward: HIGH/LOW  Reverse: LOW/HIGH  Stop: LOW/LOW
  Right motor → IN3, IN4:   Forward: HIGH/LOW  Reverse: LOW/HIGH  Stop: LOW/LOW

  If a motor spins the wrong way, swap its two wires on the L298N output
  terminals (OUT1/OUT2 or OUT3/OUT4). Do not change the code.
*/
void driveMotors(float v_right, float v_left) {
  if      (v_left  >  DEAD_ZONE) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else if (v_left  < -DEAD_ZONE) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else                           { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }

  if      (v_right >  DEAD_ZONE) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else if (v_right < -DEAD_ZONE) { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  else                           { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
}

void stopAll() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
