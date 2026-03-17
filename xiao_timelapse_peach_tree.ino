#include <Arduino.h>
#include <WiFi.h>
#include <algorithm>
#include <time.h>
#include <vector>

#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "WebServer.h"
#include "esp_sleep.h"

namespace {

constexpr char WIFI_SSID[] = "user_SSID";
constexpr char WIFI_PASSWORD[] = "SSID_password";
constexpr char TZ_INFO[] = "EST5EDT,M3.2.0/2,M11.1.0/2";

// Set these for your peach tree location.
constexpr double LATITUDE = 33.01000;
constexpr double LONGITUDE = -80.00000;

// Optional battery monitor for a 1-cell Li-ion/LiPo.
// Wire battery + through a resistor divider to A0 so the ADC never sees more than 3.3V.
// Simple divider example:
//   battery + -> 100k -> A0 -> 100k -> GND
//   battery - ------------------------> GND
// With that 1:1 divider, set BATTERY_DIVIDER_RATIO to 2.0.
// Leave BATTERY_MONITOR_ENABLED false until this wiring is installed.
constexpr int SD_CS_PIN = 21;
constexpr int CAPTURE_INTERVAL_MINUTES = 60;
constexpr int DAYLIGHT_SPILLOVER_MINUTES = 30;
// External download button:
// wire a normally-open pushbutton between A1/D1 and GND.
// Pressing it wakes the board and starts the SD download server for 30 minutes.
constexpr uint8_t DOWNLOAD_MODE_PIN = A1;
constexpr uint32_t DOWNLOAD_WINDOW_SECONDS = 30UL * 60UL;
constexpr char DOWNLOAD_AP_SSID[] = "gardencam";
constexpr char DOWNLOAD_AP_PASSWORD[] = "gardentree";
constexpr bool BATTERY_MONITOR_ENABLED = false;
constexpr uint8_t BATTERY_ADC_PIN = A0;
constexpr float BATTERY_DIVIDER_RATIO = 2.0f;
constexpr int BATTERY_LOW_MV = 3550;
constexpr int BATTERY_CRITICAL_MV = 3450;
constexpr uint32_t LOW_BATTERY_SLEEP_SECONDS = 2UL * 60UL * 60UL;
constexpr uint32_t CRITICAL_BATTERY_SLEEP_SECONDS = 6UL * 60UL * 60UL;
constexpr uint64_t US_PER_SECOND = 1000000ULL;
constexpr int SECONDS_PER_DAY = 86400;

// Official ESP32 core pin map for CAMERA_MODEL_XIAO_ESP32S3.
// This matches the stock XIAO ESP32S3 Sense camera connector.
constexpr int PWDN_GPIO_NUM = -1;
constexpr int RESET_GPIO_NUM = -1;
constexpr int XCLK_GPIO_NUM = 10;
constexpr int SIOD_GPIO_NUM = 40;
constexpr int SIOC_GPIO_NUM = 39;

constexpr int Y9_GPIO_NUM = 48;
constexpr int Y8_GPIO_NUM = 11;
constexpr int Y7_GPIO_NUM = 12;
constexpr int Y6_GPIO_NUM = 14;
constexpr int Y5_GPIO_NUM = 16;
constexpr int Y4_GPIO_NUM = 18;
constexpr int Y3_GPIO_NUM = 17;
constexpr int Y2_GPIO_NUM = 15;
constexpr int VSYNC_GPIO_NUM = 38;
constexpr int HREF_GPIO_NUM = 47;
constexpr int PCLK_GPIO_NUM = 13;

RTC_DATA_ATTR uint32_t imageCount = 0;
WebServer server(80);
bool cameraInitialized = false;
unsigned long downloadDeadlineMs = 0;

struct PhotoEntry {
  String path;
  size_t size = 0;
};

double degToRad(double degrees) {
  return degrees * PI / 180.0;
}

double radToDeg(double radians) {
  return radians * 180.0 / PI;
}

int dayOfYear(const tm &timeInfo) {
  return timeInfo.tm_yday + 1;
}

double normalizeDegrees(double value) {
  while (value < 0.0) {
    value += 360.0;
  }
  while (value >= 360.0) {
    value -= 360.0;
  }
  return value;
}

int minutesFromMidnight(const tm &timeInfo) {
  return (timeInfo.tm_hour * 60) + timeInfo.tm_min;
}

time_t epochAtMinutes(const tm &timeInfo, int minutesFromStartOfDay) {
  tm eventTime = timeInfo;
  eventTime.tm_hour = minutesFromStartOfDay / 60;
  eventTime.tm_min = minutesFromStartOfDay % 60;
  eventTime.tm_sec = 0;
  return mktime(&eventTime);
}

bool initCamera() {
  if (cameraInitialized) {
    return true;
  }

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  if (!psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count = 1;
    config.jpeg_quality = 12;
  } else {
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    if (sensor->id.PID == OV3660_PID) {
      sensor->set_vflip(sensor, 1);
      sensor->set_brightness(sensor, 1);
      sensor->set_saturation(sensor, -2);
    }

    // Keep default auto exposure/gain behavior for the stock OV2640.
    sensor->set_brightness(sensor, 1);
    sensor->set_contrast(sensor, 1);
    sensor->set_saturation(sensor, -1);
  }

  cameraInitialized = true;
  return true;
}

void warmUpCamera() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb != nullptr) {
    esp_camera_fb_return(fb);
  }
  delay(200);
}

void configureCameraForDownloadMode() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    return;
  }

  sensor->set_framesize(sensor, FRAMESIZE_VGA);
  sensor->set_quality(sensor, 12);
}

void configureCameraForCaptureMode() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    return;
  }

  sensor->set_framesize(sensor, psramFound() ? FRAMESIZE_UXGA : FRAMESIZE_SVGA);
  sensor->set_quality(sensor, psramFound() ? 10 : 12);
}

bool initSdCard() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init failed");
    return false;
  }

  if (SD.cardType() == CARD_NONE) {
    Serial.println("No SD card detected");
    return false;
  }

  return true;
}

bool isSafeFilePath(const String &path) {
  return path.length() > 1 && path[0] == '/' && path.indexOf("..") == -1;
}

String contentTypeForPath(const String &path) {
  if (path.endsWith(".jpg") || path.endsWith(".jpeg")) {
    return "image/jpeg";
  }
  if (path.endsWith(".htm") || path.endsWith(".html")) {
    return "text/html";
  }
  if (path.endsWith(".txt")) {
    return "text/plain";
  }
  return "application/octet-stream";
}

String escapeJson(const String &value) {
  String escaped;
  escaped.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    switch (c) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += c;
        break;
    }
  }
  return escaped;
}

bool isPhotoPath(const String &path) {
  const String lower = path;
  return lower.endsWith(".jpg") || lower.endsWith(".jpeg");
}

std::vector<PhotoEntry> listPhotos() {
  std::vector<PhotoEntry> photos;
  File root = SD.open("/");
  if (!root || !root.isDirectory()) {
    return photos;
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      const String path = String(file.name());
      if (isPhotoPath(path)) {
        photos.push_back({path, file.size()});
      }
    }
    file = root.openNextFile();
  }

  std::sort(photos.begin(), photos.end(), [](const PhotoEntry &lhs, const PhotoEntry &rhs) {
    return lhs.path > rhs.path;
  });
  return photos;
}

bool streamFilePath(const String &path, bool asAttachment) {
  if (!isSafeFilePath(path) || !SD.exists(path)) {
    return false;
  }

  File file = SD.open(path, FILE_READ);
  if (!file) {
    return false;
  }

  if (asAttachment) {
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + path.substring(1) + "\"");
  }
  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(file, contentTypeForPath(path));
  file.close();
  return true;
}

String latestPhotoPath() {
  const std::vector<PhotoEntry> photos = listPhotos();
  if (photos.empty()) {
    return String();
  }
  return photos.front().path;
}

bool deletePhotoPath(const String &path) {
  return isSafeFilePath(path) && SD.exists(path) && SD.remove(path);
}

void handleDownloadRoot() {
  String html;
  html.reserve(9000);
  html += "<!doctype html><html><head><meta charset='utf-8'><title>Peach Cam</title>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<style>:root{color-scheme:light;background:#f6f1e8;color:#1e1b18;--accent:#8c4f2b;--panel:#fffaf2;--line:#dfd1bd;}";
  html += "body{margin:0;font-family:Georgia,serif;background:linear-gradient(180deg,#f3eadb,#f9f5ee);}";
  html += "main{max-width:1100px;margin:0 auto;padding:20px 16px 40px;}";
  html += "h1{margin:0 0 8px;font-size:2.2rem;}p{line-height:1.4;}a,button{font:inherit;}";
  html += ".hero,.panel,.card{background:rgba(255,250,242,.92);border:1px solid var(--line);border-radius:18px;box-shadow:0 10px 30px rgba(78,52,32,.08);}";
  html += ".hero{padding:20px;margin-bottom:18px;}.actions{display:flex;flex-wrap:wrap;gap:10px;margin-top:14px;}";
  html += ".btn{display:inline-block;padding:10px 14px;border-radius:999px;border:1px solid var(--accent);background:var(--accent);color:#fff;text-decoration:none;cursor:pointer;}";
  html += ".btn.alt{background:transparent;color:var(--accent);}button.danger{border-color:#9c2f2f;background:#9c2f2f;color:#fff;}";
  html += ".grid{display:grid;grid-template-columns:2fr 1fr;gap:18px;}.panel{padding:16px;}.gallery{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:14px;}";
  html += ".card{overflow:hidden;}.thumb{aspect-ratio:4/3;width:100%;object-fit:cover;display:block;background:#eadfce;}";
  html += ".meta{padding:10px 12px;}.meta strong{display:block;font-size:.95rem;word-break:break-all;}";
  html += ".meta small{display:block;color:#6a584a;margin:6px 0 10px;}.row{display:flex;gap:8px;flex-wrap:wrap;}";
  html += ".empty{padding:18px;border:1px dashed var(--line);border-radius:14px;background:#fffdf9;}.latest{width:100%;border-radius:14px;display:block;background:#eadfce;}";
  html += "@media (max-width:880px){.grid{grid-template-columns:1fr;}}</style></head><body><main>";
  html += "<section class='hero'><h1>Peach Tree Timelapse</h1><p>Download mode is active over Wi-Fi. Browse the SD card, download photos, view the newest frame, or delete files remotely.</p>";
  html += "<div class='actions'><a class='btn' href='/latest' target='_blank'>Open Latest Photo</a><a class='btn alt' href='/snapshot' target='_blank'>Live Snapshot</a><a class='btn alt' href='/stream' target='_blank'>Live Stream</a><button class='btn danger' id='deleteAllBtn' type='button'>Delete All Photos</button></div></section>";
  html += "<section class='grid'><div class='panel'><h2>Gallery</h2><div id='gallery' class='gallery'></div></div>";
  html += "<aside class='panel'><h2>Latest Saved Photo</h2><div id='latestPanel'></div></aside></section>";
  html += "<script>";
  html += "const gallery=document.getElementById('gallery');const latestPanel=document.getElementById('latestPanel');";
  html += "function esc(v){return v.replace(/[&<>\\\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\\\"':'&quot;',\"'\":'&#39;'}[c]));}";
  html += "function fmtSize(bytes){return (bytes/1024).toFixed(1)+' KB';}";
  html += "async function api(path,options){const r=await fetch(path,options);if(!r.ok){throw new Error(await r.text()||('HTTP '+r.status));}return r;}";
  html += "function renderLatest(photo){if(!photo){latestPanel.innerHTML=\"<div class='empty'>No saved photos found on the SD card.</div>\";return;}";
  html += "latestPanel.innerHTML=`<a href='/file?name=${encodeURIComponent(photo.name)}' target='_blank'><img class='latest' src='/file?name=${encodeURIComponent(photo.name)}&inline=1' alt='Latest photo'></a><p><strong>${esc(photo.name)}</strong><br><small>${fmtSize(photo.size)}</small></p><div class='row'><a class='btn' href='/file?name=${encodeURIComponent(photo.name)}&download=1'>Download</a><button class='btn alt' id='latestDeleteBtn' type='button'>Delete</button></div>`;";
  html += "document.getElementById('latestDeleteBtn').addEventListener('click',()=>deleteOne(photo.name));}";
  html += "function renderGallery(items){if(!items.length){gallery.innerHTML=\"<div class='empty'>No JPEG images on the SD card.</div>\";renderLatest(null);return;}";
  html += "renderLatest(items[0]);gallery.innerHTML=items.map(item=>`<article class='card'><a href='/file?name=${encodeURIComponent(item.name)}' target='_blank'><img class='thumb' loading='lazy' src='/file?name=${encodeURIComponent(item.name)}&inline=1' alt='${esc(item.name)}'></a><div class='meta'><strong>${esc(item.name)}</strong><small>${fmtSize(item.size)}</small><div class='row'><a class='btn' href='/file?name=${encodeURIComponent(item.name)}&download=1'>Download</a><button class='btn alt' type='button' data-name='${esc(item.name)}'>Delete</button></div></div></article>`).join('');";
  html += "gallery.querySelectorAll('button[data-name]').forEach(btn=>btn.addEventListener('click',()=>deleteOne(btn.dataset.name)));}";
  html += "async function loadList(){const data=await (await api('/list')).json();renderGallery(data.files||[]);}";
  html += "async function deleteOne(name){if(!confirm('Delete '+name+'?')) return;await api('/delete?name='+encodeURIComponent(name),{method:'POST'});await loadList();}";
  html += "document.getElementById('deleteAllBtn').addEventListener('click',async()=>{if(!confirm('Delete all photos from the SD card?')) return;await api('/delete-all',{method:'POST'});await loadList();});";
  html += "loadList().catch(err=>{gallery.innerHTML=`<div class='empty'>${esc(err.message)}</div>`;latestPanel.innerHTML='';});";
  html += "</script></main></body></html>";
  server.send(200, "text/html", html);
}

void handleListFiles() {
  const std::vector<PhotoEntry> photos = listPhotos();

  String json = "{\"files\":[";
  for (size_t i = 0; i < photos.size(); ++i) {
    if (i > 0) {
      json += ',';
    }
    json += "{\"name\":\"";
    json += escapeJson(photos[i].path.substring(1));
    json += "\",\"size\":";
    json += String(static_cast<unsigned>(photos[i].size));
    json += "}";
  }
  json += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleSnapshot() {
  if (!initCamera()) {
    server.send(500, "text/plain", "Camera init failed");
    return;
  }

  configureCameraForDownloadMode();
  warmUpCamera();
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    server.send(500, "text/plain", "Snapshot failed");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/jpeg", reinterpret_cast<const char *>(fb->buf), fb->len);
  esp_camera_fb_return(fb);
}

void handleStream() {
  if (!initCamera()) {
    server.send(500, "text/plain", "Camera init failed");
    return;
  }

  configureCameraForDownloadMode();
  warmUpCamera();

  WiFiClient client = server.client();
  client.printf("HTTP/1.1 200 OK\r\n");
  client.printf("Content-Type: multipart/x-mixed-replace; boundary=frame\r\n");
  client.printf("Cache-Control: no-store\r\n");
  client.printf("Pragma: no-cache\r\n\r\n");

  while (client.connected() && millis() < downloadDeadlineMs) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr) {
      delay(50);
      continue;
    }

    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                  static_cast<unsigned>(fb->len));
    client.write(fb->buf, fb->len);
    client.printf("\r\n");
    esp_camera_fb_return(fb);
    delay(80);
  }
}

void handleFileRequest() {
  String path = server.arg("name");
  if (path.isEmpty()) {
    path = server.arg("file");
  }
  if (path.isEmpty()) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  if (path[0] != '/') {
    path = "/" + path;
  }

  const bool asAttachment = server.hasArg("download") && server.arg("download") != "0";
  if (!streamFilePath(path, asAttachment)) {
    server.send(404, "text/plain", "File not found");
    return;
  }
}

void handleLatestFile() {
  const String path = latestPhotoPath();
  if (path.isEmpty()) {
    server.send(404, "text/plain", "No photos available");
    return;
  }

  if (!streamFilePath(path, false)) {
    server.send(500, "text/plain", "Failed to open latest photo");
  }
}

void handleDeleteFile() {
  String path = server.arg("name");
  if (path.isEmpty()) {
    path = server.arg("file");
  }
  if (path.isEmpty()) {
    server.send(400, "text/plain", "Missing file name");
    return;
  }
  if (path[0] != '/') {
    path = "/" + path;
  }

  if (!deletePhotoPath(path)) {
    server.send(404, "text/plain", "Delete failed");
    return;
  }

  server.send(200, "text/plain", "Deleted");
}

void handleDeleteAllFiles() {
  const std::vector<PhotoEntry> photos = listPhotos();
  size_t deletedCount = 0;
  for (const PhotoEntry &photo : photos) {
    if (SD.remove(photo.path)) {
      ++deletedCount;
    }
  }

  server.send(200, "application/json", "{\"deleted\":" + String(static_cast<unsigned>(deletedCount)) + "}");
}

bool connectWifiForDownloadMode() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(DOWNLOAD_AP_SSID, DOWNLOAD_AP_PASSWORD);
  Serial.print("Download AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("Join SSID '%s' with password '%s'\n", DOWNLOAD_AP_SSID, DOWNLOAD_AP_PASSWORD);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 15000UL) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Download STA IP: ");
    Serial.println(WiFi.localIP());
  }
  return true;
}

void startDownloadServer() {
  server.on("/", HTTP_GET, handleDownloadRoot);
  server.on("/list", HTTP_GET, handleListFiles);
  server.on("/file", HTTP_GET, handleFileRequest);
  server.on("/download", HTTP_GET, handleFileRequest);
  server.on("/latest", HTTP_GET, handleLatestFile);
  server.on("/delete", HTTP_POST, handleDeleteFile);
  server.on("/delete-all", HTTP_POST, handleDeleteAllFiles);
  server.on("/snapshot", HTTP_GET, handleSnapshot);
  server.on("/stream", HTTP_GET, handleStream);
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
  server.begin();
  Serial.println("HTTP file server started");
}

void stopDownloadServer() {
  server.stop();
  configureCameraForCaptureMode();
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
}

bool readBatteryMillivolts(int &batteryMillivolts) {
  if (!BATTERY_MONITOR_ENABLED) {
    return false;
  }

  analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

  uint32_t totalMillivolts = 0;
  constexpr int sampleCount = 8;
  for (int i = 0; i < sampleCount; ++i) {
    totalMillivolts += analogReadMilliVolts(BATTERY_ADC_PIN);
    delay(5);
  }

  const float pinMillivolts = static_cast<float>(totalMillivolts) / sampleCount;
  batteryMillivolts = static_cast<int>(lround(pinMillivolts * BATTERY_DIVIDER_RATIO));
  return batteryMillivolts > 500;
}

bool connectWifiAndSyncTime() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 30000UL) {
    delay(250);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connect failed");
    return false;
  }

  configTzTime(TZ_INFO, "pool.ntp.org", "time.nist.gov");

  tm timeInfo = {};
  const unsigned long syncStartMs = millis();
  while (!getLocalTime(&timeInfo, 1000) && millis() - syncStartMs < 30000UL) {
  }

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);

  if (timeInfo.tm_year < 124) {
    Serial.println("Time sync failed");
    return false;
  }

  return true;
}

int utcOffsetMinutes(const tm &timeInfo) {
  char offset[8] = {};
  if (strftime(offset, sizeof(offset), "%z", &timeInfo) != 5) {
    return 0;
  }

  const int sign = offset[0] == '-' ? -1 : 1;
  const int hours = ((offset[1] - '0') * 10) + (offset[2] - '0');
  const int minutes = ((offset[3] - '0') * 10) + (offset[4] - '0');
  return sign * ((hours * 60) + minutes);
}

bool computeSunEventMinutes(const tm &timeInfo, bool sunrise, int &eventMinutes) {
  const int n = dayOfYear(timeInfo);
  const double lngHour = LONGITUDE / 15.0;
  const double baseTime = sunrise ? 6.0 : 18.0;
  const double t = n + ((baseTime - lngHour) / 24.0);

  double meanAnomaly = (0.9856 * t) - 3.289;
  double trueLongitude = meanAnomaly + (1.916 * sin(degToRad(meanAnomaly))) +
                         (0.020 * sin(2.0 * degToRad(meanAnomaly))) + 282.634;
  trueLongitude = normalizeDegrees(trueLongitude);

  double rightAscension = radToDeg(atan(0.91764 * tan(degToRad(trueLongitude))));
  rightAscension = normalizeDegrees(rightAscension);

  const double lQuadrant = floor(trueLongitude / 90.0) * 90.0;
  const double raQuadrant = floor(rightAscension / 90.0) * 90.0;
  rightAscension += lQuadrant - raQuadrant;
  rightAscension /= 15.0;

  const double sinDec = 0.39782 * sin(degToRad(trueLongitude));
  const double cosDec = cos(asin(sinDec));

  const double zenith = 90.833;
  const double cosH = (cos(degToRad(zenith)) - (sinDec * sin(degToRad(LATITUDE)))) /
                      (cosDec * cos(degToRad(LATITUDE)));

  if (cosH < -1.0 || cosH > 1.0) {
    return false;
  }

  double hourAngle = sunrise ? 360.0 - radToDeg(acos(cosH)) : radToDeg(acos(cosH));
  hourAngle /= 15.0;

  const double localMeanTime = hourAngle + rightAscension - (0.06571 * t) - 6.622;
  double universalTime = localMeanTime - lngHour;
  while (universalTime < 0.0) {
    universalTime += 24.0;
  }
  while (universalTime >= 24.0) {
    universalTime -= 24.0;
  }

  int localMinutes = static_cast<int>(lround(universalTime * 60.0 + utcOffsetMinutes(timeInfo)));
  localMinutes %= 24 * 60;
  if (localMinutes < 0) {
    localMinutes += 24 * 60;
  }

  eventMinutes = localMinutes;
  return true;
}

bool shouldCaptureNow(const tm &timeInfo, int &startMinutes, int &endMinutes) {
  int sunriseMinutes = 0;
  int sunsetMinutes = 0;
  if (!computeSunEventMinutes(timeInfo, true, sunriseMinutes) ||
      !computeSunEventMinutes(timeInfo, false, sunsetMinutes)) {
    return false;
  }

  startMinutes = sunriseMinutes - DAYLIGHT_SPILLOVER_MINUTES;
  endMinutes = sunsetMinutes + DAYLIGHT_SPILLOVER_MINUTES;

  if (startMinutes < 0) {
    startMinutes = 0;
  }
  if (endMinutes > 24 * 60) {
    endMinutes = 24 * 60;
  }

  const int nowMinutes = minutesFromMidnight(timeInfo);
  return nowMinutes >= startMinutes && nowMinutes <= endMinutes;
}

time_t nextWakeTime(const tm &timeInfo, bool insideWindow, int startMinutes, int endMinutes) {
  tm nowTime = timeInfo;
  const time_t nowEpoch = mktime(&nowTime);
  time_t startEpoch = epochAtMinutes(timeInfo, startMinutes);
  const time_t endEpoch = epochAtMinutes(timeInfo, endMinutes);

  if (insideWindow) {
    tm nextTime = timeInfo;
    nextTime.tm_sec = 0;
    nextTime.tm_min = ((timeInfo.tm_min / CAPTURE_INTERVAL_MINUTES) + 1) * CAPTURE_INTERVAL_MINUTES;
    nextTime.tm_hour = timeInfo.tm_hour;

    if (nextTime.tm_min >= 60) {
      nextTime.tm_min = 0;
      nextTime.tm_hour += 1;
    }

    time_t nextEpoch = mktime(&nextTime);
    if (nextEpoch <= endEpoch) {
      return nextEpoch;
    }

    if (nowEpoch < endEpoch) {
      return endEpoch;
    }
  }

  const int nowMinutes = minutesFromMidnight(timeInfo);
  if (insideWindow || nowMinutes > endMinutes || nowEpoch >= endEpoch) {
    startEpoch += SECONDS_PER_DAY;
  }

  return startEpoch;
}

void printLocalTime(const char *label, time_t epochSeconds) {
  tm localTime = {};
  localtime_r(&epochSeconds, &localTime);

  char buffer[32];
  if (strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &localTime) == 0) {
    Serial.printf("%s: <format failed>\n", label);
    return;
  }

  Serial.printf("%s: %s\n", label, buffer);
}

gpio_num_t downloadWakeGpio() {
  return static_cast<gpio_num_t>(digitalPinToGPIONumber(DOWNLOAD_MODE_PIN));
}

void enableDownloadWakeTrigger() {
  const gpio_num_t wakePin = downloadWakeGpio();
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup(wakePin, 0);
}

void savePhoto(const tm &timeInfo) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    Serial.println("Capture failed");
    return;
  }

  char path[48];
  snprintf(path, sizeof(path), "/%04d%02d%02d_%02d%02d%02d.jpg",
           timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
           timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);

  File file = SD.open(path, FILE_WRITE);
  if (!file) {
    Serial.printf("Open failed: %s\n", path);
    esp_camera_fb_return(fb);
    return;
  }

  const size_t written = file.write(fb->buf, fb->len);
  file.close();
  esp_camera_fb_return(fb);

  if (written != fb->len) {
    Serial.printf("Write failed: %s\n", path);
    return;
  }

  ++imageCount;
  Serial.printf("Saved %s (%u bytes)\n", path, static_cast<unsigned>(written));
}

[[noreturn]] void sleepUntil(time_t targetTime) {
  const time_t now = time(nullptr);
  int64_t sleepSeconds = static_cast<int64_t>(targetTime - now);
  if (sleepSeconds < 60) {
    sleepSeconds = 60;
  }

  Serial.printf("Sleeping for %lld seconds\n", static_cast<long long>(sleepSeconds));
  Serial.flush();
  enableDownloadWakeTrigger();
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(sleepSeconds) * US_PER_SECOND);
  esp_deep_sleep_start();

  while (true) {
  }
}

[[noreturn]] void sleepForSeconds(uint32_t sleepSeconds) {
  if (sleepSeconds < 60) {
    sleepSeconds = 60;
  }

  Serial.printf("Sleeping for %lu seconds\n", static_cast<unsigned long>(sleepSeconds));
  Serial.flush();
  enableDownloadWakeTrigger();
  esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(sleepSeconds) * US_PER_SECOND);
  esp_deep_sleep_start();

  while (true) {
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(DOWNLOAD_MODE_PIN, INPUT_PULLUP);
  enableDownloadWakeTrigger();
  const esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();
  const bool downloadRequested =
      wakeCause == ESP_SLEEP_WAKEUP_EXT0 || digitalRead(DOWNLOAD_MODE_PIN) == LOW;
  if (downloadRequested) {
    Serial.println("Download mode requested");
    if (!initSdCard()) {
      return;
    }
    if (!initCamera()) {
      return;
    }
    configureCameraForDownloadMode();
    connectWifiForDownloadMode();
    startDownloadServer();

    downloadDeadlineMs = millis() + (DOWNLOAD_WINDOW_SECONDS * 1000UL);
    while (millis() < downloadDeadlineMs) {
      server.handleClient();
      delay(2);
    }

    stopDownloadServer();

    if (!connectWifiAndSyncTime()) {
      sleepForSeconds(15UL * 60UL);
    }
  }

  int batteryMillivolts = 0;
  if (readBatteryMillivolts(batteryMillivolts)) {
    Serial.printf("Battery: %d mV\n", batteryMillivolts);
    if (batteryMillivolts <= BATTERY_CRITICAL_MV) {
      Serial.println("Battery critical, skipping work");
      sleepForSeconds(CRITICAL_BATTERY_SLEEP_SECONDS);
    }
    if (batteryMillivolts <= BATTERY_LOW_MV) {
      Serial.println("Battery low, skipping work");
      sleepForSeconds(LOW_BATTERY_SLEEP_SECONDS);
    }
  }

  if (!initSdCard()) {
    return;
  }

  if (!connectWifiAndSyncTime()) {
    sleepForSeconds(15UL * 60UL);
  }

  tm timeInfo = {};
  if (!getLocalTime(&timeInfo, 1000)) {
    sleepForSeconds(15UL * 60UL);
  }

  int startMinutes = 0;
  int endMinutes = 0;
  const bool insideWindow = shouldCaptureNow(timeInfo, startMinutes, endMinutes);

  Serial.printf("Window: %02d:%02d to %02d:%02d\n",
                startMinutes / 60, startMinutes % 60, endMinutes / 60, endMinutes % 60);
  tm nowTime = timeInfo;
  printLocalTime("Current time", mktime(&nowTime));

  if (insideWindow) {
    if (!initCamera()) {
      sleepForSeconds(15UL * 60UL);
    }
    configureCameraForCaptureMode();
    warmUpCamera();
    savePhoto(timeInfo);
  } else {
    Serial.println("Outside daylight window, skipping capture");
  }

  const time_t wakeEpoch = nextWakeTime(timeInfo, insideWindow, startMinutes, endMinutes);
  printLocalTime("Next wake", wakeEpoch);
  sleepUntil(wakeEpoch);
}

void loop() {
}
