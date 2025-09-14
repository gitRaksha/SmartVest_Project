#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <time.h>

String fallLog = "";  // ✅ Add this to declare the log variable

// Replace with your WiFi credentials
const char* ssid = "Redmi Note 12";         // <-- Change this
const char* password = "00000001"; // <-- Change this

// ======== LoRa Pins ========
#define SCK     5
#define MISO    19
#define MOSI    27
#define SS      18
#define RST     14
#define DIO0    26
#define LED_BUILTIN 2

WebServer server(80);

String latestData = "{}";
String warningLog = "";

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}

void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for time sync");
  while (time(nullptr) < 100000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nTime synchronized.");
}

String getTimestamp() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  return String(buffer);
}
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", generateHTML());
  });

  server.on("/data", HTTP_GET, []() {
    server.send(200, "application/json", latestData);
  });

  server.on("/log", HTTP_GET, []() {
    server.send(200, "text/plain", warningLog);
  });

  server.begin();
  Serial.println("HTTP server started");
}


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);

  pinMode(LED_BUILTIN, OUTPUT);
  setupWiFi();
  setupTime();

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed.");
    while (true) delay(1000);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(9);
  LoRa.setSyncWord(0x34);

  setupWebServer();
}

void loop() {
  server.handleClient();

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedData;
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }

    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, receivedData) == DeserializationError::Ok) {
      bool fall = doc["fall"];
      float co = doc["co"];
      float acc = doc["acc"];
      int rssi = LoRa.packetRssi();

      DynamicJsonDocument jsonOut(256);
      jsonOut["fall"] = fall;
      jsonOut["co"] = co;
      jsonOut["acc"] = acc;
      jsonOut["rssi"] = rssi;
      jsonOut["time"] = getTimestamp();
      serializeJson(jsonOut, latestData);

      if (fall) {
  // Generate timestamp
  unsigned long seconds = millis() / 1000;
  unsigned long mins = seconds / 60;
  unsigned long hrs = mins / 60;

  String timeStamp = "[" + String(hrs % 24) + ":" + String(mins % 60) + ":" + String(seconds % 60) + "]";

  // Add fall log entry to HTML
  fallLog += "<tr><td>" + timeStamp + "</td><td>Fall detected!</td></tr>";
}



      Serial.println(latestData);
    }
  }

  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
String generateHTML() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Smart Vest Dashboard</title>
    <style>
      body {
        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        background-color: #f0f2f5;
        color: #333;
        text-align: center;
        margin: 0;
        padding: 20px;
      }
      h1 {
        color: #007BFF;
      }
      .card {
        background-color: white;
        border-radius: 10px;
        box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        padding: 20px;
        margin: 20px auto;
        max-width: 500px;
      }
      .data {
        font-size: 1.5em;
        margin: 10px 0;
      }
      #fall {
        font-weight: bold;
      }
      table {
        margin: 20px auto;
        border-collapse: collapse;
        width: 90%;
        max-width: 600px;
      }
      th, td {
        border: 1px solid #ccc;
        padding: 8px 12px;
        text-align: center;
      }
      th {
        background-color: #007BFF;
        color: white;
      }
      tr:nth-child(even) {
        background-color: #f9f9f9;
      }
    </style>
  </head>
  <body>
    <h1>Smart Vest Monitoring</h1>

    <div class="card">
      <div class="data"><strong>Fall:</strong> <span id="fall">Loading...</span></div>
      <div class="data"><strong>CO Level:</strong> <span id="co">Loading...</span></div>
      <div class="data"><strong>Acceleration:</strong> <span id="acc">Loading...</span></div>
      <div class="data"><strong>Signal Strength (RSSI):</strong> <span id="rssi">Loading...</span></div>
    </div>

    <h2>Warning Log</h2>
    <table id="logTable">
      <tr><th>Time</th><th>Event</th></tr>
    </table>

    <script>
      function updateDashboard() {
        fetch('/data')
          .then(res => res.json())
          .then(data => {
            document.getElementById('fall').textContent = data.fall ? "FALL DETECTED!" : "No fall";
            document.getElementById('fall').style.color = data.fall ? "red" : "lime";
            document.getElementById('co').textContent = data.co + " ppm";
            document.getElementById('acc').textContent = data.acc + " mg";
            document.getElementById('rssi').textContent = data.rssi + " dBm";
          });

        fetch('/log')
          .then(res => res.text())
          .then(rows => {
            document.getElementById('logTable').innerHTML = "<tr><th>Time</th><th>Event</th></tr>" + rows;
          });
      }

      setInterval(updateDashboard, 2000);
      window.onload = updateDashboard;
    </script>
  </body>
  </html>
  )rawliteral";
  return html;
}


