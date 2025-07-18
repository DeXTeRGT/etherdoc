#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <esp_mac.h>
#include <ShiftRegister74HC595.h>
#include <Preferences.h>
#include <ArduinoJson.h>

#define ETH_CS 5
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18
#define RESET_CONFIG_PIN 22
#define FIRMWARE_VERSION "1.8.2"
#define MAX_RADIOS 2
#define RX1 12
#define TX1 26
#define RX2 13
#define TX2 27

// EEPROM Keys
const char * PREF_NS = "netcfg";
const char * KEY_IP = "ip";
const char * KEY_GW = "gw";
const char * KEY_SUBNET = "subnet";
const char * KEY_DNS = "dns";
const char * KEY_PORT = "port";
const char * KEY_CFGMODE = "config_mode";
const char * KEY_SIG = "sig";
const char * KEY_USEUDP = "use_udp";
const char * KEY_TCP_PORT = "tcp_port";
const char * KEY_USETCP = "use_tcp";
const uint32_t EEPROM_SIGNATURE = 0xDEADBEEF;

Preferences prefs;
EthernetServer server(80);
EthernetServer tcpServer(12345);

bool configMode = false;
bool useUdp = true;
bool useTcp = false;
uint16_t tcpPort;
unsigned long lastEthCheck = 0;
const unsigned long ethCheckInterval = 5000;

struct NetConfig {
  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;
  uint16_t udpPort;
  uint16_t tcpPort;
};

const IPAddress DEFAULT_IP(192, 168, 0, 201);
const IPAddress DEFAULT_GATEWAY(192, 168, 0, 1);
const IPAddress DEFAULT_SUBNET(255, 255, 255, 0);
const IPAddress DEFAULT_DNS(192, 168, 0, 128);
const uint16_t DEFAULT_UDP_PORT = 50011;

NetConfig netConfig;
EthernetUDP Udp;
char packetBuffer[1024];

struct RadioState {
  uint8_t radioNr;
  int8_t lastBandIndex;
};
RadioState radios[MAX_RADIOS];

const uint8_t bandToOutputBit[6] = {
  1, // 160M → Q0
  2, // 80M  → Q1
  3, // 40M  → Q2
  4, // 20M  → Q3
  5, // 15M  → Q4
  6 // 10M  → Q5
};

ShiftRegister74HC595 < 2 > sr(21, 4, 15);

int getBandIndex(long freqHz) {
  if (freqHz >= 180000 && freqHz <= 200000) return 0;
  if (freqHz >= 350000 && freqHz <= 400000) return 1;
  if (freqHz >= 700000 && freqHz <= 730000) return 2;
  if (freqHz >= 1400000 && freqHz <= 1435000) return 3;
  if (freqHz >= 2100000 && freqHz <= 2145000) return 4;
  if (freqHz >= 2800000 && freqHz <= 2970000) return 5;
  return -1;
}

RadioState * getRadioState(int radioNr) {
  if (radioNr < 1 || radioNr > MAX_RADIOS) return nullptr;
  for (int i = 0; i < MAX_RADIOS; ++i)
    if (radios[i].radioNr == radioNr) return & radios[i];
  return nullptr;
}

bool findTagValue(const char * xml,
  const char * tag, char * out, size_t maxLen) {
  char tagFmt[16];
  snprintf(tagFmt, sizeof(tagFmt), "<%s>", tag);
  const char * start = strstr(xml, tagFmt);
  if (!start) return false;
  start += strlen(tagFmt);

  snprintf(tagFmt, sizeof(tagFmt), "</%s>", tag);
  const char * end = strstr(start, tagFmt);
  if (!end || end <= start) return false;

  size_t len = end - start;
  if (len >= maxLen) return false;

  memcpy(out, start, len);
  out[len] = '\0';
  return true;
}

void updateShiftRegister(int radioIndex) {
  static uint8_t outputs[2] = {
    0,
    0
  };
  outputs[radioIndex] = 0;
  int bandIdx = radios[radioIndex].lastBandIndex;
  if (bandIdx >= 0 && bandIdx < 6) {
    outputs[radioIndex] |= (1 << bandToOutputBit[bandIdx]);
  }
  sr.setAll(outputs);
}

// --- Setup and Loop Logic ---

void applyAndSaveDefaultConfig() {
  netConfig.ip = DEFAULT_IP;
  netConfig.gateway = DEFAULT_GATEWAY;
  netConfig.subnet = DEFAULT_SUBNET;
  netConfig.dns = DEFAULT_DNS;
  netConfig.udpPort = DEFAULT_UDP_PORT;
  prefs.putUInt(KEY_SIG, EEPROM_SIGNATURE);
  prefs.putString(KEY_IP, DEFAULT_IP.toString());
  prefs.putString(KEY_GW, DEFAULT_GATEWAY.toString());
  prefs.putString(KEY_SUBNET, DEFAULT_SUBNET.toString());
  prefs.putString(KEY_DNS, DEFAULT_DNS.toString());
  prefs.putUShort(KEY_PORT, DEFAULT_UDP_PORT);
  prefs.putBool(KEY_CFGMODE, true);
  prefs.putBool(KEY_USEUDP, true);
  prefs.putUShort(KEY_TCP_PORT, 60000);
  prefs.putBool(KEY_USETCP, false);
}

void loadConfig(bool resetToDefaults = false) {
  prefs.begin(PREF_NS, false);
  bool hasValidSig = prefs.getUInt(KEY_SIG, 0) == EEPROM_SIGNATURE;
  if (resetToDefaults || !hasValidSig) {
    applyAndSaveDefaultConfig();
    prefs.end();
    Serial.println("Reset to default config. Rebooting...");
    delay(1000);
    ESP.restart();
  } else {
    netConfig.ip.fromString(prefs.getString(KEY_IP));
    netConfig.gateway.fromString(prefs.getString(KEY_GW));
    netConfig.subnet.fromString(prefs.getString(KEY_SUBNET));
    netConfig.dns.fromString(prefs.getString(KEY_DNS));
    netConfig.udpPort = prefs.getUShort(KEY_PORT);
    netConfig.tcpPort = prefs.getUShort(KEY_TCP_PORT);
    configMode = prefs.getBool(KEY_CFGMODE, true);
    useUdp = prefs.getBool(KEY_USEUDP, true);
    useTcp = prefs.getBool(KEY_USETCP, false);
    prefs.end();
  }
}

void checkEthernetLink(bool blocking) {
  static unsigned long lastCheck = 0;
  static uint8_t retryCount = 0;
  const uint8_t maxRetries = 20;
  const unsigned long retryInterval = 1000; // ms

  if (blocking) {
    retryCount = 0;
    while (Ethernet.linkStatus() == LinkOFF && retryCount++ < maxRetries) {
      Serial.printf("Waiting for Ethernet link... (%d/%d)\n", retryCount, maxRetries);
      delay(retryInterval);
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet failed to link. Rebooting...");
      delay(1000);
      ESP.restart();
    }
  } else {
    if (millis() - lastCheck >= retryInterval) {
      lastCheck = millis();

      if (Ethernet.linkStatus() == LinkOFF) {
        retryCount++;
        Serial.printf("Link check failed (%d/%d)\n", retryCount, maxRetries);
        if (retryCount >= maxRetries) {
          Serial.println("Link down too long. Rebooting...");
          delay(100);
          ESP.restart();
        }
      } else {
        // Link is up — reset retry counter
        if (retryCount != 0) {
          Serial.println("Ethernet link restored.");
          retryCount = 0;
        }
      }
    }
  }
}

void saveConfigFromWeb(EthernetClient & client, String ip, String gw, String subnet, String dns, uint16_t port, bool useUdpFlag, uint16_t tcpPort, bool useTcpFlag) {
  prefs.begin(PREF_NS, false);
  prefs.putString(KEY_IP, ip);
  prefs.putString(KEY_GW, gw);
  prefs.putString(KEY_SUBNET, subnet);
  prefs.putString(KEY_DNS, dns);
  prefs.putUShort(KEY_PORT, port);
  prefs.putUShort(KEY_TCP_PORT, tcpPort);
  prefs.putBool(KEY_CFGMODE, false);
  prefs.putBool(KEY_USEUDP, useUdpFlag);
  prefs.putBool(KEY_USETCP, useTcpFlag);
  prefs.end();

  //useUdp = useUdpFlag;
  Serial.println("Config saved. Rebooting...");

  String html = "<html><body>";
  html += "<h3>Configuration saved.</h3>";
  html += "<p>Please wait a few seconds for reboot, then <a href='/'>click here to return to the main page</a>.</p>";
  html += "</body></html>";

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.print(html);
  client.flush();
  delay(3000);
  ESP.restart();
}

void handleApiJson(EthernetClient& client, const String& requestBody) {
  if (useUdp) {
    client.println("HTTP/1.1 403 Forbidden");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println("{\"error\":\"UDP mode is enabled. API access is disabled.\"}");
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, requestBody);
  if (err) {
    client.println("HTTP/1.1 400 Bad Request");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println("{\"error\":\"Invalid JSON\"}");
    return;
  }

  bool updated = false;

  // Handle clear flags first
  if (doc["clear_radio1"] == true) {
    radios[0].lastBandIndex = -1;
    updateShiftRegister(0);
    updated = true;
  }

  if (doc["clear_radio2"] == true) {
    radios[1].lastBandIndex = -1;
    updateShiftRegister(1);
    updated = true;
  }

  // Then handle band index updates
  if (doc.containsKey("radio1")) {
    int band1 = doc["radio1"];
    if (band1 >= 0 && band1 < 6) {
      radios[0].lastBandIndex = band1;
      updateShiftRegister(0);
      updated = true;
    }
  }

  if (doc.containsKey("radio2")) {
    int band2 = doc["radio2"];
    if (band2 >= 0 && band2 < 6) {
      radios[1].lastBandIndex = band2;
      updateShiftRegister(1);
      updated = true;
    }
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  client.print("{\"status\":\"");
  client.print(updated ? "updated" : "no_change");
  client.println("\"}");
}



void serveWebConfig() {
  EthernetClient client = server.available();
  if (!client) return;

  String requestLine, headerLine;
  int contentLength = 0;

  Serial.println("=== BEGIN HTTP REQUEST ===");

  // Wait for request data
  unsigned long start = millis();
  while (!client.available() && millis() - start < 500) delay(1);

  // --- Read Request Line ---
  requestLine = client.readStringUntil('\n');
  requestLine.trim();
  Serial.print("Request Line: ");
  Serial.println(requestLine);

  // --- Read Headers ---
  while (client.connected()) {
    headerLine = client.readStringUntil('\n');
    headerLine.trim();
    if (headerLine.length() == 0) break;
    Serial.println(headerLine);

    if (headerLine.startsWith("Content-Length:") || headerLine.startsWith("content-length:")) {
      contentLength = headerLine.substring(headerLine.indexOf(':') + 1).toInt();
    }
  }

  Serial.print("Parsed Content-Length: ");
  Serial.println(contentLength);

  // === ROUTE HANDLING ===

  // --- POST /set ---
  if (requestLine.startsWith("POST /set")) {
    if (useUdp) {
      client.println("HTTP/1.1 403 Forbidden");
      client.println("Content-Type: application/json");
      client.println("Connection: close");
      client.println();
      client.println("{\"error\":\"API access is disabled while UDP is active.\"}");
      client.stop();
      return;
    }

    String body = "";
    int bytesRead = 0;
    unsigned long bodyStart = millis();
    while (bytesRead < contentLength && millis() - bodyStart < 2000) {
      if (client.available()) {
        char c = client.read();
        body += c;
        bytesRead++;
      }
    }

    Serial.print("Received JSON Body: ");
    Serial.println(body);

    handleApiJson(client, body);
    client.stop();
    return;
  }

  // --- GET /status ---
  if (requestLine.startsWith("GET /status")) {
    StaticJsonDocument<128> doc;
    doc["radio1"] = radios[0].lastBandIndex;
    doc["radio2"] = radios[1].lastBandIndex;

    String response;
    serializeJson(doc, response);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println(response);
    client.stop();
    return;
  }

  // --- GET / (main page) ---
  if (requestLine.startsWith("GET / ")) {
    String html = R"rawliteral(
    <html><head><style>
    body{font-family:Arial,sans-serif;background:#f8f9fa;padding:20px;}
    h2{color:#333;}
    table{border-collapse:collapse;width:100%;max-width:600px;background:#fff;box-shadow:0 2px 4px rgba(0,0,0,0.1);}
    th,td{padding:10px;text-align:left;}
    tr:nth-child(even){background-color:#f2f2f2;}
    input[type=text],input[type=number]{width:100%;padding:6px;border:1px solid #ccc;border-radius:4px;}
    input[type=checkbox]{transform:scale(1.2);}
    input[type=submit]{background-color:#007bff;color:white;padding:10px 15px;border:none;border-radius:4px;cursor:pointer;}
    input[type=submit]:hover{background-color:#0056b3;}
    </style></head><body>
    <h2>TCP SO2R / Band Decoder Controller (c) YO3HEX</h2>
    <form method='GET' action='/save'><table>
    )rawliteral";

    html += "<tr><td>IP Address</td><td><input name='ip' value='" + netConfig.ip.toString() + "'></td></tr>";
    html += "<tr><td>Gateway</td><td><input name='gw' value='" + netConfig.gateway.toString() + "'></td></tr>";
    html += "<tr><td>Subnet</td><td><input name='subnet' value='" + netConfig.subnet.toString() + "'></td></tr>";
    html += "<tr><td>DNS</td><td><input name='dns' value='" + netConfig.dns.toString() + "'></td></tr>";
    html += "<tr><td>UDP Port</td><td><input name='port' value='" + String(netConfig.udpPort) + "'></td></tr>";
    html += "<tr><td>Enable UDP</td><td><input type='checkbox' name='useudp' ";
    if (useUdp) html += "checked";
    html += "></td></tr>";
    html += "<tr><td>TCP Port</td><td><input name='tcpport' value='" + String(netConfig.tcpPort) + "'></td></tr>";
    html += "<tr><td>Enable TCP</td><td><input type='checkbox' name='usetcp' ";
    if (useTcp) html += "checked";
    html += "></td></tr>";
    html += "<tr><td colspan='2' style='text-align:center'><input type='submit' value='Save'></td></tr>";
    html += "</table><hr>FW Ver: ";
    html += FIRMWARE_VERSION;
    html += "</form></body></html>";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.print(html);
    client.stop();
    return;
  }

  // --- GET /save? ---
  if (requestLine.startsWith("GET /save?")) {
    String params = requestLine.substring(requestLine.indexOf("/save?") + 6);
    params.replace(" HTTP/1.1", "");

    String ip = "", gw = "", subnet = "", dns = "", port = "", tcpPort = "";
    int i1 = params.indexOf("ip=");
    int i2 = params.indexOf("&gw=");
    int i3 = params.indexOf("&subnet=");
    int i4 = params.indexOf("&dns=");
    int i5 = params.indexOf("&port=");
    int i6 = params.indexOf("&tcpport=");
    bool udpCheckbox = params.indexOf("useudp") >= 0;
    bool tcpCheckbox = params.indexOf("usetcp") >= 0;

    if (i1 >= 0 && i2 >= 0 && i3 >= 0 && i4 >= 0 && i5 >= 0 && i6 >= 0) {
      ip = params.substring(i1 + 3, i2);
      gw = params.substring(i2 + 4, i3);
      subnet = params.substring(i3 + 8, i4);
      dns = params.substring(i4 + 5, i5);
      port = params.substring(i5 + 6);
      tcpPort = params.substring(i6 + 9);
      saveConfigFromWeb(client, ip, gw, subnet, dns, port.toInt(), udpCheckbox, tcpPort.toInt(), tcpCheckbox);
      return;
    }
  }

  delay(1);
  client.stop();
}



void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(RESET_CONFIG_PIN, INPUT_PULLUP);

  pinMode(RX1, OUTPUT);
  pinMode(TX1, OUTPUT);
  pinMode(TX2, OUTPUT);
  pinMode(RX2, OUTPUT);

  delay(100);
  bool resetRequired = digitalRead(RESET_CONFIG_PIN) == LOW;

  for (int i = 0; i < MAX_RADIOS; ++i) {
    radios[i].radioNr = i + 1;
    radios[i].lastBandIndex = -1;
  }

  loadConfig(resetRequired);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, ETH_CS);
  Ethernet.init(ETH_CS);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_ETH);
  Ethernet.begin(mac, netConfig.ip, netConfig.gateway, netConfig.subnet);

  checkEthernetLink(true);

  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());

  server.begin();
  Serial.println("Config server started");

  if (useUdp) {
    Udp.begin(netConfig.udpPort);
    Serial.printf("Listening on UDP port %u...\n", netConfig.udpPort);
  } else {
    Serial.println("UDP disabled by configuration");
  }

  if (useTcp) {
    tcpServer = EthernetServer(netConfig.tcpPort);
    tcpServer.begin();
    Serial.printf("TCP server started on port %u...\n", netConfig.tcpPort);
  } else {
    Serial.println("TCP disabled by configuration");
  }

  sr.setAllLow();
}

void handleUdpPacket() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    memset(packetBuffer, 0, sizeof(packetBuffer));
    int len = Udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    packetBuffer[len] = '\0';

    char tempBuf[16];
    bool freqOk = findTagValue(packetBuffer, "Freq", tempBuf, sizeof(tempBuf));
    long freq = freqOk ? atol(tempBuf) : 0;

    bool radioOk = findTagValue(packetBuffer, "RadioNr", tempBuf, sizeof(tempBuf));
    int radioNr = radioOk ? atoi(tempBuf) : 0;

    if (freqOk && radioOk) {
      int bandIndex = getBandIndex(freq);
      RadioState * state = getRadioState(radioNr);

      if (state && bandIndex != state -> lastBandIndex) {
        Serial.printf("Radio %d → Band #%d (%ld Hz)\n", radioNr, bandIndex, freq);
        state -> lastBandIndex = bandIndex;
        updateShiftRegister(radioNr - 1);
      }
    }
  }
}


void handleTcpClient() {
  static EthernetClient tcpClient;
  static char lineBuffer[256];
  static int linePos = 0;

  // Accept new client if previous is gone
  if (!tcpClient || !tcpClient.connected()) {
    if (tcpClient.connected()) tcpClient.stop(); // only stop if needed
    tcpClient = tcpServer.available();
    linePos = 0;
  }

  // Process incoming data from connected client
  if (tcpClient && tcpClient.connected() && tcpClient.available()) {
    while (tcpClient.available()) {
      char c = tcpClient.read();
      if (c == '\n' || c == '\r') {
        if (linePos > 0) {
          lineBuffer[linePos] = '\0';
          //Serial.print("TCP Client: ");
          //Serial.println(lineBuffer);

          // Simple command parsing
          if (strcmp(lineBuffer, "RX1") == 0) {
            //Serial.println("RX1 detected → lighting LED for RX1");
            digitalWrite(RX1, HIGH);
            digitalWrite(RX2, LOW);
          } else if (strcmp(lineBuffer, "RX2") == 0) {
            //Serial.println("RX2 detected → lighting LED for RX2");
            digitalWrite(RX1, LOW);
            digitalWrite(RX2, HIGH);
          } else if (strcmp(lineBuffer, "RX1S") == 0) {
            digitalWrite(RX1, HIGH);
            digitalWrite(RX2, HIGH);
          } else if (strcmp(lineBuffer, "TX1") == 0) {
            digitalWrite(TX1, HIGH);
            digitalWrite(TX2, LOW);
          } else if (strcmp(lineBuffer, "TX2") == 0) {
            digitalWrite(TX1, LOW);
            digitalWrite(TX2, HIGH);
          } 
          else {
            Serial.println("Unknown command");
          }
          linePos = 0;
        }
      } else if (linePos < sizeof(lineBuffer) - 1) {
        lineBuffer[linePos++] = c;
      }
    }
  }
}

void loop() {

  if (useUdp) handleUdpPacket();
  if (useTcp) handleTcpClient();

  checkEthernetLink(false);

  serveWebConfig();
  yield();
}