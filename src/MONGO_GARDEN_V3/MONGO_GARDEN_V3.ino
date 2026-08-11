//libsAndUtils 
  #include <Arduino.h>
  #include <Wire.h>
  #include <ThreeWire.h>
  #include <RtcDS1302.h>
  #include <DHT.h>
  #include <DHT_U.h>
  #include <Adafruit_Sensor.h>
  #include <SD.h>
  #include <SPI.h>
  #include <WiFi101.h>
  #include <MQTT.h>
  #include <ArduinoJson.h>
  #define USE_WIFI101_PRO           true
  #include <Adafruit_BMP085.h>
  #include <WebSocketsServer_Generic.h>
  #include <Adafruit_TSL2561_U.h>
  #define BATTERYPIN A6
  #define SIG1 A0
  #define SIG2 A1
  #define RELAYR A2
  #define RELAYL A3
  #define RELAYS A4
  #define S0 0
  #define S1 1
  #define S2 2
  #define S3 3
  #define EN 4
  enum FunctionMode { SET_AND_READ = 0, SET_ONLY = 1, READ_ONLY = 2 };
  enum GardenSide { RIGHT_SIDE = 0, LEFT_SIDE = 1, BOTH_SIDES = 2 };
  const int CS_PIN = 7;

//startVars-
  Adafruit_BMP085 bmp;
    float barometricPressure;
    float barometerAirTemperature;
    bool BMP_ONLINE = false;

  ThreeWire wireClock(13, 14, A5);
  RtcDS1302<ThreeWire> Rtcmod(wireClock);
  String dateTime;


  DHT dhtexterno(5, DHT11);
    float internalTemp;
    float internalAirHumidity;


  DHT dhtinterno(6, DHT11);
    float externalAirTemp;
    float externalAirHumidity;

  WiFiClient net;
  WiFiServer server(80);          // Servidor HTTP normal para cargar la web (Puerto 80)
  WebSocketsServer webSocket = WebSocketsServer(81); // Servidor WebSocket (Puerto 81)

  MQTTClient mqttClient(256);
    char MQTT_HOST[48] = "157.137.230.39";
    char MQTT_USERNAME[48] = "mkr1000-Main";
    char MQTT_PASSWORD[65] = "BDAFE5BE";
    int  MQTT_PORT = 1883;

    bool mqttConectado = false;
    
    // Usaremos segundos del RTC como "tiempo base" cuando sea posible
    long lastDhtPublishRtcSeconds  = -1;
    long lastBmpPublishRtcSeconds  = -1;
    long lastTslPublishRtcSeconds  = -1;
    long lastSoilPublishRtcSeconds = -1;

    long DHT_INTERVAL_SEC  = 60;
    long BMP_INTERVAL_SEC  = 1800;
    long TSL_INTERVAL_SEC  = 1800;
    long SOIL_INTERVAL_SEC = 3600;
    String currentTimeSource = "";      // "RTC" o "MILLIS"

  Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
    bool TSL_ONLINE = false;
    float uvIndex;
  
  File archivo;
  

  
  int humidityValuesRight[16];
  int humidityValuesLeft[16];
  
  
  int pingResult;
  int status = 0;
  String hostName = "www.google.com";
  bool wifiConectado = false;
  bool serverRunning = false;
  char ap_ssid[] = "MONGO_GARDEN";
  char ap_pass[] = "mongo_pwr";
  
// ============================================================
// CONFIGURACIÓN PERSISTENTE: SD / WiFi / MQTT / TELEMETRÍA
// ============================================================

  bool sdOnline = false;

  const char* STARTUP_DIR       = "/startup";
  const char* DATA_DIR          = "/data";
  const char* STARTUP_JSON_PATH = "/startup/startUpVars.json";
  const char* SENSOR_CSV_PATH   = "/data/sensor_log.csv";

  // Tres redes WiFi: se probarán en este orden.
  const uint8_t WIFI_NETWORK_COUNT = 3;

  char wifiSsids[WIFI_NETWORK_COUNT][33] = {
    "Semillero ASI",
    "A15 de SANTIAGO",
    "WIFI-ITM"
  };

  char wifiPasswords[WIFI_NETWORK_COUNT][65] = {
    "semilleroasik601",
    "1036451694",
    ""
  };

  // Frecuencias editables desde startUpVars.json.
  unsigned long DHT_INTERVAL_MS  = 60UL * 1000UL;
  unsigned long BMP_INTERVAL_MS  = 1800UL * 1000UL;
  unsigned long TSL_INTERVAL_MS  = 1800UL * 1000UL;
  unsigned long SOIL_INTERVAL_MS = 3600UL * 1000UL;

  // Cada cuánto se guarda una fila completa en CSV cuando MQTT está caído.
  // Por defecto queda cada hora, igual al intervalo de suelo.
  unsigned long OFFLINE_SNAPSHOT_INTERVAL_MS = 3600UL * 1000UL;


  unsigned long lastOfflineSnapshotMillis = 0;
  unsigned long lastWiFiRetryMillis = 0;

  const unsigned long WIFI_RETRY_INTERVAL_MS = 60000UL; // Reintenta conexión cada 60 s



  float battery;
//end

void setup() 
{
  pinMode(BATTERYPIN, INPUT);
  pinMode(RELAYR, OUTPUT);
  pinMode(RELAYL, OUTPUT);
  pinMode(SIG1, INPUT);
  pinMode(SIG2, INPUT);
  pinMode(RELAYS, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, OUTPUT); // CS PIN
  //--------------ESTO NO HACE FALTA DECLARARLO, PERO SE DEJA COMENTADO PARA TERMINOS PRACTICOS------
  // pinMode(8, OUTPUT); MOSI PIN
  // pinMode(9, OUTPUT); SCK PIN
  // pinMode(10, OUTPUT); MISO PIN
  // pinMode(11, OUTPUT); I/O PIN
  // pinMode(12, INPUT); SCLK
  // pinMode(13, INPUT); RST 
  //-------------------------------------------------------------------------------------------------
  Serial.begin(9600);
  delay(5000);
  Serial.println("..--..--..--..");
  Serial.println("Serial inicializado");
  dhtexterno.begin();
  Serial.println("DHT11 Externo inicializado");
  dhtinterno.begin();
  Serial.println("DHT11 Interno inicializado");
  Rtcmod.Begin();
  Serial.println("Modulo de reloj inicializado - NC");
  if (!bmp.begin()) {Serial.println("Error inicializando BMP180"); BMP_ONLINE = false;} else {Serial.println("BMP180 inicializado"); BMP_ONLINE = true;}
  if (!tsl.begin()) {Serial.println("Error inicializando TSL2561"); TSL_ONLINE = false;} else {Serial.println("TSL2561 inicializado"); tsl.setGain(TSL2561_GAIN_1X); tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS); TSL_ONLINE = true;}
  if (!initStorageAndConfiguration()) {
  Serial.println("ADVERTENCIA: SD o configuracion no disponible");}
  Serial.println("Version - 2026-08-11-13-54");
  Serial.println("------FIN INICIO------");
  digitalWrite(RELAYS, LOW);
  digitalWrite(RELAYR, LOW);
  digitalWrite(RELAYL, LOW);

  if (!connectWiFi()) {
  Serial.println("Advertencia: WiFi no disponible. Se activara respaldo CSV.");}

  connectMQTT();

}

void loop()
{
  // Reintento de WiFi cada 60 segundos si se perdió la conexión.
  if (WiFi.status() != WL_CONNECTED) {
    wifiConectado = false;

    if (millis() - lastWiFiRetryMillis >= WIFI_RETRY_INTERVAL_MS) {
      lastWiFiRetryMillis = millis();
      connectWiFi();
    }
  } else {
    wifiConectado = true;
  }

  // MQTT solo se gestiona si WiFi está disponible.
  if (wifiConectado) {
    if (!mqttClient.connected()) {
      mqttConectado = false;
      connectMQTT();
    }

    if (mqttClient.connected()) {
      mqttConectado = true;
      mqttClient.loop();
    }
  }


  // Los comandos Serial siguen funcionando incluso sin red.
  serialRead();
}

void serialRead()
{
    if (Serial.available() <= 0) return;

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    if (handleSimpleCommand(input)) return;
    if (handleWiFiCommand(input)) return;
    if (handleSetTimeCommand(input)) return;
    if (handleValveCommand(input)) return;
    if (handleSoilCommand(input)) return;

    Serial.println("Comando no reconocido. Usa uno de estos:");
    Serial.println("  GET_BATTERY");
    Serial.println("  SYNC_RTC");
    Serial.println("  GET_TIME");
    Serial.println("  GET_BMP_DATA");
    Serial.println("  GET_INTERNAL_DHT");
    Serial.println("  GET_EXTERNAL_DHT");
    Serial.println("  GET_TSL_DATA");
    Serial.println("  SET_TIME,2026,07,26,21,45,00");
    Serial.println("  SET_WIFI,1,\"SSID\",\"PASSWORD\"");
    Serial.println("  OPEN_RIGHT_VALVE,2000");
    Serial.println("  OPEN_LEFT_VALVE,2000");
    Serial.println("  SET_AND_READ, BOTH_SIDES, 5, 5");
    Serial.println("|---|");
}

/*void handleTelemetryLoop()
{
  demoMode = digitalRead(DEMOPIN);
  if (demoMode == 0) {
    return;
  }

  unsigned long nowMillis = millis();

  long nowRtcSeconds = getRtcSeconds();
  bool rtcOk = (nowRtcSeconds >= 0);

  if (rtcOk) {
    publishTimeSource("RTC");
  } else {
    publishTimeSource("MILLIS");
  }

  // -----------------------------
  // DHT interno / externo
  // -----------------------------
  if (rtcOk) {
    if (lastDhtPublishRtcSeconds < 0 ||
        (nowRtcSeconds - lastDhtPublishRtcSeconds) >= DHT_INTERVAL_SEC) {

      publishDhtInternal();
      publishDhtExternal();
      lastDhtPublishRtcSeconds = nowRtcSeconds;
    }
  } else {
    static unsigned long lastDhtPublishMillis = 0;
    if (lastDhtPublishMillis == 0 ||
        (nowMillis - lastDhtPublishMillis) >= DHT_INTERVAL_MS) {

      publishDhtInternal();
      publishDhtExternal();
      lastDhtPublishMillis = nowMillis;
    }
  }

  // -----------------------------
  // BMP180
  // -----------------------------
  if (rtcOk) {
    if (lastBmpPublishRtcSeconds < 0 ||
        (nowRtcSeconds - lastBmpPublishRtcSeconds) >= BMP_INTERVAL_SEC) {

      publishBmp();
      lastBmpPublishRtcSeconds = nowRtcSeconds;
    }
  } else {
    static unsigned long lastBmpPublishMillis = 0;
    if (lastBmpPublishMillis == 0 ||
        (nowMillis - lastBmpPublishMillis) >= BMP_INTERVAL_MS) {

      publishBmp();
      lastBmpPublishMillis = nowMillis;
    }
  }

  // -----------------------------
  // TSL2561
  // -----------------------------
  if (rtcOk) {
    if (lastTslPublishRtcSeconds < 0 ||
        (nowRtcSeconds - lastTslPublishRtcSeconds) >= TSL_INTERVAL_SEC) {

      publishTsl();
      lastTslPublishRtcSeconds = nowRtcSeconds;
    }
  } else {
    static unsigned long lastTslPublishMillis = 0;
    if (lastTslPublishMillis == 0 ||
        (nowMillis - lastTslPublishMillis) >= TSL_INTERVAL_MS) {

      publishTsl();
      lastTslPublishMillis = nowMillis;
    }
  }

  // -----------------------------
  // Humedad de suelo (todos los slots)
  // -----------------------------
  if (rtcOk) {
    if (lastSoilPublishRtcSeconds < 0 ||
        (nowRtcSeconds - lastSoilPublishRtcSeconds) >= SOIL_INTERVAL_SEC) {

      publishSoilAll();
      lastSoilPublishRtcSeconds = nowRtcSeconds;
    }
  } else {
    static unsigned long lastSoilPublishMillis = 0;
    if (lastSoilPublishMillis == 0 ||
        (nowMillis - lastSoilPublishMillis) >= SOIL_INTERVAL_MS) {

      publishSoilAll();
      lastSoilPublishMillis = nowMillis;
    }
  }
} */

void mqttMessageReceived(String &topic, String &payload)
{
  payload.trim();

  Serial.print("MQTT mensaje recibido | Topic: ");
  Serial.print(topic);
  Serial.print(" | Payload: ");
  Serial.println(payload);

  // ===================================
  // COMANDOS DE VÁLVULAS
  // ===================================

  if (topic == "mongo_garden/cmd/openRightValve") {
    if (!isValidInteger(payload)) {
      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: tiempo invalido para openRightValve| " + String(getTime())
      );
      return;
    }

    long timeValue = payload.toInt();

    if (timeValue <= 0) {
      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: tiempo debe ser mayor que 0| " + String(getTime())
      );
      return;
    }

    String result = openRightValve((unsigned long)timeValue);

    Serial.println(result);
    mqttClient.publish("mongo_garden/status/system", result + " | " + String(getTime()));

    return;
  }

  if (topic == "mongo_garden/cmd/openLeftValve") {
    if (!isValidInteger(payload)) {
      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: tiempo invalido para openLeftValve| " + String(getTime())
      );
      return;
    }

    long timeValue = payload.toInt();

    if (timeValue <= 0) {
      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: tiempo debe ser mayor que 0| " + String(getTime())
      );
      return;
    }

    String result = openLeftValve((unsigned long)timeValue);

    Serial.println(result);
    mqttClient.publish("mongo_garden/status/system", result + " | " + String(getTime()));

    return;
  }

  // ===================================
  // COMANDOS DE LECTURA INDIVIDUAL
  // ===================================

  // Leer y publicar estado de la bateria
  if (topic == "mongo_garden/cmd/read/battery") {
    battery = analogRead(BATTERYPIN);
    mqttClient.publish("mongo_garden/telemetry/battery", String(battery));

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_BATTERY_COMPLETED| " + String(getTime())
    );

    return;
  }

  // Leer y publicar DHT interno
  if (topic == "mongo_garden/cmd/read/dht/internal") {
    publishDhtInternal();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_DHT_INTERNAL_COMPLETED| " + String(getTime())
    );

    return;
  }

  // Leer y publicar DHT externo
  if (topic == "mongo_garden/cmd/read/dht/external") {
    publishDhtExternal();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_DHT_EXTERNAL_COMPLETED| " + String(getTime())
    );

    return;
  }

  // Leer y publicar BMP180
  if (topic == "mongo_garden/cmd/read/bmp") {
    publishBmp();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_BMP_COMPLETED| " + String(getTime())
    );

    return;
  }

  // Leer y publicar TSL2561
  if (topic == "mongo_garden/cmd/read/tsl") {
    publishTsl();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_TSL_COMPLETED| " + String(getTime())
    );

    return;
  }

  // Leer y publicar todos los slots de suelo
  if (topic == "mongo_garden/cmd/read/soil/all") {
    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_SOIL_ALL_STARTED| " + String(getTime())
    );

    // publishSoilAll() debe usar measureSoilSlot()
    // para activar RELAYS, seleccionar canal y leer ambos lados.
    publishSoilAll();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_SOIL_ALL_COMPLETED " + String(getTime())
    );

    return;
  }

  // Leer un slot puntual de ambos lados
  // Payload esperado: canal,slot
  // Ejemplo: 5,5
  if (topic == "mongo_garden/cmd/read/soil/slot") {
    int commaPosition = payload.indexOf(',');

    if (commaPosition == -1) {
      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: payload soil slot debe ser canal,slot| " + String(getTime())
      );
      return;
    }

    String channelText = payload.substring(0, commaPosition);
    String slotText = payload.substring(commaPosition + 1);

    channelText.trim();
    slotText.trim();

    if (!isValidInteger(channelText) ||
        !isValidInteger(slotText)) {

      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: canal o slot no numerico| " + String(getTime())
      );

      return;
    }

    int channel = channelText.toInt();
    int slot = slotText.toInt();

    if (channel < 0 || channel > 15 ||
        slot < 0 || slot > 15) {

      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: canal y slot deben estar entre 0 y 15| " + String(getTime())
      );

      return;
    }

    // Misma secuencia que funciona por Serial:
    // activar relé maestro -> leer ambos lados -> apagar relé
    digitalWrite(RELAYS, HIGH);
    delay(500);

    String result = getSoilHumidity(
      SET_AND_READ,
      BOTH_SIDES,
      channel,
      slot
    );

    delay(500);
    digitalWrite(RELAYS, LOW);

    Serial.println(result);

    // Publicar resultado crudo de lectura puntual
    mqttClient.publish(
      "mongo_garden/telemetry/soil/onDemand",
      result
    );

    // Publicar los valores por topic individual también
    String topicRight =
      "mongo_garden/telemetry/soil/rightside/" + String(slot);

    String topicLeft =
      "mongo_garden/telemetry/soil/leftside/" + String(slot);

    mqttClient.publish(
      topicRight,
      String(humidityValuesRight[slot])
    );

    mqttClient.publish(
      topicLeft,
      String(humidityValuesLeft[slot])
    );

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_SOIL_SLOT_COMPLETED| " + String(getTime())
    );

    return;
  }

  // ===================================
  // COMANDOS DE RTC
  // ===================================

  if (topic == "mongo_garden/cmd/setTime") {
    String result = handleSetTimePayload(payload);

    Serial.println(result);
    mqttClient.publish("mongo_garden/status/rtc", result);

    return;
  }

  if (topic == "mongo_garden/cmd/syncRtc") {
    String result = syncRtcWithInternet();

    Serial.println(result);
    mqttClient.publish("mongo_garden/status/rtc", result);

    return;
  }

  // ===================================
  // COMANDOS WIFI
  // ===================================

  if (topic == "mongo_garden/cmd/setWiFiParameters") {
    String result = handleSetWiFiPayload(payload);

    Serial.println(result);

    // Se envía antes de desconectar MQTT.
    mqttClient.publish(
      "mongo_garden/status/wifi",
      result + " | reiniciando conexion WiFi"
    );

    // Se fuerza un nuevo ciclo de conexión.
    WiFi.disconnect();
    wifiConectado = false;
    mqttConectado = false;

    if (connectWiFi()) {
      connectMQTT();
    }

    return;
  }

  // ===================================
  // COMANDO readSoil ORIGINAL
  // ===================================
  // Payload:
  // SET_AND_READ,BOTH_SIDES,5,5

  if (topic == "mongo_garden/cmd/readSoil") {
    String result = handleReadSoilPayload(payload);

    Serial.println(result);

    mqttClient.publish(
      "mongo_garden/telemetry/soil/onDemand",
      result
    );

    return;
  }

  // ===================================
  // COMANDO GENÉRICO DE SISTEMA
  // ===================================

  if (topic == "mongo_garden/cmd/system") {
    String result = "SYSTEM_CMD:" + payload;

    Serial.println(result);
    mqttClient.publish("mongo_garden/status/system", result + " | " + String(getTime()));

    return;
  }

  // ===================================
  // TOPIC DESCONOCIDO
  // ===================================

  Serial.println("MQTT comando no reconocido");

  mqttClient.publish(
    "mongo_garden/status/system",
    "ERROR: MQTT_TOPIC_NOT_RECOGNIZED| " + String(getTime())
  );
}

bool handleSetTimeCommand(String input)
{
    if (!input.startsWith("SET_TIME,")) {
      return false;
    }

    int p1 = input.indexOf(',');
    int p2 = input.indexOf(',', p1 + 1);
    int p3 = input.indexOf(',', p2 + 1);
    int p4 = input.indexOf(',', p3 + 1);
    int p5 = input.indexOf(',', p4 + 1);
    int p6 = input.indexOf(',', p5 + 1);

    // Deben existir exactamente 6 valores después de SET_TIME
    if (p1 == -1 || p2 == -1 || p3 == -1 || p4 == -1 || p5 == -1) {
      Serial.println("Formato invalido. Usa: SET_TIME,YYYY,MM,DD,HH,MM,SS");
      return true;
    }

    String yearText   = input.substring(p1 + 1, p2);
    String monthText  = input.substring(p2 + 1, p3);
    String dayText    = input.substring(p3 + 1, p4);
    String hourText   = input.substring(p4 + 1, p5);
    String minuteText = (p6 == -1) ? input.substring(p5 + 1) : input.substring(p5 + 1, p6);
    String secondText = "";

    if (p6 == -1) {
      Serial.println("Formato invalido. Usa: SET_TIME,YYYY,MM,DD,HH,MM,SS");
      return true;
    } else {
      secondText = input.substring(p6 + 1);
    }

    yearText.trim();
    monthText.trim();
    dayText.trim();
    hourText.trim();
    minuteText.trim();
    secondText.trim();

    if (!isValidInteger(yearText) ||
        !isValidInteger(monthText) ||
        !isValidInteger(dayText) ||
        !isValidInteger(hourText) ||
        !isValidInteger(minuteText) ||
        !isValidInteger(secondText)) {
      Serial.println("Formato invalido. Todos los campos de fecha y hora deben ser numericos");
      return true;
    }

    int year   = yearText.toInt();
    int month  = monthText.toInt();
    int day    = dayText.toInt();
    int hour   = hourText.toInt();
    int minute = minuteText.toInt();
    int second = secondText.toInt();

    Serial.println(setManualTime(year, month, day, hour, minute, second));
    return true;
}

String handleSetTimePayload(String payload)
{
  // Esperamos: YYYY,MM,DD,HH,MM,SS
  int p1 = payload.indexOf(',');
  int p2 = payload.indexOf(',', p1 + 1);
  int p3 = payload.indexOf(',', p2 + 1);
  int p4 = payload.indexOf(',', p3 + 1);
  int p5 = payload.indexOf(',', p4 + 1);

  if (p1 == -1 || p2 == -1 || p3 == -1 || p4 == -1 || p5 == -1) {
    return "Formato invalido. Usa: YYYY,MM,DD,HH,MM,SS";
  }

  String yearText   = payload.substring(p1 + 1, p2);
  String monthText  = payload.substring(p2 + 1, p3);
  String dayText    = payload.substring(p3 + 1, p4);
  String hourText   = payload.substring(p4 + 1, p5);
  String minuteText = payload.substring(p5 + 1);
  String secondText = "";   // Si quieres segundo separado, ajusta

  // Aquí puedo asumir que minuteText ya incluye segundos en otro formato,
  // pero para mantenerlo simple uso 6 campos.

  // Arreglo para 6 campos:
  int p6 = payload.indexOf(',', p5 + 1);
  if (p6 == -1) {
    return "Formato invalido. Usa: YYYY,MM,DD,HH,MM,SS";
  }

  minuteText = payload.substring(p5 + 1, p6);
  secondText = payload.substring(p6 + 1);

  yearText.trim();
  monthText.trim();
  dayText.trim();
  hourText.trim();
  minuteText.trim();
  secondText.trim();

  if (!isValidInteger(yearText) ||
      !isValidInteger(monthText) ||
      !isValidInteger(dayText) ||
      !isValidInteger(hourText) ||
      !isValidInteger(minuteText) ||
      !isValidInteger(secondText)) {
    return "Formato invalido. Todos los campos deben ser numericos";
  }

  int year   = yearText.toInt();
  int month  = monthText.toInt();
  int day    = dayText.toInt();
  int hour   = hourText.toInt();
  int minute = minuteText.toInt();
  int second = secondText.toInt();

  return setManualTime(year, month, day, hour, minute, second);
}

bool handleSimpleCommand(String input)
{
    if (input == "GET_BATTERY") {
      battery = analogRead(BATTERYPIN);
      Serial.println("ESTADO DE LA BATERÍA: " + String(battery));
      return true;
    }
    
    if (input == "SYNC_RTC") {
      connectWiFi();
      Serial.println(syncRtcWithInternet());
      return true;
    }

    if (input == "GET_TIME") {
      Serial.println(getTime());
      return true;
    }

    if (input == "GET_BMP_DATA") {
      Serial.println(getBarometricPressure());
      return true;
    }

    if (input == "GET_INTERNAL_DHT") {
      Serial.println(getInternalTemperature());
      return true;
    }

    if (input == "GET_EXTERNAL_DHT") {
      Serial.println(getExternalTemperature());
      return true;
    }

    if (input == "GET_TSL_DATA") {
      Serial.println(getTSL2561());
      return true;
    }

    return false;
}

bool handleSoilCommand(String input)
{
    int firstComma  = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);
    int thirdComma  = input.indexOf(',', secondComma + 1);

    // Si no hay 3 comas, no parece ser este comando
    if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
      return false;
    }

    // Extraer campos
    String modeText    = input.substring(0, firstComma);
    String sideText    = input.substring(firstComma + 1, secondComma);
    String channelText = input.substring(secondComma + 1, thirdComma);
    String slotText    = input.substring(thirdComma + 1);

    modeText.trim();
    sideText.trim();
    channelText.trim();
    slotText.trim();

    // Validar campos vacíos
    if (modeText.length() == 0 || sideText.length() == 0 || channelText.length() == 0 || slotText.length() == 0) {
      Serial.println("Formato invalido. Hay argumentos vacios");
      return true;
    }

    // Convertir textos a enum
    FunctionMode mode;
    GardenSide side;

    if (modeText == "SET_AND_READ") {
      mode = SET_AND_READ;
    }
    else if (modeText == "SET_ONLY") {
      mode = SET_ONLY;
    }
    else if (modeText == "READ_ONLY") {
      mode = READ_ONLY;
    }
    else {
      Serial.println("functionMode invalido");
      return true;
    }

    if (sideText == "RIGHT_SIDE") {
      side = RIGHT_SIDE;
    }
    else if (sideText == "LEFT_SIDE") {
      side = LEFT_SIDE;
    }
    else if (sideText == "BOTH_SIDES") {
      side = BOTH_SIDES;
    }
    else {
      Serial.println("gardenSide invalido");
      return true;
    }

    // Validar que los textos numéricos sí sean enteros válidos
    if (!isValidInteger(channelText)) {
      Serial.println("channelMultiplexor invalido");
      return true;
    }

    if (!isValidInteger(slotText)) {
      Serial.println("soilSlot invalido");
      return true;
    }

    int channelMultiplexor = channelText.toInt();
    int soilSlot = slotText.toInt();

    digitalWrite(RELAYS, HIGH);
    delay(500);
    Serial.println(getSoilHumidity(mode, side, channelMultiplexor, soilSlot));
    delay(500);
    digitalWrite(RELAYS, LOW);
    return true;
}

String handleReadSoilPayload(String payload)
{
  int firstComma  = payload.indexOf(',');
  int secondComma = payload.indexOf(',', firstComma + 1);
  int thirdComma  = payload.indexOf(',', secondComma + 1);

  if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
    return "Formato invalido. Usa: SET_AND_READ, BOTH_SIDES, channel, slot";
  }

  String modeText    = payload.substring(0, firstComma);
  String sideText    = payload.substring(firstComma + 1, secondComma);
  String channelText = payload.substring(secondComma + 1, thirdComma);
  String slotText    = payload.substring(thirdComma + 1);

  modeText.trim();
  sideText.trim();
  channelText.trim();
  slotText.trim();

  FunctionMode mode;
  GardenSide side;

  if (!parseFunctionMode(modeText, mode)) {
    return "functionMode invalido";
  }

  if (!parseGardenSide(sideText, side)) {
    return "gardenSide invalido";
  }

  if (!isValidInteger(channelText)) {
    return "channelMultiplexor invalido";
  }
  if (!isValidInteger(slotText)) {
    return "soilSlot invalido";
  }

  int channelMultiplexor = channelText.toInt();
  int soilSlot           = slotText.toInt();

  return getSoilHumidity(mode, side, channelMultiplexor, soilSlot);
}

bool isValidInteger(String text)
{
    if (text.length() == 0) return false;

    int startIndex = 0;

    if (text[0] == '-') {
      if (text.length() == 1) return false;
      startIndex = 1;
    }

    for (int i = startIndex; i < text.length(); i++) {
      if (!isDigit(text[i])) {
        return false;
      }
    }

    return true;
}

bool handleWiFiCommand(String input)
{
  const String prefix = "SET_WIFI,";

  if (!input.startsWith(prefix)) {
    return false;
  }

  // Formato:
  // SET_WIFI,1,"SSID","PASSWORD"

  int firstComma = input.indexOf(',');
  int secondComma = input.indexOf(',', firstComma + 1);

  if (firstComma == -1 || secondComma == -1) {
    Serial.println("Formato invalido. Usa: SET_WIFI,1,\"SSID\",\"PASSWORD\"");
    return true;
  }

  String networkIndexText = input.substring(firstComma + 1, secondComma);
  networkIndexText.trim();

  if (!isValidInteger(networkIndexText)) {
    Serial.println("Indice de red invalido. Usa 1, 2 o 3");
    return true;
  }

  int networkIndex = networkIndexText.toInt();

  if (networkIndex < 1 || networkIndex > WIFI_NETWORK_COUNT) {
    Serial.println("Indice de red invalido. Usa 1, 2 o 3");
    return true;
  }

  int firstQuote = input.indexOf('"', secondComma + 1);
  int secondQuote = input.indexOf('"', firstQuote + 1);
  int thirdQuote = input.indexOf('"', secondQuote + 1);
  int fourthQuote = input.indexOf('"', thirdQuote + 1);

  if (firstQuote == -1 ||
      secondQuote == -1 ||
      thirdQuote == -1 ||
      fourthQuote == -1) {

    Serial.println("Formato invalido. Usa: SET_WIFI,1,\"SSID\",\"PASSWORD\"");
    return true;
  }

  String extraText = input.substring(fourthQuote + 1);
  extraText.trim();

  if (extraText.length() > 0) {
    Serial.println("Formato invalido. Hay caracteres extra al final");
    return true;
  }

  String newSsid = input.substring(firstQuote + 1, secondQuote);
  String newPassword = input.substring(thirdQuote + 1, fourthQuote);

  String result = setWiFiParameters(
    (uint8_t)networkIndex,
    newSsid,
    newPassword
  );

  Serial.println(result);

  // Desconecta la conexión anterior y fuerza que se prueben las tres redes.
  WiFi.disconnect();
  wifiConectado = false;
  mqttConectado = false;

  if (connectWiFi()) {
    connectMQTT();
  } else {
    Serial.println("No se pudo conectar a ninguna red configurada");
  }

  return true;
}

String handleSetWiFiPayload(String payload)
{
  // Formato MQTT:
  // 1;SSID;PASSWORD

  int firstSeparator = payload.indexOf(';');
  int secondSeparator = payload.indexOf(';', firstSeparator + 1);

  if (firstSeparator == -1 || secondSeparator == -1) {
    return "ERROR: formato invalido. Usa: indice;SSID;PASSWORD";
  }

  String networkIndexText = payload.substring(0, firstSeparator);
  String newSsid = payload.substring(firstSeparator + 1, secondSeparator);
  String newPassword = payload.substring(secondSeparator + 1);

  networkIndexText.trim();
  newSsid.trim();
  newPassword.trim();

  if (!isValidInteger(networkIndexText)) {
    return "ERROR: indice de red invalido. Usa 1, 2 o 3";
  }

  int networkIndex = networkIndexText.toInt();

  if (networkIndex < 1 || networkIndex > WIFI_NETWORK_COUNT) {
    return "ERROR: indice de red invalido. Usa 1, 2 o 3";
  }

  return setWiFiParameters(
    (uint8_t)networkIndex,
    newSsid,
    newPassword
  );
}

bool handleValveCommand(String input)
{
  String commandPrefix = "";
  bool openRight = false;
  bool openLeft = false;

  // Detectar cuál de los dos comandos llegó
  if (input.startsWith("OPEN_RIGHT_VALVE,")) {
    commandPrefix = "OPEN_RIGHT_VALVE,";
    openRight = true;
  }
  else if (input.startsWith("OPEN_LEFT_VALVE,")) {
    commandPrefix = "OPEN_LEFT_VALVE,";
    openLeft = true;
  }
  else {
    return false;
  }

  // Extraer el tiempo después de la coma
  String timeText = input.substring(commandPrefix.length());
  timeText.trim();

  // Validar que no esté vacío
  if (timeText.length() == 0) {
    Serial.println("Formato invalido. Usa: OPEN_RIGHT_VALVE,2000");
    return true;
  }

  // Validar que sea entero
  if (!isValidInteger(timeText)) {
    Serial.println("Tiempo invalido");
    return true;
  }

  long timeValue = timeText.toInt();

  // Validar rango lógico
  if (timeValue <= 0) {
    Serial.println("Tiempo invalido. Debe ser mayor que 0");
    return true;
  }

  // Ejecutar la función correspondiente
  if (openRight) {
    Serial.println(openRightValve((unsigned long)timeValue));
    return true;
  }

  if (openLeft) {
    Serial.println(openLeftValve((unsigned long)timeValue));
    return true;
  }

  return true;
}

String getSoilHumidity(FunctionMode functionMode, GardenSide gardenSide, int channelMultiplexor, int soilSlot)
{
    if (channelMultiplexor < 0 || channelMultiplexor > 15) {return "Invalid channelMultiplexor value";}
    if (soilSlot < 0 || soilSlot > 15) {return "Invalid soilSlot value";}

    String output = "";

    if (functionMode == SET_AND_READ || functionMode == SET_ONLY) {
      output = setMultiplexerChannel(channelMultiplexor);
    }

    if (functionMode == SET_AND_READ || functionMode == READ_ONLY) {
      if (gardenSide == RIGHT_SIDE || gardenSide == BOTH_SIDES) {
        readRightSensors(soilSlot);
        output += (output.length() > 0 ? " | " : "");
        output += "R" + String(soilSlot) + ": " + String(humidityValuesRight[soilSlot]);
      }

      if (gardenSide == LEFT_SIDE || gardenSide == BOTH_SIDES) {
        readLeftSensors(soilSlot);
        output += (output.length() > 0 ? " | " : "");
        output += "L" + String(soilSlot) + ": " + String(humidityValuesLeft[soilSlot]);
      }
    }
    setMultiplexerChannel(0);
    return output;
}

String setMultiplexerChannel(int channel)
{
    if (channel < 0 || channel > 15) 
      {
        digitalWrite(EN, HIGH);
        return "Canal " + String(channel) + " invalido";
      }

    digitalWrite(EN, LOW);

    digitalWrite(S0, (channel & 0x01) ? HIGH : LOW);
    digitalWrite(S1, (channel & 0x02) ? HIGH : LOW);
    digitalWrite(S2, (channel & 0x04) ? HIGH : LOW);
    digitalWrite(S3, (channel & 0x08) ? HIGH : LOW);

    return "Canal " + String(channel) + " seleccionado";
}

String readRightSensors(int slot)
{
  if (slot < 0 || slot > 15) 
    {
     return "Slot " + String(slot) + " invalido";
    }

  delay(100);
  humidityValuesRight[slot] = analogRead(SIG1);
  String outputMessage = String("R")+String(slot)+String(": ")+String(humidityValuesRight[slot]);
  return outputMessage;
}

String readLeftSensors(int slot)
{
  if (slot < 0 || slot > 15) 
    {
     return "Slot " + String(slot) + " invalido";
    }

  delay(100);
  humidityValuesLeft[slot] = analogRead(SIG2);
  String outputMessage = String("L")+String(slot)+String(": ")+String(humidityValuesLeft[slot]);
  return outputMessage;
}

String getTime()
{
  RtcDateTime ahora = Rtcmod.GetDateTime();
  return dateTime = (String(ahora.Day())+"/"+String(ahora.Month())+"/"+String(ahora.Year())+"||"+String(ahora.Hour())+":"+String(ahora.Minute())+":"+String(ahora.Second())+",");
}

long getRtcSeconds()
{
  if (!Rtcmod.IsDateTimeValid()) {
    return -1;  // indicador de que el RTC no sirve
  }

  RtcDateTime now = Rtcmod.GetDateTime();

  long seconds = (long)now.Hour() * 3600L +
                 (long)now.Minute() * 60L +
                 (long)now.Second();

  return seconds;
}

String setManualTime(int year, int month, int day, int hour, int minute, int second)
{
    // Validaciones básicas de rango
    if (year < 2000 || year > 2099) return "Year invalido";
    if (month < 1 || month > 12) return "Month invalido";
    if (day < 1 || day > 31) return "Day invalido";
    if (hour < 0 || hour > 23) return "Hour invalido";
    if (minute < 0 || minute > 59) return "Minute invalido";
    if (second < 0 || second > 59) return "Second invalido";

    // Desactivar protección de escritura y asegurar que el RTC esté corriendo
    Rtcmod.SetIsWriteProtected(false);
    Rtcmod.SetIsRunning(true);

    // Crear fecha/hora manual y escribirla al RTC
    RtcDateTime manualTime(year, month, day, hour, minute, second);
    Rtcmod.SetDateTime(manualTime);

    // Verificar lectura posterior
    RtcDateTime now = Rtcmod.GetDateTime();

    if (!now.IsValid()) {
      return "RTC actualizado, pero la fecha no parece valida";
    }

    return "RTC manual actualizado: " +
          String(now.Day()) + "/" +
          String(now.Month()) + "/" +
          String(now.Year()) + " " +
          String(now.Hour()) + ":" +
          String(now.Minute()) + ":" +
          String(now.Second());
}

String syncRtcWithInternet()
{
    const long utcOffsetSeconds = -5L * 3600L;   // Colombia UTC-5

    if (WiFi.status() != WL_CONNECTED) {
      return "WiFi no conectado";
    }

    unsigned long epoch = WiFi.getTime();

    if (epoch == 0) {
      return "No se pudo obtener hora desde internet";
    }

    epoch += utcOffsetSeconds;

    if ((long)epoch < 0) {
      return "Epoch invalido despues del ajuste horario";
    }

    RtcDateTime internetTime(epoch);
    Rtcmod.SetDateTime(internetTime);

    if (!Rtcmod.IsDateTimeValid()) {
      return "RTC actualizado, pero la fecha no parece valida";
    }

    return "RTC sincronizado: " +
          String(internetTime.Day()) + "/" +
          String(internetTime.Month()) + "/" +
          String(internetTime.Year()) + " " +
          String(internetTime.Hour()) + ":" +
          String(internetTime.Minute()) + ":" +
          String(internetTime.Second());
}

bool connectWiFi()
{
  if (WiFi.status() == WL_CONNECTED) {
    wifiConectado = true;
    return true;
  }

  const uint8_t maxRetriesPerNetwork = 3;
  const unsigned long connectionTimeoutMs = 8000UL;
  const unsigned long retryDelayMs = 2500UL;

  wifiConectado = false;

  for (uint8_t networkIndex = 0; networkIndex < WIFI_NETWORK_COUNT; networkIndex++) {
    // No intenta una red no configurada.
    if (strlen(wifiSsids[networkIndex]) == 0) {
      continue;
    }

    Serial.print("Probando red WiFi ");
    Serial.print(networkIndex + 1);
    Serial.print(": ");
    Serial.println(wifiSsids[networkIndex]);

    for (uint8_t attempt = 0; attempt < maxRetriesPerNetwork; attempt++) {
      Serial.print("Intento ");
      Serial.print(attempt + 1);
      Serial.print("/");
      Serial.println(maxRetriesPerNetwork);

      WiFi.begin(wifiSsids[networkIndex], wifiPasswords[networkIndex]);

      unsigned long startedAt = millis();

      while (millis() - startedAt < connectionTimeoutMs) {
        if (WiFi.status() == WL_CONNECTED) {
          wifiConectado = true;

          Serial.println("WiFi conectado");
          Serial.print("Red activa: ");
          Serial.println(wifiSsids[networkIndex]);
          Serial.print("IP: ");
          Serial.println(WiFi.localIP());

          return true;
        }

        delay(100);
      }

      Serial.println("Intento fallido");
      delay(retryDelayMs);
    }

    Serial.print("No fue posible conectar a: ");
    Serial.println(wifiSsids[networkIndex]);
  }

  wifiConectado = false;

  Serial.println("ERROR: ninguna red WiFi conocida esta disponible");
  Serial.print("Estado WiFi: ");
  Serial.println(WiFi.status());

  return false;
}

void connectMQTT()
{
  if (mqttConectado && mqttClient.connected()) {
    return;
  }

  if (!wifiConectado || WiFi.status() != WL_CONNECTED) {
    Serial.println("No se puede conectar a MQTT: WiFi no disponible");
    mqttConectado = false;
    return;
  }

  Serial.print("Conectando a MQTT en ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  mqttClient.begin(MQTT_HOST, MQTT_PORT, net);
  mqttClient.onMessage(mqttMessageReceived);

  const int maxRetries = 5;
  int retryCount = 0;

  while (!mqttClient.connect(
           "mongo_garden_mkr1000",
           MQTT_USERNAME,
           MQTT_PASSWORD
         ) && retryCount < maxRetries) {

    Serial.print("Intento MQTT ");
    Serial.print(retryCount + 1);
    Serial.println(" fallido. Reintentando en 2 segundos...");

    retryCount++;
    delay(2000);
  }

  if (!mqttClient.connected()) {
    Serial.println("No se pudo conectar al broker MQTT");
    mqttConectado = false;
    return;
  }

  Serial.println("MQTT conectado");
  mqttClient.subscribe("mongo_garden/cmd/openRightValve");
  mqttClient.subscribe("mongo_garden/cmd/openLeftValve");

  mqttClient.subscribe("mongo_garden/cmd/setTime");
  mqttClient.subscribe("mongo_garden/cmd/syncRtc");
  mqttClient.subscribe("mongo_garden/cmd/setWiFiParameters");

  mqttClient.subscribe("mongo_garden/cmd/readSoil");

  mqttClient.subscribe("mongo_garden/cmd/read/dht/internal");
  mqttClient.subscribe("mongo_garden/cmd/read/dht/external");
  mqttClient.subscribe("mongo_garden/cmd/read/bmp");
  mqttClient.subscribe("mongo_garden/cmd/read/tsl");
  mqttClient.subscribe("mongo_garden/cmd/read/battery");

  mqttClient.subscribe("mongo_garden/cmd/read/soil/all");
  mqttClient.subscribe("mongo_garden/cmd/read/soil/slot");

  mqttClient.subscribe("mongo_garden/cmd/system");

  mqttConectado = true;

  String Hora = getTime();
  mqttClient.publish("mongo_garden/status/system", "MQTT_CONNECTED " + Hora);
}

String setWiFiParameters(uint8_t networkIndex, String newSsid, String newPassword)
  {
  if (networkIndex < 1 || networkIndex > WIFI_NETWORK_COUNT) {
    return "ERROR: indice de red invalido. Usa 1, 2 o 3";
  }

  newSsid.trim();
  newPassword.trim();

  if (newSsid.length() == 0) {
    return "ERROR: SSID invalido";
  }

  if (newSsid.length() >= sizeof(wifiSsids[0])) {
    return "ERROR: SSID demasiado largo";
  }

  if (newPassword.length() >= sizeof(wifiPasswords[0])) {
    return "ERROR: password demasiado largo";
  }

  uint8_t arrayIndex = networkIndex - 1;

  newSsid.toCharArray(
    wifiSsids[arrayIndex],
    sizeof(wifiSsids[arrayIndex])
  );

  newPassword.toCharArray(
    wifiPasswords[arrayIndex],
    sizeof(wifiPasswords[arrayIndex])
  );

  wifiConectado = false;
  mqttConectado = false;

  if (sdOnline && !saveStartupVars()) {
    return "ERROR: red actualizada en RAM, pero no se pudo guardar en SD";
  }

  return "WiFi " + String(networkIndex) +
         " actualizada. SSID: " + newSsid;
  }

String getBarometricPressure()
{
    if (BMP_ONLINE == false) 
    {
    return "Error en BMP085"; 
    }
  
  barometricPressure = bmp.readPressure();
  barometerAirTemperature = bmp.readTemperature();
    
  return "Presión atmosférica: " + String(barometricPressure) + " Pa. | Temperatura del aire: " + String(barometerAirTemperature) + "° Celcius.";
}

String getInternalTemperature()
{
  Serial.println("Obteniendo datos...");
  delay(2000); // El DHT11 requiere un intervalo de 1-2 segundos entre lecturas

  float humedad = dhtinterno.readHumidity();
  float temperatura = dhtinterno.readTemperature(); // Temperatura en grados Celsius

  // Validar las lecturas
    if (isnan(humedad) || isnan(temperatura)) 
    {
      return "Error al leer del sensor DHT11 interno";
    }

  return "La temperatura interna del módulo de comando es: " + String(temperatura) + "° Celcius, la humedad interna del módulo es: " + String(humedad) + " RH.";
  
}

String getExternalTemperature()
{
  Serial.println("Obteniendo datos...");
  delay(2000); // El DHT11 requiere un intervalo de 1-2 segundos entre lecturas

  float humedad = dhtexterno.readHumidity();
  float temperatura = dhtexterno.readTemperature(); // Temperatura en grados Celsius

  // Validar las lecturas
    if (isnan(humedad) || isnan(temperatura)) 
    {
      return "Error al leer del sensor DHT11 externo";
    }

  return "La temperatura del aire es aproximadamente: " + String(temperatura) + "° Celcius, la humedad exterior aproximada es: " + String(humedad) + " RH.";
  
}

String getTSL2561() 
{
    if (TSL_ONLINE == false)
    {
      return "Error en TSL2561";
    }

  sensors_event_t event;
  tsl.getEvent(&event);

  String clasificacion;
  String Nota = "";
  if (event.light < 100) {
    clasificacion = "NOCHE / Amanecer / Anochecer";
  } else if (event.light < 1000) {
    clasificacion = "AMANECER / ATARDECER";
  } else if (event.light < 10000) {
    clasificacion = "NUBLADO";
  } else if (event.light < 50000) {
    clasificacion = "SOL DIRECTO";
  } else {
    clasificacion = "FUERA DE RANGO";
    if (String(event.light,1) == "65536.0" ){
      Nota = " - Posible falla del sensor.";
    }
  }

  float uvIndex = event.light / 2500.0;

  return "Luz exterior: " + String(event.light, 1) + " lux\n" +
         clasificacion + Nota + "\n" +
         "Índice UV aproximado: " + String(uvIndex, 1) + "\n---";
}

String openRightValve(unsigned long time)
{
  if (time == 0) {
    return "Tiempo invalido";
  }

  analogWrite(RELAYR, 1023);
  delay(time);
  analogWrite(RELAYR, LOW);

  return "Valvula derecha abierta por " + String(time) + " ms";
}

String openLeftValve(unsigned long time)
{
  if (time == 0) {
    return "Tiempo invalido";
  }

  analogWrite(RELAYL, 1023);
  delay(time);
  analogWrite(RELAYL, LOW);

  return "Valvula izquierda abierta por " + String(time) + " ms";
}

void publishDhtInternal()
{
  float h = dhtinterno.readHumidity();
  float t = dhtinterno.readTemperature();      // °C
  float f = dhtinterno.readTemperature(true);  // °F

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Error leyendo DHT interno");
    mqttClient.publish("mongo_garden/telemetry/dht/internal", "Error leyendo DHT interno");
    return;
  }

  String payload = "T=" + String(t, 1) + "C," +
                   "F=" + String(f, 1) + "F," +
                   "H=" + String(h, 1) + "%";

  Serial.print("MQTT publish DHT interno: ");
  Serial.println(payload);

  mqttClient.publish("mongo_garden/telemetry/dht/internal", payload);
}

void publishDhtExternal()
{
  float h = dhtexterno.readHumidity();
  float t = dhtexterno.readTemperature();      // °C
  float f = dhtexterno.readTemperature(true);  // °F

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Error leyendo DHT externo");
    mqttClient.publish("mongo_garden/telemetry/dht/external", "Error en DHT externo");
    return;
  }

  String payload = "T=" + String(t, 1) + "C," +
                   "F=" + String(f, 1) + "F," +
                   "H=" + String(h, 1) + "%";

  Serial.print("MQTT publish DHT externo: ");
  Serial.println(payload);

  mqttClient.publish("mongo_garden/telemetry/dht/external", payload);
}

void publishBmp()
{
  // Si por alguna razón no está inicializado, no hacemos nada
  if (BMP_ONLINE == false) {
    Serial.println("BMP180 no inicializado");
    mqttClient.publish("mongo_garden/telemetry/bmp", "Error en bmp");
    return;
  }

  float temperature = bmp.readTemperature();      // °C
  int32_t pressure  = bmp.readPressure();         // Pa
  float altitude    = bmp.readAltitude();         // m

  String payload = "T=" + String(temperature, 1) + "C," +
                   "P=" + String(pressure) + "Pa," +
                   "Alt=" + String(altitude, 1) + "m";

  Serial.print("MQTT publish BMP180: ");
  Serial.println(payload);

  mqttClient.publish("mongo_garden/telemetry/bmp", payload);
}  

void publishTsl()
{
  if (TSL_ONLINE == false)
    {
      Serial.println("TSL2561 no inicializado");
      mqttClient.publish("mongo_garden/telemetry/tsl", "Error en TSL");
      return;
    }
  
  String Nota = "";
  sensors_event_t event;
  tsl.getEvent(&event);
  
  if (String(event.light,1) == "65536.0" ){
      Nota = " - Posible falla del sensor.";
    }

  String payload = "LUX=" + String(event.light, 1);

  Serial.print("MQTT publish TSL2561: ");
  Serial.println(payload);

  mqttClient.publish("mongo_garden/telemetry/tsl", payload);
}

void publishTimeSource(const String &source)
{
  // Solo publicar si cambió
 // if (source == currentTimeSource) {
 //   return;
 // }

  currentTimeSource = source;

  Serial.print("MQTT publish Time Source: ");
  Serial.println(currentTimeSource);

  mqttClient.publish("mongo_garden/status/timeSource", currentTimeSource);
}

void publishSoilRight(int slot)
{
  if (slot < 0 || slot > 15) {
    Serial.println("Slot de suelo derecho invalido");
    return;
  }

  // Medimos ambos lados a través de la función maestra
  String result = measureSoilSlot(slot);

  Serial.print("measureSoilSlot RIGHT/LEFT, slot ");
  Serial.print(slot);
  Serial.print(" → ");
  Serial.println(result);

  int valueRight = humidityValuesRight[slot];

  String topic   = "mongo_garden/telemetry/soil/rightside/" + String(slot);
  String payload = String(valueRight);

  Serial.print("MQTT publish Soil Right [");
  Serial.print(slot);
  Serial.print("]: ");
  Serial.println(payload);

  mqttClient.publish(topic, payload);
}

void publishSoilLeft(int slot)
{
  if (slot < 0 || slot > 15) {
    Serial.println("Slot de suelo izquierdo invalido");
    return;
  }

  // Medimos ambos lados a través de la función maestra
  String result = measureSoilSlot(slot);

  Serial.print("measureSoilSlot RIGHT/LEFT, slot ");
  Serial.print(slot);
  Serial.print(" → ");
  Serial.println(result);

  int valueLeft = humidityValuesLeft[slot];

  String topic   = "mongo_garden/telemetry/soil/leftside/" + String(slot);
  String payload = String(valueLeft);

  Serial.print("MQTT publish Soil Left [");
  Serial.print(slot);
  Serial.print("]: ");
  Serial.println(payload);

  mqttClient.publish(topic, payload);
}

void publishSoilAll()
{
  for (int slot = 0; slot < 16; slot++) {
    // Medir ambos lados usando la función maestra
    String result = measureSoilSlot(slot);

    Serial.print("[Telemetry] soil slot ");
    Serial.print(slot);
    Serial.print(" → ");
    Serial.println(result);

    // Después de measureSoilSlot, los arrays humidityValuesRight/Left
    // ya deberían estar actualizados por getSoilHumidity()
    String topicRight = "mongo_garden/telemetry/soil/rightside/" + String(slot);
    String topicLeft  = "mongo_garden/telemetry/soil/leftside/" + String(slot);

    String payloadRight = String(humidityValuesRight[slot]);
    String payloadLeft  = String(humidityValuesLeft[slot]);

    Serial.print("MQTT publish Soil Right [");
    Serial.print(slot);
    Serial.print("]: ");
    Serial.println(payloadRight);

    Serial.print("MQTT publish Soil Left [");
    Serial.print(slot);
    Serial.print("]: ");
    Serial.println(payloadLeft);

    mqttClient.publish(topicRight, payloadRight);
    mqttClient.publish(topicLeft,  payloadLeft);
  }
}

String measureSoilSlot(int slot)
{
  if (slot < 0 || slot > 15) {
    return "Slot " + String(slot) + " invalido";
  }

  // Activar relé maestro como en handleSoilCommand
  digitalWrite(RELAYS, HIGH);
  delay(500);

  // Usar la función maestra con los mismos parámetros que el comando bueno
  FunctionMode mode   = SET_AND_READ;
  GardenSide  side    = BOTH_SIDES;
  int         channel = slot;   // asumiendo canal == slot

  String result = getSoilHumidity(mode, side, channel, slot);

  delay(500);
  digitalWrite(RELAYS, LOW);

  return result;
}

bool parseFunctionMode(String modeText, FunctionMode &mode)
{
  modeText.trim();

  if (modeText == "SET_AND_READ") {
    mode = SET_AND_READ;
    return true;
  }
  if (modeText == "SET_ONLY") {
    mode = SET_ONLY;
    return true;
  }
  if (modeText == "READ_ONLY") {
    mode = READ_ONLY;
    return true;
  }

  return false;
}

bool parseGardenSide(String sideText, GardenSide &side)
{
  sideText.trim();

  if (sideText == "RIGHT_SIDE") {
    side = RIGHT_SIDE;
    return true;
  }
  if (sideText == "LEFT_SIDE") {
    side = LEFT_SIDE;
    return true;
  }
  if (sideText == "BOTH_SIDES") {
    side = BOTH_SIDES;
    return true;
  }

  return false;
}

void handlerError(String Message, String Topic, String Mode)
{
  if (Mode.equals("SERIAL_AND_MQTT"))
  {
    Serial.println("Error: " + Message);
    mqttClient.publish(Topic, Message);
    return;
  }
  if (Mode.equals("SERIAL"))
  {
    Serial.println("Error: " + Message);
    return;
  }
  if (Mode.equals("MQTT"))
  {
    mqttClient.publish(Topic, Message);
    return;
  }

}

bool ensureDirectory(const char* path)
{
  if (SD.exists(path)) {
    return true;
  }

  return SD.mkdir(path);
}

bool copyJsonString(const char* source, char* destination, size_t destinationSize)
{
  if (source == nullptr || destination == nullptr || destinationSize == 0) {
    return false;
  }

  if (strlen(source) >= destinationSize) {
    return false;
  }

  strcpy(destination, source);
  return true;
}

unsigned long validateInterval(unsigned long value, unsigned long fallback)
{
  // No se permiten intervalos menores de 1 segundo ni mayores de 24 horas.
  if (value < 1000UL || value > 86400000UL) {
    return fallback;
  }

  return value;
}

void updateTelemetryIntervalsInSeconds()
{
  DHT_INTERVAL_SEC  = max(1L, (long)(DHT_INTERVAL_MS / 1000UL));
  BMP_INTERVAL_SEC  = max(1L, (long)(BMP_INTERVAL_MS / 1000UL));
  TSL_INTERVAL_SEC  = max(1L, (long)(TSL_INTERVAL_MS / 1000UL));
  SOIL_INTERVAL_SEC = max(1L, (long)(SOIL_INTERVAL_MS / 1000UL));
}

bool saveStartupVars()
{
  if (!sdOnline) {
    return false;
  }

  StaticJsonDocument<1536> doc;

  JsonArray networks = doc.createNestedArray("wifiNetworks");
  for (uint8_t i = 0; i < WIFI_NETWORK_COUNT; i++) {
    JsonObject network = networks.createNestedObject();
    network["ssid"] = wifiSsids[i];
    network["password"] = wifiPasswords[i];
  }

  JsonObject mqtt = doc.createNestedObject("mqtt");
  mqtt["host"] = MQTT_HOST;
  mqtt["port"] = MQTT_PORT;
  mqtt["username"] = MQTT_USERNAME;
  mqtt["password"] = MQTT_PASSWORD;

  JsonObject ap = doc.createNestedObject("ap");
  ap["ssid"] = ap_ssid;
  ap["password"] = ap_pass;

  JsonObject telemetry = doc.createNestedObject("telemetryIntervalsMs");
  telemetry["dht"] = DHT_INTERVAL_MS;
  telemetry["bmp"] = BMP_INTERVAL_MS;
  telemetry["tsl"] = TSL_INTERVAL_MS;
  telemetry["soil"] = SOIL_INTERVAL_MS;
  telemetry["offlineSnapshot"] = OFFLINE_SNAPSHOT_INTERVAL_MS;

  if (SD.exists(STARTUP_JSON_PATH)) {
    SD.remove(STARTUP_JSON_PATH);
  }

  File file = SD.open(STARTUP_JSON_PATH, FILE_WRITE);
  if (!file) {
    Serial.println("ERROR: no se pudo crear startUpVars.json");
    return false;
  }

  size_t written = serializeJsonPretty(doc, file);
  file.close();

  if (written == 0) {
    Serial.println("ERROR: no se pudo escribir startUpVars.json");
    return false;
  }

  Serial.println("startUpVars.json guardado");
  return true;
}

bool loadStartupVars()
{
  if (!sdOnline || !SD.exists(STARTUP_JSON_PATH)) {
    return false;
  }

  File file = SD.open(STARTUP_JSON_PATH, FILE_READ);
  if (!file) {
    Serial.println("ERROR: no se pudo abrir startUpVars.json");
    return false;
  }

  StaticJsonDocument<1536> doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();

  if (error) {
    Serial.print("ERROR JSON: ");
    Serial.println(error.c_str());
    return false;
  }

  JsonArray networks = doc["wifiNetworks"].as<JsonArray>();

  for (uint8_t i = 0; i < WIFI_NETWORK_COUNT; i++) {
    if (i >= networks.size()) break;

    const char* loadedSsid = networks[i]["ssid"];
    const char* loadedPassword = networks[i]["password"];

    if (loadedSsid && strlen(loadedSsid) > 0) {
      copyJsonString(loadedSsid, wifiSsids[i], sizeof(wifiSsids[i]));
    }
    if (loadedPassword) {
      copyJsonString(loadedPassword, wifiPasswords[i], sizeof(wifiPasswords[i]));
    }
  }

  JsonObject mqtt = doc["mqtt"];
  if (!mqtt.isNull()) {
    const char* host = mqtt["host"];
    const char* user = mqtt["username"];
    const char* password = mqtt["password"];

    if (host)     copyJsonString(host, MQTT_HOST, sizeof(MQTT_HOST));
    if (user)     copyJsonString(user, MQTT_USERNAME, sizeof(MQTT_USERNAME));
    if (password) copyJsonString(password, MQTT_PASSWORD, sizeof(MQTT_PASSWORD));

    if (mqtt["port"].is<int>()) {
      int port = mqtt["port"];
      if (port > 0 && port <= 65535) {
        MQTT_PORT = port;
      }
    }
  }

  JsonObject ap = doc["ap"];
  if (!ap.isNull()) {
    const char* accessPointSsid = ap["ssid"];
    const char* accessPointPassword = ap["password"];

    if (accessPointSsid) {
      copyJsonString(accessPointSsid, ap_ssid, sizeof(ap_ssid));
    }
    if (accessPointPassword) {
      copyJsonString(accessPointPassword, ap_pass, sizeof(ap_pass));
    }
  }

  JsonObject telemetry = doc["telemetryIntervalsMs"];
  if (!telemetry.isNull()) {
    if (!telemetry["dht"].isNull()) {
      DHT_INTERVAL_MS = validateInterval(telemetry["dht"].as<unsigned long>(), DHT_INTERVAL_MS);
    }
    if (!telemetry["bmp"].isNull()) {
      BMP_INTERVAL_MS = validateInterval(telemetry["bmp"].as<unsigned long>(), BMP_INTERVAL_MS);
    }
    if (!telemetry["tsl"].isNull()) {
      TSL_INTERVAL_MS = validateInterval(telemetry["tsl"].as<unsigned long>(), TSL_INTERVAL_MS);
    }
    if (!telemetry["soil"].isNull()) {
      SOIL_INTERVAL_MS = validateInterval(telemetry["soil"].as<unsigned long>(), SOIL_INTERVAL_MS);
    }
    if (!telemetry["offlineSnapshot"].isNull()) {
      OFFLINE_SNAPSHOT_INTERVAL_MS =
        validateInterval(telemetry["offlineSnapshot"].as<unsigned long>(), OFFLINE_SNAPSHOT_INTERVAL_MS);
    }
  }

  updateTelemetryIntervalsInSeconds();

  Serial.println("Configuracion cargada desde startUpVars.json");
  return true;
}

bool ensureSensorCsv()
{
  if (!sdOnline) {
    return false;
  }

  if (SD.exists(SENSOR_CSV_PATH)) {
    return true;
  }

  File file = SD.open(SENSOR_CSV_PATH, FILE_WRITE);

  if (!file) {
    Serial.println("ERROR: no se pudo crear sensor_log.csv");
    return false;
  }

  file.print("timestamp,timeSource,wifiConnected,mqttConnected,");
  file.print("internalTempC,internalHumidity,");
  file.print("externalTempC,externalHumidity,");
  file.print("bmpTempC,pressurePa,altitudeM,lux,uvIndex");

  for (uint8_t slot = 0; slot < 16; slot++) {
    file.print(",soilRight");
    file.print(slot);
    file.print(",soilLeft");
    file.print(slot);
  }

  file.println();
  file.close();

  Serial.println("sensor_log.csv creado");
  return true;
}

bool initStorageAndConfiguration()
{
  sdOnline = SD.begin(CS_PIN);

  if (!sdOnline) {
    Serial.println("ERROR: modulo SD no disponible");
    return false;
  }

  Serial.println("Modulo SD inicializado");

  if (!ensureDirectory(STARTUP_DIR)) {
    Serial.println("ERROR: no se pudo crear /startup");
    return false;
  }

  if (!ensureDirectory(DATA_DIR)) {
    Serial.println("ERROR: no se pudo crear /data");
    return false;
  }

  // Si el archivo no existe, se crea con los valores definidos en el sketch.
  if (!SD.exists(STARTUP_JSON_PATH)) {
    Serial.println("No existe startUpVars.json. Creando configuracion inicial...");

    if (!saveStartupVars()) {
      return false;
    }
  }

  // Si existe, tiene prioridad la configuración guardada en SD.
  if (!loadStartupVars()) {
    Serial.println("Usando valores de respaldo definidos en el sketch");
  }

  if (!ensureSensorCsv()) {
    return false;
  }

  return true;
}

void printCsvFloat(File& file, float value, uint8_t decimals)
{
  if (isnan(value)) {
    return;
  }

  file.print(value, decimals);
}

void getCsvTimestamp(char* buffer, size_t bufferSize)
{
  if (Rtcmod.IsDateTimeValid()) {
    RtcDateTime now = Rtcmod.GetDateTime();

    snprintf(
      buffer,
      bufferSize,
      "%04u-%02u-%02u %02u:%02u:%02u",
      now.Year(),
      now.Month(),
      now.Day(),
      now.Hour(),
      now.Minute(),
      now.Second()
    );

    return;
  }

  snprintf(buffer, bufferSize, "millis_%lu", millis());
}

bool logOfflineSnapshot()
{
  if (!sdOnline) {
    return false;
  }

  File file = SD.open(SENSOR_CSV_PATH, FILE_WRITE);

  if (!file) {
    Serial.println("ERROR: no se pudo abrir sensor_log.csv");
    return false;
  }

  // Lecturas DHT.
  float internalHumidity = dhtinterno.readHumidity();
  float internalTemp = dhtinterno.readTemperature();

  float externalHumidity = dhtexterno.readHumidity();
  float externalTemp = dhtexterno.readTemperature();

  // Lecturas BMP180.
  float bmpTemp = NAN;
  float altitude = NAN;
  int32_t pressure = -1;

  if (BMP_ONLINE) {
    bmpTemp = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitude = bmp.readAltitude();
  }

  // Lecturas TSL2561.
  float lux = NAN;
  float uvIndex = NAN;

  if (TSL_ONLINE) {
    sensors_event_t event;
    tsl.getEvent(&event);

    lux = event.light;
    uvIndex = event.light / 2500.0f;
  }

  // Se actualizan los 16 sensores por ambos lados.
  for (uint8_t slot = 0; slot < 16; slot++) {
    measureSoilSlot(slot);
  }

  char timestamp[24];
  getCsvTimestamp(timestamp, sizeof(timestamp));

  file.print(timestamp);
  file.print(",");
  file.print(Rtcmod.IsDateTimeValid() ? "RTC" : "MILLIS");
  file.print(",");
  file.print((WiFi.status() == WL_CONNECTED) ? "1" : "0");
  file.print(",");
  file.print((mqttConectado && mqttClient.connected()) ? "1" : "0");
  file.print(",");

  printCsvFloat(file, internalTemp, 1);
  file.print(",");
  printCsvFloat(file, internalHumidity, 1);
  file.print(",");
  printCsvFloat(file, externalTemp, 1);
  file.print(",");
  printCsvFloat(file, externalHumidity, 1);
  file.print(",");
  printCsvFloat(file, bmpTemp, 1);
  file.print(",");

  if (pressure >= 0) {
    file.print(pressure);
  }

  file.print(",");
  printCsvFloat(file, altitude, 1);
  file.print(",");
  printCsvFloat(file, lux, 1);
  file.print(",");
  printCsvFloat(file, uvIndex, 2);

  for (uint8_t slot = 0; slot < 16; slot++) {
    file.print(",");
    file.print(humidityValuesRight[slot]);
    file.print(",");
    file.print(humidityValuesLeft[slot]);
  }

  file.println();
  file.close();

  Serial.println("Snapshot offline almacenado en sensor_log.csv");
  return true;
}

void handleOfflineLogging()
{
  bool mqttAvailable =
    wifiConectado &&
    WiFi.status() == WL_CONNECTED &&
    mqttConectado &&
    mqttClient.connected();

  // Si MQTT funciona, no se duplica la telemetría en SD.
  if (mqttAvailable) {
    return;
  }

  unsigned long now = millis();

  if (lastOfflineSnapshotMillis == 0 ||
      now - lastOfflineSnapshotMillis >= OFFLINE_SNAPSHOT_INTERVAL_MS) {

    if (logOfflineSnapshot()) {
      lastOfflineSnapshotMillis = now;
    }
  }
}