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
  #define USE_WIFI101_PRO           true
  #include <Adafruit_BMP085.h>
  #include <WebSocketsServer_Generic.h>
  #include <Adafruit_TSL2561_U.h>
  #define DEMOPIN A6
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
    const char* MQTT_HOST     = "157.137.230.39";
    const int   MQTT_PORT     = 1883;
    const char* MQTT_USERNAME = "mkr1000-Main";
    const char* MQTT_PASSWORD = "BDAFE5BE";

    bool mqttConectado = false;
    
    unsigned long lastDhtPublishMillis  = 0;
    unsigned long lastBmpPublishMillis  = 0;
    unsigned long lastTslPublishMillis  = 0;
    unsigned long lastSoilPublishMillis = 0;

    const unsigned long DHT_INTERVAL_MS  = 60UL * 1000UL;  // 30 s
    const unsigned long BMP_INTERVAL_MS  = 1800UL * 1000UL;  // 60 s
    const unsigned long TSL_INTERVAL_MS  = 1800UL * 1000UL;  // 60 s
    const unsigned long SOIL_INTERVAL_MS = 3600UL * 1000UL;  // 60 s;

    // Usaremos segundos del RTC como "tiempo base" cuando sea posible
    long lastDhtPublishRtcSeconds  = -1;
    long lastBmpPublishRtcSeconds  = -1;
    long lastTslPublishRtcSeconds  = -1;
    long lastSoilPublishRtcSeconds = -1;

    const long DHT_INTERVAL_SEC  = 60;
    const long BMP_INTERVAL_SEC  = 1800;
    const long TSL_INTERVAL_SEC  = 1800;
    const long SOIL_INTERVAL_SEC = 3600;
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
  char ssid[32] = "SANTIAGO";        // your network SSID (name)
  char pass[64] = "43102996";    // your network password (use for WPA, or use as key for WEP)
  /*
  char ssid[32] = "Semillero ASI";        // your network SSID (name)
  char pass[64] = "semilleroasik601";   
  */
  char ap_ssid[] = "MONGO_GARDEN";
  char ap_pass[] = "mongo_pwr";
  

  int demoMode;
//end

void setup() 
{
  pinMode(DEMOPIN, INPUT);
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
  if (!(SD.begin(CS_PIN))) {Serial.println("Error inicializando modulo SD");} else { Serial.println("Modulo SD inicializado");}
  demoMode = digitalRead(DEMOPIN);
  if (demoMode == 1) {Serial.println("DEMO MODE ACTIVE");}
  Serial.println("Version - 202608010241");
  Serial.println("------FIN INICIO------");
  digitalWrite(RELAYS, LOW);
  digitalWrite(RELAYR, LOW);
  digitalWrite(RELAYL, LOW);

  if (!connectWiFi()) {
    Serial.println("Advertencia: WiFi no disponible. Algunas funciones no funcionarán.");
  }

  connectMQTT();

}

void loop()
{
  // Si tienes MQTT y ya hay WiFi:
  if (wifiConectado && WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      mqttConectado = false;
      connectMQTT();
    }

    mqttClient.loop();
    handleTelemetryLoop();
  }

  // Siempre escuchas Serial para comandos, aunque no haya WiFi
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
    Serial.println("  SYNC_RTC");
    Serial.println("  GET_TIME");
    Serial.println("  GET_BMP_DATA");
    Serial.println("  GET_INTERNAL_DHT");
    Serial.println("  GET_EXTERNAL_DHT");
    Serial.println("  GET_TSL_DATA");
    Serial.println("  SET_TIME,2026,07,26,21,45,00");
    Serial.println("  SET_WiFi_PARAMETERS_\"SSID\"_\"PASSWORD\"");
    Serial.println("  OPEN_RIGHT_VALVE,2000");
    Serial.println("  OPEN_LEFT_VALVE,2000");
    Serial.println("  SET_AND_READ, BOTH_SIDES, 5, 5");
    Serial.println("|---|");
}

void handleTelemetryLoop()
{
  demoMode = digitalRead(DEMOPIN);
  if (demoMode == 0) {return;}
  // Tiempo relativo (backup)
  unsigned long nowMillis = millis();

  // Tiempo absoluto del RTC (si está disponible)
  long nowRtcSeconds = getRtcSeconds();
  bool rtcOk = (nowRtcSeconds >= 0);

  // Publicar qué fuente de tiempo estamos usando
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
    if ((nowMillis - lastDhtPublishMillis) >= DHT_INTERVAL_MS) {
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
    if ((nowMillis - lastBmpPublishMillis) >= BMP_INTERVAL_MS) {
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
    if ((nowMillis - lastTslPublishMillis) >= TSL_INTERVAL_MS) {
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
    if ((nowMillis - lastSoilPublishMillis) >= SOIL_INTERVAL_MS) {
      publishSoilAll();
      lastSoilPublishMillis = nowMillis;
    }
  }
}

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
        "ERROR: tiempo invalido para openRightValve"
      );
      return;
    }

    long timeValue = payload.toInt();

    if (timeValue <= 0) {
      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: tiempo debe ser mayor que 0"
      );
      return;
    }

    String result = openRightValve((unsigned long)timeValue);

    Serial.println(result);
    mqttClient.publish("mongo_garden/status/system", result);

    return;
  }

  if (topic == "mongo_garden/cmd/openLeftValve") {
    if (!isValidInteger(payload)) {
      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: tiempo invalido para openLeftValve"
      );
      return;
    }

    long timeValue = payload.toInt();

    if (timeValue <= 0) {
      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: tiempo debe ser mayor que 0"
      );
      return;
    }

    String result = openLeftValve((unsigned long)timeValue);

    Serial.println(result);
    mqttClient.publish("mongo_garden/status/system", result);

    return;
  }

  // ===================================
  // COMANDOS DE LECTURA INDIVIDUAL
  // ===================================

  // Leer y publicar DHT interno
  if (topic == "mongo_garden/cmd/read/dht/internal") {
    publishDhtInternal();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_DHT_INTERNAL_COMPLETED"
    );

    return;
  }

  // Leer y publicar DHT externo
  if (topic == "mongo_garden/cmd/read/dht/external") {
    publishDhtExternal();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_DHT_EXTERNAL_COMPLETED"
    );

    return;
  }

  // Leer y publicar BMP180
  if (topic == "mongo_garden/cmd/read/bmp") {
    publishBmp();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_BMP_COMPLETED"
    );

    return;
  }

  // Leer y publicar TSL2561
  if (topic == "mongo_garden/cmd/read/tsl") {
    publishTsl();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_TSL_COMPLETED"
    );

    return;
  }

  // Leer y publicar todos los slots de suelo
  if (topic == "mongo_garden/cmd/read/soil/all") {
    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_SOIL_ALL_STARTED"
    );

    // publishSoilAll() debe usar measureSoilSlot()
    // para activar RELAYS, seleccionar canal y leer ambos lados.
    publishSoilAll();

    mqttClient.publish(
      "mongo_garden/status/system",
      "READ_SOIL_ALL_COMPLETED"
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
        "ERROR: payload soil slot debe ser canal,slot"
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
        "ERROR: canal o slot no numerico"
      );

      return;
    }

    int channel = channelText.toInt();
    int slot = slotText.toInt();

    if (channel < 0 || channel > 15 ||
        slot < 0 || slot > 15) {

      mqttClient.publish(
        "mongo_garden/status/system",
        "ERROR: canal y slot deben estar entre 0 y 15"
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
      "READ_SOIL_SLOT_COMPLETED"
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

    // Se publica antes de perder la conexión MQTT
    mqttClient.publish("mongo_garden/status/wifi", result);

    // Si las credenciales son inválidas, connectWiFi()
    // falla después de sus intentos limitados y el programa sigue vivo.
    if (connectWiFi()) {
      mqttConectado = false;
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
    mqttClient.publish("mongo_garden/status/system", result);

    return;
  }

  // ===================================
  // TOPIC DESCONOCIDO
  // ===================================

  Serial.println("MQTT comando no reconocido");

  mqttClient.publish(
    "mongo_garden/status/system",
    "ERROR: MQTT_TOPIC_NOT_RECOGNIZED"
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
    const String prefix = "SET_WiFi_PARAMETERS_";

    // Si no empieza por el prefijo, no es este comando
    if (!input.startsWith(prefix)) {
      return false;
    }

    // Buscar la primera comilla del SSID (después del prefijo)
    int firstQuote = input.indexOf('"', prefix.length());
    if (firstQuote == -1) {
      Serial.println("Formato invalido. Falta la primera comilla del SSID");
      return true;
    }

    // Buscar la segunda comilla del SSID
    int secondQuote = input.indexOf('"', firstQuote + 1);
    if (secondQuote == -1) {
      Serial.println("Formato invalido. Falta la segunda comilla del SSID");
      return true;
    }

    // Buscar la primera comilla del password (debe aparecer después de secondQuote)
    int thirdQuote = input.indexOf('"', secondQuote + 1);
    if (thirdQuote == -1) {
      Serial.println("Formato invalido. Falta la primera comilla del password");
      return true;
    }

    // Buscar la segunda comilla del password
    int fourthQuote = input.indexOf('"', thirdQuote + 1);
    if (fourthQuote == -1) {
      Serial.println("Formato invalido. Falta la segunda comilla del password");
      return true;
    }

    // Validar que no haya texto extra después del último argumento
    String extraText = input.substring(fourthQuote + 1);
    extraText.trim();
    if (extraText.length() > 0) {
      Serial.println("Formato invalido. Hay caracteres extra al final");
      return true;
    }

    // Extraer SSID y password entre comillas
    String newSsid = input.substring(firstQuote + 1, secondQuote);
    String newPass = input.substring(thirdQuote + 1, fourthQuote);

    // Actualizar parámetros WiFi (setWiFiParameters hace la validación de longitudes)
    String result = setWiFiParameters(newSsid, newPass);
    Serial.println(result);

    // Intentar conexión con las nuevas credenciales, pero UNA VEZ
    if (!connectWiFi()) {
      Serial.println("WiFi sigue sin conectar con las nuevas credenciales");
    } else {
      Serial.println("WiFi conectado con nuevas credenciales");
    }

    return true;
}

String handleSetWiFiPayload(String payload)
{
  int sep = payload.indexOf(';');
  if (sep == -1) {
    return "Formato invalido. Usa: SSID;PASSWORD";
  }

  String newSsid = payload.substring(0, sep);
  String newPass = payload.substring(sep + 1);

  newSsid.trim();
  newPass.trim();

  return setWiFiParameters(newSsid, newPass);
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
  // Si ya está conectado, no hacer nada
  if (WiFi.status() == WL_CONNECTED) {
    wifiConectado = true;
    return true;
  }

  Serial.print("Conectando a WiFi");

  const int maxRetries   = 5;
  const int retryDelayMs = 3000;
  int retryCount         = 0;

  while (retryCount < maxRetries) {
    WiFi.begin(ssid, pass);

    unsigned long startAttempt = millis();
    while (millis() - startAttempt < 5000) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("WiFi conectado");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        wifiConectado = true;
        return true;
      }
      delay(100);
    }

    Serial.print(".");
    retryCount++;
    delay(retryDelayMs);
  }

  Serial.println();
  Serial.println("No se pudo conectar a la red WiFi.");
  Serial.print("Estado WiFi: ");
  Serial.println(WiFi.status());

  wifiConectado = false;
  return false;
}

void connectMQTT()
{
  // Si MQTT ya está conectado, no hacer nada
  if (mqttConectado && mqttClient.connected()) {
    return;
  }

  // No intentar MQTT si WiFi no está disponible
  if (!wifiConectado || WiFi.status() != WL_CONNECTED) {
    Serial.println("No se puede conectar a MQTT: WiFi no disponible");
    mqttConectado = false;
    return;
  }

  Serial.print("Conectando a MQTT en ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  // Configurar cliente y callback
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

  // Salir si no logró conectar
  if (!mqttClient.connected()) {
    Serial.println("No se pudo conectar al broker MQTT");
    mqttConectado = false;
    return;
  }

  Serial.println("MQTT conectado");

  // -----------------------------------
  // COMANDOS DE VÁLVULAS
  // -----------------------------------
  mqttClient.subscribe("mongo_garden/cmd/openRightValve");
  mqttClient.subscribe("mongo_garden/cmd/openLeftValve");

  // -----------------------------------
  // COMANDOS DE RTC Y WIFI
  // -----------------------------------
  mqttClient.subscribe("mongo_garden/cmd/setTime");
  mqttClient.subscribe("mongo_garden/cmd/syncRtc");
  mqttClient.subscribe("mongo_garden/cmd/setWiFiParameters");

  // -----------------------------------
  // COMANDOS DE SUELO YA EXISTENTES
  // -----------------------------------
  mqttClient.subscribe("mongo_garden/cmd/readSoil");

  // -----------------------------------
  // NUEVOS COMANDOS DE LECTURA INDIVIDUAL
  // -----------------------------------
  mqttClient.subscribe("mongo_garden/cmd/read/dht/internal");
  mqttClient.subscribe("mongo_garden/cmd/read/dht/external");
  mqttClient.subscribe("mongo_garden/cmd/read/bmp");
  mqttClient.subscribe("mongo_garden/cmd/read/tsl");

  // Publica los 16 slots de ambos lados
  mqttClient.subscribe("mongo_garden/cmd/read/soil/all");

  // Payload esperado: canal,slot
  // Ejemplo: 5,5
  mqttClient.subscribe("mongo_garden/cmd/read/soil/slot");

  // Comandos genéricos del sistema
  mqttClient.subscribe("mongo_garden/cmd/system");

  mqttConectado = true;

  // Avisar al dashboard que el MKR1000 está conectado
  mqttClient.publish(
    "mongo_garden/status/system",
    "MQTT_CONNECTED"
  );
}

String setWiFiParameters(String newSsid, String newPass)
{
    newSsid.trim();
    newPass.trim();

    if (newSsid.length() == 0) {
      return "SSID invalido";
    }

    if (newSsid.length() >= sizeof(ssid)) {
      return "SSID demasiado largo";
    }

    if (newPass.length() >= sizeof(pass)) {
      return "Password demasiado largo";
    }

    newSsid.toCharArray(ssid, sizeof(ssid));
    newPass.toCharArray(pass, sizeof(pass));

    wifiConectado = false;

    return "WiFi parameters actualizados | SSID: " + newSsid + " | PASSWORD: " + newPass;
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
    Serial.println("BMP180 no inicializado, no se publica");
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
  if (source == currentTimeSource) {
    return;
  }

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


