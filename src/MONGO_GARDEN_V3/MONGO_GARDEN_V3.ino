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


  WiFiServer server(80);          // Servidor HTTP normal para cargar la web (Puerto 80)
  WebSocketsServer webSocket = WebSocketsServer(81); // Servidor WebSocket (Puerto 81)


  Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
    bool TSL_ONLINE = false;
    float uvIndex;
  
  File archivo;
  

  
  int humidityValuesRight[15];
  int humidityValuesLeft[15];
  
  
  int pingResult;
  int status = 0;
  String hostName = "www.google.com";
  bool wifiConectado = false;
  bool serverRunning = false;
  char ssid[32] = "Semillero ASI";        // your network SSID (name)
  char pass[64] = "semilleroasik601";    // your network password (use for WPA, or use as key for WEP)
  char ap_ssid[] = "MONGO_GARDEN";
  char ap_pass[] = "mongo_pwr";
  


  int demoMode;
  //ab


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

  Serial.println("------FIN INICIO------");
  digitalWrite(RELAYS, LOW);


  digitalWrite(RELAYS, HIGH);

  }

void loop() 
  {
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
    Serial.println("  SET_AND_READ, BOTH_SIDES, 5, 3");
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

    Serial.println(getSoilHumidity(mode, side, channelMultiplexor, soilSlot));
    return true;
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

    connectWiFi();

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

void connectWiFi()
  {
    if (WiFi.status() == WL_CONNECTED) return;

    Serial.print("Conectando a WiFi");
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
      Serial.print(".");
      delay(3000);
    }

    Serial.println();
    Serial.println("WiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
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