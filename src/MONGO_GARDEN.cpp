// initials 
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
  #include <Adafruit_BMP085.h>
  #include <WebSocketsServer_Generic.h>
  #include <Adafruit_TSL2561_U.h>
  #define DEMOPIN A6
  #define SIG1 A0
  #define SIG2 A1
  #define RELAYR A2
  #define RELAYL A3
  #define RELAYS A4
  Adafruit_BMP085 bmp;
  ThreeWire wireClock(13, 14, A5);
  RtcDS1302<ThreeWire> Rtcmod(wireClock);
  DHT dhtexterno(5, DHT11);
  DHT dhtinterno(6, DHT11);
  WiFiServer server(80);          // Servidor HTTP normal para cargar la web (Puerto 80)
  WebSocketsServer webSocket = WebSocketsServer(81); // Servidor WebSocket (Puerto 81)
  Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
  File archivo;
  String FYH;
  String hostName = "www.google.com";
  int LA1, LA2, LA3, LA4, LA5, LB1, LB2, LB3, LB4, LB5, LC1, LC2, LC3, LC4, LC5, RA1, RA2, RA3, RA4, RA5, RB1, RB2, RB3, RB4, RB5, RC1, RC2, RC3, RC4, RC5;
  int H, TC, TF, HIC, HIF;
  int pingResult;
  int status = 0;
  const int CS_PIN = 7;
  int demomode;
  bool wifiConectado = false;
  bool serverRunning = false;
  char ssid[] = "Semillero ASI";        // your network SSID (name)
  char pass[] = "semilleroasik601";    // your network password (use for WPA, or use as key for WEP)
  char ap_ssid[] = "MONGO_GARDEN";
  char ap_pass[] = "mongo_pwr";
  //ab
  
void setup() 
  {
  pinMode(DEMOPIN, INPUT);
  pinMode(RELAYR, OUTPUT);
  pinMode(RELAYL, OUTPUT);
  pinMode(SIG1, INPUT);
  pinMode(SIG2, INPUT);
  pinMode(RELAYS, OUTPUT);
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
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
  Serial.println("Modulo de reloj inicializado");
  if (!bmp.begin()) {Serial.println("Error inicializando BMP180");} else {Serial.println("BMP180 inicializado");}
  if (!tsl.begin()) {Serial.println("Error inicializando TSL2561");} else {Serial.println("TSL2561 inicializado");tsl.setGain(TSL2561_GAIN_1X);tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);}
  if (!(SD.begin(CS_PIN))) {Serial.println("Error inicializando modulo SD");} else { Serial.println("Modulo SD inicializado");}
  demomode = digitalRead(DEMOPIN);
  if (demomode == 1) {Serial.println("DEMO MODE ACTIVE");setRTCtoCompileTime();}




  Serial.println("------FIN INICIO------");
  digitalWrite(RELAYS, LOW);


  }

void loop() 
  {

    comserial();    
  }
void comserial() 
  {
  if (Serial.available() > 0) { // Si hay datos disponibles para leer
    String comando = Serial.readStringUntil('\n'); // Leer hasta salto de línea
    comando.trim(); // Eliminar espacios en blanco al inicio y al final
    Serial.println("comando ingresado: " + comando);
    // Comparar comando y llamar a función correspondiente
    if (comando.equals("help")) {
      Serial.println("Funciones disponibles: ");
    }
    else if (comando.equals("server")) {
      handleWebServer();
    }
    else if (comando.equals("WifiLab")) {
      WifiLab();
    }
    else if (comando.equals("AP")) {
      startWebServerAP();
    }
    else if (comando.equals("1")) {
      leersensores(1000);
    } 
    else if (comando.equals("2")) {
      obtenerhora();
    }
    else if (comando.equals("3")) {
      imprimirhora();
    }
    else if (comando.equals("4")) {
      obtenereimprimirhora();
    }
    else if (comando.equals("5")) {
      humedaddht11interno();
    }
    else if (comando.equals("6")) {
      humedaddht11externo();
    }
    else if (comando.equals("7")) {
      printWiFiData();
    }
    else if (comando.equals("8")) {
      pingToHost();
    }
    else if (comando.equals("9")) {
      escribirDatos();
    }
    else if (comando.equals("10")) {
      barometro();
    }
    else if (comando.equals("11")) {
      TSL2561();
    }
    else {
      Serial.println("Comando no reconocido, escriba help para ver la tabla de funciones");
    }
  }
  }
void obtenerhora()
  {
  RtcDateTime ahora = Rtcmod.GetDateTime();
  FYH = (String(ahora.Day())+"/"+String(ahora.Month())+"/"+String(ahora.Year())+"||"+String(ahora.Hour())+":"+String(ahora.Minute())+":"+String(ahora.Second())+",");
  }
void imprimirhora()
  {
  Serial.println(FYH);
  }
void obtenereimprimirhora()
  {
    obtenerhora();
    imprimirhora();
  }
void leersensores(int time_between_readings)
  {
  digitalWrite(RELAYS, HIGH); //RELE SENSORE ON
  digitalWrite(4, LOW); // Habilitar multiplexor

  // Canal 0: 0000
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  Serial.println("canal 0");
  delay(time_between_readings);

  // Canal 1: 1000
  digitalWrite(0, HIGH);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  Serial.println("canal 1");
  delay(100);
  RA1 = analogRead(SIG1);
  LA1 = analogRead(SIG2);
  Serial.println(String("RA1: ") + String(RA1) + " " + String("LA1: ") + String(LA1));
  delay(time_between_readings);

  // Canal 2: 0100
  digitalWrite(0, LOW);
  digitalWrite(1, HIGH);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  Serial.println("canal 2");
  delay(100);
  RA2 = analogRead(SIG1);
  LA2 = analogRead(SIG2);
  Serial.println(String("RA2: ") + String(RA2) + " " + String("LA2: ") + String(LA2));
  delay(time_between_readings);

  // Canal 3: 1100
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  Serial.println("canal 3");
  delay(100);
  RA3 = analogRead(SIG1);
  LA3 = analogRead(SIG2);
  Serial.println(String("RA3: ") + String(RA3) + " " + String("LA3: ") + String(LA3));
  delay(time_between_readings);

  // Canal 4: 0010
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
  Serial.println("canal 4");
  delay(100);
  RA4 = analogRead(SIG1);
  LA4 = analogRead(SIG2);
  Serial.println(String("RA4: ") + String(RA4) + " " + String("LA4: ") + String(LA4));
  delay(time_between_readings);

  // Canal 5: 1010
  digitalWrite(0, HIGH);
  digitalWrite(1, LOW);
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
  Serial.println("canal 5");
  delay(100);
  RA5 = analogRead(SIG1);
  LA5 = analogRead(SIG2);
  Serial.println(String("RA5: ") + String(RA5) + " " + String("LA5: ") + String(LA5));
  delay(time_between_readings);

  // Canal 6: 0110
  digitalWrite(0, LOW);
  digitalWrite(1, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
  Serial.println("canal 6");
  delay(100);
  RB1 = analogRead(SIG1);
  LB1 = analogRead(SIG2);
  Serial.println(String(RB1) + " " + String(LB1));
  delay(time_between_readings);

  // Canal 7: 1110
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
  Serial.println("canal 7");
  delay(100);
  RB2 = analogRead(SIG1);
  LB2 = analogRead(SIG2);
  Serial.println(String(RB2) + " " + String(LB2));
  delay(time_between_readings);

  // Canal 8: 0001
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  Serial.println("canal 8");
  delay(100);
  RB3 = analogRead(SIG1);
  LB3 = analogRead(SIG2);
  Serial.println(String(RB3) + " " + String(LB3));
  delay(time_between_readings);

  // Canal 9: 1001
  digitalWrite(0, HIGH);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  Serial.println("canal 9");
  delay(100);
  RB4 = analogRead(SIG1);
  LB4 = analogRead(SIG2);
  Serial.println(String(RB4) + " " + String(LB4));
  delay(time_between_readings);

  // Canal 10: 0101
  digitalWrite(0, LOW);
  digitalWrite(1, HIGH);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  Serial.println("canal 10");
  delay(100);
  RB5 = analogRead(SIG1);
  LB5 = analogRead(SIG2);
  Serial.println(String(RB5) + " " + String(LB5));
  delay(time_between_readings);

  // Canal 11: 1101
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  Serial.println("canal 11");
  delay(100);
  RC1 = analogRead(SIG1);
  LC1 = analogRead(SIG2);
  Serial.println(String(RC1) + " " + String(LC1));
  delay(time_between_readings);

  // Canal 12: 0011
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  Serial.println("canal 12");
  delay(100);
  RC2 = analogRead(SIG1);
  LC2 = analogRead(SIG2);
  Serial.println(String(RC2) + " " + String(LC2));
  delay(time_between_readings);

  // Canal 13: 1011
  digitalWrite(0, HIGH);
  digitalWrite(1, LOW);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  Serial.println("canal 13");
  delay(100);
  RC3 = analogRead(SIG1);
  LC3 = analogRead(SIG2);
  Serial.println(String(RC3) + " " + String(LC3));
  delay(time_between_readings);

  // Canal 14: 0111
  digitalWrite(0, LOW);
  digitalWrite(1, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  Serial.println("canal 14");delay(100);
  RC4 = analogRead(SIG1);
  LC4 = analogRead(SIG2);
  Serial.println(String(RC4) + " " + String(LC4));
  delay(time_between_readings);

  // Canal 15: 1111
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  Serial.println("canal 15");
  delay(100);
  RC5 = analogRead(SIG1);
  LC5 = analogRead(SIG2);
  Serial.println(String(RC5) + " " + String(LC5));
  delay(time_between_readings);

  digitalWrite(4, HIGH); // Deshabilitar multiplexor
  digitalWrite(RELAYS, LOW); //RELE SENSORES OFF
  delay(100);
  Serial.println("ciclo de lectura de sensores finalizado");

  }
void setRTCtoCompileTime() 
  {
  
  char monthStr[5];
    int day, year;
    int hour, min, sec;

    sscanf(__DATE__, "%s %d %d", monthStr, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &hour, &min, &sec);

    // Traduce mes en texto a número
    int month = 0;
    if (strcmp(monthStr, "Jan") == 0) month = 1;
    else if (strcmp(monthStr, "Feb") == 0) month = 2;
    else if (strcmp(monthStr, "Mar") == 0) month = 3;
    else if (strcmp(monthStr, "Apr") == 0) month = 4;
    else if (strcmp(monthStr, "May") == 0) month = 5;
    else if (strcmp(monthStr, "Jun") == 0) month = 6;
    else if (strcmp(monthStr, "Jul") == 0) month = 7;
    else if (strcmp(monthStr, "Aug") == 0) month = 8;
    else if (strcmp(monthStr, "Sep") == 0) month = 9;
    else if (strcmp(monthStr, "Oct") == 0) month = 10;
    else if (strcmp(monthStr, "Nov") == 0) month = 11;
    else if (strcmp(monthStr, "Dec") == 0) month = 12;
    RtcDateTime compiled(year, month, day, hour, min, sec);

    Rtcmod.SetDateTime(compiled);
    Serial.println("hora establecida");
  }
void humedaddht11interno()
  {
  delay(2000); // El DHT11 requiere un intervalo de 1-2 segundos entre lecturas

  float humedad = dhtinterno.readHumidity();
  float temperatura = dhtinterno.readTemperature(); // Temperatura en grados Celsius

  // Validar las lecturas
  if (isnan(humedad) || isnan(temperatura)) {
    Serial.println("Error al leer del sensor DHT11 interno");
    return;
  }

  Serial.print("Temperatura interna: ");
  Serial.print(temperatura);
  Serial.print("°C  |  Humedad interna: ");
  Serial.print(humedad);
  Serial.println("%");
  }
void humedaddht11externo()
  {
  delay(2000); // El DHT11 requiere un intervalo de 1-2 segundos entre lecturas

  float humedad = dhtexterno.readHumidity();
  float temperatura = dhtexterno.readTemperature(); // Temperatura en grados Celsius

  // Validar las lecturas
  if (isnan(humedad) || isnan(temperatura)) {
    Serial.println("Error al leer del sensor DHT11 externo");
    return;
  }

  Serial.print("Temperatura externa: ");
  Serial.print(temperatura);
  Serial.print("°C  |  Humedad externa: ");
  Serial.print(humedad);
  Serial.println("%");
  }
void escribirDatos() 
  {
  // Verificar si data.txt existe
  if (!SD.exists("data.txt")) {
    Serial.println("data.txt no existe, creando archivo...");
    // Crear archivo vacío para luego escribir
    archivo = SD.open("data.txt", FILE_WRITE);
    if (!archivo) {
      Serial.println("Error creando data.txt");
      return;
    }
    archivo.close();
  }
  
  // Abrir archivo para escritura al final sin borrar contenido
  archivo = SD.open("data.txt", FILE_WRITE);
  if (!archivo) {
    Serial.println("Error abriendo data.txt para escritura");
    return;
  }

  // Escribir fecha y hora
  obtenerhora();
  archivo.print(FYH);
  archivo.print(" | ");

  // Escribir las 30 variables separadas por comas
  archivo.print(LA1); archivo.print(", ");
  archivo.print(LA2); archivo.print(", ");
  archivo.print(LA3); archivo.print(", ");
  archivo.print(LA4); archivo.print(", ");
  archivo.print(LA5); archivo.print(", ");
  
  archivo.print(LB1); archivo.print(", ");
  archivo.print(LB2); archivo.print(", ");
  archivo.print(LB3); archivo.print(", ");
  archivo.print(LB4); archivo.print(", ");
  archivo.print(LB5); archivo.print(", ");

  archivo.print(LC1); archivo.print(", ");
  archivo.print(LC2); archivo.print(", ");
  archivo.print(LC3); archivo.print(", ");
  archivo.print(LC4); archivo.print(", ");
  archivo.print(LC5); archivo.print(", ");

  archivo.print(RA1); archivo.print(", ");
  archivo.print(RA2); archivo.print(", ");
  archivo.print(RA3); archivo.print(", ");
  archivo.print(RA4); archivo.print(", ");
  archivo.print(RA5); archivo.print(", ");

  archivo.print(RB1); archivo.print(", ");
  archivo.print(RB2); archivo.print(", ");
  archivo.print(RB3); archivo.print(", ");
  archivo.print(RB4); archivo.print(", ");
  archivo.print(RB5); archivo.print(", ");

  archivo.print(RC1); archivo.print(", ");
  archivo.print(RC2); archivo.print(", ");
  archivo.print(RC3); archivo.print(", ");
  archivo.print(RC4); archivo.print(", ");
  archivo.print(RC5);

  archivo.println(); // Nueva línea para la siguiente fila de datos
  archivo.close();

  Serial.println("Datos guardados correctamente.");
  }
void pingToHost() 
  {
  Serial.print("Pinging ");
  Serial.print(hostName);
  Serial.print(": ");

  pingResult = WiFi.ping(hostName);

  if (pingResult >= 0) {
    Serial.print("SUCCESS! RTT = ");
    Serial.print(pingResult);
    Serial.println(" ms");
  } else {
    Serial.print("FAILED! Error code: ");
    Serial.println(pingResult);
  }

  delay(5000);
  }  
void WiFiconnect()
  {
  //----------------FUNCIÓN EN DESUSO------------------
  int status = WL_IDLE_STATUS;     // the WiFi radio's status
  // Specify IP address or hostname
  int pingResult;

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 5 seconds for connection:
    delay(5000);
  }

  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  printCurrentNet();
  printWiFiData();
  }
void printWiFiData() 
  {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP address : ");
  Serial.println(ip);

  Serial.print("Subnet mask: ");
  Serial.println((IPAddress)WiFi.subnetMask());

  Serial.print("Gateway IP : ");
  Serial.println((IPAddress)WiFi.gatewayIP());

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
  }
void printCurrentNet() 
  {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI): ");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type: ");
  Serial.println(encryption, HEX);
  Serial.println();
  }
void printMacAddress(byte mac[]) 
  {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
  }  
void barometro() {
  Serial.print("Temperatura: ");
  Serial.print(bmp.readTemperature());
  Serial.println(" °C");

  Serial.print("Presión: ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print("Altitud: ");
  Serial.print(bmp.readAltitude(101325));  // Presión al nivel del mar estándar
  Serial.println(" metros");

  Serial.println("---");
  delay(2000);
}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
  switch (type)
  {
    case WStype_DISCONNECTED:
      Serial.print("[");
      Serial.print(num);
      Serial.println("] Cliente desconectado");
      break;

    case WStype_CONNECTED:
      Serial.print("[");
      Serial.print(num);
      Serial.println("] Cliente conectado");

      // Mensaje de bienvenida al cliente que se conectó
      webSocket.sendTXT(num, "OK: Conectado al MKR1000");
      break;

    case WStype_TEXT:
    {
      // Convertir payload -> String de forma segura usando length
      char msg[/*length*/ 1]; // placeholder para compilar en algunos editores
      // En Arduino IDE / GCC, puedes usar VLA (char msg[length+1]) pero no es estándar C++.
      // Mejor: usa un buffer fijo si controlas el tamaño máximo del mensaje.
      // Aquí va una versión segura con buffer fijo:
      const size_t MAX_MSG = 128;               // Ajusta según tus comandos
      size_t n = (length < MAX_MSG) ? length : (MAX_MSG - 1);
      char buf[MAX_MSG];
      memcpy(buf, payload, n);
      buf[n] = '\0';

      String comando = String(buf);             // (char*)payload es común, pero aquí ya es seguro [web:78]
      comando.trim();

      Serial.print("[");
      Serial.print(num);
      Serial.print("] Texto recibido: ");
      Serial.println(comando);

      if (comando == "help") {
        webSocket.sendTXT(num, "Comandos: help, 1=leersensores, 2=obtenerhora, 3=imprimirhora, 4=obtener+imprimir, 5=DHT int, 6=DHT ext, 8=ping, 9=guardar SD");
      }
      else if (comando == "1") {
        webSocket.sendTXT(num, "OK: iniciando lectura sensores...");
        leersensores(1000);                     // OJO: esto bloquea bastante por tus delays
        webSocket.sendTXT(num, "OK: lectura sensores finalizada");
      }
      else if (comando == "2") {
        obtenerhora();
        webSocket.sendTXT(num, "OK: hora obtenida (usa comando 3 para imprimir)");
      }
      else if (comando == "3") {
        // FYH ya debe estar actualizado con obtenerhora()
        webSocket.sendTXT(num, FYH.c_str());
      }
      else if (comando == "4") {
        obtenereimprimirhora();
        webSocket.sendTXT(num, FYH.c_str());
      }
      else if (comando == "5") {
        // Tu función actual imprime a Serial; si quieres responder al navegador,
        // conviértela a una función que retorne String y envíala acá.
        humedaddht11interno();
        webSocket.sendTXT(num, "OK: DHT interno leído (ver Serial).");
      }
      else if (comando == "6") {
        humedaddht11externo();
        webSocket.sendTXT(num, "OK: DHT externo leído (ver Serial).");
      }
      else if (comando == "8") {
        pingToHost();
        webSocket.sendTXT(num, "OK: ping ejecutado (ver Serial).");
      }
      else if (comando == "9") {
        escribirDatos();
        webSocket.sendTXT(num, "OK: datos guardados en SD");
      }
      else {
        webSocket.sendTXT(num, "ERR: comando no reconocido (usa help)");
      }
      break;
    }

    default:
      // Para otros tipos (BIN, PING, PONG, etc.)
      break;
  }
}
void TSL2561() {
  sensors_event_t event;
  tsl.getEvent(&event);

  Serial.print("Luz exterior: ");
  Serial.print(event.light, 1);  // 1 decimal para precisión
  Serial.println(" lux");
  
  // Clasificación específica para AIRE LIBRE
  if (event.light < 100) {
    Serial.println("NOCHE / Amanecer / Anochecer");
  } else if (event.light < 1000) {
    Serial.println("AMANECER / ATARDECER");
  } else if (event.light < 10000) {
    Serial.println("NUBLADO");
  } else if (event.light < 50000) {
    Serial.println("SOL DIRECTO");
  } else {
    Serial.println("FUERA DE RANGO");
  }
  
  // Conversión aproximada a índice UV (correlación aproximada)
  float uvIndex = event.light / 2500.0;
  Serial.print("Índice UV aproximado: ");
  Serial.println(uvIndex, 1);
  
  Serial.println("---");
}
void WifiLab()
  {
      int intentosWiFi = 0;
  int maxIntentos = 3;
  
  Serial.println("Iniciando conexion WiFi...");
  
  // Intenta conectar mientras NO esté conectado Y los intentos sean menores al máximo
  while (WiFi.begin(ssid, pass) != WL_CONNECTED && intentosWiFi < maxIntentos) 
  {
    intentosWiFi++;
    Serial.print("Intento ");
    Serial.print(intentosWiFi);
    Serial.println(" fallido. Reintentando en 5 segundos...");
    delay(5000); // El chip WiFi101 necesita tiempo entre intentos
  }

  // Evaluamos cómo salió del bucle
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi Conectado! IP: ");
    IPAddress ip = WiFi.localIP();
    Serial.print("IP address : ");
    Serial.println(ip);
    wifiConectado = true;
    
    // Iniciamos el servidor WebSocket SOLAMENTE si hay WiFi
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

  } else {
    Serial.println("Error de red: Imposible conectar tras 3 intentos.");
    Serial.println("Iniciando en MODO OFFLINE. Los datos se guardaran localmente en la SD.");
    wifiConectado = false;
  }

  }

void startWebServerAP() {
  if (serverRunning) {
    Serial.println("Servidor ya está corriendo");
    return;
  }

  Serial.println("Iniciando Access Point Web Server");


  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield no presente");
    while (true);
  }

  Serial.print("Creando access point: ");
  Serial.println(ssid);

  status = WiFi.beginAP(ap_ssid, 10, ap_pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Fallo al crear access point");
    while (true);
  }

  delay(10000);
  server.begin();
  serverRunning = true;
}

void handleWebServer() {
  while (serverRunning) {  // Bucle bloqueante, se rompe solo si serverRunning = false
    if (status != WiFi.status()) {
      status = WiFi.status();
      if (status == WL_AP_CONNECTED) {
        byte remoteMac[6];
        Serial.print("Dispositivo conectado, MAC: ");
        WiFi.APClientMacAddress(remoteMac);
        printMacAddress(remoteMac);
      } else {
        Serial.println("Dispositivo desconectado del AP");
      }
    }

    WiFiClient client = server.available();
    if (client) {
      Serial.println("Nuevo cliente");
      String currentLine = "";
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          Serial.write(c);
          if (c == '\n') {
            if (currentLine.length() == 0) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();
              client.print("Click <a href=\"/H\">here</a> para encender LED<br>");
              client.print("Click <a href=\"/L\">here</a> para apagar LED<br>");
              client.println();
              break;
            } else {
              currentLine = "";
            }
          } else if (c != '\r') {
            currentLine += c;
          }

          if (currentLine.endsWith("GET /H")) {
            Serial.println("OBTENIDO H");
          }
          if (currentLine.endsWith("GET /L")) {
            Serial.println("OBTENIDO L");
          }
        }
      }
      client.stop();
      Serial.println("Cliente desconectado");
      serverRunning = false;
    }
  }
  Serial.println("Servidor web detenido");
}


//a
