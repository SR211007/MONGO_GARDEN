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
//-------------------------------
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
  int humidityValuesRight[15];
  int humidityValuesLeft[15];
  int H, TC, TF, HIC, HIF;
  int pingResult;
  int status = 0;
  const int CS_PIN = 7;
  int demoMode;
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
  if (!bmp.begin()) {Serial.println("Error inicializando BMP180");} else {Serial.println("BMP180 inicializado");}
  if (!tsl.begin()) {Serial.println("Error inicializando TSL2561");} else {Serial.println("TSL2561 inicializado");tsl.setGain(TSL2561_GAIN_1X);tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);}
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
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input.length() == 0) return;

      int firstComma  = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      int thirdComma  = input.indexOf(',', secondComma + 1);

      if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
        Serial.println("Formato invalido. Usa: SET_AND_READ, BOTH_SIDES, 5, 3");
        return;
      }

      String modeText    = input.substring(0, firstComma);
      String sideText    = input.substring(firstComma + 1, secondComma);
      String channelText = input.substring(secondComma + 1, thirdComma);
      String slotText    = input.substring(thirdComma + 1);

      modeText.trim();
      sideText.trim();
      channelText.trim();
      slotText.trim();

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
        return;
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
        return;
      }

      int channelMultiplexor = channelText.toInt();
      int soilSlot = slotText.toInt();

      Serial.println(getSoilHumidity(mode, side, channelMultiplexor, soilSlot));
    }
  }

String getSoilHumidity(FunctionMode functionMode, GardenSide gardenSide, int channelMultiplexor, int soilSlot)
  {
    if (channelMultiplexor < 0 || channelMultiplexor > 14) {return "Invalid channelMultiplexor value";}
    if (soilSlot < 0 || soilSlot > 14) {return "Invalid soilSlot value";}

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
  if (slot < 0 || slot > 14) 
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
  if (slot < 0 || slot > 14) 
    {
     return "Slot " + String(slot) + " invalido";
    }

  delay(100);
  humidityValuesLeft[slot] = analogRead(SIG2);
  String outputMessage = String("L")+String(slot)+String(": ")+String(humidityValuesLeft[slot]);
  return outputMessage;
 }








