/*
  ============================================================
  ESP32-WROOM-32U - STEROWNIK AKWARIUM
  ============================================================

  SPRZET:

  ADS1115 / pH:
    SDA  -> GPIO21
    SCL  -> GPIO22

  Pompa kalkwasser STEP/DIR:
    STEP -> GPIO19
    DIR  -> GPIO18

  2x DS18B20:
    DATA -> GPIO16
    VCC  -> 3.3V
    GND  -> GND
    jeden rezystor 4.7k pomiedzy GPIO16 i 3.3V

  Zachowane:
    GPIO25 -> OUTPUT HIGH
    GPIO23 -> OUTPUT HIGH

  SP301:
    sterowanie lokalne przez WiFi / Tuya 3.3
    brak polaczenia elektrycznego z GPIO ESP32

  ============================================================
  FUNKCJE
  ============================================================

  - 2x temperatura DS18B20
  - autonomiczny termostat
  - Setti+ SP301 ON/OFF
  - odczyt W / A / V z SP301
  - pomiar pH ADS1115
  - kalibracja sondy pH przez Serial
  - pompa kalkwasser STEP/DIR
  - kalkwasser 24h/dobe
  - ograniczenie dozowania przez pH
  - TFT
  - konfiguracja WWW
  - WiFi AP + STA
  - OTA
  - NVS
  - reef-pi

  REEF-PI:

    OUTLET 0:
      POST /outlets/0/on
      POST /outlets/0/off
      -> GRZANIE AUTO

    OUTLET 1:
      POST /outlets/1/on
      POST /outlets/1/off
      -> KALK AUTO

    ANALOG:

      /analog_inputs/0 -> temperatura 1
      /analog_inputs/1 -> temperatura 2
      /analog_inputs/2 -> pH
      /analog_inputs/3 -> W
      /analog_inputs/4 -> A
      /analog_inputs/5 -> V
      /analog_inputs/6 -> dKH

  ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>

#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_ADS1X15.h>
#include "DFRobot_ESP_PH_WITH_ADC.h"

#include <TFT_eSPI.h>
#include <SPI.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <mbedtls/aes.h>
#include <esp_arduino_version.h>

#include <time.h>
#include <math.h>


// ============================================================
// WIFI / AP / OTA
// ============================================================

const char* AP_SSID = "ESP32-AKWARIUM";
const char* AP_PASSWORD = "esp32pass";

const char* OTA_HOST = "esp32-akwarium";
const char* OTA_PASS = "otapass";

// Uzywane tylko jezeli nie ma WiFi zapisanego w NVS.
const char* DEFAULT_WIFI_SSID = "PLAY_Swiatlowodowy_24";
const char* DEFAULT_WIFI_PASS = "E1T8C&ZdAbwx";


// ============================================================
// SP301 / TUYA
// ============================================================

const char* TUYA_IP = "192.168.0.221";
const uint16_t TUYA_PORT = 6668;

// Device ID Twojego SP301:
const char* DEVICE_ID = "bf6bcf672c49f840f9ka0v";

// WSTAW SWOJ 16-ZNAKOWY LOCAL KEY.
// Nie wpisuj tutaj API Secret.
const char* LOCAL_KEY = "L=7gRvCbE+o>uSb<";

uint32_t tuyaSeq = 1;


// ============================================================
// PINY
// ============================================================

// ADS1115
#define I2C_SDA 21
#define I2C_SCL 22

// Pompa perystaltyczna
#define PUMP_STEP_PIN 19
#define PUMP_DIR_PIN  18

// Zachowane ze starego programu
#define PIN_AUX 25
#define PIN_23  23

// DWA DS18B20 NA JEDNEJ MAGISTRALI
#define ONE_WIRE_PIN 16


// ============================================================
// HTTP / NVS
// ============================================================

WebServer server(80);
Preferences prefs;


// ============================================================
// pH
// ============================================================

DFRobot_ESP_PH_WITH_ADC ph;
Adafruit_ADS1115 ads;

bool gADSOK = false;
uint8_t gADSAddr = 0;

float voltage = 0.0f;
float phValue = 0.0f;
float phAverage = 0.0f;

// Do kompensacji temperaturowej sondy pH.
float temperature = 25.0f;

static const int PH_AVG_N = 30;

float phReadings[PH_AVG_N] = {0};
int phReadingsIndex = 0;

bool phPrimed = false;

unsigned long lastPHUpdateTime = 0;

const unsigned long PH_UPDATE_INTERVAL = 1000;


// ============================================================
// KALKWASSER
// ============================================================

// Predkosc w krokach/s.
static const uint16_t DEFAULT_PUMP_SPEED = 200;
static const uint16_t MAX_PUMP_SPEED = 2000;

uint16_t pumpSpeed = DEFAULT_PUMP_SPEED;


// Na razie 0 = limit objetosciowy nieaktywny.
// Po kalibracji pompy zrobimy ml/krok i ml/dobe.
float dailyKalkLimitML = 0.0f;


// Progi startowe pH.
//
// ponizej / rowne 8.35 -> znow mozna dozowac
// 8.45                 -> zatrzymanie
// 8.60                 -> bezwzgledny STOP
//
float kalkPHRestart = 8.35f;
float kalkPHStop    = 8.45f;
float kalkPHHard    = 8.60f;


// Kalk moze pracowac cala dobe.
// reef-pi moze pozniej wlaczac/wylaczac AUTO
// wedlug harmonogramu.
bool kalkAutoEnabled = true;


// Po restarcie pompa NIE rusza natychmiast.
// Najpierw musi pojawic sie poprawne pH.
bool kalkPHBlocked = true;


// Fizyczny stan pompy
bool pumpPhysicalRunning = false;
uint16_t pumpPhysicalSpeed = 0;


// Kierunek pracy pompy.
//
// Jezeli po uruchomieniu bedzie krecic w zla strone,
// zmien HIGH na LOW.
const uint8_t PUMP_FORWARD_LEVEL = HIGH;


#if ESP_ARDUINO_VERSION_MAJOR < 3

const uint8_t PUMP_PWM_CHANNEL = 7;

#endif


// ============================================================
// dKH
// ============================================================

float savedDKH = 0.0f;


// ============================================================
// TEMPERATURA
// ============================================================

#define TEMP_SENSOR_COUNT 2

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature tempBus(&oneWire);

DeviceAddress tempAddress[TEMP_SENSOR_COUNT];

int foundTempSensors = 0;

float tempC[TEMP_SENSOR_COUNT] =
{
  DEVICE_DISCONNECTED_C,
  DEVICE_DISCONNECTED_C
};

bool tempValid[TEMP_SENSOR_COUNT] =
{
  false,
  false
};

unsigned long lastTempUpdateTime = 0;

const unsigned long TEMP_UPDATE_INTERVAL = 2000;


// ============================================================
// GRZALKA
// ============================================================

float heaterOnTemp  = 24.80f;
float heaterOffTemp = 25.20f;

float heaterHardMax = 27.00f;

// Jezeli T1 i T2 roznia sie bardziej niz 1 C,
// grzalka zostaje wylaczona.
float maxTempDifference = 1.00f;


// Minimalny czas pomiedzy zwyklymi przelaczeniami.
const unsigned long MIN_HEATER_SWITCH_MS = 10000;


bool heaterAutoEnabled = true;

// Stan, ktory ESP chce miec na SP301.
bool heaterDesiredState = false;

unsigned long lastHeaterSwitchMs = 0;
unsigned long lastHeaterCommandMs = 0;


// ============================================================
// SP301 TELEMETRIA
// ============================================================

bool plugRelayKnown = false;
bool plugRelayState = false;

bool plugTelemetryValid = false;

float plugPowerW = 0.0f;
float plugCurrentA = 0.0f;
float plugVoltageV = 0.0f;

unsigned long lastPowerUpdateTime = 0;

const unsigned long POWER_UPDATE_INTERVAL = 5000;


// ============================================================
// TFT
// ============================================================

TFT_eSPI myGLCD = TFT_eSPI();

unsigned long lastTFTUpdate = 0;

const unsigned long TFT_UPDATE_INTERVAL = 1000;


// ============================================================
// NARZEDZIA BIG-ENDIAN
// ============================================================

void write32be(uint8_t* p, uint32_t value)
{
  p[0] = (value >> 24) & 0xFF;
  p[1] = (value >> 16) & 0xFF;
  p[2] = (value >> 8) & 0xFF;
  p[3] = value & 0xFF;
}


uint32_t read32be(const uint8_t* p)
{
  return
      ((uint32_t)p[0] << 24)
    | ((uint32_t)p[1] << 16)
    | ((uint32_t)p[2] << 8)
    | ((uint32_t)p[3]);
}


// ============================================================
// TUYA CRC32
// ============================================================

uint32_t crc32Tuya(const uint8_t* data, size_t len)
{
  uint32_t crc = 0xFFFFFFFF;

  for(size_t i = 0; i < len; i++)
  {
    crc ^= data[i];

    for(int j = 0; j < 8; j++)
    {
      if(crc & 1)
      {
        crc =
          (crc >> 1)
          ^ 0xEDB88320;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc ^ 0xFFFFFFFF;
}


// ============================================================
// AES-128 ECB
// ============================================================

size_t encryptAES(
  const uint8_t* input,
  size_t inputLen,
  uint8_t* output
)
{
  size_t padding =
    16 - (inputLen % 16);

  size_t paddedLen =
    inputLen + padding;


  uint8_t* padded =
    new uint8_t[paddedLen];


  memcpy(
    padded,
    input,
    inputLen
  );


  for(
    size_t i = inputLen;
    i < paddedLen;
    i++
  )
  {
    padded[i] = padding;
  }


  mbedtls_aes_context aes;

  mbedtls_aes_init(&aes);


  mbedtls_aes_setkey_enc(
    &aes,
    (const unsigned char*)LOCAL_KEY,
    128
  );


  for(
    size_t i = 0;
    i < paddedLen;
    i += 16
  )
  {
    mbedtls_aes_crypt_ecb(
      &aes,
      MBEDTLS_AES_ENCRYPT,
      padded + i,
      output + i
    );
  }


  mbedtls_aes_free(&aes);

  delete[] padded;

  return paddedLen;
}


size_t decryptAES(
  const uint8_t* input,
  size_t inputLen,
  uint8_t* output
)
{
  if(
    inputLen == 0 ||
    (inputLen % 16) != 0
  )
  {
    return 0;
  }


  mbedtls_aes_context aes;

  mbedtls_aes_init(&aes);


  mbedtls_aes_setkey_dec(
    &aes,
    (const unsigned char*)LOCAL_KEY,
    128
  );


  for(
    size_t i = 0;
    i < inputLen;
    i += 16
  )
  {
    mbedtls_aes_crypt_ecb(
      &aes,
      MBEDTLS_AES_DECRYPT,
      input + i,
      output + i
    );
  }


  mbedtls_aes_free(&aes);


  uint8_t padding =
    output[inputLen - 1];


  if(
    padding >= 1 &&
    padding <= 16
  )
  {
    bool goodPadding = true;

    for(
      size_t i =
        inputLen - padding;
      i < inputLen;
      i++
    )
    {
      if(output[i] != padding)
      {
        goodPadding = false;
        break;
      }
    }

    if(goodPadding)
    {
      return inputLen - padding;
    }
  }


  return inputLen;
}


// ============================================================
// TIMESTAMP TUYA
// ============================================================

uint32_t getTuyaTimestamp()
{
  time_t now = time(nullptr);

  // Prawidlowy Unix time.
  if(now > 1700000000)
  {
    return (uint32_t)now;
  }

  // Awaryjny timestamp gdy brak Internetu/NTP.
  return
    1700000000UL
    + millis() / 1000UL;
}


// ============================================================
// TCP - ODCZYT DOKLADNEJ LICZBY BAJTOW
// ============================================================

bool readExact(
  WiFiClient& client,
  uint8_t* buffer,
  size_t length,
  unsigned long timeoutMs
)
{
  size_t received = 0;

  unsigned long start =
    millis();


  while(
    received < length &&
    millis() - start < timeoutMs
  )
  {
    int avail =
      client.available();


    if(avail > 0)
    {
      size_t want =
        length - received;

      if((size_t)avail < want)
      {
        want = avail;
      }


      int n =
        client.read(
          buffer + received,
          want
        );


      if(n > 0)
      {
        received += n;
      }
    }
    else
    {
      delay(1);
    }
  }


  return received == length;
}


// ============================================================
// ODCZYT RAMKI TUYA 3.3
// ============================================================

bool readTuyaFrame(
  WiFiClient& client,
  String& jsonOut,
  uint32_t& retCode
)
{
  jsonOut = "";
  retCode = 0xFFFFFFFF;


  uint8_t header[16];


  if(!readExact(
      client,
      header,
      sizeof(header),
      1000
  ))
  {
    return false;
  }


  if(read32be(header) != 0x000055AA)
  {
    Serial.println(
      "[TUYA] zly prefix"
    );

    return false;
  }


  uint32_t bodyLength =
    read32be(header + 12);


  if(
    bodyLength < 8 ||
    bodyLength > 4096
  )
  {
    Serial.printf(
      "[TUYA] zla dlugosc: %lu\n",
      (unsigned long)bodyLength
    );

    return false;
  }


  uint8_t* body =
    new uint8_t[bodyLength];


  if(!readExact(
      client,
      body,
      bodyLength,
      1000
  ))
  {
    delete[] body;
    return false;
  }


  if(bodyLength < 12)
  {
    delete[] body;
    return false;
  }


  retCode =
    read32be(body);


  uint32_t suffix =
    read32be(
      body +
      bodyLength -
      4
    );


  if(suffix != 0x0000AA55)
  {
    Serial.println(
      "[TUYA] zly suffix"
    );

    delete[] body;

    return false;
  }


  /*
    BODY ODPOWIEDZI:

      4 B  retCode
      ...  payload
      4 B  CRC
      4 B  AA55
  */

  size_t payloadLength =
    bodyLength - 12;


  if(payloadLength == 0)
  {
    delete[] body;
    return true;
  }


  uint8_t* payload =
    body + 4;


  // Niektore odpowiedzi sa zwyklym JSON.
  if(payload[0] == '{')
  {
    char* temp =
      new char[payloadLength + 1];


    memcpy(
      temp,
      payload,
      payloadLength
    );


    temp[payloadLength] = 0;

    jsonOut = String(temp);


    delete[] temp;
    delete[] body;

    return true;
  }


  /*
    Tuya 3.3:
      "3.3" + 12 bajtow
  */

  if(
    payloadLength >= 15 &&
    payload[0] == '3' &&
    payload[1] == '.' &&
    payload[2] == '3'
  )
  {
    payload += 15;
    payloadLength -= 15;
  }
  else if(
    payloadLength > 15 &&
    (payloadLength % 16) != 0
  )
  {
    // device22
    payload += 15;
    payloadLength -= 15;
  }


  if(
    payloadLength == 0 ||
    (payloadLength % 16) != 0
  )
  {
    delete[] body;

    return true;
  }


  uint8_t* decrypted =
    new uint8_t[
      payloadLength + 1
    ];


  size_t decryptedLength =
    decryptAES(
      payload,
      payloadLength,
      decrypted
    );


  decrypted[decryptedLength] = 0;


  if(decryptedLength > 0)
  {
    String result =
      String(
        (char*)decrypted
      );


    int first =
      result.indexOf('{');

    int last =
      result.lastIndexOf('}');


    if(
      first >= 0 &&
      last >= first
    )
    {
      jsonOut =
        result.substring(
          first,
          last + 1
        );
    }
  }


  delete[] decrypted;
  delete[] body;

  return true;
}


// ============================================================
// WYSYLANIE TUYA
// ============================================================

bool sendTuyaJson(
  uint32_t command,
  const String& json,
  String* responseJson = nullptr
)
{
  if(strlen(LOCAL_KEY) != 16)
  {
    Serial.println(
      "[TUYA] BLAD: LOCAL_KEY musi miec 16 znakow"
    );

    return false;
  }


  if(WiFi.status() != WL_CONNECTED)
  {
    Serial.println(
      "[TUYA] brak WiFi"
    );

    return false;
  }


  WiFiClient client;

  client.setNoDelay(true);


  if(!client.connect(
      TUYA_IP,
      TUYA_PORT
  ))
  {
    Serial.println(
      "[TUYA] SP301 offline"
    );

    return false;
  }


  size_t maxEncrypted =
    ((json.length() / 16) + 1)
    * 16;


  uint8_t* encrypted =
    new uint8_t[maxEncrypted];


  size_t encryptedLength =
    encryptAES(
      (const uint8_t*)
        json.c_str(),
      json.length(),
      encrypted
    );


  const size_t versionHeaderLength =
    15;


  size_t payloadLength =
    versionHeaderLength
    + encryptedLength;


  uint8_t* payload =
    new uint8_t[payloadLength];


  payload[0] = '3';
  payload[1] = '.';
  payload[2] = '3';


  memset(
    payload + 3,
    0,
    12
  );


  memcpy(
    payload + 15,
    encrypted,
    encryptedLength
  );


  delete[] encrypted;


  uint32_t lengthField =
    payloadLength + 8;


  size_t packetWithoutTrailer =
    16 + payloadLength;


  size_t totalPacket =
    packetWithoutTrailer + 8;


  uint8_t* packet =
    new uint8_t[totalPacket];


  write32be(
    packet + 0,
    0x000055AA
  );


  write32be(
    packet + 4,
    tuyaSeq++
  );


  write32be(
    packet + 8,
    command
  );


  write32be(
    packet + 12,
    lengthField
  );


  memcpy(
    packet + 16,
    payload,
    payloadLength
  );


  delete[] payload;


  uint32_t crc =
    crc32Tuya(
      packet,
      packetWithoutTrailer
    );


  write32be(
    packet +
    packetWithoutTrailer,
    crc
  );


  write32be(
    packet +
    packetWithoutTrailer +
    4,
    0x0000AA55
  );


  client.write(
    packet,
    totalPacket
  );


  client.flush();

  delete[] packet;


  bool gotFrame = false;


  for(int i = 0; i < 3; i++)
  {
    String decoded;

    uint32_t retCode;


    if(!readTuyaFrame(
        client,
        decoded,
        retCode
    ))
    {
      break;
    }


    gotFrame = true;


    if(retCode != 0)
    {
      Serial.printf(
        "[TUYA] retCode=%lu\n",
        (unsigned long)retCode
      );
    }


    if(
      responseJson != nullptr &&
      decoded.length() > 0
    )
    {
      *responseJson =
        decoded;

      client.stop();

      return retCode == 0;
    }


    // Dla CONTROL pusty ACK jest prawidlowa odpowiedzia.
    if(responseJson == nullptr)
    {
      client.stop();

      return retCode == 0;
    }
  }


  client.stop();


  if(responseJson != nullptr)
  {
    return false;
  }


  return gotFrame;
}


// ============================================================
// PROSTE PARSOWANIE DPS
// ============================================================

bool jsonGetLong(
  const String& json,
  const char* key,
  long& value
)
{
  String token =
    "\"" +
    String(key) +
    "\":";


  int pos =
    json.indexOf(token);


  if(pos < 0)
  {
    return false;
  }


  pos += token.length();


  while(
    pos < json.length() &&
    (
      json[pos] == ' ' ||
      json[pos] == '"'
    )
  )
  {
    pos++;
  }


  bool negative = false;


  if(
    pos < json.length() &&
    json[pos] == '-'
  )
  {
    negative = true;
    pos++;
  }


  if(
    pos >= json.length() ||
    !isDigit(json[pos])
  )
  {
    return false;
  }


  long result = 0;


  while(
    pos < json.length() &&
    isDigit(json[pos])
  )
  {
    result =
      result * 10
      + json[pos]
      - '0';

    pos++;
  }


  value =
    negative
    ? -result
    : result;


  return true;
}


bool jsonGetBool(
  const String& json,
  const char* key,
  bool& value
)
{
  String token =
    "\"" +
    String(key) +
    "\":";


  int pos =
    json.indexOf(token);


  if(pos < 0)
  {
    return false;
  }


  pos += token.length();


  while(
    pos < json.length() &&
    json[pos] == ' '
  )
  {
    pos++;
  }


  if(
    json.substring(
      pos,
      pos + 4
    ) == "true"
  )
  {
    value = true;

    return true;
  }


  if(
    json.substring(
      pos,
      pos + 5
    ) == "false"
  )
  {
    value = false;

    return true;
  }


  return false;
}


// ============================================================
// SP301 - GRZALKA ON/OFF
// ============================================================

bool sendHeaterCommand(bool state)
{
  String json =
    "{\"devId\":\"" +
    String(DEVICE_ID) +

    "\",\"uid\":\"" +
    String(DEVICE_ID) +

    "\",\"t\":\"" +
    String(getTuyaTimestamp()) +

    "\",\"dps\":{\"1\":" +
    String(
      state
      ? "true"
      : "false"
    ) +

    "}}";


  Serial.printf(
    "[HEATER] SP301 -> %s\n",
    state
      ? "ON"
      : "OFF"
  );


  bool ok =
    sendTuyaJson(
      0x07,
      json
    );


  lastHeaterCommandMs =
    millis();


  if(ok)
  {
    heaterDesiredState =
      state;

    lastHeaterSwitchMs =
      millis();
  }


  return ok;
}


// ============================================================
// SP301 - TELEMETRIA
// ============================================================

bool readPlugTelemetry()
{
  /*
    Twoj SP301 jest device22.

    DPS:
      1  = relay
      18 = prad mA
      19 = moc /10 W
      20 = napiecie /10 V
  */

  String json =
    "{\"devId\":\"" +
    String(DEVICE_ID) +

    "\",\"uid\":\"" +
    String(DEVICE_ID) +

    "\",\"t\":\"" +
    String(getTuyaTimestamp()) +

    "\",\"dps\":{"

    "\"1\":null,"
    "\"18\":null,"
    "\"19\":null,"
    "\"20\":null"

    "}}";


  String response;


  // device22 DP QUERY -> CONTROL_NEW 0x0D
  bool ok =
    sendTuyaJson(
      0x0D,
      json,
      &response
    );


  if(
    !ok ||
    response.length() == 0
  )
  {
    plugTelemetryValid = false;

    Serial.println(
      "[SP301] brak telemetrii"
    );

    return false;
  }


  bool relay = false;

  long currentRaw = 0;
  long powerRaw = 0;
  long voltageRaw = 0;


  bool haveRelay =
    jsonGetBool(
      response,
      "1",
      relay
    );


  bool haveCurrent =
    jsonGetLong(
      response,
      "18",
      currentRaw
    );


  bool havePower =
    jsonGetLong(
      response,
      "19",
      powerRaw
    );


  bool haveVoltage =
    jsonGetLong(
      response,
      "20",
      voltageRaw
    );


  if(haveRelay)
  {
    plugRelayKnown = true;

    plugRelayState =
      relay;
  }


  if(haveCurrent)
  {
    plugCurrentA =
      currentRaw /
      1000.0f;
  }


  if(havePower)
  {
    plugPowerW =
      powerRaw /
      10.0f;
  }


  if(haveVoltage)
  {
    plugVoltageV =
      voltageRaw /
      10.0f;
  }


  plugTelemetryValid =
    haveCurrent &&
    havePower &&
    haveVoltage;


  Serial.printf(
    "[SP301] %s | %.1f W | %.3f A | %.1f V\n",

    plugRelayKnown
      ? (
          plugRelayState
          ? "ON"
          : "OFF"
        )
      : "?",

    plugPowerW,
    plugCurrentA,
    plugVoltageV
  );


  return plugTelemetryValid;
}


// ============================================================
// POMPA - SPRZETOWE GENEROWANIE STEP
// ============================================================

void setupPump()
{
  pinMode(
    PUMP_DIR_PIN,
    OUTPUT
  );


  digitalWrite(
    PUMP_DIR_PIN,
    PUMP_FORWARD_LEVEL
  );


#if ESP_ARDUINO_VERSION_MAJOR >= 3

  ledcAttach(
    PUMP_STEP_PIN,
    1000,
    8
  );


  ledcWrite(
    PUMP_STEP_PIN,
    0
  );

#else

  ledcSetup(
    PUMP_PWM_CHANNEL,
    1000,
    8
  );


  ledcAttachPin(
    PUMP_STEP_PIN,
    PUMP_PWM_CHANNEL
  );


  ledcWrite(
    PUMP_PWM_CHANNEL,
    0
  );

#endif


  pumpPhysicalRunning = false;
  pumpPhysicalSpeed = 0;
}


void setPumpOutput(uint16_t speed)
{
  if(speed > MAX_PUMP_SPEED)
  {
    speed = MAX_PUMP_SPEED;
  }


  if(pumpPhysicalSpeed == speed)
  {
    return;
  }


  pumpPhysicalSpeed =
    speed;


  if(speed == 0)
  {

#if ESP_ARDUINO_VERSION_MAJOR >= 3

    ledcWrite(
      PUMP_STEP_PIN,
      0
    );

#else

    ledcWrite(
      PUMP_PWM_CHANNEL,
      0
    );

#endif

    pumpPhysicalRunning = false;


    Serial.println(
      "[KALK] POMPA STOP"
    );


    return;
  }


  digitalWrite(
    PUMP_DIR_PIN,
    PUMP_FORWARD_LEVEL
  );


#if ESP_ARDUINO_VERSION_MAJOR >= 3

  ledcWriteTone(
    PUMP_STEP_PIN,
    speed
  );

#else

  ledcWriteTone(
    PUMP_PWM_CHANNEL,
    speed
  );

#endif


  pumpPhysicalRunning = true;


  Serial.printf(
    "[KALK] POMPA %u krokow/s\n",
    speed
  );
}


// ============================================================
// NVS
// ============================================================

void loadSettings()
{
  prefs.begin(
    "controller",
    true
  );


  pumpSpeed =
    prefs.getUShort(
      "pumpSpeed",
      DEFAULT_PUMP_SPEED
    );


  savedDKH =
    prefs.getFloat(
      "dkh",
      0.0f
    );


  heaterAutoEnabled =
    prefs.getBool(
      "heatAuto",
      true
    );


  kalkAutoEnabled =
    prefs.getBool(
      "kalkAuto",
      true
    );


  prefs.end();


  if(pumpSpeed > MAX_PUMP_SPEED)
  {
    pumpSpeed =
      DEFAULT_PUMP_SPEED;
  }


  if(
    !isfinite(savedDKH) ||
    savedDKH < 0.0f ||
    savedDKH > 30.0f
  )
  {
    savedDKH = 0.0f;
  }
}


void savePumpSpeed()
{
  prefs.begin(
    "controller",
    false
  );


  prefs.putUShort(
    "pumpSpeed",
    pumpSpeed
  );


  prefs.end();
}


void saveDKH()
{
  prefs.begin(
    "controller",
    false
  );


  prefs.putFloat(
    "dkh",
    savedDKH
  );


  prefs.end();
}


void saveAutomation()
{
  prefs.begin(
    "controller",
    false
  );


  prefs.putBool(
    "heatAuto",
    heaterAutoEnabled
  );


  prefs.putBool(
    "kalkAuto",
    kalkAutoEnabled
  );


  prefs.end();
}


// ============================================================
// ADS1115
// ============================================================

void i2cScan()
{
  Serial.println(
    "[I2C] scan"
  );


  byte count = 0;


  for(
    byte addr = 1;
    addr < 127;
    addr++
  )
  {
    Wire.beginTransmission(addr);

    byte err =
      Wire.endTransmission();


    if(err == 0)
    {
      Serial.printf(
        "  -> 0x%02X\n",
        addr
      );

      count++;
    }
  }


  if(!count)
  {
    Serial.println(
      "  brak urzadzen I2C"
    );
  }
}


/*
  UWAGA:

  Nie ma juz automatycznej proby na GPIO16/17.

  GPIO16 nalezy TERAZ wylacznie do DS18B20.

  ADS1115 pozostaje:
    SDA 21
    SCL 22
*/

bool initI2CAndADS()
{
  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );


  Wire.setClock(
    100000
  );


  delay(10);


  Serial.printf(
    "[I2C] SDA=%d SCL=%d\n",
    I2C_SDA,
    I2C_SCL
  );


  i2cScan();


  const uint8_t addrs[] =
  {
    0x48,
    0x49,
    0x4A,
    0x4B
  };


  for(
    const auto address :
    addrs
  )
  {
    Serial.printf(
      "[ADS] proba 0x%02X... ",
      address
    );


    if(ads.begin(address))
    {
      Serial.println("OK");


      ads.setGain(
        GAIN_ONE
      );


      gADSOK = true;

      gADSAddr =
        address;


      return true;
    }


    Serial.println(
      "brak"
    );
  }


  return false;
}


// ============================================================
// DS18B20
// ============================================================

bool validAquariumTemp(float value)
{
  return
    value != DEVICE_DISCONNECTED_C &&
    isfinite(value) &&
    value > 5.0f &&
    value < 40.0f;
}


void printDeviceAddress(
  DeviceAddress addr
)
{
  for(int i = 0; i < 8; i++)
  {
    if(addr[i] < 16)
    {
      Serial.print("0");
    }


    Serial.print(
      addr[i],
      HEX
    );
  }
}


void sortTempAddresses()
{
  if(foundTempSensors < 2)
  {
    return;
  }


  if(
    memcmp(
      tempAddress[0],
      tempAddress[1],
      8
    ) > 0
  )
  {
    uint8_t tmp[8];


    memcpy(
      tmp,
      tempAddress[0],
      8
    );


    memcpy(
      tempAddress[0],
      tempAddress[1],
      8
    );


    memcpy(
      tempAddress[1],
      tmp,
      8
    );
  }
}


void setupTemperatureSensors()
{
  tempBus.begin();


  int count =
    tempBus.getDeviceCount();


  foundTempSensors = 0;


  for(
    int i = 0;
    i < count &&
    foundTempSensors <
      TEMP_SENSOR_COUNT;
    i++
  )
  {
    if(
      tempBus.getAddress(
        tempAddress[
          foundTempSensors
        ],
        i
      )
    )
    {
      foundTempSensors++;
    }
  }


  sortTempAddresses();


  Serial.printf(
    "[TEMP] znalezionych: %d\n",
    count
  );


  for(
    int i = 0;
    i < foundTempSensors;
    i++
  )
  {
    tempBus.setResolution(
      tempAddress[i],
      11
    );


    Serial.printf(
      "[TEMP] sensor %d ROM: ",
      i + 1
    );


    printDeviceAddress(
      tempAddress[i]
    );


    Serial.println();
  }
}


void readTemperatures()
{
  tempBus.requestTemperatures();


  for(
    int i = 0;
    i < TEMP_SENSOR_COUNT;
    i++
  )
  {
    if(i >= foundTempSensors)
    {
      tempValid[i] = false;

      tempC[i] =
        DEVICE_DISCONNECTED_C;

      continue;
    }


    float value =
      tempBus.getTempC(
        tempAddress[i]
      );


    tempC[i] =
      value;


    tempValid[i] =
      validAquariumTemp(
        value
      );
  }


  /*
    Temperatura do kompensacji pH.
  */

  if(
    tempValid[0] &&
    tempValid[1]
  )
  {
    temperature =
      (
        tempC[0] +
        tempC[1]
      )
      / 2.0f;
  }
  else if(tempValid[0])
  {
    temperature =
      tempC[0];
  }
  else if(tempValid[1])
  {
    temperature =
      tempC[1];
  }
  else
  {
    temperature =
      25.0f;
  }


  Serial.printf(
    "[TEMP] T1 %.2f %s | T2 %.2f %s\n",

    tempC[0],
    tempValid[0]
      ? "OK"
      : "ERR",

    tempC[1],
    tempValid[1]
      ? "OK"
      : "ERR"
  );
}


// ============================================================
// PH
// ============================================================

void calculatePHAverage()
{
  if(!phPrimed)
  {
    phAverage = 0.0f;

    return;
  }


  float sum = 0.0f;


  for(
    int i = 0;
    i < PH_AVG_N;
    i++
  )
  {
    sum +=
      phReadings[i];
  }


  phAverage =
    sum /
    (float)PH_AVG_N;
}


bool validPH()
{
  return
    gADSOK &&
    phPrimed &&
    isfinite(phValue) &&
    isfinite(phAverage) &&
    phValue > 6.0f &&
    phValue < 10.0f &&
    phAverage > 6.0f &&
    phAverage < 10.0f;
}


void updatePHReading()
{
  if(!gADSOK)
  {
    return;
  }


  /*
    ZOSTAWIONE DOKLADNIE TAK JAK
    W TWOIM DZIALAJACYM PROGRAMIE.

    Nie zmieniamy przeliczenia,
    zeby nie rozwalic kalibracji.
  */

  voltage =
    ads.readADC_SingleEnded(1)
    / 10.0f;


  phValue =
    ph.readPH(
      voltage,
      temperature
    );


  if(!phPrimed)
  {
    for(
      int i = 0;
      i < PH_AVG_N;
      i++
    )
    {
      phReadings[i] =
        phValue;
    }


    phPrimed = true;

    phReadingsIndex = 0;
  }
  else
  {
    phReadings[
      phReadingsIndex
    ] = phValue;


    phReadingsIndex++;


    if(
      phReadingsIndex >=
      PH_AVG_N
    )
    {
      phReadingsIndex = 0;
    }
  }


  calculatePHAverage();


  Serial.printf(
    "[PH] raw %.3f | avg %.3f\n",
    phValue,
    phAverage
  );
}


// ============================================================
// KALKWASSER AUTOMATYKA
// ============================================================

void updateKalkControl()
{
  /*
    REEF-PI / WWW ustawilo AUTO OFF.
  */

  if(!kalkAutoEnabled)
  {
    setPumpOutput(0);

    return;
  }


  /*
    Bez wiarygodnego pH kalkwasser
    NIE MOZE pracowac.
  */

  if(!validPH())
  {
    kalkPHBlocked = true;

    setPumpOutput(0);

    return;
  }


  /*
    HARD STOP.

    Sprawdzamy zarowno srednia,
    jak i najnowszy odczyt.
  */

  if(
    phValue >= kalkPHHard ||
    phAverage >= kalkPHHard
  )
  {
    kalkPHBlocked = true;

    setPumpOutput(0);


    Serial.println(
      "[KALK] HARD STOP PH"
    );


    return;
  }


  /*
    Histereza:

      >= 8.45 -> blokada
      <= 8.35 -> odblokowanie
  */

  if(phAverage >= kalkPHStop)
  {
    kalkPHBlocked = true;
  }


  if(phAverage <= kalkPHRestart)
  {
    kalkPHBlocked = false;
  }


  if(
    kalkPHBlocked ||
    pumpSpeed == 0
  )
  {
    setPumpOutput(0);
  }
  else
  {
    setPumpOutput(
      pumpSpeed
    );
  }
}


// ============================================================
// GRZALKA - SAFETY OFF
// ============================================================

void heaterSafetyOff(
  const char* reason
)
{
  Serial.printf(
    "[HEATER] SAFETY OFF: %s\n",
    reason
  );


  bool needOff =
    heaterDesiredState ||
    !plugRelayKnown ||
    plugRelayState;


  if(
    needOff &&
    millis() -
      lastHeaterCommandMs >
      1500
  )
  {
    sendHeaterCommand(
      false
    );
  }
}


// ============================================================
// GRZALKA - TERMOSTAT
// ============================================================

void updateHeaterControl()
{
  /*
    AUTO OFF = grzalka bezwarunkowo OFF.
  */

  if(!heaterAutoEnabled)
  {
    heaterSafetyOff(
      "AUTO OFF"
    );

    return;
  }


  /*
    Wymagamy DWOCH sond.
  */

  if(
    !tempValid[0] ||
    !tempValid[1]
  )
  {
    heaterSafetyOff(
      "BLAD CZUJNIKA TEMP"
    );

    return;
  }


  /*
    Sprawdzamy zgodnosc sond.
  */

  float difference =
    fabs(
      tempC[0] -
      tempC[1]
    );


  if(
    difference >
    maxTempDifference
  )
  {
    heaterSafetyOff(
      "ROZNICA T1/T2"
    );

    return;
  }


  /*
    HARD STOP.

    Wystarczy, ze JEDNA sonda
    osiagnie limit.
  */

  float highest =
    max(
      tempC[0],
      tempC[1]
    );


  if(
    highest >=
    heaterHardMax
  )
  {
    heaterSafetyOff(
      "27C HARD MAX"
    );

    return;
  }


  float average =
    (
      tempC[0] +
      tempC[1]
    )
    / 2.0f;


  /*
    GRZALKA JEST WLACZONA.
  */

  if(heaterDesiredState)
  {
    if(
      average >=
      heaterOffTemp
    )
    {
      sendHeaterCommand(
        false
      );
    }


    return;
  }


  /*
    GRZALKA JEST WYLACZONA.
  */

  if(
    average <=
    heaterOnTemp
  )
  {
    bool switchAllowed =
      lastHeaterSwitchMs == 0 ||

      millis() -
      lastHeaterSwitchMs >=
      MIN_HEATER_SWITCH_MS;


    if(switchAllowed)
    {
      sendHeaterCommand(
        true
      );
    }
  }
}


// ============================================================
// SPRAWDZENIE FIZYCZNEGO STANU GNIAZDKA
// ============================================================

void enforcePlugState()
{
  if(!plugRelayKnown)
  {
    return;
  }


  /*
    Jezeli fizyczny stan SP301
    nie zgadza sie z tym czego chce ESP,
    ponawiamy komende.
  */

  if(
    plugRelayState !=
    heaterDesiredState
  )
  {
    if(
      millis() -
      lastHeaterCommandMs >
      3000
    )
    {
      Serial.println(
        "[HEATER] SP301 ma zly stan - poprawiam"
      );


      sendHeaterCommand(
        heaterDesiredState
      );
    }
  }
}


// ============================================================
// AUTO ON/OFF + NVS
// ============================================================

void setHeaterAuto(bool enabled)
{
  heaterAutoEnabled =
    enabled;


  saveAutomation();


  if(!enabled)
  {
    sendHeaterCommand(
      false
    );
  }
  else
  {
    updateHeaterControl();
  }
}


void setKalkAuto(bool enabled)
{
  kalkAutoEnabled =
    enabled;


  saveAutomation();


  if(!enabled)
  {
    setPumpOutput(0);
  }
  else
  {
    /*
      Nie wlaczamy pompy "na chama".

      Aktualne pH decyduje,
      czy pompa moze ruszyc.
    */

    updateKalkControl();
  }
}


// ============================================================
// TFT
// ============================================================

void drawTFT()
{
  myGLCD.fillScreen(
    TFT_BLACK
  );


  myGLCD.setTextColor(
    TFT_WHITE,
    TFT_BLACK
  );


  /*
    Co 5 sekund zmiana strony.
  */

  bool secondPage =
    (
      millis() /
      5000UL
    ) % 2;


  // ==========================================================
  // STRONA 1 - pH / KALK
  // ==========================================================

  if(!secondPage)
  {
    myGLCD.setTextSize(2);


    myGLCD.setCursor(
      5,
      5
    );


    myGLCD.print(
      "pH"
    );


    myGLCD.setTextSize(4);


    myGLCD.setCursor(
      40,
      4
    );


    if(validPH())
    {
      myGLCD.printf(
        "%.2f",
        phAverage
      );
    }
    else
    {
      myGLCD.print(
        "--"
      );
    }


    myGLCD.setTextSize(2);


    myGLCD.setCursor(
      5,
      55
    );


    myGLCD.print(
      "KALK: "
    );


    if(!kalkAutoEnabled)
    {
      myGLCD.print(
        "OFF"
      );
    }
    else if(kalkPHBlocked)
    {
      myGLCD.print(
        "PAUZA"
      );
    }
    else
    {
      myGLCD.print(
        pumpPhysicalRunning
        ? "ON"
        : "STOP"
      );
    }


    myGLCD.setCursor(
      5,
      82
    );


    myGLCD.printf(
      "P: %u st/s",
      pumpPhysicalSpeed
    );


    myGLCD.setCursor(
      5,
      106
    );


    myGLCD.printf(
      "dKH: %.2f",
      savedDKH
    );
  }

  // ==========================================================
  // STRONA 2 - TEMPERATURA / GRZALKA
  // ==========================================================

  else
  {
    myGLCD.setTextSize(2);


    myGLCD.setCursor(
      5,
      5
    );


    if(tempValid[0])
    {
      myGLCD.printf(
        "T1 %.2f",
        tempC[0]
      );
    }
    else
    {
      myGLCD.print(
        "T1 ERROR"
      );
    }


    myGLCD.setCursor(
      5,
      30
    );


    if(tempValid[1])
    {
      myGLCD.printf(
        "T2 %.2f",
        tempC[1]
      );
    }
    else
    {
      myGLCD.print(
        "T2 ERROR"
      );
    }


    myGLCD.setCursor(
      5,
      58
    );


    myGLCD.print(
      "HEAT: "
    );


    myGLCD.print(
      heaterDesiredState
      ? "ON"
      : "OFF"
    );


    myGLCD.setCursor(
      5,
      83
    );


    if(plugTelemetryValid)
    {
      myGLCD.printf(
        "%.1f W",
        plugPowerW
      );
    }
    else
    {
      myGLCD.print(
        "W: --"
      );
    }


    myGLCD.setTextSize(1);


    myGLCD.setCursor(
      5,
      110
    );


    if(plugTelemetryValid)
    {
      myGLCD.printf(
        "%.1f V  %.3f A",
        plugVoltageV,
        plugCurrentA
      );
    }
    else
    {
      myGLCD.print(
        "SP301 telemetry --"
      );
    }
  }
}


// ============================================================
// STATUS JSON
// ============================================================

void handleStatus()
{
  String json;

  json.reserve(800);


  json += "{";


  json += "\"temp1\":";
  json += String(
    tempC[0],
    2
  );


  json += ",\"temp1_ok\":";
  json +=
    tempValid[0]
    ? "true"
    : "false";


  json += ",\"temp2\":";
  json += String(
    tempC[1],
    2
  );


  json += ",\"temp2_ok\":";
  json +=
    tempValid[1]
    ? "true"
    : "false";


  json += ",\"ph\":";
  json += String(
    phAverage,
    3
  );


  json += ",\"ph_raw\":";
  json += String(
    phValue,
    3
  );


  json += ",\"ph_ok\":";
  json +=
    validPH()
    ? "true"
    : "false";


  json += ",\"dkh\":";
  json += String(
    savedDKH,
    2
  );


  json += ",\"heater_auto\":";
  json +=
    heaterAutoEnabled
    ? "true"
    : "false";


  json += ",\"heater_requested\":";
  json +=
    heaterDesiredState
    ? "true"
    : "false";


  json += ",\"sp301_known\":";
  json +=
    plugRelayKnown
    ? "true"
    : "false";


  json += ",\"sp301_relay\":";
  json +=
    plugRelayState
    ? "true"
    : "false";


  json += ",\"watts\":";
  json += String(
    plugPowerW,
    1
  );


  json += ",\"amps\":";
  json += String(
    plugCurrentA,
    3
  );


  json += ",\"volts\":";
  json += String(
    plugVoltageV,
    1
  );


  json += ",\"kalk_auto\":";
  json +=
    kalkAutoEnabled
    ? "true"
    : "false";


  json += ",\"kalk_ph_blocked\":";
  json +=
    kalkPHBlocked
    ? "true"
    : "false";


  json += ",\"pump_running\":";
  json +=
    pumpPhysicalRunning
    ? "true"
    : "false";


  json += ",\"pump_speed\":";
  json += String(
    pumpPhysicalSpeed
  );


  json += ",\"wifi_rssi\":";
  json += String(
    WiFi.RSSI()
  );


  json += "}";


  server.send(
    200,
    "application/json",
    json
  );
}


// ============================================================
// STRONA WWW
// ============================================================

void handleRoot()
{
  String html;

  html.reserve(7500);


  html += F(
    "<!doctype html>"
    "<html lang='pl'>"

    "<head>"

    "<meta charset='utf-8'>"

    "<meta name='viewport' "
    "content='width=device-width,initial-scale=1'>"

    "<meta http-equiv='refresh' content='5'>"

    "<title>Akwarium ESP32</title>"

    "<style>"

    "body{"
    "font-family:system-ui,Arial,sans-serif;"
    "margin:0;"
    "padding:16px;"
    "background:#0b0c10;"
    "color:#eee"
    "}"

    ".card{"
    "max-width:600px;"
    "margin:0 auto 14px;"
    "padding:18px;"
    "background:#17191f;"
    "border-radius:14px"
    "}"

    "h1,h2{margin-top:0}"

    ".row{"
    "display:flex;"
    "justify-content:space-between;"
    "padding:6px 0"
    "}"

    ".muted{"
    "color:#aaa;"
    "font-size:14px"
    "}"

    "input{"
    "box-sizing:border-box;"
    "width:100%;"
    "padding:10px;"
    "margin:5px 0 12px;"
    "font-size:17px;"
    "background:#0f1116;"
    "color:#fff;"
    "border:1px solid #444;"
    "border-radius:8px"
    "}"

    "button{"
    "padding:11px 14px;"
    "margin:4px;"
    "border:0;"
    "border-radius:8px;"
    "font-weight:700"
    "}"

    "a{color:#9ac1ff}"

    "</style>"

    "</head>"

    "<body>"

    "<div class='card'>"

    "<h1>Akwarium ESP32</h1>"
  );


  // ==========================================================
  // pH / dKH
  // ==========================================================

  html +=
    "<div class='row'>"
    "<span>pH</span>"
    "<b>";


  if(validPH())
  {
    html +=
      String(
        phAverage,
        2
      );
  }
  else
  {
    html +=
      "BRAK";
  }


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>dKH</span>"
    "<b>";


  html +=
    String(
      savedDKH,
      2
    );


  html +=
    "</b></div>"

    "</div>";


  // ==========================================================
  // GRZALKA
  // ==========================================================

  html += F(
    "<div class='card'>"

    "<h2>Temperatura / grzalka</h2>"
  );


  html +=
    "<div class='row'>"
    "<span>T1</span>"
    "<b>";


  if(tempValid[0])
  {
    html +=
      String(tempC[0], 2)
      + " C";
  }
  else
  {
    html += "ERROR";
  }


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>T2</span>"
    "<b>";


  if(tempValid[1])
  {
    html +=
      String(tempC[1], 2)
      + " C";
  }
  else
  {
    html += "ERROR";
  }


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>Grzanie AUTO</span>"
    "<b>";


  html +=
    heaterAutoEnabled
    ? "ON"
    : "OFF";


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>SP301</span>"
    "<b>";


  if(plugRelayKnown)
  {
    html +=
      plugRelayState
      ? "ON"
      : "OFF";
  }
  else
  {
    html += "?";
  }


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>Moc</span><b>";


  html +=
    String(
      plugPowerW,
      1
    );


  html +=
    " W</b></div>";


  html +=
    "<div class='row'>"
    "<span>Prad</span><b>";


  html +=
    String(
      plugCurrentA,
      3
    );


  html +=
    " A</b></div>";


  html +=
    "<div class='row'>"
    "<span>Napiecie</span><b>";


  html +=
    String(
      plugVoltageV,
      1
    );


  html +=
    " V</b></div>";


  html += F(
    "<form method='POST' action='/heater/on'>"
    "<button>GRZANIE AUTO ON</button>"
    "</form>"

    "<form method='POST' action='/heater/off'>"
    "<button>GRZANIE AUTO OFF</button>"
    "</form>"

    "</div>"
  );


  // ==========================================================
  // KALK
  // ==========================================================

  html += F(
    "<div class='card'>"

    "<h2>Kalkwasser</h2>"
  );


  html +=
    "<div class='row'>"
    "<span>AUTO</span>"
    "<b>";


  html +=
    kalkAutoEnabled
    ? "ON"
    : "OFF";


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>Blokada pH</span>"
    "<b>";


  html +=
    kalkPHBlocked
    ? "TAK"
    : "NIE";


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>Pompa</span>"
    "<b>";


  html +=
    pumpPhysicalRunning
    ? "PRACUJE"
    : "STOP";


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>Predkosc</span><b>";


  html +=
    String(
      pumpSpeed
    );


  html +=
    " krokow/s</b></div>";


  html +=
    "<div class='row'>"
    "<span>pH restart</span><b>";


  html +=
    String(
      kalkPHRestart,
      2
    );


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>pH stop</span><b>";


  html +=
    String(
      kalkPHStop,
      2
    );


  html +=
    "</b></div>";


  html +=
    "<div class='row'>"
    "<span>pH HARD</span><b>";


  html +=
    String(
      kalkPHHard,
      2
    );


  html +=
    "</b></div>";


  html += F(
    "<form method='POST' action='/kalk/on'>"
    "<button>KALK AUTO ON</button>"
    "</form>"

    "<form method='POST' action='/kalk/off'>"
    "<button>KALK AUTO OFF</button>"
    "</form>"

    "</div>"
  );


  // ==========================================================
  // USTAWIENIA
  // ==========================================================

  html += F(
    "<div class='card'>"

    "<h2>Ustawienia</h2>"

    "<form method='POST' action='/save'>"

    "<label>"
    "Predkosc pompy 0-2000 krokow/s"
    "</label>"

    "<input "
    "name='speed' "
    "type='number' "
    "min='0' "
    "max='2000' "
    "step='1' "
    "required "
    "value='"
  );


  html +=
    String(
      pumpSpeed
    );


  html += F(
    "'>"

    "<label>dKH</label>"

    "<input "
    "name='dkh' "
    "type='number' "
    "min='0' "
    "max='30' "
    "step='0.01' "
    "required "
    "value='"
  );


  html +=
    String(
      savedDKH,
      2
    );


  html += F(
    "'>"

    "<button type='submit'>"
    "Zapisz"
    "</button>"

    "</form>"

    "<p class='muted'>"
    "Limit ml/dobe = 0. "
    "Kalibracje pompy dodamy po pomiarze wydajnosci."
    "</p>"

    "</div>"

    "<div class='card muted'>"

    "<a href='/status'>JSON</a> | "
    "<a href='/wifi'>Wi-Fi</a> | "
    "<a href='/reboot'>Restart ESP</a>"

    "</div>"

    "</body>"
    "</html>"
  );


  server.send(
    200,
    "text/html; charset=utf-8",
    html
  );
}


// ============================================================
// ZAPIS SPEED / dKH
// ============================================================

void handleSaveSettings()
{
  if(
    !server.hasArg("speed") ||
    !server.hasArg("dkh")
  )
  {
    server.send(
      400,
      "text/plain",
      "Brak danych"
    );

    return;
  }


  long requestedSpeed =
    server.arg(
      "speed"
    ).toInt();


  float requestedDKH =
    server.arg(
      "dkh"
    ).toFloat();


  if(
    requestedSpeed < 0 ||
    requestedSpeed >
      MAX_PUMP_SPEED
  )
  {
    server.send(
      400,
      "text/plain",
      "Predkosc 0-2000"
    );

    return;
  }


  if(
    !isfinite(requestedDKH) ||
    requestedDKH < 0 ||
    requestedDKH > 30
  )
  {
    server.send(
      400,
      "text/plain",
      "dKH 0-30"
    );

    return;
  }


  pumpSpeed =
    (uint16_t)
    requestedSpeed;


  savedDKH =
    requestedDKH;


  savePumpSpeed();
  saveDKH();


  updateKalkControl();


  server.sendHeader(
    "Location",
    "/",
    true
  );


  server.send(
    303,
    "text/plain",
    ""
  );
}


// ============================================================
// REEF-PI ANALOG INPUTS
// ============================================================

void sendAnalogInput(int id)
{
  switch(id)
  {
    // TEMP 1
    case 0:

      if(!tempValid[0])
      {
        server.send(
          503,
          "text/plain",
          "TEMP1 ERROR"
        );

        return;
      }


      server.send(
        200,
        "text/plain",
        String(
          tempC[0],
          2
        )
      );

      return;


    // TEMP 2
    case 1:

      if(!tempValid[1])
      {
        server.send(
          503,
          "text/plain",
          "TEMP2 ERROR"
        );

        return;
      }


      server.send(
        200,
        "text/plain",
        String(
          tempC[1],
          2
        )
      );

      return;


    // pH
    case 2:

      if(!validPH())
      {
        server.send(
          503,
          "text/plain",
          "PH ERROR"
        );

        return;
      }


      server.send(
        200,
        "text/plain",
        String(
          phAverage,
          3
        )
      );

      return;


    // W
    case 3:

      if(!plugTelemetryValid)
      {
        server.send(
          503,
          "text/plain",
          "POWER ERROR"
        );

        return;
      }


      server.send(
        200,
        "text/plain",
        String(
          plugPowerW,
          1
        )
      );

      return;


    // A
    case 4:

      if(!plugTelemetryValid)
      {
        server.send(
          503,
          "text/plain",
          "CURRENT ERROR"
        );

        return;
      }


      server.send(
        200,
        "text/plain",
        String(
          plugCurrentA,
          3
        )
      );

      return;


    // V
    case 5:

      if(!plugTelemetryValid)
      {
        server.send(
          503,
          "text/plain",
          "VOLTAGE ERROR"
        );

        return;
      }


      server.send(
        200,
        "text/plain",
        String(
          plugVoltageV,
          1
        )
      );

      return;


    // dKH
    case 6:

      server.send(
        200,
        "text/plain",
        String(
          savedDKH,
          2
        )
      );

      return;
  }


  server.send(
    404,
    "text/plain",
    "invalid analog input"
  );
}


// ============================================================
// WIFI WWW
// ============================================================

void handleWiFiGet()
{
  String html = F(
    "<!doctype html>"

    "<html lang='pl'>"

    "<head>"

    "<meta charset='utf-8'>"

    "<meta name='viewport' "
    "content='width=device-width,initial-scale=1'>"

    "<title>Wi-Fi</title>"

    "</head>"

    "<body style='font-family:system-ui;padding:16px'>"

    "<h2>Konfiguracja Wi-Fi</h2>"

    "<form method='POST'>"

    "SSID:<br>"

    "<input name='ssid' required>"

    "<br>"

    "Haslo:<br>"

    "<input "
    "name='pass' "
    "type='password'>"

    "<br><br>"

    "<button>"
    "Zapisz i zrestartuj"
    "</button>"

    "</form>"

    "<p>"
    "<a href='/'>Wroc</a>"
    "</p>"

    "</body>"

    "</html>"
  );


  server.send(
    200,
    "text/html; charset=utf-8",
    html
  );
}


void handleWiFiPost()
{
  if(
    !server.hasArg("ssid") ||
    !server.hasArg("pass")
  )
  {
    server.send(
      400,
      "text/plain",
      "Brak SSID lub hasla"
    );

    return;
  }


  prefs.begin(
    "wifi",
    false
  );


  prefs.putString(
    "ssid",
    server.arg("ssid")
  );


  prefs.putString(
    "pass",
    server.arg("pass")
  );


  prefs.end();


  server.send(
    200,
    "text/html",
    "Zapisano. Restart..."
  );


  delay(1500);

  ESP.restart();
}


// ============================================================
// WIFI
// ============================================================

void setupWiFi()
{
  prefs.begin(
    "wifi",
    true
  );


  String ssid =
    prefs.getString(
      "ssid",
      ""
    );


  String pass =
    prefs.getString(
      "pass",
      ""
    );


  prefs.end();


  if(
    !ssid.length() &&
    DEFAULT_WIFI_SSID &&
    DEFAULT_WIFI_SSID[0]
  )
  {
    ssid =
      DEFAULT_WIFI_SSID;


    pass =
      DEFAULT_WIFI_PASS
      ? DEFAULT_WIFI_PASS
      : "";
  }


  WiFi.persistent(false);

  WiFi.setSleep(false);

  WiFi.mode(
    WIFI_AP_STA
  );


  IPAddress apIP(
    192,
    168,
    4,
    1
  );


  IPAddress apGW(
    192,
    168,
    4,
    1
  );


  IPAddress apNM(
    255,
    255,
    255,
    0
  );


  WiFi.softAPConfig(
    apIP,
    apGW,
    apNM
  );


  bool apOK =
    WiFi.softAP(
      AP_SSID,
      AP_PASSWORD
    );


  Serial.printf(
    "[WIFI] AP: %s\n",
    apOK
      ? "OK"
      : "FAIL"
  );


  Serial.print(
    "[WIFI] AP IP: "
  );


  Serial.println(
    WiFi.softAPIP()
  );


  if(ssid.length())
  {
    Serial.print(
      "[WIFI] laczenie: "
    );


    Serial.println(
      ssid
    );


    WiFi.begin(
      ssid.c_str(),
      pass.c_str()
    );


    uint32_t startTime =
      millis();


    while(
      WiFi.status() !=
        WL_CONNECTED &&

      millis() -
        startTime <
        15000
    )
    {
      delay(250);

      Serial.print(".");
    }


    Serial.println();


    if(
      WiFi.status() ==
      WL_CONNECTED
    )
    {
      Serial.print(
        "[WIFI] STA IP: "
      );


      Serial.println(
        WiFi.localIP()
      );
    }
    else
    {
      Serial.println(
        "[WIFI] brak STA - pozostaje AP"
      );
    }
  }
}


// ============================================================
// OTA
// ============================================================

void setupOTA()
{
  ArduinoOTA.setHostname(
    OTA_HOST
  );


  if(
    String(
      OTA_PASS
    ).length()
  )
  {
    ArduinoOTA.setPassword(
      OTA_PASS
    );
  }


  ArduinoOTA.onStart(
    []()
    {
      Serial.println(
        "[OTA] start"
      );
    }
  );


  ArduinoOTA.onEnd(
    []()
    {
      Serial.println(
        "\n[OTA] end"
      );
    }
  );


  ArduinoOTA.onProgress(
    [](
      unsigned int progress,
      unsigned int total
    )
    {
      unsigned int percent =
        total == 0
        ? 0
        : (
            progress *
            100U
          ) / total;


      Serial.printf(
        "[OTA] %u%%\r",
        percent
      );
    }
  );


  ArduinoOTA.onError(
    [](ota_error_t error)
    {
      Serial.printf(
        "[OTA] ERROR %u\n",
        error
      );
    }
  );


  ArduinoOTA.begin();


  Serial.println(
    "[OTA] gotowe"
  );
}


// ============================================================
// HTTP
// ============================================================

void setupWeb()
{
  // ----------------------------------------------------------
  // WWW
  // ----------------------------------------------------------

  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );


  server.on(
    "/status",
    HTTP_GET,
    handleStatus
  );


  server.on(
    "/save",
    HTTP_POST,
    handleSaveSettings
  );


  // ----------------------------------------------------------
  // LOCAL HEATER
  // ----------------------------------------------------------

  server.on(
    "/heater/on",
    HTTP_POST,
    []()
    {
      setHeaterAuto(
        true
      );


      server.sendHeader(
        "Location",
        "/",
        true
      );


      server.send(
        303,
        "text/plain",
        ""
      );
    }
  );


  server.on(
    "/heater/off",
    HTTP_POST,
    []()
    {
      setHeaterAuto(
        false
      );


      server.sendHeader(
        "Location",
        "/",
        true
      );


      server.send(
        303,
        "text/plain",
        ""
      );
    }
  );


  // ----------------------------------------------------------
  // LOCAL KALK
  // ----------------------------------------------------------

  server.on(
    "/kalk/on",
    HTTP_POST,
    []()
    {
      setKalkAuto(
        true
      );


      server.sendHeader(
        "Location",
        "/",
        true
      );


      server.send(
        303,
        "text/plain",
        ""
      );
    }
  );


  server.on(
    "/kalk/off",
    HTTP_POST,
    []()
    {
      setKalkAuto(
        false
      );


      server.sendHeader(
        "Location",
        "/",
        true
      );


      server.send(
        303,
        "text/plain",
        ""
      );
    }
  );


  // ==========================================================
  // REEF-PI OUTLET 0
  // GRZANIE AUTO
  // ==========================================================

  server.on(
    "/outlets/0/on",
    HTTP_POST,
    []()
    {
      setHeaterAuto(
        true
      );


      server.send(
        200,
        "text/plain",
        "high"
      );
    }
  );


  server.on(
    "/outlets/0/off",
    HTTP_POST,
    []()
    {
      setHeaterAuto(
        false
      );


      server.send(
        200,
        "text/plain",
        "low"
      );
    }
  );


  // ==========================================================
  // REEF-PI OUTLET 1
  // KALK AUTO
  // ==========================================================

  server.on(
    "/outlets/1/on",
    HTTP_POST,
    []()
    {
      setKalkAuto(
        true
      );


      server.send(
        200,
        "text/plain",
        "high"
      );
    }
  );


  server.on(
    "/outlets/1/off",
    HTTP_POST,
    []()
    {
      setKalkAuto(
        false
      );


      server.send(
        200,
        "text/plain",
        "low"
      );
    }
  );


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  server.on(
    "/wifi",
    HTTP_GET,
    handleWiFiGet
  );


  server.on(
    "/wifi",
    HTTP_POST,
    handleWiFiPost
  );


  // ----------------------------------------------------------
  // PING
  // ----------------------------------------------------------

  server.on(
    "/ping",
    HTTP_GET,
    []()
    {
      server.send(
        200,
        "text/plain",
        "pong"
      );
    }
  );


  server.on(
    "/health",
    HTTP_GET,
    []()
    {
      server.send(
        200,
        "text/plain",
        "OK"
      );
    }
  );


  // ----------------------------------------------------------
  // RESTART
  // ----------------------------------------------------------

  server.on(
    "/reboot",
    HTTP_GET,
    []()
    {
      server.send(
        200,
        "text/plain",
        "Restart..."
      );


      delay(500);

      ESP.restart();
    }
  );


  // ==========================================================
  // REEF-PI ANALOG INPUTS
  // ==========================================================

  server.onNotFound(
    []()
    {
      String uri =
        server.uri();


      if(
        server.method() ==
          HTTP_GET &&

        uri.startsWith(
          "/analog_inputs/"
        )
      )
      {
        int id = -1;


        if(
          sscanf(
            uri.c_str(),
            "/analog_inputs/%d",
            &id
          ) == 1
        )
        {
          sendAnalogInput(id);

          return;
        }
      }


      server.send(
        404,
        "text/plain",
        String(
          "404: "
        ) + uri
      );
    }
  );


  server.begin();


  Serial.println(
    "[HTTP] serwer start"
  );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(300);


  Serial.println();

  Serial.println(
    "====================================="
  );

  Serial.println(
    " ESP32 AQUARIUM CONTROLLER"
  );

  Serial.println(
    "====================================="
  );


  EEPROM.begin(32);


  // ==========================================================
  // TFT
  // ==========================================================

  myGLCD.init();

  myGLCD.setRotation(3);

  myGLCD.fillScreen(
    TFT_BLACK
  );


  // ==========================================================
  // STARE GPIO
  // ==========================================================

  pinMode(
    PIN_AUX,
    OUTPUT
  );


  pinMode(
    PIN_23,
    OUTPUT
  );


  digitalWrite(
    PIN_AUX,
    HIGH
  );


  digitalWrite(
    PIN_23,
    HIGH
  );


  // ==========================================================
  // USTAWIENIA
  // ==========================================================

  loadSettings();


  // ==========================================================
  // POMPA
  // ==========================================================

  setupPump();


  // Bezpieczny start.
  setPumpOutput(0);


  // ==========================================================
  // PH
  // ==========================================================

  ph.begin();


  if(!initI2CAndADS())
  {
    Serial.println(
      "[ADS] NIE WYKRYTY"
    );
  }


  // ==========================================================
  // DS18B20 GPIO16
  // ==========================================================

  setupTemperatureSensors();


  // ==========================================================
  // WIFI
  // ==========================================================

  setupWiFi();


  /*
    NTP jest opcjonalne.

    SP301 jest sterowane lokalnie,
    a nie przez chmure.
  */

  configTime(
    0,
    0,
    "pool.ntp.org",
    "time.google.com"
  );


  // ==========================================================
  // HTTP / OTA
  // ==========================================================

  setupWeb();

  setupOTA();


  // ==========================================================
  // BEZPIECZNY START GRZALKI
  // ==========================================================

  if(
    WiFi.status() ==
    WL_CONNECTED
  )
  {
    /*
      Najpierw wymuszamy OFF.

      Dopiero po odczycie temperatur
      automat moze wlaczyc grzalke.
    */

    sendHeaterCommand(
      false
    );


    delay(200);


    readPlugTelemetry();
  }


  // ==========================================================
  // PIERWSZE POMIARY
  // ==========================================================

  readTemperatures();

  updatePHReading();


  // ==========================================================
  // AUTOMATYKA
  // ==========================================================

  updateHeaterControl();

  updateKalkControl();


  // ==========================================================
  // TFT
  // ==========================================================

  drawTFT();


  // ==========================================================
  // INFO
  // ==========================================================

  Serial.println();

  Serial.printf(
    "[CONFIG] GPIO DS18B20: %d\n",
    ONE_WIRE_PIN
  );


  Serial.printf(
    "[CONFIG] pompa: %u krokow/s\n",
    pumpSpeed
  );


  Serial.printf(
    "[CONFIG] dKH: %.2f\n",
    savedDKH
  );


  Serial.printf(
    "[CONFIG] heater AUTO: %s\n",
    heaterAutoEnabled
    ? "ON"
    : "OFF"
  );


  Serial.printf(
    "[CONFIG] kalk AUTO: %s\n",
    kalkAutoEnabled
    ? "ON"
    : "OFF"
  );


  Serial.printf(
    "[CONFIG] heater ON <= %.2f\n",
    heaterOnTemp
  );


  Serial.printf(
    "[CONFIG] heater OFF >= %.2f\n",
    heaterOffTemp
  );


  Serial.printf(
    "[CONFIG] heater HARD >= %.2f\n",
    heaterHardMax
  );


  Serial.printf(
    "[CONFIG] kalk restart <= %.2f\n",
    kalkPHRestart
  );


  Serial.printf(
    "[CONFIG] kalk stop >= %.2f\n",
    kalkPHStop
  );


  Serial.printf(
    "[CONFIG] kalk HARD >= %.2f\n",
    kalkPHHard
  );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  unsigned long now =
    millis();


  // ==========================================================
  // HTTP
  // ==========================================================

  server.handleClient();


  // ==========================================================
  // OTA
  // ==========================================================

  ArduinoOTA.handle();


  // ==========================================================
  // pH + KALK
  // ==========================================================

  if(
    now -
    lastPHUpdateTime >=
    PH_UPDATE_INTERVAL
  )
  {
    lastPHUpdateTime =
      now;


    updatePHReading();


    updateKalkControl();
  }


  // ==========================================================
  // TEMPERATURA + GRZALKA
  // ==========================================================

  if(
    now -
    lastTempUpdateTime >=
    TEMP_UPDATE_INTERVAL
  )
  {
    lastTempUpdateTime =
      now;


    readTemperatures();


    updateHeaterControl();
  }


  // ==========================================================
  // SP301 W / A / V
  // ==========================================================

  if(
    now -
    lastPowerUpdateTime >=
    POWER_UPDATE_INTERVAL
  )
  {
    lastPowerUpdateTime =
      now;


    if(
      WiFi.status() ==
      WL_CONNECTED
    )
    {
      if(readPlugTelemetry())
      {
        enforcePlugState();
      }
    }
  }


  // ==========================================================
  // TFT
  // ==========================================================

  if(
    now -
    lastTFTUpdate >=
    TFT_UPDATE_INTERVAL
  )
  {
    lastTFTUpdate =
      now;


    drawTFT();
  }


  // ==========================================================
  // WIFI RECONNECT
  // ==========================================================

  if(
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    static unsigned long
      lastReconnect = 0;


    if(
      now -
      lastReconnect >
      5000
    )
    {
      lastReconnect =
        now;


      Serial.println(
        "[WIFI] reconnect..."
      );


      WiFi.reconnect();
    }
  }


  // ==========================================================
  // KALIBRACJA pH
  //
  // Serial Monitor:
  //
  // enterph
  // calph
  // exitph
  //
  // ==========================================================

  ph.calibration(
    voltage,
    temperature
  );


  delay(2);
}
