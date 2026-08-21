# ESP32 Aquarium Controller

Sterownik akwarium morskiego oparty na ESP32.

Projekt łączy w jednym kontrolerze:

- pomiar pH,
- sterowanie pompą perystaltyczną do kalkwassera,
- dwa czujniki temperatury DS18B20,
- autonomiczne sterowanie grzałką,
- lokalne sterowanie gniazdkiem Setti+ SP301 przez protokół Tuya 3.3,
- odczyt mocy, prądu i napięcia z SP301,
- wyświetlacz TFT,
- panel WWW,
- OTA,
- zapis ustawień w pamięci NVS,
- API zgodne z driverem ESP32 w reef-pi.

---

# 1. Założenia projektu

Najważniejsza zasada:

**ESP32 działa autonomicznie.**

reef-pi nie jest wymagany do podstawowego działania akwarium.

Jeżeli komputer z reef-pi zostanie wyłączony lub straci połączenie z ESP32:

- regulacja temperatury nadal działa,
- zabezpieczenia temperatury nadal działają,
- pomiar pH nadal działa,
- sterowanie kalkwasserem nadal działa,
- ustawienia zapisane w ESP32 pozostają aktywne.

reef-pi służy głównie do:

- wizualizacji,
- wykresów,
- historii pomiarów,
- harmonogramów,
- włączania i wyłączania trybów AUTO.

---

# 2. Sprzęt

## ESP32

Płytka:

```text
ESP32-WROOM-32U

W Arduino IDE:

Board:
ESP32 Dev Module
3. Podłączenie pinów
ADS1115 / sonda pH
ESP32        ADS1115

GPIO21  ->   SDA
GPIO22  ->   SCL
3.3V    ->   VCC
GND     ->   GND

Sygnał modułu pH podłączony jest do:

ADS1115 A1
Pompa perystaltyczna kalkwasser

Sterownik STEP/DIR:

GPIO19 -> STEP
GPIO18 -> DIR

Pompa pracuje tylko w jednym kierunku.

Jeżeli kierunek jest odwrotny, należy zmienić:

const uint8_t PUMP_FORWARD_LEVEL = HIGH;

na:

const uint8_t PUMP_FORWARD_LEVEL = LOW;
4. Czujniki temperatury

Używane są dwa DS18B20 na jednej magistrali OneWire.

GPIO16 -> DATA obu DS18B20

Podłączenie:

                        +---- VCC DS18B20 #1
3.3V -------------------+---- VCC DS18B20 #2
  |
  |
 około 5 kΩ
  |
  +------------------------- GPIO16
                              |
                              +---- DATA DS18B20 #1
                              |
                              +---- DATA DS18B20 #2


GND ------------------------- GND DS18B20 #1
  |
  +-------------------------- GND DS18B20 #2

Można użyć:

1 x 4.7 kΩ

lub:

2 x 10 kΩ równolegle

co daje około:

5 kΩ

Czujniki pracują w trybie 3-przewodowym.

Nie używać parasite power.

5. Regulacja temperatury

Ustawienia startowe:

temperatura zadana: około 25.0°C

grzałka ON:
<= 24.80°C

grzałka OFF:
>= 25.20°C

awaryjny HARD STOP:
>= 27.00°C

Sterownik używa dwóch czujników temperatury.

Normalna regulacja korzysta ze średniej:

(T1 + T2) / 2
Zabezpieczenia

Grzałka zostanie wyłączona jeżeli:

jeden z DS18B20 nie działa,
oba czujniki nie są dostępne,
różnica pomiędzy T1 i T2 przekracza 1°C,
dowolny czujnik osiągnie 27°C,
tryb grzania AUTO zostanie wyłączony.

Przykład:

T1 = 25.10°C
T2 = 25.18°C

różnica = 0.08°C

OK

Przykład błędu:

T1 = 25.0°C
T2 = 27.0°C

różnica = 2.0°C

GRZAŁKA OFF
6. Setti+ SP301

Grzałka podłączona jest do inteligentnego gniazdka:

Setti+ SP301

ESP32 komunikuje się z gniazdkiem bezpośrednio przez sieć LAN.

Nie jest wymagane wysyłanie poleceń przez serwer Tuya.

Schemat:

ESP32
  |
  | Wi-Fi LAN
  |
Router / Access Point
  |
  |
SP301
  |
  |
Grzałka

Po uzyskaniu local_key do działania lokalnego nie jest potrzebna chmura Tuya.

Router / Access Point nadal jest wymagany do komunikacji:

ESP32 <-> SP301
7. Telemetria SP301

ESP32 odczytuje z SP301:

DPS 1  -> stan przekaźnika
DPS 18 -> prąd
DPS 19 -> moc
DPS 20 -> napięcie

Przeliczenia:

DPS 18:
mA / 1000 = A

DPS 19:
wartość / 10 = W

DPS 20:
wartość / 10 = V

Przykład:

SP301 ON
198.4 W
0.873 A
227.3 V
8. Dane Tuya

W kodzie należy ustawić:

const char* TUYA_IP = "<IP_SP301>";

const char* DEVICE_ID = "<DEVICE_ID>";

const char* LOCAL_KEY = "<16_ZNAKOW_LOCAL_KEY>";

LOCAL_KEY musi mieć dokładnie:

16 znaków

Nie należy wpisywać w to miejsce:

Tuya API Key,
Access Secret,
Project Code.

Potrzebny jest local_key konkretnego urządzenia.

9. Pomiar pH

Pomiar wykonywany jest przez:

sonda pH
  |
moduł pH
  |
ADS1115
  |
ESP32

ADS1115 pracuje przez I2C:

SDA = GPIO21
SCL = GPIO22

Kod zachowuje dotychczasowe przeliczenie:

voltage = ads.readADC_SingleEnded(1) / 10.0f;

phValue = ph.readPH(voltage, temperature);

Pomiar pH jest dodatkowo wygładzany średnią z:

30 próbek
10. Kompensacja temperatury pH

Jeżeli oba DS18B20 działają:

temperatura pH = średnia T1 i T2

Jeżeli działa tylko jeden:

temperatura pH = działający czujnik

Jeżeli nie działa żaden:

temperatura pH = 25°C
11. Kalibracja sondy pH

Kalibracja wykonywana jest przez Serial Monitor.

Ustaw:

115200 baud

Komendy:

enterph
calph
exitph

Czyli:

enterph

wejście w kalibrację,

następnie:

calph

kalibracja,

na końcu:

exitph

zapis i wyjście.

12. Kalkwasser

Pompa kalkwassera może pracować przez całą dobę.

Aktualnie głównym ograniczeniem bezpieczeństwa jest pH.

Ustawienia startowe:

pH <= 8.35
dozowanie może zostać wznowione

pH >= 8.45
pompa STOP

pH >= 8.60
HARD STOP

Działa tu histereza:

8.35 -------- 8.45
 START          STOP

Dzięki temu pompa nie włącza się i nie wyłącza bez przerwy przy jednej wartości pH.

13. Bezpieczeństwo kalkwassera

Pompa kalkwassera zostanie zatrzymana jeżeli:

pH jest nieprawidłowe,
ADS1115 nie działa,
pH przekroczy wartość STOP,
pH przekroczy HARD STOP,
KALK AUTO zostanie wyłączony,
ustawiona prędkość pompy wynosi 0.

Po restarcie ESP32 pompa nie powinna od razu rozpocząć dozowania.

Najpierw musi zostać uzyskany poprawny odczyt pH.

14. Kalibracja wydajności pompy

Na razie sterownik pracuje w jednostce:

kroki / sekundę

Nie znamy jeszcze zależności:

liczba kroków -> ml kalkwassera

Dlatego limit:

ml / dobę

jest obecnie wyłączony.

Docelowo należy wykonać test.

Przykład:

1. Uruchomić pompę na określoną liczbę kroków.
2. Zebrać ciecz do cylindra / strzykawki.
3. Zmierzyć ilość ml.
4. Obliczyć ml/krok.

Przykład:

10000 kroków = 25 ml

wtedy:

25 / 10000 = 0.0025 ml/krok

Po wykonaniu tej kalibracji można dodać:

ml/min,
ml/h,
ml/dobę,
dzienny limit bezpieczeństwa,
zaplanowaną dawkę kalkwassera.
15. dKH

Wartość dKH jest obecnie wpisywana ręcznie.

Zakres:

0 - 30 dKH

Wartość jest zapisywana w NVS i nie znika po restarcie ESP32.

Docelowo może służyć do oceny konsumpcji alkaliczności i ustalania dawki kalkwassera.

16. Pamięć NVS

ESP32 zapisuje ustawienia w pamięci nieulotnej.

Zapisywane są między innymi:

pumpSpeed
dKH
heater AUTO
kalkwasser AUTO
Wi-Fi

Po restarcie ESP32 wartości są ponownie odczytywane.

17. Panel WWW

Po uruchomieniu ESP32 należy znaleźć jego adres IP w routerze lub Serial Monitorze.

Przykład:

[WIFI] STA IP: 192.168.0.xxx

Następnie otworzyć:

http://IP_ESP32/

Panel pokazuje:

pH,
dKH,
temperaturę T1,
temperaturę T2,
stan grzania AUTO,
stan SP301,
moc W,
prąd A,
napięcie V,
stan kalkwassera,
blokadę pH,
prędkość pompy.
18. Status JSON

Pełny status sterownika:

http://IP_ESP32/status

Przykładowa odpowiedź:

{
  "temp1": 26.25,
  "temp1_ok": true,
  "temp2": 26.12,
  "temp2_ok": true,
  "ph": 8.086,
  "ph_raw": 8.082,
  "ph_ok": true,
  "dkh": 9.20,
  "heater_auto": true,
  "heater_requested": false,
  "sp301_known": true,
  "sp301_relay": false,
  "watts": 0.0,
  "amps": 0.000,
  "volts": 231.8,
  "kalk_auto": true,
  "kalk_ph_blocked": false,
  "pump_running": false,
  "pump_speed": 0,
  "wifi_rssi": -59
}
19. Test połączenia
http://IP_ESP32/ping

Powinno zwrócić:

pong

Dodatkowo:

http://IP_ESP32/health

powinno zwrócić:

OK
20. API reef-pi

ESP32 udostępnia API kompatybilne z driverem esp32 w reef-pi.

Sterowanie grzałką

Włączenie trybu AUTO:

POST /outlets/0/on

Wyłączenie trybu AUTO:

POST /outlets/0/off

UWAGA:

/outlets/0/on nie oznacza bezwarunkowego włączenia grzałki.

Oznacza:

GRZANIE AUTO = ENABLED

To ESP32 nadal podejmuje decyzję czy grzałka faktycznie ma być ON czy OFF.

21. Sterowanie kalkwasserem przez reef-pi

Włączenie:

POST /outlets/1/on

Wyłączenie:

POST /outlets/1/off

Tak samo:

/outlets/1/on

oznacza pozwolenie na automatyczne dozowanie.

Nie omija zabezpieczenia pH.

Jeżeli pH jest za wysokie:

reef-pi = ON

ale:

ESP32 = POMPA OFF

Zabezpieczenie lokalne ESP32 ma pierwszeństwo.

22. Analog inputs dla reef-pi
/analog_inputs/0

Temperatura 1.

/analog_inputs/1

Temperatura 2.

/analog_inputs/2

pH.

/analog_inputs/3

Moc grzałki w W.

/analog_inputs/4

Prąd w A.

/analog_inputs/5

Napięcie w V.

/analog_inputs/6

dKH.

Przykład:

http://IP_ESP32/analog_inputs/2

może zwrócić:

8.086
23. Konfiguracja drivera reef-pi

Docelowa konfiguracja:

Driver type:
esp32

Address:
IP_ESP32

DigitalOutput:
2

DigitalInput:
0

PWM:
0

AnalogInput:
7

Mapowanie:

OUTLET 0 -> HEATER AUTO
OUTLET 1 -> KALK AUTO

ANALOG 0 -> TEMP1
ANALOG 1 -> TEMP2
ANALOG 2 -> pH
ANALOG 3 -> W
ANALOG 4 -> A
ANALOG 5 -> V
ANALOG 6 -> dKH
24. TFT

Wyświetlacz pokazuje dwie strony.

Strona pH / kalkwasser

Przykład:

pH      8.32

KALK    ON
POMPA   180 st/s

dKH     8.20
Strona temperatura / grzałka

Przykład:

T1 25.12
T2 25.08

HEAT ON
198 W

231.8 V
0.86 A

Strony są zmieniane automatycznie.

25. Wi-Fi

ESP32 działa jednocześnie jako:

STA

czyli klient domowego Wi-Fi,

oraz:

AP

czyli własny Access Point awaryjny.

Jeżeli domowe Wi-Fi nie działa, nadal można połączyć się bezpośrednio z AP ESP32.

26. Zmiana Wi-Fi

Panel:

http://IP_ESP32/wifi

pozwala zapisać:

SSID
hasło

Po zapisaniu ESP32 wykonuje restart.

27. OTA

Firmware można aktualizować przez sieć bez przewodu USB.

Hostname:

esp32-akwarium

OTA należy zabezpieczyć własnym hasłem.

Nie zostawiać domyślnych haseł w wersji produkcyjnej.

28. Wymagane biblioteki Arduino

Potrzebne są między innymi:

OneWire
DallasTemperature
Adafruit ADS1X15
Adafruit BusIO
DFRobot ESP PH WITH ADC
TFT_eSPI

Biblioteki ESP32:

WiFi
WebServer
ArduinoOTA
Preferences
Wire
SPI
EEPROM
29. Ważne ustawienia przed kompilacją

W kodzie należy ustawić:

const char* DEFAULT_WIFI_SSID = "TWOJE_WIFI";
const char* DEFAULT_WIFI_PASS = "TWOJE_HASLO";

oraz:

const char* TUYA_IP = "<IP_SP301>";
const char* DEVICE_ID = "<DEVICE_ID>";
const char* LOCAL_KEY = "<LOCAL_KEY>";

Nie publikować pliku zawierającego:

Wi-Fi password
Tuya local_key
API Secret
Access Secret

Jeżeli projekt trafia na GitHub, dane powinny zostać przeniesione do osobnego pliku, np.:

secrets.h

i dodane do:

.gitignore
30. Testowanie

Przed podłączeniem kalkwassera do akwarium należy sprawdzić:

1. oba DS18B20,
2. pH,
3. pompę bez płynu,
4. kierunek pompy,
5. SP301 ON/OFF,
6. odczyt W/A/V,
7. zachowanie przy uszkodzeniu czujnika temperatury,
8. zachowanie przy wysokim pH,
9. restart ESP32,
10. utratę reef-pi.
31. Obecne zabezpieczenia

Sterownik posiada programowe zabezpieczenia:

GRZAŁKA

brak T1/T2       -> OFF
różnica > 1°C    -> OFF
temperatura 27°C -> OFF
AUTO OFF         -> OFF

oraz:

KALKWASSER

brak pH          -> STOP
pH >= 8.45       -> STOP
pH >= 8.60       -> HARD STOP
AUTO OFF         -> STOP
speed = 0        -> STOP
32. Ważne ograniczenie obecnej wersji

SP301 jest sterowany przez Wi-Fi.

Oznacza to, że komunikacja:

ESP32 -> router/AP -> SP301

jest konieczna do zmiany stanu grzałki.

Jeżeli router lub Wi-Fi całkowicie przestaną działać w momencie, kiedy grzałka jest fizycznie włączona, ESP32 nie będzie mogło wysłać do SP301 komendy OFF.

Dlatego w przyszłości warto rozważyć dodatkowe, niezależne zabezpieczenie sprzętowe:

niezależny termostat
lub
fizyczny przekaźnik bezpieczeństwa

Nie jest ono jeszcze częścią obecnej wersji.

33. Planowane funkcje

Do dodania po dalszych testach:

kalibracja pompy kroki -> ml,
rzeczywiste ml/min,
licznik podanego kalkwassera,
limit ml/dobę,
dzienna dawka kalkwassera,
sterowanie dawką na podstawie konsumpcji dKH,
historia pH,
alarm grzałka ON + 0 W,
alarm grzałka OFF + wykryta moc,
wykrywanie starej / utraconej telemetrii SP301,
ustawianie progów przez WWW,
zapis wszystkich progów do NVS,
przypisanie fizycznych sond temperatury do ich ROM ID,
dodatkowy sprzętowy fail-safe grzałki.
34. Aktualny etap projektu

Aktualnie potwierdzone są:

2x DS18B20        -> działają
pH                -> działa
ADS1115           -> działa
panel HTTP        -> działa
/status           -> działa
SP301 relay state -> działa
SP301 voltage     -> działa
reef-pi API       -> zaimplementowane

Następne testy:

pompa perystaltyczna
SP301 pod obciążeniem
odczyt rzeczywistej mocy W/A
integracja z reef-pi
kalibracja pompy ml/krok
35. Bezpieczeństwo

Projekt steruje urządzeniami pracującymi z napięciem sieciowym.

ESP32 nie powinno być bezpośrednio podłączane do 230 V.

Grzałka jest zasilana przez gotowe gniazdko SP301.

Przy akwarium morskim należy dodatkowo zwrócić szczególną uwagę na:

wilgoć,
zasolenie,
przewody,
izolację,
połączenia masy,
zabezpieczenie różnicowoprądowe,
mechaniczne zabezpieczenie elektroniki przed wodą.

Automatyka programowa nie zastępuje niezależnych zabezpieczeń elektrycznych.


Jedna rzecz, którą warto zrobić od razu: trzymaj w README tylko placeholdery typu `<LOCAL_KEY>` i `<HASLO_WIFI>`. Prawdziwych danych Tuya i Wi-Fi nie wrzucaj do README ani do publicznego GitHuba.
