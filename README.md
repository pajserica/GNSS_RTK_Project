# 🛰️ RTK Rover Project - ESP32 Implementation

Kompletan ESP-IDF projekt za RTK (Real-Time Kinematic) rover sistem sa ESP32 mikrokontrolerom.

## 📁 Struktura Projekta

```
/app/
├── rtk_base_station/          # Bazna stanica
│   ├── platformio.ini         # PlatformIO konfiguracija
│   └── src/
│       ├── main.cpp           # Glavni program
│       ├── gnss_module.*      # GNSS komunikacija
│       ├── espnow_comm.*      # ESP-NOW komunikacija
│       ├── wifi_ap.*          # WiFi Access Point
│       └── web_server.*       # HTTP web server
└── rtk_rover/                 # Rover
    ├── platformio.ini         # PlatformIO konfiguracija
    └── src/
        ├── main.cpp           # Glavni program
        ├── gnss_rover.*       # GNSS komunikacija
        ├── espnow_rover.*     # ESP-NOW komunikacija
        ├── imu_module.*       # IMU senzor (MPU6050)
        ├── motor_control.*    # PWM kontrola motora
        └── navigation.*       # Navigacioni algoritam
```

## 🔧 Hardverska Konfiguracija

### 📡 Bazna Stanica (ESP32 + LC29HBS)
- **UART komunikacija sa GNSS**: GPIO16 (RX), GPIO17 (TX), 115200 baud
- **WiFi Access Point**: `RTK_Base_Station` / `rtk123456`
- **Web interfejs**: http://192.168.4.1
- **ESP-NOW**: Kanal 1

### 🤖 Rover (ESP32 + LC29HDA + MPU6050)
- **UART komunikacija sa GNSS**: GPIO16 (RX), GPIO17 (TX), 115200 baud
- **I2C komunikacija sa IMU**: GPIO21 (SDA), GPIO22 (SCL)
- **PWM kontrola motora**: GPIO32 (levi), GPIO33 (desni)
- **ESP-NOW**: Kanal 1

## ⚙️ Funkcionalnosti

### Bazna Stanica
- ✅ Automatsko čitanje RTCM podataka sa LC29HBS modula
- ✅ ESP-NOW slanje RTCM podataka ka roveru
- ✅ WiFi Access Point za monitoring
- ✅ Web interfejs sa real-time statusom
- ✅ JSON API za status informacije
- ✅ FreeRTOS task arhitektura

### Rover
- ✅ ESP-NOW prijem RTCM podataka
- ✅ Prosleđivanje RTCM podataka GNSS modulu
- ✅ RTK pozicioniranje sa centimetarskom preciznošću
- ✅ IMU integracija (MPU6050) za orijentaciju
- ✅ PWM kontrola motora (RC kompatibilno)
- ✅ Autonomna navigacija kroz waypoint-ove
- ✅ Naprednaa navigaciona logika
- ✅ FreeRTOS task arhitektura

## 🚀 Instalacija i Pokretanje

### Preduslovi
1. **PlatformIO IDE** ili **PlatformIO Core**
2. **ESP-IDF v5.1.2** (automatski se instalira)
3. ESP32 development board-ovi
4. GNSS moduli (LC29HBS, LC29HDA)
5. IMU senzor (MPU6050)

### Kompajliranje i Flash

```bash
# Bazna stanica
cd rtk_base_station/
platformio run --target upload

# Rover
cd rtk_rover/
platformio run --target upload
```

### Serial Monitor
```bash
platformio device monitor --baud 115200
```

## 📊 Web Interfejs

Bazna stanica kreira WiFi Access Point:
- **SSID**: `RTK_Base_Station`
- **Password**: `rtk123456`
- **URL**: http://192.168.4.1

### Web interfejs prikazuje:
- RTK status
- Broj satelita
- Broj poslanih RTCM paketa
- Uptime sistema
- Status GNSS i rover konekcije

### API Endpoints:
- `GET /` - Web interfejs
- `GET /api/status` - JSON status podatci

## 🗺️ Waypoint Konfiguracija

Waypoint-ovi su definsani u `rtk_rover/src/main.cpp`:

```cpp
const coordinate_t waypoints[] = {
    {45.123456, 19.654321, 150.0},  // Point 1
    {45.123556, 19.654421, 150.0},  // Point 2  
    {45.123656, 19.654521, 150.0},  // Point 3
    {45.123756, 19.654621, 150.0},  // Point 4
    {45.123456, 19.654321, 150.0}   // Return to start
};
```

**⚠️ VAŽNO**: Zamenite ove koordinate sa stvarnim koordinatama vaše lokacije!

## 🔄 ESP-NOW Komunikacija

Rover i bazna stanica komuniciraju preko ESP-NOW protokola:
- **Kanal**: 1
- **RTCM paketi**: Bazna stanica → Rover
- **Heartbeat**: Rover → Bazna stanica (svakih 5 sekundi)
- **Status paketi**: Dvosmerni

### MAC Adrese
Inicijalno se koriste broadcast MAC adrese (`FF:FF:FF:FF:FF:FF`). Tokom rada, uređaji uče jedni drugih MAC adrese.

## 🧭 Navigacioni Algoritam

Rover koristi hibridni navigacioni pristup:

1. **Pozicioniranje**: RTK GNSS (centimetarska preciznost)
2. **Orijentacija**: IMU gyroscope (kompenzuje magnetnu interferencu motora)
3. **Kontrola motora**: Differential drive sa PWM signalima
4. **Navigacija**: Haversine formula za distancu, bearing calculation za smer

### Navigacioni parametri:
- **Waypoint tolerancija**: 2.0m
- **Maksimalna brzina**: 70%
- **Minimalna brzina**: 30%
- **Heading tolerancija**: 15°

## 📝 Logovanje

Koristi se ESP-IDF logging sistem:
- **ESP_LOGI**: Osnovne informacije
- **ESP_LOGD**: Debug informacije
- **ESP_LOGW**: Upozorenja
- **ESP_LOGE**: Greške

Debug level se može podesiti u `platformio.ini`:
```ini
build_flags = 
    -DCORE_DEBUG_LEVEL=3
    -DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG
```

## 🛠️ Kustomizacija

### PIN konfiguracija
Svi pinovi su definsani u header fajlovima i mogu se lako promeniti:

```cpp
// GNSS UART pinovi
#define GNSS_TX_PIN GPIO_NUM_17
#define GNSS_RX_PIN GPIO_NUM_16

// IMU I2C pinovi
#define IMU_SDA_PIN GPIO_NUM_21
#define IMU_SCL_PIN GPIO_NUM_22

// Motor PWM pinovi
#define MOTOR_LEFT_PIN GPIO_NUM_32
#define MOTOR_RIGHT_PIN GPIO_NUM_33
```

### Navigacioni parametri
```cpp
#define WAYPOINT_TOLERANCE_M 2.0
#define MAX_SPEED 0.7
#define MIN_SPEED 0.3
#define HEADING_TOLERANCE_DEG 15.0
```

## 🐛 Troubleshooting

### Česti problemi:

1. **GNSS nema podatke**
   - Proverite UART konekcije
   - Proverite antenna konekciju
   - Čekajte da sateliti budu vidljivi (spoljašnja lokacija)

2. **ESP-NOW komunikacija ne radi**
   - Proverite da oba ESP32 koriste isti kanal
   - Proverite MAC adrese u log-u
   - Rastojanje između uređaja < 100m

3. **IMU kalibracoja neuspešna**
   - Držite rover nepomično tokom kalibracije
   - Proverite I2C konekcije
   - Proverite MPU6050 adresu (0x68)

4. **Motori ne reaguju**
   - Proverite PWM pinove
   - Proverite da ESC-ovi podržavaju 50Hz PWM
   - Proverite napajanje motora

### Debug komande:
```bash
# Serial monitor sa filter-om
platformio device monitor --baud 115200 --filter esp32_exception_decoder

# Kompajliranje sa verbose output
platformio run --verbose
```

## 📚 Literatura i Reference

- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [ESP-NOW Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [RTCM Standard 10403.3](https://www.rtcm.org/)
- [NMEA 0183 Standard](https://www.nmea.org/)

## 📄 Licenca

Ovaj projekat je open source i dostupan je za obrazovne i komercijalne svrhe.

---

**Autor**: E1 AI Assistant  
**Verzija**: 1.0  
**Datum**: Juni 2025