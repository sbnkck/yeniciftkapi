#pragma region
#define kanal1 1 //  Choose pwm signal channel for motor
#define kanal2 2 //  Choose pwm signal channel for motor
#define kanal3 3 //  Choose pwm signal channel for motor
// configSUPPORT_DYNAMIC_ALLOCATION
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX_TX "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

#include <Arduino.h> //  Arduino için gerekli kütüphaneler eklendi.
#include <math.h>
#include "stdio.h"             //  Standart C/C++ kütüphaneleri eklendi.
#include "freertos/FreeRTOS.h" //  Gömülü OS için gerekli kütüphaneler eklendi.
#include "freertos/task.h"     //  Multitask için gerekli kütüphane eklendi.
#include "freertos/queue.h"    //  Kuyruk (buffer) için gerekli kütüphane eklendi.
#include "freertos/semphr.h"
#include "esp_attr.h"           //  Datanın kaydedileceği ortamı belirleyen kütüphane eklendi.
#include "soc/rtc.h"            //  Gün, ay, yıl gibi bilgilerin oluşturulması için kütüphane eklendi.
#include "rom/rtc.h"            //  Reset sebepleri için kullanılan kütüphanedir.
#include "driver/mcpwm.h"       //  PWM sinyalü üretmek için gerekli olan kütüphane eklendi.
#include "soc/mcpwm_reg.h"      //  BLDC motor sürüş kütüphanesidir.
#include "soc/mcpwm_struct.h"   //  BLDC motor sürüş kütüphanesidir.
#include "esp_intr_alloc.h"     //  Interrupt için gerekli kütüphane eklendi.
#include "driver/periph_ctrl.h" //  Seri haberleşme birimlerinin kontrolü için kütüphane eklendi.
#include "SimpleKalmanFilter.h" //  Analog değerlerin filtrelenmesi için kütüphane eklendi.
#include "EEPROM.h"             //  EEPROM kullanımı için gerekli kütüphane eklendi.
#include "WiFi.h"               //  Wi-Fi bağlanmak için gerekli kütüphaneler eklendi.
#include "HTTPClient.h"         //  HTTP protokolü için gerekli kütüphaneler eklendi.
#include <ESP32httpUpdate.h>    // ESP32 Update Kütüphanesi eklendi.
#include "BLEDevice.h"          //  BLE ile ilgili kütüphaneler eklendi.
#include "BLEServer.h"          //  BLE ile ilgili kütüphaneler eklendi.
#include "BLEUtils.h"           //  BLE ile ilgili kütüphaneler eklendi.
#include "BLE2902.h"            //  BLE ile ilgili kütüphaneler eklendi.
#include "defines.h"
#include "esp_wifi.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
static BLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID charUUID("6E400004-B5A3-F393-E0A9-E50E24DCCA9E");
int32_t COZUNURLUK = 2048;   //  Choose duty cycle resolution for pwm signal
char MACID[] = "kapi_devas"; //  ESP32'nin BLE ismi tanımlanmıştır.kapi_devas
bool deviceConnected = 0;    //  BLE cihaz bağlandığı zaman bu değişken 1 olur.
BLECharacteristic *pCharacteristic;
BLECharacteristic *pCharacteristic_2;
mcpwm_pin_config_t pin_config;
float derece_katsayi;
double derece;
int8_t CW = 1;   //  Assign a value to represent clock wise rotation
int8_t CCW = -1; //  Assign a value to represent counter-clock wise rotation

/* Pinler */
const int kilit_pini_role = 2;
const int kilit_pini = 16; // 2; //  Kapı kilidini açan ve kapatan pindir.
const int state_led = 12;  // 4;  //  Yanıp sönen bildirim LED'idir.
// const int tork_jumper = 15;  //  Başlangıçta tork arttırmak için kullanılan jumper kapı yönü ayarlaması için kullanılmaktadır.
const int bl_led = 4;           // 16;       //  Dış aydınlatma LED'inin pinidir.
const int kapat_pini = 23;      // 18;      //  Kapat sintalinin geldiği pindir.
const int ac_pini = 22;         // 22 fotosel veya isin perdesi gpio su
const int asansor_ac_pini = 18; // 21; //  18 asansorden gelen aç sinyaline göre hareket ettirecek pin
const int stop_pini = 21;       // 23;       //22;    //  Stop düğmesi için kullanılan pindir.
const int x1_jumper = 19;       // 23;    //  tork arttırır
const int encodera = 34;        //  s1 Encoder pinidir.
const int encoderb = 35;        //  s2 Encoder pinidir.
const int encoderc = 32;        //  s3 Encoder pinidir.
const int H1 = 15;              // 13;           //  H1 bobininin pinidir.
const int H2 = 13;              // 5;            //  H2 bobininin pinidir.
const int H3 = 14;              //  H3 bobininin pinidir.
const int L1 = 27;              //  L1 bobininin pinidir.
const int L2 = 26;              //  L2 bobininin pinidir.
const int L3 = 25;              //  L3 bobininin pinidir.
const int fault = 33;           //  2132 entegresinin aşırı akım çekip çekmediğini denetleyen pindir.
#ifdef EDL_select
const int sense_pini = 39; //  Akım okuyan pindir.
const int x2_jumper = 36;  // dc barada kullanılıyor
const int yon_jumper = 5;  //  Başlangıçta tek başına yön ayarlamak için kullanılan jumper kapı yönü ayarlaması için kullanılmaktadır.

#else
const int sense_pini = 36; //  Akım okuyan pindir.
const int x2_jumper = 39;  // dc barada kullanılıyor
const int yon_jumper = 17; //  Başlangıçta tek başına yön ayarlamak için kullanılan jumper kapı yönü ayarlaması için kullanılmaktadır.

#endif

/* Integer Değişkenleri */
int adim = 0; // Motor hareket ederken hall sensorlerinin okunduğu değişkendir.
int eski_adim = 9;
double duty;                         // Motor hareketinde verilen duty değişkenidir.
int direct = 0;                      // Interrupt içerisinde motorun dönüş yönünü gösteren değişkendir.
int16_t rpm = 0;                     // Kesmelerde ölçtüğümüz devir sayısı.
volatile int16_t adim_sayisi = 3000; // Kesmelerde belirlediğimiz motorun adim sayisi.
int16_t eski_adim_sayisi = 0;        // Kesmelerde belirlediğimiz motorun eski adim sayisi.
int16_t rpm_list[10];                // 5 rpm değerinin ortalamasını alan değişken.
uint8_t rpm_count = 0;
/* Volatile Integer Değişkenleri */
volatile int alcak_duty = 0;  //  Fren bobinlerine uygulanan duty miktarıdır.
volatile int yuksek_duty = 0; //  Hareket bobinlerine uygulanan duty miktarıdır.

/* Bool Değişkenleri */
bool s1_state;
bool s2_state;
bool s3_state;
bool motor_yonu = kapi_ac;                    // Motor_yonu=1 ise kapı kapanıyordur, Motor_yonu=0 ise kapı açılıyordur..
uint8_t hareket_sinyali = kapi_kapat_sinyali; // Kapı açmak için verilen sinyal.

/* Byte Değişkenleri */

/* Struct Değişkenleri */
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_2 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_3 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_4 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = NULL;
SemaphoreHandle_t UartMutex;
SemaphoreHandle_t akim_mutex;
/*******************BLUETOOTH*****************/

double toplam_fark = 0;
int32_t toplam_duty = 0;
bool kapat_time_out_flag = false;
uint16_t ilk_kapanma_count = 0;
int eeprom_sayaci = 0;
volatile int ble_alinan_deger = 0; //  ble_gelen_dizi_global[3] elemanının eşitlendiği değerdir.
uint8_t ble_yollanan_dizi_global[250];
uint8_t ble_gelen_dizi_global[250];
// char MACID[] = "giris";                   //  ESP32'nin BLE ismi tanımlanmıştır.
byte sifre_1 = 0;                //  BLE üzerinden gelen dizinin 1. elemanıdır.
byte sifre_2 = 0;                //  BLE üzerinden gelen dizinin 2. elemanıdır.
byte adres = 0;                  //  BLE üzerinden gelen dizinin 3. elemanıdır.
int acma_hizlanma_rampasi = 0;   //  Duty haritası değişkenidir.
int acma_durma_rampasi = 0;      //  Duty haritası değişkenidir.
int acma_baski_gucu = 0;         //  Duty haritası değişkenidir.
int acma_carpma_siniri = 0;      //  Duty haritası değişkenidir.
int kapama_hizlanma_rampasi = 0; //  Duty haritası değişkenidir.
int kapama_baski_suresi = 0;
int kapama_baski_suresi_ks = 10; // ble den gelen ve eepromdan okunan değerle çarparak asıl olan ms değerini bulur
int kapama_durma_rampasi = 0;    //  Duty haritası değişkenidir.
int kapama_baski_gucu = 0;       //  Duty haritası değişkenidir.
int kapama_carpma_siniri = 0;    //  Duty haritası değişkenidir.
byte kapi_acma_acisi = 0;        //  Kapının kaç derece açılacağını belirten açıdır.
byte asiri_akim_siniri = 0;      //  Maksimum çekilmesine izin verilen akım miktarıdır.
float local_hesap_akim = 0;      //  EEPROM'dan okunan ve BLE'den gelen akım değişkeninden maksimum akım değerinin hesaplanması için gereken değişkendir.
float maksimum_akim_degeri = 99; //  Motorun çekebileceği maksimum akımı sınırlayan değişkendir. Kapı aç ve kapat fonksiyonlarının içerisinde kullanılır.
byte calisma_yontemi = 1;        //  Değişken 1 ise aç ve kapat sinyali işlemciye gelir, 2 ise aç sinyali ve zaman gecikmeli kapanma vardır, 3 ise aç sinyalinin kesilmesi ile kapı kapanır.
int otomatik_kapanma_zamani = 0; //  Çalışma yöntemi 2 iken kapının açıldıktan kaç saniye sonra kapanacağını belirten değişkendir.
float birinci_akim_tam = 0;      //  BLE cihazdan gelen 1. akım değeridir.
float ikinci_akim_tam = 0;       //  BLE cihazdan gelen 2. akım değeridir.
float birinci_akim_ondalik = 0;  //  BLE cihazdan gelen 1. akım değeridir.
float ikinci_akim_ondalik = 0;   //  BLE cihazdan gelen 2. akım değeridir.
bool akim_alindi = 0;            //  Kalibrasyon fonksiyonu içerisinde akım değişkeninin alınıp alınmadığını denetleyen fonksiyondur. Böylelikle diğer aşamaya geçilir.
byte test_modu_aktif = 0;        //  Test modunun açık olup olmadığını gösteren değişkendir. Demo için de kullanılabilir.
int ble_ekran = 0;               //  BLE uygulamasının hangi ekranda olduğunu belirten değişkendir. ToDo: Bu değişkene göre BLE gönderme hızı ve notify ayarları değişmelidir.
int ble_sayaci = 0;              //  Loop dönügsü içerisindeki BLE data gönderme fonksiyonun çalışma sayacıdır.
volatile int isr_aktif = 0;      //  Kesme olup olmadığını gösteren değişkendir.
byte ble_offset = 0;             //  Float olan değişkenin BLE üzerinden geönderilmesi için oluşturulmuştur.
float offset = 0;                //  Mosfet çıkışlarının PWM 0 iken ürettiği offset voltajıdır.

volatile int eeproma_yaz_istegi = 0; //  EEPROM commit fonksiyonunun çalıştırılması için gerekli değişkendir.
// volatile int eeproma_yaz_istegi = 0;   //  EEPROM commit fonksiyonunun çalıştırılması için gerekli değişkendir.

volatile int kalibrasyon_aktif = 0; //  Kalibrasyon fonksiyonunun aktif olup olmadığını belirten değişkendir.
volatile int olculen_kapi_boyu = 0; //  Kapının o anki konumudur.

/***************************************/

/* UİNT Değişkenleri */

uint8_t u_bobin = 0;
uint8_t v_bobin = 1;
uint8_t w_bobin = 2;
uint8_t bobin_durumu;

/* Bobinler arası süre için kullanılan değişkenler */
double bobin_fark_sure = 0, bobin_eski_sure = 0;
double hedef_sure = 5330;
double RPM_katsayisi = 1.2;
double rpm_fark = 0;
/****************************/
// extern void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName );
//  ConfigCHECK_FOR_STACK_OVERFLOW
void DutyHesaplama();
void MotorBekletme();
bool connectToServer();                  ////ble telefona bağlanma işleminde devreye girer ve duruma göre true döndürür.
void adim_oku();                         // bldc motorun encoderını okur ve adımını belirler
void rpm_tespit_fn();                    // motor rpm ölçümü esnasında encoderdan gelen bilgiler parazt kaynaklı ise rpm ölçümü yaptırmaz
static void test_task(void *arg);        // kapı  her kapandığında aç sinyali verdirip test etmek istiyorsak uygulamadn aktif edilince aç sinyali vermeye başlar
static void ac_task(void *arg);          // ac kapa butonu ve stop butonunu kontrol edeceğiz
static void motor_ilk_tahrik(void *arg); // motor durduğu anlarda devreye sokuyoruz ve motora hareket veriyor
static void rpm_olcum(void *arg);        // rpm ölçümü yapılan task
static void hareket_kontrol(void *arg);  // genel olarak kapının açılma, kapanma, durma vb işlmelerin takip edilip yönetildiği taks
static void seri_yazdir(void *arg);      // sistemde görmek istediğimiz verileri ekrana yazdırmak istiyorsak aktif edeceğimiz task
static void seri_oku(void *arg);         // seri haberleşmeden geln komutların okunup uygulandığı task
/*encoder kesmelerinde devreye giren fonksiyonlar*/
static void IRAM_ATTR encoder_a_kesme();
static void IRAM_ATTR encoder_b_kesme();
static void IRAM_ATTR encoder_c_kesme();
/*ac pini interruptında yapılcak işlemleri kapsar*/
static void IRAM_ATTR pin_kesmesi_ac();
/*ac sinyali olduğu müddetçe açmasını sağlayan butondur. bu butonun kesme fonksiyonu*/
static void IRAM_ATTR pin_kesmesi_asansor_ac();
/*stop pini kesmesi*/
static void IRAM_ATTR pin_kesmesi_stop();
/*fault pini kesme işlemleri*/
static void IRAM_ATTR fault_kesme();
/*kapı açılırken rpm haritası ve temel açma işlmelerinin yapıldığı fonk*/
void kapi_ac_hazirlik();
/*kapı kapanırken rpm haritası ve temel açma işlmelerinin yapıldığı fonk*/
void kapi_kapat_hazirlik();
/*motor duty parametresi verilince hangi adıma göre hangi duty verilecek belirleyen ve pwm uygulayan fonk */
void motor_surme(uint16_t param);
/*kapı açılırken oluşacak durumlara göre süreci ayarlayan fonk*/
void kapi_ac_fonksiyonu();
/*kapı kapanırken oluşacak durumlara göre süreci ayarlayan fonk*/
void kapi_kapa_fonksiyonu();
/*kapı ilk kapanma da  oluşacak durumlara göre süreci ayarlayan fonk*/
void ilk_kapi_kapanma();
/*kapı kiliti ne zman açılır ne zaman kapanacak belirleyen fonksiyonlar*/
void kilit_ac();
void kilit_kapat();
/*epromdan kayırlı verileri okur*/
void eeprom_oku_fn();
/*ble de başlatma esnasında temel ayarların yapılıp ble nin başlatıldığı fonk*/
void ble_baslat_fn();
/*ble den gelen dataları işleyerek ilgili değişkenlere atayn ve kayıt işlemini aktif eden fonk*/
void ble_data_al();
/*ble aplikasyonuna veri güncellemesi gönderen fonk*/
void ble_data_guncelle();
/*eprom kayıt işlemini belirli zamnlarda yaptırarak pwm in bozulmasını engelleyen fonk*/
void eeprom_kontrol();
/*sayaç işlemlerinin tutulduğu fonk*/
void sayac_eeprom_yaz();///sayaç değerlerini hafızaya yazar
void sayac_sifirla_fn();                      //  Sayaçları sıfırlayan fonksiyondur.
void print_reset_reason(RESET_REASON reason); //  İşlemcinin son kapanmasında neden kapandığını bize bildiren fonksiyondur.
/*dc bara ölçümü ya*pan task*/
void dc_bara(void *arg);
/*st ledini hangi durumda nasıl yanacağını ayarlar*/
void status_led_task(void *arg);
/*motor akımını okuyan task*/
static void motor_akim_oku(void *arg);
/*bel işlemlerinin yürütüldüğü task*/
static void ble_task(void *arg);//ble süreçlerinin yönetildiği task
void kalibrasyon_fn();                     // kullanılmıyor
static void led_kontrol_fn(void *arg);     // kapı ledinin nasıl yanacağını ayarlar
double bobin_ortalama_alma(double fark);   // bobin kesmelerinde oluşan sürelerin ortalmasını alarak motorun daha yumuşak hareket etmesni sağlar
double amper_siniri_func();                // motorun çektiği akıma göre duty i kısar
void akim_sinirlama(int16_t *data);        // kullanılmıyor
void rpm_haritasi_olustur_engelli_kapat(); // kapaı kapatırken engel algılarsa tekrar kapanırken engel algıladığı yeri daha yavaş geçmesini sağlar
/*kart yazılımın güncel olup olmadığını serverdan kontrol eden fonk*/
void GetUpdateLink(String url);
/*update işlemlerinin yönetildiği fonk*/
void GetUpdate();
String convertToString(char *a, int size);
/*kapı kilit rolesinin durumunun işendiği task*/
void role_state(void *arg);
/*motor soketinin varlığını, voltaj düzeyini ve bobinlerde kısa devre varmı diye kontrol eden fonk*/
void test_func();
/*bleden data alım sürecini işleyen task*/
void ble_data_al_task(void *arg);
/*fault kesmesinden gelen dataya göre süreci işleyen task*/
void fault_task(void *arg);
/*aplikasyondan log kayıtları akti edilirse server a data göndermek için kullanılan fonk.kart datalarını uzaktan okumak için kullanıyoruz*/
void PrintLog(String Txt);
void PostJson(String Url, String PostData);
String GetPostJson(String _Val);
/*başlangıç datalarınınn servera yzdrılıdığı fonk*/
void print_log_baslangic();
/*çift kanat seçimi yapıldığında client olarak bağlanmasına ve data göndermesini yöneten task*/
void ble_client_task(void *arg);
/*trasnsistör lerin durumunu yöneten task aplikasyondan seçilen görev atamasına göre farklı şekilde transistörler farklı çalışacak*/
void tr_mission_task(void *arg);
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;
TaskHandle_t xHandle = NULL;
double toplam_rpm[10];
double onlarin_rpm;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
class MyClientCallback : public BLEClientCallbacks
{
 void onConnect(BLEClient *pclient)
 {
 }

 void onDisconnect(BLEClient *pclient)
 {
  connected = false;
  Serial.println("onDisconnect");
 }
};
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
 /**
        Called for each advertising BLE server.
    */
 void onResult(BLEAdvertisedDevice advertisedDevice)
 {
  Serial.print("BLE Advertised Device found: ");
  Serial.println(advertisedDevice.toString().c_str());

  // We have found a device, let us now see if it contains the service we are looking for.
  if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID))
  {

   BLEDevice::getScan()->stop();
   myDevice = new BLEAdvertisedDevice(advertisedDevice);
   doConnect = true;
   doScan = true;

  } // Found our server
 }  // onResult
};  // MyAdvertisedDeviceCallbacks

class MyServerCallbacks : public BLEServerCallbacks
{
 void onConnect(BLEServer *pServer)
 { //  BLE cihaz bağlandığı zaman tek seferliğine çalışan bir fonksiyondur.
  deviceConnected = true;
  ble_data_send = true;
  Serial.println("BLE Fn:cihaz CN");
 };
 void onDisconnect(BLEServer *pServer)
 { //  BLE cihaz bağlantısı koptuğu zaman tek seferliğine çalışan bir fonksiyondur.
  deviceConnected = false;
  Serial.println("BLE Fn:no");
 }
};

class ble_alma_class : public BLECharacteristicCallbacks
{
 void onWrite(BLECharacteristic *pCharacteristic)
 { //  BLE cihazından data geldiği zaman tek seferlik çalışan fonksiyondur.
  ble_gelen_dizi = pCharacteristic->getValue();
  ble_data_al_flag = true;
 }
};
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.1);
#pragma endregion

QueueHandle_t Amper_Queue; // amper işlemleri yapılırken tasklarda birbirini engelleme meydana geliyordu.engellemek için queue kullandık ve çözüldü

void setup()
{
 Amper_Queue = xQueueCreate(5, sizeof(double));
 Serial.begin(115200);      //  Seri Haberleşme başlatıldı.
 EEPROM.begin(eeprom_size); //  EEPROM belirlenen boyutta başlatıldı.
 /****************************/
 akim_mutex = xSemaphoreCreateMutex();
 xSemaphoreGive(akim_mutex);
 get_update_fl = EEPROM.read(101);

 /*  INPUTLARIN BELİRLENMESİ */
 pinMode(encodera, INPUT);  //  GPIO 34 numaralı pin
 pinMode(encoderb, INPUT);  //  GPIO 35 numaralı pin
 pinMode(encoderc, INPUT);  //  GPIO 32 numaralı pin
 pinMode(fault, INPUT);     //  GPIO 33 numaralı pin
 pinMode(x2_jumper, INPUT); //  GPIO 39 numaralı pin
 // pinMode(tork_jumper, INPUT);       //  GPIO 15 numaralı pin
 pinMode(yon_jumper, INPUT_PULLUP); //  GPIO 17 numaralı pin
 pinMode(kapat_pini, INPUT);        //  GPIO 18 numaralı pin
 pinMode(ac_pini, INPUT);           //  GPIO 19 numaralı pin
 pinMode(asansor_ac_pini, INPUT);   //  GPIO 21 numaralı pin
 pinMode(stop_pini, INPUT);         //  GPIO 22 numaralı pin
 pinMode(x1_jumper, INPUT_PULLUP);  //  GPIO 23 numaralı pin
 pinMode(sense_pini, INPUT);        //  GPIO 36 numaralı pin
 /*  OUTPUTLARIN BELİRLENMESİ */
 pinMode(H1, OUTPUT);         //  GPIO 13 numaralı pin
 pinMode(H2, OUTPUT);         //  GPIO 5 numaralı pin
 pinMode(H3, OUTPUT);         //  GPIO 14 numaralı pin
 pinMode(L1, OUTPUT);         //  GPIO 27 numaralı pin
 pinMode(L2, OUTPUT);         //  GPIO 26 numaralı pin
 pinMode(L3, OUTPUT);         //  GPIO 25 numaralı pin
 pinMode(kilit_pini, OUTPUT); //  GPIO 16 numaralı pin
 eeprom_oku_fn();             //  EEPROM okunuyor.
 if (select_tr_mission == kapi_acik)
 {
  digitalWrite(kilit_pini, 1);
 }
 else
 {
  digitalWrite(kilit_pini, 0);
 }

 pinMode(kilit_pini_role, OUTPUT); // pinMode(bl_led, OUTPUT);     //  GPIO 2 numaralı pin
                                   // WiFi.begin("ssid.c_str()", "password.c_str()");

 pinMode(state_led, OUTPUT);                                                  //  GPIO 4 numaralı pin
                                                                              /* Intterrupt Fonksiyonlarının Belirlenmesi */
 attachInterrupt(digitalPinToInterrupt(stop_pini), pin_kesmesi_stop, CHANGE); //  stop için interrupt eklendi ve tanımlandı.

 attachInterrupt(digitalPinToInterrupt(encodera), encoder_a_kesme, CHANGE); //  Adım okumak için interrupt eklendi ve tanımlandı.
 attachInterrupt(digitalPinToInterrupt(encoderb), encoder_b_kesme, CHANGE); //  Adım okumak için interrupt eklendi ve tanımlandı.
 attachInterrupt(digitalPinToInterrupt(encoderc), encoder_c_kesme, CHANGE); //  Adım okumak için interrupt eklendi ve tanımlandı.
 attachInterrupt(digitalPinToInterrupt(fault), fault_kesme, CHANGE);        //  Adım okumak için interrupt eklendi ve tanımlandı.

 /* Motor Sürme */
 uint32_t freakns = 20000;
 ledcSetup(kilit_channel, freakns, 12);
 // ledcAttachPin(kilit_pini, kilit_channel);
 ledcSetup(led_channel, freakns, 12);
 ledcSetup(kanal1, freakns, 12);
 ledcSetup(kanal2, freakns, 12);
 ledcSetup(kanal3, freakns, 12);
 ledcAttachPin(H1, kanal1);
 ledcAttachPin(L1, kanal1);
 ledcAttachPin(H2, kanal2);
 ledcAttachPin(L2, kanal2);
 ledcAttachPin(H3, kanal3);
 ledcAttachPin(L3, kanal3);

#ifdef EDL_select
 GPIO.func_out_sel_cfg[L1].inv_sel = 1;
 GPIO.func_out_sel_cfg[L2].inv_sel = 1;
 GPIO.func_out_sel_cfg[L3].inv_sel = 1;

#else
 GPIO.func_out_sel_cfg[H1].inv_sel = 1;
 GPIO.func_out_sel_cfg[H2].inv_sel = 1;
 GPIO.func_out_sel_cfg[H3].inv_sel = 1;
#endif
 s1_state = digitalRead(encodera);
 s2_state = digitalRead(encoderb);
 s3_state = digitalRead(encoderc);
 adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));

 // test_func();

 // başlangiç ladi dans ettirmemiz istendi
 ledcAttachPin(bl_led, led_channel);
 if (select_tr_mission == standart_led)
 {
  ledcAttachPin(kilit_pini, kilit_channel);
 }

 // int dutyCycle = 0;
 while (dutyCycle <= 4095)
 {
  dutyCycle += 250;
  //  Serial.println(dutyCycle);
  ledcWrite(led_channel, dutyCycle);
  if (select_tr_mission == standart_led)
  {
   ledcWrite(kilit_channel, dutyCycle);
  }
  vTaskDelay(1 / portTICK_PERIOD_MS);
 }
 while (dutyCycle > 3000)
 {
  dutyCycle -= 250;
  ledcWrite(led_channel, dutyCycle);
  if (select_tr_mission == standart_led)
  {
   ledcWrite(kilit_channel, dutyCycle);
  }
  vTaskDelay(1 / portTICK_PERIOD_MS);
 }
 aydinlatma_led_state = 2;
 /***************************************************************/
 motor_surme(0);

 Serial.print("MAC: ");
 Serial.println(WiFi.macAddress());
 Serial.print("Verison : ");
 Serial.println(version);
 if (EEPROM.read(200) > 200 || memcmp("aaaaaaaa", ssid.c_str(), 8) == 0) // ssid eepromda varmi yok mi bakiliyor
 {
  Serial.println("SSID BOS");
  konsol_aktif_flag = false;
 }
 else
 {
  Serial.println("SSID DOLU");
  if (!get_update_fl)
  {
   get_update_fl = true;
   EEPROM.write(101, get_update_fl);
   EEPROM.commit();
   vTaskDelay(10 / portTICK_RATE_MS);
  }
  else
  {
   for (uint8_t i = 0; i < 10; i++)
   {
    digitalWrite(state_led, !digitalRead(state_led));
    delay(100);
   }

   get_update_fl = false;
   EEPROM.write(101, get_update_fl);
   EEPROM.commit();
   vTaskDelay(10 / portTICK_RATE_MS);
   GetUpdate();

   esp_restart();
  }
 }

 if (konsol_aktif_flag) // sistem kapanınca  konsolpasif olacak
 {
  EEPROM.write(14, 0);
  EEPROM.commit();
 }

 /****************************/
 if (konsol_aktif_flag == false) // kosnol aktifse ble kapalı olacak
  ble_baslat_fn();
 delay(1000);

 /****************rpm haritası oluşturma**************/ //////////
 kapi_ac_hazirlik();
 kapi_kapat_hazirlik();

 print_log_baslangic();

 /*********************************/
 if (demo_modu_flag)
 {
  xTaskCreate(test_task, "test_task", 2048, NULL, 10, &test_task_arg);
 }
 baski_duty = kapanma_baski_gucu_min;

 /*  test func çalışmsı için akım ölçümünü başlatmak gerek*/
 /*ble işlmelerini yapan tasklar core 0 daçalışacak diğerleri 1 de .
 Bu şekilde yapılmadığı takdirde motor pwm i engellenebiliyor ve motorda ciddi vuruntular olabiliyor.*/
 xTaskCreatePinnedToCore(motor_akim_oku, "motor_akim_oku", 2048 * 2, NULL, 1, &motor_akim_oku_arg, 1);
 xTaskCreatePinnedToCore(ble_task, "ble_task", 1024 * 3, NULL, 1, &ble_arg, 0);
 xTaskCreate(tr_mission_task, "tr_mission_task", 1024, NULL, 1, &tr_mission_task_arg);

 test_func(); // güncelleme ve ble işlemlerinden sonra test yaptırıyoruz ki sıkıntı çıktığı taktıirde güncelleme yapabilelim

 xTaskCreate(seri_yazdir, "seri_yazdir", 2048 * 4, NULL, 1, &seri_yazdir_arg);
 xTaskCreate(fault_task, "fault_task", 2048 * 4, NULL, 12, &fault_task_arg);
 xTaskCreatePinnedToCore(hareket_kontrol, "hareket_kontrol", 2048 * 4, NULL, 11, &hareket_kontrol_arg, 1);
 // xTaskCreate(hareket_kontrol, "hareket_kontrol", 2048 * 4, NULL, 11, &hareket_kontrol_arg);
 xTaskCreate(ac_task, "ac_task", 2048 * 10, NULL, 10, &ac_task_arg);
 xTaskCreate(seri_oku, "seri_oku", 2048, NULL, 3, &seri_oku_arg);
 xTaskCreatePinnedToCore(rpm_olcum, "rpm_olcum", 2048, NULL, 8, &rpm_olcum_arg, 1);
 xTaskCreate(dc_bara, "dc_bara", 2048, NULL, 2, &dc_bara_arg); // void dc_bara(void *arg)
 xTaskCreate(status_led_task, "status_led_task", 2048, NULL, 1, &status_led_arg);

 xTaskCreate(led_kontrol_fn, "led_kontrol_fn", 1024, NULL, 2, &led_kontrol_arg);

 xTaskCreate(role_state, "role_state", 1024 * 2, NULL, 1, &role_state_arg);
 xTaskCreatePinnedToCore(ble_data_al_task, "ble_data_al_task", 1024 * 4, NULL, 1, &ble_data_al_task_arg, 0);

 UartMutex = xSemaphoreCreateMutex(); // mutex oluşturuluyor

 if (akim_mutex != NULL)
 {
  Serial.println("olusturuldu mutex ");
 }

 if (mentese_yonu == 1)
 {
  CW = -1;
  CCW = 1;
 }
 else
 {
  CW = 1;
  CCW = -1;
 }
 ilk_kapanma_suresi = millis();

 setup_flag = true;
}

void loop()
{
 while (1)
 {
  vTaskDelay(40000 / portTICK_RATE_MS);
 }
}

void motor_ilk_tahrik(void *arg)
{
 // duty=0;
 while (1)
 {
  vTaskDelay(1 / portTICK_RATE_MS);

  s1_state = digitalRead(encodera);
  s2_state = digitalRead(encoderb);
  s3_state = digitalRead(encoderc);
  adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));

  bobin_fark_sure = micros() - bobin_eski_sure;

  if (bobin_fark_sure < (-hedef_sure))
  {
   bobin_fark_sure = (-hedef_sure);
  }
  int32_t adim_tasi = adim_sayisi;

  if (adim_tasi < 0)
  {
   adim_tasi = 0;
  }

  Max_RPM = hedefRPMharitasi[adim_tasi];
  hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi);

  if (hedef_sure == INFINITY)
  {
   hedef_sure = 10000;
  }

  hata = (hedef_sure - bobin_fark_sure);
DutyHesaplama();

  xSemaphoreTake(UartMutex, portMAX_DELAY);
  Serial.print("hesaplanan : ");
  Serial.println(hesaplanan);
  Serial.print("MiT ");
  xSemaphoreGive(UartMutex);
  vTaskDelay(1 / portTICK_RATE_MS);
  eski_adim = adim;

  vTaskDelete(NULL);
 }
}
static void IRAM_ATTR pin_kesmesi_stop()
{
 portENTER_CRITICAL_ISR(&mux_3);
 if (digitalRead(stop_pini) == 1)
 {
  acil_stop_int_flag_kapat = true;
  if (ilk_kapanma == false)
  {
   ilk_kapanma_suresi = millis();
  }

  // acil_stop_flag = false;
 }
 if (digitalRead(stop_pini) == 0)
 {
  acil_stop_int_flag = true;
 }
 portEXIT_CRITICAL_ISR(&mux_3);
}
static void IRAM_ATTR pin_kesmesi_ac()
{
 portENTER_CRITICAL_ISR(&mux_3);
 if (digitalRead(ac_pini) == 1)
 {
  ac_flag = true;
 }
 portEXIT_CRITICAL_ISR(&mux_3);
}
static void IRAM_ATTR pin_kesmesi_asansor_ac()
{
 portENTER_CRITICAL_ISR(&mux_3);
 if (digitalRead(asansor_ac_pini) == 1)
 {
  ac_flag = true;
 }
 if (digitalRead(asansor_ac_pini) == 0)
 {
  kapat_flag = true;
 }
 portEXIT_CRITICAL_ISR(&mux_3);
}
static void IRAM_ATTR pin_kesmesi_kapa()
{
 portENTER_CRITICAL_ISR(&mux_3);
 if (digitalRead(kapat_pini) == 1)
 {
  kapat_flag = true;
 }

 portEXIT_CRITICAL_ISR(&mux_3);
}

static void IRAM_ATTR fault_kesme()
{
 portENTER_CRITICAL_ISR(&mux_3);
 fault_kesme_flag_int = true;
 // if (digitalRead(fault) == 0)
 // vTaskSuspend(hareket_kontrol_arg);
 //  fault_kesme_flag = true;
 portEXIT_CRITICAL_ISR(&mux_3);
}

static void IRAM_ATTR encoder_a_kesme()
{
 portENTER_CRITICAL_ISR(&mux);
 bobin_yeni_sure = micros();
 s3_state = digitalRead(encoderc); // Read the current W hall sensor value
 s2_state = digitalRead(encoderb); // Read the current V (or U) hall sensor value
 s1_state = digitalRead(encodera); // Read the current U (or W) hall sensor value
 adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));
 if ((s1_state == 1 && s2_state == 1 and s3_state == 1) || (s1_state == 0 && s2_state == 0 and s3_state == 0))
 {
  // Serial.print("********HATALI DURUM******** ");
 }
 else if (adim != eski_adim)
 {

  direct = (s1_state == s3_state) ? CCW : CW;
  adim_sayisi = adim_sayisi + (1 * direct);
  bobin_durumu = u_bobin;

  eski_adim = adim;
 }

 rpm_tespit_fn();
 portEXIT_CRITICAL_ISR(&mux);
}

static void IRAM_ATTR encoder_b_kesme()
{
 portENTER_CRITICAL_ISR(&mux);
 bobin_yeni_sure = micros();
 s3_state = digitalRead(encoderc); // Read the current W hall sensor value
 s2_state = digitalRead(encoderb); // Read the current V (or U) hall sensor value
 s1_state = digitalRead(encodera); // Read the current U (or W) hall sensor value
 adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));
 if ((s1_state == 1 && s2_state == 1 and s3_state == 1) || (s1_state == 0 && s2_state == 0 and s3_state == 0))
 {
  // Serial.print("********HATALI DURUM******** ");
 }
 else if (adim != eski_adim)
 {

  direct = (s2_state == s1_state) ? CCW : CW;
  adim_sayisi = adim_sayisi + (1 * direct);

  bobin_durumu = v_bobin;
  eski_adim = adim;
 }

 rpm_tespit_fn();
 portEXIT_CRITICAL_ISR(&mux);
}

static void IRAM_ATTR encoder_c_kesme()
{
 portENTER_CRITICAL_ISR(&mux);
 bobin_yeni_sure = micros();
 s3_state = digitalRead(encoderc); // Read the current W hall sensor value
 s2_state = digitalRead(encoderb); // Read the current V (or U) hall sensor value
 s1_state = digitalRead(encodera); // Read the current U (or W) hall sensor value
 adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));
 if ((s1_state == 1 && s2_state == 1 and s3_state == 1) || (s1_state == 0 && s2_state == 0 and s3_state == 0))
 {
  // Serial.print("********HATALI DURUM******** ");
 }
 else if (adim != eski_adim)
 {

  direct = (s3_state == s2_state) ? CCW : CW;
  adim_sayisi = adim_sayisi + (1 * direct);
  bobin_durumu = w_bobin;

  eski_adim = adim;
 }
 rpm_tespit_fn();
 portEXIT_CRITICAL_ISR(&mux);
}

static void rpm_olcum(void *arg)
{
 while (1)
 {
  vTaskDelay(10 / portTICK_RATE_MS);

  if (hiz_hesapla_flag)
  {

   rpm = (abs(eski_adim_sayisi - adim_sayisi) / 24.0) * 6000;

   rpm_list[rpm_count] = rpm;
   rpm = 0;
   for (uint8_t i = 0; i < 10; i++)
   {
    rpm = rpm + rpm_list[i];
   }
   rpm = rpm / 10;

   eski_adim_sayisi = adim_sayisi;
   rpm_count++;
   if (rpm_count > 9)
    rpm_count = 0;

   derece = (82.1 / 1000.0) * adim_sayisi;
  }
  else
  {
   rpm = 0;
   rpm_list[rpm_count] = rpm;
   for (uint8_t i = 0; i < 10; i++)
   {
    rpm = rpm + rpm_list[i];
   }
   rpm = rpm / 10;

   eski_adim_sayisi = adim_sayisi;
   rpm_count++;
   if (rpm_count > 9)
    rpm_count = 0;

   derece = (82.1 / 1000.0) * adim_sayisi;
  }
 }
}

// void motor_surme(uint16_t param)
// {

//  if (motor_yonu == 0)
//  {
//   yuksek_duty = COZUNURLUK + param;
//   alcak_duty = COZUNURLUK - param;
//   switch (adim)
//   {
//   case 101:
//    ledcWrite(kanal1, alcak_duty);  // 1048
//    ledcWrite(kanal2, yuksek_duty); // 3048
//    ledcWrite(kanal3, yuksek_duty); // 3048
//    break;
//   case 100:
//    ledcWrite(kanal1, alcak_duty);
//    ledcWrite(kanal2, alcak_duty);
//    ledcWrite(kanal3, yuksek_duty);
//    break;
//   case 110:
//    ledcWrite(kanal1, yuksek_duty);
//    ledcWrite(kanal2, alcak_duty);
//    ledcWrite(kanal3, yuksek_duty);
//    break;
//   case 10:
//    ledcWrite(kanal1, yuksek_duty);
//    ledcWrite(kanal2, alcak_duty);
//    ledcWrite(kanal3, alcak_duty);
//    break;
//   case 11:
//    ledcWrite(kanal1, yuksek_duty);
//    ledcWrite(kanal2, yuksek_duty);
//    ledcWrite(kanal3, alcak_duty);
//    break;
//   case 1:
//    ledcWrite(kanal1, alcak_duty);
//    ledcWrite(kanal2, yuksek_duty);
//    ledcWrite(kanal3, alcak_duty);
//    break;
//   }
//  }
//  if (motor_yonu == 1)
//  {
//   yuksek_duty = COZUNURLUK - param;
//   alcak_duty = COZUNURLUK + param;
//   switch (adim)
//   {
//   case 101:
//    ledcWrite(kanal1, alcak_duty);  // 3048
//    ledcWrite(kanal2, alcak_duty);  // 3048
//    ledcWrite(kanal3, yuksek_duty); // 1048
//    break;
//   case 100:
//    ledcWrite(kanal1, yuksek_duty);
//    ledcWrite(kanal2, alcak_duty);
//    ledcWrite(kanal3, yuksek_duty);
//    break;
//   case 110:
//    ledcWrite(kanal1, yuksek_duty);
//    ledcWrite(kanal2, alcak_duty);
//    ledcWrite(kanal3, alcak_duty);
//    break;
//   case 10:
//    ledcWrite(kanal1, yuksek_duty);
//    ledcWrite(kanal2, yuksek_duty);
//    ledcWrite(kanal3, alcak_duty);
//    break;
//   case 11:
//    ledcWrite(kanal1, alcak_duty);
//    ledcWrite(kanal2, yuksek_duty);
//    ledcWrite(kanal3, alcak_duty);
//    break;
//   case 1:
//    ledcWrite(kanal1, alcak_duty);
//    ledcWrite(kanal2, yuksek_duty);
//    ledcWrite(kanal3, yuksek_duty);
//    break;
//   }
//  }
// }
void motor_surme(uint16_t param)
{

 if (motor_yonu == 0)
 {
  yuksek_duty = COZUNURLUK + param;
  alcak_duty = COZUNURLUK - param;
  switch (adim)
  {
  case 101:
   ledcWrite(kanal1, alcak_duty);  // 3048
   ledcWrite(kanal2, alcak_duty);  // 3048
   ledcWrite(kanal3, yuksek_duty); // 1048
   break;
  case 1:
   ledcWrite(kanal1, yuksek_duty);
   ledcWrite(kanal2, alcak_duty);
   ledcWrite(kanal3, yuksek_duty);
   break;
  case 11:
   ledcWrite(kanal1, yuksek_duty);
   ledcWrite(kanal2, alcak_duty);
   ledcWrite(kanal3, alcak_duty);
   break;
  case 10:
   ledcWrite(kanal1, yuksek_duty);
   ledcWrite(kanal2, yuksek_duty);
   ledcWrite(kanal3, alcak_duty);
   break;
  case 110:
   ledcWrite(kanal1, alcak_duty);
   ledcWrite(kanal2, yuksek_duty);
   ledcWrite(kanal3, alcak_duty);
   break;
  case 100:
   ledcWrite(kanal1, alcak_duty);
   ledcWrite(kanal2, yuksek_duty);
   ledcWrite(kanal3, yuksek_duty);
   break;
  }
 }
 if (motor_yonu == 1)
 {
  yuksek_duty = COZUNURLUK - param;
  alcak_duty = COZUNURLUK + param;
  switch (adim)
  {
  case 101:
   ledcWrite(kanal1, alcak_duty);  // 3048
   ledcWrite(kanal2, alcak_duty);  // 3048
   ledcWrite(kanal3, yuksek_duty); // 1048
   break;
  case 1:
   ledcWrite(kanal1, yuksek_duty);
   ledcWrite(kanal2, alcak_duty);
   ledcWrite(kanal3, yuksek_duty);
   break;
  case 11:
   ledcWrite(kanal1, yuksek_duty);
   ledcWrite(kanal2, alcak_duty);
   ledcWrite(kanal3, alcak_duty);
   break;
  case 10:
   ledcWrite(kanal1, yuksek_duty);
   ledcWrite(kanal2, yuksek_duty);
   ledcWrite(kanal3, alcak_duty);
   break;
  case 110:
   ledcWrite(kanal1, alcak_duty);
   ledcWrite(kanal2, yuksek_duty);
   ledcWrite(kanal3, alcak_duty);
   break;
  case 100:
   ledcWrite(kanal1, alcak_duty);
   ledcWrite(kanal2, yuksek_duty);
   ledcWrite(kanal3, yuksek_duty);
   break;
  }
 }
}

static void hareket_kontrol(void *arg)
{
 // int a = 0;
 while (1)
 {
  vTaskDelay(1 / portTICK_RATE_MS);

  eeprom_kontrol();
  while (ilk_kapanma == false)
  {
   vTaskDelay(1 / portTICK_RATE_MS);
   if (digitalRead(stop_pini) == 1)
   {
    ilk_kapi_kapanma();
   }

   // adim_sayisi = 0;
  }

  if (acil_stop_flag == false)
  {

   if (hareket_sinyali == kapi_ac_sinyali)
   {
    kapi_ac_fonksiyonu();
   }
   if (hareket_sinyali == kapi_kapat_sinyali)
   {
    kapi_kapa_fonksiyonu();
   }
   if (baski_flag)
   {
    if (ble_data_geldi == true)
    {
     eeproma_yaz_istegi = 1;
     ble_data_geldi = false;
    }

    s1_state = digitalRead(encodera);
    s2_state = digitalRead(encoderb);
    s3_state = digitalRead(encoderc);
    adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));
    eski_adim = adim;

    if (adim_sayisi > 25 && rpm == 0 && amper < 0.600) // kapı ihtiyaç duyunca baskı duty ı arttıracak
    {
     baski_duty++;
    }

    if (baski_duty > kapama_baski_gucu) // uygulamadna baskı gücü azaltırlırsa burada bizde azaltıyoruz ve baskı dutynın uçmasını engelliyoruz.
     baski_duty = kapama_baski_gucu;

    motor_surme(baski_duty);
    vTaskDelay(10 / portTICK_RATE_MS);

    if (adim_sayisi > acma_siniri && push_run_flag == true)
    {
     ac_flag = true;           // aç sinyalinin çalışması için falg aktif edilir
     bluetooth_kapi_ac = true; // kapıya aç sinyali gönder
     baski_flag = false;       // baskıyı bitir
    }
    if (mod_flag == cift_kanat && (adim_sayisi < 30)) // kapanaırken kilit noktasndan sonrasına geldiğini servere saöyluyor
    {
     server_data[0] = 121;
     server_data[1] = 123;
     server_data[client_kilit_index] = 1;

     pCharacteristic_2->setValue(server_data, sizeof(server_data));
    }
   }
  }

  // else if (acil_stop_flag == true && adim_sayisi < hizlanma_boy_baslangici)
  // {
  // if (baski_flag)
  // {
  //  s1_state = digitalRead(encodera);
  //  s2_state = digitalRead(encoderb);
  //  s3_state = digitalRead(encoderc);
  //  adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));
  //  eski_adim = adim;
  //  motor_surme(kapama_baski_gucu);
  //  vTaskDelay(10 / portTICK_RATE_MS);

  //  if (adim_sayisi > acma_siniri && push_run_flag == true)
  //  {
  //   ac_flag = true;
  //   bluetooth_kapi_ac = true;
  //   baski_flag = false;
  //  }
  //  if (mod_flag == cift_kanat && (adim_sayisi < 200))
  //  {
  //   server_data[0] = 121;
  //   server_data[1] = 123;
  //   server_data[client_kilit_index] = 1;

  //   pCharacteristic_2->setValue(server_data, sizeof(server_data));
  //  }
  // }
  // }
 }
}

static void seri_oku(void *arg)
{
 Serial.println("Seri Oku Fn: Basladi");

 int alinan;
 while (1)
 {
  vTaskDelay(500 / portTICK_RATE_MS);
  alinan = 0; //  Alınan değer için kullanılan değişken burada sıfırlanmıştır.
  if (Serial.available() > 0)
  {
   alinan = Serial.parseInt();
   Serial.print("Seri Oku: Alinan = ");
   Serial.println(alinan);

   switch (alinan)
   {

   case 100:

    break;
   case 101:
    /*
eRunning = 0,	/*!< A task is querying the state of itself, so must be running.
 eReady,			/*!< The task being queried is in a read or pending ready list.
 eBlocked,		/*!< The task being queried is in the Blocked state.
 eSuspended,		/*!< The task being queried is in the Suspended state, or is in the Blocked state with an infinite time out.
 eDeleted		!< The task being queried has been deleted, but its TCB has not yet been freed.

*/
    Serial.print("ble_arg : ");
    Serial.println(eTaskGetState(ble_arg));

    Serial.print("ac_task_arg : ");
    Serial.println(eTaskGetState(ac_task_arg));

    Serial.print("seri_oku_arg : ");
    Serial.println(eTaskGetState(seri_oku_arg));

    Serial.print("rpm_olcum_arg : ");
    Serial.println(eTaskGetState(rpm_olcum_arg));

    Serial.print("dc_bara_arg : ");
    Serial.println(eTaskGetState(dc_bara_arg));

    Serial.print("hareket_kontrol_arg : ");
    Serial.println(eTaskGetState(hareket_kontrol_arg));

    Serial.print("status_led_arg : ");
    Serial.println(eTaskGetState(status_led_arg));

    Serial.print("motor_akim_oku_arg : ");
    Serial.println(eTaskGetState(motor_akim_oku_arg));

    Serial.print("led_kontrol_arg : ");
    Serial.println(eTaskGetState(led_kontrol_arg));

    // Serial.print("adim_hata_counter : ");
    // Serial.println(adim_hata_counter);

    Serial.print("Hesaplanan  duty : ");
    Serial.println(hesaplanan);

    Serial.print("duty deger : ");
    Serial.println(duty);

    Serial.print("hareket_sinyali : ");
    Serial.println(hareket_sinyali);

    Serial.print("baski_flag : ");
    Serial.println(baski_flag);

    Serial.print("kapi_ac_sayac : ");
    Serial.println(kapi_ac_sayac);

    Serial.print("kapama_baski_gucu : ");
    Serial.println(kapama_baski_gucu);

    Serial.print("carpan : ");
    printf("carpan : %f\n", carpan);

    Serial.print("amper : ");
    Serial.println(amper);

    Serial.print("analog : ");
    Serial.println(analog);

    Serial.print("amper_siniri_func : ");
    Serial.println(amper_siniri_func());

    Serial.print("rpm : ");
    Serial.println(rpm);

    Serial.print("Max_RPM : ");
    Serial.println(Max_RPM);
       Serial.print("kapama_max_rpm : ");
    Serial.println(kapama_max_rpm);

    Serial.print("maksimum_kapi_boyu : ");
    Serial.println(maksimum_kapi_boyu);

    kapi_acma_derecesi = EEPROM.read(20);
    kapi_acma_derecesi = kapi_acma_derecesi * 2;

    Serial.print("kapi_acma_derecesi : ");
    Serial.println(kapi_acma_derecesi);

    Serial.print("voltaj deger : ");
    Serial.println(voltaj);

    Serial.print("adim_sayisi : ");
    Serial.println(adim_sayisi);

    Serial.print("hedef_sure : ");
    Serial.println(hedef_sure);

    Serial.print("tanima_hizi_flag : ");
    Serial.println(tanima_hizi_flag);

    Serial.print("bobin_fark_sure : ");
    Serial.println(bobin_fark_sure);

    Serial.print("derece : ");
    Serial.println(derece);

    Serial.print("ilk_kapanma : ");
    Serial.println(ilk_kapanma);

    Serial.print("tanima_hizi_flag : ");
    Serial.println(tanima_hizi_flag);

    Serial.print("select_tr_mission : ");
    Serial.println(select_tr_mission);
    break;
   case 102:
    Serial.println("tek kanat moduna alindi..");
    EEPROM.write(79, tek_kanat);
    mod = tek_kanat;
    eeproma_yaz_istegi = 1;
    break;
   case 103:
    Serial.println("cift kanat moduna alindi..");
    EEPROM.write(79, cift_kanat);
    mod = cift_kanat;
    eeproma_yaz_istegi = 1;
    break;

   case 113:
    // Serial.println("Seri Oku: Demo baslatildi"); //  Fuarlar için gereklidir.
    break;
   case 114:
    Serial.println("menteşe yonu değişti");
    Serial.print("menteşe yonu : ");
    Serial.println(mentese_yonu);
    mentese_yonu = (!mentese_yonu);

    if (mentese_yonu == 1)
    {
     CW = -1;
     CCW = 1;
    }
    else
    {
     CW = 1;
     CCW = -1;
    }

    break;
   case 115:
    if (hareket_sinyali != kapi_ac_sinyali)
    {
     bluetooth_kapi_ac = true;
     ac_flag = true;
    }

    zaman_timeout = 0;
    Serial.println("Seri Oku: Kapi ac");
    break;
   case 116:
    Serial.println("Seri Oku: ESP Reset");
    delay(500);
    esp_restart();
    break;
   case 117:
    Serial.println("Seri Oku: Kalibrasyon");
    kalibrasyon_aktif = 1;
    kalibrasyon_fn();
    break;
   case 122:
    sayac_sifirla_fn();
    break;
   case 123:
    Serial.println("seri kapi kapat");

    bluetooth_kapi_kapa = true;
    kapat_flag = true;
    break;
   case 124: // test modunu aktif et
    test_func();
    // test_modu_aktif = 1;
    // EEPROM.write(73, test_modu_aktif);
    // eeproma_yaz_istegi = 1;
    // vTaskDelay(1000 / portTICK_RATE_MS);
    // esp_restart();
    break;
   case 125:
    vTaskSuspend(seri_yazdir_arg);
    break;
   case 126:
    vTaskResume(seri_yazdir_arg);
    break;
   }
  }
 }
}
void DutyHesaplama()
{
 if(rpm>0)rpmTespit=0;
   float gain_scale = 5;
  if ((adim_sayisi < 100) && (hareket_sinyali==kapi_ac_sinyali)) gain_scale = 1;
  if ((adim_sayisi > 100) && (hareket_sinyali==kapi_ac_sinyali)) gain_scale = 15;
  if ((adim_sayisi > 200) && (hareket_sinyali==kapi_ac_sinyali)) gain_scale = 5;
  // if (rpm < 150) gain_scale = 2;
sure_integral=gain_scale;
  double hata = (hedef_sure - bobin_fark_sure);
  // if (fabs(hata) < 0)
  //     hata = 0;

  double delta = -(hata * duty_Kp * gain_scale)
                 - ((hata - eski_hata) * Kd * gain_scale);
                 /*- amper_siniri_func()
                 - ((!digitalRead(fault)) * duty * 0.1);*/
  eski_hata = hata;

  duty += delta;

  if (duty < min_duty) duty = min_duty;
  if (duty > max_duty) duty = max_duty;

  // İsteğe bağlı filtreleme
  static float duty_filtered = min_duty;
  duty_filtered = duty_filtered * 0.8f + duty * 0.2f;
  hesaplanan=duty_filtered;
  motor_surme(duty_filtered);
}
void MotorBekletme()
{
    static double hata_integral = 0;
    static double hata_eskisi = 0;

  // PID hesapları
  double hata = (bobin_fark_sure - hedef_sure) / hedef_sure;
  hata_integral = constrain(hata_integral + hata, -5.0, 5.0);
  double hata_turev = hata - hata_eskisi;
  hata_eskisi = hata;

  const double Kp = 0.4;
  const double Ki = 0.05;
  const double Kd = 0.15;

  double pid_raw = (Kp*hata + Ki*hata_integral + Kd*hata_turev);

  // Dengeleme: hızlanma az, yavaşlama güçlü
  if (pid_raw > 0)
      pid_raw = pow(pid_raw, 1.4);
  else
      pid_raw = pid_raw * 0.2;

  // Dinamik limitler
  double min_sure = hedef_sure * 0.4;
  double max_sure = hedef_sure * 2.0;

  // Ek güvenlik offseti
  double sure_pid = hedef_sure * (0.2 + (1 + pid_raw) * 0.8);
  sure_pid = constrain(sure_pid, min_sure, max_sure);
  sure_global=sure_pid-bobin_fark_sure;
  // Zaman bekleme
 unsigned long t0 = micros();
 while (micros() - t0 < (sure_pid - bobin_fark_sure)) {
     delayMicroseconds(100);
 }

}
void kapi_ac_fonksiyonu()
{
 if (ac_test_aktif == true) // kapatırken baki yediyse kapatırken soket arızasına bakar
 {
  test_func();
  ac_test_aktif = false;
 }

 if (bobin_durumu < 4) // bobin kesmesi geldiğinde gerekli işlemleri taskın içersinde yapacak
 {
  bobin_durumu = 5;
  bobin_fark_sure = bobin_yeni_sure - bobin_eski_sure;
  bobin_eski_sure = bobin_yeni_sure;
  if (bobin_fark_sure < (-hedef_sure))
  {
   bobin_fark_sure = (-hedef_sure);
  }
  int32_t adim_tasi = adim_sayisi;

  if (adim_tasi < 0)
  {
   adim_tasi = 0;
  }
  if (tanima_hizi_flag == true) // kapı açılırken time outa uğrarsa tanıma hızında ilerleyecek
  {
   Max_RPM = tanima_hizi;
   hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi);
  }
  else
  {
   Max_RPM = hedefRPMharitasi[adim_tasi];
   hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi);
  }
  if (hedef_sure == INFINITY)
  {
   hedef_sure = 10000;
  }
  if (adim_sayisi < maksimum_kapi_boyu) // aç işlemi esansında devreye girer
  {
   if (hedef_sure > bobin_fark_sure)
   {
    hata = (hedef_sure - bobin_fark_sure);
    if (hata > 20000)
    {
     hata = 20000;
    }
    //duty = duty - (hata * duty_Kp + (hata - eski_hata) * Kd) - amper_siniri_func() - ((!digitalRead(fault)) * duty * 0.1); //+ ((hata - eski_hata) * (100.0 / 5000.0)));
   DutyHesaplama();


    MotorBekletme();


   }
   if (hedef_sure < bobin_fark_sure)
   {
    hata = (hedef_sure - bobin_fark_sure);
    if (hata > 20000)
    {
     hata = 20000;
    }

    // duty = duty - (hata * duty_Kp + (hata - eski_hata) * Kd) - amper_siniri_func() - ((!digitalRead(fault)) * duty * 0.1); //+ ((hata - eski_hata) * (100.0 / 5000.0)));
    // eski_hata = hata;
     
    DutyHesaplama();
   }
  }
 }
 if (rpm == 0 && adim_sayisi < maksimum_kapi_boyu && tanima_hizi_flag == false && faulttan_kapata == false) // motor durduğunda tekrar ilk hareketin sağlanması için
 {
  xTaskCreate(motor_ilk_tahrik, "motor_ilk_tahrik", 2048, NULL, 3, NULL);
  vTaskDelay(10 / portTICK_RATE_MS);
  int32_t zorlama_adim_sayisi = adim_sayisi;
  uint16_t count = 0;
  Serial.println(" acarken  timeout a girildi");
  // PrintLog("acarken+timeout+a+girildi");
  while (hareket_sinyali != kapi_kapat_sinyali)
  {

   if (adim_sayisi < hizlanma_boy_baslangici + 150 && count > 5) // iki kanatlıda baskı yediği zaman diğer kanata da dur diyor
   {
    client_data[client_dur_index] = 1;
   }

   count++;
   //    duty=motor_baslangic_duty;
   // bobin_eski_sure=micros();
   xTaskCreate(motor_ilk_tahrik, "motor_ilk_tahrik", 2048, NULL, 3, NULL);
   vTaskDelay(10 / portTICK_RATE_MS);
   if (adim_sayisi > zorlama_adim_sayisi + 5) // motor tahrik verildiğinde ilerliyorsa döngüden çıkacak
   {
    Serial.println();
    Serial.println("kapi ilerledi..");
    rpmTespit=0;
    // duty = motor_baslangic_duty;
    client_data[client_ac_index] = 1;
    break;
   }
   if (count > (kapama_baski_suresi / 10)) // 1.5 sn bekleyecek açılamazsa kapata geçecek
   {
    Serial.println("acilirken time out a gitti ..");
    kapat_test_aktif = true;
    tanima_hizi_flag = true;
    kapanma_error_flag = false;
    hesaplanan = bekleme_duty;
    motor_surme(bekleme_duty);
    PrintLog("acilirken+time+out+a+gitti");
    // vTaskDelay(1000 / portTICK_RATE_MS);
    kapi_basarisiz_ac_sayac++;
    kapi_ac_sayac--;
    // actan_kapata = true;
    kapat_flag = true;
    // eeproma_yaz_istegi = 1;
    baski_led_flag = true;
    break;
   }
  }
 }

 /***************************
  * (adim_sayisi >= maksimum_kapi_boyu && digitalRead(ac_pini) == 0) aç sinyali kesiline ve kapı boyu maksimumdan fazla ise kapanmaya gider
  * (tanima_hizi_flag==true && digitalRead(ac_pini) == 0) timeout(engel/hata) uğrayınca ve aç sinyali kesilince kapanmaya gider
  * **********************************/

 if ((adim_sayisi >= maksimum_kapi_boyu && digitalRead(ac_pini) == 0) ||
     (tanima_hizi_flag == true && digitalRead(ac_pini) == 0) || // açılırken engel algılandığında time outa uğramamsı için
     faulttan_kapata == true)
 //(fault_siniri>0 && digitalRead(ac_pini) == 0)) // açma - durdurma işlemi.
 {
  Serial.println("kapanma 1");
  PrintLog("kapanma+1");
  faulttan_kapata = false; // faulttan sonra ac sinyali kesilince bu if e girdi.bayrağı sıfırlıyoruz
  if (mod_flag == cift_kanat)
  {
   server_data[0] = 121;
   server_data[1] = 123;
   server_data[client_kilit_index] = 0;

   pCharacteristic_2->setValue(server_data, sizeof(server_data));
  }
  fault_siniri = 0;             // fault ledi kesmesi sistemi durdurma sayacı sıfırlandı
  kapattan_aca = false;         // ac tamamlanınca kapattan aca flag sıfırlanıyor
  memset(bobin_ortalama, 0, 6); // başlangıçta ortalama değerleri tutan dizi sıfırlandı.
  test_flag = true;
  adim_oku();
  motor_surme(bekleme_duty);
  vTaskDelay(1 / portTICK_RATE_MS);
  Serial.print("adim_sayisi : ");
  Serial.println(adim_sayisi);
  tanima_hizi_flag = false;
  kapanma_error_flag = false; // kapanma hatası sıfırlanıyor
  // duty = tanima_hizi;
  kapi_ac_sayac++;
  if (calisma_yontemi == otomatik_kapan) // otomatik
  {
   zaman_timeout = 0;
   Serial.println("time out basladi");
   PrintLog("time+out+basladi");
   while (adim_sayisi >= (maksimum_kapi_boyu-10))
   {
    zaman_timeout++;
    if (zaman_timeout <= acik_kalma_suresi)
    {
     vTaskDelay(10 / portTICK_RATE_MS);
     if (digitalRead(ac_pini) == 1 || digitalRead(asansor_ac_pini) == 1)
     {
      zaman_timeout = 0;
     }
    }
    else
    {
     break;
    }
   }
   if (mod == cift_kanat)
   {
    client_data[client_kapa_index] = 1;
    vTaskDelay(2500 / portTICK_RATE_MS);
   }
   Serial.println("time out bitti");
   PrintLog("time+out+bitti");
   hareket_sinyali = kapi_kapat_sinyali;
   kapat_time_out_flag = true;
  }
  else if (calisma_yontemi == ac_kapat) // || calisma_yontemi == ac_ac) //kapat sinyalini bekle
  {
   hareket_sinyali = kapi_bosta_sinyali;
   Serial.println("calisma yontemi ac_kapat..");
   PrintLog("calisma+yontemi+ac_kapat");
   Serial.println("kapat bekleniyor..");
  }
  else if (calisma_yontemi == ac_ac)
  {

   hareket_sinyali = kapi_bosta_sinyali;
   Serial.println("calisma yontemi ac_ac..");
   PrintLog("calisma+yontemi+ac_ac");
   Serial.println("kapat bekleniyor..");
  }
 }

 if (actan_kapata == true && hareket_sinyali != kapi_bosta_sinyali) // kapat hareketi aç içersinde verildiyse
 {
  Serial.println("actan kapata gidildi..");
  PrintLog("actan+kapata+gidildi");
  actan_kapata = true; // kapı acılırken kapa verildiyse true edilmesi gerekiyor

  for (int16_t i = hesaplanan; i > 100; i = i - 10)//yumuşak duruş
  {
   Serial.println(i);
   s1_state = digitalRead(encodera);
   s2_state = digitalRead(encoderb);
   s3_state = digitalRead(encoderc);
   adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));
   motor_surme(i);
   vTaskDelay(5 / portTICK_RATE_MS);
  }
  hesaplanan = bekleme_duty;
  hareket_sinyali = kapi_kapat_sinyali;
  vTaskDelay(1000 / portTICK_RATE_MS);
 }
}

void kapi_kapa_fonksiyonu()
{
 if (kapat_test_aktif == true) // açarken baski yediyse kapatırken soket arızasına bakar
 {
  test_func();
  kapat_test_aktif = false;
 }

 if (mod == cift_kanat && adim_sayisi < 250 && server_data[client_kilit_index] == 0)//server klit bölgesine gelmediyse
 {
  motor_surme(200);
  while (server_data[client_kilit_index] == 0 && ac_flag == false)//ben kilit bölgesine geldim
  {
   vTaskDelay(100 / portTICK_RATE_MS);
   client_data[client_kapa_index] = 1;
  }
 }
 if (mod_flag == cift_kanat && (adim_sayisi < 200))//klit bölgesine geldiysen geldiğini söyle
 {
  server_data[0] = 121;
  server_data[1] = 123;
  server_data[client_kilit_index] = 1;

  pCharacteristic_2->setValue(server_data, sizeof(server_data));
 }

 if (kapat_time_out_flag)
 {
  if (kapanirken_engel_algiladi_flag)
  {
   kapanirken_engel_algiladi_flag = false;
   rpm_haritasi_olustur_engelli_kapat();
  }
  else
  {
   kapi_kapat_hazirlik();
  }

  if (actan_kapata == true) // aca giderken kapat verildiyse rpm haritasi güncelleniyor
  {
   actan_kapata = false;
   int hiz_aktar = 200;
   int32_t adim_tasi = adim_sayisi - 200;
   if (adim_tasi < 0)
   {
    adim_tasi = 0;
   }
   for (uint16_t i = adim_sayisi; i > adim_tasi; i--)
   {
    varilacak_hiz = hedefRPMharitasi_kapa[adim_tasi];
    hiz_aktar = hiz_aktar + (varilacak_hiz - 200) / (200);
    hedefRPMharitasi_kapa[i] = hiz_aktar;
   }
   Serial.println("kapanma rpm haritasi guncellendi..");
   PrintLog("kapanma+rpm+haritasi+guncellendi");
  }

  kapat_time_out_flag = false;
  Serial.println("kapat basladi");
  PrintLog("kapat+basladi");
  if (mentese_yonu == 0)
  {
   motor_yonu = kapi_kapa;
  }
  else
  {
   motor_yonu = (!kapi_kapa);
  }
  int32_t adim_tasi = adim_sayisi;
  if (adim_tasi < 0)
  {
   adim_tasi = 0;
  }
  Max_RPM = hedefRPMharitasi_kapa[adim_tasi]; // hedefRPMharitasi_kapa[adim_tasi];
  hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi);
  if (hedef_sure == INFINITY)
  {
   hedef_sure = 10000;
  }

  xTaskCreate(motor_ilk_tahrik, "motor_ilk_tahrik", 2048, NULL, 3, NULL);
  vTaskDelay(10 / portTICK_RATE_MS);
 }
 if (bobin_durumu < 4) // bobin kesmesi geldiğinde gerekli işlemleri taskın içersinde yapacak
 {
  bobin_durumu = 5;
  bobin_fark_sure = bobin_yeni_sure - bobin_eski_sure;
  bobin_eski_sure = bobin_yeni_sure;
  if (bobin_fark_sure < (-hedef_sure))
  {
   bobin_fark_sure = (-hedef_sure);
  }

  int32_t adim_tasi = adim_sayisi;

  if (adim_tasi < 0)
  {
   adim_tasi = 0;
  }
  Max_RPM = hedefRPMharitasi_kapa[adim_tasi]; // hedefRPMharitasi[adim_tasi];
  hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi);
  if (hedef_sure == INFINITY)
  {
   hedef_sure = 10000;
  }

  if (hedef_sure > bobin_fark_sure)
  {
   hata = (hedef_sure - bobin_fark_sure);
  DutyHesaplama();

   MotorBekletme();

  }

  if (hedef_sure < bobin_fark_sure)
  {

   hata = (hedef_sure - bobin_fark_sure);

  DutyHesaplama();

  }
 }

 if (rpm == 0 && digitalRead(fault) == 1) // motor durduğunda tekrar ilk hareketin sağlanması için
 {
  Max_RPM = hedefRPMharitasi_kapa[adim_sayisi]; // hedefRPMharitasi[adim_tasi];
  hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi);

  Serial.println("rpm sifir tespit edildi..");
  rpmTespit++;
  if(rpmTespit<10){

  if (adim_sayisi < kapanma_guncelleme_noktasi) // belirli bir adımın altında baskı yerse kapı kapanma noktasını gümcelletiriyoruz
  {
   Serial.println("kapat kilit sonrasi timeout a girildi..");
   // PrintLog("kapat+kilit+sonrasi+timeout+a+girildi");
   int32_t zorlama_adim_sayisi = adim_sayisi;
   uint16_t count = 0;
   while (ac_flag == false)
   {
    count++;
    xTaskCreate(motor_ilk_tahrik, "motor_ilk_tahrik", 2048, NULL, 3, NULL);
    vTaskDelay(10 / portTICK_RATE_MS);
    if ((adim_sayisi < (zorlama_adim_sayisi - 5)) || ac_flag == true) // motor tahrik verildiğinde ilerliyorsa döngüden çıkacak
    {
     Serial.println();
     Serial.print("kapi ilerledi : ");
     rpmTespit=0;
     // duty = motor_baslangic_duty;
     // PrintLog("kapi+ilerledi");
     // vTaskDelay(10 / portTICK_RATE_MS);
     break;
    }

    if (count > (kapama_baski_suresi / 10)) // 1.5 sn bekleyecek açılamazsa kapata geçecek
    {
     Serial.println();
     Serial.println("===========Kapi kapanma noktasi guncellendi==============");
     fault_siniri = 0; // fault ledi kesmesi sistemi durdurma sayacı sıfırlandı
     baski_duty = kapanma_baski_gucu_min;
     memset(bobin_ortalama, 0, 6); // başlangıçta ortalama değerleri tutan dizi sıfırlandı.
     adim_sayisi = 0;
     duty = bekleme_duty;

     s1_state = digitalRead(encodera);
     s2_state = digitalRead(encoderb);
     s3_state = digitalRead(encoderc);
     adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));

     motor_surme(bekleme_duty);
     vTaskDelay(1 / portTICK_RATE_MS);
     PrintLog("Kapi+kapanma+noktasi+guncellendi");
     kapanma_error_flag = false;
     baski_flag = true;
     if (hareket_sinyali == kapi_kapat_sinyali)
     {
      aydinlatma_led_state = 0;
     }
     hareket_sinyali = kapi_bosta_sinyali;
     kapi_kapat_sayac++;
     eeproma_yaz_istegi = 1;
     break;
    }
   }
  }
  if (adim_sayisi >= kapanma_guncelleme_noktasi && konsol_aktif_flag == false && digitalRead(fault) == 1)
  {
   Serial.println(" kapat kilit oncesi timeout");
   // PrintLog("kapat+kilit+oncesi+timeout");
   int32_t zorlama_adim_sayisi = adim_sayisi;
   uint16_t count = 0;
   while (ac_flag == false)
   {
    if (count > 5)
    {
     if (adim_sayisi < hizlanma_boy_baslangici + 150)
     {
      client_data[client_dur_index] = 1;
     }
    }

    count++;
    // bobin_eski_sure=micros();
    // duty=motor_baslangic_duty;
    xTaskCreate(motor_ilk_tahrik, "motor_ilk_tahrik", 2048, NULL, 3, NULL);
    vTaskDelay(10 / portTICK_RATE_MS);
    if (adim_sayisi < (zorlama_adim_sayisi - 5) || ac_flag == true) // motor tahrik verildiğinde ilerliyorsa döngüden çıkacak
    {
     Serial.println();
     rpmTespit=0;
     Serial.print("kapi 5 adim ilerledi : ");
     client_data[client_kapa_index] = 1;
     // PrintLog("kapı+5+adım+ilerledi");
     // vTaskDelay(10 / portTICK_RATE_MS);
     break;
    }

    if (count > (kapama_baski_suresi / 10)) // ayarlanan  sn kadar bekleyecek kapatılamazsa aça geçecek
    {
     Serial.println("kapatta iken kilit oncesi  time out girildi");
     client_data[client_ac_index] = 1;
     // vTaskDelay(1500 / portTICK_RATE_MS);
     ac_test_aktif = true;
     /**********kapı kapatırken time outa girerse aç sinyali göndeririyoruz***************/
     hareket_sinyali = kapi_bosta_sinyali;
     ac_flag = true;
     bluetooth_kapi_ac = true;
     engel_algilandi_flag = true;
     kapanirken_engel_algiladi_flag = true;
     baski_adimi = adim_sayisi;
     /*************************************/
     Max_RPM = tanima_hizi;
     hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi);
     s1_state = digitalRead(encodera);
     s2_state = digitalRead(encoderb);
     s3_state = digitalRead(encoderc);
     adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));

     motor_surme(50);
     vTaskDelay(10 / portTICK_RATE_MS);
     kapi_basarisiz_kapat_sayac++;
     kapi_kapat_sayac--;
     baski_led_flag = true;
     break;
    }
   }
  }
 
  }
 }

 if ((ac_flag == true && calisma_yontemi != ac_ac && engel_algilandi_flag == false) ||
     (ac_ac_flag_kapama == true && engel_algilandi_flag == false))
 {
  ac_ac_flag_kapama = false;
  kapattan_aca = true; // kapı kapanırken ac verildiyse true edilmesi gerekiyor
  for (int16_t i = hesaplanan; i > 100; i = i - 10)
  {
   Serial.println(i);
   s1_state = digitalRead(encodera);
   s2_state = digitalRead(encoderb);
   s3_state = digitalRead(encoderc);
   adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));
   motor_surme(i);
   vTaskDelay(5 / portTICK_RATE_MS);
  }
  Serial.println("kapatta iken ac sinyali geldi..");
  kilit_state = 1; // baskıda iken klit aççmıyordu o yüzzden koyduk...
  PrintLog("kapatta+iken+ac+sinyali+geldi");
  if (mentese_yonu == 0) // mentese değişenine göre yönun terslendği yer
  {
   motor_yonu = kapi_ac;
  }
  else
  {
   motor_yonu = (!kapi_ac);
  }
  int hiz_aktar = 200;
  kapi_ac_hazirlik();
  for (uint16_t i = adim_sayisi; i < adim_sayisi + 200; i++) // kapat işlemi yapılırken aça basılınca rpm haritasi yeniden hesaplanıyor
  {
   varilacak_hiz = hedefRPMharitasi[adim_sayisi + 200];
   hiz_aktar = hiz_aktar + (varilacak_hiz - 200) / (200);
   hedefRPMharitasi[i] = hiz_aktar;
  }
  Serial.println("rpm haritası güncellendi....");
  hesaplanan = bekleme_duty;
  kapattan_aca = false;
  hareket_sinyali = kapi_ac_sinyali;
  ac_flag = false;
  vTaskDelay(1000 / portTICK_RATE_MS);
 }
 engel_algilandi_flag = false; // yumuşatarak durma geçtiği iin tekrar false yaptık
}

void ilk_kapi_kapanma()
{

 if (ilk_kapama_flag)
 {
  /*her açılışta soket kontrol edeilecek*/
  // if (test_modu_aktif == 1)
  // {
  // test_func();
  // test_modu_aktif = 0; //  test modu kapatiliyor
  // EEPROM.write(73, test_modu_aktif);
  // eeproma_yaz_istegi = 1;
  // }
  adim_oku();
  Max_RPM = 350; // hedefRPMharitasi_kapa[adim_sayisi]; // hedefRPMharitasi_kapa[adim_tasi];
  hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi);
  Serial.print("hedef_sure : ");
  Serial.println(hedef_sure);
  // digitalWrite(bl_led, HIGH);
  //  aydinlatma_led_state = 1;
  ilk_kapama_flag = false;
  Serial.println("kapat basladi");
  PrintLog("kapat+basladi");
  if (mentese_yonu == 0)
  {
   motor_yonu = kapi_kapa;
  }
  else
  {
   motor_yonu = (!kapi_kapa);
  }
  if (hedef_sure == INFINITY)
  {
   hedef_sure = 10000;
  }

  xTaskCreate(motor_ilk_tahrik, "motor_ilk_tahrik", 2048, NULL, 3, NULL);
  vTaskDelay(10 / portTICK_RATE_MS);
 }

 if ((millis() - ilk_kapanma_suresi) > ilk_kapanma_suresi_def)
 {
  ilk_kapama_flag = true;
  adim_sayisi = 0;
  ilk_kapanma = true;
  motor_surme(200);
  Serial.println();
  Serial.println("====ILK KAPANMA YAPILDI ZAMAN ASIMI ILE======");
  PrintLog("ILK+KAPANMA+YAPILDI+ZAMAN+ASIMI+ILE");
  attachInterrupt(digitalPinToInterrupt(ac_pini), pin_kesmesi_ac, RISING);                 //  Kapı açmak için interrupt eklendi ve tanımlandı.
  attachInterrupt(digitalPinToInterrupt(kapat_pini), pin_kesmesi_kapa, RISING);            //  Kapı açmak için interrupt eklendi ve tanımlandı.
  attachInterrupt(digitalPinToInterrupt(asansor_ac_pini), pin_kesmesi_asansor_ac, CHANGE); //

  aydinlatma_led_state = 0;
  baski_flag = true;
  hareket_sinyali = kapi_bosta_sinyali;

  test_flag = true;
 }

 if (bobin_durumu < 4) // bobin kesmesi geldiğinde gerekli işlemleri taskın içersinde yapacak
 {
  Max_RPM = 350; //ilk kapanma hızı
  hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi);

  bobin_durumu = 5;
  bobin_fark_sure = bobin_yeni_sure - bobin_eski_sure;
  bobin_eski_sure = bobin_yeni_sure;
  if (hedef_sure > bobin_fark_sure)
  {
   hata = (hedef_sure - bobin_fark_sure);
   duty = duty - (hata * 0.03) - amper_siniri_func() - ((!digitalRead(fault)) * duty * 0.1); //+ ((hata - eski_hata) * (100.0 / 5000.0)));
                                                                                             // eski_hata = hata;
   if (duty < min_duty)
   {
    duty = min_duty;
   }
   if (duty > max_duty)
   {
    duty = max_duty;
   }

   hesaplanan = duty + ((set_voltage - voltaj) * set_volatage_factor / set_voltage);

   if (hesaplanan > max_duty + ((set_voltage - voltaj) * set_volatage_factor / set_voltage))
   {
    hesaplanan = max_duty + ((set_voltage - voltaj) * set_volatage_factor / set_voltage);
   }

   if (hesaplanan < hesaplanan_min_duty)
   {
    hesaplanan = hesaplanan_min_duty;
   }

   eski_hata = hata;

   motor_surme(hesaplanan);
   vTaskDelay(1 / portTICK_RATE_MS);
   double eski = 0, zaman = 0;
   double sure = hedef_sure - bobin_fark_sure;
   eski = micros();
   zaman = micros() - eski;
   sure_global = sure;
   while (zaman < sure)
   {
    zaman = micros() - eski;
    vTaskDelay(1 / portTICK_RATE_MS);
   }
  }
  if (hedef_sure < bobin_fark_sure)
  {
   hata = (hedef_sure - bobin_fark_sure);
   duty = duty - (hata * duty_Kp) - amper_siniri_func() - ((!digitalRead(fault)) * duty * 0.1); // + ((hata - eski_hata) * (100.0 / 5000.0)));

   if (duty > max_duty)
   {
    duty = max_duty;
   }
   if (duty < min_duty)
   {
    duty = min_duty;
   }
   hesaplanan = duty + ((set_voltage - voltaj) * set_volatage_factor / set_voltage);

   if (hesaplanan > max_duty + ((set_voltage - voltaj) * set_volatage_factor / set_voltage))
   {
    hesaplanan = max_duty + ((set_voltage - voltaj) * set_volatage_factor / set_voltage);
   }

   if (hesaplanan < hesaplanan_min_duty)
   {
    hesaplanan = hesaplanan_min_duty;
   }

   motor_surme(hesaplanan);
   vTaskDelay(1 / portTICK_RATE_MS);
  }
 }
 if (rpm == 0)
 {

  baslangic = millis();
  int32_t zorlama_adim_sayisi = adim_sayisi;
  ilk_kapanma_count = 0;
  Serial.println("rpm sifir tespit edildi..");
  while (1)
  {

   xTaskCreate(motor_ilk_tahrik, "motor_ilk_tahrik", 2048, NULL, 3, NULL);
   ilk_kapanma_count++;
   vTaskDelay(10 / portTICK_RATE_MS);
   if (ilk_kapanma_count > (kapama_baski_suresi / 10))
   {

    adim_sayisi = 0;
    ilk_kapanma = true;
    motor_surme(200);
    Serial.println();
    Serial.println("====ILK KAPANMA YAPILDI======");
    PrintLog("ILK+KAPANMA+YAPILDI");
    attachInterrupt(digitalPinToInterrupt(ac_pini), pin_kesmesi_ac, RISING);      //  Kapı açmak için interrupt eklendi ve tanımlandı.
    attachInterrupt(digitalPinToInterrupt(kapat_pini), pin_kesmesi_kapa, RISING); //  Kapı açmak için interrupt eklendi ve tanımlandı.
    attachInterrupt(digitalPinToInterrupt(asansor_ac_pini), pin_kesmesi_asansor_ac, CHANGE);
    aydinlatma_led_state = 0;
    baski_flag = true;
    hareket_sinyali = kapi_bosta_sinyali;
    test_flag = true;
    break;
   }
   if ((adim_sayisi < (zorlama_adim_sayisi - 5))) // motor tahrik verildiğinde ilerliyorsa döngüden çıkacak
   {
    Serial.println();
    Serial.print("kapı ilerledi : ");
    rpmTespit=0;
    // vTaskDelay(10 / portTICK_RATE_MS);
    break;
   }
  }
 }
}

void kapi_ac_hazirlik() {
 bobin_fark_sure=0;
 sure=0;
 maksimum_kapi_boyu = (double)(kapi_acma_derecesi * (10000.0 / 821.0))/2.5;
 Max_RPM = EEPROM.read(11);
  Max_RPM =Max_RPM * hiz_katsayisi;
      const float base_rpm = 75.0;       // taban kalkış RPM
    const float max_rpm = Max_RPM;       // açılış üst sınırı
    const float ramp_up_ratio = 0.45;  // ilk %25'te hızlanma
    const float ramp_down_ratio = 0.45;// son %25'te yavaşlama
    const int total_steps = maksimum_kapi_boyu;

    for (int i = 0; i < total_steps; i++)
    {
        float pos = (float)i / total_steps;
        float rpm;

        if (pos < ramp_up_ratio)
        {
            // Yumuşak (S-eğrili) kalkış
            float t = pos / ramp_up_ratio;
            rpm = base_rpm + (max_rpm - base_rpm) * (0.5 - 0.5 * cos(M_PI * t));
        }
        else if (pos < (1.0 - ramp_down_ratio))
        {
            // Sabit hız bölgesi
            rpm = max_rpm;
        }
        else
        {
            // Yavaşlama bölgesi (parabolik azalış)
            float t = (1.0 - pos) / ramp_down_ratio;
            rpm = base_rpm + (max_rpm - base_rpm) * (t * t);
        }

        hedefRPMharitasi[i] = (uint16_t)rpm;
    }
 //    for (uint16_t i = 0; i <= 2000; i++)
 // {
 //  printf("rpm=haritasi : %d : %d  \n ", i, hedefRPMharitasi[i]);
 // }
}

void rpm_haritasi_olustur_engelli_kapat()
{
 if (mod_flag == tek_kanat)
 {
  kapama_max_rpm = EEPROM.read(16);
  kapama_max_rpm = kapama_max_rpm * hiz_katsayisi;
 }

 kapi_acma_derecesi = int(float(adim_sayisi) * 0.0821); // 10000.0; //kapama anında hangi adımda ise o adıma göre rpm harştası oluştursun diye
 Serial.print("kapi_acma_derecesi engelli: ");
 Serial.println(kapi_acma_derecesi);
 // if (adim_sayisi < 300)
 // {
 //  kapama_max_rpm = 300;
 // }

 // kapama_max_rpm = kapama_max_rpm - (maksimum_kapi_boyu - adim_sayisi) / 3;
 // kapama_max_rpm = map(adim_sayisi, 0, maksimum_kapi_boyu, 300, kapama_max_rpm);

 if (kapi_acma_derecesi <= 30 && kapama_max_rpm > 300) // küçük açılarda sıçraama olmasın diye 90 dereceyi belirledik
 {
  kapama_max_rpm = 300;
 }
 if (kapi_acma_derecesi > 30 && kapi_acma_derecesi <= 60 && kapama_max_rpm > 600) // küçük açılarda sıçraama olmasın diye 90 dereceyi belirledik
 {
  kapama_max_rpm = 600;
 }
 if (kapi_acma_derecesi > 60 && kapi_acma_derecesi < 90 && kapama_max_rpm > 1000) // küçük açılarda sıçraama olmasın diye 90 dereceyi belirledik
 {
  kapama_max_rpm = 1000;
 }
 maksimum_kapi_boyu = kapi_acma_derecesi / (82.1 / 1000.0)/2.2;

 // hizlanma_boy_baslangici = 200; //kiliten kurtulma noktası

 // hizlanma_boy_bitisi = ((adim_sayisi - 200)  / 3) + 200;

 // yavaslama_boy_baslangici = ((adim_sayisi)*5 / 8); //hizlanma_boy_bitisi+ ((maksimum_kapi_boyu-adim_sayisi) / 6);
 // yavaslama_boy_bitisi = adim_sayisi - 150;
 /**
  *              hizlanma              yavaslama
  *              boy bitisi            boy bas.
  *               |                        |
  *               /------------------------\
  *              /                          \
  *             /                            \
  *            /                              \
  *kapali-----/                                \-----------acik
  *  0        |                                 |
  *          hizlanma                          yavaslama
  *          boy baş.                          boy bitis
  *
  *
  *  
  * */

 Serial.print(" kapama_max_rpm : ");
 Serial.println(kapama_max_rpm);

 printf("baski_adimi : %.1f \n", baski_adimi);

 if (baski_adimi < (adim_sayisi - rampa_yok_cm)) // rampa_yok_cm den kısa alanda baskı yediyse harita oluşturmayacak
 {
  hizlanma_boy_baslangici = baski_adimi + 100; // baskıya ne kadar kala yavaşlayacak
  hizlanma_boy_bitisi = baski_adimi + (((adim_sayisi - baski_adimi)) / 3);

  yavaslama_boy_baslangici = baski_adimi + ((adim_sayisi - baski_adimi) * 5 / 8); // hizlanma_boy_bitisi+ ((maksimum_kapi_boyu-adim_sayisi) / 6);
  yavaslama_boy_bitisi = adim_sayisi - 150;

  // kapama_max_rpm = map(adim_sayisi, baski_adimi, maksimum_kapi_boyu, 300, kapama_max_rpm);
  kapama_max_rpm *= (((maksimum_kapi_boyu - baski_adimi) * 1.25) / maksimum_kapi_boyu);
  if (kapama_max_rpm < 300)
  {
   kapama_max_rpm = 300;
  }
  for (int i = (adim_sayisi + 200); i >= (baski_adimi); i--)
  {
   if (i > yavaslama_boy_bitisi)
   {
    hedefRPMharitasi_kapa[i] = kapama_baslangic_RPM;

    //  Serial.print(i); Serial.print("AA"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i <= yavaslama_boy_bitisi and i > yavaslama_boy_baslangici)
   {
    hedefRPMharitasi_kapa[i] = map(i, yavaslama_boy_bitisi, yavaslama_boy_baslangici, kapama_baslangic_RPM, kapama_max_rpm);

    // Serial.print(i); Serial.print("A"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i <= yavaslama_boy_baslangici and i >= hizlanma_boy_bitisi)
   {
    hedefRPMharitasi_kapa[i] = kapama_max_rpm;

    //  Serial.print(i); Serial.print("B"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i < hizlanma_boy_bitisi and i >= hizlanma_boy_baslangici)
   {
    hedefRPMharitasi_kapa[i] = map(i, hizlanma_boy_bitisi, hizlanma_boy_baslangici, kapama_max_rpm, kapama_sonu_RPM);

    //  Serial.print(i); Serial.print("C"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i < hizlanma_boy_baslangici and i > 0)
   {
    hedefRPMharitasi_kapa[i] = kapama_sonu_RPM;

    //  Serial.print(i); Serial.print("D"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i <= 0)
   {
    hedefRPMharitasi_kapa[i] = 0;

    //  Serial.print(i); Serial.print("E"); Serial.println(hedefRPMharitasi[i]);
   }
  }
 }
 else
 {
  for (volatile uint16_t i = baski_adimi; i < (adim_sayisi + 200); i++)
  {
   hedefRPMharitasi_kapa[i] = kapama_baslangic_RPM;
   // printf("i : %d , hedefRPMharitasi : %d \n", i, hedefRPMharitasi[i]);
  }
 }
 /**
  *
  *
  * baskı adımına kadar olan rpm ayarlandı sonrası ayarlanıyor
  *
  *
  *
  */
 hizlanma_boy_baslangici = 200; // kiliten kurtulma noktası
 if (baski_adimi > (rampa_yok_cm + hizlanma_boy_baslangici))
 {

  hizlanma_boy_bitisi = (((baski_adimi)-200) / 3) + 200;

  yavaslama_boy_baslangici = ((baski_adimi)-150);
  yavaslama_boy_bitisi = baski_adimi - 100; // baskıdan ne kadar osnra hızlanacak
  if (mod_flag == tek_kanat)
  {
   kapama_max_rpm = EEPROM.read(16);
   kapama_max_rpm = kapama_max_rpm * hiz_katsayisi;
  }
  // kapama_max_rpm = map(baski_adimi, 0, maksimum_kapi_boyu, 300, kapama_max_rpm);
  kapama_max_rpm *= ((baski_adimi * 1.25) / maksimum_kapi_boyu);
  if (kapama_max_rpm < 300)
  {
   kapama_max_rpm = 300;
  }
  for (int i = baski_adimi; i >= 0; i--)
  {
   if (i > yavaslama_boy_bitisi)
   {
    hedefRPMharitasi_kapa[i] = kapama_baslangic_RPM;

    //  Serial.print(i); Serial.print("AA"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i <= yavaslama_boy_bitisi and i > yavaslama_boy_baslangici)
   {
    hedefRPMharitasi_kapa[i] = map(i, yavaslama_boy_bitisi, yavaslama_boy_baslangici, kapama_baslangic_RPM, kapama_max_rpm);

    // Serial.print(i); Serial.print("A"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i <= yavaslama_boy_baslangici and i >= hizlanma_boy_bitisi)
   {
    hedefRPMharitasi_kapa[i] = kapama_max_rpm;

    //  Serial.print(i); Serial.print("B"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i < hizlanma_boy_bitisi and i >= hizlanma_boy_baslangici)
   {
    hedefRPMharitasi_kapa[i] = map(i, hizlanma_boy_bitisi, hizlanma_boy_baslangici, kapama_max_rpm, kapama_sonu_RPM);

    //  Serial.print(i); Serial.print("C"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i < hizlanma_boy_baslangici and i > 0)
   {
    hedefRPMharitasi_kapa[i] = kapama_sonu_RPM;

    //  Serial.print(i); Serial.print("D"); Serial.println(hedefRPMharitasi[i]);
   }
   if (i <= 0)
   {
    hedefRPMharitasi_kapa[i] = 0;

    //  Serial.print(i); Serial.print("E"); Serial.println(hedefRPMharitasi[i]);
   }
  }
 }
 else
 {
  for (volatile uint16_t i = 0; i < baski_adimi; i++)
  {
   hedefRPMharitasi_kapa[i] = kapama_baslangic_RPM;
   // printf("i : %d , hedefRPMharitasi : %d \n", i, hedefRPMharitasi[i]);
  }
 }

 // for (uint16_t i = 0; i <= 2000; i++)
 // {
 //  printf("hedefRPMharitasi_kapa : %d : %d  \n ", i, hedefRPMharitasi_kapa[i]);
 //  vTaskDelay(1 / portTICK_RATE_MS);
 // }

 Serial.println("rpm_haritasi_olustur_engelli...");
}
void kapi_kapat_hazirlik()
{
  bobin_fark_sure=0;
 sure=0;
  maksimum_kapi_boyu = (double)(kapi_acma_derecesi * (10000.0 / 821.0))/2.5;

    const float base_rpm = 75.0;       // kalkış momenti için taban RPM
    const float max_rpm = kapama_max_rpm;       // kapama maksimum hızı
    const float ramp_up_ratio = 0.45;  // hızlı kalkış
    const float ramp_down_ratio = 0.45;// uzun yavaşlama
    const int total_steps = maksimum_kapi_boyu;

    for (int i = 0; i < total_steps; i++)
    {
        float pos = (float)i / total_steps;
        float rpm;

        if (pos < ramp_up_ratio)
        {
            // Hızlı parabolik kalkış
            float t = pos / ramp_up_ratio;
            rpm = base_rpm + (max_rpm - base_rpm) * (t * t);
        }
        else if (pos < (1.0 - ramp_down_ratio))
        {
            rpm = max_rpm;
        }
        else
        {
            // Baskı bölgesi: çok yumuşak azalma
            float t = (1.0 - pos) / ramp_down_ratio;
            rpm = base_rpm + (max_rpm - base_rpm) * (0.5 - 0.5 * cos(M_PI * t));
        }

        hedefRPMharitasi_kapa[i] = (uint16_t)rpm;
    }
 //       for (uint16_t i = 0; i <= 2000; i++)
 //  {
 //   printf("hedefRPMharitasi_kapa : %d : %d  \n ", i, hedefRPMharitasi_kapa[i]);
 // vTaskDelay(1 / portTICK_RATE_MS);
 // }
}



void kilit_ac()
{
 Serial.println("kilit_ac fn");
 if (adim_sayisi < (kilit_birakma_noktasi * 1000.0 / 82.1)) // sadece kapalıyken kilidi açbilmesi için bu flaga bakılıyor.
 {
  if (select_tr_mission == kilit_cikisi)
  {
   digitalWrite(kilit_pini, HIGH);
  }

  digitalWrite(kilit_pini_role, HIGH);
  kilit_timeout = millis();
  Serial.println("Aç Fn: Kilit ON");
  vTaskDelay(10 / portTICK_RATE_MS);
 }
}

void kilit_kapat()
{
 if (select_tr_mission == kilit_cikisi)
 {
  digitalWrite(kilit_pini, LOW);
 }
 digitalWrite(kilit_pini_role, LOW);
 kilit_state = 0;
 Serial.println("Aç Fn: Kilit OFF");
}

static void ac_task(void *arg)
{
 while (1)
 {
  vTaskDelay(100 / portTICK_RATE_MS);

  if (acil_stop_int_flag == true && digitalRead(fault) == 1) // stopa basildi
  {
   vTaskDelay(200 / portTICK_RATE_MS); // faultun geç tepki vermesinden dolayı 30 dan 100 e çıkardık
   if (acil_stop_int_flag == true && (digitalRead(stop_pini) == 0) && digitalRead(fault) == 1)
   {
    Serial.println("stopa basildi...");
    hareket_sinyali = kapi_bosta_sinyali;
    acil_stop_flag = true;

    client_data[client_dur_index] = 1;
    acil_stop_sayici++;
    eeproma_yaz_istegi = 1;
    motor_surme(0);
   }
   acil_stop_int_flag = false;
  }

  if (acil_stop_int_flag_kapat == true) // stoptan çekilme
  {
   vTaskDelay(200 / portTICK_RATE_MS);
   if (acil_stop_int_flag_kapat == true && (digitalRead(stop_pini) == 1))
   {
    Serial.println("stoptan cekildi...");
    ble_data_send = true;
    ble_send_data_repeat = 5;
    acil_stop_flag = false;
   }
   acil_stop_int_flag_kapat = false;
  }

  if (ac_flag == true && digitalRead(stop_pini) == 1) // ac sinyalindeki paaziti engellemek için eklendi.
  {

   vTaskDelay(100 / portTICK_RATE_MS);

   if ((hareket_sinyali != kapi_ac_sinyali && (digitalRead(ac_pini) == 1 || digitalRead(asansor_ac_pini) == 1)) || (hareket_sinyali != kapi_ac_sinyali && bluetooth_kapi_ac == true)) // if ((ac_flag == true && digitalRead(stop_pini) == false && adim_sayisi > hizlanma_boy_baslangici) || (ac_flag == true && digitalRead(stop_pini) == true)) //ac sinyalindeki paaziti engellemek için eklendi.// if ((hareket_sinyali != kapi_ac_sinyali && (digitalRead(ac_pini) == 1)) || (hareket_sinyali != kapi_ac_sinyali && bluetooth_kapi_ac == true))
   {
    kilit_state = 1;
    if (calisma_yontemi == otomatik_kapan || calisma_yontemi == ac_kapat)
    {

     if (kapattan_aca == false) // kapatırken ac verildiyse motor ilk tahrik butonda devreye girmyecek
     {

      kilit_state = 1;
      vTaskDelay(100 / portTICK_RATE_MS);
      if (mentese_yonu == 0) // mentese değişenine göre yönun terslendği yer
      {
       motor_yonu = kapi_ac;
      }
      else
      {
       motor_yonu = (!kapi_ac);
      }
      vTaskDelay(10 / portTICK_RATE_MS);
     }
     else
     {
     }
     memset(bobin_ortalama, 0, 6); // başlangıçta ortalama değerleri tutan dizi sıfırlandı.
     // adim_hata_counter = 0;
     counter_adim = 0;
     ac_time_out = millis();
     baski_flag = false;
     bluetooth_kapi_ac = false;
     Serial.println("ac hareketi yontem 1-2");
     PrintLog("ac+hareketi+yontem+1+2");
     kapi_ac_hazirlik();
     duty = motor_baslangic_duty; // Max_RPM / RPM_katsayisi;
     Max_RPM = hedefRPMharitasi[0];
     hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi); // 8333; // (4800000.0 * 1.2) / Max_RPM;
     aydinlatma_led_state = 1;
     hareket_sinyali = kapi_ac_sinyali;
     client_data[client_ac_index] = 1;
     Serial.print("hareket_sinyali : ");
     Serial.println(hareket_sinyali);
    }
   }

   if ((calisma_yontemi == ac_ac) && ((digitalRead(ac_pini) == 1) || (bluetooth_kapi_ac == true && (calisma_yontemi == ac_ac)) || digitalRead(asansor_ac_pini) == 1))
   {
    kilit_state = 1;
    if (ac_ac_flag == false) // kapiyi ac ile ac
    {
     ac_ac_flag_kapama = true;

     if (kapattan_aca == false) // kapatırken ac verildiyse motor ilk tahrik butonda devreye girmyecek
     {
      kilit_state = 1;
      vTaskDelay(100 / portTICK_RATE_MS);
      if (mentese_yonu == 0) // mentese değişenine göre yönun terslendği yer
      {
       motor_yonu = kapi_ac;
      }
      else
      {
       motor_yonu = (!kapi_ac);
      }
     }
     memset(bobin_ortalama, 0, 6); // başlangıçta ortalama değerleri tutan dizi sıfırlandı.
     counter_adim = 0;
     baski_flag = false;
     bluetooth_kapi_ac = false;
     Serial.println("ac hareketi yontem 3");
     PrintLog("ac+hareketi+yontem+3");
     kapi_ac_hazirlik();
     duty = motor_baslangic_duty; // Max_RPM / RPM_katsayisi;
     Max_RPM = hedefRPMharitasi[0];
     hedef_sure = 60.0 * 1000000.0 / (Max_RPM * tur_sayisi); // 8333; // (4800000.0 * 1.2) / Max_RPM;
     aydinlatma_led_state = 1;
     hareket_sinyali = kapi_ac_sinyali;
     client_data[client_ac_index] = 1;
    }
    else // kapiyi ac ile kapa
    {
     kapat_flag = true;
     bluetooth_kapi_kapa = true;
     client_data[client_kapa_index] = 1;
     Serial.println("ac butonu ile kapama...");
     PrintLog("ac+butonu+ile+kapama");
    }
    ac_ac_flag = (!ac_ac_flag);
    Serial.print("ac_ac_flag : ");
    Serial.println(ac_ac_flag);
   }
   ac_ac_flag_kapama = false;
   ac_flag = false;
  }

  if (calisma_yontemi != ac_ac)
  {
   if (digitalRead(ac_pini) == 1 || digitalRead(asansor_ac_pini) == 1)
   {
    ac_hata_tespit++;
   }
   if (digitalRead(ac_pini) == 0)
   {
    ac_hata_tespit = 0;
   }
   if (ac_hata_tespit == 500) // 50 saniyeden fazla ac sinail kesilmiyorsa hata verecek
   {
    ac_hata_sayaci++;

    eeproma_yaz_istegi = 1;
   }
   if (digitalRead(kapat_pini) == 1)
   {
    kapat_hata_tespit++;
   }
   if (digitalRead(kapat_pini) == 0)
   {
    kapat_hata_tespit = 0;
   }
   if (kapat_hata_tespit == 500) // 50 saniyeden fazla kapat sinail kesilmiyorsa hata verecek
   {
    kapat_hata_sayaci++;

    eeproma_yaz_istegi = 1;
   }
  }

  if (kapat_flag == true && digitalRead(stop_pini) == 1)
  {

   vTaskDelay(100 / portTICK_RATE_MS);
   if ((hareket_sinyali != kapi_kapat_sinyali && (digitalRead(kapat_pini) == 1 || digitalRead(asansor_ac_pini) == 0) && adim_sayisi > 50) ||
       (hareket_sinyali != kapi_kapat_sinyali && bluetooth_kapi_kapa == true && adim_sayisi > 50))
   {
    actan_kapata = true;

    Serial.println("kapanma buton...");
    PrintLog("kapanma+buton");

    kapat_time_out_flag = true;
    tanima_hizi_flag = false;
    kapanma_error_flag = false; // kapanma hatası sıfırlanıyor
    duty = tanima_hizi;
    vTaskDelay(200 / portTICK_RATE_MS);
    hareket_sinyali = kapi_kapat_sinyali;
    client_data[client_kapa_index] = 1;
   }
   kapat_flag = false;
  }
 }
}
void ble_baslat_fn()

{
 Serial.println("BLE Baslat Fn: Basladi");
 for (int i = 0; i < 250; i++)
 { //  BLE gönderme ve alma dizilerinin içerisindeki değerler sıfırlandı.
  ble_yollanan_dizi_global[i] = 0;
  ble_gelen_dizi_global[i] = 0;
 }
 char ble_name[30];
 sprintf(ble_name, "%s:%s", MACID, proje_no);
 printf("ble_name-------------------------:%s", ble_name);
 if (mod == cift_kanat)
 {

  BLEDevice::init(ble_name); //  Belirlenen MAC ID ile bluetooth hizmeti başlatıldı.
  // BLEDevice::setPower(ESP_PWR_LVL_P9); //  Bluetooth devresinin güç seviyesi ayarlandı.
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false);
  if (doConnect == true)
  {
   xTaskCreate(ble_client_task, "ble_client_task", 2048 * 4, NULL, 1, &ble_client_task_arg);
   Serial.println("BLE Baslat Fn: BLE Client Task Basladi");
  }
  else
  {
   // delay(1000);
   BLEDevice::getScan()->stop();
   pBLEScan->stop();
   // BLEDevice::deinit();
   Serial.println("BLE kapatildi");
   // delay(1000);
   mod = tek_kanat;
  }
 }

 if (mod == tek_kanat)
 {
  Serial.println("BLE Baslat Fn: BLE Server Task Basladi");
  BLEDevice::init(ble_name); //  Belirlenen MAC ID ile bluetooth hizmeti başlatıldı.

  BLEDevice::setPower(ESP_PWR_LVL_P9); //  Bluetooth devresinin güç seviyesi ayarlandı.

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic_2 = pService->createCharacteristic(CHARACTERISTIC_UUID_RX_TX,
                                                     BLECharacteristic::PROPERTY_NOTIFY |
                                                         BLECharacteristic::PROPERTY_READ |
                                                         BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic_2->setCallbacks(new ble_alma_class());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
 }
 // const uint8_t *point = esp_bt_dev_get_address();
 // for (int i = 0; i < 6; i++)
 // {

 //  char str[3];

 //  sprintf(str, "%02X", (int)point[i]);
 //  Serial.print(str);

 //  if (i < 5)
 //  {
 //   Serial.print(":");
 //  }
 // }
}
/**/
void ble_data_guncelle()
{
 /**********************************************************************************************************
  * BLE Sendig Communication Protocol:
  * ...
  * Sent(10):                              acma_hizlanma_rampasi
  * Sent(11):                              Max_RPM
  * Sent(12):                              acma_durma_rampasi////push-run açık kapalı
  * Sent(13):                              acma_baski_gucu
  * Sent(14):                              acma_carpma_siniri
  * Sent(15):                              kapama_hizlanma_rampasi//kapama_baski_suresi
  * Sent(16):                              kapama_hizi
  * Sent(17):                              kapama_durma_rampasi
  * Sent(18):                              kapama_baski_gucu
  * Sent(19):                              kapama_carpma_siniri
  * Sent(20):                              kapi_acma_acisi
  * Sent(21):                              asiri_akim_siniri
  * Sent(22):                              otomatik_kapanma_zamani
  * Sent(23):                              calisma_yontemi
  * Sent(24):                              birinci_akim_tam
  * Sent(25):                              birinci_akim_ondalik
  * Sent(26):                              ikinci_akim_tam
  * Sent(27):                              ikinci_akim_ondalik
  * Sent(28):                              ble_offset
  * Sent(29):                              adc1_alt
  * Sent(30):                              adc1_ust
  * Sent(31):                              adc2_alt
  * Sent(32):                              adc2_ust
  * Sent(33):                              kapi_basarisiz_ac_sayac_alt
  * Sent(34):                              kapi_basarisiz_ac_sayac_ust
  * Sent(35):                              kapi_ac_sayac_alt
  * Sent(36):                              kapi_ac_sayac_ust
  * Sent(37):                              kapi_basarisiz_kapat_sayac_alt
  * Sent(38):                              kapi_basarisiz_kapat_sayac_ust
  * Sent(39):                              kapi_kapat_sayac_alt
  * Sent(40):                              kapi_kapat_sayac_ust
  * Sent(41):                              restart_sayac_alt
  * Sent(42):                              restart_sayac_ust
  * Sent(43):                              sayac_poweron_reset_alt
  * Sent(44):                              sayac_poweron_reset_ust
  * Sent(45):                              sayac_sw_reset_alt
  * Sent(46):                              sayac_sw_reset_ust
  * Sent(47):                              sayac_owdt_reset_alt
  * Sent(48):                              sayac_owdt_reset_ust
  * Sent(49):                              sayac_deepsleep_reset_alt
  * Sent(50):                              sayac_deepsleep_reset_ust
  * Sent(51):                              sayac_sdio_reset_alt
  * Sent(52):                              sayac_sdio_reset_ust
  * Sent(53):                              sayac_timer0_wd_reset_alt
  * Sent(54):                              sayac_timer0_wd_reset_ust
  * Sent(55):                              sayac_timer1_wd_reset_alt
  * Sent(56):                              sayac_timer1_wd_reset_ust
  * Sent(57):                              sayac_rtc_wd_reset_alt
  * Sent(58):                              sayac_rtc_wd_reset_ust
  * Sent(59):                              sayac_intrusion_reset_alt
  * Sent(60):                              sayac_intrusion_reset_ust
  * Sent(61):                              sayac_time_group_reset_alt
  * Sent(62):                              sayac_time_group_reset_ust
  * Sent(63):                              sayac_sw_cpu_reset_alt
  * Sent(64):                              sayac_sw_cpu_reset_ust
  * Sent(65):                              sayac_rtc_wdc_reset_alt
  * Sent(66):                              sayac_rtc_wdc_reset_ust
  * Sent(67):                              sayac_ext_cpu_reset_alt
  * Sent(68):                              sayac_ext_cpu_reset_ust
  * Sent(69):                              sayac_brownout_reset_alt
  * Sent(70):                              sayac_brownout_reset_ust
  * Sent(71):                              sayac_rtc_wcdt_reset_alt
  * Sent(72):                              sayac_rtc_wcdt_reset_ust
  * Sent(73):                              test_modu_aktif
  * Sent(74):                              kapi_montaj_yonu
  * Sent(75):                              version
  * Sent(76):                              version
  * Sent(78):                              version
  * Sent(79):                              version
  * Sent(80):                              demo_modu_flag
  * Sent(81):                              kilit_birakma_noktasi
  * sent(82):                              acil_stop_sayici_low_byte
  * sent(83):                              acil_stop_sayici_high_byte
  * sent(84):                              dusuk_voltaj_sayici_low_byte
  * sent(85):                              dusuk_voltaj_sayici_high_byte
  * sent(86):                              fault_sayici_low_byte
  * sent(87):                              fault_sayici_high_byte
  * sent(90):                              ac_hata_sayaci_low_byte
  * sent(91):                              ac_hata_sayaci_high_byte
  * sent(92):                              kapat_hata_sayaci_low_byte
  * sent(93):                              kapat_hata_sayaci_high_byte
  *
  * ...
  * Sent(95):                              ble_amper
  * Sent(96):                              ble_carpan
  * Sent(97):                              rpm gonderiliyor
  * Sent(98):                              motor_hareketi
  * Sent(99):                              kilit_state
  * Sent(100):                             kapi_ilk_kapanma_yapildi
  * Sent(101):                             maksimum_akim_degeri
  * Sent(102):                             maksimum_kapi_boyu
  * Sent(103):                             kapi_sabit_tutma_esigi
  * Sent(104):                             kapi_sabit_cikma_esigi
  * ...
  * Sent(115):                             acilma_fn_hata
  * Sent(116):                             kapatma_fn_hata
  * Sent(117):                             kapatma_fn_loop_hata
  * Sent(118):                             fault_hata
  * Sent(119):                             loop_akim_hata
  * Sent(120):                             jumper_hatasi
  * Sent(121):                             motor_hatasi
  * Sent(122):                             motor_rpm
  * Sent(123):                             olculen_kapi_boyu
  * Sent(124):                             hareket_yonu
  * Sent(125):                             ac_sinyali
  * Sent(126):                             kapat_sinyali
  * Sent(127):                             h1_hata
  * Sent(128):                             h2_hata
  * Sent(129):                             h3_hata
  * Sent(130):                             l1_hata
  * Sent(131):                             l2_hata
  * Sent(132):                             l3_hata
  * Sent(133):                             hata
  * Sent(134):                             9
  * Sent(135):                             9
  * Sent(136):                             9
  * Sent(137):                             9
  * Sent(138):                             9
  * Sent(139):                             9
  **********************************************************************************************************/
 /* EEPROM Değişkenleri Güncelleniyor */
 // ble_sayaci++; //  Bu sayaç BLE dataları için kullanılmaktadır.
 // if (ble_sayaci > 401)
 // {
 //  ble_sayaci = 400;
 // } //  Sayacın maksimum büyüklüğü 400 ms olacaktır.
 BaseType_t status;
 double amper_transfer = 0;
 if (deviceConnected == 1 && ble_data_send == false)
 {
  ble_yollanan_dizi_global[0] = 200;
  ble_yollanan_dizi_global[1] = 200;
  ble_yollanan_dizi_global[2] = 200;

  status = xQueueReceive(Amper_Queue, &amper_transfer, 0);
  if (status == pdPASS)
  {
   ble_amper = amper_transfer * 100;
  }

  ble_yollanan_dizi_global[3] = ble_amper;

  ble_yollanan_dizi_global[4] = (rpm / hiz_katsayisi);

  pCharacteristic_2->setValue(ble_yollanan_dizi_global, 5);
  printf("ble_amper : %d rpm : %d \n", ble_yollanan_dizi_global[3], ble_yollanan_dizi_global[4]);
 }

 if (deviceConnected == 1 && ble_data_send == true)
 { //  BLE cihazı bağlı ise BLE datası gönderilecektir.
   // if (ble_sayaci >= 400)
   // { //  300ms de bir olmak üzere BLE dataları güncellenmektedir.
   // ble_sayaci = 0;
  // ble_data_send = false;
  status = xQueueReceive(Amper_Queue, &amper_transfer, 0);
  if (status == pdPASS)
  {
   ble_amper = amper_transfer * 100;
  }
  ble_yollanan_dizi_global[0] = 0;
  ble_yollanan_dizi_global[1] = 0;
  ble_yollanan_dizi_global[2] = 0;
  kapi_ac_sayac_alt = kapi_ac_sayac & 255;
  kapi_ac_sayac_ust = ((kapi_ac_sayac >> 8) & 255);
  kapi_basarisiz_ac_sayac_ust = (kapi_basarisiz_ac_sayac >> 8) & 255;
  kapi_basarisiz_ac_sayac_alt = (kapi_basarisiz_ac_sayac & 255);
  kapi_kapat_sayac_alt = (kapi_kapat_sayac & 255);
  kapi_kapat_sayac_ust = (kapi_kapat_sayac >> 8) & 255;
  kapi_basarisiz_kapat_sayac_alt = kapi_basarisiz_kapat_sayac & 255;
  kapi_basarisiz_kapat_sayac_ust = (kapi_basarisiz_kapat_sayac >> 8) & 255;

  ble_yollanan_dizi_global[lift_type_adres] = lift_type;
  ble_yollanan_dizi_global[tr_ble_adres] = select_tr_mission;
  ble_yollanan_dizi_global[10] = acma_hizlanma_rampasi;
  ble_yollanan_dizi_global[11] = EEPROM.read(11); // Max_rpm
  ble_yollanan_dizi_global[12] = push_run_flag;
  ble_yollanan_dizi_global[13] = tanima_hizi;
  ble_yollanan_dizi_global[14] = acma_carpma_siniri;
  ble_yollanan_dizi_global[15] = kapama_baski_suresi / (kapama_baski_suresi_ks); // kapama_hizlanma_rampasi;
  ble_yollanan_dizi_global[16] = EEPROM.read(16);
  ble_yollanan_dizi_global[17] = kapama_max_duty / 20; // kapama_durma_rampasi;
  ble_yollanan_dizi_global[18] = kapama_baski_gucu / kapama_baski_gucu_ks;
  ble_yollanan_dizi_global[19] = kapama_carpma_siniri;
  ble_yollanan_dizi_global[20] = EEPROM.read(20) * 2; // kapi_acma_derecesi
  ble_yollanan_dizi_global[21] = EEPROM.read(21) * 2; // kuvvet siniri eepromdan okunuyor
  ble_yollanan_dizi_global[22] = acik_kalma_suresi / 100;
  ble_yollanan_dizi_global[23] = calisma_yontemi;
  ble_yollanan_dizi_global[24] = 0; // birinci_akim_tam;
  ble_yollanan_dizi_global[25] = 0; // birinci_akim_ondalik;
  ble_yollanan_dizi_global[26] = 0; // ikinci_akim_tam;
  ble_yollanan_dizi_global[27] = 0; // ikinci_akim_ondalik;
  ble_offset = offset * 100;
  ble_yollanan_dizi_global[28] = 0; // ble_offset;
  ble_yollanan_dizi_global[29] = 0; // adc1_alt;
  ble_yollanan_dizi_global[30] = 0; // adc1_ust;
  ble_yollanan_dizi_global[31] = 0; // adc2_alt;
  ble_yollanan_dizi_global[32] = 0; // adc2_ust;
  ble_yollanan_dizi_global[33] = kapi_basarisiz_ac_sayac_alt;
  ble_yollanan_dizi_global[34] = kapi_basarisiz_ac_sayac_ust;
  ble_yollanan_dizi_global[35] = kapi_ac_sayac_alt;
  ble_yollanan_dizi_global[36] = kapi_ac_sayac_ust;
  ble_yollanan_dizi_global[37] = kapi_basarisiz_kapat_sayac_alt;
  ble_yollanan_dizi_global[38] = kapi_basarisiz_kapat_sayac_ust;
  ble_yollanan_dizi_global[39] = kapi_kapat_sayac_alt;
  ble_yollanan_dizi_global[40] = kapi_kapat_sayac_ust;
  ble_yollanan_dizi_global[41] = restart_sayac_alt;
  ble_yollanan_dizi_global[42] = restart_sayac_ust;
  ble_yollanan_dizi_global[43] = sayac_poweron_reset_alt;
  ble_yollanan_dizi_global[44] = sayac_poweron_reset_ust;
  ble_yollanan_dizi_global[45] = sayac_sw_reset_alt;
  ble_yollanan_dizi_global[46] = sayac_sw_reset_ust;
  ble_yollanan_dizi_global[47] = sayac_owdt_reset_alt;
  ble_yollanan_dizi_global[48] = sayac_owdt_reset_ust;
  ble_yollanan_dizi_global[49] = sayac_deepsleep_reset_alt;
  ble_yollanan_dizi_global[50] = sayac_deepsleep_reset_ust;
  ble_yollanan_dizi_global[51] = sayac_sdio_reset_alt;
  ble_yollanan_dizi_global[52] = sayac_sdio_reset_ust;
  ble_yollanan_dizi_global[53] = sayac_timer0_wd_reset_alt;
  ble_yollanan_dizi_global[54] = sayac_timer0_wd_reset_ust;
  ble_yollanan_dizi_global[55] = sayac_timer1_wd_reset_alt;
  ble_yollanan_dizi_global[56] = sayac_timer1_wd_reset_ust;
  ble_yollanan_dizi_global[57] = sayac_rtc_wd_reset_alt;
  ble_yollanan_dizi_global[58] = sayac_rtc_wd_reset_ust;
  ble_yollanan_dizi_global[59] = sayac_intrusion_reset_alt;
  ble_yollanan_dizi_global[60] = sayac_intrusion_reset_ust;
  ble_yollanan_dizi_global[61] = sayac_time_group_reset_alt;
  ble_yollanan_dizi_global[62] = sayac_time_group_reset_ust;
  ble_yollanan_dizi_global[63] = sayac_sw_cpu_reset_alt;
  ble_yollanan_dizi_global[64] = sayac_sw_cpu_reset_ust;
  ble_yollanan_dizi_global[65] = sayac_rtc_wdc_reset_alt;
  ble_yollanan_dizi_global[66] = sayac_rtc_wdc_reset_ust;
  ble_yollanan_dizi_global[67] = sayac_ext_cpu_reset_alt;
  ble_yollanan_dizi_global[68] = sayac_ext_cpu_reset_ust;
  ble_yollanan_dizi_global[69] = sayac_brownout_reset_alt;
  ble_yollanan_dizi_global[70] = sayac_brownout_reset_ust;
  ble_yollanan_dizi_global[71] = sayac_rtc_wcdt_reset_alt;
  ble_yollanan_dizi_global[72] = sayac_rtc_wcdt_reset_ust;
  // ble_yollanan_dizi_global[73] = test_modu_aktif;
  ble_yollanan_dizi_global[74] = (mentese_yonu);
  ble_yollanan_dizi_global[75] = version.charAt(0);
  ble_yollanan_dizi_global[76] = version.charAt(2);
  ble_yollanan_dizi_global[77] = version.charAt(4);
  ble_yollanan_dizi_global[78] = version.charAt(6);

  ble_yollanan_dizi_global[80] = demo_modu_flag;
  ble_yollanan_dizi_global[81] = kilit_birakma_noktasi;
  ble_yollanan_dizi_global[82] = (acil_stop_sayici & 255);
  ble_yollanan_dizi_global[83] = ((acil_stop_sayici >> 8) & 255);
  ble_yollanan_dizi_global[84] = (dusuk_voltaj_sayici & 255);
  ble_yollanan_dizi_global[85] = ((dusuk_voltaj_sayici >> 8) & 255);
  ble_yollanan_dizi_global[86] = (fault_sayici & 255);
  ble_yollanan_dizi_global[87] = ((fault_sayici >> 8) & 255);
  ble_yollanan_dizi_global[88] = digitalRead(ac_pini);
  ble_yollanan_dizi_global[89] = digitalRead(kapat_pini);
  ble_yollanan_dizi_global[90] = (ac_hata_sayaci & 255);
  ble_yollanan_dizi_global[91] = ((ac_hata_sayaci >> 8) & 255);
  ble_yollanan_dizi_global[92] = (kapat_hata_sayaci & 255);
  ble_yollanan_dizi_global[93] = ((kapat_hata_sayaci >> 8) & 255);

  // /* Telemetri Değerleri Güncelleniyor */
  ble_amper = amper_transfer * 100;
  ble_yollanan_dizi_global[95] = ble_amper;
  ble_carpan = carpan * 1000000;
  ble_yollanan_dizi_global[96] = ble_carpan;
  ble_yollanan_dizi_global[97] = (rpm / hiz_katsayisi);
  ble_yollanan_dizi_global[98] = socket_error & 255;
  ble_yollanan_dizi_global[99] = (socket_error >> 8) & 255;
  //  ble_yollanan_dizi_global[100] = kapi_ilk_kapanma_yapildi;
  //  ble_yollanan_dizi_global[101] = asiri_akim_siniri;
  //  ble_yollanan_dizi_global[102] = maksimum_kapi_boyu;

  // ble_yollanan_dizi_global[103] = a_hata_durumu;
  // ble_yollanan_dizi_global[104] = b_hata_durumu;
  ble_yollanan_dizi_global[115] = a_hata_durumu;
  ble_yollanan_dizi_global[116] = b_hata_durumu;
  ble_yollanan_dizi_global[117] = c_hata_durumu;
  ble_yollanan_dizi_global[118] = u_hata_durumu;
  ble_yollanan_dizi_global[119] = v_hata_durumu;
  ble_yollanan_dizi_global[120] = w_hata_durumu;
  // ble_yollanan_dizi_global[121] = dc_hata_durumu;
  ble_yollanan_dizi_global[121] = int(voltaj_test);
  ble_yollanan_dizi_global[122] = int(voltaj_test * 100) % 100;
  // ble_yollanan_dizi_global[124] = hareket_yonu;
  // ble_yollanan_dizi_global[125] = ac_sinyali;
  // ble_yollanan_dizi_global[126] = kapat_sinyali;
  // ble_yollanan_dizi_global[127] = h1_hata;
  // ble_yollanan_dizi_global[128] = h2_hata;
  // ble_yollanan_dizi_global[129] = h3_hata;
  // ble_yollanan_dizi_global[130] = l1_hata;
  // ble_yollanan_dizi_global[131] = l2_hata;
  // ble_yollanan_dizi_global[132] = l3_hata;
  ble_yollanan_dizi_global[133] = acil_stop_flag;
  ble_yollanan_dizi_global[134] = 9; // uygulamda ekran seçimi için 9 kalmalı
  ble_yollanan_dizi_global[135] = 9;
  ble_yollanan_dizi_global[136] = 9;
  ble_yollanan_dizi_global[137] = 9;
  ble_yollanan_dizi_global[138] = 9;
  ble_yollanan_dizi_global[139] = 9;

  // uint8_t d[10];
  // memset(d,100, 10);

  // pCharacteristic_2->setValue(d, 10);
  pCharacteristic_2->setValue(ble_yollanan_dizi_global, 140);
  // pCharacteristic->notify();
  // Serial.println("BLE Gonder Fn: Datalar Gonderildi");
  //   for(int i = 10; i < 140; i++) {
  // Serial.print("amper: ");
  // Serial.println(ble_yollanan_dizi_global[95]);
  //   }
  Serial.println("ble uzun data");
  // }
 }
}

void ble_data_al()
{
 /***********************************************************************************************
  * BLE Recieving Communication Protocol:
  * Recieved(1):            sifre_1                 121
  * Recieved(2):            sifre_2                 122
  * Recieved(3):            adres
  *                         ...
  *                         adres(10):              get_update_flag                    0-1
  *                         adres(11):              Max_RPM                            10-70 Slider
  *                         adres(12):              acma_durma_rampasi                 10-40 Slider//push-run olarak ayarlandı açık kapalıkonum gelecek
  *                         adres(13):              acma_baski_gucu                    5-20  Slider
  *                         adres(14):             //konsol_aktif_flag                 0-30  Slider
  *                         adres(15):              kapama_hizlanma_rampasi            10-40 Slider//kapama baskı süresi olarak değişecek slider 20-200
  *                         adres(16):              kapama_hizi                        10-70 Slider
  *                         adres(17):              kapama_durma_rampasi               10-40 Slider
  *                         adres(18):              kapama_baski_gucu                  5-20  Slider
  *                         adres(19):              kapama_carpma_siniri               0-30  Slider
  *                         adres(20):              kapi_acma_acisi                    0-180 Slider
  *                                                 maksimum_kapi_boyu
  *                         adres(21):              asiri_akim_siniri                  0-10  Slider
  *                                                 maksimum_akim_degeri
  *                         adres(22):              otomatik_kapanma_zamani            0-10  Slider
  *                         adres(23):              calisma_yontemi                    1-2-3 Var
  *                         adres(24):              birinci_akim_tam                   2 digit var
  *                         adres(25):              birinci_akim_ondalik               2 digit var
  *                         adres(26):              ikinci_akim_tam                    2 digit var
  *                         adres(27):              ikinci_akim_ondalik                2 digit var
  *                         ...
  *                         adres(73):              test_modu_aktif                    0-1 Dropdown
  *                         adres(74):              kapi_montaj_yonu                   0-1 Dropdown
  *                         adres(79):              mod
  *                         adres(80):              demo modu
  *                         adres(81):              Kilit bırakma noktası              100-500/50-250
  *                         adres(100):             sayac_sifirla_fn();                Button
  *                         adres(101):             esp_restart();                     Button
  *                         adres(102):             kapi_kapat_degiskeni               Buton
  *                         adres(103):             kapi_ac_degiskeni                  Buton
  *                         adres(104):             kalibrasyon_aktif                  OK
  *                         adres(105):             varsayilan_parametreler();         Buton
  *                                                 eeprom_oku_fn();
  *                         adres(106):             html_reset                         Buton
  * Recieved(4):            ble_alinan_deger
  *****************************************************************************************************/
 sifre_1 = ble_gelen_dizi_global[0];
 sifre_2 = ble_gelen_dizi_global[1];
 adres = ble_gelen_dizi_global[2];
 ble_alinan_deger = ble_gelen_dizi_global[3];

 // xSemaphoreTake(UartMutex,portMAX_DELAY);
 Serial.println("╔════════   BLE DEN BIR DEGER ALINDI   ══════╗");
 Serial.print("║     Node ID____________________= ");
 Serial.print(sifre_1);
 if (sifre_1 < 10)
 {
  Serial.println("         ║");
 }
 else if (sifre_1 < 100)
 {
  Serial.println("        ║");
 }
 else if (sifre_1 < 1000)
 {
  Serial.println("       ║");
 }
 Serial.print("║     Sifre______________________= ");
 Serial.print(sifre_2);
 if (sifre_2 < 10)
 {
  Serial.println("         ║");
 }
 else if (sifre_2 < 100)
 {
  Serial.println("        ║");
 }
 else if (sifre_2 < 1000)
 {
  Serial.println("       ║");
 }
 Serial.print("║     Adres_____________________ = ");
 Serial.print(adres);
 if (adres < 10)
 {
  Serial.println("         ║");
 }
 else if (adres < 100)
 {
  Serial.println("        ║");
 }
 else if (adres < 1000)
 {
  Serial.println("       ║");
 }
 Serial.print("║     Deger______________________= ");
 Serial.print(ble_alinan_deger);
 if (ble_alinan_deger < 10)
 {
  Serial.println("         ║");
 }
 else if (ble_alinan_deger < 100)
 {
  Serial.println("        ║");
 }
 else if (ble_alinan_deger < 1000)
 {
  Serial.println("       ║");
 }
 Serial.println("╚════════════════════════════════════════════╝ ");

 /********gelen data client denmi bakılıyor*/ ////////
 if (sifre_1 == 121 and sifre_2 == 123)
 {

  Serial.print("Client datasi...");
  for (int i = 0; i < 15; i++)
  {
   printf("i : %d data : %d\n", i, uint8_t(ble_gelen_dizi_global[i]));
  }
  mod_flag = cift_kanat;
  calisma_yontemi = ac_kapat;
  if (ble_gelen_dizi_global[client_ac_index] == 1 && ble_gelen_dizi_global[client_dur_index] == 0)
  {

   Max_RPM = double(ble_gelen_dizi_global[client_max_rpm_index]) * hiz_katsayisi;
   kapama_max_rpm = double(ble_gelen_dizi_global[client_kapama_max_rpm_index]) * hiz_katsayisi;
   mentese_yonu = !bool(ble_gelen_dizi_global[client_mentese_index]);
   acil_stop_client = uint8_t(ble_gelen_dizi_global[client_acil_stop_index]);
   kapama_baski_suresi = double(ble_gelen_dizi_global[client_baski_suresi_index]) * kapama_baski_suresi_ks * 2;
   acik_kalma_suresi = (double(ble_gelen_dizi_global[client_acik_kalma_suresi_index]) * 100);
   akim_siniri = double(ble_gelen_dizi_global[client_kuvvet_siniri_index]);
   akim_siniri = 2 * akim_siniri / 10;
   akim_siniri = 0.2535 * akim_siniri - 1.1008;
   push_run_flag = bool(ble_gelen_dizi_global[client_push_run_index]);
   kapama_baski_gucu = double(ble_gelen_dizi_global[client_baski_gucu_index]) * kapama_baski_gucu_ks;
   Serial.print("ble_gelen_dizi_global[client_derece_index] : ");
   Serial.println(double(ble_gelen_dizi_global[client_derece_index]));
   kapi_acma_derecesi = double(ble_gelen_dizi_global[client_derece_index]) * 2;
   maksimum_kapi_boyu = ((kapi_acma_derecesi) * (10000.0 / 821.0))/2.5;
   vTaskDelay(500 / portTICK_RATE_MS);
   ac_flag = true;
   bluetooth_kapi_ac = true;
  }
  if (ble_gelen_dizi_global[client_kapa_index] == 1)
  {

   Max_RPM = double(ble_gelen_dizi_global[client_max_rpm_index]) * hiz_katsayisi;
   kapama_max_rpm = double(ble_gelen_dizi_global[client_kapama_max_rpm_index]) * hiz_katsayisi;
   Serial.print(" kapama_max_rpm : ");
   Serial.println(kapama_max_rpm);
   mentese_yonu = !bool(ble_gelen_dizi_global[client_mentese_index]);
   acil_stop_client = uint8_t(ble_gelen_dizi_global[client_acil_stop_index]);
   kapama_baski_suresi = double(ble_gelen_dizi_global[client_baski_suresi_index]) * kapama_baski_suresi_ks * 2;
   acik_kalma_suresi = (double(ble_gelen_dizi_global[client_acik_kalma_suresi_index]) * 100);
   akim_siniri = double(ble_gelen_dizi_global[client_kuvvet_siniri_index]);
   akim_siniri = 2 * akim_siniri / 10;
   akim_siniri = 0.2535 * akim_siniri - 1.1008;
   push_run_flag = bool(ble_gelen_dizi_global[client_push_run_index]);
   kapama_baski_gucu = double(ble_gelen_dizi_global[client_baski_gucu_index]) * kapama_baski_gucu_ks;
   Serial.print("ble_gelen_dizi_global[client_derece_index] : ");
   Serial.println(double(ble_gelen_dizi_global[client_derece_index]));
   kapi_acma_derecesi = double(ble_gelen_dizi_global[client_derece_index]) * 2;
   maksimum_kapi_boyu = ((kapi_acma_derecesi) * (10000.0 / 821.0))/2.5;

   bluetooth_kapi_kapa = true;
   kapat_flag = true;
  }
  if (ble_gelen_dizi_global[client_dur_index] == 1)
  {
   hareket_sinyali = kapi_bosta_sinyali;
   // if (adim_sayisi < hizlanma_boy_baslangici)
   // {
   //  baski_flag = true;
   // }

   motor_surme(200);
  }
 }

 /* BLE Gelen Data */
 if (sifre_1 == 121 and sifre_2 == 122)
 {
  ble_data_send = true; // ble den data gelince doğrulama datası göndermek için bu bayrağı aktif ediyoruz
  mod_flag = tek_kanat;
  switch (adres)
  { //  BLE uygulamasından gelen istekler burada yer almaktadır.
  // case 10:
  //  acma_hizlanma_rampasi = ble_alinan_deger;
  //  Serial.print("BLE Al Fn: acma_hizlanma_rampasi = ");
  //  Serial.println(acma_hizlanma_rampasi);
  //  ;
  //  break;
  case lift_type_adres:
   lift_type = ble_alinan_deger;
   Serial.print("ble_alinan_deger = ");
   Serial.println(ble_alinan_deger);
   if (lift_type != VK)
   {
    Max_RPM = 22;
    EEPROM.write(11, Max_RPM);
    Max_RPM = Max_RPM * hiz_katsayisi;
    printf("acma_hizi___________________________: %d \n", Max_RPM);

    kapama_baski_suresi = 50;
    EEPROM.write(15, kapama_baski_suresi / 2);
    kapama_baski_suresi = EEPROM.read(15) * kapama_baski_suresi_ks * 2;
    printf("kapama_baski_suresi_vrsyln______: %d \n", kapama_baski_suresi);

    kapama_max_rpm = 22;
    EEPROM.write(16, kapama_max_rpm);
    kapama_max_rpm = kapama_max_rpm * hiz_katsayisi;
    printf("kapama_max_rpm_vrsyln__________________: %d \n", kapama_max_rpm);

    kapama_baski_gucu = 15;
    EEPROM.write(18, kapama_baski_gucu);
    kapama_baski_gucu = 15 * kapama_baski_gucu_ks;
    printf("kapama_baski_gucu___________________: %d \n", kapama_baski_gucu);

    kapi_acma_derecesi = 90;
    maksimum_kapi_boyu = (kapi_acma_derecesi / (82.1 / 1000.0))/2.5;
    EEPROM.write(20, kapi_acma_derecesi / 2);
    printf("kapi_acma_derecesi_vrsyln______________: %d \n", kapi_acma_derecesi);

    EEPROM.write(21, 150 / 2);
    akim_siniri = EEPROM.read(21) * 2;
    akim_siniri = ((akim_siniri)) / 10;
    akim_siniri = 0.2535 * akim_siniri - 1.1008;
    printf("kuvvet_siniri_vrsyln____________: %d \n", 150);
    printf("asiri_akim_siniri_vrsyln____________: %.1f \n", akim_siniri);
   }
   break;

  case tr_ble_adres:
   Serial.print("ble_alinan_deger = ");
   Serial.println(ble_alinan_deger);
   select_tr_mission = ble_alinan_deger;
   break;

  case 10: // gonderilen veri basarı ile alındı mesajı
   Serial.print("ble_alinan_deger = ");
   Serial.println(ble_alinan_deger);
   if (ble_alinan_deger == 1)
   {
    ble_data_send = false;
   }
   Serial.print("ble_data_send = ");
   Serial.println(ble_data_send);
   break;

  case 11:
   Max_RPM = hiz_katsayisi * ble_alinan_deger;
   Serial.print("BLE Al Fn: Max_RPM = ");
   Serial.println(Max_RPM);

   break;

  case 12:
   push_run_flag = ble_alinan_deger;
   Serial.print("BLE Al Fn: push_run_flag = ");
   Serial.println(push_run_flag);
   break;

   // case 13:
   //  tanima_hizi = ble_alinan_deger;
   //  tanima_hizi = tanima_hizi * 20;
   //  Serial.print("BLE Al Fn: acma_baski_gucu = ");
   //  Serial.println(tanima_hizi);
   //  break;

   // case 14:
   //  acma_carpma_siniri = ble_alinan_deger;
   //  Serial.print("BLE Al Fn: acma_carpma_siniri = ");
   //  Serial.println(acma_carpma_siniri);
   //  break;
  case 14:
   konsol_aktif_flag = ble_alinan_deger;
   Serial.print("BLE Al Fn: konsol_aktif_flag = ");
   Serial.print(konsol_aktif_flag);
   break;

  case 15:
   kapama_baski_suresi = ble_alinan_deger * kapama_baski_suresi_ks * 2;
   Serial.print("BLE Al Fn: kapama_baski_suresi = ");
   Serial.println(kapama_baski_suresi);
   break;

  case 16:
   kapama_max_rpm = hiz_katsayisi * ble_alinan_deger;
   Serial.print("BLE Al Fn: kapama_hizi = ");

   Serial.println(hiz_katsayisi * ble_alinan_deger);
   break;

  case 17:
   kapama_max_duty = ble_alinan_deger * 20;
   Serial.print("BLE Al Fn: kapama_max_duty = ");
   Serial.println(kapama_max_duty);
   break;

  case 18:
   if (ble_alinan_deger < 8) // alt sınır koymak gerekti
   {
    ble_alinan_deger = 8;
   }

   kapama_baski_gucu = ble_alinan_deger;
   kapama_baski_gucu = kapama_baski_gucu * kapama_baski_gucu_ks;
   Serial.print("BLE Al Fn: kapama_baski_gucu = ");
   Serial.println(kapama_baski_gucu);
   break;

   // case 19:
   //  kapama_carpma_siniri = ble_alinan_deger;
   //  Serial.print("BLE Al Fn: kapama_carpma_siniri = ");
   //  Serial.println(kapama_carpma_siniri);
   //  break;

  case 20:
   kapi_acma_derecesi = ble_alinan_deger;
   kapi_acma_derecesi = kapi_acma_derecesi * 2;
   printf("BLE Al Fn: kapi_acma_derecesi = %d \n", kapi_acma_derecesi);
  // maksimum_kapi_boyu = kapi_acma_derecesi / (82.1 / 1000.0);
   printf("BLE Al Fn: maksimum_kapi_boyu = %f \n", maksimum_kapi_boyu);
   break;

  case 21:
   akim_siniri = (ble_alinan_deger * 2) / 10;
   akim_siniri = 0.2535 * akim_siniri - 1.1008;
   printf("BLE Al Fn: asiri_akim_siniri = %.1f \n", akim_siniri);
   // local_hesap_akim = asiri_akim_siniri;
   //  akim_siniri_d_h = akim_siniri * 2;

   // Serial.print("BLE Al Fn: maksimum_akim_degeri = ");
   // Serial.println(akim_siniri, 2);
   // Serial.print("BLE Al Fn: akim_siniri_d_h = ");
   // Serial.println(akim_siniri_d_h, 2);
   break;

  case 22:
   acik_kalma_suresi = ble_alinan_deger * 100;
   printf("BLE Al Fn: otomatik_kapanma_zamani = %d \n", acik_kalma_suresi);
   break;

  case 23:
   calisma_yontemi = ble_alinan_deger;
   printf("BLE Al Fn: calisma_yontemi = %d \n", calisma_yontemi);
   break;

  case 24:
   birinci_akim_tam = ble_alinan_deger;
   Serial.print("BLE Al Fn: 1.akim Tam = ");
   Serial.print(birinci_akim_tam);
   break;

  case 25:
   birinci_akim_ondalik = ble_alinan_deger;
   Serial.print("BLE Al Fn: 1.akim ondalik = ");
   Serial.print(birinci_akim_ondalik);
   akim_alindi = 1;
   break;

  case 26:
   ikinci_akim_tam = ble_alinan_deger;
   Serial.print("BLE Al Fn: 2.akim Tam = ");
   Serial.print(ikinci_akim_tam);
   break;

  case 27:
   ikinci_akim_ondalik = ble_alinan_deger;
   Serial.print("BLE Al Fn: 2.akim ondalik = ");
   Serial.print(ikinci_akim_ondalik);
   akim_alindi = 1;
   break;

  case 61:
   if (ble_alinan_deger == 0)
   {
    Serial.println("version guncel degil");
    esp_restart();
   }
   else
   {
    Serial.println("version guncel");
   }

   akim_alindi = 1;
   break;
   // case 30:
   //  test_modu_aktif = ble_alinan_deger;
   //  Serial.print("test_modu_aktif = ");
   //  Serial.print(test_modu_aktif);
   //  adres -= 11; //eepromda 19 a kaydetmek için
   //  break;

  case 73:
   test_modu_aktif = ble_alinan_deger;
   printf("BLE Al Fn: test_modu_aktif = %d \n", test_modu_aktif);
   // EEProm.write(adres, ble_alinan_deger);
   // EEPROM.commit();
   // vTaskDelay(1000 / portTICK_RATE_MS);
   break;

  case 74:
   // ble_alinan_deger = (!ble_alinan_deger);
   mentese_yonu = (ble_alinan_deger);

   if (mentese_yonu == 1)
   {
    CW = -1;
    CCW = 1;
   }
   else
   {
    CW = 1;
    CCW = -1;
   }

   printf("BLE Al Fn: kapi_montaj_yonu = %d \n", mentese_yonu);
   vTaskDelay(500 / portTICK_RATE_MS);

   break;
  case 79:
   mod = ble_alinan_deger;
   printf("BLE Al Fn: mod = %d \n", mod);
   vTaskDelay(100 / portTICK_RATE_MS);

   break;
  case 80:
   demo_modu_flag = ble_alinan_deger;
   if (demo_modu_flag)
   {
    test_flag = true;
    xTaskCreate(test_task, "test_task", 2048, NULL, 10, &test_task_arg);
   }
   else
   {
    vTaskDelete(test_task_arg);
   }

   printf("BLE Al Fn: demo_modu_flag = %d \n", demo_modu_flag);
   vTaskDelay(100 / portTICK_RATE_MS);

   break;
  case 81:
   kilit_birakma_noktasi = ble_alinan_deger;
   printf("BLE Al Fn: kilit_birakma_noktasi = %d \n", kilit_birakma_noktasi);

   break;

  case 100:
   Serial.println("BLE Al Fn: Sayac Reset");
   sayac_sifirla_fn();
   break;

  case 101:
   Serial.println("BLE Al Fn: reset");
   delay(500);
   esp_restart();
   break;

  case 102:
   Serial.println("BLE Al Fn: BLE kapi kapat");
   bluetooth_kapi_kapa = true;
   kapat_flag = true;

   break;

  case 103:
   Serial.println("BLE Al Fn: BLE kapi ac");
   if (hareket_sinyali != kapi_ac_sinyali)
   {
    bluetooth_kapi_ac = true;
    ac_flag = true;
   }

   zaman_timeout = 0;
   break;

  case 104:
   Serial.println("BLE Al Fn: BLE Kalibrasyon baslat");
   kalibrasyon_aktif = 1;
   break;

  case 105:
   Serial.println("BLE Al Fn: Varsayılan Parametreler");
   for (int i = 0; i < 33; i++)
   {
    EEPROM.write(i, 255);
   }

   for (int i = 73; i < 82; i++)
   {
    EEPROM.write(i, 255);
   }

   for (int i = 200; i < 301; i++)
   {
    EEPROM.write(i, 255);
   }
   EEPROM.commit();
   vTaskDelay(500 / portTICK_RATE_MS);
   esp_restart();

   break;
  }
  if (adres == 29)
  {

   char buff[250];
   char parser[4] = "=>>";

   memset(proje_no, 0, sizeof(proje_no));
   memcpy(buff, ble_gelen_dizi_global, 250);
   for (uint8_t i = 0; i < 250; i++)
   {
    Serial.print(char(buff[i]));
    Serial.print(",");
   }
   Serial.println();

   for (uint8_t i = 0; i < strlen(buff); i++)
   {
    if (memcmp(&buff[i], parser, 3) == 0)
    {
     Serial.println("deger bulundu.");
     Serial.println(i);

     for (uint8_t j = 3; j < i; j++)
     {
      proje_no[j - 3] = buff[j];
     }
    }
   }

   for (uint8_t i = 0; i < 50; i++)
   {
    EEPROM.write(300 + i, uint8_t(proje_no[i]));
    vTaskDelay(1 / portTICK_RATE_MS);
   }
   Serial.print("proje_no = ");
   Serial.println(proje_no);
   eeproma_yaz_istegi = 1;
  }
  if (adres == 28)
  {
   char buff[250];
   char parser[4] = "=>>";

   memset(user_id, 0, sizeof(user_id));
   memset(wifi_password, 0, sizeof(wifi_password));
   memcpy(buff, ble_gelen_dizi_global, 250);
   for (uint8_t i = 0; i < 250; i++)
   {
    Serial.print(char(buff[i]));
    Serial.print(",");
   }
   Serial.println();

   for (uint8_t i = 0; i < strlen(buff); i++)
   {
    if (memcmp(&buff[i], parser, 3) == 0)
    {
     Serial.println("deger bulundu.");
     Serial.println(i);

     for (uint8_t j = 3; j < i; j++)
     {
      user_id[j - 3] = buff[j];
     }

     for (uint8_t k = i + 3; k < strlen(buff); k++)
     {
      wifi_password[k - (i + 3)] = buff[k];
     }
    }
   }
   for (uint8_t i = 0; i < 50; i++)
   {
    EEPROM.write(200 + i, uint8_t(user_id[i]));
    vTaskDelay(1 / portTICK_RATE_MS);
   }
   for (uint8_t i = 0; i < 50; i++)
   {
    EEPROM.write(250 + i, uint8_t(wifi_password[i]));
    vTaskDelay(1 / portTICK_RATE_MS);
   }

   eeproma_yaz_istegi = 1;
   ble_data_geldi = true;
   Serial.print("User ID= ");
   Serial.println(user_id);

   Serial.print("Password= ");
   Serial.println(wifi_password);
  }

  // ble_data_guncelle();
  if (adres < 100)
  {
   if (EEPROM.read(adres) != ble_alinan_deger)
   {
    EEPROM.write(adres, ble_alinan_deger);
    vTaskDelay(10 / portTICK_RATE_MS);
    Serial.print("Adres");
    Serial.println(adres);
    Serial.print("ble_alinan_deger");
    Serial.println(ble_alinan_deger);
    // eeproma_yaz_istegi = 1;
    ble_data_geldi = true;
   }

   if (adres == 74) // menteşe güncellendiyse
   {
    eeproma_yaz_istegi = 1;

    while (eeproma_yaz_istegi == 1)
    {
     vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    esp_restart();
   }
  }
 }
 // xSemaphoreGive(UartMutex);

 memset(ble_gelen_dizi_global, 0, 250);
}

void eeprom_oku_fn()
{
 /**********************************************************************************************************
  * EEPROM Allocation:
  * ...
  * EEPROM(10):                            acma_hizlanma_rampasi//get_update_flag
  * EEPROM(11):                            acma_hizi
  * EEPROM(12):                            acma_durma_rampasi//push-run açık kapalı
  * EEPROM(13):                            acma_baski_gucu
  * EEPROM(14):                            acma_carpma_siniri//konsol_aktif_flag
  * EEPROM(15):                            kapama_hizlanma_rampasi//kapama baski suresi
  * EEPROM(16):                            kapama_hizi//kapama max rmp olarak değiştirildi
  * EEPROM(17):                            kapama_durma_rampasi//kapama max duty kullanılmıyor
  * EEPROM(18):                            kapama_baski_gucu
  * EEPROM(19):                            test_modu_aktif
  * EEPROM(20):                            kapi_acma_acisi
  * EEPROM(21):                            asiri_akim_siniri
  * EEPROM(22):                            otomatik_kapanma_zamani
  * EEPROM(23):                            calisma_yontemi
  * EEPROM(24):                            birinci_akim_tam
  * EEPROM(25):                            birinci_akim_ondalik
  * EEPROM(26):                            ikinci_akim_tam
  * EEPROM(27):                            ikinci_akim_ondalik
  * EEPROM(28):                            offset
  * EEPROM(29):                            adc1_alt
  * EEPROM(30):                            adc1_ust
  * EEPROM(31):                            adc2_alt
  * EEPROM(32):                            adc2_ust
  * EEPROM(33):                            kapi_basarisiz_ac_sayac_alt
  * EEPROM(34):                            kapi_basarisiz_ac_sayac_ust
  * EEPROM(35):                            kapi_ac_sayac_alt
  * EEPROM(36):                            kapi_ac_sayac_ust
  * EEPROM(37):                            kapi_basarisiz_kapat_sayac_alt
  * EEPROM(38):                            kapi_basarisiz_kapat_sayac_ust
  * EEPROM(39):                            kapi_kapat_sayac_alt
  * EEPROM(40):                            kapi_kapat_sayac_ust
  * EEPROM(41):                            restart_sayac_alt
  * EEPROM(42):                            restart_sayac_ust
  * EEPROM(43):                            sayac_poweron_reset_alt
  * EEPROM(44):                            sayac_poweron_reset_ust
  * EEPROM(45):                            sayac_sw_reset_alt
  * EEPROM(46):                            sayac_sw_reset_ust
  * EEPROM(47):                            sayac_owdt_reset_alt
  * EEPROM(48):                            sayac_owdt_reset_ust
  * EEPROM(49):                            sayac_deepsleep_reset_alt
  * EEPROM(50):                            sayac_deepsleep_reset_ust
  * EEPROM(51):                            sayac_sdio_reset_alt
  * EEPROM(52):                            sayac_sdio_reset_ust
  * EEPROM(53):                            sayac_timer0_wd_reset_alt
  * EEPROM(54):                            sayac_timer0_wd_reset_ust
  * EEPROM(55):                            sayac_timer1_wd_reset_alt
  * EEPROM(56):                            sayac_timer1_wd_reset_ust
  * EEPROM(57):                            sayac_rtc_wd_reset_alt
  * EEPROM(58):                            sayac_rtc_wd_reset_ust
  * EEPROM(59):                            sayac_intrusion_reset_alt
  * EEPROM(60):                            sayac_intrusion_reset_ust
  * EEPROM(61):                            sayac_time_group_reset_alt
  * EEPROM(62):                            sayac_time_group_reset_ust
  * EEPROM(63):                            sayac_sw_cpu_reset_alt
  * EEPROM(64):                            sayac_sw_cpu_reset_ust
  * EEPROM(65):                            sayac_rtc_wdc_reset_alt
  * EEPROM(66):                            sayac_rtc_wdc_reset_ust
  * EEPROM(67):                            sayac_ext_cpu_reset_alt
  * EEPROM(68):                            sayac_ext_cpu_reset_ust
  * EEPROM(69):                            sayac_brownout_reset_alt
  * EEPROM(70):                            sayac_brownout_reset_ust
  * EEPROM(71):                            sayac_rtc_wcdt_reset_alt
  * EEPROM(72):                            sayac_rtc_wcdt_reset_ust
  * EEPROM(73):                            test_modu_aktif motor kabloları ve dc bara testi yapar
  * EEPROM(74):                            kapi_montaj_yonu//mentese_yonu
  * EEPROM(75):                            kapanma_akim_hata_sayac_alt
  * EEPROM(76):                            kapanma_akim_hata_sayac_ust
  * EEPROM(77):                            acilma_akim_hata_sayac_alt
  * EEPROM(78):                            acilma_akim_hata_sayac_ust
  * EEPROM(79):                            mod tek kanat cift kanat
  * EEPROM(80):                            demo modu
  * EEPROM(81):                            kilit_birakma_noktasi
  * EEPROM(82):                            acil_stop_sayici_low_byte
  * EEPROM(83):                            acil_stop_sayici_high_byte
  * EEPROM(84):                            dusuk_voltaj_sayici_low_byte
  * EEPROM(85):                            dusuk_voltaj_sayici_high_byte
  * EEPROM(86):                            fault_sayici_low_byte
  * EEPROM(87):                            fault_sayici_high_byte
  * EEPROM(200):                           wifi user id
  * EEPROM(250):                           wifi pasword
  **********************************************************************************************************/

 /**
  * printlog  çalışabimesi için ssid ve
  * pasword ilk olarak çekmek gerekir
  * */
 Serial.println("EEPROM Oku Fn: Girildi");
 for (uint8_t i = 0; i < 50; i++)
 {
  user_id[i] = EEPROM.read(200 + i);
  vTaskDelay(1 / portTICK_RATE_MS);
 }
 for (uint8_t i = 0; i < 50; i++)
 {
  wifi_password[i] = EEPROM.read(250 + i);
  vTaskDelay(1 / portTICK_RATE_MS);
 }
 for (uint8_t i = 0; i < 50; i++)
 {
  proje_no[i] = EEPROM.read(300 + i);
  vTaskDelay(1 / portTICK_RATE_MS);
 }
 if (proje_no[0] > 200)
 {
  sprintf(proje_no, "no_name");
 }

 ssid = convertToString(user_id, strlen(user_id));
 password = convertToString(wifi_password, strlen(wifi_password));
 deviceKey = convertToString(proje_no, strlen(proje_no));
 Serial.print("User ID= ");
 Serial.println(ssid);
 Serial.print("Password= ");
 Serial.println(password);
 Serial.print("proje_no= ");
 Serial.println(proje_no);

 /* EEPROM'dan Veri Okunması */

 lift_type = EEPROM.read(lift_type_adres);
 if (select_tr_mission == 255)
 {
  lift_type = VK;
 }
 printf("lift_type ___________: %d \n", lift_type);

 select_tr_mission = EEPROM.read(tr_ble_adres);
 if (select_tr_mission == 255)
 {
  select_tr_mission = standart_led;
 }
 printf("select_tr_mission ___________: %d \n", select_tr_mission);

 Max_RPM = EEPROM.read(11); //  EEPROM(11) Okunuyor.
 if (Max_RPM > 200)
 {              //  Açma Hızı
  Max_RPM = 33; //  Varsayılan Değer = 20
  EEPROM.write(11, Max_RPM);
  Max_RPM = Max_RPM * hiz_katsayisi;
  printf("acma_hizi_vrsyln____________________: %d \n", Max_RPM);
 }
 else
 {
  Max_RPM =Max_RPM * hiz_katsayisi;

  printf("acma_hizi___________________________: %d \n", Max_RPM);
 }

 int buff = EEPROM.read(12); //  EEPROM(12) Okunuyor.
 if (buff > 200)
 {                       //  Açma Durma Rampası
  push_run_flag = false; //  Varsayılan Değer = 40
  EEPROM.write(12, push_run_flag);
  push_run_flag = EEPROM.read(12);
  printf("push_run_flag_vrsyln___________: %d \n", push_run_flag);
 }
 else
 {
  push_run_flag = buff;
  printf("push_run_flag__________________: %d \n", push_run_flag);
 }

 tanima_hizi = EEPROM.read(13); //  EEPROM(13) Okunuyor.
 if (tanima_hizi > 200)
 {                   //  Açma Baskı Gücü
  tanima_hizi = 100; //  Varsayılan Değer = 5
  EEPROM.write(13, 5);
  printf("tanima_hizi_vrsyln______________: %d \n", tanima_hizi);
 }
 else
 {
  tanima_hizi = tanima_hizi * 20;
  printf("tanima_hizi_____________________: %d \n", tanima_hizi);
 }
 int konsol_aktif_buff = EEPROM.read(14); //  EEPROM(10) Okunuyor.
 if (konsol_aktif_buff > 200)
 {                           //  Açma Hızlanma Rampası
  konsol_aktif_flag = false; //  Varsayılan Değer = 40
  EEPROM.write(14, konsol_aktif_flag);
  konsol_aktif_flag = EEPROM.read(14);
  printf("konsol_aktif_flag_vrsyln________: %d \n", konsol_aktif_flag);
 }
 else
 {
  konsol_aktif_flag = konsol_aktif_buff;
  printf("konsol_aktif_flag_______________: %d \n", konsol_aktif_flag);
 }

 kapama_baski_suresi = EEPROM.read(15); //  EEPROM(15) Okunuyor.
                                        // printf("kapama_baski_suresi_____________: %d \n", kapama_baski_suresi);
 if (kapama_baski_suresi > 200)
 {                          //  Kapatma Hızlanma Rampası
  kapama_baski_suresi = 20; //  Varsayılan Değer 200 ms olarak ayarlandı
  EEPROM.write(15, kapama_baski_suresi / 2);
  kapama_baski_suresi = EEPROM.read(15) * kapama_baski_suresi_ks * 2;
  printf("kapama_baski_suresi_vrsyln______: %d \n", kapama_baski_suresi);
 }
 else
 {
  kapama_baski_suresi = kapama_baski_suresi * kapama_baski_suresi_ks * 2;
  printf("kapama_baski_suresi_____________: %d \n", kapama_baski_suresi);
 }
 kapama_max_rpm = EEPROM.read(16); //  EEPROM(16) Okunuyor.
 if (kapama_max_rpm > 200)
 {                     //  Kapatma Hızı
  kapama_max_rpm = 33; //  Varsayılan Değer = 20
  EEPROM.write(16, kapama_max_rpm);
  kapama_max_rpm = kapama_max_rpm * hiz_katsayisi;
  printf("kapama_max_rpm_vrsyln__________________: %d \n", kapama_max_rpm);
 }
 else
 {
  kapama_max_rpm = kapama_max_rpm * hiz_katsayisi;
  printf("kapama_max_rpm_________________________: %d \n", kapama_max_rpm);
 }

 // kapama_max_duty = EEPROM.read(17); //  EEPROM(17) Okunuyor.
 // if (kapama_max_duty > 200)
 // {                            //  Kapatma Durma Rampası
 //  kapama_max_duty = max_duty; //  Varsayılan Değer = 40
 //  EEPROM.write(17, kapama_max_duty / 20);
 //  printf("kapama_max_duty_vrsyln_________: %d \n", kapama_max_duty);
 // }
 // else
 // {
 //  kapama_max_duty = kapama_max_duty * 20;
 //  printf("kapama_max_duty________________: %d \n", kapama_max_duty);
 // }
 kapama_baski_gucu = EEPROM.read(18); //  EEPROM(18) Okunuyor.
 if (kapama_baski_gucu < 8)
 {
  kapama_baski_gucu = 8;
 }

 if (kapama_baski_gucu > 200)
 { //  Kapatma Baskı Gücü,,,,,,,,,,,,,,
  kapama_baski_gucu = 8;
  EEPROM.write(18, kapama_baski_gucu);
  kapama_baski_gucu = 8 * kapama_baski_gucu_ks;
  printf("kapama_baski_gucu_vrsyln____________: %d \n", kapama_baski_gucu);
 }
 else
 {
  kapama_baski_gucu = kapama_baski_gucu * kapama_baski_gucu_ks;
  printf("kapama_baski_gucu___________________: %d \n", kapama_baski_gucu);
 }
 // kapama_carpma_siniri = EEPROM.read(19); //  EEPROM(19) Okunuyor.
 // if (kapama_carpma_siniri > 200)
 // {                          //  Kapatma Çarpma Sınırı
 //  kapama_carpma_siniri = 5; //  Varsayılan Değer = 5
 //  EEPROM.write(19, kapama_carpma_siniri);
 //  kapama_carpma_siniri = EEPROM.read(19);
 //  printf("kapama_carpma_siniri_vrsyln_________: %d \n", kapama_carpma_siniri);
 // }
 // else
 // {

 //  printf("kapama_carpma_siniri________________: %d \n", kapama_carpma_siniri);
 // }

 kapi_acma_derecesi = EEPROM.read(20); //  EEPROM(20) Okunuyor.
 if (kapi_acma_derecesi > 200)
 {                          //  Kapı Açma Açısı
  kapi_acma_derecesi = 90; //  Varsayılan Değer = 90
  maksimum_kapi_boyu = (kapi_acma_derecesi / (82.1 / 1000.0))/2.5;
  EEPROM.write(20, kapi_acma_derecesi / 2);
  printf("kapi_acma_derecesi_vrsyln______________: %d \n", kapi_acma_derecesi);
 }
 else
 {
  kapi_acma_derecesi = kapi_acma_derecesi*2;

  maksimum_kapi_boyu = (double)(kapi_acma_derecesi * (10000.0 / 821.0))/2.5;

  printf("kapi_acma_derecesi_____________________: %d \n", kapi_acma_derecesi);
 }
 akim_siniri = EEPROM.read(21); //  EEPROM(21) Okunuyor.
 if (akim_siniri > 200)
 { //  Aşırı Akım Sınırı
  // akim_siniri = 1; //  Varsayılan Değer = 1
  EEPROM.write(21, 120 / 2); // 120 n varsayılan değer
  akim_siniri = EEPROM.read(21) * 2;
  akim_siniri = ((akim_siniri)) / 10;
  akim_siniri = 0.2535 * akim_siniri - 1.1008;
  // akim_siniri_d_h = akim_siniri * 2;
  printf("kuvvet_siniri_vrsyln____________: %d \n", EEPROM.read(21) * 2);
  printf("asiri_akim_siniri_vrsyln____________: %.1f \n", akim_siniri);
  if (digitalRead(x1_jumper) == 0)
  {
   akim_siniri = akim_siniri + 0.2 * akim_siniri;
   printf(" x1 takili asiri_akim_siniri_vrsyln____________: %.1f \n", akim_siniri);
  }
 }
 else
 {
  // akim_siniri /= 2.0;
  akim_siniri = 2 * akim_siniri / 10;
  akim_siniri = 0.2535 * akim_siniri - 1.1008;
  // akim_siniri_d_h = akim_siniri * 2;
  printf("kuvvet_siniri___________________: %d \n", EEPROM.read(21));
  printf("asiri_akim_siniri___________________: %.1f \n", akim_siniri);
  if (digitalRead(x1_jumper) == 0)
  {
   akim_siniri = akim_siniri + 0.2 * akim_siniri;
   printf(" x1 takili asiri_akim_siniri_vrsyln____________: %.1f \n", akim_siniri);
  }
 }
 acik_kalma_suresi = EEPROM.read(22); //  EEPROM(22) Okunuyor.
 if (acik_kalma_suresi > 200)
 { //  Otomatik Kapanma Zamanı
  // acik_kalma_suresi = 500; //  Varsayılan Değer = 5
  EEPROM.write(22, 5);
  acik_kalma_suresi = EEPROM.read(22) * 100;
  printf("acik_kalma_suresi_vrsyln______: %d \n", acik_kalma_suresi);
 }
 else
 {
  acik_kalma_suresi = acik_kalma_suresi * 100;
  printf("acik_kalma_suresi_____________: %d \n", acik_kalma_suresi);
 }
 calisma_yontemi = EEPROM.read(23); //  EEPROM(23) Okunuyor.
 if ((calisma_yontemi == 0) || (calisma_yontemi > 3))
 {                                   //  Çalışma Yöntemi
  calisma_yontemi = 2;               //  Varsayılan Değer = 2
  EEPROM.write(23, calisma_yontemi); //  Aç Sinyali ile kapıyı aç, aç sinyali kesilince kapıyı kapat
  calisma_yontemi = EEPROM.read(23);
  printf("calisma_yontemi_vrsyln______________: %d \n", calisma_yontemi);
 }
 else
 {
  // calisma_yontemi = 3;
  printf("calisma_yontemi_____________________: %d \n", calisma_yontemi);
 }
 birinci_akim_tam = EEPROM.read(24);                                     //  EEPROM(24) Okunuyor.
 birinci_akim_ondalik = EEPROM.read(25);                                 //  EEPROM(25) Okunuyor.
 printf("birinci_akim_tam_______________: %f \n", birinci_akim_tam);     //  Amper1
 printf("birinci_akim_ondalik___________: %f \n", birinci_akim_ondalik); //  Varsayılan Değer = 0.5
 if (birinci_akim_tam > 200 and birinci_akim_ondalik > 200)
 { //  Birinci kalibrasyon akımıdır. Multimetreden okunan değerdir.
  birinci_akim_tam = 0;
  birinci_akim_ondalik = 24;
  EEPROM.write(24, birinci_akim_tam);
  EEPROM.write(25, birinci_akim_ondalik);
  amper1 = 0.24;
  printf("amper1_vrsyln_______________________: %f \n", amper1);
 }
 else
 {
  amper1 = ((birinci_akim_tam * 100) + birinci_akim_ondalik) / 100;
  printf("amper1______________________________: %f \n", amper1);
 }
 ikinci_akim_tam = EEPROM.read(26);                                     //  EEPROM(26) Okunuyor.
 ikinci_akim_ondalik = EEPROM.read(27);                                 //  EEPROM(27) Okunuyor.
 printf("ikinci_akim_tam________________: %f \n", ikinci_akim_tam);     //  Amper2
 printf("ikinci_akim_ondalik____________: %f \n", ikinci_akim_ondalik); //  Varsayılan Değer = 2.5
 if (ikinci_akim_tam > 200 and ikinci_akim_ondalik > 200)
 { //  İkinci kalibrasyon akımıdır. Multimetreden okunan değerdir.
  amper2 = 2.0;
  ikinci_akim_tam = 2;
  ikinci_akim_ondalik = 0;
  EEPROM.write(26, ikinci_akim_tam);
  EEPROM.write(27, ikinci_akim_ondalik);
  printf("amper2_vrsyln_______________________: %f \n", amper2);
 }
 else
 {
  amper2 = ((ikinci_akim_tam * 100) + ikinci_akim_ondalik) / 100;
  printf("amper2______________________________: %f \n", amper2);
 }
 offset = EEPROM.read(28); //  EEPROM(28) Okunuyor.
 if (offset > 200)
 {             //  Offset
  offset = 70; //  Varsayılan Değer = 0
  EEPROM.write(28, offset);
  offset = EEPROM.read(28);
  printf("offset_vrsyln_______________________: %f \n", offset);
 }
 else
 {
  printf("offset______________________________: %f \n", offset);
 }
 adc1_alt = EEPROM.read(29);                                 //  EEPROM(29) Okunuyor.
 adc1_ust = EEPROM.read(30);                                 //  EEPROM(30) Okunuyor.
 printf("adc1_alt_______________________: %d \n", adc1_alt); //  Adc1
 printf("adc1_ust_______________________: %d \n", adc1_ust); //  Varsayılan Değer = 435
 if (adc1_alt > 200 and adc1_ust > 200)
 { //  Birinci kalibrasyon akımıdır. sense_pini'nden okunan değerdir.
  adc1 = 435;
  EEPROM.write(29, (adc1 & 255));
  EEPROM.write(30, ((adc1 >> 8) & 255));
  adc1_alt = EEPROM.read(29); //  EEPROM(29) Okunuyor.
  adc1_ust = EEPROM.read(30); //  EEPROM(30) Okunuyor.
  printf("adc1_vrsyln_________________________: %d \n", adc1);
 }
 else
 {
  adc1 = (adc1_ust * 256) + adc1_alt;
  printf("adc1________________________________: %d \n", adc1);
 }
 adc2_alt = EEPROM.read(31); //  EEPROM(31) Okunuyor.
 adc2_ust = EEPROM.read(32); //  EEPROM(32) Okunuyor.
 //  printf("adc2_alt_______________________: %d \n", adc2_alt);                     //  Adc2
 //  printf("adc2_ust_______________________: %d \n", adc2_ust);                     //  Varsayılan Değer = 3115
 if (adc2_alt > 200 and adc2_ust > 200)
 { //  İkinci kalibrasyon akımıdır. sense_pini'nden okunan değerdir.
  adc2 = 3115;
  EEPROM.write(31, (adc2 & 255));
  EEPROM.write(32, ((adc2 >> 8) & 255));
  adc2_alt = EEPROM.read(31); //  EEPROM(31) Okunuyor.
  adc2_ust = EEPROM.read(32); //  EEPROM(32) Okunuyor.
  printf("adc2_vrsyln_________________________: %d \n", adc2);
 }
 else
 {
  adc2 = (adc2_ust * 256) + adc2_alt;
  printf("adc2________________________________: %d \n", adc2);
 }
 carpan = ((amper2 - amper1) / double(adc2 - adc1)); //  Analog pinden yapılan ölçüm değerlerinin gerçek ölçümlere yaklaşması için katsayı değer hesaplandı.
 printf("carpan______________________________: %f \n", carpan);

 kapi_basarisiz_ac_sayac_alt = EEPROM.read(33);
 kapi_basarisiz_ac_sayac_ust = EEPROM.read(34);
 kapi_ac_sayac_alt = EEPROM.read(35);
 kapi_ac_sayac_ust = EEPROM.read(36);
 kapi_basarisiz_kapat_sayac_alt = EEPROM.read(37);
 kapi_basarisiz_kapat_sayac_ust = EEPROM.read(38);
 kapi_kapat_sayac_alt = EEPROM.read(39);
 kapi_kapat_sayac_ust = EEPROM.read(40);
 restart_sayac_alt = EEPROM.read(41);          //  printf("restart_sayac_alt______________: %d \n", restart_sayac_alt);
 restart_sayac_ust = EEPROM.read(42);          //  printf("restart_sayac_ust______________: %d \n", restart_sayac_ust);
 sayac_poweron_reset_alt = EEPROM.read(43);    //  printf("sayac_poweron_reset_alt________: %d \n", sayac_poweron_reset_alt);
 sayac_poweron_reset_ust = EEPROM.read(44);    //  printf("sayac_poweron_reset_ust________: %d \n", sayac_poweron_reset_ust);
 sayac_sw_reset_alt = EEPROM.read(45);         //  printf("sayac_sw_reset_alt_____________: %d \n", sayac_sw_reset_alt);
 sayac_sw_reset_ust = EEPROM.read(46);         //  printf("sayac_sw_reset_ust_____________: %d \n", sayac_sw_reset_ust);
 sayac_owdt_reset_alt = EEPROM.read(47);       //  printf("sayac_owdt_reset_alt___________: %d \n", sayac_owdt_reset_alt);
 sayac_owdt_reset_ust = EEPROM.read(48);       //  printf("sayac_owdt_reset_ust___________: %d \n", sayac_owdt_reset_ust);
 sayac_deepsleep_reset_alt = EEPROM.read(49);  //  printf("sayac_deepsleep_reset_alt______: %d \n", sayac_deepsleep_reset_alt);
 sayac_deepsleep_reset_ust = EEPROM.read(50);  //  printf("sayac_deepsleep_reset_ust______: %d \n", sayac_deepsleep_reset_ust);
 sayac_sdio_reset_alt = EEPROM.read(51);       //  printf("sayac_sdio_reset_alt___________: %d \n", sayac_sdio_reset_alt);
 sayac_sdio_reset_ust = EEPROM.read(52);       //  printf("sayac_sdio_reset_ust___________: %d \n", sayac_sdio_reset_ust);
 sayac_timer0_wd_reset_alt = EEPROM.read(53);  //  printf("sayac_timer0_wd_reset_alt______: %d \n", sayac_timer0_wd_reset_alt);
 sayac_timer0_wd_reset_ust = EEPROM.read(54);  //  printf("sayac_timer0_wd_reset_ust______: %d \n", sayac_timer0_wd_reset_ust);
 sayac_timer1_wd_reset_alt = EEPROM.read(55);  //  printf("sayac_timer1_wd_reset_alt______: %d \n", sayac_timer1_wd_reset_alt);
 sayac_timer1_wd_reset_ust = EEPROM.read(56);  //  printf("sayac_timer1_wd_reset_ust______: %d \n", sayac_timer1_wd_reset_ust);
 sayac_rtc_wd_reset_alt = EEPROM.read(57);     //  printf("sayac_rtc_wd_reset_alt_________: %d \n", sayac_rtc_wd_reset_alt);
 sayac_rtc_wd_reset_ust = EEPROM.read(58);     //  printf("sayac_rtc_wd_reset_ust_________: %d \n", sayac_rtc_wd_reset_ust);
 sayac_intrusion_reset_alt = EEPROM.read(59);  //  printf("sayac_intrusion_reset_alt______: %d \n", sayac_intrusion_reset_alt);
 sayac_intrusion_reset_ust = EEPROM.read(60);  //  printf("sayac_intrusion_reset_ust______: %d \n", sayac_intrusion_reset_ust);
 sayac_time_group_reset_alt = EEPROM.read(61); //  printf("sayac_time_group_reset_alt_____: %d \n", sayac_time_group_reset_alt);
 sayac_time_group_reset_ust = EEPROM.read(62); //  printf("sayac_time_group_reset_ust_____: %d \n", sayac_time_group_reset_ust);
 sayac_sw_cpu_reset_alt = EEPROM.read(63);     //  printf("sayac_sw_cpu_reset_alt_________: %d \n", sayac_sw_cpu_reset_alt);
 sayac_sw_cpu_reset_ust = EEPROM.read(64);     //  printf("sayac_sw_cpu_reset_ust_________: %d \n", sayac_sw_cpu_reset_ust);
 sayac_rtc_wdc_reset_alt = EEPROM.read(65);    //  printf("sayac_rtc_wdc_reset_alt________: %d \n", sayac_rtc_wdc_reset_alt);
 sayac_rtc_wdc_reset_ust = EEPROM.read(66);    //  printf("sayac_rtc_wdc_reset_ust________: %d \n", sayac_rtc_wdc_reset_ust);
 sayac_ext_cpu_reset_alt = EEPROM.read(67);    //  printf("sayac_ext_cpu_reset_alt________: %d \n", sayac_ext_cpu_reset_alt);
 sayac_ext_cpu_reset_ust = EEPROM.read(68);    //  printf("sayac_ext_cpu_reset_ust________: %d \n", sayac_ext_cpu_reset_ust);
 sayac_brownout_reset_alt = EEPROM.read(69);   //  printf("sayac_brownout_reset_alt_______: %d \n", sayac_brownout_reset_alt);
 sayac_brownout_reset_ust = EEPROM.read(70);   //  printf("sayac_brownout_reset_ust_______: %d \n", sayac_brownout_reset_ust);
 sayac_rtc_wcdt_reset_alt = EEPROM.read(71);   //  printf("sayac_rtc_wcdt_reset_alt_______: %d \n", sayac_rtc_wcdt_reset_alt);
 sayac_rtc_wcdt_reset_ust = EEPROM.read(72);   //  printf("sayac_rtc_wcdt_reset_ust_______: %d \n", sayac_rtc_wcdt_reset_ust);

 test_modu_aktif = EEPROM.read(73); //  EEPROM(73) Okunuyor.
 if (test_modu_aktif > 200)
 {                     //  Test Modu Aktif
  test_modu_aktif = 0; //  Varsayılan Değer = 0
  EEPROM.write(73, test_modu_aktif);
  test_modu_aktif = EEPROM.read(73);
  printf("test_modu_aktif_vrsyln______________: %d \n", test_modu_aktif);
 }
 else
 {
  printf("test_modu_aktif_____________________: %d \n", test_modu_aktif);
 }
 int mentese_buff = EEPROM.read(74);
 // mentese_yonu = EEPROM.read(74);
 if (mentese_buff > 200)
 {
  mentese_yonu = sag;
  EEPROM.write(74, mentese_yonu);
  mentese_yonu = EEPROM.read(74);
 }
 else
 {
  mentese_yonu = mentese_buff;
  printf("mentese_yonu_____________________: %d \n", mentese_yonu);
 }

 if (digitalRead(yon_jumper) == 0)
 {
  mentese_yonu = sol;
  printf("yon jumper ile mentese yonu ayarlandi_: %d \n", mentese_yonu);
 }

 mod = EEPROM.read(79);
 if (mod > 200)
 {
  mod = tek_kanat; //  Varsayılan Değer  tek kanat
  EEPROM.write(79, mod);
  mod = EEPROM.read(79);
  printf("mod_vrsyln______________: %d \n", mod);
 }
 else
 {
  printf("mod_____________________: %d \n", mod);
 }
 int x = EEPROM.read(80);
 if (x > 200)
 {
  demo_modu_flag = 0; //  Varsayılan Değer  tek kanat
  EEPROM.write(80, demo_modu_flag);
  demo_modu_flag = EEPROM.read(80);
  printf("mod_vrsyln______________: %d \n", demo_modu_flag);
 }
 else
 {
  demo_modu_flag = x;
  test_flag = x;
  printf("demo_modu_flag_____________________: %d \n", demo_modu_flag);
 }

 kilit_birakma_noktasi = EEPROM.read(81);
 if (kilit_birakma_noktasi > 200)
 {
  kilit_birakma_noktasi = 20; //  Varsayılan Değer  250 adim
  EEPROM.write(81, kilit_birakma_noktasi);
  kilit_birakma_noktasi = EEPROM.read(81);
  printf("kilit_birakma_noktasi_vrsyln______________: %d \n", kilit_birakma_noktasi);
 }
 else
 {
  printf("kilit_birakma_noktasi_____________________: %d \n", kilit_birakma_noktasi);
 }

 acil_stop_sayici = EEPROM.read(82) + EEPROM.read(83) * 256;
 printf("acil_stop_sayici_____________: %d \n", acil_stop_sayici);

 dusuk_voltaj_sayici = EEPROM.read(84) + EEPROM.read(85) * 256;
 printf("dusuk_voltaj_sayici_____________: %d \n", dusuk_voltaj_sayici);

 fault_sayici = EEPROM.read(86) + EEPROM.read(87) * 256;
 printf("fault_sayici_____________: %d \n", fault_sayici);

 ac_hata_sayaci = EEPROM.read(90) + EEPROM.read(91) * 256;
 printf("ac_hata_sayaci_____________: %d \n", ac_hata_sayaci);

 kapat_hata_sayaci = EEPROM.read(92) + EEPROM.read(93) * 256;
 printf("kapat_hata_sayaci_____________: %d \n", kapat_hata_sayaci);

 socket_error = EEPROM.read(94) + EEPROM.read(95) * 256;
 printf("socket_error_____________: %d \n", socket_error);

 /* Yön Jumperi Denetlemesi Yapılmaktadır */
 // jumper_kontrol();
 /* EEPROM Okumasına Devam Ediliyor */
 // kapanma_akim_hata_sayac_alt = EEPROM.read(75); //  printf("kapanma_akim_hata_sayac_alt____: %d \n", kapanma_akim_hata_sayac_alt);
 // kapanma_akim_hata_sayac_ust = EEPROM.read(76); //  printf("kapanma_akim_hata_sayac_ust____: %d \n", kapanma_akim_hata_sayac_ust);
 // acilma_akim_hata_sayac_alt = EEPROM.read(77);  //  printf("acilma_akim_hata_sayac_alt_____: %d \n", acilma_akim_hata_sayac_alt);
 // acilma_akim_hata_sayac_ust = EEPROM.read(78);  //  printf("acilma_akim_hata_sayac_ust_____: %d \n", acilma_akim_hata_sayac_ust);
 // /* EEPROM'dan Okunan Akım Kalibrasyon Değerlerin Hesaplanması*/
 // local_hesap_akim = asiri_akim_siniri;
 // maksimum_akim_degeri = local_hesap_akim / 2; //  İzin verilen maksimım akım miktarı hesaplandı.
 // printf("maksimum_akim_degeri________________: %f \n", maksimum_akim_degeri);
 // maksimum_kapi_boyu = map(kapi_acma_acisi, 0, 180, 0, 2336); // 4408                                               //  Kapının açılacağı maksimum açı hesaplandı.
 // printf("maksimum_kapi_boyu__________________: %d \n", maksimum_kapi_boyu);
 // kapi_sabit_tutma_esigi = maksimum_kapi_boyu / 100 * 5;
 // printf("kapi_sabit_tutma_esigi______________: %d \n", kapi_sabit_tutma_esigi); //  Kapı sabit tutma fonksiyonunun devreye girme eşiğidir.
 // kapi_sabit_cikma_esigi = maksimum_kapi_boyu / 100 * 3;
 // printf("kapi_sabit_cikma_esigi______________: %d \n", kapi_sabit_cikma_esigi); //  Kapı sabit tutma fonksiyonunun devreden çıkma eşiğidir.
 // /* EEPROM'dan Okunan Sayaç Değerlerin Hesaplanması*/
 kapi_basarisiz_ac_sayac = (kapi_basarisiz_ac_sayac_ust * 256) + kapi_basarisiz_ac_sayac_alt; //  Kapının başarısız açılma sayısı hesaplandı.
 // printf("kapi_basarisiz_ac_sayac_____________: %d \n", kapi_basarisiz_ac_sayac);
 kapi_ac_sayac = (kapi_ac_sayac_ust * 256) + kapi_ac_sayac_alt; //  Kapının başarılı açılma sayısı hesaplandı.
 // printf("kapi_ac_sayac_______________________: %d \n", kapi_ac_sayac);
 kapi_basarisiz_kapat_sayac = (kapi_basarisiz_kapat_sayac_ust * 256) + kapi_basarisiz_kapat_sayac_alt; //  Kapının başarısız kapanma sayısı hesaplandı.
 // printf("kapi_basarisiz_kapat_sayac__________: %d \n", kapi_basarisiz_kapat_sayac);
 kapi_kapat_sayac = (kapi_kapat_sayac_ust * 256) + kapi_kapat_sayac_alt; //  Kapının başarılı kapanma sayısı hesaplandı.
                                                                         // printf("kapi_kapat_sayac____________________: %d \n", kapi_kapat_sayac);
 restart_sayac = (restart_sayac_ust * 256) + restart_sayac_alt;          //  Kapının resetleme sayısı hesaplandı.
 // printf("restart_sayac_______________________: %d \n", restart_sayac);
 sayac_poweron_reset = (sayac_poweron_reset_ust * 256) + sayac_poweron_reset_alt; //  Güç kesintisinden dolayı oluşan reset hesaplandı.
 // printf("sayac_poweron_reset_________________: %d \n", sayac_poweron_reset);
 sayac_sw_reset = (sayac_sw_reset_ust * 256) + sayac_sw_reset_alt; //  Yazılımsal reset sayısı hesaplandı.
 // printf("sayac_sw_reset______________________: %d \n", sayac_sw_reset);
 sayac_owdt_reset = (sayac_owdt_reset_ust * 256) + sayac_owdt_reset_alt; //  Watchdog reset sayısı hesaplandı.
 // printf("sayac_owdt_reset____________________: %d \n", sayac_owdt_reset);
 sayac_deepsleep_reset = (sayac_deepsleep_reset_ust * 256) + sayac_deepsleep_reset_alt; //  Derin uyku reset sayısı hesaplandı.
 // printf("sayac_owdt_reset____________________: %d \n", sayac_owdt_reset);
 sayac_sdio_reset = (sayac_sdio_reset_ust * 256) + sayac_sdio_reset_alt;
 // printf("sayac_sdio_reset____________________: %d \n", sayac_sdio_reset);
 sayac_timer0_wd_reset = (sayac_timer0_wd_reset_ust * 256) + sayac_timer0_wd_reset_alt; //  Watchdog reset sayısı hesaplandı.
 // printf("sayac_timer0_wd_reset_______________: %d \n", sayac_timer0_wd_reset);
 sayac_timer1_wd_reset = (sayac_timer1_wd_reset_ust * 256) + sayac_timer1_wd_reset_alt; //  Watchdog reset sayısı hesaplandı.
 // printf("sayac_timer0_wd_reset_______________: %d \n", sayac_timer0_wd_reset);
 sayac_rtc_wd_reset = (sayac_rtc_wd_reset_ust * 256) + sayac_rtc_wd_reset_alt; //  Watchdog reset sayısı hesaplandı.
 // printf("sayac_rtc_wd_reset__________________: %d \n", sayac_rtc_wd_reset);
 sayac_intrusion_reset = (sayac_intrusion_reset_ust * 256) + sayac_intrusion_reset_alt; //  İşlemcinin yanlış instruction ile çalışmasından dolayı reset hesaplandı.
 // printf("sayac_intrusion_reset_______________: %d \n", sayac_intrusion_reset);
 sayac_time_group_reset = (sayac_time_group_reset_ust * 256) + sayac_time_group_reset_alt; //  Timer reset hesaplandı.
 // printf("sayac_time_group_reset______________: %d \n", sayac_time_group_reset);
 sayac_sw_cpu_reset = (sayac_sw_cpu_reset_ust * 256) + sayac_sw_cpu_reset_alt; //  Yazılımsal hard reset sayısı hesaplandı.
 // printf("sayac_sw_cpu_reset__________________: %d \n", sayac_sw_cpu_reset);
 sayac_rtc_wdc_reset = (sayac_rtc_wdc_reset_ust * 256) + sayac_rtc_wdc_reset_alt; //  Watchdog reset sayısı hesaplandı.
 // printf("sayac_rtc_wdc_reset_________________: %d \n", sayac_rtc_wdc_reset);
 sayac_ext_cpu_reset = (sayac_ext_cpu_reset_ust * 256) + sayac_ext_cpu_reset_alt; //  Birbirini resetleyen işlemci reset sayısı hesaplandı.
 // printf("sayac_ext_cpu_reset_________________: %d \n", sayac_ext_cpu_reset);
 sayac_brownout_reset = (sayac_brownout_reset_ust * 256) + sayac_brownout_reset_alt; //  Düşük voltaj reset sayısı hesaplandı.
 // printf("sayac_brownout_reset________________: %d \n", sayac_brownout_reset);
 sayac_rtc_wcdt_reset = (sayac_rtc_wcdt_reset_ust * 256) + sayac_rtc_wcdt_reset_alt; //  Watchdog reset sayısı hesaplandı.
 // printf("sayac_rtc_wcdt_reset________________: %d \n", sayac_rtc_wcdt_reset);

 /* İşlemci Hatalarının Yazdırılması */
 Serial.print("İlk Ayarlar Fn: CPU0 Reset Sebebi: ");
 print_reset_reason(rtc_get_reset_reason(0));
 Serial.print("İlk Ayarlar Fn: CPU1 Reset Sebebi: ");
 print_reset_reason(rtc_get_reset_reason(1));

 restart_sayac++; //  Toplam restartları sayan sayaçtır.
 /* İşlemci Hatalarının EEPROM'a Kaydedilmesi */
 EEPROM.write(41, (restart_sayac & 255));
 EEPROM.write(42, ((restart_sayac >> 8) & 255));
 EEPROM.write(43, (sayac_poweron_reset & 255));
 EEPROM.write(44, ((sayac_poweron_reset >> 8) & 255));
 EEPROM.write(45, (sayac_sw_reset & 255));
 EEPROM.write(46, ((sayac_sw_reset >> 8) & 255));
 EEPROM.write(47, (sayac_owdt_reset & 255));
 EEPROM.write(48, ((sayac_owdt_reset >> 8) & 255));
 EEPROM.write(49, (sayac_deepsleep_reset & 255));
 EEPROM.write(50, ((sayac_deepsleep_reset >> 8) & 255));
 EEPROM.write(51, (sayac_sdio_reset & 255));
 EEPROM.write(52, ((sayac_sdio_reset >> 8) & 255));
 EEPROM.write(53, (sayac_timer0_wd_reset & 255));
 EEPROM.write(54, ((sayac_timer0_wd_reset >> 8) & 255));
 EEPROM.write(55, (sayac_timer1_wd_reset & 255));
 EEPROM.write(56, ((sayac_timer1_wd_reset >> 8) & 255));
 EEPROM.write(57, (sayac_rtc_wd_reset & 255));
 EEPROM.write(58, ((sayac_rtc_wd_reset >> 8) & 255));
 EEPROM.write(59, (sayac_intrusion_reset & 255));
 EEPROM.write(60, ((sayac_intrusion_reset >> 8) & 255));
 EEPROM.write(61, (sayac_time_group_reset & 255));
 EEPROM.write(62, ((sayac_time_group_reset >> 8) & 255));
 EEPROM.write(63, (sayac_sw_cpu_reset & 255));
 EEPROM.write(64, ((sayac_sw_cpu_reset >> 8) & 255));
 EEPROM.write(65, (sayac_rtc_wdc_reset & 255));
 EEPROM.write(66, ((sayac_rtc_wdc_reset >> 8) & 255));
 EEPROM.write(67, (sayac_ext_cpu_reset & 255));
 EEPROM.write(68, ((sayac_ext_cpu_reset >> 8) & 255));
 EEPROM.write(69, (sayac_brownout_reset & 255));
 EEPROM.write(70, ((sayac_brownout_reset >> 8) & 255));
 EEPROM.write(71, (sayac_rtc_wcdt_reset & 255));
 EEPROM.write(72, ((sayac_rtc_wcdt_reset >> 8) & 255));
 eeproma_yaz_istegi = 1;
 printf("restart_sayac_______________________: %d \n", restart_sayac);
 printf("sayac_poweron_reset_________________: %d \n", sayac_poweron_reset);
 printf("sayac_sw_reset______________________: %d \n", sayac_sw_reset);
 printf("sayac_owdt_reset____________________: %d \n", sayac_owdt_reset);
 printf("sayac_owdt_reset____________________: %d \n", sayac_owdt_reset);
 printf("sayac_sdio_reset____________________: %d \n", sayac_sdio_reset);
 printf("sayac_timer0_wd_reset_______________: %d \n", sayac_timer0_wd_reset);
 printf("sayac_timer0_wd_reset_______________: %d \n", sayac_timer0_wd_reset);
 printf("sayac_rtc_wd_reset__________________: %d \n", sayac_rtc_wd_reset);
 printf("sayac_intrusion_reset_______________: %d \n", sayac_intrusion_reset);
 printf("sayac_time_group_reset______________: %d \n", sayac_time_group_reset);
 printf("sayac_sw_cpu_reset__________________: %d \n", sayac_sw_cpu_reset);
 printf("sayac_rtc_wdc_reset_________________: %d \n", sayac_rtc_wdc_reset);
 printf("sayac_ext_cpu_reset_________________: %d \n", sayac_ext_cpu_reset);
 printf("sayac_brownout_reset________________: %d \n", sayac_brownout_reset);
 printf("sayac_rtc_wcdt_reset________________: %d \n", sayac_rtc_wcdt_reset);

 delay(100);
}

void eeprom_kontrol()

{
 if (eeproma_yaz_istegi == 1)
 { //  Sayaç ve diğer parametre değişiklikleri burada EEPROM'a kaydedilmektedir.

  detachInterrupt(ac_pini);
  detachInterrupt(encodera);
  detachInterrupt(encoderb);
  detachInterrupt(encoderc);
  vTaskDelay(100 / portTICK_RATE_MS);
  sayac_eeprom_yaz();
  vTaskDelay(100 / portTICK_RATE_MS);
  EEPROM.commit();
  vTaskDelay(100 / portTICK_RATE_MS);
  attachInterrupt(digitalPinToInterrupt(encodera), encoder_a_kesme, CHANGE); //  Adım okumak için interrupt eklendi ve tanımlandı.
  attachInterrupt(digitalPinToInterrupt(encoderb), encoder_b_kesme, CHANGE); //  Adım okumak için interrupt eklendi ve tanımlandı.
  attachInterrupt(digitalPinToInterrupt(encoderc), encoder_c_kesme, CHANGE); //  Adım okumak için interrupt eklendi ve tanımlandı.

  if (ilk_kapanma == true)
  {
   attachInterrupt(digitalPinToInterrupt(ac_pini), pin_kesmesi_ac, RISING); //  Kapı açmak için interrupt eklendi ve tanımlandı.
   attachInterrupt(digitalPinToInterrupt(asansor_ac_pini), pin_kesmesi_asansor_ac, CHANGE);
  }

  s1_state = digitalRead(encodera);
  s2_state = digitalRead(encoderb);
  s3_state = digitalRead(encoderc);
  adim = (s1_state * 100) + (s2_state * 10) + (s3_state);
  vTaskDelay(100 / portTICK_RATE_MS);
  eeproma_yaz_istegi = 0;
  Serial.println("EEp: OK");
 }
}

void sayac_eeprom_yaz()
{
 EEPROM.write(33, (kapi_basarisiz_ac_sayac & 255));
 EEPROM.write(34, ((kapi_basarisiz_ac_sayac >> 8) & 255));
 EEPROM.write(35, (kapi_ac_sayac & 255));
 EEPROM.write(36, ((kapi_ac_sayac >> 8) & 255));
 EEPROM.write(37, (kapi_basarisiz_kapat_sayac & 255));
 EEPROM.write(38, ((kapi_basarisiz_kapat_sayac >> 8) & 255));
 EEPROM.write(39, (kapi_kapat_sayac & 255));
 EEPROM.write(40, ((kapi_kapat_sayac >> 8) & 255));

 EEPROM.write(82, (acil_stop_sayici & 255));
 EEPROM.write(83, ((acil_stop_sayici >> 8) & 255));

 EEPROM.write(84, (dusuk_voltaj_sayici & 255));
 EEPROM.write(85, ((dusuk_voltaj_sayici >> 8) & 255));

 EEPROM.write(86, (fault_sayici & 255));
 EEPROM.write(87, ((fault_sayici >> 8) & 255));

 EEPROM.write(90, ac_hata_sayaci & 255);
 EEPROM.write(91, (ac_hata_sayaci >> 8) & 255);

 EEPROM.write(92, kapat_hata_sayaci & 255);
 EEPROM.write(93, (kapat_hata_sayaci >> 8) & 255);

 EEPROM.write(94, socket_error & 255);
 EEPROM.write(95, (socket_error >> 8) & 255);

 kapi_basarisiz_ac_sayac_alt = EEPROM.read(33);    //  printf("kapi_basarisiz_ac_sayac_alt____: %d \n", kapi_basarisiz_ac_sayac_alt);
 kapi_basarisiz_ac_sayac_ust = EEPROM.read(34);    //  printf("kapi_basarisiz_ac_sayac_ust____: %d \n", kapi_basarisiz_ac_sayac_ust);
 kapi_ac_sayac_alt = EEPROM.read(35);              //  printf("kapi_ac_sayac_alt______________: %d \n", kapi_ac_sayac_alt);
 kapi_ac_sayac_ust = EEPROM.read(36);              //  printf("kapi_ac_sayac_ust______________: %d \n", kapi_ac_sayac_ust);
 kapi_basarisiz_kapat_sayac_alt = EEPROM.read(37); //  printf("kapi_basarisiz_kapat_sayac_alt_: %d \n", kapi_basarisiz_kapat_sayac_alt);
 kapi_basarisiz_kapat_sayac_ust = EEPROM.read(38); //  printf("kapi_basarisiz_kapat_sayac_ust_: %d \n", kapi_basarisiz_kapat_sayac_ust);
 kapi_kapat_sayac_alt = EEPROM.read(39);           //  printf("kapi_kapat_sayac_alt___________: %d \n", kapi_kapat_sayac_alt);
 kapi_kapat_sayac_ust = EEPROM.read(40);           //  printf("kapi_kapat_sayac_ust___________: %d \n", kapi_kapat_sayac_ust);
 restart_sayac_alt = EEPROM.read(41);              //  printf("restart_sayac_alt______________: %d \n", restart_sayac_alt);
 restart_sayac_ust = EEPROM.read(42);              //  printf("restart_sayac_ust______________: %d \n", restart_sayac_ust);
 sayac_poweron_reset_alt = EEPROM.read(43);        //  printf("sayac_poweron_reset_alt________: %d \n", sayac_poweron_reset_alt);
 sayac_poweron_reset_ust = EEPROM.read(44);        //  printf("sayac_poweron_reset_ust________: %d \n", sayac_poweron_reset_ust);
 sayac_sw_reset_alt = EEPROM.read(45);             //  printf("sayac_sw_reset_alt_____________: %d \n", sayac_sw_reset_alt);
 sayac_sw_reset_ust = EEPROM.read(46);             //  printf("sayac_sw_reset_ust_____________: %d \n", sayac_sw_reset_ust);
 sayac_owdt_reset_alt = EEPROM.read(47);           //  printf("sayac_owdt_reset_alt___________: %d \n", sayac_owdt_reset_alt);
 sayac_owdt_reset_ust = EEPROM.read(48);           //  printf("sayac_owdt_reset_ust___________: %d \n", sayac_owdt_reset_ust);
 sayac_deepsleep_reset_alt = EEPROM.read(49);      //  printf("sayac_deepsleep_reset_alt______: %d \n", sayac_deepsleep_reset_alt);
 sayac_deepsleep_reset_ust = EEPROM.read(50);      //  printf("sayac_deepsleep_reset_ust______: %d \n", sayac_deepsleep_reset_ust);
 sayac_sdio_reset_alt = EEPROM.read(51);           //  printf("sayac_sdio_reset_alt___________: %d \n", sayac_sdio_reset_alt);
 sayac_sdio_reset_ust = EEPROM.read(52);           //  printf("sayac_sdio_reset_ust___________: %d \n", sayac_sdio_reset_ust);
 sayac_timer0_wd_reset_alt = EEPROM.read(53);      //  printf("sayac_timer0_wd_reset_alt______: %d \n", sayac_timer0_wd_reset_alt);
 sayac_timer0_wd_reset_ust = EEPROM.read(54);      //  printf("sayac_timer0_wd_reset_ust______: %d \n", sayac_timer0_wd_reset_ust);
 sayac_timer1_wd_reset_alt = EEPROM.read(55);      //  printf("sayac_timer1_wd_reset_alt______: %d \n", sayac_timer1_wd_reset_alt);
 sayac_timer1_wd_reset_ust = EEPROM.read(56);      //  printf("sayac_timer1_wd_reset_ust______: %d \n", sayac_timer1_wd_reset_ust);
 sayac_rtc_wd_reset_alt = EEPROM.read(57);         //  printf("sayac_rtc_wd_reset_alt_________: %d \n", sayac_rtc_wd_reset_alt);
 sayac_rtc_wd_reset_ust = EEPROM.read(58);         //  printf("sayac_rtc_wd_reset_ust_________: %d \n", sayac_rtc_wd_reset_ust);
 sayac_intrusion_reset_alt = EEPROM.read(59);      //  printf("sayac_intrusion_reset_alt______: %d \n", sayac_intrusion_reset_alt);
 sayac_intrusion_reset_ust = EEPROM.read(60);      //  printf("sayac_intrusion_reset_ust______: %d \n", sayac_intrusion_reset_ust);
 sayac_time_group_reset_alt = EEPROM.read(61);     //  printf("sayac_time_group_reset_alt_____: %d \n", sayac_time_group_reset_alt);
 sayac_time_group_reset_ust = EEPROM.read(62);     //  printf("sayac_time_group_reset_ust_____: %d \n", sayac_time_group_reset_ust);
 sayac_sw_cpu_reset_alt = EEPROM.read(63);         //  printf("sayac_sw_cpu_reset_alt_________: %d \n", sayac_sw_cpu_reset_alt);
 sayac_sw_cpu_reset_ust = EEPROM.read(64);         //  printf("sayac_sw_cpu_reset_ust_________: %d \n", sayac_sw_cpu_reset_ust);
 sayac_rtc_wdc_reset_alt = EEPROM.read(65);        //  printf("sayac_rtc_wdc_reset_alt_________: %d \n", sayac_rtc_wdc_reset_alt);
 sayac_rtc_wdc_reset_ust = EEPROM.read(66);        //  printf("sayac_rtc_wdc_reset_ust_________: %d \n", sayac_rtc_wdc_reset_ust);
 sayac_ext_cpu_reset_alt = EEPROM.read(67);        //  printf("sayac_ext_cpu_reset_alt________: %d \n", sayac_ext_cpu_reset_alt);
 sayac_ext_cpu_reset_ust = EEPROM.read(68);        //  printf("sayac_ext_cpu_reset_ust________: %d \n", sayac_ext_cpu_reset_ust);
 sayac_brownout_reset_alt = EEPROM.read(69);       //  printf("sayac_brownout_reset_alt_______: %d \n", sayac_brownout_reset_alt);
 sayac_brownout_reset_ust = EEPROM.read(70);       //  printf("sayac_brownout_reset_ust_______: %d \n", sayac_brownout_reset_ust);
 sayac_rtc_wcdt_reset_alt = EEPROM.read(71);       //  printf("sayac_rtc_wcdt_reset_alt_______: %d \n", sayac_rtc_wcdt_reset_alt);
 sayac_rtc_wcdt_reset_ust = EEPROM.read(72);       //  printf("sayac_rtc_wcdt_reset_ust_______: %d \n", sayac_rtc_wcdt_reset_ust);
 // kapanma_akim_hata_sayac_alt =O EEPROM.read(75);                                               //  printf("kapanma_akim_hata_sayac_alt____: %d \n", kapanma_akim_hata_sayac_alt);
 // kapanma_akim_hata_sayac_ust = EEPROM.read(76);                                               //  printf("kapanma_akim_hata_sayac_ust____: %d \n", kapanma_akim_hata_sayac_ust);
 // acilma_akim_hata_sayac_alt = EEPROM.read(77);                                                //  printf("acilma_akim_hata_sayac_alt_____: %d \n", acilma_akim_hata_sayac_alt);
 // acilma_akim_hata_sayac_ust = EEPRM.read(78);                                                //  printf("acilma_akim_hata_sayac_ust_____: %d \n", acilma_akim_hata_sayac_ust);
 kapi_basarisiz_ac_sayac = (kapi_basarisiz_ac_sayac_ust * 256) + kapi_basarisiz_ac_sayac_alt;          //  Kapının başarısız açılma sayısı hesaplandı.
                                                                                                       // printf("kapi_basarisiz_ac_sayac_____________: %d \n", kapi_basarisiz_ac_sayac);
 kapi_ac_sayac = (kapi_ac_sayac_ust * 256) + kapi_ac_sayac_alt;                                        //  Kapının başarılı açılma sayısı hesaplandı.
                                                                                                       // printf("kapi_ac_sayac_______________________: %d \n", kapi_ac_sayac);
 kapi_basarisiz_kapat_sayac = (kapi_basarisiz_kapat_sayac_ust * 256) + kapi_basarisiz_kapat_sayac_alt; //  Kapının başarısız kapanma sayısı hesaplandı.
                                                                                                       // printf("kapi_basarisiz_kapat_sayac__________: %d \n", kapi_basarisiz_kapat_sayac);
 kapi_kapat_sayac = (kapi_kapat_sayac_ust * 256) + kapi_kapat_sayac_alt;                               //  Kapının başarılı kapanma sayısı hesaplandı.
                                                                                                       // printf("kapi_kapat_sayac____________________: %d \n", kapi_kapat_sayac);
 // kapanma_akim_hata_sayac = (kapanma_akim_hata_sayac_ust * 256) + kapanma_akim_hata_sayac_alt; //  Kapının akım hatası hesaplandı.
 // //  printf("kapanma_akim_hata_sayac_____________: %d \n", kapanma_akim_hata_sayac);
 // acilma_akim_hata_sayac = (acilma_akim_hata_sayac_ust * 256) + acilma_akim_hata_sayac_alt; //  Kapının akım hatası hesaplandı.
 // //  printf("acilma_akim_hata_sayac______________: %d \n", acilma_akim_hata_sayac);
 // restart_sayac = (restart_sayac_ust * 256) + restart_sayac_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("restart_sayac_______________________: %d \n", restart_sayac);
 // sayac_poweron_reset = (sayac_poweron_reset_ust * 256) + sayac_poweron_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_poweron_reset_________________: %d \n", sayac_poweron_reset);
 // sayac_sw_reset = (sayac_sw_reset_ust * 256) + sayac_sw_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_sw_reset______________________: %d \n", sayac_sw_reset);
 // sayac_owdt_reset = (sayac_owdt_reset_ust * 256) + sayac_owdt_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_owdt_reset____________________: %d \n", sayac_owdt_reset);
 // sayac_deepsleep_reset = (sayac_deepsleep_reset_ust * 256) + sayac_deepsleep_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_owdt_reset____________________: %d \n", sayac_owdt_reset);
 // sayac_sdio_reset = (sayac_sdio_reset_ust * 256) + sayac_sdio_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_sdio_reset____________________: %d \n", sayac_sdio_reset);
 // sayac_timer0_wd_reset = (sayac_timer0_wd_reset_ust * 256) + sayac_timer0_wd_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_timer0_wd_reset_______________: %d \n", sayac_timer0_wd_reset);
 // sayac_timer1_wd_reset = (sayac_timer1_wd_reset_ust * 256) + sayac_timer1_wd_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_timer0_wd_reset_______________: %d \n", sayac_timer0_wd_reset);
 // sayac_rtc_wd_reset = (sayac_rtc_wd_reset_ust * 256) + sayac_rtc_wd_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_rtc_wd_reset__________________: %d \n", sayac_rtc_wd_reset);
 // sayac_intrusion_reset = (sayac_intrusion_reset_ust * 256) + sayac_intrusion_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_intrusion_reset_______________: %d \n", sayac_intrusion_reset);
 // sayac_time_group_reset = (sayac_time_group_reset_ust * 256) + sayac_time_group_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_time_group_reset______________: %d \n", sayac_time_group_reset);
 // sayac_sw_cpu_reset = (sayac_sw_cpu_reset_ust * 256) + sayac_sw_cpu_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_sw_cpu_reset__________________: %d \n", sayac_sw_cpu_reset);
 // sayac_rtc_wdc_reset = (sayac_rtc_wdc_reset_ust * 256) + sayac_rtc_wdc_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_rtc_wdc_reset__________________: %d \n", sayac_rtc_wdc_reset);
 // sayac_ext_cpu_reset = (sayac_ext_cpu_reset_ust * 256) + sayac_ext_cpu_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_ext_cpu_reset_________________: %d \n", sayac_ext_cpu_reset);
 // sayac_brownout_reset = (sayac_brownout_reset_ust * 256) + sayac_brownout_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_brownout_reset________________: %d \n", sayac_brownout_reset);
 // sayac_rtc_wcdt_reset = (sayac_rtc_wcdt_reset_ust * 256) + sayac_rtc_wcdt_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 // //  printf("sayac_rtc_wcdt_reset________________: %d \n", sayac_rtc_wcdt_reset);
 // eeproma_yaz_istegi = 1;
 Serial.println("sayac_eeprom_yaz fn : Sayac Yaz OK");
}

static void seri_yazdir(void *arg)
{
 while (1)
 {
  vTaskDelay(50 / portTICK_RATE_MS);
  // xSemaphoreTake(UartMutex,portMAX_DELAY);
  // Serial.println("-----------------------------");
  printf("sure : %.2f rpm : %d bobin_fark_sure : %.2f hedef_sure : %.2f adim_sayisi : %d hesaplanan :%d duty :%.2f hata : %.2f sure_integral : %2.f hedefRPMharitasi : %d \n",
         sure_global, rpm, bobin_fark_sure, hedef_sure, adim_sayisi, hesaplanan, duty, hata, sure_integral, hedefRPMharitasi[adim_sayisi]);
 }
}

void sayac_sifirla_fn()
{
 // vTaskSuspend(ble_arg);
 Serial.println("Sayac Sifirla Fn");
 for (int i = 33; i < 73; i++)
 {
  EEPROM.write(i, 0);
  vTaskDelay(1 / portTICK_RATE_MS);
 }
 for (int i = 75; i < 79; i++)
 {
  EEPROM.write(i, 0);
  vTaskDelay(1 / portTICK_RATE_MS);
 }
 for (int i = 82; i < 95; i++)
 {
  EEPROM.write(i, 0);
  vTaskDelay(1 / portTICK_RATE_MS);
 }

 vTaskDelay(1000 / portTICK_RATE_MS);
 kapi_basarisiz_ac_sayac_alt = EEPROM.read(33);    // printf("kapi_basarisiz_ac_sayac_alt____: %d \n", kapi_basarisiz_ac_sayac_alt);
 kapi_basarisiz_ac_sayac_ust = EEPROM.read(34);    // printf("kapi_basarisiz_ac_sayac_ust____: %d \n", kapi_basarisiz_ac_sayac_ust);
 kapi_ac_sayac_alt = EEPROM.read(35);              //  printf("kapi_ac_sayac_alt______________: %d \n", kapi_ac_sayac_alt);
 kapi_ac_sayac_ust = EEPROM.read(36);              //  printf("kapi_ac_sayac_ust______________: %d \n", kapi_ac_sayac_ust);
 kapi_basarisiz_kapat_sayac_alt = EEPROM.read(37); //  printf("kapi_basarisiz_kapat_sayac_alt_: %d \n", kapi_basarisiz_kapat_sayac_alt);
 kapi_basarisiz_kapat_sayac_ust = EEPROM.read(38); //  printf("kapi_basarisiz_kapat_sayac_ust_: %d \n", kapi_basarisiz_kapat_sayac_ust);
 kapi_kapat_sayac_alt = EEPROM.read(39);           //  printf("kapi_kapat_sayac_alt___________: %d \n", kapi_kapat_sayac_alt);
 kapi_kapat_sayac_ust = EEPROM.read(40);           //  printf("kapi_kapat_sayac_ust___________: %d \n", kapi_kapat_sayac_ust);
 restart_sayac_alt = EEPROM.read(41);              //  printf("restart_sayac_alt______________: %d \n", restart_sayac_alt);
 restart_sayac_ust = EEPROM.read(42);              //  printf("restart_sayac_ust______________: %d \n", restart_sayac_ust);
 sayac_poweron_reset_alt = EEPROM.read(43);        //  printf("sayac_poweron_reset_alt________: %d \n", sayac_poweron_reset_alt);
 sayac_poweron_reset_ust = EEPROM.read(44);        //  printf("sayac_poweron_reset_ust________: %d \n", sayac_poweron_reset_ust);
 sayac_sw_reset_alt = EEPROM.read(45);             //  printf("sayac_sw_reset_alt_____________: %d \n", sayac_sw_reset_alt);
 sayac_sw_reset_ust = EEPROM.read(46);             //  printf("sayac_sw_reset_ust_____________: %d \n", sayac_sw_reset_ust);
 sayac_owdt_reset_alt = EEPROM.read(47);           //  printf("sayac_owdt_reset_alt___________: %d \n", sayac_owdt_reset_alt);
 sayac_owdt_reset_ust = EEPROM.read(48);           //  printf("sayac_owdt_reset_ust___________: %d \n", sayac_owdt_reset_ust);
 sayac_deepsleep_reset_alt = EEPROM.read(49);      //  printf("sayac_deepsleep_reset_alt______: %d \n", sayac_deepsleep_reset_alt);
 sayac_deepsleep_reset_ust = EEPROM.read(50);      //  printf("sayac_deepsleep_reset_ust______: %d \n", sayac_deepsleep_reset_ust);
 sayac_sdio_reset_alt = EEPROM.read(51);           //  printf("sayac_sdio_reset_alt___________: %d \n", sayac_sdio_reset_alt);
 sayac_sdio_reset_ust = EEPROM.read(52);           //  printf("sayac_sdio_reset_ust___________: %d \n", sayac_sdio_reset_ust);
 sayac_timer0_wd_reset_alt = EEPROM.read(53);      //  printf("sayac_timer0_wd_reset_alt______: %d \n", sayac_timer0_wd_reset_alt);
 sayac_timer0_wd_reset_ust = EEPROM.read(54);      //  printf("sayac_timer0_wd_reset_ust______: %d \n", sayac_timer0_wd_reset_ust);
 sayac_timer1_wd_reset_alt = EEPROM.read(55);      //  printf("sayac_timer1_wd_reset_alt______: %d \n", sayac_timer1_wd_reset_alt);
 sayac_timer1_wd_reset_ust = EEPROM.read(56);      //  printf("sayac_timer1_wd_reset_ust______: %d \n", sayac_timer1_wd_reset_ust);
 sayac_rtc_wd_reset_alt = EEPROM.read(57);         //  printf("sayac_rtc_wd_reset_alt_________: %d \n", sayac_rtc_wd_reset_alt);
 sayac_rtc_wd_reset_ust = EEPROM.read(58);         //  printf("sayac_rtc_wd_reset_ust_________: %d \n", sayac_rtc_wd_reset_ust);
 sayac_intrusion_reset_alt = EEPROM.read(59);      //  printf("sayac_intrusion_reset_alt______: %d \n", sayac_intrusion_reset_alt);
 sayac_intrusion_reset_ust = EEPROM.read(60);      //  printf("sayac_intrusion_reset_ust______: %d \n", sayac_intrusion_reset_ust);
 sayac_time_group_reset_alt = EEPROM.read(61);     //  printf("sayac_time_group_reset_alt_____: %d \n", sayac_time_group_reset_alt);
 sayac_time_group_reset_ust = EEPROM.read(62);     //  printf("sayac_time_group_reset_ust_____: %d \n", sayac_time_group_reset_ust);
 sayac_sw_cpu_reset_alt = EEPROM.read(63);         //  printf("sayac_sw_cpu_reset_alt_________: %d \n", sayac_sw_cpu_reset_alt);
 sayac_sw_cpu_reset_ust = EEPROM.read(64);         //  printf("sayac_sw_cpu_reset_ust_________: %d \n", sayac_sw_cpu_reset_ust);
 sayac_rtc_wdc_reset_alt = EEPROM.read(65);        //  printf("sayac_rtc_wdc_reset_alt_________: %d \n", sayac_rtc_wdc_reset_alt);
 sayac_rtc_wdc_reset_ust = EEPROM.read(66);        //  printf("sayac_rtc_wdc_reset_ust_________: %d \n", sayac_rtc_wdc_reset_ust);
 sayac_ext_cpu_reset_alt = EEPROM.read(67);        //  printf("sayac_ext_cpu_reset_alt________: %d \n", sayac_ext_cpu_reset_alt);
 sayac_ext_cpu_reset_ust = EEPROM.read(68);        //  printf("sayac_ext_cpu_reset_ust________: %d \n", sayac_ext_cpu_reset_ust);
 sayac_brownout_reset_alt = EEPROM.read(69);       //  printf("sayac_brownout_reset_alt_______: %d \n", sayac_brownout_reset_alt);
 sayac_brownout_reset_ust = EEPROM.read(70);       //  printf("sayac_brownout_reset_ust_______: %d \n", sayac_brownout_reset_ust);
 sayac_rtc_wcdt_reset_alt = EEPROM.read(71);       //  printf("sayac_rtc_wcdt_reset_alt_______: %d \n", sayac_rtc_wcdt_reset_alt);
 sayac_rtc_wcdt_reset_ust = EEPROM.read(72);       //  printf("sayac_rtc_wcdt_reset_ust_______: %d \n", sayac_rtc_wcdt_reset_ust);
 kapanma_akim_hata_sayac_alt = EEPROM.read(75);    //  printf("kapanma_akim_hata_sayac_alt____: %d \n", kapanma_akim_hata_sayac_alt);
 kapanma_akim_hata_sayac_ust = EEPROM.read(76);    //  printf("kapanma_akim_hata_sayac_ust____: %d \n", kapanma_akim_hata_sayac_ust);
 acilma_akim_hata_sayac_alt = EEPROM.read(77);     //  printf("acilma_akim_hata_sayac_alt_____: %d \n", acilma_akim_hata_sayac_alt);
 acilma_akim_hata_sayac_ust = EEPROM.read(78);     //  printf("acilma_akim_hata_sayac_ust_____: %d \n", acilma_akim_hata_sayac_ust);

 kapi_basarisiz_ac_sayac = (kapi_basarisiz_ac_sayac_ust * 256) + kapi_basarisiz_ac_sayac_alt; //  Kapının başarısız açılma sayısı hesaplandı.
 printf("kapi_basarisiz_ac_sayac_____________: %d \n", kapi_basarisiz_ac_sayac);
 kapi_ac_sayac = (kapi_ac_sayac_ust * 256) + kapi_ac_sayac_alt; //  Kapının başarılı açılma sayısı hesaplandı.
 printf("kapi_ac_sayac_______________________: %d \n", kapi_ac_sayac);
 kapi_basarisiz_kapat_sayac = (kapi_basarisiz_kapat_sayac_ust * 256) + kapi_basarisiz_kapat_sayac_alt; //  Kapının başarısız kapanma sayısı hesaplandı.
 printf("kapi_basarisiz_kapat_sayac__________: %d \n", kapi_basarisiz_kapat_sayac);
 kapi_kapat_sayac = (kapi_kapat_sayac_ust * 256) + kapi_kapat_sayac_alt; //  Kapının başarılı kapanma sayısı hesaplandı.
 printf("kapi_kapat_sayac____________________: %d \n", kapi_kapat_sayac);
 kapanma_akim_hata_sayac = (kapanma_akim_hata_sayac_ust * 256) + kapanma_akim_hata_sayac_alt; //  Kapının akım hatası hesaplandı.
 printf("kapanma_akim_hata_sayac_____________: %d \n", kapanma_akim_hata_sayac);
 acilma_akim_hata_sayac = (acilma_akim_hata_sayac_ust * 256) + acilma_akim_hata_sayac_alt; //  Kapının akım hatası hesaplandı.
 printf("acilma_akim_hata_sayac______________: %d \n", acilma_akim_hata_sayac);
 restart_sayac = (restart_sayac_ust * 256) + restart_sayac_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("restart_sayac_______________________: %d \n", restart_sayac);
 sayac_poweron_reset = (sayac_poweron_reset_ust * 256) + sayac_poweron_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_poweron_reset_________________: %d \n", sayac_poweron_reset);
 sayac_sw_reset = (sayac_sw_reset_ust * 256) + sayac_sw_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_sw_reset______________________: %d \n", sayac_sw_reset);
 sayac_owdt_reset = (sayac_owdt_reset_ust * 256) + sayac_owdt_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_owdt_reset____________________: %d \n", sayac_owdt_reset);
 sayac_deepsleep_reset = (sayac_deepsleep_reset_ust * 256) + sayac_deepsleep_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_owdt_reset____________________: %d \n", sayac_owdt_reset);
 sayac_sdio_reset = (sayac_sdio_reset_ust * 256) + sayac_sdio_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_sdio_reset____________________: %d \n", sayac_sdio_reset);
 sayac_timer0_wd_reset = (sayac_timer0_wd_reset_ust * 256) + sayac_timer0_wd_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_timer0_wd_reset_______________: %d \n", sayac_timer0_wd_reset);
 sayac_timer1_wd_reset = (sayac_timer1_wd_reset_ust * 256) + sayac_timer1_wd_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_timer0_wd_reset_______________: %d \n", sayac_timer0_wd_reset);
 sayac_rtc_wd_reset = (sayac_rtc_wd_reset_ust * 256) + sayac_rtc_wd_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_rtc_wd_reset__________________: %d \n", sayac_rtc_wd_reset);
 sayac_intrusion_reset = (sayac_intrusion_reset_ust * 256) + sayac_intrusion_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_intrusion_reset_______________: %d \n", sayac_intrusion_reset);
 sayac_time_group_reset = (sayac_time_group_reset_ust * 256) + sayac_time_group_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_time_group_reset______________: %d \n", sayac_time_group_reset);
 sayac_sw_cpu_reset = (sayac_sw_cpu_reset_ust * 256) + sayac_sw_cpu_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_sw_cpu_reset__________________: %d \n", sayac_sw_cpu_reset);
 sayac_rtc_wdc_reset = (sayac_rtc_wdc_reset_ust * 256) + sayac_rtc_wdc_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_rtc_wdc_reset__________________: %d \n", sayac_rtc_wdc_reset);
 sayac_ext_cpu_reset = (sayac_ext_cpu_reset_ust * 256) + sayac_ext_cpu_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_ext_cpu_reset_________________: %d \n", sayac_ext_cpu_reset);
 sayac_brownout_reset = (sayac_brownout_reset_ust * 256) + sayac_brownout_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_brownout_reset________________: %d \n", sayac_brownout_reset);
 sayac_rtc_wcdt_reset = (sayac_rtc_wcdt_reset_ust * 256) + sayac_rtc_wcdt_reset_alt; //  Kapının resetleme sayısı hesaplandı.
 printf("sayac_rtc_wcdt_reset________________: %d \n", sayac_rtc_wcdt_reset);
 acil_stop_sayici = EEPROM.read(82) + EEPROM.read(83) * 256;
 printf("acil_stop_sayici_____________: %d \n", acil_stop_sayici);
 dusuk_voltaj_sayici = EEPROM.read(84) + EEPROM.read(85) * 256;
 printf("dusuk_voltaj_sayici_____________: %d \n", dusuk_voltaj_sayici);
 fault_sayici = EEPROM.read(86) + EEPROM.read(87) * 256;
 printf("fault_sayici_____________: %d \n", fault_sayici);
 ac_hata_sayaci = EEPROM.read(90) + EEPROM.read(91) * 256;
 printf("ac_hata_sayaci_____________: %d \n", ac_hata_sayaci);
 kapat_hata_sayaci = EEPROM.read(92) + EEPROM.read(93) * 256;
 printf("kapat_hata_sayaci_____________: %d \n", kapat_hata_sayaci);
 // vTaskResume(ble_arg);
 eeproma_yaz_istegi = 1;
 // EEPROM.commit();
}

void print_reset_reason(RESET_REASON reason)
{
 switch (reason)
 {
 case 1:
  Serial.println("POWERON_RESET"); //  Bataryadan dolayı işlemci resetlendi.
  sayac_poweron_reset++;
  break;

 case 3:
  Serial.println("SW_RESET"); //  Yazılım sanal işlemciyi resetledi.
  sayac_sw_reset++;
  break;

 case 4:
  Serial.println("OWDT_RESET"); //  Legacy watch dog işlemciyi resetledi.
  sayac_owdt_reset++;
  break;

 case 5:
  Serial.println("DEEPSLEEP_RESET"); //  Deep sleep işlemciyi resetledi.
  sayac_deepsleep_reset++;
  break;

 case 6:
  Serial.println("SDIO_RESET"); //  SLC işlemciyi resetledi. I2C modülü üzerinden reset geldi.
  sayac_sdio_reset++;
  break;

 case 7:
  Serial.println("TG0WDT_SYS_RESET"); //  Timer Group0 Watch dog işlemciyi resetledi.
  sayac_timer0_wd_reset++;
  break;

 case 8:
  Serial.println("TG1WDT_SYS_RESET"); //  Timer Group1 Watch dog işlemciyi resetledi.
  sayac_timer1_wd_reset++;
  break;

 case 9:
  Serial.println("RTCWDT_SYS_RESET"); //  RTC Watch dog işlemciyi resetledi.
  sayac_rtc_wd_reset++;
  break;

 case 10:
  Serial.println("INTRUSION_RESET"); //  İşlemcide ihlal olduğu için resetlendi.
  sayac_intrusion_reset++;
  break;

 case 11:
  Serial.println("TGWDT_CPU_RESET"); //  Time group işlemciyi resetledi.
  sayac_time_group_reset++;
  break;

 case 12:
  Serial.println("SW_CPU_RESET"); //  Yazılım işlemciyi resetledi.
  sayac_sw_cpu_reset++;
  break;

 case 13:
  Serial.println("RTCWDT_CPU_RESET"); //  RTC Watch dog işlemciyi resetledi.
  sayac_rtc_wdc_reset++;
  break;

 case 14:
  Serial.println("EXT_CPU_RESET"); //  Başka bir işlemci bu işlemciyi resetledi.
  sayac_ext_cpu_reset++;
  break;

 case 15:
  Serial.println("RTCWDT_BROWN_OUT_RESET"); //  İşlemci besleme voltajı stable olmadığı için resetlendi.
  sayac_brownout_reset++;
  break;

 case 16:
  Serial.println("RTCWDT_RTC_RESET"); //  RTC watchdog RTC modülünü resetledi.
  sayac_rtc_wcdt_reset++;
  break;

 default:
  Serial.println("NO_ERR"); //  Spesifik bir hata yoktur.
 }
}

static void test_task(void *arg)
{

 while (1)
 {

  vTaskDelay(100 / portTICK_RATE_MS);
  if (test_flag == true && hareket_sinyali != kapi_ac_sinyali && adim_sayisi < 50)
  {
   vTaskDelay(3000 / portTICK_RATE_MS);
   Serial.println("Test ac sinyali....");
   ac_flag = true;
   bluetooth_kapi_ac = true;
   test_flag = false;
  }
 }
}

void dc_bara(void *arg)
{
 while (1)
 {
  vTaskDelay(80 / portTICK_RATE_MS);
  analogReadResolution(12);
  int x = 0;
  for (int i = 0; i < 10; i++)
  {
   x = x + analogRead(x2_jumper);
   vTaskDelay(2 / portTICK_RATE_MS);
  }
  x = x / 10;
  // voltaj = ((3.6 / 4095.0) * x) * (25.5 / 1.82);
  voltaj = ((3.3 / 4095.0) * x) * (22.5 / 1.72) + 2;
  if (voltaj < 18 && dusuk_voltaj_flag == 0)
  {
   // vTaskDelay(3 / portTICK_RATE_MS);
   for (int i = 0; i < 10; i++)
   {
    x = x + analogRead(x2_jumper);
    vTaskDelay(2 / portTICK_RATE_MS);
   }
   x = x / 10;
   // voltaj = ((3.6 / 4095.0) * x) * (25.5 / 1.82);
   voltaj = ((3.3 / 4095.0) * x) * (22.5 / 1.72) + 2;

   if (voltaj < 18 && dusuk_voltaj_flag == 0)
   {
    printf("dusuk voltaj: %.2f \n", voltaj);
    dusuk_voltaj_flag = 1;
    voltaj = 18;
   }
  }
  if (dusuk_voltaj_flag == 1)
  {
   dusuk_voltaj_sayici++;
   dusuk_voltaj_flag = 2;
   printf("dusuk_voltaj_sayici: %d \n", dusuk_voltaj_sayici);
   eeproma_yaz_istegi = 1;
  }
  if (voltaj >= 20 && dusuk_voltaj_flag == 2)
  {
   dusuk_voltaj_flag = 0;
  }
 }
}
void status_led_task(void *arg)
{
 while (1)
 {
  vTaskDelay(status_led_delay / portTICK_RATE_MS);
  digitalWrite(state_led, !digitalRead(state_led));
  if (baski_led_flag == true)
  {
   digitalWrite(state_led, 1);
   vTaskDelay(2000 / portTICK_RATE_MS);
   baski_led_flag = false;
  }
 }
}
/*şönt akımının okunma yöntemine göre makro değiştirilmeli*/
static void motor_akim_oku(void *arg)
{
 // delay(100);
 Serial.println("Motor Akım Oku Fn Basladi");
 uint32_t akim_zaman = 0;
 while (1)
 {
  akim_zaman = millis();
  vTaskDelay(1 / portTICK_PERIOD_MS);
  /* Akım Hesaplama */
  analog_kalmanli_akim = 0;
  analog_okunan_akim = 0;
  analog_toplam_akim = 0;

  analogReadResolution(12);

#ifdef AKIM_V2
  for (int i = 0; i < ornekleme_adedi; i++)
  {
   analog_toplam_akim += analogRead(sense_pini);
   vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  analog_toplam_akim /= ornekleme_adedi;
  analog = analog_toplam_akim;
  amper = (0.0011 * analog * analog + 0.4262 * analog + 18.603) / 1000.0;
#else
  int counter = 0;
  int counter_e = 0;
  for (int i = 0; i < ornekleme_adedi; i++)
  {
   analog_okunan_akim = analogRead(sense_pini);
   if (i > 1)
   {
    if (analog_okunan_akim > analog_eski)
    {
     analog_toplam_akim += analog_okunan_akim;
     counter++;
     analog_eski = analog_okunan_akim;
    }
    else
    {
     counter_e++;
    }
    if (counter_e > 16)
    {
     analog_toplam_akim += analog_okunan_akim;
     counter++;
     analog_eski = analog_okunan_akim;
     counter_e = 0;
    }
   }
   else
   {
    analog_eski = analog_okunan_akim;
    analog_toplam_akim += analog_okunan_akim;
    counter++;
   }
  }

  analog_toplam_akim /= counter;
  analog = simpleKalmanFilter.updateEstimate(analog_toplam_akim);
  amper = (0.0007 * analog + 0.1768); // 0.0768
#endif
  if (amper < 0)
  {
   amper = 0;
  }
  xQueueSendToBack(Amper_Queue, &amper, 0);
  // printf("akim zmani  : %d \n",(millis()-akim_zaman));
 }
}

void kalibrasyon_fn()//kullanılmıyor
{

 int a = 1;
 ledcWrite(kanal1, 2048 + a);
 ledcWrite(kanal2, 2048 - a);
 ledcWrite(kanal3, 2048 - a);
 vTaskDelay(500 / portTICK_RATE_MS);
 offset = analog;
 Serial.print("offset = ");
 Serial.println(offset);

 a = 100;
 ledcWrite(kanal1, 2048 + a);
 ledcWrite(kanal2, 2048 - a);
 ledcWrite(kanal3, 2048 - a);
 /*****************************************/
 Serial.println("Kalibrasyon Fn: 1.akim degerini girin: ");
 while (1)
 {
  vTaskDelay(200 / portTICK_RATE_MS);
  adc1 = analog - offset;
  if (akim_alindi == 1)
  { //  Akım verisi alındı değişkeni 1 olursa işlemler tamamlanır ve döngüden çıkılır.
   akim_alindi = 0;
   amper1 = ((birinci_akim_tam * 100) + birinci_akim_ondalik) / 100;
   Serial.print("Kalibrasyon Fn: Amper1 = ");
   Serial.println(amper1);
   break;
  }
 }
 /* İkinci Akım Katsayısının Belirlenmesi */
 //  kalibrasyon_enerji_ver(40);   //  Kalibrasyon için 15 PWM ile ikinci akım katsayısı belirlenmektedir.
 /*U  ve V den 1500 duty lik güç uyguladık*/
 a = 1000;
 ledcWrite(kanal1, 2048 + a);
 ledcWrite(kanal2, 2048 - a);
 ledcWrite(kanal3, 2048 - a);
 /*****************************************/

 Serial.println("Kalibrasyon Fn: 2.akim degerini girin: ");
 while (1)
 {
  vTaskDelay(200 / portTICK_RATE_MS);
  adc2 = analog - offset;
  if (akim_alindi == 1)
  { //  Akım verisi alındı değişkeni 1 olursa işlemler tamamlanır ve döngüden çıkılır.
   akim_alindi = 0;
   amper2 = ((ikinci_akim_tam * 100) + ikinci_akim_ondalik) / 100;
   Serial.print("Kalibrasyon Fn: Amper2 = ");
   Serial.println(amper2);
   break;
  }
 }
 carpan = ((amper2 - amper1) / (adc2 - adc1));
 EEPROM.write(24, birinci_akim_tam);
 EEPROM.write(25, birinci_akim_ondalik);
 EEPROM.write(26, ikinci_akim_tam);
 EEPROM.write(27, ikinci_akim_ondalik);
 EEPROM.write(29, (adc1 & 255));
 EEPROM.write(30, ((adc1 >> 8) & 255));
 EEPROM.write(31, (adc2 & 255));
 EEPROM.write(32, ((adc2 >> 8) & 255));
 EEPROM.write(28, offset); //  Offset değeri EEPROM'a kaydedildi.
 Serial.print("Kalibrasyon Fn: Carpan = ");
 Serial.println(carpan);
 /* Offset Değerinin Belirlenmesi */
 eeproma_yaz_istegi = 1;
 Serial.println("Kalibrasyon Fn: duty (500) Deneme");
 /* Kalibrasyon Değerlerinin test Edilmesi */
 /*U  ve V den 1000 duty lik güç uyguladık*/
 a = 500;
 ledcWrite(kanal1, 2048 + a);
 ledcWrite(kanal2, 2048 - a);
 ledcWrite(kanal3, 2048 - a);
 /*****************************************/
 uint32_t akim_millis = millis();
 while ((millis() < (akim_millis + 10000)))
 {
  Serial.print("Kalibrasyon Fn: analog = ");
  Serial.println(analog);
  Serial.print("Kalibrasyob Fn: Amper = ");
  Serial.println(amper);

  vTaskDelay(200 / portTICK_RATE_MS);
 }
 a = 5;
 ledcWrite(kanal1, 2048 + a);
 ledcWrite(kanal2, 2048 - a);
 ledcWrite(kanal3, 2048 - a);
 vTaskDelay(1 / portTICK_RATE_MS);
 eeproma_yaz_istegi = 1;
 kalibrasyon_aktif = 0;
}
// static void ble_task(void *arg)
// {

//  while (1)
//  {

//   vTaskDelay(500 / portTICK_RATE_MS);

//   /* BLE Data Gönderme */
//   if (mod_flag == tek_kanat)
//   {
//    ble_data_guncelle();
//   }

//   /* Kalibrasyon Kontrolü */
//   if (kalibrasyon_aktif == 1)
//   {
//    kalibrasyon_fn();
//   }
//  }
// }
static void ble_task(void *arg)
{

 while (1)
 {

  vTaskDelay(500 / portTICK_RATE_MS);
  // xSemaphoreTake(akim_mutex, 100);
  /* BLE Data Gönderme */
  if (mod_flag == tek_kanat)
  {
   if (ble_send_data_repeat == 0)
   {
    ble_data_guncelle();
   }
   else
   {
    for (int i = 0; i < ble_send_data_repeat; i++)
    {
     ble_data_send = true;
     ble_data_guncelle();
     vTaskDelay(500 / portTICK_RATE_MS);
    }
    ble_send_data_repeat = 0;
   }
   if (acil_stop_flag == true)
   {
    ble_data_send = true;
    ble_data_guncelle();
   }
  }

  /* Kalibrasyon Kontrolü */
  if (kalibrasyon_aktif == 1)
  {
   kalibrasyon_fn();
  }
  // xSemaphoreGive(akim_mutex);
 }
}

static void led_kontrol_fn(void *arg)

{
 vTaskDelay(150 / portTICK_PERIOD_MS);
 Serial.println("LED Kontrol Fn: Basladi");

 // ledcAttachPin(bl_led, led_channel);
 // ledcAttachPin(kilit_pini, kilit_channel);
 // ledcWrite(led_channel, dutyCycle);
 // ledcWrite(kilit_channel, dutyCycle);

 while (1)
 {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  /* Aydınlatma LED'inin Fade In Yapılması */
  if (aydinlatma_led_state == 1)
  { //  %100 PWM için 63 duty verilmelidir.
   xSemaphoreTake(UartMutex, portMAX_DELAY);
   Serial.println("LED Kontrol Fn: LED ON");
   xSemaphoreGive(UartMutex);
   while (dutyCycle <= 4095)
   {
    dutyCycle++;
    //  Serial.println(dutyCycle);
    ledcWrite(led_channel, dutyCycle);
    if (select_tr_mission == standart_led)
    {
     ledcWrite(kilit_channel, dutyCycle);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    if (aydinlatma_led_state == 0)
    { //  Açma ve kapama fonksiyonları ile LED kontrol arasında oluşacak senkron kaymasını önlemek içindir.
     break;
    }
   }
   while (dutyCycle > 3000)
   {
    dutyCycle--;
    ledcWrite(led_channel, dutyCycle);
    if (select_tr_mission == standart_led)
    {
     ledcWrite(kilit_channel, dutyCycle);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    if (aydinlatma_led_state == 0)
    { //  Açma ve kapama fonksiyonları ile LED kontrol arasında oluşacak senkron kaymasını önlemek içindir.
     break;
    }
   }
   if (aydinlatma_led_state == 1)
   {
    aydinlatma_led_state = 2;
   }
  }
  /* Aydınlatma LED'inin Fade Out Yapılması */
  if (aydinlatma_led_state == 0)
  {
   xSemaphoreTake(UartMutex, portMAX_DELAY);
   Serial.println("LED Kontrol Fn: LED OFF.");
   xSemaphoreGive(UartMutex);
   while (dutyCycle > 0)
   {
    dutyCycle--;
    ledcWrite(led_channel, dutyCycle);
    if (select_tr_mission == standart_led)
    {
     ledcWrite(kilit_channel, dutyCycle);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    if (aydinlatma_led_state == 1)
    { //  Açma ve kapama fonksiyonları ile LED kontrol arasında oluşacak senkron kaymasını önlemek içindir.
     break;
    }
   }
   if (aydinlatma_led_state == 0)
   {
    aydinlatma_led_state = 2;
   }
  }
  /* Print actual size of stack has used */
 }
}
/*bobin fark sürelerin ortalmasını almak için yaptık ki çok ilgisisz değerler gelmesin*/
double bobin_ortalama_alma(double fark)
{

 bobin_ortalama[fark_count] = (fark);
 fark_count++;
 if (fark_count > 5)
 {
  fark_count = 0;
 }
 double ortalama = 0;
 for (uint8_t i = 0; i < 6; i++)
 {
  ortalama = ortalama + bobin_ortalama[i];
 }
 ortalama = ortalama / 6;

 return ortalama;
}

//akım değeri yukarı ıkınca sistemi korumak için duty kısılması amacı ile yazıldı
double amper_siniri_func()
{
 double amper_sinirla = 0;
 // xSemaphoreTake(akim_mutex, 100);
 BaseType_t status;
 double amper_transfer = 0;
 status = xQueueReceive(Amper_Queue, &amper_transfer, 0);
 if (status == pdPASS)//amper değerei hesaplandı ise
 {
  if (adim_sayisi < hizlanma_boy_baslangici) // kilit  noktasında sınıf 2 den aza alınamasın diye
  {
   if (amper_transfer > 3)
   {
    amper_sinirla = (amper_transfer - 3) * 450;
    if (amper_sinirla > max_duty)
    {
     amper_sinirla = max_duty;
    }
   }
  }
  else
  {
   if (amper_transfer > akim_siniri)
   {
    amper_sinirla = (amper_transfer - akim_siniri) * 450;
    if (amper_sinirla > max_duty)
    {
     amper_sinirla = max_duty;
    }
   }
  }
  return amper_sinirla;
 }
 else
 {
  return amper_sinirla;
 }

 // xSemaphoreGive(akim_mutex);
}
void akim_sinirlama(int16_t *data)
{

 if (amper > akim_siniri && *data > akim_hesaplanan_siniri)
 {
  *data = akim_hesaplanan_siniri;
  Serial.print(".");
 }
}

/*
motor durduğunda baskı gücünü arttırınca tırıldama yapıyordu
ve motor baskıdan dolayo motor bir ileri bir geri gidiyor rpm sıfırlanmıyr.
rpmi sıfırlamak içi belli adım kontrolü yaparak motorun ilerleyip ilerlemediği
analiz eden ve rpm hesabının yapılmasına müsade eden fonksiyon

*/
void rpm_tespit_fn()
{
 for (uint8_t i = 0; i < 6; i++)
 {
  if (adim_siralama[i] == adim)
  {
   index_adim = i;

   break;
  }
 }

 if (hareket_sinyali == (kapaniyor ^ mentese_yonu))
 {

  for (uint8_t i = 1; i < 6; i++)
  {
   if ((index_adim - i) < 0)
   {
    index_gecis[i - 1] = 6 + (index_adim - i);
   }
   else
   {
    index_gecis[i - 1] = index_adim - i;
   }
  }

  if (adim_siralama[index_gecis[0]] == eski_adim_sirasi[0] &&
      adim_siralama[index_gecis[1]] == eski_adim_sirasi[1]) // &&
                                                            // adim_siralama[index_gecis[2]] == eski_adim_sirasi[2] &&
                                                            // adim_siralama[index_gecis[3]] == eski_adim_sirasi[3] &&
                                                            // adim_siralama[index_gecis[4]] == eski_adim_sirasi[4])
  {
   hiz_hesapla_flag = true;
  }
  else
  {
   hiz_hesapla_flag = false;
   // adim_hata_counter++;
  }
 }

 if (hareket_sinyali == (aciliyor ^ mentese_yonu))
 {

  for (uint8_t i = 1; i < 6; i++)
  {
   if ((index_adim + i) > 5)
   {
    index_gecis[i - 1] = ((index_adim + i) - 6);
   }
   else
   {
    index_gecis[i - 1] = index_adim + i;
   }
  }

  if (adim_siralama[index_gecis[0]] == eski_adim_sirasi[0] &&
      adim_siralama[index_gecis[1]] == eski_adim_sirasi[1]) // &&
                                                            // adim_siralama[index_gecis[2]] == eski_adim_sirasi[2] &&
                                                            // adim_siralama[index_gecis[3]] == eski_adim_sirasi[3] &&
                                                            // adim_siralama[index_gecis[4]] == eski_adim_sirasi[4])
  {
   hiz_hesapla_flag = true;
  }
  else
  {
   hiz_hesapla_flag = false;
   // adim_hata_counter++;
  }
 }
 eski_adim_sirasi[4] = eski_adim_sirasi[3];
 eski_adim_sirasi[3] = eski_adim_sirasi[2];
 eski_adim_sirasi[2] = eski_adim_sirasi[1];
 eski_adim_sirasi[1] = eski_adim_sirasi[0];
 eski_adim_sirasi[0] = adim_siralama[index_adim];

 // adim_tutucu[counter_adim] = amper;
 // hesaplanan_tutucu[counter_adim] = hesaplanan;
 // rpm_tutucu[counter_adim] = rpm;
 // bobin_fark_tutucu[counter_adim] = bobin_fark_sure;
 counter_adim++;
 ////*********************************************************/
}

void adim_oku()
{
 s3_state = digitalRead(encoderc); // Read the current W hall sensor value
 s2_state = digitalRead(encoderb); // Read the current V (or U) hall sensor value
 s1_state = digitalRead(encodera); // Read the current U (or W) hall sensor value
 adim = ((100 * s1_state) + (10 * s2_state) + (s3_state));
}

void GetUpdate()
{

 Serial.println("*******GetUpdate Fonk_*******");
 interruptCounter = 0;
 update_flag = false;
 EEPROM.write(10, update_flag);
 EEPROM.commit();
 vTaskDelay(300 / portTICK_RATE_MS);
 Serial.println("Wi-Fi Begin basladi..............");
 WiFi.begin(ssid.c_str(), password.c_str());
 vTaskDelay(1000 / portTICK_RATE_MS);
 int i = 0;
 while (WiFi.status() != WL_CONNECTED)
 {
  delay(500);
  Serial.print(".");
  if (i >= 10)
  {
   break;
  }
  i++;
 }

 if ((WiFi.status() == WL_CONNECTED))
 {
  Serial.println("WiFi'ye Baglandi");
  GetUpdateLink(host + "/update?version=" + version + "&kartTipi=" + kartTipi);
 }
 else
 {
  Serial.println("WiFi'ye Baglanamadi");
 }
 if (server_flag == true)
 {
  Serial.print("update Link:  ");
  Serial.println(payload);
  t_httpUpdate_return ret = ESPhttpUpdate.update(payload);

  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
   Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
   // ESP.restart();
   break;

  case HTTP_UPDATE_NO_UPDATES:
   Serial.println("HTTP_UPDATE_NO_UPDATES");
   // ESP.restart();
   break;

  case HTTP_UPDATE_OK:
   Serial.println("HTTP_UPDATE_OK");
   delay(500);
   // ESP.restart();
   break;
  default:
   Serial.print("_Update Hatası ");
   Serial.println(ret);

   break;
  }
 }
}

void GetUpdateLink(String url)
{
 Serial.println("Update Link Fonksiyonuna Girildi ");
 Serial.print("URL:");
 Serial.println(url);
 HTTPClient http;
 http.begin(url);

 int httpCode = http.GET();
 Serial.print("httpCode: ");
 Serial.println(httpCode);

 if (httpCode > 0)
 {

  payload = http.getString();
  Serial.print("HTTP Response Kodu: ");
  Serial.println(httpCode);
  Serial.println(payload);
  Serial.print("URL: ");
  Serial.println(url);

  char link[10];
  char test[10] = "https:";
  payload.toCharArray(link, 5);

  if (memcmp(link, test, 4) == 0)
  {
   for (uint8_t i = 0; i < 20; i++)
   {
    digitalWrite(state_led, !digitalRead(state_led));
    delay(100);
   }
   server_flag = true;
   delay(100);
   Serial.println(server_flag);
  }
 }
 else
 {
  Serial.println("HTTP Request hatasi");
 }
 http.end();
 vTaskDelay(1000 / portTICK_RATE_MS);
}

String convertToString(char *a, int size)
{
 int i;
 String s = "";
 for (i = 0; i < size; i++)
 {
  s = s + a[i];
 }
 return s;
}

void role_state(void *arg)
{
 while (1)
 {
  vTaskDelay(10 / portTICK_RATE_MS);
  if (kilit_state == 1)
  {
   kilit_ac();
   kilit_state = 0;
  }
  else if ((kilit_state == 0 && (adim_sayisi > (kilit_birakma_noktasi * 1000.0 / 82.1))) || (((millis() - kilit_timeout) > 5000) && kilit_state == 0)) //
  {
   kilit_kapat();
   kilit_state = 2; // role bosa çekilir
  }
 }
}
/**
 * @brief u v w da akım düşük çıkarsa veya abc encoderlerinde değişim olmazsa soket hatası verir
 * ekstra olarak kısa devre algılarsa tekrar soket hatasına bakmaz pwm i kapatırki devreyi korusun
 *
 */
void test_func()
{
 {

 test_tekrar:
  float temp[2];
  for (uint8_t i = 0; i < 5; i++) // dc bara hesaplanıyor
  {
   vTaskDelay(80 / portTICK_RATE_MS);
   analogReadResolution(12);
   int x = 0;
   for (int i = 0; i < 10; i++)
   {
    x = x + analogRead(x2_jumper);
    vTaskDelay(1 / portTICK_RATE_MS);
   }
   x = x / 10;
   // voltaj = ((3.6 / 4095.0) * x) * (25.5 / 1.82);
   voltaj = ((3.3 / 4095.0) * x) * (22.5 / 1.72);
  }
  voltaj_test = voltaj;
  if (voltaj <= 18)
  {
   Serial.println("dc bara hatasi...");
   Serial.println(voltaj);
   dc_hata_durumu = hatali;
  }
  else
  {
   Serial.print("dc bara : ");
   Serial.println(voltaj);
   dc_hata_durumu = saglam;
  }

  ble_yollanan_dizi_global[121] = int(voltaj_test);
  ble_yollanan_dizi_global[122] = int(voltaj_test * 100) % 100;
  // Serial.print("ble_yollanan_dizi_global[121] : ");
  // Serial.println(ble_yollanan_dizi_global[121]);
  // Serial.print("ble_yollanan_dizi_global[122] : ");
  // Serial.println(ble_yollanan_dizi_global[122]);
  PrintLog("dc+bara+" + String(voltaj));

  int a = 0;
  Serial.print("a");
  Serial.println(a);

  Serial.print("adim : ");
  Serial.println(adim);

  s1_state = digitalRead(encodera);
  s2_state = digitalRead(encoderb);
  s3_state = digitalRead(encoderc);

  a_hal_test[0] = s1_state;
  b_hal_test[0] = s2_state;
  c_hal_test[0] = s3_state;
  a = 5;
  ledcWrite(kanal1, 2048 - a);
  ledcWrite(kanal2, 2048 + a);
  ledcWrite(kanal3, 2048 + a);
  vTaskDelay(test_time / portTICK_PERIOD_MS);

  temp[0] = amper;
  printf("low amper : %.2f \n", temp[0]);
  a = 1200;
  ledcWrite(kanal1, 2048 - a);
  ledcWrite(kanal2, 2048 + a);
  ledcWrite(kanal3, 2048 + a);
  vTaskDelay(test_time / portTICK_PERIOD_MS);
  temp[1] = amper;
  printf("amper : %.2f \n", temp[1]);
  if ((temp[1] - temp[0]) < bobin_taban_akim)
  {
   Serial.println("u soket takili degil");
   PrintLog("u+soket+takili+degil");
   u_hata_durumu = hatali;
  }
  if (temp[1] > bobin_ust_akim)
  {

   ledcWrite(kanal1, 0);
   ledcWrite(kanal2, 0);
   ledcWrite(kanal3, 0);
   status_led_delay = 100;
   if (setup_flag == true)
   {
    vTaskSuspend(ac_task_arg);
    vTaskSuspend(led_kontrol_arg);
   }
   while (1)
   {
    if (setup_flag == false)
    {
     digitalWrite(state_led, !digitalRead(state_led));
    }
    Serial.println("u soketi kisa devre..");
    u_hata_durumu = kisa_devre;
    ble_data_send = true;
    vTaskDelay(100 / portTICK_RATE_MS);
   }
  }
  else if ((temp[1] - temp[0]) > bobin_taban_akim)
  {
   Serial.println(" u soket takili");
   PrintLog("u+soket+takili");
   u_hata_durumu = saglam;
  }

  Serial.print("adim : ");
  Serial.println(adim);
  s1_state = digitalRead(encodera);
  s2_state = digitalRead(encoderb);
  s3_state = digitalRead(encoderc);

  a_hal_test[1] = s1_state;
  b_hal_test[1] = s2_state;
  c_hal_test[1] = s3_state;
  a = 5; // düşük duty veriyoruz
  ledcWrite(kanal1, 2048 + a);
  ledcWrite(kanal2, 2048 - a);
  ledcWrite(kanal3, 2048 + a);
  vTaskDelay(test_time / portTICK_PERIOD_MS);
  temp[0] = amper;
  printf("low amper : %.2f \n", temp[0]);
  a = 1200; // yüksek duty veriyoruz
  ledcWrite(kanal1, 2048 + a);
  ledcWrite(kanal2, 2048 - a);
  ledcWrite(kanal3, 2048 + a);
  vTaskDelay(test_time / portTICK_PERIOD_MS);
  temp[1] = amper;
  printf("amper : %.2f \n", temp[1]);

  if ((temp[1] - temp[0]) < bobin_taban_akim) // akım farki istenilenden büyükse soket takılıdır.
  {
   Serial.println("v soket takili degil");
   PrintLog("v+soket+takili+degil");
   v_hata_durumu = hatali;
  }
  if (temp[1] > bobin_ust_akim) // akım farkı istenilen den yüksek ise aşırı akım eçkiyordur kısa devre vardır pwmler kapatılır
  {

   status_led_delay = 100;
   ledcWrite(kanal1, 0);
   ledcWrite(kanal2, 0);
   ledcWrite(kanal3, 0);
   if (setup_flag == true)
   {
    vTaskSuspend(ac_task_arg);
    vTaskSuspend(led_kontrol_arg);
   }
   while (1)
   {
    if (setup_flag == false)
    {
     digitalWrite(state_led, !digitalRead(state_led));
    }
    Serial.println("v soketi kisa devre..");
    v_hata_durumu = kisa_devre;
    ble_data_send = true;
    vTaskDelay(100 / portTICK_RATE_MS);
   }
  }
  else if ((temp[1] - temp[0]) > bobin_taban_akim)
  {
   Serial.println(" v soket takili");
   PrintLog("v+soket+takili");
   v_hata_durumu = saglam;
  }

  Serial.print("adim : ");
  Serial.println(adim);
  s1_state = digitalRead(encodera);
  s2_state = digitalRead(encoderb);
  s3_state = digitalRead(encoderc);

  a_hal_test[2] = s1_state;
  b_hal_test[2] = s2_state;
  c_hal_test[2] = s3_state;
  a = 5;
  ledcWrite(kanal1, 2048 + a);
  ledcWrite(kanal2, 2048 + a);
  ledcWrite(kanal3, 2048 - a);
  vTaskDelay(test_time / portTICK_PERIOD_MS);
  temp[0] = amper;
  printf("low amper : %.2f \n", temp[0]);
  a = 1200; //300;   ;(04/07-2025 rev);
  ledcWrite(kanal1, 2048 + a);
  ledcWrite(kanal2, 2048 + a);
  ledcWrite(kanal3, 2048 - a);
  vTaskDelay(test_time / portTICK_PERIOD_MS);
  temp[1] = amper;
  printf("amper : %.2f \n", temp[1]);

  if ((temp[1] - temp[0]) < bobin_taban_akim)
  {
   Serial.println("w soket takili degil");
   PrintLog("w+soket+takili+degil");
   w_hata_durumu = hatali;
  }
  if (temp[1] > bobin_ust_akim)
  {
   ledcWrite(kanal1, 0);
   ledcWrite(kanal2, 0);
   ledcWrite(kanal3, 0);
   status_led_delay = 100;
   if (setup_flag == true)
   {
    vTaskSuspend(ac_task_arg);
    vTaskSuspend(led_kontrol_arg);
   }
   while (1)
   {
    if (setup_flag == false)
    {
     digitalWrite(state_led, !digitalRead(state_led));
    }
    Serial.println("w soketi kisa devre..");
    w_hata_durumu = kisa_devre;
    ble_data_send = true;
    vTaskDelay(100 / portTICK_RATE_MS);
   }
  }
  else if ((temp[1] - temp[0]) > bobin_taban_akim)
  {
   Serial.println(" w soket takili");
   PrintLog("w+soket+takili");
   w_hata_durumu = saglam;
  }
  Serial.print("adim : ");
  Serial.println(adim);
  s1_state = digitalRead(encodera);
  s2_state = digitalRead(encoderb);
  s3_state = digitalRead(encoderc);

  a_hal_test[3] = s1_state;
  b_hal_test[3] = s2_state;
  c_hal_test[3] = s3_state;

  ledcWrite(kanal1, 2048 - a);
  ledcWrite(kanal2, 2048 + a);
  ledcWrite(kanal3, 2048 + a);
  vTaskDelay(test_time / portTICK_PERIOD_MS);

  Serial.print("adim : ");
  Serial.println(adim);
  s1_state = digitalRead(encodera);
  s2_state = digitalRead(encoderb);
  s3_state = digitalRead(encoderc);

  a_hal_test[4] = s1_state;
  b_hal_test[4] = s2_state;
  c_hal_test[4] = s3_state;
  a = 0;
  ledcWrite(kanal1, 2048 - a);
  ledcWrite(kanal2, 2048 + a);
  ledcWrite(kanal3, 2048 + a);

  if (memcmp(a_hal_test, hal_hata0, 5) == 0 || memcmp(a_hal_test, hal_hata1, 5) == 0)
  {
   Serial.println("hal a bozuk..");
   PrintLog("hal+a+bozuk");
   a_hata_durumu = hatali;
  }
  else
  {
   a_hata_durumu = saglam;
   Serial.println("hal a saglam..");
  }

  if (memcmp(b_hal_test, hal_hata0, 5) == 0 || memcmp(b_hal_test, hal_hata1, 5) == 0)
  {
   Serial.println("hal b bozuk..");
   PrintLog("hal+b+bozuk");
   b_hata_durumu = hatali;
  }
  else
  {
   b_hata_durumu = saglam;
   Serial.println("hal b saglam..");
  }

  if (memcmp(c_hal_test, hal_hata0, 5) == 0 || memcmp(c_hal_test, hal_hata1, 5) == 0)
  {
   Serial.println("hal c bozuk..");
   PrintLog("hal+c+bozuk");
   c_hata_durumu = hatali;
  }
  else
  {
   c_hata_durumu = saglam;
   Serial.println("hal c saglam..");
  }
  int count = 0;
  while (((w_hata_durumu + v_hata_durumu + u_hata_durumu) != 6) ||
         ((a_hata_durumu + b_hata_durumu + c_hata_durumu) != 6) || (voltaj <= 18)) // bobin hatası varsa sistem hataya geçer ve burada kalır
  {
   if (socket_flag == true)
   {
    socket_error++;
    socket_flag = false;
    printf("socket_error_____________: %d \n", socket_error);
   }

   count++;
   if (count > 50) // 5 saniyede bir soket kontrol et
   {
    baski_flag = false;
    ilk_kapama_flag = true;
    ilk_kapanma = false;
    hareket_sinyali = kapi_kapat_sinyali;
    ilk_kapanma_suresi = millis();
    hesaplanan = bekleme_duty;
    status_led_delay = 500;
    kapanirken_engel_algiladi_flag = false;
    if (setup_flag == true)
    {
     vTaskResume(ac_task_arg);
     vTaskResume(led_kontrol_arg);
    }
    goto test_tekrar;
   }

   status_led_delay = 100;
   motor_surme(0);
   Serial.println("Motor bobin, encoder yada voltaj hatasi..");
   // vTaskSuspend(hareket_kontrol_arg);
   if (setup_flag == true)
   {
    vTaskSuspend(ac_task_arg);
    vTaskSuspend(led_kontrol_arg);
   }
   if (setup_flag == false)
   {
    digitalWrite(state_led, !digitalRead(state_led));
   }
   ble_data_send = true;
   vTaskDelay(100 / portTICK_RATE_MS);
  }

  ble_data_send = true;
  ble_send_data_repeat = 5;
  socket_flag = true;
 }
}

void ble_data_al_task(void *arg)
{

 while (1)
 {
  vTaskDelay(1 / portTICK_RATE_MS);
  if (ble_data_al_flag)
  {

   if (ble_gelen_dizi.length() > 0)
   {
    Serial.println("BLE Fn:Data");
    for (int i = 0; i <= ble_gelen_dizi.length(); i++)
    {
     ble_gelen_dizi_global[i] = ble_gelen_dizi[i]; //  Gelen bilgilerin global bir diziye ataması yapıldı.
    }

    ble_gelen_dizi.clear();
    Serial.println(" ");
    ble_data_al();
   }
   ble_data_al_flag = false;
  }
 }
}
void PrintLog(String Txt)
{

 if (konsol_aktif_flag && EEPROM.read(200) < 128)
 {

  Serial.println(Txt);
  Serial.println("*******Printlog Fonk_*******");
  BLEDevice::deinit();
  if (WiFi.status() != WL_CONNECTED)
  {
   WiFi.begin(ssid.c_str(), password.c_str());
   vTaskDelay(1 / portTICK_RATE_MS);
  }

  int j = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
   vTaskDelay(300 / portTICK_RATE_MS);
   Serial.print(".");
   if (j == 10)
   {
    Serial.println("wifi ye baglanamdim.");
    break;
   }

   j++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {

   String _url = "/serial?cihaz=" + deviceKey + "&mesaj=" + Txt;
   String PostData = GetPostJson(Txt);
   PostJson(_url, PostData);
  }
 }
 // else
 // {
 //  Serial.println("*******Printlog calismiyor*******");
 // }
}

String GetPostJson(String _Val)
{
 String PostData = "cihaz=" + deviceKey + "&mesaj=" + _Val;
 return PostData;
}

void PostJson(String Url, String PostData)
{
 HTTPClient http;

 http.setTimeout(2500);
 http.begin(posthost + Url);
 Serial.println(posthost + Url);
 http.addHeader("Content-Type", "application/x-www-form-urlencoded");
 int httpResponseCode = http.POST(PostData);
 Serial.print("httpResponseCode: ");
 Serial.println(httpResponseCode);
 vTaskDelay(10 / portTICK_RATE_MS);
 http.end();
}

void print_log_baslangic()
{
 PrintLog("basladi");
 PrintLog("konsol_aktif_flag________:" + String(konsol_aktif_flag));
 PrintLog("Max_RPM___________________________:" + String(Max_RPM));
 PrintLog("push_run_flag__________________:" + String(push_run_flag));
 PrintLog("tanima_hizi_____________________:" + String(tanima_hizi));
 PrintLog("kapama_baski_suresi_____________:" + String(kapama_baski_suresi));
 PrintLog("kapama_max_rpm_________________________:" + String(kapama_max_rpm));
 PrintLog("kapama_baski_gucu___________________:" + String(kapama_baski_gucu));
 PrintLog("kapi_acma_derecesi_____________________:" + String(EEPROM.read(20) * 2));
 PrintLog("kuvvet_siniri___________________:" + String(EEPROM.read(21) * 2));
 PrintLog("asiri_akim_siniri___________________:" + String(akim_siniri));
 PrintLog("acik_kalma_suresi_____________:" + String(acik_kalma_suresi));
 PrintLog("calisma_yontemi_____________________:" + String(calisma_yontemi));
 PrintLog("amper1______________________________:" + String(amper1));
 PrintLog("amper2______________________________:" + String(amper2));
 PrintLog("offset______________________________:" + String(offset));
 PrintLog("adc1________________________________:" + String(adc1));
 PrintLog("adc2________________________________:" + String(adc2));
 PrintLog("carpan______________________________:" + String(carpan));
 PrintLog("mentese_yonu_____________________:" + String(mentese_yonu));
 if (digitalRead(yon_jumper) == 0)
 {
  PrintLog("yon+jumper+ile+mentese+yonu+ayarlandi+:" + String(mentese_yonu));
 }
 PrintLog("kapi_ac_sayac_______________________:" + String(kapi_ac_sayac));
 PrintLog("kapi_basarisiz_ac_sayac_____________:" + String(kapi_basarisiz_ac_sayac));
 PrintLog("kapi_kapat_sayac____________________:" + String(kapi_kapat_sayac));
 PrintLog("kapi_basarisiz_kapat_sayac__________:" + String(kapi_basarisiz_kapat_sayac));
 PrintLog("proje_no__________:" + String(proje_no));
}

bool connectToServer()
{
 Serial.print("Forming a connection to ");
 Serial.println(myDevice->getAddress().toString().c_str());

 BLEClient *pClient = BLEDevice::createClient();
 Serial.println(" - Created client");

 pClient->setClientCallbacks(new MyClientCallback());

 // Connect to the remove BLE Server.
 pClient->connect(myDevice); // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
 Serial.println(" - Connected to server");

 // Obtain a reference to the service we are after in the remote BLE server.
 BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
 if (pRemoteService == nullptr)
 {
  Serial.print("Failed to find our service UUID: ");
  Serial.println(serviceUUID.toString().c_str());
  pClient->disconnect();
  return false;
 }
 Serial.println(" - Found our service");

 // Obtain a reference to the characteristic in the service of the remote BLE server.
 pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
 if (pRemoteCharacteristic == nullptr)
 {
  Serial.print("Failed to find our characteristic UUID: ");
  Serial.println(charUUID.toString().c_str());
  pClient->disconnect();
  return false;
 }
 Serial.println(" - Found our characteristic");

 // Read the value of the characteristic.
 if (pRemoteCharacteristic->canRead())
 {
  std::string value = pRemoteCharacteristic->readValue();
  Serial.print("The characteristic value was: ");
  // Serial.println(value.c_str());
 }

 connected = true;
 return true;
}

void ble_client_task(void *arg)
{

 while (1)
 {
  vTaskDelay(500 / portTICK_RATE_MS);
  if (doConnect == true)
  {
   if (connectToServer())
   {
    Serial.println("We are now connected to the BLE Server.");
   }
   else
   {
    Serial.println("We have failed to connect to the server; there is nothin more we will do.");
   }
   doConnect = false;
  }
  if (connected)
  {

   if (pRemoteCharacteristic->canRead())
   {

    std::string gelen = pRemoteCharacteristic->readValue();
    for (int i = 0; i < 20; i++)
    {
     printf("i : %d data : %d\n", i, uint8_t(gelen[i]));

     server_data[i] = uint8_t(gelen[i]);
    }
   }
   /*butonlara basılınca gönderilmesi gereken datalar belirlenip gönderiyoruz*/
   if (client_data[client_ac_index] == 1 || client_data[client_kapa_index] == 1 || client_data[client_dur_index] == 1)
   {
    client_data[0] = 121;
    client_data[1] = 123;
    client_data[client_max_rpm_index] = EEPROM.read(11);
    client_data[client_kapama_max_rpm_index] = EEPROM.read(16);
    client_data[client_mentese_index] = mentese_yonu;
    client_data[client_acil_stop_index] = digitalRead(stop_pini);
    client_data[client_baski_suresi_index] = kapama_baski_suresi / (kapama_baski_suresi_ks * 2);
    client_data[client_mentese_index] = mentese_yonu;
    client_data[client_acik_kalma_suresi_index] = acik_kalma_suresi / 100;
    client_data[client_kuvvet_siniri_index] = EEPROM.read(21);
    client_data[client_push_run_index] = push_run_flag;
    client_data[client_baski_gucu_index] = kapama_baski_gucu / kapama_baski_gucu_ks;
    client_data[client_derece_index] = kapi_acma_derecesi / 2; //*
    pRemoteCharacteristic->writeValue(client_data, sizeof(client_data));
    // for (int i = 0; i < 15; i++)
    //{
    // printf("i : %d data : %d\n", i, uint8_t(client_data[i]));
    // }
   }

   client_data[client_ac_index] = 0;
   client_data[client_kapa_index] = 0;
   client_data[client_dur_index] = 0;
  }
  else if (doScan)
  {
   BLEDevice::getScan()->start(0); // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }
 }
}

void fault_task(void *arg) // faultta oluşan paraziti engellemek için oluşturulan task
{
 while (1)
 {
  vTaskDelay(1 / portTICK_RATE_MS);
  if (fault_kesme_flag_int)
  {

   if (digitalRead(fault) == 0) // ilk saykılda hemen sayıcıyı arttımayacak
   {
    vTaskDelay(50 / portTICK_RATE_MS);
    if (digitalRead(fault) == 0) // ilk saykılda hemen sayıcıyı arttımayacak
    {
     int count = 0;
     while (digitalRead(fault) == 0 && count < 100)
     {
      count++;
      vTaskDelay(10 / portTICK_RATE_MS);
     }
     if (count == 100)
     {
      Serial.print("sistem 1 saniye faultta kaldi..30 sn bekle");
      PrintLog("fault+sinyali+1+saniye");

      vTaskSuspend(hareket_kontrol_arg);
      motor_surme(0);
      for (int i = 0; i < 30; i++)
      {
       vTaskDelay(1000 / portTICK_RATE_MS);
      }
      vTaskResume(hareket_kontrol_arg);
     }

     // faulttan_kapata = false;
     // vTaskSuspend(hareket_kontrol_arg);
     // motor_surme(0);
     // fault_kesme_flag = true;
     // for (int i = 0; i < maksimum_kapi_boyu; i++) //motoru durdurduk
     // {
     //  hedefRPMharitasi[i] = 0;
     // }
     // fault_siniri++; //eproma yazma işlemini deamlı yapmasın diye counter belirledik
     fault_sayici++;
     // if (fault_siniri > 3) //counter sınırı aştıysa motor hep duracak
     // {
     //  vTaskSuspend(status_led_arg);
     //  while (1)
     //  {
     //   vTaskDelay(100 / portTICK_RATE_MS);
     //   digitalWrite(state_led, !digitalRead(state_led));
     //   printf("fault 3 den fazla sinyal verdi.sistem durduruldu.\n");
     //  }
     // }
     eeproma_yaz_istegi = 1;
     eeprom_kontrol();

     // printf("fault_siniri : %d \n", fault_siniri);
     // vTaskDelay(1000 / portTICK_RATE_MS);

     printf("faultt geldi.\n");
     PrintLog("fault+sinyali");
    }
   }
   // if (digitalRead(fault) == 1)
   // {
   //  vTaskDelay(50 / portTICK_RATE_MS);
   //  if (digitalRead(fault) == 1)
   //  {
   //   vTaskDelay(1000 / portTICK_RATE_MS);
   //   fault_kesme_flag = false;

   //   for (int i = 0; i <= maksimum_kapi_boyu; i++) //açrpm haritasi
   //   {
   //    hedefRPMharitasi[i] = tanima_hizi;
   //   }
   //   for (int i = 0; i <= maksimum_kapi_boyu; i++) //kapa rpm haritası
   //   {
   //    hedefRPMharitasi_kapa[i] = tanima_hizi;
   //   }
   //   if (digitalRead(ac_pini) == 0 && hareket_sinyali == kapi_ac_sinyali) //aç sinyali kesildiyse kapata geç
   //   {
   //    faulttan_kapata = true;
   //    Serial.println("faulttan sonra ac sinyali kesildi kapanmaya git...");
   //   }
   //   printf("fault kesildi\n");
   //   vTaskResume(hareket_kontrol_arg);
   //  }
   // }

   fault_kesme_flag_int = false;
  }
  // if (digitalRead(ac_pini) == 0 && fault_siniri > 0 && digitalRead(fault) == 1) //aç sinyali kesildiyse kapata geç
  // {
  //  vTaskDelay(100 / portTICK_RATE_MS);
  //  if (digitalRead(ac_pini) == 0 && fault_siniri > 0 && digitalRead(fault) == 1 && hareket_sinyali == kapi_ac_sinyali) //aç sinyali kesildiyse kapata geç
  //  {

  //   tanima_hizi_flag = true;
  //   Serial.println("faulttan sonra ac sinyali kesildi kapanmaya git...");
  //  }
  // }
 }
}
/*
transistor görevlerinin bir kısmı bu task içersinde kontrol edilecek
*/
void tr_mission_task(void *arg)
{
 while (1)
 {

  if (select_tr_mission == kapi_acik && adim_sayisi > 50)
  {
   digitalWrite(kilit_pini, 1);
  }
  else if (select_tr_mission == kapi_acik && adim_sayisi <= 50)
  {

   digitalWrite(kilit_pini, 0);
  }

  if (select_tr_mission == hata_cikisi)
  {
   if ((((w_hata_durumu + v_hata_durumu + u_hata_durumu) != 6) ||
        ((a_hata_durumu + b_hata_durumu + c_hata_durumu) != 6) ||
        (voltaj <= 18)))
   {
    digitalWrite(kilit_pini, 1);
   }
   else
   {
    digitalWrite(kilit_pini, 0);
   }
  }

  vTaskDelay(10 / portTICK_RATE_MS);
 }
}