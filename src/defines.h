
#ifndef defines_h
#define defines_h
#include <string>
#define EDL_select true // 6edl sistemine geçildiğinde aktif edilmeli
#define AKIM_V2 true    // LMV 321 li devremiz kullanılmaya başladığında aktif edilmeli
bool motor_ilk_tahrik_flag = false;
#define MASTER 1
#define SLAVE 0
uint8_t kapi_rutbesi = MASTER;
#define SERIAL_SIZE 50
uint8_t tx_data[SERIAL_SIZE] = {0};
uint8_t rx_data[SERIAL_SIZE] = {0};
volatile bool uart_ack_geldi = false;
uint16_t ref_adim_sayisi=0;
#define DATA_TIMEOUT 75
/*//motor sürüşünde tepki katsayısından biri bu değerler önemli.motorun hızlanma ve yavaşlama süresini engelle karşı tepki süresini değiştirir.*/
double duty_Kp = 0.0001;
double sure_Kp = 0.5;
double Kd = 0.01;
double sure_global = 0;
double sure_integral = 0;
double sure;
int rpmTespit = 0;
#define ac_button_ofset 0
#define kapi_ac 0
#define kapi_kapa 1
int kapi_acma_derecesi = 150;
#define tur_sayisi 24
#define kapat_hareketi_ofset 5
#define max_duty 1500
#define min_duty 100
#define hiz_katsayisi 10 // mobil uygulamadan gelen hız bu katsayı ile çarpılıp rpm e dönüştürülüyor
#define bekleme_duty 400
#define set_voltage 30
#define set_volatage_factor 150 // voltaj değişimin duty e etkileceğei oranın katsayısı
#define hesaplanan_min_duty 100
#define hesaplanan_max_duty 200
#define kapanma_guncelleme_noktasi 100
#define akim_hesaplanan_siniri 400
#define acma_siniri 200
#define motor_i_t_max_duty 2000 // motor ilk tahrikte akımı sınırlamak için
#define kapama_baski_gucu_ks 20
#define kapanma_baski_gucu_min 25
uint16_t baski_duty = 0;
#define aciliyor 0
#define kapaniyor 1
#define ilk_kapanma_suresi_def 25000
#define sag 0
#define sol 1
#define ac_kapat 1
#define otomatik_kapan 2
#define ac_ac 3
#define tek_kanat 0  // eğer bir clientbu durumda bağlandıysa server olacak
#define cift_kanat 1 // client olduğunu bildirecek
#define kapi_ac_sinyali int(0)
#define kapi_kapat_sinyali 1
#define kapi_bosta_sinyali 2
/*serverden gelen dataların dizi indexi*/
#define client_max_rpm_index 16
#define client_adim_sayisi_MSB_index 17
#define client_adim_sayisi_LSB_index 18
#define client_kapama_max_rpm_index 3
#define client_mentese_index 4
#define client_acil_stop_index 5
#define client_baski_suresi_index 6
#define client_acik_kalma_suresi_index 7
#define client_kuvvet_siniri_index 8
#define client_push_run_index 9
#define client_baski_gucu_index 10
#define client_derece_index 11
#define client_ac_index 12    // buton için
#define client_kapa_index 13  // buton için
#define client_dur_index 14   // buton için
#define client_kilit_index 15 // kiliti geçip geçmediğini söyleyecek
#ifdef AKIM_V2
#define test_time 50 // test func bobinlere akım uygulama süresi
#else
#define test_time 100 // test func bobinlere akım uygulama süresi
#endif
bool setup_flag = false;           // setup func yapıldımı bakmak için test func için
bool kapat_test_aktif = false;     // açarken baski yerse kapattırken test yap bayrağı
bool ac_test_aktif = false;        // kapatırken baski yerse açarken test yap bayrağı
bool engel_algilandi_flag = false; // engel algılandıında true edilecek yumuşatarak durmadaki karışıklığı önlemek için
bool acil_stop_flag = false;       // acil stop kesmesi oluştuğunu söyeyen flag
bool acil_stop_int_flag = false;   // acil stopa basıildimi kontrolunu taskta yaptırmak için kullandık
bool acil_stop_int_flag_kapat = false;
uint16_t acil_stop_sayici = 0;
bool ble_data_geldi = false;
bool acil_stop_client = true; // planalandi ama kullanilmadi
bool baski_led_flag = false;  // kapi baski aldığında ledi uzun süre yanık tutmak için
bool mod_flag = tek_kanat;    // server tarafında client baglaninca cift kanat moduna cekilerek serverincift kanatta oldugunu belirler
uint8_t mod = cift_kanat;     // client de cift kanat moduna alınması yeterli başalngıçta kontrol ediliyor
bool demo_modu_flag = false;
bool konsol_aktif_flag = false; // uzaktan print işlemlerini takip için
bool push_run_flag = false;     // ittirince kapının açılması için ayarlanan flag
bool ble_data_al_flag = false;  // ble den data gelince true edilecek
std::string ble_gelen_dizi;
bool ac_ac_flag = false;
bool ac_ac_flag_kapama = false; // tek butanla aç kapa yaparken kapa esnasında aç verilince yumuşatarak durması için kullanıyoruz
                                // #define eski_kart 1 //eski yeni kart gpio seçimi
float bobin_taban_akim = 0.0005;

float bobin_ust_akim = 2.250;
float bobin_orta_akim = bobin_ust_akim;
float akim_siniri = 1; // erken tepki versin diye 1 yaptık 2 ye gelen kadar anca sınırlayabiliyor.
double akim_siniri_d_h = 2;
bool kapattan_aca = false; // kapatta giderken ac verilince ac task teki motor ilk tahrik devreye girmesin diye yazılan flag
bool actan_kapata = false; // ac esnasında kapat gelirse açılacak flag
uint32_t ilk_kapanma_suresi = 0;
bool bluetooth_kapi_kapa = false;
// int adim_hata_counter = 0;
//  double adim_tutucu[1800];
//  int hesaplanan_tutucu[1800];
//  int rpm_tutucu[1800];
//  int bobin_fark_tutucu[1800];
bool kapanirken_engel_algiladi_flag = false; // kapanırken engel algılandığında aktif edilecek tekrar kapanırken engelli bir rpm haritası oluşturulacak
int counter_adim = 0;
#define rampa_yok_cm 100                               // rampa_yok_cm den kısa alanda baskı yediyse harita oluşturmayacak
double baski_adimi = 0;                                // hangi adımda baski yedi ise o daımı bu değişkende tutacağiz
uint8_t adim_siralama[6] = {101, 100, 110, 10, 11, 1}; // adimların dizide tutulduğu yer
int8_t index_adim = 9, index_gecis[6];                 // rpm için karşılaştırmada kullanılacak index değişkenleri
uint8_t eski_adim_sirasi[6];                           // rpm kontrolünde kullanılacak adım kontrolündde geçmiş adımı tutan değiken
bool hiz_hesapla_flag = true;                          // eğer motor doğru yönde ilerliyorsa rpm in hesaplanması için açılan fkag

uint32_t baslangic = 0, bitis = 0;

double bobin_ortalama[20]; // bobin bekleme sürelerinin ortalamasını hesaplamada kullanılan dizi

uint8_t fark_count = 0; // bekleme ortalaması alırken kullanılan counter
int status_led_delay = 500;
int kapama_max_duty = 800; // kapı kapanırkenki max duty eepromda bu değişkene aktarılır. uygulamadan ayarlanabilir durumda
int fren = 0;              // kapı ittitildiğinde kapıya fren yaptırmak için çöznürlük değerini belli oranda arttırmak için kullandık.
// #define Print_Faktor true
int zaman_timeout;
int acik_kalma_suresi = 400;
double hata = 0;
double eski_hata = 0;
volatile bool tanima_hizi_flag = false;
bool kapanma_error_flag = false;
bool baski_flag = false;
int tanima_hizi = 150;
uint16_t motor_baslangic_duty = 200;
int eeprom_size = 500;
bool mentese_yonu = true; // menteşe yonu değiştirme
int Max_RPM = 1200;
bool ilk_kapama_flag = true; // ilk kapanmadan motorilk tahrik vermek için kullandık
bool ilk_kapanma = false;    // ilk kapanma yapılıca değeri değişir
int16_t hesaplanan;
int hedefRPMharitasi[3000];
int hedefRPMharitasi_kapa[3000];
// int olusan_duty[12];
// int32_t olusan_sure[12];
// int32_t mevcut_hedef_Sure[12];
int acma_sonu_RPM = 100;
int varilacak_hiz;
double maksimum_kapi_boyu = kapi_acma_derecesi * (10000 / 821);
int rampa_boy = maksimum_kapi_boyu - 400;
int acma_baslangic_RPM = 100; // kilitten kurtuluna kadar uygulanacak rpm
int ilk_rampa_katsayi = 30;
int ikinci_rampa_katsayi = 30;

int kilit_birakma_noktasi = 0;

int hizlanma_boy_baslangici = 200; // kiliten kurtulma noktası
int yavaslama_boy_baslangici = (maksimum_kapi_boyu / 2) + (maksimum_kapi_boyu / 4);
int hizlanma_boy_bitisi = (maksimum_kapi_boyu / 2) - (maksimum_kapi_boyu / 4);
int yavaslama_boy_bitisi = maksimum_kapi_boyu - 200;
/************************************************/
int ac_ac_counter = 0;
int dutyCycle = 0; // 24 v ledlerin duty si
// int hizlanma_boy_baslangici=200;
// double hizlanma_boy_bitisi=200+((ilk_rampa_katsayi/100)*rampa_boy);
// double yavaslama_boy_baslangici=200+(((100-ikinci_rampa_katsayi)/100)*rampa_boy);
// int yavaslama_boy_bitisi = maksimum_kapi_boyu-200;
// int hizlanma_boy_bitisi = yavaslama_boy_baslangici;

/*************************************************/
int kapama_baslangic_RPM = 100;
int kapama_sonu_RPM = 100;
int kapama_max_rpm = 800;
float voltaj;
uint16_t dusuk_voltaj_sayici = 0;
uint16_t fault_sayici = 0;
// uint16_t fault_sistem_kesme_sayaci = 0;
uint8_t fault_siniri = 0; // belirlis süre fault yanıksa kabul etmek için counter koyduk
uint32_t kilit_timeout = 0;
volatile bool fault_kesme_flag = true;      // hareket kontrole fault tan sinyal geldiyse true gönderilen flag
volatile bool fault_kesme_flag_int = false; // kesmeden taskin içersine gönderilen flag
bool faulttan_kapata = false;               // faulttan sonra ac sinyali yoksa kapata gitmesi için
uint8_t dusuk_voltaj_flag = 0;
/********************************/

const int lock_channel = 5;
int kilit_state = 0;
int kilit_acik_tutma_suresi = 5; //  Kilidin kaç saniye boyunca açık kalacağını belirten değişkendir.
int kilit_millis = 0;            //  Kilidin kaç ms açık kaldığını sayan değişkendir.

double bobin_yeni_sure = 0;

bool ac_flag = false;
uint16_t ac_hata_sayaci = 0;
uint16_t ac_hata_tespit = 0;
uint16_t kapat_hata_tespit = 0;
uint16_t kapat_hata_sayaci = 0;
bool kapat_flag = false;

uint32_t ac_time_out = 0;

bool bluetooth_kapi_ac = false;

/*sayaclar***************/

byte kapi_ac_sayac_alt = 0;              //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte kapi_ac_sayac_ust = 0;              //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte kapi_kapat_sayac_alt = 0;           //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte kapi_kapat_sayac_ust = 0;           //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte kapi_basarisiz_ac_sayac_alt = 0;    //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte kapi_basarisiz_ac_sayac_ust = 0;    //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte kapi_basarisiz_kapat_sayac_alt = 0; //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte kapi_basarisiz_kapat_sayac_ust = 0; //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte kapanma_akim_hata_sayac_alt = 0;    //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte kapanma_akim_hata_sayac_ust = 0;    //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte acilma_akim_hata_sayac_alt = 0;     //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte acilma_akim_hata_sayac_ust = 0;     //  Kapı açma ve kapama sayaçları burada tanımlanmıştır.
byte restart_sayac_alt = 0;              //  Restart sayaçları burada tanımlanmıştır.
byte restart_sayac_ust = 0;              //  Restart sayaçları burada tanımlanmıştır.
byte sayac_poweron_reset_alt = 0;        //  Restart sayaçları burada tanımlanmıştır.
byte sayac_poweron_reset_ust = 0;        //  Restart sayaçları burada tanımlanmıştır.
byte sayac_sw_reset_alt = 0;             //  Restart sayaçları burada tanımlanmıştır.
byte sayac_sw_reset_ust = 0;             //  Restart sayaçları burada tanımlanmıştır.
byte sayac_owdt_reset_alt = 0;           //  Restart sayaçları burada tanımlanmıştır.
byte sayac_owdt_reset_ust = 0;           //  Restart sayaçları burada tanımlanmıştır.
byte sayac_deepsleep_reset_alt = 0;      //  Restart sayaçları burada tanımlanmıştır.
byte sayac_deepsleep_reset_ust = 0;      //  Restart sayaçları burada tanımlanmıştır.
byte sayac_sdio_reset_alt = 0;           //  Restart sayaçları burada tanımlanmıştır.
byte sayac_sdio_reset_ust = 0;           //  Restart sayaçları burada tanımlanmıştır.
byte sayac_timer0_wd_reset_alt = 0;      //  Restart sayaçları burada tanımlanmıştır.
byte sayac_timer0_wd_reset_ust = 0;      //  Restart sayaçları burada tanımlanmıştır.
byte sayac_timer1_wd_reset_alt = 0;      //  Restart sayaçları burada tanımlanmıştır.
byte sayac_timer1_wd_reset_ust = 0;      //  Restart sayaçları burada tanımlanmıştır.
byte sayac_rtc_wd_reset_alt = 0;         //  Restart sayaçları burada tanımlanmıştır.
byte sayac_rtc_wd_reset_ust = 0;         //  Restart sayaçları burada tanımlanmıştır.
byte sayac_intrusion_reset_alt = 0;      //  Restart sayaçları burada tanımlanmıştır.
byte sayac_intrusion_reset_ust = 0;      //  Restart sayaçları burada tanımlanmıştır.
byte sayac_time_group_reset_alt = 0;     //  Restart sayaçları burada tanımlanmıştır.
byte sayac_time_group_reset_ust = 0;     //  Restart sayaçları burada tanımlanmıştır.
byte sayac_sw_cpu_reset_alt = 0;         //  Restart sayaçları burada tanımlanmıştır.
byte sayac_sw_cpu_reset_ust = 0;         //  Restart sayaçları burada tanımlanmıştır.
byte sayac_rtc_wdc_reset_alt = 0;        //  Restart sayaçları burada tanımlanmıştır.
byte sayac_rtc_wdc_reset_ust = 0;        //  Restart sayaçları burada tanımlanmıştır.
byte sayac_ext_cpu_reset_alt = 0;        //  Restart sayaçları burada tanımlanmıştır.
byte sayac_ext_cpu_reset_ust = 0;        //  Restart sayaçları burada tanımlanmıştır.
byte sayac_brownout_reset_alt = 0;       //  Restart sayaçları burada tanımlanmıştır.
byte sayac_brownout_reset_ust = 0;       //  Restart sayaçları burada tanımlanmıştır.
byte sayac_rtc_wcdt_reset_alt = 0;       //  Restart sayaçları burada tanımlanmıştır.
byte sayac_rtc_wcdt_reset_ust = 0;       //  Restart sayaçları burada tanımlanmıştır.
byte eski_motor_guc = 0;
byte zorlatma_sayaci = 0;
int kapi_ac_sayac = 0;              //  Kapının kaç defa başarılı açıldığını sayan sayaçtır.
int kapi_kapat_sayac = 0;           //  Kapının kaç defa başarılı kapandğını sayan sayaçtır.
int kapi_basarisiz_ac_sayac = 0;    //  Kapının kaç defa başarısız açıldığını sayan sayaçtır.
int kapi_basarisiz_kapat_sayac = 0; //  Kapının kaç defa başarısız kapandığını sayan sayaçtır.
int sayac_poweron_reset = 0;        //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_sw_reset = 0;             //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_owdt_reset = 0;           //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_deepsleep_reset = 0;      //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_sdio_reset = 0;           //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_timer0_wd_reset = 0;      //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_timer1_wd_reset = 0;      //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_rtc_wd_reset = 0;         //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_intrusion_reset = 0;      //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_time_group_reset = 0;     //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_sw_cpu_reset = 0;         //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_rtc_wdc_reset = 0;        //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_ext_cpu_reset = 0;        //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_brownout_reset = 0;       //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int sayac_rtc_wcdt_reset = 0;       //  Reset sebeplerinin sayaçları burada tanımlanmıştır.
int acilma_akim_hata_sayac = 0;     //  Açılma fonksiyonundan akim hatası ile çıkış yapılmış ise bu sayaç arttırılır.
int kapanma_akim_hata_sayac = 0;    //  Kapanma fonksiyonundan akim hatası ile çıkış yapılmış ise bu sayaç arttırılır.
int restart_sayac = 0;              //  Kartın kaç defa resetlendiğini sayan sayaçtır.
                                    /**************************/
float analog_okunan_akim = 0;       //  Analog pinden okunan ham akım değeridir.
double analog_toplam_akim = 0;      //  Akım okuma fonksiyonunda kullanılan toplam akım değişkenidir.
double analog_eski = 0;
double analog = 0;              //  Analog pinden okunan kalmanlı ve ortalaması alınmış akım değeridir.
float analog_kalmanli_akim = 0; //  Kalibrasyon fonksiyonunda okunan PWM0 iken adc değeridir.
#ifdef AKIM_V2
const int ornekleme_adedi = 8; //  Akım okuma fonksiyonunda kullanılan döngünün uzunluğunu belirlemektedir.
#else
const int ornekleme_adedi = 128;
#endif
double amper = 0;
double amper1 = 0;         //  Kalibrasyon fonksiyonunda multimetre üzerinden okunan birinci akım değişkenidir.
double amper2 = 0;         //  Kalibrasyon fonksiyonunda multimetre üzerinden okunan ikinci akım değişkenidir.
byte adc1_alt = 0;         //  Akım kalibrasyonu sırasında analog pinden okunan değerdir.
byte adc1_ust = 0;         //  Akım kalibrasyonu sırasında analog pinden okunan değerdir.
byte adc2_alt = 0;         //  Akım kalibrasyonu sırasında analog pinden okunan değerdir.
byte adc2_ust = 0;         //  Akım kalibrasyonu sırasında analog pinden okunan değerdir.
int adc1 = 0;              //  Akım kalibrasyonu esnasında analog giriş pininden okunan birinci değerdir.
int adc2 = 0;              //  Akım kalibrasyonu esnasında analog giriş pininden okunan ikinci değerdir.
double carpan = 0;         //  adc1 ve adc2 değişkenlerinin BLE cihazdan okunan akım değerlerine kalibre edilmesi için gereken değişkendir.
byte ble_carpan = 0;       //  Float olan değişkenin BLE üzerinden geönderilmesi için oluşturulmuştur.
byte ble_amper = 0;        //  Float olan değişkenin BLE üzerinden geönderilmesi için oluşturulmuştur.
const int led_channel = 0; //  Aydınlatma LED'inin PWM kanalı ayarları burada tanımlanmıştır.
const int kilit_channel = 4;
int aydinlatma_led_state = 0; //  Aydınlatma LED'inin açık mı kapalı mı olduğu denetlenmektedir.
TaskHandle_t led_kontrol_arg = NULL;
TaskHandle_t seri_yazdir_arg = NULL;
TaskHandle_t fault_task_arg = NULL;
TaskHandle_t hareket_kontrol_arg = NULL;
TaskHandle_t test_task_arg = NULL;
TaskHandle_t role_state_arg = NULL;
TaskHandle_t dc_bara_arg = NULL;
TaskHandle_t ac_task_arg = NULL;
TaskHandle_t seri_oku_arg = NULL;
TaskHandle_t rpm_olcum_arg = NULL;
TaskHandle_t status_led_arg = NULL;
TaskHandle_t motor_akim_oku_arg = NULL;
TaskHandle_t ble_arg = NULL;
TaskHandle_t ble_data_al_task_arg = NULL;
TaskHandle_t ble_client_task_arg = NULL;
TaskHandle_t tr_mission_task_arg = NULL;
UBaseType_t uxHighWaterMark;

/*********update*/ ////////////

String ssid = "";     // "hydrolift";//"DEVAS";          //
String password = ""; //"5422373710devas";//"5422373710"; //
String version = "1.0.3.0";
String kartTipi = "kapi";
String host = "https://intense-gorge-88917.herokuapp.com";   // update dosyaları yükelenen link
String posthost = "https://mighty-peak-21985.herokuapp.com"; // log dosyalarının  gönderildiği link
String payload = "";
String data = "";
String deviceKey = "ESP1";
volatile int interruptCounter = 0;
bool server_flag = false; // Update işlemi url bayragı
bool update_flag = 0;     // Update işlemi kontrol bayragı
int flashMemory = 81;     // FlashMemory hafıza degişkeni
int get_update_fl = true;
/****************************/

/********user id ve şifre*****************************/

char user_id[50];
char wifi_password[50];
char proje_no[50];
/****************************/
// typedef union amper_parse
// {
// 	float amper;
// 	uint8_t amper_1;
//  uint8_t amper_2;
//  uint8_t amper_3;
//  uint8_t amper_4;

// }AMPER_parse;
// AMPER_parse amper1;
// AMPER_parse amper2;
bool test_flag = false;

/***********************************/

uint8_t a_hal_test[5];

uint8_t b_hal_test[5];

uint8_t c_hal_test[5];

uint8_t hal_hata1[5] = {1, 1, 1, 1, 1};
uint8_t hal_hata0[5] = {0, 0, 0, 0, 0};

uint8_t a_hata_durumu = 0;
uint8_t b_hata_durumu = 0;
uint8_t c_hata_durumu = 0;
uint8_t u_hata_durumu = 0;
uint8_t v_hata_durumu = 0;
uint8_t w_hata_durumu = 0;
uint8_t dc_hata_durumu = 0;
double voltaj_test = 0;
#define hatali 1
#define saglam 2
#define test_yapilmadi 0
#define kisa_devre 3
uint16_t socket_error = 0;
bool socket_flag = true;

/**************************************/

bool acil_actan_kapata = false;
bool acil_kapattan_aca = false;
bool ble_data_send = false;
bool ble_data_stop = false;
uint8_t ble_send_data_repeat = 0;
/*******transistor gorev atama ********/
#define standart_led 0
#define kapi_acik 1
#define hata_cikisi 2
#define kilit_cikisi 3

#define tr_ble_adres 9
uint8_t select_tr_mission = hata_cikisi;

/****tüp-ok-VK*/
#define VK 0
#define tup 1
#define octa 2
#define lift_type_adres 8

uint8_t lift_type = VK;

/*****************************/

#endif