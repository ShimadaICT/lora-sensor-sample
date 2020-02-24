/* *******************************************************************************
 * Copyright（c）2015 Thomas Telkamp and Matthijs Kooijman
 *
 *許可は、誰でも無料で許可されます。
 *この文書と添付ファイルのコピーを入手する、
 *彼らは何の制限もなく彼らと一緒にやりたいことをする。
 *複製、改変、再配布を含みますが、これに限定されません。
 *いかなる種類の保証も提供されていません。
 * H
 *この例は有効なLoRaWANパケットをペイロード "Hello、
 * world！ "と一致する周波数と暗号化設定を使用する
 *（初期プロトタイプ版）The Things Network。
 *
 *注：サブバンドのデューティサイクル制限ごとにLoRaWANが強制されます（g1で1％
 * g2の0.1％）。
 *
 DEVADDRを一意のアドレスに変更してください！
 * http://thethingsnetwork.org/wiki/AddressSpaceを参照してください。
 *
 * config.hで無線タイプを正しく定義することを忘れないでください。
 *
 *
 *必要なライブラリ： 
 * * https://github.com/matthijskooijman/arduino-lmic 
 * 
 *ハードウェアを必要とする：
 * * LoRa Shield + Arduino
 * * LoRa GPSシールド+ Arduino 
 * * LoRa Miniなど 
******************************************************************************* */
//MCCIライブラリを使う
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>
#include <DHT.h>
#include <LowPower.h>

#define SLEEP_MIN 36

// LoRaWAN NwkSKey、ネットワークセッションキー
//これはプロトタイプのTTNによって使用されるデフォルトのSemtechキーです
//最初にネットワーク。
// ttn
static const PROGMEM u1_t NWKSKEY[16] = { 0x6A, 0xCB, 0xE1, 0xCB, 0xAF, 0x38, 0x04, 0xD5, 0xA1, 0xF9, 0x2E, 0xEB, 0xCB, 0x47, 0x52, 0xA5 };

// LoRaWAN AppSKey、アプリケーションセッションキー
//これはプロトタイプのTTNによって使用されるデフォルトのSemtechキーです
//最初にネットワーク。
// ttn
static const u1_t PROGMEM APPSKEY[16] = { 0xC4, 0x4D, 0xBC, 0xA1, 0x95, 0x7D, 0xD9, 0xDB, 0x7F, 0x1E, 0x25, 0x60, 0x67, 0x02, 0xBA, 0xC5 };

// LoRaWANエンドデバイスアドレス（DevAddr）
// http://thethingsnetwork.org/wiki/AddressSpaceを参照してください。
// ttn
static const u4_t DEVADDR = 0x2604194E ; // <-- Change this address for every node!

//これらのコールバックはOTAによるアクティブ化でのみ使用されるため、
//ここに空のままにしておいてください（ただし、
// DISABLE_JOINはconfig.hで設定されます。そうしないと、リンカは不平を言います）。
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// static uint8_t mydata[] = "12345678901";
uint8_t mydata[12];
static osjob_t initjob,sendjob,blinkjob;

// 超音波Maxsonar
// FETによって制御されるセンサーを有効にする Enable FET
#define SENSOR_ENABLE 5
// センサのPWM入力がオンになっているピンを固定 Pin that sensor PWM input is on
// #define SENSOR_PIN 3
// 温度計DHT11
#define DHTTYPE DHT11
#define DHTPIN  4

// これを数秒ごとにスケジュールする（デューティにより長くなるかもしれない
// サイクル制限）
const unsigned TX_INTERVAL = 5;

//ピンマッピング
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

// バッテリー監視
/* readVcc - 現在のバッテリ電圧を読み取ります
 * @return Vcc（ミリボルト）
 */
int readVcc() {
  // AVccに対して1.1Vリファレンスを読み込む
  // リファレンスをVccに、測定値を内部1.1Vリファレンスに設定します
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  // ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  // ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

  delay(2); // Vrefが解決するのを待つ
  ADCSRA |= _BV(ADSC); // 変換を開始する
  while (bit_is_set(ADCSRA,ADSC)); // 測定する

  uint8_t low  = ADCL; // ADCLを最初に読み込む必要があります -  ADCHをロックします 
  uint8_t high = ADCH; // 両方のロックを解除

  long result = (high<<8) | low;

  result = 1125300L / result; // Vccを計算する (mV); 1125300 = 1.1*1023*1000
  return (int)result; // Vccミリボルト
}
// バッテリー監視コードの終わり

// 温度センサーコードの開始
void getTemp(){
  uint32_t outTemperature = 0.0;
  uint32_t outHumidity = 0.0;
  int outmV = 0;
  uint8_t i=0;

  // float temperature = 0.0;
  // Variables for DHT11 values
  float t = 0.0;
  float h = 0.0;
  bool Temperatura = false;
  bool Humidity = false;
  DHT dht(DHTPIN, DHTTYPE);
  
  // センサーをONにする
  digitalWrite(SENSOR_ENABLE, HIGH); 
  delay(50);

  // 温度センサーが見つかった場合は温度補正を使用する
  t = dht.readTemperature();
  h = dht.readHumidity();
  delay(50);

  // 必要なくなったセンサーをオフにする
  digitalWrite(SENSOR_ENABLE, LOW);
    
  outTemperature = t * 10; // TTN_CayenneLLP桁合わせ
  outHumidity = h * 2; // TTN_CayenneLLP桁合わせ
  
// 電圧読み込み
  int mV = readVcc();
  outmV = mV / 10;    // TTN_CayenneLLP桁合わせ
  
  mydata[0] = 0x01; // Cayenneチャンネル番号
  mydata[1] = 0x67; // 温度、アナログINは02、照度は65
  mydata[2] = outTemperature >> 8;
  mydata[3] = outTemperature >> 0;
  mydata[4] = 0x02; // Cayenneチャンネル番号
  mydata[5] = 0x68; // アナログIN、照度は65、湿度68
  mydata[6] = outHumidity >> 0;
  mydata[7] = 0x03; // Cayenneチャンネル番号
  mydata[8] = 0x02; // 温度、アナログINは02、照度は65
  mydata[9] = outmV >> 8;
  mydata[10] = outmV >> 0;
  Serial.println("");
  // 10倍のTTN-送信データ
  Serial.print(outTemperature);
  Serial.println("*C-dt");
  // 計算生データ
  Serial.print(t);
  Serial.println("*C");
  // 2倍のTTN-送信データ
  Serial.print(outHumidity);
  Serial.println("*H-dt");
  // 湿度
  Serial.print(h);
  Serial.println("*H");
  // return true; 
}
// 温度センサコードの終わり
// **************************************

void onEvent (ev_t ev) {
    // Serial.print(os_getTime());
    // Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println("EV_SCAN_TIMEOUT");
            break;
        case EV_BEACON_FOUND:
            Serial.println("EV_BEACON_FOUND");
            break;
        case EV_BEACON_MISSED:
            Serial.println("EV_BEACON_MISSED");
            break;
        case EV_BEACON_TRACKED:
            Serial.println("EV_BEACON_TRACKED");
            break;
        case EV_JOINING:
            Serial.println("EV_JOINING");
            break;
        case EV_JOINED:
            Serial.println("EV_JOINED");
            break;
        case EV_RFU1:
            Serial.println("EV_RFU1");
            break;
        case EV_JOIN_FAILED:
            Serial.println("EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            Serial.println("EV_REJOIN_FAILED");
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
            if(LMIC.dataLen) {
                // txの後にrxスロットで受信したデータ
                Serial.print("Data Received: ");
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // 次の伝送をスケジュールするマキラボVer.2スリープ
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            for (int i=0; i<SLEEP_MIN; i++) {
            // Use library from https://github.com/rocketscream/Low-Power
            // スリープ時間は8s×37 = 296s（約5分）
                // LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
                LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
            }
            do_send(&sendjob);
            break;
        case EV_LOST_TSYNC:
            Serial.println("EV_LOST_TSYNC");
            break;
        case EV_RESET:
            Serial.println("EV_RESET");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println("EV_RXCOMPLETE");
            break;
        case EV_LINK_DEAD:
            Serial.println("EV_LINK_DEAD");
            break;
        case EV_LINK_ALIVE:
            Serial.println("EV_LINK_ALIVE");
            break;
         default:
            Serial.println("Unknown event");
            break;
    }
}

void do_send(osjob_t* j){
    // 温度測定
    getTemp();
    
    // 現在のTX/RXジョブが実行されていないかどうかを確認する
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("OP_TXRXPEND, not sending");
    } else {
        // 次の可能な時にアップストリームデータ伝送を準備します
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        
        Serial.print("Packet queued");
        Serial.print(": ");
        // レシーブデータの取り扱い
        Serial.println(LMIC.freq);
        // Serial.print(": ");
        // Serial.println(ds);
    }
    // 次のTXはTX_COMPLETEイベントの後にスケジュールされます。
}

void setup() {
    Serial.begin(9600);
    while(!Serial);
    Serial.println("Starting");

    // センサーのピンを設定する
    pinMode(SENSOR_ENABLE, OUTPUT);
    digitalWrite(SENSOR_ENABLE, LOW);
    // pinMode(SENSOR_PIN, INPUT); //超音波

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
    
    // LMIC init
    os_init();
    // MAC状態をリセットします。セッションと保留中のデータ転送は破棄されます。
    LMIC_reset();

    // LMIC_setClockError（MAX_CLOCK_ERROR * 1/100）;
    // 静的セッションパラメータを設定します。動的にセッションを確立する代わりに
    // ネットワークに参加することにより、事前計算されたセッションパラメータが提供される。
    #ifdef PROGMEM
    // AVRでは、これらの値はフラッシュに保存され、RAMにのみコピーされます
    // 一度。ここでそれらを一時バッファにコピーすると、LMIC_setSessionは
    // それらを自身のバッファに再度コピーします。
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // PROGMEMでAVRを実行していない場合は、配列を直接使用してください
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // リンクチェックの有効化を無効にする
    LMIC_setLinkCheckMode(0);

    // TTNはRX2ウィンドウにSF9を使用します。
    LMIC.dn2Dr = DR_SF9;

    // データレートと送信電力を設定します（注：txpowはライブラリによって無視されるようです）
    LMIC_setDrTxpow(DR_SF7,14);
    
    // ジョブを開始する
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
