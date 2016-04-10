//#include <MsTimer2.h>
#include "pins_arduino.h"
#include <SPI.h>
#include <TimerOne.h>
#include <inttypes.h>

#include <Wire.h>
#include <Time.h>
#include <DS1307RTC.h>
#include <InputTerminal.h>
#include <Time.h>
#include <Wire.h>
#include <EEPROM.h>

#define  ON 1
#define  OFF 0

//#define DISP_SHIFTOUT       // SPIを使わない
//#define DISP_TEST           // 表示テスト
//#define KEY_TEST

// Display
#define SCK 13                // クロック信号出力ピン
#define RCK 10                // ラッチ動作出力ピン
#define SI  11                // データ信号出力ピン
//#define DISP_KIGO 4           // 9桁目

#define VFD_BLANKING 7        // HV5812 BLANKING 13pin

// Switch定義
#define BTN1 6
#define BTN2 5
#define BTN3 2
unsigned char sw_list[] = { BTN1, BTN2, BTN3 };
InputTerminal tm(sw_list, sizeof(sw_list));

/* DC-DC */
#define TIMER1_INTTIME  50      // タイマインタラプト周期
#define DCDC_PWM  9             // DCDC PWM出力Pin
#define DCDC_FDBA 3             // DCDC フィードバックAD入力Pin
#define DCDC_OUTV 700           // 28v
//#define DCDC_OUTV 550           // 22.5v
//#define DCDC_OUTV 700           // 28v
#define DCDC_PWM_PGAIN  4       // Pゲイン
#define DCDC_PWM_IGAIN  2       // Iゲイン
#define DCDC_PWM_IGAEN_MAX  200 // Iゲイン最大値
#define DCDC_PWM_MAX  250       // PWM設定値上限


/* Err Code */
#define ERR_NON        0      // エラーなし
#define ERR_DCDCFDB    1      // DCDC フィードバック電圧範囲外
#define ERR_DCDCDRV    2      // DCDC PWM出力異常

/* DISPLAY */
unsigned long keta_dat[] = {
  0x040000,    // 1
  0x080000,    // 2
  0x000001,    // 3
  0x008000,    // 4
  0x400000,    // 5
  0x000100,    // 6
  0x004000,    // 7
  0x002000,    // 8
  0x000800     // 9
};
#define DISP_00  0
#define DISP_01  1
#define DISP_02  2
#define DISP_03  3
#define DISP_04  4
#define DISP_05  5
#define DISP_06  6
#define DISP_07  7
#define DISP_08  8
#define DISP_09  9
#define DISP_K0  10
#define DISP_K1  11
#define DISP_K2  12
#define DISP_K3  13
#define DISP_NON  13

#define DISP_A  0x010000
#define DISP_B  0x020000
#define DISP_C  0x100000
#define DISP_D  0x800000
#define DISP_E  0x000200
#define DISP_F  0x000400
#define DISP_G  0x001000
#define DISP_H  0x200000

unsigned long font[] = {
  DISP_A | DISP_B | DISP_C | DISP_D | DISP_E | DISP_F,          // 0
  DISP_B | DISP_C,                                              // 1
  DISP_A | DISP_B | DISP_G | DISP_E | DISP_D,                   // 2
  DISP_A | DISP_B | DISP_C | DISP_D | DISP_G,                   // 3
  DISP_B | DISP_C | DISP_F | DISP_G,                            // 4
  DISP_A | DISP_F | DISP_G | DISP_C | DISP_D,                   // 5
  DISP_A | DISP_C | DISP_D | DISP_E | DISP_F | DISP_G,          // 6
  DISP_A | DISP_B | DISP_C | DISP_F,                            // 7
  DISP_A | DISP_B | DISP_C | DISP_D | DISP_E | DISP_F | DISP_G, // 8
  DISP_A | DISP_B | DISP_C | DISP_D | DISP_F | DISP_G,          // 9
  DISP_F,                                                       // ●
  DISP_G,                                                       // －
  DISP_F | DISP_G,                                              // ●－
  0x000000                                                     // NULL
};
#define FONT_MAX sizeof(font)
unsigned int disp[9];          // 数値表示データ
unsigned char disp_p[9];       // 各桁ピリオドデータ
// VFD点灯順行列
// 以下の行列の順にVFDは点灯する。
// 特定の桁が暗かったり明るかったりする場合は、この行列で点灯頻度を変える事で調整可能
// 明るいVFDの番号を少なく、暗いVFDの番号を多くする。
// VFD番号の順序はあまり気にしなくても大丈夫
unsigned char keta[] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9
};
/*unsigned char keta[] = {
  1,1,2,3,4,5,6,7,8,9,8,9,
  1,1,2,3,4,5,      6,7,8,9,8,9,
  1,1,2,3,4,5, 9,      6,7,8,9,8,9
};*/
#define DISP_KETAMAX sizeof(keta)
unsigned char disp_ketapwm[] = {
  10, 10, 10, 10, 10, 10, 10, 10, 10     // 0～15で設定
};
#define DISP_KETAPWM_MAX sizeof(disp_ketapwm)
#define DISP_PWM_MAX  15                  // 最大輝度0x0f

//unsigned char i,j;
unsigned char mode;      // 動作モード
#define MODE_CLOCK             0    // 時計表示
#define MODE_CLOCK_ADJ         1    // 時計調整
#define MODE_CAL               2    // カレンダー表示
#define MODE_CAL_ADJ           3    // カレンダー調整
#define MODE_BRIGHTNESS_ADJ    4    // VFD輝度調整
#define MODE_BRIGHTNESS_SAVE   5    // VFD輝度記録
#define MODE_FILAMENT_SETUP    6    // VFDフィラメント電圧調整　全消灯

#define MODE_ERR_              100  // エラー番号閾値　100以上はエラーモード定義として使用する。
#define MODE_ERR_CPU_VOLTAGE   101  // CPU電圧エラー
#define MODE_ERR_DCDC_STOP_VOLTAGE      102  // DCDC停止時電圧エラー
#define MODE_ERR_DCDC_STARTUP_VOLTAGE   101  // DCDC始動時電圧エラー
#define MODE_ERR_STATETRANSITION  MODE_CLOCK  // 状態遷移エラー
#define MODE_DEFAULT  MODE_CLOCK              // デフォルト設定

unsigned char adj_point;    // 調整場所
#define ADJ_HOUR   0
#define ADJ_MIN    1
#define ADJ_YEAR   2
#define ADJ_MONTH  3
#define ADJ_DAY    4
#define ADJ_BR1    0
#define ADJ_BR2    1
#define ADJ_BR3    2
#define ADJ_BR4    3
#define ADJ_BR5    4
#define ADJ_BR6    5
#define ADJ_BR7    6
#define ADJ_BR8    7
#define ADJ_BR9    8
unsigned char adj_runf;    // 調整実行したフラグ
unsigned char adj_data[3];
unsigned char adj_datamax[] = { 23, 59 , 99, 12, 31};
unsigned char adj_datamini[] = { 0, 0 , 0, 1, 1};

#define CL_ADJ_DIGUP  0  // 時刻変更桁変更
#define CL_ADJ_UP 1
#define CL_ADJ_DOWN 2
#define CAL_ADJ_DIGUP 0  // カレンダー調整桁変更
#define CAL_ADJ_UP  1
#define CAL_ADJ_DOWN  2

tmElements_t tim;

unsigned int count;
unsigned int second_counterw;
unsigned char date_time[7];
unsigned int key_now;           // 入力されたキー入力データ

int dcdc_ini(void);
void dcdc_ctr(int setpwmw);
void disp_ini(void);
void disp_vfd_iv21(void);

//void itm_ini(void);
//void itm(void);
void keyman(void);

void rtc_chk(void);                            // RTC読み出し
void clockdisplay(unsigned char *disp_tmp, unsigned char *piriod_tmp);   // 時刻表示データ作成

// LED表示設定
#define LED_PIN         4           // LED port
#define LED_BRIGHT_MAX  31          // LED表示輝度MAX
#define LED_BRIGHT      2           // LED表示輝度
unsigned char led_statew;           // LED状態
unsigned long led_previous_millis;  // 前回LED_state更新時間
unsigned long led_interval;         // LED点滅間隔
#define LED_INTERVAL_SET  200       // LED点滅間隔時間基本値 = 200mSec
unsigned char led_blink_sqf;        // LED点滅シーケンス
#define LED_BLINKSQ_HED  0          // ヘッダ
#define LED_BLINKSQ_SP1  1          // スペーサ１
#define LED_BLINKSQ_COM  2          // コマンド
#define LED_BLINKSQ_SP2  3          // スペーサ２

void setup() {
  int startmode_sw;

  Serial.begin(9600);
  while (!Serial) ; // wait for Arduino Serial Monitor

  startmode_sw = digitalRead(sw_list[0]);  // 起動時SW1読込み

  mode = MODE_CLOCK;
  second_counterw = int(1000000 / TIMER1_INTTIME);  // 1秒間の割り込み回数
  count = 0;
  key_now = 0;

  // DCDC Setup
  dcdc_ini();        // DCDC制御初期化

  // LED Setup
  led_ini();

  // VFD Setup
  disp_ini();        // VFD表示初期化

  Timer1.attachInterrupt(int_count);
//  MsTimer2::set(2, int_count); // 500ms period
//  MsTimer2::start();

  // RTC Setup
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  
  // mode Setup
  if (mode < MODE_ERR_) { // エラー発生なし
    if (startmode_sw == 0) { // 電源ON時SW1入力あり
      modeset(MODE_FILAMENT_SETUP);    // VFDフィラメント電圧調整
      Serial.println("Mode : Filament Setup.");
      // LED表示 VFD表示調整モード
      // DCDC OFF
    }
    else {
      modeset(MODE_CLOCK);
      Serial.println("Mode : Clock.");
    }
  }
  else {   // エラー発生
    // DCDC OFF
  }

}

void loop() {

  dcdc_ctr(DCDC_OUTV);        // DCDC制御
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  rtc_chk();                  // RTC読み出し
  disp_datamake();            // 表示データ作成
  tm.scan();
  keyman();

  led_man(mode);

}

void modeset(unsigned char setmode)
{
  if (setmode == MODE_CLOCK) {
    mode = MODE_CLOCK;
      Serial.println("Mode : Clock.");
  }
  else if (setmode == MODE_CLOCK_ADJ) {
    adj_point = ADJ_HOUR;    // 時間調整から開始する
    adj_runf = OFF;          // 調整実行フラグ初期化
    adj_data[ADJ_HOUR - ADJ_HOUR] = hour();
    adj_data[ADJ_MIN - ADJ_HOUR] = minute();
    mode = MODE_CLOCK_ADJ;
      Serial.println("Mode : Clock ADJ.");
  }
  else if (setmode == MODE_CAL) {
    mode = MODE_CAL;
      Serial.println("Mode : Calender.");
  }
  else if (setmode == MODE_CAL_ADJ) {
    adj_point = ADJ_YEAR;    // 年調整から開始する
    adj_runf = OFF;          // 調整実行フラグ初期化
    //    adj_data[ADJ_YEAR - ADJ_YEAR] = 現在年;
    //    adj_data[ADJ_MONTH - ADJ_YEAR] = 現在月;
    //    adj_data[ADJ_DAY - ADJ_YEAR] = 現在日;
    mode = MODE_CAL_ADJ;
      Serial.println("Mode : Calender ADJ.");
  }
  else if (setmode == MODE_BRIGHTNESS_ADJ) {
    adj_point = ADJ_BR1;     // 1桁目から開始する
    adj_runf = OFF;          // 調整実行フラグ初期化
    mode = MODE_BRIGHTNESS_ADJ;
      Serial.println("Mode : Brightness ADJ.");
  }
  else if (setmode == MODE_BRIGHTNESS_SAVE) {
//    brightness_eeprom_save();    // 輝度をEEPROMに保存
    modeset(MODE_CLOCK);         // 動作モードを時計表示にする。
  }
  else if (setmode == MODE_FILAMENT_SETUP) {
    mode = MODE_FILAMENT_SETUP;
  }

  return;
}

void rtc_chk(void) {
  date_time[0] = second();
  if (mode == MODE_CLOCK) {
    date_time[1] = minute();
    date_time[2] = hour();
  }
  else if (mode == MODE_CAL) {
    date_time[3] = day();
    //    date_time[4] = Rtc.weekdays();
    date_time[5] = month();
    date_time[6] = year();
  }
/*  else if (mode == MODE_RTC_TEST) {
    date_time[1] = minute();
    date_time[2] = hour();
    date_time[3] = day();
    //    date_time[4] = Rtc.weekdays();
    date_time[5] = month();
    date_time[6] = year();
  }
*/
  return;

}

void disp_datamake(void) {
  unsigned int i;
  unsigned char disp_tmp[9];
  unsigned char piriod_tmp[8];

#ifdef KEY_TEST
  disp_tmp[0] = key_now % 10;
  disp_tmp[1] = key_now / 10;
  disp_tmp[2] = DISP_NON;
  disp_tmp[3] = DISP_NON;
  disp_tmp[4] = DISP_NON;
  disp_tmp[5] = DISP_NON;
  disp_tmp[6] = DISP_NON;
  disp_tmp[7] = DISP_NON;
  disp_tmp[8] = DISP_NON;
  piriod_tmp[0] = 0x01;
  piriod_tmp[1] = 0x01;
  piriod_tmp[2] = 0x01;
  piriod_tmp[3] = 0x01;
  piriod_tmp[4] = 0x01;
  piriod_tmp[5] = 0x01;
  piriod_tmp[6] = 0x01;
  piriod_tmp[7] = 0x01;
  piriod_tmp[8] = 0x01;
#else
  if (mode == MODE_CLOCK) {
  // 時刻情報作成
    clock_display(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_CLOCK_ADJ) {
    clock_adj_dispdat_make(disp_tmp, piriod_tmp);
    }
  else if (mode == MODE_CAL) {
    calender_display(disp_tmp, piriod_tmp);
    }
  else if (mode == MODE_CAL_ADJ) {}
  else if (mode == MODE_BRIGHTNESS_ADJ) { brightness_adj_dispdat_make(disp_tmp, piriod_tmp); }
  else if (mode == MODE_FILAMENT_SETUP) { disp_alloff(disp_tmp, piriod_tmp);}
#endif

  noInterrupts();      // 割り込み禁止
  for (i = 0; i < 9; i++) {
    disp[i] = disp_tmp[i];
    disp_p[i] = piriod_tmp[i];
  }
  interrupts();        // 割り込み許可

  return;
}
void clock_display(unsigned char *disp_tmp, unsigned char *piriod_tmp)
{
#ifdef DISP_TEST
  disp_tmp[0] = date_time[0] % 10;
  disp_tmp[1] = disp_tmp[0];
  disp_tmp[2] = disp_tmp[0];
  disp_tmp[3] = disp_tmp[0];
  disp_tmp[4] = disp_tmp[0];
  disp_tmp[5] = disp_tmp[0];
  disp_tmp[6] = disp_tmp[0];
  disp_tmp[7] = disp_tmp[0];
  disp_tmp[8] = disp_tmp[0];
  piriod_tmp[0] = 0x01;
  piriod_tmp[1] = 0x01;
  piriod_tmp[2] = 0x01;
  piriod_tmp[3] = 0x01;
  piriod_tmp[4] = 0x01;
  piriod_tmp[5] = 0x01;
  piriod_tmp[6] = 0x01;
  piriod_tmp[7] = 0x01;
  piriod_tmp[8] = 0x01;
#else
/*
  disp_tmp[0] = date_time[0] % 10;
  disp_tmp[1] = date_time[0] / 10;
  disp_tmp[2] = date_time[1] % 10;
  disp_tmp[3] = date_time[1] / 10;
  disp_tmp[4] = date_time[2] % 10;
  disp_tmp[5] = date_time[2] / 10;
  disp_tmp[6] = DISP_K3;
  disp_tmp[7] = DISP_K3;
//  disp_tmp[8] = DISP_K3;
*/
  disp_tmp[0] = DISP_K3;
  disp_tmp[1] = date_time[0] % 10;
  disp_tmp[2] = date_time[0] / 10;
  disp_tmp[3] = date_time[1] % 10;
  disp_tmp[4] = date_time[1] / 10;
  disp_tmp[5] = date_time[2] % 10;
  disp_tmp[6] = date_time[2] / 10;
  disp_tmp[7] = DISP_K3;

  if (count >= (second_counterw / 2)) {
    // ピリオド消灯処理
    disp_tmp[8] = DISP_NON;
  }
  else {
    // ピリオド点灯処理
    disp_tmp[8] = DISP_K0;
  }

//  piriod_tmp[0] = piriod_tmp[2] = piriod_tmp[4] = 0x01;
//  piriod_tmp[1] = piriod_tmp[3] = piriod_tmp[5] = piriod_tmp[6] = piriod_tmp[7] = piriod_tmp[8] = 0x00;
  piriod_tmp[1] = piriod_tmp[3] = piriod_tmp[5] = 0x01;
  piriod_tmp[0] = piriod_tmp[2] = piriod_tmp[4] = piriod_tmp[6] = piriod_tmp[7] = piriod_tmp[8] = 0x00;
#endif

  return;
}
void disp_alloff(unsigned char *disp_tmp, unsigned char *piriod_tmp){
  for(unsigned char i=0;i<9;i++){
    disp_tmp[i] = DISP_NON;
    piriod_tmp[i]=0;
  }

  return;
}

void brightness_adj_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp){
  for(unsigned char i=0;i<9;i++){
    disp_tmp[i] = DISP_08;
    piriod_tmp[i]=0x00;
  }
  if (count >= (second_counterw / 2)) {
    // ピリオド消灯処理
    piriod_tmp[adj_point] = 0x00;
  }
  else {
    // ピリオド点灯処理
    piriod_tmp[adj_point] = 0x01;
  }

  return;
}


/* キー入力処理 */
void keyman(void)
{
  unsigned int shortonw;
  unsigned int longonw;

  shortonw = tm.read_s();
  longonw = tm.read_l();

  if (shortonw != 0) {
    switch (shortonw) {
      case 0x01:
        if (mode == MODE_BRIGHTNESS_ADJ) {
//          brightness_adj(BR_ADJ_DIGUP);
        }
        else if (mode == MODE_CAL_ADJ) {
//          calender_adj(CAL_ADJ_DIGUP);
        }
        else if (mode == MODE_CLOCK_ADJ) {
          clock_adj(CL_ADJ_DIGUP);
        }
        Serial.println("S1");
        break;
      case 0x02:
        if (mode == MODE_BRIGHTNESS_ADJ) {
//          brightness_adj(BR_ADJ_BRUP);
        }
        else if (mode == MODE_CAL_ADJ) {
//          calender_adj(CAL_ADJ_UP);
        }
        else if (mode == MODE_CLOCK_ADJ) {
          clock_adj(CL_ADJ_UP);
        }
        Serial.println("S2");
//        digitalWrite(VFD_BLANKING, HIGH);
        break;
      case 0x03:
//        Serial.println("S3");
        break;
      case 0x04:
        if (mode == MODE_BRIGHTNESS_ADJ) {
//          brightness_adj(BR_ADJ_BRDOWN);
        }
        else if (mode == MODE_CAL_ADJ) {
//          calender_adj(CAL_ADJ_DOWN);
        }
        else if (mode == MODE_CLOCK_ADJ) {
          clock_adj(CL_ADJ_DOWN);
        }
//        digitalWrite(VFD_BLANKING, LOW);
        Serial.println("S4");
        break;
      default:
//        Serial.println("SOther");
        break;
    }
    //    tm.shorton_keydataw = 0;    // 処理完了　キー入力情報クリア
#ifdef KEY_TEST
    key_now = shortonw;
#endif
  }
  else if (longonw != 0) {
    switch (longonw) {
      case 0x01:
        if (mode == MODE_CLOCK) {
          // 時計設定モードへ
          modeset(MODE_CLOCK_ADJ);
        }
        else if (mode == MODE_CLOCK_ADJ) {
          // 時計表示モードへ　設定反映せず
          modeset(MODE_CLOCK);
        }
        else if (mode == MODE_CAL) {
          // カレンダー設定モードへ
//          modeset(MODE_CAL_ADJ);
        }
        else if (mode == MODE_CAL_ADJ) {
          // カレンダー表示モードへ　設定反映せず
          modeset(MODE_CAL);
        }
//        Serial.println("L1");
        break;
      case 0x02:
        if (mode == MODE_CLOCK) {
          // カレンダー表示モードへ
          modeset(MODE_CAL);
        }
        else if (mode == MODE_CAL) {
          // 時計表示モードへ
          modeset(MODE_CLOCK);
        }
//        Serial.println("L2");
        break;
      case 0x03:
        if (mode == MODE_CLOCK) {
          // 輝度調整処理へ
          modeset(MODE_BRIGHTNESS_ADJ);
        }
        else if (mode == MODE_BRIGHTNESS_ADJ) {
          // 輝度調整処理終了
          modeset(MODE_BRIGHTNESS_SAVE);
        }
//        Serial.println("L3");
        break;
      default:
//        Serial.println("LOther");
        break;
    }
    //    tm.longon_keydataw = 0;    // 処理完了　キー入力情報クリア
#ifdef KEY_TEST
    key_now = longonw * 10;
#endif
  }
  else {}

  return;
}


void clock_adj_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp) // 時刻調整時表示データ作成
{
  disp_tmp[0] = DISP_NON;
  disp_tmp[1] = date_time[0] % 10;
  disp_tmp[2] = date_time[0] / 10;
  disp_tmp[3] = adj_data[ADJ_MIN - ADJ_HOUR] % 10;
  disp_tmp[4] = adj_data[ADJ_MIN - ADJ_HOUR] / 10;
  disp_tmp[5] = adj_data[ADJ_HOUR - ADJ_HOUR] % 10;
  disp_tmp[6] = adj_data[ADJ_HOUR - ADJ_HOUR] / 10;
  disp_tmp[7] = DISP_NON;
  disp_tmp[8] = DISP_NON;


  // 調整桁点滅処理
  if (count >= (second_counterw / 2)) {
    // 消灯処理
    if(adj_point == ADJ_HOUR){
      disp_tmp[5] = disp_tmp[6] = DISP_NON;
    }
    else if (adj_point == ADJ_MIN) {
      disp_tmp[3] = disp_tmp[4] = DISP_NON;
    }
  }

  piriod_tmp[1] = piriod_tmp[3] = piriod_tmp[5] = 0x01;
  piriod_tmp[0] = piriod_tmp[2] = piriod_tmp[4] = piriod_tmp[6] = piriod_tmp[7] = piriod_tmp[8] = 0x00;

  return;
}

void clock_adj(unsigned char keyw)  // 時刻合わせ
{
  unsigned char tmpp;

  if (keyw == CL_ADJ_DIGUP) {  // 時刻調整桁変更
    if (adj_point == ADJ_HOUR) {
      adj_point = ADJ_MIN;    // 分調整
    }
    else if (adj_point == ADJ_MIN) {
      // 終了処理
      if (adj_runf == ON) {  // 調整実行された
        //時刻とRTCに反映
        tim.Hour = adj_data[ADJ_HOUR - ADJ_HOUR];
        tim.Minute = adj_data[ADJ_MIN - ADJ_HOUR];
        tim.Second = 0;
        RTC.write(tim);
      }
      modeset(MODE_CLOCK);    // 時計表示モードにする
    }
    else {
      // 状態遷移異常
      modeset(MODE_ERR_STATETRANSITION);    // 時計表示モードにする
    }
  }
  else if (keyw == CL_ADJ_UP) {             // Data ADD
    adj_runf = ON;          // 調整実行フラグセット
    if ((adj_point == ADJ_HOUR) || (adj_point == ADJ_MIN)) {
      tmpp = adj_point - ADJ_HOUR;
      if (adj_data[tmpp] >= adj_datamax[tmpp]) {
        adj_data[tmpp] = adj_datamini[tmpp];   // 上限チェック
      }
      else {
        adj_data[tmpp]++;
      }
    }
    else {
      modeset(MODE_ERR_STATETRANSITION);    // 状態遷移異常
    }
  }
  else if (keyw == CL_ADJ_DOWN) {          // Data DEL
    adj_runf = ON;          // 調整実行フラグセット
    if ((adj_point == ADJ_HOUR) || (adj_point == ADJ_MIN)) {
      tmpp = adj_point - ADJ_HOUR;
      if (adj_data[tmpp] <= adj_datamini[tmpp]) {
        adj_data[tmpp] = adj_datamax[tmpp];   // 下限チェック
      }
      else {
        adj_data[tmpp]--;
      }
    }
    else {
      modeset(MODE_ERR_STATETRANSITION);    // 状態遷移異常
    }
  }

  return;
}

void calender_display(unsigned char *disp_tmp, unsigned char *piriod_tmp)
{
  disp_tmp[0] = date_time[3] % 10;
  disp_tmp[1] = date_time[3] / 10;
  disp_tmp[2] = DISP_K1;
  disp_tmp[3] = date_time[5] % 10;
  disp_tmp[4] = date_time[5] / 10;
  disp_tmp[5] = DISP_K1;
  disp_tmp[6] = date_time[6] % 10;
  disp_tmp[7] = date_time[6] / 10;

  if (count >= (second_counterw / 2)) {
    // ピリオド消灯処理
    disp_tmp[8] = DISP_NON;
  }
  else {
    // ピリオド点灯処理
    disp_tmp[8] = DISP_K0;
  }

  piriod_tmp[1] = piriod_tmp[3] = piriod_tmp[5] = 0x00;
  piriod_tmp[0] = piriod_tmp[2] = piriod_tmp[4] = piriod_tmp[6] = piriod_tmp[7] = piriod_tmp[8] = 0x00;

  return;
}
void calender_adj_dispdat_make(unsigned char *disp, unsigned char *piriod) // 時刻調整時表示データ作成
{
  // 調整用表示データ作成
  disp[0] = adj_data[ADJ_DAY - ADJ_YEAR] % 10;
  disp[1] = adj_data[ADJ_DAY - ADJ_YEAR] / 10;
  disp[2] = 13;
  disp[3] = adj_data[ADJ_MONTH - ADJ_YEAR] % 10;
  disp[4] = adj_data[ADJ_MONTH - ADJ_YEAR] / 10;
  disp[5] = 13;
  disp[6] = adj_data[ADJ_YEAR - ADJ_YEAR] % 10;
  disp[7] = adj_data[ADJ_YEAR - ADJ_YEAR] / 10;  //現在年;

  // 調整桁点滅処理
  
  
  return;
}
void calender_adj(unsigned char keyw)  // カレンダー合わせ
{
  unsigned char tmpp;
  unsigned char daymaxw;

  if (keyw == CAL_ADJ_DIGUP) {  // カレンダー調整桁変更
    if (adj_point == ADJ_YEAR) {
      adj_point = ADJ_MONTH;    // 月調整
    }
    else if (adj_point == ADJ_MONTH) {
      adj_point = ADJ_DAY;      // 日調整
    }
    else if (adj_point == ADJ_DAY) {
      // 終了処理
      if (adj_runf == ON) {  // 調整実行された
        //RTCに反映
      }
      modeset(MODE_CAL);    // カレンダー表示モードにする
    }
    else {
      // 状態遷移異常
      modeset(MODE_ERR_STATETRANSITION);    // 時計表示モードにする
    }
  }
  else if (keyw == CAL_ADJ_UP) {
    adj_runf = ON;          // 調整実行フラグセット
    if ((adj_point == ADJ_YEAR) || (adj_point == ADJ_MONTH)) {
      tmpp = adj_point - ADJ_HOUR;
      if (adj_data[tmpp] >= adj_datamax[tmpp]) {
        adj_data[tmpp] = adj_datamini[tmpp];   // 上限チェック
      }
      else {
        adj_data[tmpp]++;
      }
    }
    else if (adj_point == ADJ_DAY) {
      tmpp = adj_point - ADJ_HOUR;
      daymaxw = getmonthdays(adj_data[ADJ_YEAR], adj_data[ADJ_MONTH]);
      if (adj_data[tmpp] >= daymaxw) {
        adj_data[tmpp] = adj_datamini[tmpp];   // 上限チェック
      }
      else {
        adj_data[tmpp]++;
      }
    }
    else {
      modeset(MODE_ERR_STATETRANSITION);    // 状態遷移異常
    }
  }
  else if (keyw == CAL_ADJ_DOWN) {
    adj_runf = ON;          // 調整実行フラグセット
    if ((adj_point == ADJ_YEAR) || (adj_point == ADJ_MONTH) || (adj_point == ADJ_DAY)) {
      tmpp = adj_point - ADJ_HOUR;
      if (adj_data[tmpp] <= adj_datamini[tmpp]) {
        adj_data[tmpp] = adj_datamax[tmpp];   // 下限チェック
      }
      else {
        adj_data[tmpp]--;
      }
    }
    else {
      modeset(MODE_ERR_STATETRANSITION);    // 状態遷移異常
    }
  }

  return;
}

unsigned char getmonthdays(unsigned char nyear, unsigned char nmonth)  // 月の最終日を返す
{
  unsigned char nmonthdays;
  if (nmonth == 2) {
    if (nyear % 4 == 0  &&  nyear % 100 != 0  ||  nyear % 400 == 0) {
      nmonthdays = 29;
    }
    else {
      nmonthdays = 28;
    }
  }
  else if (nmonth == 4 || nmonth == 6 || nmonth == 9 || nmonth == 11) {
    nmonthdays = 30;
  }
  else {
    nmonthdays = 31;
  }

  return (nmonthdays);
}


/* -- DCDCコンバータ制御 --*/
int dcdc_ini(void)
{
  int val = 0;               // 電圧フィードバック入力
  int vfdbw;                 // 電圧フィードバック測定平均計算
  int i;
  int errw = ERR_NON;        // エラーコード

  Timer1.initialize(TIMER1_INTTIME);      // 割り込み周期設定
  Timer1.pwm(DCDC_PWM, 0);       // PWM出力設定
  //  delay(10);

  vfdbw = 0;
  for (i = 0; i < 10; i++) {
    val = analogRead(DCDC_FDBA);      // 電圧フィードバック入力
    vfdbw += val;
//    Serial.print("Read data:");
//    Serial.println(val);
  }
//  Serial.print("Read data_val:");
//  Serial.println(vfdbw / 10);
  if (((vfdbw / 10) < 132) || ((vfdbw / 10) > 161)) {
    errw = ERR_DCDCFDB;
  }

  if (errw == ERR_NON) {
    Timer1.pwm(DCDC_PWM, 10);       // PWM出力設定
    vfdbw = 0;
    for (i = 0; i < 10; i++) {
      val = analogRead(DCDC_FDBA);      // 電圧フィードバック入力
      vfdbw += val;
//      Serial.print("Read data:");
//      Serial.println(val);
    }
//    Serial.print("Read data_val:");
//    Serial.println(vfdbw / 10);
    if ((vfdbw / 10) < 161) {
      errw = ERR_DCDCDRV;
    }

  }
  return (errw);
}
void dcdc_ctr(int setpwmw)
{
  int val = 0;               // 電圧フィードバック入力
  int pwmw = 0;              // PWM出力設定値
  int set_pwmpw = 0;         // P項
  int pwmidw;                // I項計算用ΔV
  static long pwmil = 0;     // I項Σdv

  val = analogRead(DCDC_FDBA);      // 電圧フィードバック入力
  set_pwmpw = setpwmw / DCDC_PWM_PGAIN;              // P計算
  pwmidw = (setpwmw - val) * DCDC_PWM_IGAIN;         // dv計算
  if (pwmil + (long)pwmidw > (DCDC_PWM_IGAEN_MAX * 0x10000)) {
    pwmil = (DCDC_PWM_IGAEN_MAX * 0x10000);          // Σdv上限
  }
  else if (pwmil + (long)pwmidw < (-1 * DCDC_PWM_IGAEN_MAX * 0x10000)) {
    pwmil = (-1 * DCDC_PWM_IGAEN_MAX * 0x10000);     // Σdv下限
  }
  else {
    pwmil = pwmil + (long)pwmidw;                        // Σdv計算
  }
  pwmw = set_pwmpw + (pwmil >> 8);        // PWM = P+I

//  Serial.print("pwmw_ori:");
//  Serial.print(pwmw);

  if (pwmw > DCDC_PWM_MAX) {pwmw = DCDC_PWM_MAX;}   // PWM設定値上限
  else if (pwmw < 0) {pwmw = 0;}                    // PWM設定値下限

  noInterrupts();                   // 割り込み禁止
  Timer1.pwm(DCDC_PWM, pwmw);       // PWM出力設定
  interrupts();                     // 割り込み許可

 
/*  Serial.print("\tRead data:");
  Serial.print(val);

  Serial.print("\tPWMp:");
  Serial.print(set_pwmpw);

  Serial.print("\tPWMi:");
  Serial.print(pwmil>>8);

  Serial.print("\tpwmidw:");
  Serial.print(pwmidw);

  Serial.print("\tpwmil:");
  Serial.print(pwmil);

  Serial.print("\tpwmw:");
  Serial.print(pwmw);
  Serial.println("\t");
*/
  return;
}
/* -- end --*/

/* -- VFD表示 --*/
void disp_ini(void)
{

#ifdef DISP_SHIFTOUT
  pinMode(SCK, OUTPUT) ;   // 制御するピンは全て出力に設定する
  pinMode(RCK, OUTPUT) ;
  pinMode(SI,  OUTPUT) ;
#else
  SPI.begin() ;                         // ＳＰＩを行う為の初期化
  SPI.setBitOrder(MSBFIRST) ;           // ビットオーダー
  SPI.setClockDivider(SPI_CLOCK_DIV4) ; // クロックをシステムクロックの1/4で使用(16MHz/4)
  SPI.setDataMode(SPI_MODE0) ;          // クロック極性０(LOW)　クロック位相０(LOW)
#endif
  digitalWrite(SS, LOW) ;               // SS(CS)ラインをLOWにする

  pinMode(VFD_BLANKING, OUTPUT);
  digitalWrite(VFD_BLANKING, LOW);

  return;
}
void disp_vfd_iv21(void)
{
  static unsigned char ketaw;
  static unsigned char pwm_countw;
  unsigned char ketapwm_tmpw;
  unsigned char dispketaw;
  unsigned long dispdata;
 

  // 点灯する桁更新
  if (ketaw >= (DISP_KETAMAX - 1)) {
    ketaw = 0;
    pwm_countw++;
  }
  else {
    ketaw++;
  }
  dispketaw = keta[ketaw] - 1;

  // 各桁表示データ作成
  if (disp[dispketaw] <= FONT_MAX) {
    dispdata = font[disp[dispketaw]] | keta_dat[dispketaw];
    if(disp_p[dispketaw] != 0x00){
      dispdata |= DISP_H;
    }
  }

  // 各桁PWM処理
  if(disp_ketapwm[dispketaw] <= DISP_PWM_MAX){
    ketapwm_tmpw = disp_ketapwm[dispketaw];
  }
  else{
    ketapwm_tmpw = DISP_PWM_MAX;
  }
  if((pwm_countw & 0x0F)  >= disp_ketapwm[dispketaw] ){   // PWM処理
    dispdata = DISP_NON;
  }
  


  //  if(dispketaw != 8){ digitalWrite(DISP_KIGO, LOW); }

  //  dispdata = 0x3C00;
  //  dispdata = 0x60001;
  /*
    Serial.print(dispketaw);
    Serial.print(":");
    Serial.print(disp[dispketaw]);
    Serial.print(":8:");
    Serial.print(disp[8]);
    Serial.print(":");
    Serial.println(dispdata);
  */

#ifdef DISP_SHIFTOUT
  digitalWrite(RCK, LOW) ;
  shiftOut(SI, SCK, MSBFIRST, dispdata) ;       // 5=0b00000101(0x05) ８ビット出力する
  shiftOut(SI, SCK, MSBFIRST, (dispdata >> 8)) ; // 5=0b00000101(0x05) ８ビット出力する
  shiftOut(SI, SCK, MSBFIRST, (dispdata >> 16)) ; // 5=0b00000101(0x05) ８ビット出力する
  digitalWrite(RCK, HIGH) ;                     // ラッチ信号を出す
#else
  digitalWrite(SS, LOW) ;
  SPI.transfer(dispdata) ;            // 5=0b00000101(0x05) ８ビット出力する
  SPI.transfer((dispdata >> 8)) ;     // 5=0b00000101(0x05) ８ビット出力する
  SPI.transfer((dispdata >> 16)) ;    // 5=0b00000101(0x05) ８ビット出力する
  digitalWrite(SS, HIGH) ;            // ラッチ信号を出す
#endif

  //  if(dispketaw == 8){ digitalWrite(DISP_KIGO, HIGH); }

  return;
}
/* -- end --*/

/************************************/

void led_ini() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  led_previous_millis = millis();
  led_interval = LED_INTERVAL_SET;
  led_statew = OFF;         // LED状態
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  return;
}

void led_man(unsigned char mode) {
  unsigned long current_millis;
  static unsigned char led_countw;    // 点滅回数カウンタ
  unsigned char led_hedw;
  unsigned char led_comw;
  unsigned char led_tmp;

  if (mode == MODE_BRIGHTNESS_ADJ) {   // 輝度調整時LED表示
    led_hedw = 3;
    led_comw = 2;
  }
  else if (mode == MODE_FILAMENT_SETUP) {    // VFDフィラメント電圧調整
    led_hedw = 3;
    led_comw = 1;
  }
  else if (mode == MODE_CLOCK_ADJ) {    //
    led_hedw = 3;
    led_comw = 3;
  }
  else if (mode == MODE_CAL_ADJ) {    //
    led_hedw = 3;
    led_comw = 4;
  }
  else if (mode == MODE_ERR_CPU_VOLTAGE) {    // CPU電圧エラー
    led_hedw = 2;
    led_comw = 1;
  }
  else if (mode == MODE_ERR_DCDC_STOP_VOLTAGE) {    // DCDC停止時電圧エラー
    led_hedw = 2;
    led_comw = 2;
  }
  else if (mode == MODE_ERR_DCDC_STARTUP_VOLTAGE) {    // DCDC始動時電圧エラー
    led_hedw = 2;
    led_comw = 3;
  }
  else {          // LED 常時消灯
    led_hedw = 0;
    led_comw = 0;
  }

  if (led_hedw != 0) {
    current_millis = millis();
    
    if (current_millis - led_previous_millis >= led_interval) {
      led_previous_millis = current_millis;
      led_countw ++;

      switch (led_blink_sqf) {
        case LED_BLINKSQ_HED:
          if (led_countw >= (led_hedw * 2)) {
            led_blink_sqf = LED_BLINKSQ_SP1;
            led_countw = 0;
            led_blink_sqf = LED_BLINKSQ_SP1;
          }
          break;
        case LED_BLINKSQ_SP1:
          if (led_countw >= 2) {
            led_countw = 0;
            led_blink_sqf = LED_BLINKSQ_COM;
          }
          break;
        case LED_BLINKSQ_COM:
          if (led_countw >= (led_comw * 2)) {
            led_countw = 0;
            led_blink_sqf = LED_BLINKSQ_SP2;
          }
          break;
        case LED_BLINKSQ_SP2:
          if (led_countw >= 6) {
            led_countw = 0;
            led_blink_sqf = LED_BLINKSQ_HED;
          }
          break;
        default:
          led_countw = 0;
          led_blink_sqf = LED_BLINKSQ_HED;
          break;
      }

  noInterrupts();
      if ( ((led_blink_sqf == LED_BLINKSQ_HED) || (led_blink_sqf == LED_BLINKSQ_COM))
           && (led_statew == OFF)) {
       led_statew = ON;
      }
      else {
        led_statew = OFF;
      }
  interrupts();
    }
  }
  else {    // LED 常時消灯
  noInterrupts();
    led_statew = OFF;
  interrupts();
  }

  return;
}

void disp_led(void){
  static char led_pwmcountw;

  led_pwmcountw++;
  if(led_pwmcountw > LED_BRIGHT_MAX){ led_pwmcountw = 0; }

  if(led_pwmcountw <= LED_BRIGHT){ digitalWrite(LED_PIN, !led_statew); }
  else{ digitalWrite(LED_PIN, HIGH); }

  return;
}

/* -- Timer1割込 -- */
void int_count(void) {
  //  static char count;
  static char last_second;

  count++;
  if (last_second != date_time[0]) {
    count = 0;
    last_second = date_time[0];
    //    sirial_out();                        // RTCテスト用シリアル出力
  }

  disp_vfd_iv21();
  disp_led();

  return;
}

/* -- end --*/





