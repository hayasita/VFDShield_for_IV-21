//#include <MsTimer2.h>
#include <stdint.h>
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
//#define DCDC_OUTV 600           // 24.9v
#define DCDC_OUTV 650             // xx.xv
//#define DCDC_OUTV 550           // 22.8v
//#define DCDC_OUTV 650           // 27v
#define DCDC_PWM_PGAIN  4       // Pゲイン
#define DCDC_PWM_IGAIN  2       // Iゲイン
#define DCDC_PWM_IGAEN_MAX  200 // Iゲイン最大値
#define DCDC_PWM_MAX  250       // PWM設定値上限
unsigned char dcdc_runningf;    // DCDCコンバータ動作許可

/* Err Code */
#define ERR_NON        0      // エラーなし
#define ERR_DCDCFDB    1      // DCDC フィードバック電圧範囲外
#define ERR_DCDCDRV    2      // DCDC PWM出力異常

/* DISPLAY */
unsigned long keta_dat[] = {
  0x000004,    // 1
  0x000008,    // 2
  0x010000,    // 3
  0x008000,    // 4
  0x000040,    // 5
  0x000100,    // 6
  0x004000,    // 7
  0x002000,    // 8
  0x000800     // 9
};
#define ELE_A  0x000001
#define ELE_B  0x000002
#define ELE_C  0x000010
#define ELE_D  0x000080
#define ELE_E  0x000200
#define ELE_F  0x000400
#define ELE_G  0x001000
#define ELE_H  0x000020

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
#define DISP_A    14
#define DISP_B    15
#define DISP_C    16
#define DISP_D    17
#define DISP_E    18
#define DISP_F    19
#define DISP_G    20
#define DISP_H    21
#define DISP_I    22
#define DISP_J    23
#define DISP_K    24
#define DISP_L    25
#define DISP_M    26
#define DISP_N    27
#define DISP_O    28
#define DISP_P    29
#define DISP_Q    30
#define DISP_R    31
#define DISP_S    32
#define DISP_T    33
#define DISP_U    34
#define DISP_V    35
#define DISP_W    36
#define DISP_X    37
#define DISP_Y    38
#define DISP_Z    39

unsigned long font[] = {
  ELE_A | ELE_B | ELE_C | ELE_D | ELE_E | ELE_F,            // 0
  ELE_B | ELE_C,                                            // 1
  ELE_A | ELE_B | ELE_G | ELE_E | ELE_D,                    // 2
  ELE_A | ELE_B | ELE_C | ELE_D | ELE_G,                    // 3
  ELE_B | ELE_C | ELE_F | ELE_G,                            // 4
  ELE_A | ELE_F | ELE_G | ELE_C | ELE_D,                    // 5
  ELE_A | ELE_C | ELE_D | ELE_E | ELE_F | ELE_G,            // 6
  ELE_A | ELE_B | ELE_C | ELE_F,                            // 7
  ELE_A | ELE_B | ELE_C | ELE_D | ELE_E | ELE_F | ELE_G,    // 8
  ELE_A | ELE_B | ELE_C | ELE_D | ELE_F | ELE_G,            // 9
  ELE_F,                                                    // ●
  ELE_G,                                                    // －
  ELE_F | ELE_G,                                            // ●－
  0x000000,                                                 // NULL
  ELE_A | ELE_B | ELE_C | ELE_E | ELE_F | ELE_G,            // A
  ELE_C | ELE_D | ELE_E | ELE_F | ELE_G,                    // B
  ELE_D | ELE_E | ELE_G,                                    // C
  ELE_B | ELE_C | ELE_D | ELE_E | ELE_G,                    // D
  ELE_A | ELE_D | ELE_E | ELE_F | ELE_G,                    // E
  ELE_A | ELE_E | ELE_F | ELE_G,                            // F
  ELE_A | ELE_C | ELE_D | ELE_E | ELE_F,                    // G
  ELE_C | ELE_E | ELE_F | ELE_G,                            // H
  ELE_E | ELE_F,                                            // I
  ELE_B | ELE_C | ELE_D | ELE_E,                            // J
  ELE_D | ELE_E | ELE_F | ELE_G,                            // K
  ELE_D | ELE_E | ELE_F,                                    // L
  ELE_A | ELE_B | ELE_C | ELE_E | ELE_F,                    // M
  ELE_C | ELE_E | ELE_G,                                    // N
  ELE_C | ELE_D | ELE_E | ELE_G,                            // O
  ELE_A | ELE_B | ELE_E | ELE_F | ELE_G,                    // P
  ELE_A | ELE_B | ELE_C | ELE_F | ELE_G,                    // Q
  ELE_E | ELE_G,                                            // R
  ELE_C | ELE_D | ELE_F | ELE_G,                            // S
  ELE_A | ELE_E | ELE_F,                                    // T
  ELE_C | ELE_D | ELE_E,                                    // U
  ELE_B | ELE_C | ELE_D | ELE_E | ELE_F,                    // V
  ELE_A | ELE_D | ELE_G,                                    // W
  ELE_B | ELE_C | ELE_E | ELE_F | ELE_G,                    // X
  ELE_B | ELE_C | ELE_D | ELE_F | ELE_G,                    // Y
  ELE_A | ELE_B | ELE_D | ELE_E,                            // Z
};
#define FONT_MAX (sizeof(font) / sizeof(unsigned long))

#define DISP_KETAMAX 9                        // VFD表示桁数
unsigned long disp[DISP_KETAMAX];             // 数値表示データ
unsigned char disp_p[DISP_KETAMAX];           // 各桁ピリオドデータ
unsigned long disp_last[DISP_KETAMAX];        // 前回数値表示データ
unsigned char disp_ketapwm[DISP_KETAMAX] = {  // 各桁輝度初期値
  15, 15, 15, 15, 15, 15, 15, 15, 15          // 0～15で設定
};
#define DISP_PWM_MAX  15                      // 最大輝度0x0f
unsigned char brightness_dig[DISP_KETAMAX];   // 表示各桁輝度
#define BR_MAX        15 // 最大輝度
#define BR_MIN        0  // 最小輝度
#define BR_ADJ_DIGUP  0  // 輝度調整桁変更
#define BR_ADJ_BRUP   1  // 輝度UP
#define BR_ADJ_BRDOWN 2  // 輝度DOWN

unsigned char disp_fadecount[DISP_KETAMAX];   // クロスフェードサイクルカウンタ
uint16_t disp_fadetimei;                      // クロスフェード時間（割込）(msec)
#define FADETIME_DEF  2;                      // クロスフェード時間初期値
#define ADJ_UP        1  // UPキー入力
#define ADJ_DOWN      2  // DOWNキー入力

struct CONFIG_DATA {    // 動作設定値
  uint8_t format_hw;    // 時間表示フォーマット 12/24H
  uint8_t fadetimew;    // クロスフェード時間(1~9)
};
struct CONFIG_DATA config_data;     // 設定データ
struct CONFIG_DATA config_tmp;      // 設定データtmp

//unsigned char i,j;
unsigned char mode_m;    // 動作モード
#define MODE_M_DISP           0     // 通常表示モード
#define MODE_M_SET            1     // 設定モード

unsigned char mode;      // 設定モード
#define MODE_CLOCK                      0     // 時計表示
#define MODE_CAL                        1     // カレンダー表示

#define MODE_CLOCK_ADJ                  10    // 時計調整
#define MODE_CLOCK_ADJ_SET              11    // 時計調整実行
#define MODE_CAL_ADJ                    12    // カレンダー調整
#define MODE_CAL_ADJ_SET                13    // カレンダー調整実行
#define MODE_CLOCK_1224SEL              14    // 12h24h表示切替
#define MODE_CLOCK_1224SEL_SET          15    // 12h24h表示切替実行
#define MODE_FADETIME_ADJ               16    // クロスフェード時間設定
#define MODE_FADETIME_ADJ_SET           17    // クロスフェード時間設定実行
#define MODE_DISPTYPE_SEL               18    // 
#define MODE_DISPTYPE_SEL_SET           19    // 
#define MODE_BRIGHTNESS_ADJ             20    // VFD輝度調整
#define MODE_BRIGHTNESS_ADJ_SET         21    // VFD輝度調整実行
#define MODE_BRIGHTNESS_SAVE            22    // VFD輝度記録
#define MODE_FILAMENT_SETUP             23    // VFDフィラメント電圧調整　全消灯

#define MODE_ERR_                       100   // エラー番号閾値　100以上はエラーモード定義として使用する。
#define MODE_ERR_CPU_VOLTAGE            101   // CPU電圧エラー
#define MODE_ERR_DCDC_STOP_VOLTAGE      102   // DCDC停止時電圧エラー
#define MODE_ERR_DCDC_STARTUP_VOLTAGE   101   // DCDC始動時電圧エラー
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
//void clockdisplay(unsigned char *disp_tmp, unsigned char *piriod_tmp);   // 時刻表示データ作成

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

// 起動時電圧測定
#define CPU_VCC_MAX  204    // 5.521V
#define CPU_VCC_NOM  225    // 5.006V
#define CPU_VCC_MIN  251    // 4.487V

void setup() {
  int startmode_sw;

  Serial.begin(9600);
  while (!Serial) ; // wait for Arduino Serial Monitor

  mode = MODE_CLOCK;
  key_now = 0;

  startmode_sw = digitalRead(sw_list[0]);  // 起動時SW1読込み

  // Startup Error Check.
  cpu_voltage_chk();

  // DCDC Setup
  dcdc_ini();        // DCDC制御初期化 DCDCエラーチェック

  // LED Setup
  led_ini();

  // VFD Setup
  disp_ini();        // VFD表示初期化

  // Timer Setup
  second_counterw = int(1000000 / TIMER1_INTTIME);  // 1秒間の割り込み回数
  count = 0;
//  Timer1.initialize(TIMER1_INTTIME);      // 割り込み周期設定  dcdc_ini()で設定する。
  Timer1.attachInterrupt(int_count);
  //  MsTimer2::set(2, int_count); // 500ms period
  //  MsTimer2::start();

  // RTC Setup
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  // mode Setup
  if (mode < MODE_ERR_) { // エラー発生なし
    if (startmode_sw == 0) { // 電源ON時SW1入力あり
      modeset_m(MODE_M_DISP);           // 通常表示モードへ
      modeset(MODE_FILAMENT_SETUP);    // VFDフィラメント電圧調整
      Serial.println("Mode : Filament Setup.");
      // DCDC OFF
      dcdc_runningf = OFF;
      Serial.println("DCDC Converter : Stop.");
    }
    else {
      modeset_m(MODE_M_DISP);           // 通常表示モードへ
      modeset(MODE_CLOCK);
      dcdc_runningf = ON;
      Serial.println("DCDC Converter : Start OK.");
    }
  }
  else {   // エラー発生
      Serial.println("Starting ERROR.");
    // DCDC OFF
      dcdc_runningf = OFF;
      Serial.println("DCDC Converter : Stop.");
  }

//  
  eerom_read();     // 設定値EEROM読み出し
  Serial.print("config_data.format_hw : ");
  Serial.println(config_data.format_hw);
  Serial.print("config_data.fadetimew : ");
  Serial.println(config_data.fadetimew);

}

void loop() {

  dcdc_ctr(DCDC_OUTV);        // DCDC制御

  rtc_chk();                  // RTC読み出し
  disp_datamake();            // 表示データ作成
  tm.scan();                  // キー入力読込
  keyman();                   // キー入力処理

  led_man(mode);              // LED表示データ管理

}

void rtc_chk(void) {
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  date_time[0] = second();
  if (mode == MODE_CLOCK) {
    date_time[1] = minute();
    date_time[2] = hour();
  }
  else if (mode == MODE_CAL) {
    date_time[3] = day();
    //    date_time[4] = Rtc.weekdays();
    date_time[5] = month();
    date_time[6] = year() % 100;
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

void modeset_m(unsigned char setmode)
{
  if (setmode == MODE_M_DISP) {                   // 表示モード
    mode_m = MODE_M_DISP;                           // 表示モードへ
    Serial.println("Mode_M : Display Mode.");
  }
  else if (setmode == MODE_M_SET) {               // 設定モード
    mode_m = MODE_M_SET;                            // 設定モードへ
    Serial.println("Mode_M : Set Mode.");
    config_tmp.fadetimew = config_data.fadetimew;   // 設定用tmp初期化
  }
  else{                                           // 仕様外の場合は、表示モード・時計表示とする
    mode_m = MODE_M_DISP;
    Serial.println("Mode_M : Display Mode.(Mode Error)");
  }
  
  return;
}

void modeset(unsigned char setmode)
{
  unsigned char lastmode;      // 前回設定モード

  if (setmode == MODE_CLOCK) {
    mode = MODE_CLOCK;
    Serial.println("Mode : Clock.");
  }
  else if (setmode == MODE_CLOCK_ADJ) {
    mode = MODE_CLOCK_ADJ;
  }
  else if (setmode == MODE_CLOCK_ADJ_SET) {
    adj_point = ADJ_HOUR;    // 時間調整から開始する
    adj_runf = OFF;          // 調整実行フラグ初期化
    adj_data[ADJ_HOUR - ADJ_HOUR] = date_time[2];
    adj_data[ADJ_MIN - ADJ_HOUR] = date_time[1] = minute();
    mode = MODE_CLOCK_ADJ_SET;
    Serial.println("Mode : Clock ADJ.");
  }
  else if (setmode == MODE_CAL) {
    mode = MODE_CAL;
    Serial.println("Mode : Calender.");
  }
  else if (setmode == MODE_CAL_ADJ) {
    mode = MODE_CAL_ADJ;
  }
  else if (setmode == MODE_CAL_ADJ_SET) {
    adj_point = ADJ_YEAR;    // 年調整から開始する
    adj_runf = OFF;          // 調整実行フラグ初期化
    adj_data[ADJ_YEAR - ADJ_YEAR] = year()-2000;
    adj_data[ADJ_MONTH - ADJ_YEAR] = date_time[5];
    adj_data[ADJ_DAY - ADJ_YEAR] = date_time[3];
    mode = MODE_CAL_ADJ_SET;
    Serial.println("Mode : Calender ADJ.");
  }
  else if (setmode == MODE_BRIGHTNESS_ADJ) {      // VFD輝度調整
    adj_point = ADJ_BR1;     // 1桁目から開始する
    adj_runf = OFF;          // 調整実行フラグ初期化
    mode = MODE_BRIGHTNESS_ADJ;
    Serial.println("Mode : Brightness ADJ.");
  }
  else if (setmode == MODE_BRIGHTNESS_SAVE) {
    brightness_eeprom_save();    // 輝度をEEPROMに保存
    modeset_m(MODE_M_DISP);       // 通常表示モードへ
    modeset(MODE_CLOCK);         // 動作モードを時計表示にする。
  }
  else if (setmode == MODE_FILAMENT_SETUP) {
    mode = MODE_FILAMENT_SETUP;
  }
  else if (setmode == MODE_CLOCK_1224SEL){
    mode = MODE_CLOCK_1224SEL;
  }
  else if (setmode == MODE_CLOCK_1224SEL_SET){
    mode = MODE_CLOCK_1224SEL_SET;
  }
  else if (setmode == MODE_FADETIME_ADJ){         // クロスフェード時間設定
    mode = MODE_FADETIME_ADJ;
    config_data.fadetimew = config_tmp.fadetimew;   // クロスフェード時間設定更新
    eerom_write();                                  // 設定値EEROM書き込み
  }
  else if (setmode == MODE_FADETIME_ADJ_SET){     // クロスフェード時間設定実行
    mode = MODE_FADETIME_ADJ_SET;
    config_tmp.fadetimew = config_data.fadetimew;   // クロスフェード時間設定用tmp作成
  }
  else{
  //  mode = setmode;
  }

  if(mode != lastmode){             // モード変更あり
    lastmode = mode;                // 前回モード = 今回モード
    display_scrolldat_make_ini();   // スクロール表示データ初期化
    display_blinking_make_ini();    // 表示データ点滅初期化
  }

  return;
}

void disp_datamake(void) {
  unsigned int i;
  unsigned char disp_tmp[DISP_KETAMAX];       // 各桁表示データ
  unsigned char piriod_tmp[DISP_KETAMAX];     // 各桁ピリオド
  unsigned long dispdata_tmp[DISP_KETAMAX];   // 各桁表示データ(font情報)
  unsigned long dispdata;                     // 表示データ作成用tmp
  uint16_t fadetime_tmpw;                     // クロスフェード時間受け渡し用データ

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
  if (mode == MODE_CLOCK) {                             // 時刻表示
    clock_display(disp_tmp, piriod_tmp);                  // 時刻情報作成
    symbol_display(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_CAL) {                          // カレンダー表示
    calender_display(disp_tmp, piriod_tmp);
  }

  else if (mode == MODE_CLOCK_ADJ) {                    // 時刻設定
    clock_adjtitle_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_CLOCK_ADJ_SET) {                // 時刻設定実行
    clock_adj_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_CAL_ADJ) {                      // カレンダー設定
    calender_adjtitle_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_CAL_ADJ_SET) {                  // カレンダー設定実行
    calender_adj_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_CLOCK_1224SEL){                 // 12h24h表示切替
    clock1224set_adjtitle_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_CLOCK_1224SEL_SET){             // 12h24h表示切替実行
    clock1224set_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_FADETIME_ADJ){                  // クロスフェード時間設定
    crossfade_adjtitle_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_FADETIME_ADJ_SET){              // クロスフェード時間設定実行
    crossfade_adj_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_BRIGHTNESS_ADJ) {               // VFD輝度調整
    brightness_adjtitle_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_BRIGHTNESS_ADJ_SET) {           // VFD輝度調整実行
    brightness_adj_dispdat_make(disp_tmp, piriod_tmp);
  }
  else if (mode == MODE_FILAMENT_SETUP) {
    disp_alloff(disp_tmp, piriod_tmp);
  }

#endif

  // 表示データ作成
  for (i = 0; i < 9; i++) {
    if (disp_tmp[i] <= FONT_MAX) {
      dispdata = font[disp_tmp[i]] | keta_dat[i];
      if (piriod_tmp[i] != 0x00) {
        dispdata |= ELE_H;
      }
    }
    dispdata_tmp[i] = dispdata;
  }

  // クロスフェード時間作成
  if(mode == MODE_FADETIME_ADJ_SET){              // クロスフェード時間設定実行
    fadetime_tmpw = config_tmp.fadetimew * 100;     // 設定用tmpから作成する
  }
  else{
    fadetime_tmpw = config_data.fadetimew * 100;
  }

  noInterrupts();      // 割り込み禁止
  for (i = 0; i < 9; i++) {
    disp[i] = dispdata_tmp[i];
  }
  disp_fadetimei = fadetime_tmpw;               // クロスフェード時間受け渡し
  interrupts();        // 割り込み許可

  return;
}

void symbol_display(unsigned char *disp_tmp, unsigned char *piriod_tmp)
{
  if (count >= (second_counterw / 2)) {
    disp_tmp[8] = DISP_NON;     // ○消灯処理
  }
  else {
    disp_tmp[8] = DISP_K0;      // ○点灯処理
  }

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
  if(config_data.format_hw == 1){
    disp_tmp[5] = date_time[2] % 10;
    disp_tmp[6] = date_time[2] / 10;
  }
  else{
    uint8_t tmpw;
    tmpw = date_time[2];
    if(date_time[2] > 12){
      tmpw = tmpw - 12;
    }
    disp_tmp[5] = tmpw % 10;
    disp_tmp[6] = tmpw / 10;
  }
  disp_tmp[7] = DISP_K3;

  //  piriod_tmp[0] = piriod_tmp[2] = piriod_tmp[4] = 0x01;
  //  piriod_tmp[1] = piriod_tmp[3] = piriod_tmp[5] = piriod_tmp[6] = piriod_tmp[7] = piriod_tmp[8] = 0x00;
  piriod_tmp[1] = piriod_tmp[3] = piriod_tmp[5] = 0x01;
  piriod_tmp[0] = piriod_tmp[2] = piriod_tmp[4] = piriod_tmp[6] = piriod_tmp[7] = piriod_tmp[8] = 0x00;
#endif

  return;
}
void disp_alloff(unsigned char *disp_tmp, unsigned char *piriod_tmp) {
  for (unsigned char i = 0; i < 9; i++) {
    disp_tmp[i] = DISP_NON;
    piriod_tmp[i] = 0x00;
  }

  return;
}

// 12h24h表示切替
void clock1224set_adjtitle_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp) {
  char disptxt[] = "12H24H SEL";
  display_scrolldat_make(disp_tmp,piriod_tmp,disptxt,5,5);
  disp_tmp[6] = DISP_NON;
  disp_tmp[7] = DISP_03;
  disp_tmp[8] = DISP_K1;
  piriod_tmp[7] = 0x01;

  return;
}

// 12h24h表示切替実行
void clock1224set_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp) {
  disp_tmp[0] = DISP_H;
  if(config_data.format_hw == 1){
    disp_tmp[1] = DISP_04;
    disp_tmp[2] = DISP_02;
  }
  else{
    disp_tmp[1] = DISP_02;
    disp_tmp[2] = DISP_01;
  }

  disp_tmp[3] = DISP_NON;
  disp_tmp[4] = DISP_NON;
  disp_tmp[5] = DISP_NON;
  disp_tmp[6] = DISP_NON;
  disp_tmp[7] = DISP_03;
  disp_tmp[8] = DISP_K1;
  piriod_tmp[7] = 0x01;

  return;
}

// クロスフェード時間設定
void crossfade_adjtitle_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp) {
  char disptxt[] = "CROSS FADE TIME SET";
  display_scrolldat_make(disp_tmp,piriod_tmp,disptxt,5,5);
  disp_tmp[6] = DISP_NON;
  disp_tmp[7] = DISP_04;
  disp_tmp[8] = DISP_K1;
  piriod_tmp[7] = 0x01;

  return;
}

// クロスフェード時間設定実行
void crossfade_adj_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp) {
  for (unsigned char i = 0; i < 9; i++) {
    disp_tmp[i] = DISP_NON;
    piriod_tmp[i] = 0;
  }

  disp_tmp[0] = DISP_00 + config_tmp.fadetimew;
  disp_tmp[8] = DISP_K1;
  display_blinking_make(disp_tmp,piriod_tmp,0,1);

  return;
}

// VFD輝度調整
void brightness_adjtitle_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp) {
  char disptxt[] = "BRIGHTNES SET";
  display_scrolldat_make(disp_tmp,piriod_tmp,disptxt,5,5);
  disp_tmp[6] = DISP_NON;
  disp_tmp[7] = DISP_05;
  disp_tmp[8] = DISP_K1;
  piriod_tmp[7] = 0x01;

  return;
}

// VFD輝度調整実行
void brightness_adj_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp) {
  for (unsigned char i = 0; i < 9; i++) {
    disp_tmp[i] = DISP_08;
    piriod_tmp[i] = 0x00;
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

/* 表示データ点滅 */
long blinking_tim_nowl;
uint8_t blinking_state;
void display_blinking_make_ini(){

  blinking_state = 0;
  blinking_tim_nowl = millis();
  return;
}
void display_blinking_make(uint8_t *disp_tmp, uint8_t *piriod_tmp,uint8_t startp,uint8_t dispnum)
{
  if(startp > 7){
    startp = 7;
  }
  if(dispnum > startp){
    dispnum = startp;
  }

  if( ( millis() - blinking_tim_nowl ) > 1000){

    if(blinking_state == 0){
      blinking_state = 1;

    }
    else{
      blinking_state = 0;
    }

    blinking_tim_nowl = millis();
  }
  if(blinking_state == 1){
        for(uint8_t i=0;i<1;i++){
        disp_tmp[startp + i] = DISP_NON;
      }
  }

  return;
}

/* 表示スクロールデータ作成 */
long scroll_tim_nowl;
unsigned char disp_point;
void display_scrolldat_make_ini(){
  disp_point = 0;
  scroll_tim_nowl = millis();
  return;
}

void display_scrolldat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp,unsigned char *disp_data,unsigned char startp,unsigned char dispnum) {
  unsigned char tmp;
  unsigned char disp_tmpp;
  long scrool_wait;

  if(startp > 7){
    startp = 7;
  }
  if(dispnum > startp){
    dispnum = startp;
  }

  if(disp_point == 0){
    scrool_wait = 0;
  }
  else if(disp_point == 1){
    scrool_wait = 2000;
  }
  else{
    scrool_wait = 700;
  }

  if( ( millis() - scroll_tim_nowl ) > scrool_wait){
    for(unsigned char i=0 ; i<=dispnum ; i++){
      disp_tmpp = startp-i;

      if((disp_point + i) < strlen(disp_data)){
        if( (disp_data[disp_point+i] >= '0') && (disp_data[disp_point+i] <= '9')){
          tmp = disp_data[disp_point+i] - '0' + DISP_00;
        }
        else if( (disp_data[disp_point+i] >= 'A') && (disp_data[disp_point+i] <= 'Z')){
          tmp = disp_data[disp_point+i] - 'A' + DISP_A;
        }
        else if(disp_data[disp_point+i] == ' '){
          tmp = disp_data[disp_point+i] - ' ' + DISP_NON;
        }
        else{
          tmp = DISP_NON;
        }
        disp_tmp[disp_tmpp] = tmp;
      }
      else{
        disp_tmp[disp_tmpp] = DISP_NON;
      }
      piriod_tmp[disp_tmpp] = 0x00;
    }
    disp_point++;
    if(disp_point > strlen(disp_data) ){
      disp_point = 0;
    }
    scroll_tim_nowl = millis();
  }

  return;
}

void format_h_make(void)        // 12/24H表示設定
{
  Serial.println(config_data.format_hw);
  if(config_data.format_hw == 1){
    config_data.format_hw = 0;
  }
  else{
    config_data.format_hw = 1;
  }
  Serial.println("format_h_make");
  Serial.println(config_data.format_hw);

  eerom_write();                // 設定値EEROM書き込み

  return;
}

void fadetime_adj(uint8_t keyw)   // クロスフェード時間設定
{
  Serial.println(config_tmp.fadetimew);

  if (keyw == ADJ_UP){
    if(config_tmp.fadetimew < 9){
      config_tmp.fadetimew ++;
    }
  }
  else if (keyw == ADJ_DOWN){
    if(config_tmp.fadetimew > 0){
      config_tmp.fadetimew --;
    }
  }
  Serial.println("fadetimew_make");
  Serial.println(config_tmp.fadetimew);

  return;
}

void brightness_adj(unsigned char keyw)  // 時刻合わせ
{
  if (keyw == BR_ADJ_DIGUP) {  // 輝度調整桁変更
    if (adj_point == ADJ_BR9) {
      adj_point = ADJ_BR1;
    }
    else if ((adj_point >=  ADJ_BR1) && (adj_point <=  ADJ_BR8)) {
      adj_point++;
    }
    else {
      // 状態遷移エラー発生
    }
  }
  else if (keyw == BR_ADJ_BRUP) { // 輝度UP
    if (brightness_dig[adj_point - ADJ_BR1] < BR_MAX) {
      brightness_dig[adj_point - ADJ_BR1]++;
    }
    else {
      brightness_dig[adj_point - ADJ_BR1] == BR_MAX;
    }
  }
  else if (keyw == BR_ADJ_BRDOWN) { // 輝度DOWN
    if (brightness_dig[adj_point - ADJ_BR1] > BR_MIN) {
      brightness_dig[adj_point - ADJ_BR1]--;
    }
    else {
      brightness_dig[adj_point - ADJ_BR1] == BR_MIN;
    }
  }

  return;
}
void brightness_ini(void)
{
  unsigned char i;
  unsigned char tmpw;
  
  tmpw = 0x00;
  
  if(tmpw == 0x5a){
    // 輝度情報EEPROM読み出し
    brightness_eeprom_load();
  }
  else{
    // EEPROM 初期化
    
    // 輝度情報初期化
    for (i = 0; i < 9; i++) {
      brightness_dig[i] = disp_ketapwm[i];
    }
    brightness_eeprom_save();
  }


  return;
}
void brightness_eeprom_load(void)    // 輝度をEEPROMから読み出し
{

  return;
}
void brightness_eeprom_save(void)    // 輝度をEEPROMに保存
{
  return;
}

// 設定値EEROM読み出し
void eerom_read(void)
{
  uint8_t err = OFF;

  EEPROM.get( 0x00, config_data );

  if(config_data.format_hw > 1){
    err = ON;
  }
  if((config_data.fadetimew > 9) || (config_data.fadetimew == 0)){
    err = ON;
  }

  if(err == ON){
    eerom_ini();
    eerom_write();
  }

  return;
}

// 設定値初期化
void eerom_ini(void)
{
  config_data.format_hw = 1;              // 24h 
  config_data.fadetimew = FADETIME_DEF;   // クロスフェード時間初期値
  return;
}

// 設定値EEROM書き込み
void eerom_write(void)
{
  EEPROM.put(0x00,config_data);

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

      case 0b01:                            // SW1 Short ON
        Serial.println("SW1 Short On");
        if (mode_m == MODE_M_SET) {           // 設定モード
          if (mode == MODE_CLOCK_ADJ) {                 // 時計設定モード
            modeset(MODE_CLOCK_ADJ_SET);                  // 時計設定実行モードへ
            Serial.println(" MODE_CLOCK_ADJ_SET chg.");
          }
          else if (mode == MODE_CLOCK_ADJ_SET) {        // 時計設定実行モード
            clock_adj(CL_ADJ_DIGUP);                      // 時計操作桁更新
            Serial.println(" CLOCK ADJ DigUp.");
          }
          else if (mode == MODE_CAL_ADJ) {              // カレンダー設定モード
            modeset(MODE_CAL_ADJ_SET);                    // カレンダー設定実行モードへ
            Serial.println(" MODE_CAL_ADJ_SET chg.");
           }
          else if (mode == MODE_CAL_ADJ_SET) {          // カレンダー実行モード
            calender_adj(CAL_ADJ_DIGUP);                  // カレンダー操作桁更新
            Serial.println(" CAL ADJ DigUp.");
          }
          else if(mode == MODE_CLOCK_1224SEL){          // 時計表示 12h<>24h設定モード
            modeset(MODE_CLOCK_1224SEL_SET);              // 時計表示 12h<>24h設定実行モードへ
          }
          else if(mode == MODE_CLOCK_1224SEL_SET){      // 時計表示 12h<>24h設定実行モード
            modeset(MODE_CLOCK_1224SEL);                  // 時計表示 12h<>24h設定モードへ
          }
          else if(mode == MODE_FADETIME_ADJ){           // クロスフェード時間設定
            modeset(MODE_FADETIME_ADJ_SET);               // クロスフェード時間設定実行へ
          }
          else if(mode == MODE_FADETIME_ADJ_SET){       // クロスフェード時間設定実行
            modeset(MODE_FADETIME_ADJ);                   // クロスフェード時間設定へ
          }
          else if (mode == MODE_BRIGHTNESS_ADJ) {       // VFD輝度調整モード
            modeset(MODE_BRIGHTNESS_ADJ_SET);             // VFD輝度調整実行モードへ
            Serial.println(" MODE_BRIGHTNESS_ADJ_SET chg.");
          }
          else if (mode == MODE_BRIGHTNESS_ADJ_SET) {   // VFD輝度調整実行モード
            brightness_adj(BR_ADJ_DIGUP);                 // VFD輝度調整桁更新
            Serial.println(" BRIGHTNESS ADJ DigUp.");
          }

          Serial.print(" mode : ");
          Serial.println(mode);
          Serial.println(" SetMode Target chg.");
        }
        break;

      case 0b10:                            // SW2 Short ON
        Serial.println("SW2 Short On");
        if (mode_m == MODE_M_SET) {           // 設定モード
          if (mode == MODE_CLOCK_ADJ) {         // 時計設定モード
            modeset(MODE_CAL_ADJ);                // カレンダー設定モードへ
            Serial.println(" MODE_CAL_ADJ.");
          }
          else if(mode == MODE_CAL_ADJ){        // カレンダー設定モード
            modeset(MODE_CLOCK_1224SEL);          // 時計表示 12h<>24h設定モードへ
            Serial.println(" MODE_CLOCK_1224SEL.");
          }
          else if(mode == MODE_CLOCK_1224SEL){  // 時計表示 12h<>24h設定モード
            modeset(MODE_FADETIME_ADJ);           // クロスフェード時間設定モードへ
            Serial.println(" MODE_FADETIME_ADJ.");
          }
          else if(mode == MODE_FADETIME_ADJ){   // クロスフェード時間設定モード
            modeset(MODE_BRIGHTNESS_ADJ);         // VFD輝度調整モードへ
            Serial.println(" MODE_BRIGHTNESS_ADJ.");
          }
          else if(mode == MODE_BRIGHTNESS_ADJ){ // VFD輝度調整モード
            modeset(MODE_CLOCK_ADJ);              // 時計設定モードへ
            Serial.println(" MODE_CLOCK_ADJ.");
          }
                                              // 設定値変更
          else if(mode == MODE_CLOCK_ADJ_SET){  // 時計設定実行モード
            clock_adj(CL_ADJ_UP);                 // 時計 Plus
            Serial.println(" CL_ADJ_UP.");
          }
          else if(mode == MODE_CAL_ADJ_SET){    // カレンダー設定実行モード
            calender_adj(CAL_ADJ_UP);             // カレンダー Plus
            Serial.println(" CAL_ADJ_UP.");
          }
          else if(mode == MODE_CLOCK_1224SEL_SET){  // 時計表示 12h<>24h設定実行モード
            format_h_make();
            Serial.println(" FormatH_ADJ_UP.");
          }
          else if(mode == MODE_FADETIME_ADJ_SET){   // クロスフェード時間設定実行
            fadetime_adj(ADJ_UP);
            Serial.println(" ADJ_UP.");
          }
          else if(mode == MODE_BRIGHTNESS_ADJ_SET){ // VFD輝度調整実行モード
            brightness_adj(BR_ADJ_BRUP);              // 輝度 Plus
            Serial.println(" BR_ADJ_BRUP.");
          }
           
          Serial.print(" mode : ");
          Serial.println(mode);
          Serial.println("SetMode UP.");
        }
        else if (mode_m == MODE_M_DISP) {     // 表示モード
          if (mode == MODE_CLOCK){              // 時計表示
            modeset(MODE_CAL);                    // カレンダー表示モードへ
          }
          else if(mode == MODE_CAL){            // カレンダー表示
            modeset(MODE_CLOCK);                  // 時計表示モードへ
          }
          else{                                 // その他
            modeset(MODE_CAL);                    // カレンダー表示モードへ
          }
        }


        //        digitalWrite(VFD_BLANKING, HIGH);
        break;
      case 0b100:                            // SW3 Short ON
        Serial.println("SW3 Short On");
        if (mode_m == MODE_M_SET) {           // 設定モード
          if (mode == MODE_CLOCK_ADJ) {         // 時計設定モード
            modeset(MODE_BRIGHTNESS_ADJ);         // VFD輝度調整モードへ
            Serial.println(" MODE_BRIGHTNESS_ADJ.");
          }
          else if(mode == MODE_CAL_ADJ){        // カレンダー設定モード
            modeset(MODE_CLOCK_ADJ);              // 時計設定モードへ
            Serial.println(" MODE_CLOCK_ADJ.");
          }
          else if(mode == MODE_CLOCK_1224SEL){  // 時計表示 12h<>24h設定モード
            modeset(MODE_CAL_ADJ);                // カレンダー設定モードへ
            Serial.println(" MODE_CAL_ADJ.");
          }
          else if(mode == MODE_FADETIME_ADJ){   // クロスフェード時間設定モード
            modeset(MODE_CLOCK_1224SEL);          // 時計表示 12h<>24h設定モードへ
            Serial.println(" MODE_CLOCK_1224SEL.");
          }
          else if(mode == MODE_BRIGHTNESS_ADJ){ // VFD輝度調整モード
            modeset(MODE_FADETIME_ADJ);           // クロスフェード時間設定モードへ
            Serial.println(" MODE_FADETIME_ADJ.");
          }

                                              // 設定値変更
          else if(mode == MODE_CLOCK_ADJ_SET){  // 時計設定実行モード
            clock_adj(CL_ADJ_DOWN);                 // 時計 Minus
            Serial.println(" CL_ADJ_DOWN.");
          }
          else if(mode == MODE_CAL_ADJ_SET){    // カレンダー設定実行モード
            calender_adj(CAL_ADJ_DOWN);             // カレンダー Minus
            Serial.println(" CAL_ADJ_DOWN.");
          }
          else if(mode == MODE_CLOCK_1224SEL_SET){  // 時計表示 12h<>24h設定実行モード
            format_h_make();
            Serial.println(" FormatH_ADJ_DOWN.");
          }
          else if(mode == MODE_FADETIME_ADJ_SET){   // クロスフェード時間設定実行
            fadetime_adj(ADJ_DOWN);
            Serial.println(" ADJ_DOWN.");
          }
          else if(mode == MODE_BRIGHTNESS_ADJ_SET){ // VFD輝度調整実行モード
            brightness_adj(BR_ADJ_BRDOWN);              // 輝度 Minus
            Serial.println(" BR_ADJ_BRDOWN.");
          }
           
          Serial.print(" mode : ");
          Serial.println(mode);
          Serial.println("SetMode DOWNDOWN.");
        }
        else if (mode_m == MODE_M_DISP) {     // 表示モード
          if (mode == MODE_CLOCK){              // 時計表示
            modeset(MODE_CAL);                    // カレンダー表示モードへ
          }
          else if(mode == MODE_CAL){            // カレンダー表示
            modeset(MODE_CLOCK);                  // 時計表示モードへ
          }
          else{                                 // その他
            modeset(MODE_CAL);                    // カレンダー表示モードへ
          }
        }



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
      case 0b01:                            // SW1 Long ON
        Serial.println("SW1 Long On");
        if (mode_m == MODE_M_DISP) {          // 通常表示モード
          Serial.println("*** To Set. ***");
          modeset_m(MODE_M_SET);                // 設定モードへ
          modeset(MODE_CLOCK_ADJ);              // 時計設定モードへ
        }
        else if (mode_m == MODE_M_SET) {      // 設定モード
          Serial.println("*** To Disp ***");
          modeset_m(MODE_M_DISP);               // 通常表示モードへ
          modeset(MODE_CLOCK);                  // 時計表示モードへ
        }
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
/*
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
          brightness_adj(BR_ADJ_DIGUP);
        }
        else if (mode == MODE_CAL_ADJ) {
          calender_adj(CAL_ADJ_DIGUP);
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
          calender_adj(CAL_ADJ_UP);
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
          calender_adj(CAL_ADJ_DOWN);
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
          modeset(MODE_CAL_ADJ);
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
*/
void clock_adjtitle_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp){

  char disptxt[] = "CLOCK SET";
  display_scrolldat_make(disp_tmp,piriod_tmp,disptxt,5,5);
  disp_tmp[6] = DISP_NON;
  disp_tmp[7] = DISP_01;
  disp_tmp[8] = DISP_K1;
  piriod_tmp[7] = 0x01;

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
    if (adj_point == ADJ_HOUR) {
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
        tim.Day = day();
        tim.Month = month();
        tim.Year = year();
        RTC.write(tim);
      }
      modeset_m(MODE_M_DISP); // 通常表示モードへ
      modeset(MODE_CLOCK);    // 時計表示モードにする
    }
    else {
      // 状態遷移異常
      modeset_m(MODE_M_DISP); // 通常表示モードへ
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
      modeset_m(MODE_M_DISP); // 通常表示モードへ
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
      modeset_m(MODE_M_DISP); // 通常表示モードへ
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

void calender_adjtitle_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp)
{
  char disptxt[] = "CALENDAR SET";
  display_scrolldat_make(disp_tmp,piriod_tmp,disptxt,5,5);
  disp_tmp[6] = DISP_NON;
  disp_tmp[7] = DISP_02;
  disp_tmp[8] = DISP_K1;
  piriod_tmp[7] = 0x01;

  return;
}

void calender_adj_dispdat_make(unsigned char *disp_tmp, unsigned char *piriod_tmp) // 時刻調整時表示データ作成
{
  // 調整用表示データ作成
  disp_tmp[0] = adj_data[ADJ_DAY - ADJ_YEAR] % 10;
  disp_tmp[1] = adj_data[ADJ_DAY - ADJ_YEAR] / 10;
  disp_tmp[2] = DISP_K1;
  disp_tmp[3] = adj_data[ADJ_MONTH - ADJ_YEAR] % 10;
  disp_tmp[4] = adj_data[ADJ_MONTH - ADJ_YEAR] / 10;
  disp_tmp[5] = DISP_K1;
  disp_tmp[6] = adj_data[ADJ_YEAR - ADJ_YEAR] % 10;
  disp_tmp[7] = adj_data[ADJ_YEAR - ADJ_YEAR] / 10;  //現在年;
  disp_tmp[8] = DISP_NON;

  // 調整桁点滅処理
  if (count >= (second_counterw / 2)) {
    // 消灯処理
    if (adj_point == ADJ_YEAR) {
      disp_tmp[6] = disp_tmp[7] = DISP_NON;
    }
    else if (adj_point == ADJ_MONTH) {
      disp_tmp[3] = disp_tmp[4] = DISP_NON;
    }
    else if (adj_point == ADJ_DAY) {
      disp_tmp[0] = disp_tmp[1] = DISP_NON;
    }
  }

  piriod_tmp[1] = piriod_tmp[3] = piriod_tmp[5] = 0x00;
  piriod_tmp[0] = piriod_tmp[2] = piriod_tmp[4] = piriod_tmp[6] = piriod_tmp[7] = piriod_tmp[8] = 0x00;

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
        tim.Second = second();   
        tim.Minute = minute();
        tim.Hour = hour();
        tim.Day = adj_data[ADJ_DAY - ADJ_YEAR];
        tim.Month = adj_data[ADJ_MONTH - ADJ_YEAR];
        tim.Year = CalendarYrToTm(2000 + adj_data[ADJ_YEAR - ADJ_YEAR]);
        RTC.write(tim);
      }
      modeset_m(MODE_M_DISP); // 通常表示モードへ
      modeset(MODE_CAL);    // カレンダー表示モードにする
    }
    else {
      // 状態遷移異常
      modeset_m(MODE_M_DISP); // 通常表示モードへ
      modeset(MODE_ERR_STATETRANSITION);    // 時計表示モードにする
    }
  }
  else if (keyw == CAL_ADJ_UP) {
    adj_runf = ON;          // 調整実行フラグセット
    if ((adj_point == ADJ_YEAR) || (adj_point == ADJ_MONTH)) {
      tmpp = adj_point - ADJ_YEAR;
      if (adj_data[tmpp] >= adj_datamax[tmpp+2]) {   // 上限チェック
        adj_data[tmpp] = adj_datamini[tmpp+2];
      }
      else {
        adj_data[tmpp]++;
      }
    }
    else if (adj_point == ADJ_DAY) {
      tmpp = adj_point - ADJ_YEAR;
      daymaxw = getmonthdays(adj_data[ADJ_YEAR-ADJ_YEAR], adj_data[ADJ_MONTH-ADJ_YEAR]);
      if (adj_data[tmpp] >= daymaxw) {   // 上限チェック
        adj_data[tmpp] = adj_datamini[tmpp+2];
      }
      else {
        adj_data[tmpp]++;
      }
    }
    else {
      modeset_m(MODE_M_DISP); // 通常表示モードへ
      modeset(MODE_ERR_STATETRANSITION);    // 状態遷移異常
    }
  }
  else if (keyw == CAL_ADJ_DOWN) {
    adj_runf = ON;          // 調整実行フラグセット
    if ((adj_point == ADJ_YEAR) || (adj_point == ADJ_MONTH)) {
       tmpp = adj_point - ADJ_YEAR;
      if (adj_data[tmpp] <= adj_datamini[tmpp+2]) {   // 下限チェック
        adj_data[tmpp] = adj_datamax[tmpp+2];
      }
      else {
        adj_data[tmpp]--;
      }
    }
    else if (adj_point == ADJ_DAY) {
       tmpp = adj_point - ADJ_YEAR;
      if (adj_data[tmpp] <= adj_datamini[tmpp+2]) {   // 下限チェック
        adj_data[tmpp] = getmonthdays(adj_data[ADJ_YEAR-ADJ_YEAR], adj_data[ADJ_MONTH-ADJ_YEAR]);
      }
      else {
        adj_data[tmpp]--;
      }
    }
    else {
      modeset_m(MODE_M_DISP); // 通常表示モードへ
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
  Timer1.pwm(DCDC_PWM, 0);       // PWM出力設定
  return (errw);
}
void dcdc_ctr(int setpwmw)
{
  int val = 0;               // 電圧フィードバック入力
  int pwmw = 0;              // PWM出力設定値
  int set_pwmpw = 0;         // P項
  int pwmidw;                // I項計算用ΔV
  static long pwmil = 0;     // I項Σdv

  if (dcdc_runningf != OFF) {

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

    if (pwmw > DCDC_PWM_MAX) {
      pwmw = DCDC_PWM_MAX; // PWM設定値上限
    }
    else if (pwmw < 0) {
      pwmw = 0; // PWM設定値下限
    }
  }
  else {
    pwmw = 0;
  }

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
  unsigned char i;

#ifdef DISP_SHIFTOUT
  pinMode(SCK, OUTPUT) ;   // 制御するピンは全て出力に設定する
  pinMode(RCK, OUTPUT) ;
  pinMode(SI,  OUTPUT) ;
  shiftOut(SI, SCK, MSBFIRST, 0) ;      // 初期化
  shiftOut(SI, SCK, MSBFIRST, 0) ;      // 初期化
  shiftOut(SI, SCK, MSBFIRST, 0) ;      // 初期化
  digitalWrite(RCK, HIGH) ;             // ラッチ信号を出す
#else
  SPI.begin() ;                         // ＳＰＩを行う為の初期化
  SPI.setBitOrder(MSBFIRST) ;           // ビットオーダー
  SPI.setClockDivider(SPI_CLOCK_DIV4) ; // クロックをシステムクロックの1/4で使用(16MHz/4)
  SPI.setDataMode(SPI_MODE0) ;          // クロック極性０(LOW)　クロック位相０(LOW)
  digitalWrite(SS, LOW) ;
  SPI.transfer(0) ;                     // 初期化
  SPI.transfer(0) ;                     // 初期化
  SPI.transfer(0) ;                     // 初期化
  digitalWrite(SS, HIGH) ;              // ラッチ信号を出す
#endif
  digitalWrite(SS, LOW) ;               // SS(CS)ラインをLOWにする

  pinMode(VFD_BLANKING, OUTPUT);
  digitalWrite(VFD_BLANKING, LOW);

  brightness_ini();                    // 輝度情報初期化

  for(i = 0; i < 9; i++){
    disp[i] = 0;            // 数値表示データ初期化
  }

  return;
}
void disp_vfd_iv21(void)
{
  static unsigned char pwm_countw;                // PWMカウンタ
  unsigned char brightness_tmpw;                  // 輝度情報オーバーフロー対策
  static unsigned char dispketaw;                 // 表示桁
  unsigned long dispdata = 0;                     // 表示データ
  static unsigned long lasttime[DISP_KETAMAX];    // クロスフェードカウンタ周期前回値
  static unsigned long fade_cyclel[DISP_KETAMAX]; // クロスフェードサイクルカウンタ更新間隔
  unsigned int disp_fade_modew;                   // クロスフェード有無

  disp_fade_modew = 1;                            // 1:フェードあり 0:フェードなし

  // 点灯する桁更新
  if (dispketaw >= (DISP_KETAMAX - 1)) {
   dispketaw = 0;
    pwm_countw++;
  }
  else {
    dispketaw++;
  }

  // 各桁輝度確認
  if (brightness_dig[dispketaw] <= DISP_PWM_MAX) {                // 輝度が仕様範囲内
    brightness_tmpw = brightness_dig[dispketaw];
  }
  else {
    brightness_tmpw = DISP_PWM_MAX;                               // 輝度が仕様範囲外の場合、最大輝度設定
  }

  // クロスフェード初期化
  if((disp_last[dispketaw] != disp[dispketaw]) && (disp_fadecount[dispketaw] == 0)){  // 表示データ更新された && クロスフェードサイクルカウンタ未更新
    if(disp_fade_modew != 0){
      lasttime[dispketaw] = millis();                             // クロスフェードサイクルカウンタ更新間隔初期化
      fade_cyclel[dispketaw] = disp_fadetimei / brightness_tmpw;  // クロスフェードサイクルカウンタ更新間隔計算　200ms/輝度
      disp_fadecount[dispketaw]++;                                // クロスフェードサイクルカウンタ更新
    }else{
      disp_last[dispketaw] = disp[dispketaw];                     // 前回データに今回データをコピー
      disp_fadecount[dispketaw] = 0;                              // クロスフェードカウンタクリア
    }
  }
  if(disp_fadecount[dispketaw] != 0){
    if(millis() - lasttime[dispketaw] > fade_cyclel[dispketaw]){  // クロスフェードサイクルカウンタ更新時間チェック
      lasttime[dispketaw] = millis();                             // サイクルカウンタ更新前回値更新
      disp_fadecount[dispketaw]++;                                // クロスフェードサイクルカウンタ更新
    }
  }
  //disp[0] = font[disp_fadecount[2]%10] | keta_dat[0];           // debug

  // 各桁輝度決定PWM処理
  if ((pwm_countw & 0x0F)  >= disp_ketapwm[dispketaw] ) {   // PWM処理
    dispdata = font[DISP_NON];                              // PWM消灯
  }else{
    if(((pwm_countw & 0x0F) < disp_fadecount[dispketaw]) || (disp_fade_modew == 0)){    // PWM点灯 // クロスフェード今回：前回表示比率判定　暫定でpwm_countwを使用する
      dispdata = disp[dispketaw];                           // 今回データを優先表示
    }else{
      dispdata = disp_last[dispketaw];                      // 前回データ表示
    }
  }

  // クロスフェード終了判定
  if(disp_fadecount[dispketaw] == disp_ketapwm[dispketaw]){ // クロスフェードカウンタ最大値（輝度）到達
    disp_last[dispketaw] = disp[dispketaw];                 // 前回データに今回データをコピー
    disp_fadecount[dispketaw] = 0;                          // クロスフェードカウンタクリア
  }

  // データ転送
#ifdef DISP_SHIFTOUT
  digitalWrite(RCK, LOW) ;
  shiftOut(SI, SCK, MSBFIRST, (dispdata >> 16)) ; // 5=0b00000101(0x05) ８ビット出力する
  shiftOut(SI, SCK, MSBFIRST, (dispdata >> 8)) ;  // 5=0b00000101(0x05) ８ビット出力する
  shiftOut(SI, SCK, MSBFIRST, dispdata) ;         // 5=0b00000101(0x05) ８ビット出力する
  digitalWrite(RCK, HIGH) ;                       // ラッチ信号を出す
#else
  digitalWrite(SS, LOW) ;
  SPI.transfer((dispdata >> 16)) ;    // 5=0b00000101(0x05) ８ビット出力する
  SPI.transfer((dispdata >> 8)) ;     // 5=0b00000101(0x05) ８ビット出力する
  SPI.transfer(dispdata) ;            // 5=0b00000101(0x05) ８ビット出力する
  digitalWrite(SS, HIGH) ;            // ラッチ信号を出す
#endif

  return;
}
/* -- end --*/

/* -- LED制御処理 -- */
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

  if (mode == MODE_BRIGHTNESS_ADJ) {   // VFD輝度調整時LED表示
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

void disp_led(void) {
  static char led_pwmcountw;

  led_pwmcountw++;
  if (led_pwmcountw > LED_BRIGHT_MAX) {
    led_pwmcountw = 0;
  }

  if (led_pwmcountw <= LED_BRIGHT) {
    digitalWrite(LED_PIN, !led_statew);
  }
  else {
    digitalWrite(LED_PIN, HIGH);
  }

  return;
}

/* -- CPU電源電圧チェック -- */
void cpu_voltage_chk(void) {
  unsigned int vcc_tmp;
  float disp_vcc;

  vcc_tmp = cpu_vcc();
  if ((vcc_tmp < CPU_VCC_MIN) && (vcc_tmp > CPU_VCC_MAX)) { // 正常範囲外
    modeset_m(MODE_M_DISP);         // 通常表示モードへ
    modeset(MODE_ERR_CPU_VOLTAGE);
  }
  
  disp_vcc = (1.1 * 10240.0) / (float)(vcc_tmp * 10);
  Serial.print("CPU Voltage : ");
  Serial.print(disp_vcc);
  Serial.println("V");

  return;
}

unsigned int cpu_vcc(void) {
  long sum = 0;
  adcSetup(0x4E);                    // Vref=AVcc, input=internal1.1V
  for (int n = 0; n < 10; n++) {
    sum = sum + adc();               // adcの値を読んで積分
  }
  return ((unsigned int)(sum / 10));
}

void adcSetup(byte data) {           // ADコンバーターの設定
  ADMUX = data;                      // ADC Multiplexer Select Reg.
  ADCSRA |= ( 1 << ADEN);            // ADC イネーブル
  ADCSRA |= 0x07;                    // AD変換クロック　CK/128
  delay(10);                         // 安定するまで待つ
}

unsigned int adc() {                 // ADCの値を読む
  unsigned int dL, dH;
  ADCSRA |= ( 1 << ADSC);            // AD変換開始
  while (ADCSRA & ( 1 << ADSC) ) {   // 変換完了待ち
  }
  dL = ADCL;                         // LSB側読み出し
  dH = ADCH;                         // MSB側
  return dL | (dH << 8);             // 10ビットに合成した値を返す
}

/* -- Timer1割込 -- */
void int_count(void) {
  //  static char count;
  static char last_second;

  count++;
  if (last_second != date_time[0]) {
    count = 0;
    last_second = date_time[0];
  }

  disp_vfd_iv21();
  disp_led();

  return;
}

/* -- end --*/
