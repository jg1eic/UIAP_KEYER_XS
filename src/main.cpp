//==================================================================//
// UIAP_keyer_for_ch32fun
// BASE Software from : https://www.gejigeji.com/?page_id=1045
// Modified by Kimio Ohe JA9OIR/JA1AOQ
//	- port to ch32fun library
//==================================================================
#include "ch32fun.h"
#include "ch32v003_GPIO_branchless.h"
#include "keyer_hal.h"
#include <stdint.h>
#define printf(...) ((void)0)
#define SSD1306_128X64
#include "ssd1306_i2c.h"
#undef printf
#include "ssd1306.h"
#include "flash_eep.h"

/* ==== FLASH EEPROM ==== */
FLASH_EEP eep;  // EEPROM emulation object

//==========================================
// 常数・マクロ
//==========================================
// スクイズ状態（半ディット単位）の状態機械
#define SQZ_FREE 0
#define SQZ_SPC0 1
#define SQZ_SPC 2
#define SQZ_DOT0 3
#define SQZ_DOT 4
#define SQZ_DAH_CONT0 5
#define SQZ_DAH_CONT1 6
#define SQZ_DAH_CONT 7
#define SQZ_DASH 8

// パドル状態
#define PDL_DOT 1
#define PDL_DASH 2
#define PDL_FREE 0

#define SQUEEZE_TYPE 0 // スクイーズモード
#define PDL_RATIO 4    // 短点・長点比率

#define WPM_MAX 40 // 最大速度
#define WPM_MIN 5  // 最小速度

// トーン設定（tone_div_val グローバル変数で動的に変更）
// 2-->976Hz  3-->651Hz  4-->488Hz

// スイッチ設定
#define SW_SCAN_DIV 20  // 0.256ms ×20 ≒5.12ms
#define SW_PRESS_TH 195 // 長押し判定 195×5ms ≒1秒
#define SW_PUSH_TH 5    // 押下判定 5×5ms = 25ms
#define SW_1 (1 << 3) 
#define SW_2 (1 << 2)
#define SW_3 (1 << 1)
#define SW_4 (1 << 0)
#define SW_INFO_CLICK 0x10
#define SW_INFO_PRESS 0x20
#define SW_INFO_DOUBLE 0x40
#define SW_CLEAR() (sw_mask = 0b00001111) // 一度スイッチを離すまでカウントをしない
#define MASK_MODE 0xf0

// EDIT操作のリピート設定（体感調整済み）
#define EDIT_REPEAT_START 15 // 15 ×10ms = 150ms
#define EDIT_REPEAT_SPEED 5  // 5 ×10ms = 50ms

// ディットモード
#define DEBUG_MODE_PRINT 0

#ifndef PWR_CTLR_CWUF
#define PWR_CTLR_CWUF ((uint16_t)0x0004)
#define PWR_CTLR_CSBF ((uint16_t)0x0008)
#define PWR_CSR_SBF   ((uint16_t)0x0002)
#endif

// FLASH記録用
#define MSG_COUNT 4
#define PAGE_MSG1 0
#define PAGE_MSG2 1
#define PAGE_MSG3 2
#define PAGE_MSG4 3
#define PAGE_SETTINGS 4   // 設定保存ページ
#define SETTINGS_MAGIC 0xA5

// メッセージ用変数
#define MSG_NUM 4  // メモリ数（SW1 / SW2 / SW3 / SW4）
#define MSG_LEN 64 // 1メッセージの最大文字数
#define EDIT_TABLE_LEN (sizeof(edit_table) - 1)

// チャタリング除去用
#define MIN_ON_TICKS 50 //0.256ms ×50 = 12.8ms
#define MIN_OFF_TICKS 50 //0.256ms ×50 = 12.8ms

//デコード用閾値
#define DIT_TICKS      250   // 0.256ms ×250 = 64ms (20WPM相当
#define DOT_MAX        (2 * dit_est)
//#define CHAR_GAP_MIN   (1.5 * dit_est)
#define CHAR_GAP_MIN   (dit_est * 3u / 2u)   // 整数演算で同等
#define WORD_GAP_MIN   (5 * dit_est)

//CWバッファサイズ
#define CW_BUF_SIZE 64

//==========================================
// 構造体定義
//==========================================

// モード定義
typedef enum
{
    MODE_KEYER = 0,   // 通常キーイング
    MODE_PLAY,        // メモリ再生中
    MODE_EDIT_SELECT, // 編集メモリ選択
    MODE_EDIT,        // 編集モード
    MODE_SETUP        // 設定（未実装）
} keyer_mode_t;

// 文字編集状態
typedef enum
{
    EDIT_CHAR_SELECT = 0, // 文字選択中
    EDIT_POS_MOVE         // カーソル移動中（将来拡張）
} edit_state_t;

// 文字編集中にパドル検出用
typedef struct
{
    bool dot;
    bool dash;
} paddle_release_t;

volatile paddle_release_t pad_rel = {0};

// CWイベント定義
typedef enum {
    EV_DOT,
    EV_DASH,
    EV_CHAR_GAP,
    EV_WORD_GAP
} cw_event_t;

//==========================================
// グローバル変数
//==========================================

volatile uint8_t tone_div = 0;        // トーン分周カウンタ
volatile uint8_t tone_div_val = 3;    // トーン分周閾値（2=976Hz, 3=651Hz, 4=488Hz）
volatile bool tone_on = false;        // トーン出力中フラグ
volatile bool edit_tick_10ms = false; // EDIT用10msタイマーフラグ

char msgs[MSG_NUM][MSG_LEN + 1]; // メッセージバッファ

const char default_msgs[MSG_NUM][MSG_LEN] = {
    "CQ TEST JO1YGK",
    "5NN 13M BK",
    "TEST MESSAGE 3",
    "TEST MESSAGE 4"
    }; // デフォルトメッセージ
static uint8_t cur_msg = 0;  // 編集メモリ番号
static uint8_t edit_pos = 0; // カーソル位置

static const char edit_table[] =
    " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/?.=+-@"; // 編集文字テーブル

#define DISP_COLS 10 // 画面に表示する文字数（編集メッセージ）

static uint8_t edit_view_left = 0; // 表示ウィンドウ先頭
static uint8_t edit_len = 0;       // 現在の文字数
volatile bool edit_tick = false;

// キーイング用変数
int key_spd = 1000;
int key_spd_sys = 1000; // システムメッセージ用の速度（固定値）
int wpm = 20;
int wpm_sys = 20; // システムメッセージ用のWPM（固定値）
bool tone_enabled = false;
bool paddle_swap = false;            // パドル左右反転
bool sidetone_enabled = true;        // サイドトーン ON/OFF
static uint8_t setup_cursor = 0;     // 設定画面カーソル
static uint8_t setup_scroll = 0;     // 設定画面スクロール位置
static uint8_t tone_freq_idx = 1;    // TONE FREQ: 0=976Hz, 1=651Hz(デフォルト), 2=488Hz
static uint8_t repeat_interval_idx = 0; // REPEAT INT: 0=1s,1=3s,2=5s,3=10s,4=30s,5=60s
static uint8_t standby_time_idx = 0;    // STANDBY: 0=10s,1=30s,2=60s,3=なし
static uint16_t bat_mv = 0;          // 電池電圧 [mV]
static bool bat_low_warn = false;    // 低電圧警告フラグ
static bool bat_blink = false;       // LOW BAT 点滅状態
int squeeze = 0;
int paddle = PDL_FREE;
int paddle_old = PDL_FREE;

volatile uint8_t sw_div_cnt = 0;
volatile uint32_t tim1_tick256 = 0;

static uint32_t key_last_tick = 0;
static bool key_state = false; // false=OFF, true=ON

// 編集カウンタ
static keyer_mode_t mode = MODE_KEYER;

// EDIT用パドル状態
static uint16_t edit_dot_cnt = 0;
static uint16_t edit_dash_cnt = 0;
static bool edit_dot_prev = false;  // 前フレームのDOT状態
static bool edit_dash_prev = false; // 前フレームのDASH状態
static uint8_t edit_first = 1;      // EDIT初回フラグ

// ==== dit推定 ====
uint32_t dit_est = DIT_TICKS;        // 初期値（20WPM相当）

/* ==== 割り込み / main loop 共有 ==== */
volatile bool in_dot = false;
volatile bool in_dash = false;

// ==== 自動送信制御 ====
// 起動時は何もしない
volatile bool auto_mode = false;  // 今、自動送信中か
volatile bool auto_armed = false; // SWAを一度でも押したか。trueで “自動送信機能が有効”
volatile bool req_start_auto = false;
volatile bool req_reset_auto = false;

// 停止理由（ラッチ停止）
typedef enum
{
    STOP_NONE = 0,
    STOP_PADDLE = 1, // DOT/DASHで停止（ラッチ）
    STOP_SWB = 2,    // SWBで停止（ラッチ）
} StopReason;

volatile StopReason stop_reason = STOP_NONE;

// リピート再生
static bool repeat_mode = false;       // リピート再生中フラグ
static uint8_t repeat_msg_idx = 0;     // リピート対象メッセージ番号

// スイッチの判定
uint8_t sw_mask = 0; // スイッチ押しっぱなしをカウントしないためのマスク
uint8_t sw_clicked = 0;
uint8_t count_sw[4]; // スイッチ長押しとかカウンター
volatile uint8_t sw_stat;
volatile uint8_t sw_mode;

// ==== 汎用 CW メッセージ再生 ====
const char *auto_msg = NULL; // 実際に再生する文字列
bool sys_msg_active = false; // システムメッセージか
volatile bool keyout_enabled = true;       // 通常はON
volatile bool ignore_paddle_input = false; // パドル入力を無視するフラグ

// ==== キータイミング計測用 ====
uint32_t key_on_ticks = 0;
uint32_t key_off_ticks = 0;
bool key_on = false;

//CWリングバッファ
volatile uint8_t cw_buf[CW_BUF_SIZE];
volatile uint8_t cw_w = 0;
volatile uint8_t cw_r = 0;

//モールス文字バッファ
volatile bool flush_done = false;
volatile bool oled_dirty = false;  // OLEDリフレッシュ要求フラグ

//モールス符号バッファ
#define MORSE_BUF_LEN 8

char morse_buf[MORSE_BUF_LEN];
uint8_t morse_len = 0;

//volatile bool oled_need_refresh = false;
//static bool oled_refreshed_this_frame = false;

// グローバルに追加
int last_wpm = -999;

// 無操作タイマー
volatile uint32_t last_activity_tick = 0;

static inline uint32_t exti_line_from_pin(uint32_t pin)
{
    return 1u << (pin & 0x0f);
}

static void exti_select_port(uint8_t line, uint8_t port)
{
    const uint32_t shift = line * 2u;
    AFIO->EXTICR = (AFIO->EXTICR & ~(0x3u << shift)) | ((uint32_t)port << shift);
}

static void prepare_gpio_for_standby(void)
{
    // Put pins that do not need to wake the keyer into analog input to avoid leakage paths.
    GPIO_port_pinMode(GPIO_port_A, GPIO_pinMode_I_analog, GPIO_Speed_In);
    GPIO_port_pinMode(GPIO_port_C, GPIO_pinMode_I_analog, GPIO_Speed_In);
    GPIO_port_pinMode(GPIO_port_D, GPIO_pinMode_I_analog, GPIO_Speed_In);

    GPIO_pinMode(PIN_SW1, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_SW2, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_SW3, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_SW4, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_DOT, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_DASH, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_ST, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
}

static void restore_after_standby(void)
{
    NVIC_SystemReset();
}

__attribute__((section(".noinit")))
uint32_t standby_magic;

#define STANDBY_MAGIC_VALUE 0xA5A5A5A5
#define SSD1306_W 128
#define SSD1306_H 64


//==========================================
// 関数プロトタイプ
//==========================================
static const char *morseForChar(char c);
static void printAsc(int8_t asciinumber);
static void printAscii(int8_t c);
uint8_t job_paddle(void);
uint8_t job_auto(void);
void startTone(void);
void stopTone(void);
static void drawstr_6px(uint8_t x, uint8_t y, const char *str, uint8_t color);
void update_speed_from_adc(void);
uint16_t read_battery_mv(void);
void update_battery_status(void);
void start_play(uint8_t msg);
void stop_play(void);
void adjust_edit_view(void);
void update_switch_status(void);
uint8_t sw_chatter(uint8_t sw, uint8_t *counter);
void sw_check(void);
uint8_t sw_get_info(void);
uint8_t sw_is_pressed();
void handle_keyer_mode(void);
void handle_play_mode(void);
void handle_edit_select(void);
void handle_edit_mode(void);
void draw_keyer_screen(void);
void draw_edit_screen(void);
void draw_edit_select(void);
void draw_setup_screen(void);
void load_settings_from_flash(void);
void save_settings_to_flash(void);
void draw_startup_screen(void);
void loop(void);
void TIM1_UP_IRQHandler(void);
void init_flash_messages(void);
void save_current_message_to_flash(void);

//==========================================
// Morse テーブル
//==========================================
static const char *morseForChar(char c)
{
    // 小文字b は “BT” (E-...-) 扱い
    // if (c == '=')
    //     return "-...-";
    // if (c == '+')
    //     return ".-.-.";
    if (c == 'k')
        return "-.--.";
    if (c == 'v')
        return "...-.-";
    if (c >= 'a' && c <= 'z')
        c = (char)(c - 'a' + 'A');

    switch (c)
    {
    // Letters
    case 'A':
        return ".-";
    case 'B':
        return "-...";
    case 'C':
        return "-.-.";
    case 'D':
        return "-..";
    case 'E':
        return ".";
    case 'F':
        return "..-.";
    case 'G':
        return "--.";
    case 'H':
        return "....";
    case 'I':
        return "..";
    case 'J':
        return ".---";
    case 'K':
        return "-.-";
    case 'L':
        return ".-..";
    case 'M':
        return "--";
    case 'N':
        return "-.";
    case 'O':
        return "---";
    case 'P':
        return ".--.";
    case 'Q':
        return "--.-";
    case 'R':
        return ".-.";
    case 'S':
        return "...";
    case 'T':
        return "-";
    case 'U':
        return "..-";
    case 'V':
        return "...-";
    case 'W':
        return ".--";
    case 'X':
        return "-..-";
    case 'Y':
        return "-.--";
    case 'Z':
        return "--..";

    // Digits
    case '0':
        return "-----";
    case '1':
        return ".----";
    case '2':
        return "..---";
    case '3':
        return "...--";
    case '4':
        return "....-";
    case '5':
        return ".....";
    case '6':
        return "-....";
    case '7':
        return "--...";
    case '8':
        return "---..";
    case '9':
        return "----.";

    // Punctuation (必要なものだけ)
    case '.':
        return ".-.-.-";
    case ',':
        return "--..--";
    case '?':
        return "..--..";
    case '/':
        return "-..-.";
    case '=':
        return "-...-";
    case '+':
        return ".-.-.";
    case '-':
        return "-....-";
    case '@':
        return ".--.-.";

    default:
        return nullptr;
    }
}




#define FONT_WIDTH 12 //was 12
#define FONT_COLOR 1
#define LINE_HEIGHT 16
#define FONT_SCALE_16X16 fontsize_16x16
const int colums = 10; // was 10

int lcdindex = 0;
uint8_t line1[colums];
uint8_t line2[colums];
uint8_t line3[colums];
uint8_t lastChar = 0;

volatile uint8_t flg = 0;

static void reset_decoded_display(void)
{
    for (int i = 0; i < colums; i++) {
        line1[i] = ' ';
        line2[i] = ' ';
        line3[i] = ' ';
    }
    lcdindex = 0;
}

//==========================================
//	printasc : print the ascii code to the lcd
//==========================================
static void redraw_lines(void)
{
    // ---- 行 = 16 ---
    for (int i = 0; i < colums; i++) {
        ssd1306_drawchar_sz(i * FONT_WIDTH, LINE_HEIGHT * 1,
                            line1[i], FONT_COLOR, FONT_SCALE_16X16);
    }

    // ---- 行 = 32 ---
    for (int i = 0; i < colums; i++) {
        ssd1306_drawchar_sz(i * FONT_WIDTH, LINE_HEIGHT * 2,
                            line2[i], FONT_COLOR, FONT_SCALE_16X16);
    }

    // ---- 行 = 48 ---
    for (int i = 0; i < colums; i++) {
        //ssd1306_fillRect(0, 48, 128, 16, 0); // 行全体消す
        ssd1306_drawchar_sz(i * FONT_WIDTH, LINE_HEIGHT * 3,
                            line3[i], FONT_COLOR, FONT_SCALE_16X16);
    }

    // ---- 右端クリア（保険として残す）---
    int used_width = FONT_WIDTH * colums;   // 16 * 8 = 128
    if (used_width < 128) {
        int clear_w = 128 - used_width;
        ssd1306_fillRect(used_width, 16, clear_w, 16, 0);
        ssd1306_fillRect(used_width, 32, clear_w, 16, 0);
        ssd1306_fillRect(used_width, 48, clear_w, 16, 0);
    }
}

static void printAsc(int8_t asciinumber)
{
    // 行がいっぱい→スクロール
    if (lcdindex >= colums) {

        // 行 →行
        memcpy(line1, line2, colums);

        // 行 →行
        memcpy(line2, line3, colums);

        // 行 をクリア
        memset(line3, ' ', colums);

        lcdindex = 0;
    }

    // 行 に新しい文字を追加
    line3[lcdindex++] = asciinumber;

    // redraw_lines/refreshはTIM1ハンドラ内で行うと13ms以上かかりギャップが伸びるため
    // メインループで実行する（oled_dirtyフラグで通知）
    oled_dirty = true;
}


//==========================================
//	printascii : print the ascii code to the lcd
//==========================================
static void printAscii(int8_t c)
{
    switch (c)
    {
    case 'b': // BT
        printAsc('B');
        printAsc('T');
        break;
    case 'a': // AR
        printAsc('A');
        printAsc('R');
        break;
    case 'k': // KN
        printAsc('K');
        printAsc('N');
        break;
    case 'v': // VA
        printAsc('V');
        printAsc('A');
        break;
    default:
        printAsc(c);
        break;
    }
}

// トーン出力トグル
static inline void toggle_tone_pin(void)
{
    GPIOC->OUTDR ^= (1 << 7); // PC7 のトグル
}


//==========================================
// フラッシュメモリ読み出し・妥当性確認
//==========================================
static bool is_valid_message(const uint8_t *buf)
{
    bool has_null = false;

    for (int i = 0; i < MSG_LEN; i++)
    {
        uint8_t c = buf[i];

        if (c == 0xFF)
            break; // 消去領域
        if (c == '\0')
        {
            has_null = true;
            break;
        }
        if (c < 0x20 || c > 0x7E) // 非ASCII
            return false;
    }
    return has_null;
}

//==========================================
// フラッシュメモリ読み出し＋初期化
//==========================================
void init_flash_messages(void)
{
    uint8_t buf[FLASH_PAGE_SIZE];

    eep.begin(MSG_COUNT + 1); // +1: 設定ページ(PAGE_SETTINGS)

    for (int i = 0; i < MSG_COUNT; i++)
    {
        memset(buf, 0, sizeof(buf));
        eep.read(i, buf);

        if (!is_valid_message(buf))
        {
            // Flashが空 or ゴミ→デフォルト投入
            strncpy(msgs[i], default_msgs[i], MSG_LEN - 1);
            msgs[i][MSG_LEN - 1] = '\0';
            //printf("MSG%d: default\n", i + 1);
        }
        else
        {
            memcpy(msgs[i], buf, MSG_LEN);
            msgs[i][MSG_LEN - 1] = '\0';
            //printf("MSG%d: loaded from flash\n", i + 1);
        }
    }
    load_settings_from_flash();
}

//==========================================
// フラッシュメモリ書き込み
//==========================================
void save_current_message_to_flash(void)
{
    uint8_t buf[FLASH_PAGE_SIZE];
    int page = cur_msg; // MSG番号 = ページ番号

    memset(buf, 0xFF, sizeof(buf));
    strncpy((char *)buf, msgs[cur_msg], MSG_LEN);

    eep.erase(page);
    eep.write(page, buf);

    //DEBUG_PRINTF("[FLASH] save MSG%d\n", cur_msg + 1);
}

void load_settings_from_flash(void)
{
    uint8_t buf[FLASH_PAGE_SIZE];
    memset(buf, 0, sizeof(buf));
    eep.read(PAGE_SETTINGS, buf);
    if (buf[0] == SETTINGS_MAGIC) {
        paddle_swap          = (buf[1] != 0);
        sidetone_enabled     = (buf[2] != 0);
        tone_freq_idx        = (buf[3] < 3) ? buf[3] : 1;  // 旧データ(0xFF)はデフォルト651Hz
        repeat_interval_idx  = (buf[4] < 6) ? buf[4] : 0;
        standby_time_idx     = (buf[5] < 4) ? buf[5] : 0;
    } else {
        paddle_swap          = false;
        sidetone_enabled     = true;
        tone_freq_idx        = 1;
        repeat_interval_idx  = 0;
        standby_time_idx     = 0;
    }
    tone_div_val = (uint8_t)(tone_freq_idx + 2);
}

void save_settings_to_flash(void)
{
    uint8_t buf[FLASH_PAGE_SIZE];
    memset(buf, 0xFF, sizeof(buf));
    buf[0] = SETTINGS_MAGIC;
    buf[1] = paddle_swap         ? 1 : 0;
    buf[2] = sidetone_enabled    ? 1 : 0;
    buf[3] = tone_freq_idx;
    buf[4] = repeat_interval_idx;
    buf[5] = standby_time_idx;
    eep.erase(PAGE_SETTINGS);
    eep.write(PAGE_SETTINGS, buf);
}

//==========================================
// 手動パドル処理（SWA/SWBは混ぜない）
//==========================================
uint8_t job_paddle()
{
    static uint32_t left_time = 0;
    uint8_t key_dot, key_dash;

    // パドル入力を無視する場合、状態をリセットして終了
    if (ignore_paddle_input)
    {
        paddle = PDL_FREE;
        paddle_old = PDL_FREE;
        squeeze = SQZ_FREE;
        left_time = 0;
        return 0;
    }

    key_dot = (!GPIO_digitalRead(PIN_DOT));
    key_dash = (!GPIO_digitalRead(PIN_DASH));
    if (paddle_swap) { uint8_t t = key_dot; key_dot = key_dash; key_dash = t; }

    if (key_dot || key_dash) {
    last_activity_tick = tim1_tick256;
    }

    if (left_time != 0)
    {
        left_time--;
    }
    else
    {
        left_time = key_spd / 2;
        if (squeeze != SQZ_FREE)
            squeeze--;
    }

    if (squeeze != SQZ_FREE)
    {
        if (paddle_old == PDL_DOT && key_dash)
            paddle = PDL_DASH;
        else if (paddle_old == PDL_DASH && key_dot)
            paddle = PDL_DOT;
    }

    if (SQUEEZE_TYPE == 0)
    {
        if (squeeze > SQZ_DASH)
            paddle = PDL_FREE;
    }
    else
    {
        if (squeeze > SQZ_SPC)
            paddle = PDL_FREE;
    }

    if (squeeze > SQZ_SPC)
        return 1;
    else if (squeeze == SQZ_SPC || squeeze == SQZ_SPC0)
        return 0;

    if (paddle == PDL_FREE)
    {
        if (key_dot)
            paddle = PDL_DOT;
        else if (key_dash)
            paddle = PDL_DASH;
    }

    if (paddle == PDL_FREE)
        return 0;

    if (paddle == PDL_DOT)
        squeeze = SQZ_DOT;
    else
    {
        uint8_t dash_len = (SQZ_SPC * PDL_RATIO + 5) / 2;
        squeeze = SQZ_SPC + dash_len;
    }

    left_time = key_spd / 2;
    paddle_old = paddle;
    paddle = PDL_FREE;
    return 1;
}

//==========================================
// 自動送信処理（半ディット単位）簡略版
//==========================================
uint8_t job_auto(void)
{
    // 半ディット単位の状態機械
    typedef enum {
        AUTO_IDLE = 0,      // 次の文字取得
        AUTO_ELEM_ON,       // 要素(dit/dah) ON 中
        AUTO_ELEM_OFF,      // 要素間OFF (1 dit)
        AUTO_CHAR_GAP,      // 文字間ギャップ (3 dit)
        AUTO_WORD_GAP,      // 単語間ギャップ (7 dit)
        AUTO_REPEAT_WAIT    // リピート待機中 (1秒インターバル)
    } auto_state_t;

    static auto_state_t state = AUTO_IDLE;
    static uint32_t left_time = 0;   // 0.5dit タイマー
    static uint8_t half_rem = 0;     // 残り half-dit 数
    static const char *seq = nullptr;
    static uint8_t elem = 0;
    static uint16_t pos = 0;
    static uint32_t gap_until = 0;   // リピート待機終了時刻
    static const uint32_t repeat_gap_tbl[6] = {3906, 11718, 19531, 39062, 117188, 234375};

    // システムメッセージ中は固定速度
    // ※WPM 可変対応：毎回最新の key_spd を参照
    int current_key_spd = sys_msg_active ? key_spd_sys : key_spd;

    // リセット要求
    if (req_reset_auto) {
        req_reset_auto = false;
        state = AUTO_IDLE;
        left_time = 0;
        half_rem = 0;
        seq = nullptr;
        elem = 0;
        pos = 0;
        gap_until = 0;
        return 0;
    }

    // 自動送信でなければ何もしない
    if (!auto_mode || auto_msg == NULL) {
        return 0;
    }

    last_activity_tick = tim1_tick256;

    // ※0.5dit タイマー：最新速度で毎回リセット
    if (left_time > 0) {
        left_time--;
    } else {
        left_time = current_key_spd / 2;  // →ここが毎回最新になる


        if (half_rem > 0) {
            half_rem--;
        }

        // half_rem い0 になったら次の状態へ
        if (half_rem == 0) {
            switch (state) {

            case AUTO_IDLE: {
                char c = auto_msg[pos];

                // メッセージ終端
                if (c == '\0') {
                    morse_len = 0;
                    cw_r = cw_w;
                    key_off_ticks = 0;
                    key_on_ticks = 0;
                    flush_done = true;

                    if (repeat_mode) {
                        // リピートモード：1秒待機してから最初から再生
                        state = AUTO_REPEAT_WAIT;
                        gap_until = tim1_tick256 + repeat_gap_tbl[repeat_interval_idx];
                        half_rem = 0;
                    } else {
                        // 通常終了
                        auto_mode = false;
                        req_reset_auto = true;
                        auto_msg = NULL;
                        sys_msg_active = false;
                        keyout_enabled = true;
                        mode = MODE_KEYER;
                    }
                    return 0;
                }

                // スペース →単語間ギャップ
                if (c == ' ') {

                    // ※追加スペースをLCDに表示
                    if (!sys_msg_active) {
                        printAscii(' ');
                    }

                    // 連続スペースをスキップ
                    while (auto_msg[pos] == ' ') {
                        pos++;
                    }
                    state = AUTO_WORD_GAP;
                    half_rem = 14; // 7 dit = 14 half-dit
                    break;
                }

                // 通常文字
                if (!sys_msg_active) {
                    printAscii(c); // 送信文字を表示
                }

                seq = morseForChar(c);
                elem = 0;

                if (seq == nullptr) {
                    // 未対応文字→スペース扱い
                    pos++;
                    state = AUTO_WORD_GAP;
                    half_rem = 14;
                    break;
                }

                // 最初の要素を送信開始
                state = AUTO_ELEM_ON;
                half_rem = (seq[elem] == '.') ? 2 : 6; // dit=1dit=2half, dah=3dit=6half
                break;
            }

            case AUTO_ELEM_ON:
                // 要素 ON 終了→要素間OFF (1 dit)
                state = AUTO_ELEM_OFF;
                half_rem = 2; // 1 dit = 2 half-dit
                break;

            case AUTO_ELEM_OFF:
                // 次の要素へ
                elem++;
                if (seq[elem] == '\0') {
                    // 文字の最後の要素が終わった
                    char next = auto_msg[pos + 1];

                    if (next == '\0') {
                        // メッセージ末尾
                        morse_len = 0;
                        cw_r = cw_w;
                        key_off_ticks = 0;
                        key_on_ticks = 0;
                        flush_done = true;

                        if (repeat_mode) {
                            // リピートモード：1秒待機してから最初から再生
                            state = AUTO_REPEAT_WAIT;
                            gap_until = tim1_tick256 + repeat_gap_tbl[repeat_interval_idx];
                            half_rem = 0;
                        } else {
                            // 通常終了
                            auto_mode = false;
                            req_reset_auto = true;
                            auto_msg = NULL;
                            sys_msg_active = false;
                            keyout_enabled = true;
                            mode = MODE_KEYER;
                        }
                        return 0;
                    } else if (next == ' ') {
                        // ※ここでスペースをつ表示する
                        if (!sys_msg_active) {
                            printAscii(' ');
                        }

                        // 次がスペース →単語間ギャップ
                        pos++; // 現在の文字を進める
                        while (auto_msg[pos] == ' ') {
                            pos++;
                        }
                        state = AUTO_WORD_GAP;
                        half_rem = 11; // ELEM_OFF(2) + WORD_GAP(11) + AUTO_IDLE(1) = 14 half-dit = 7dit
                    } else {
                        // 次も文字→文字間ギャップ
                        pos++; // 次の文字へ
                        state = AUTO_CHAR_GAP;
                        half_rem = 3; // ELEM_OFF(2) + CHAR_GAP(3) + AUTO_IDLE(1) = 6 half-dit = 3dit
                    }
                } else {
                    // まだ要素が残ってい →次の要素 ON
                    state = AUTO_ELEM_ON;
                    half_rem = (seq[elem] == '.') ? 2 : 6;
                }
                break;

            case AUTO_CHAR_GAP:
                // 文字間ギャップ終了→次の文字へ
                state = AUTO_IDLE;
                break;

            case AUTO_WORD_GAP:
                // 単語間ギャップ終了→次の文字へ
                state = AUTO_IDLE;
                break;

            case AUTO_REPEAT_WAIT:
                // リピート待機中：1秒経過したら最初から再生
                if ((int32_t)(tim1_tick256 - gap_until) >= 0) {
                    pos = 0;
                    seq = nullptr;
                    elem = 0;
                    state = AUTO_IDLE;
                }
                break;

            default:
                state = AUTO_IDLE;
                break;
            }
        }
    }

    // 出力：要素 ON 状態のときだけキー ON
    return (state == AUTO_ELEM_ON) ? 1 : 0;
}

//==========================================
// トーン制御
//==========================================
void startTone()
{
    if (sidetone_enabled) {
        tone_on = true;
    }
    if (keyout_enabled)
    {
        GPIO_digitalWrite(PIN_KEYOUT, high); // ※常のみキーイング
    }
}

void stopTone()
{
    tone_on = false;

    GPIO_digitalWrite(PIN_TONE, low);

    if (keyout_enabled)
    {
        GPIO_digitalWrite(PIN_KEYOUT, low); // ※常のみキーイング
    }
}

inline void keydown(void)
{
    if (!key_state)
    {
        key_state = true;
        key_last_tick = tim1_tick256;
    }
    startTone();
}

inline void keyup(void)
{
    if (key_state)
    {
        key_state = false;
        key_last_tick = tim1_tick256;
    }
    stopTone();
}


static inline long map(long x,
                       long in_min, long in_max,
                       long out_min, long out_max)
{
    // Arduino本家と同じくゼロ除算チェックはしない（n_max == in_min だと未定義動作）
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void draw_wpm_value(void)
{
    char buf[6];
    buf[0] = (wpm < 10) ? ' ' : ('0' + wpm / 10);
    buf[1] = '0' + wpm % 10;
    buf[2] = 'W'; buf[3] = 'P'; buf[4] = 'M';
    buf[5] = '\0';
    drawstr_6px(48, 0, buf, 1);
}

static void draw_bat_value(void)
{
    ssd1306_fillRect(100, 0, 28, 8, 0);
    if (bat_mv < 800) {
        drawstr_6px(100, 0, "USB", 1);
    } else {
        uint16_t rounded = bat_mv + 50;
        ssd1306_drawchar_sz(100, 0, '0' + (rounded / 1000),       1, fontsize_8x8);
        ssd1306_drawchar_sz(107, 0, '.',                           1, fontsize_8x8);
        ssd1306_drawchar_sz(111, 0, '0' + ((rounded % 1000)/100), 1, fontsize_8x8);
        ssd1306_drawchar_sz(119, 0, 'V',                           1, fontsize_8x8);
    }
}

//==========================================
// ADCからスピード読み込み
//==========================================
void update_speed_from_adc()
{
    GPIO_digitalWrite(PIN_VR_PWR, high); // VR電源ON
    Delay_Us(100);                        // 安定待ち
    int adc = GPIO_analogRead(GPIO_Ain0_A2);
    GPIO_digitalWrite(PIN_VR_PWR, low);  // VR電源OFF
    wpm = map(adc, 0, 1023, WPM_MAX, WPM_MIN);
    dit_est = (1200UL * 1000)/(wpm * 256);
    key_spd = dit_est;

    if (wpm == last_wpm) {
        return;
    }
    last_wpm = wpm;

    // WPM描画
    draw_wpm_value();

    // メモリ再生中は printAsc() いrefresh する
    if (mode == MODE_PLAY) {
        return;
    }

    // ※行全体を描画（page6/page7の上部）
    for (int i = 0; i < colums; i++) {
        ssd1306_drawchar_sz(i * FONT_WIDTH, LINE_HEIGHT * 3,
                            line3[i], FONT_COLOR, FONT_SCALE_16X16);
    }

    // ※右端8px（x=120、w=27, y=56、h=3）をクリア
    //ssd1306_fillRect(120, 56, 8, 8, 0);

    // refresh
    ssd1306_refresh();
}



//==========================================
// 電池電圧測定
//==========================================
#define ADC_VREF_MV 3074  // 実測VCC（DC/DC 3.3V出力 − ダイオード降下）
uint16_t read_battery_mv(void)
{
    int adc = GPIO_analogRead(GPIO_Ain6_D6);
    return (uint16_t)((uint32_t)adc * ADC_VREF_MV / 1023);
}

void update_battery_status(void)
{
    bat_mv = read_battery_mv();
    bat_low_warn = (bat_mv >= 800 && bat_mv < 2000);
}

//==========================================
// スイッチ状態確認（各ハンドラから呼ぶ)
//==========================================
void update_switch_status(void)
{

    // スイッチ確認
    sw_stat = sw_get_info();
    sw_mode = sw_stat & MASK_MODE;
    sw_stat &= ~MASK_MODE;

    // ディット表示
    // if (sw_stat)
    // {
    //     //printf("SW mode=%02X stat=%02X mask=%02X\n",
    //            //sw_mode, sw_stat, sw_mask);
    // }
}

/**
 * @brief  スイッチカウンタ
 */
uint8_t sw_chatter(uint8_t sw, uint8_t *counter)
{
    uint8_t is_clicked = 0;

    if (sw)
    {
        // カウンタが55に達していなければ、カウンタを増やす
        *counter += ((*counter != 255) ? 1 : 0);
    }
    else
    {
        is_clicked = ((SW_PUSH_TH < *counter) && (*counter < SW_PRESS_TH)) ? 1 : 0;
        *counter = 0;
    }
    return (is_clicked);
}

/**
 * @brief  スイッチチェック
 * タイマー割り込みで呼ばれる
 */
void sw_check()
{
    uint8_t temp_pin;

    bool s1 = !GPIO_digitalRead(PIN_SW1);
    bool s2 = !GPIO_digitalRead(PIN_SW2);
    bool s3 = !GPIO_digitalRead(PIN_SW3);
    bool s4 = !GPIO_digitalRead(PIN_SW4);

    temp_pin = (s1 << 3) | (s2 << 2) | (s3 << 1) | (s4 << 0);

    if (sw_mask != 0)
    {
        count_sw[0] = count_sw[1] = count_sw[2] = count_sw[3] = 0;
        sw_clicked = 0;

        if ((temp_pin & 0x0F) == 0)
            sw_mask = 0;

        return;
    }

    sw_clicked |= (sw_chatter(temp_pin & SW_1, &count_sw[0])) ? SW_1 : 0;
    sw_clicked |= (sw_chatter(temp_pin & SW_2, &count_sw[1])) ? SW_2 : 0;
    sw_clicked |= (sw_chatter(temp_pin & SW_3, &count_sw[2])) ? SW_3 : 0;
    sw_clicked |= (sw_chatter(temp_pin & SW_4, &count_sw[3])) ? SW_4 : 0;

    if (sw_clicked) {
    last_activity_tick = tim1_tick256;
    }
}

//****************************
// スイッチを操作したら呼ぶ
//  メモリ再生の停止用
// ****************************
uint8_t sw_is_pressed()
{
    if (sw_mask != 0)
    {
        return (0);
    }

    // なんか押し操作された
    if ((count_sw[0] > SW_PUSH_TH) || (count_sw[1] > SW_PUSH_TH) || (count_sw[2] > SW_PUSH_TH) || (count_sw[3] > SW_PUSH_TH))
    {
        return (1);
    }
    return (0);
}

/*****************************************************************************
 スイッチ状態読み込み　(8ビット情報を返す)
 xxxxdcba
            dcba sw[1,2,3,4] に対必要
            xxxx フラグ
                0000 何もない
                0001 クリック
                0010 長押い
                0100 ダブル押い
 ******************************************************************************/
uint8_t sw_get_info()
{

    volatile uint8_t count = 0;

    if (sw_mask != 0)
    {
        // 1bitでもスイッチマスクがかかっていら、押されてないとにする
        return (0);
    }

    // 複数押されている！
    if (count_sw[0] > SW_PUSH_TH)
    {
        count += 1;
        flg |= SW_1;
    }
    if (count_sw[1] > SW_PUSH_TH)
    {
        count += 1;
        flg |= SW_2;
    }
    if (count_sw[2] > SW_PUSH_TH)
    {
        count += 1;
        flg |= SW_3;
    }
    if (count_sw[3] > SW_PUSH_TH)
    {
        count += 1;
        flg |= SW_4;
    }
    if (count >= 2)
    {
        return (SW_INFO_DOUBLE | flg);
    }

    flg = 0;
    // なんか長押しされた
    if (count_sw[0] > SW_PRESS_TH)
    {
        flg |= SW_1;
    }
    if (count_sw[1] > SW_PRESS_TH)
    {
        flg |= SW_2;
    }
    if (count_sw[2] > SW_PRESS_TH)
    {
        flg |= SW_3;
    }
    if (count_sw[3] > SW_PRESS_TH)
    {
        flg |= SW_4;
    }
    if (flg != 0)
    {
        sw_mask = 1;
        return (SW_INFO_PRESS | flg);
    }

    flg = 0;
    if (sw_clicked != 0)
    {
        // なんかクリックされていた！
        flg = SW_INFO_CLICK | sw_clicked;
        sw_clicked = 0;
    }
    return (flg);
}

//==========================================
// 編集の次の文字へ
//==========================================
char next_char(char c)
{
    const char *p = strchr(edit_table, c);
    if (!p) return edit_table[0];

    p++;  // 次へ

    // 末尾なら先頭へ
    if (*p == '\0')
        return edit_table[0];

    return *p;
}



//==========================================
// 編集の前の文字へ
//==========================================
char prev_char(char c)
{
    const char *p = strchr(edit_table, c);
    if (!p) return edit_table[0];

    if (p == edit_table)
        p = edit_table + strlen(edit_table) - 1;
    else
        p--;

    return *p;
}



//==========================================
// 編集のそれ以降の文字を削除
//==========================================
// void edit_clear_after_cursor(void)
// {
//     for (uint8_t i = edit_pos; i < MSG_LEN; i++)
//     {
//         msgs[cur_msg][i] = '\0';
//     }
//     edit_len = edit_pos;
// }

//==========================================
// 編集の表示を調整
//==========================================
void adjust_edit_view(void)
{
    if (edit_pos < edit_view_left)
    {
        edit_view_left = edit_pos;
    }
    else if (edit_pos >= edit_view_left + DISP_COLS)
    {
        edit_view_left = edit_pos - DISP_COLS + 1;
    }
}

//==========================================
// 汎用システムメッセージ送信開始
//==========================================
void play_sys_msg(const char *msg, uint8_t wpm_val)
{
    auto_msg = msg;
    sys_msg_active = true;
    last_activity_tick = tim1_tick256;

    // システムメッセージ用の固定WPMを設定
    wpm_sys = wpm_val;
    key_spd_sys = 4687 / wpm_sys;

    keyout_enabled = false; // RFキーイングしない
    req_reset_auto = true;
    auto_mode = true;
    mode = MODE_PLAY;
}

void play_mem_msg(uint8_t n)
{
    auto_msg = msgs[n];
    sys_msg_active = false;
    last_activity_tick = tim1_tick256;

    keyout_enabled = true; // 通常キーイング
    req_reset_auto = true;
    auto_mode = true;
    mode = MODE_PLAY;
}

//==========================================
//  ストレートキー読み込み
//==========================================
bool read_straight_key(void)
{
    return !GPIO_digitalRead(PIN_ST); // 押されたら true
}


//==========================================
//  CWイベントをリングバッファにプッシュ
//==========================================
static inline void cw_push(cw_event_t ev)
{
    // 再生中（自動送信）やPLAYモード中はデコード用バッファに入れない
    if (auto_mode || mode == MODE_PLAY) return;

    uint8_t next = (cw_w + 1) % CW_BUF_SIZE;
    if (next != cw_r) {
        cw_buf[cw_w] = ev;
        cw_w = next;
    }
}

//==========================================
//  モールス符号から文字へ変換
//==========================================
char morse_to_char(const char *m)
{
    static const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.,?/=+";
    for (const char *p = charset; *p; p++) {
        const char *code = morseForChar(*p);
        if (code && strcmp(code, m) == 0) return *p;
    }
    return '*';
}


//==========================================
//  ON確定時の処理
//==========================================
void process_on(uint32_t ticks)
{
    if (ticks < MIN_ON_TICKS) return;

    // ==== DIT ====
    if (ticks <= DOT_MAX) {
        cw_push(EV_DOT);
        //("[.] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return;
    }
    // ==== DAH ====
    else {
        cw_push(EV_DASH);
        //printf("[-] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return; 
    }
}

//==========================================
//  OFF確定時の処理
//==========================================
void process_off(uint32_t ticks)
{
    if (ticks < MIN_OFF_TICKS) return;

    //語間
    if (ticks >= WORD_GAP_MIN) {
        //printf("[7-ON] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return; 
    }

    /* ==== 文字間 ==== */
    if (ticks >= CHAR_GAP_MIN) {
        cw_push(EV_CHAR_GAP);
        //printf("[3] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return;
    }

    /* ==== 符号閁E(1dit) ==== */
    if (ticks < CHAR_GAP_MIN) {
        //printf("[ ] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return;
    }
}


//==========================================
//  デコード処理
//==========================================
void cw_decode_task(void)
{
    /*
     * Make a local snapshot of the producer index to avoid races with
     * the IRQ that pushes events (single producer in ISR, single consumer
     * here). This avoids reading cw_w multiple times while it changes.
     */
    uint8_t w = cw_w;
    while (cw_r != w)
    {
        cw_event_t ev = (cw_event_t)cw_buf[cw_r];
        cw_r = (cw_r + 1) % CW_BUF_SIZE;

        switch (ev)
        {
        case EV_DOT:
            if (morse_len < MORSE_BUF_LEN - 1) {
                morse_buf[morse_len++] = '.';
            }
            break;

        case EV_DASH:
            if (morse_len < MORSE_BUF_LEN - 1) {
                morse_buf[morse_len++] = '-';
            }
            break;

        case EV_CHAR_GAP:
            if (morse_len > 0) {
                morse_buf[morse_len] = '\0';   // 文字列化
                char c = morse_to_char(morse_buf);
                if (mode != MODE_PLAY && !auto_mode) {
                    printAscii(c); // LCD表示
                }
                //if (mode != MODE_PLAY) DEBUG_PRINTF("Decoded='%c', Buf=%s, Len=%d\r\n", c, morse_buf, morse_len);
                morse_len = 0;
            }
            break;

        case EV_WORD_GAP:
            if (morse_len > 0) {
                morse_buf[morse_len] = '\0';
                char c = morse_to_char(morse_buf);
                if (mode != MODE_PLAY && !auto_mode) {
                    printAscii(c); // LCD表示
                }
                //if (mode != MODE_PLAY) DEBUG_PRINTF("Decoded='%c', Buf=%s, Len=%d\r\n", c, morse_buf, morse_len);
                morse_len = 0;
            }
            if (mode != MODE_PLAY && !auto_mode) {
                printAscii(32); // スペース表示
            }
            break;

        }
        /* refresh local snapshot in case producer advanced while processing */
        w = cw_w;
    }
}


//==========================================
//  KEY出力処理（割り込みから呼ぶこと）
//==========================================
void service_keyer(void)
{
    static bool prev_on = false;
    bool on = false;

    if (mode == MODE_PLAY)
    {
        on = auto_mode ? job_auto() : false;
    }
else if (mode == MODE_KEYER)
{
    bool st = read_straight_key();  // 電鍵
    bool pd = job_paddle();         // パドルの状態機械
    on = st || pd;

    // ※ストレートキーでスリープ復帰
    if (st) {
        last_activity_tick = tim1_tick256;

        // ※メモリ再生中なら強制停止
        if (mode == MODE_PLAY || auto_mode) {
            stop_play();
        }
    }
}

    /* tickカウント（唯一の場所）*/
    if (on) {
        last_activity_tick = tim1_tick256;
        key_on_ticks++;
    } else {
        key_off_ticks++;
    }

    /* 状態変化検知 */
    if (on && !prev_on)
    {
        // OFF →ON
        process_off(key_off_ticks);        

        key_off_ticks = 0;
        key_on_ticks = 0;
        keydown();
    }
    else if (!on && prev_on)
    {
        // ON →OFF
        process_on(key_on_ticks);

        key_on_ticks = 0;
        key_off_ticks = 0;
        keyup();
    }

    prev_on = on;
}


//==========================================
//  #1 KEYERモード処理
//==========================================
void handle_keyer_mode(void)
{

    /* パドル入力無視フラグをクリア（パドルが両方離されたら）*/
    bool dot = !GPIO_digitalRead(PIN_DOT);
    bool dash = !GPIO_digitalRead(PIN_DASH);
    if (ignore_paddle_input && !dot && !dash)
    {
        ignore_paddle_input = false;
    }

    // スイッチ状態取取得
    update_switch_status();

    /* 編集モードへ */
    //update_switch_status();

    if (sw_mode == SW_INFO_DOUBLE && (sw_stat & (SW_1 | SW_2)) == (SW_1 | SW_2))
    {
        SW_CLEAR();
        mode = MODE_EDIT_SELECT;
        draw_edit_select();
        return;
    }

    /* F3+F4ダブルクリック → 設定モードへ */
    if (sw_mode == SW_INFO_DOUBLE && (sw_stat & (SW_3 | SW_4)) == (SW_3 | SW_4))
    {
        SW_CLEAR();
        setup_cursor = 0;
        setup_scroll = 0;
        mode = MODE_SETUP;
        draw_setup_screen();
        return;
    }

    /* メモリ再生 */
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_1)
    {
        SW_CLEAR();
        //printf("START PLAY MSG1\r\n");
        mode = MODE_PLAY;
        start_play(0);
    }

    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_2)
    {
        SW_CLEAR();
        //printf("START PLAY MSG2\r\n");
        mode = MODE_PLAY;
        start_play(1);
    }

        if (sw_mode == SW_INFO_CLICK && sw_stat == SW_3)
    {
        SW_CLEAR();
        //printf("START PLAY MSG3\r\n");
        mode = MODE_PLAY;
        start_play(2);
    }

    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_4)
    {
        SW_CLEAR();
        //printf("START PLAY MSG4\r\n");
        mode = MODE_PLAY;
        start_play(3);
    }

    // SWx 長押し → リピート再生開始
    {
        uint8_t rpt = 0xff;
        if      (sw_mode == SW_INFO_PRESS && sw_stat == SW_1) rpt = 0;
        else if (sw_mode == SW_INFO_PRESS && sw_stat == SW_2) rpt = 1;
        else if (sw_mode == SW_INFO_PRESS && sw_stat == SW_3) rpt = 2;
        else if (sw_mode == SW_INFO_PRESS && sw_stat == SW_4) rpt = 3;
        if (rpt != 0xff) {
            SW_CLEAR();
            repeat_mode = true;
            repeat_msg_idx = rpt;
            // タイトルを "RPT" に更新
            ssd1306_fillRect(0, 0, 40, 8, 0);
            drawstr_6px(0, 0, "RPT", 1);
            ssd1306_refresh();
            start_play(rpt);  // play_mem_msgより完全な初期化
        }
    }
}

//==========================================
//  #2 PLAYモード（自動リピート防止）
//==========================================
void handle_play_mode(void)
{
    // パドル入力を直接読み込み
    bool dot = !GPIO_digitalRead(PIN_DOT);
    bool dash = !GPIO_digitalRead(PIN_DASH);
    bool st = read_straight_key();   // ※ストレートキー追加

    //パドル無視フラグがONで、パドルが両方離されたらフラグをクリア
    if (ignore_paddle_input && !dot && !dash)
    {
        ignore_paddle_input = false;
    }

    // 何か操作したら止める（リピートも解除）
    if (sw_is_pressed() || dot || dash || st)
    {
        SW_CLEAR();
        stop_play();
        repeat_mode = false;
        mode = MODE_KEYER;
        draw_keyer_screen();
        ssd1306_refresh();
        return;
    }

    // 再生完了チェック
    // リピートモード中は job_auto 内の AUTO_REPEAT_WAIT が1秒待機後に
    // 自動で先頭へ戻るため、ここでは非リピート完了のみ処理する
    if (!auto_mode && !repeat_mode)
    {
        stop_play();
        mode = MODE_KEYER;
        last_activity_tick = tim1_tick256;
        flush_done = true;
        key_off_ticks = 0;
        key_on_ticks = 0;
    }
}

//==========================================
//  #3 編集モード処理：メモリ選択
//==========================================
void handle_edit_select(void)
{
    // スイッチ状態取取得
    update_switch_status();


    if (sw_mode == SW_INFO_CLICK) {

        if (sw_stat & SW_1) cur_msg = 0;
        if (sw_stat & SW_2) cur_msg = 1;
        if (sw_stat & SW_3) cur_msg = 2;
        if (sw_stat & SW_4) cur_msg = 3;

        // 編集開始
        edit_pos = 0;
        edit_len = strlen(msgs[cur_msg]);

        mode = MODE_EDIT;
        draw_edit_screen();
        return;
    }

    if (sw_mode == SW_INFO_DOUBLE && (sw_stat & (SW_1 | SW_2)) == (SW_1 | SW_2))
    {
        SW_CLEAR();
        mode = MODE_KEYER;
        last_activity_tick = tim1_tick256;
        draw_keyer_screen();
        ssd1306_refresh();
        //printf("BACK TO KEYER\r\n");
        return;
    }
}

//==========================================
//  #4 編集モード処理：文字選択
//==========================================
void handle_edit_mode(void)
{
    // 編集：毎回描画
    if (edit_first)
    {
        draw_edit_screen();
        edit_first = 0;
        edit_dot_prev = false;
        edit_dash_prev = false;
    }

    // スイッチ状態取取得
    update_switch_status();

    bool dot = !GPIO_digitalRead(PIN_DOT);
    bool dash = !GPIO_digitalRead(PIN_DASH);

    /* ===== 10ms周期でのみ編集処理 ===== */
    if (edit_tick_10ms)
    {
        edit_tick_10ms = false;

        /* ---- DOT：戻る！ --- */
        if (dot)
        {
            // 新規押下（前フレームが前で今フレームが押されている） 即座に反映
            if (!edit_dot_prev)
            {
                msgs[cur_msg][edit_pos] =
                    prev_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                draw_edit_screen();
                edit_dot_cnt = 0;
            }
            else
            {
                // 押し続け →リピート開始
                edit_dot_cnt++;
                if (edit_dot_cnt == EDIT_REPEAT_START ||
                    (edit_dot_cnt > EDIT_REPEAT_START &&
                     (edit_dot_cnt - EDIT_REPEAT_START) % EDIT_REPEAT_SPEED == 0))
                {

                    msgs[cur_msg][edit_pos] =
                        prev_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                    draw_edit_screen();
                }
            }
            edit_dot_prev = true;
        }
        else
        {
            edit_dot_cnt = 0;
            edit_dot_prev = false;
        }

        /* ---- DASH：進む --- */
        if (dash)
        {
            // 新規押下（前フレームが前で今フレームが押されている） 即座に反映
            if (!edit_dash_prev)
            {
                msgs[cur_msg][edit_pos] =
                    next_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                draw_edit_screen();
                edit_dash_cnt = 0;
            }
            else
            {
                // 押し続け →リピート開始
                edit_dash_cnt++;
                if (edit_dash_cnt == EDIT_REPEAT_START ||
                    (edit_dash_cnt > EDIT_REPEAT_START &&
                     (edit_dash_cnt - EDIT_REPEAT_START) % EDIT_REPEAT_SPEED == 0))
                {

                    msgs[cur_msg][edit_pos] =
                        next_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                    draw_edit_screen();
                }
            }
            edit_dash_prev = true;
        }
        else
        {
            edit_dash_cnt = 0;
            edit_dash_prev = false;
        }
    }

    /* ===== 以下は従来どおり・即時反応でOK ===== */

    // ※SW4 長押い→保存して終了
    if (sw_mode == SW_INFO_PRESS && sw_stat == SW_4)
    {
        SW_CLEAR();
        save_current_message_to_flash();
        //printf("Message recorded\r\n");
        mode = MODE_KEYER;
        last_activity_tick = tim1_tick256;
        draw_keyer_screen();
        ssd1306_refresh();
        return;
    }

    // ※SW1 長押い→カーソル以降削除
    if (sw_mode == SW_INFO_PRESS && sw_stat == SW_1)
    {
        SW_CLEAR();
        //edit_clear_after_cursor();
        msgs[cur_msg][edit_pos] = '\0';
        edit_len = strlen(msgs[cur_msg]);
        draw_edit_screen();
        return;
    }

        // ※SW1 →前の文字へ
    if (sw_mode == SW_INFO_CLICK && (sw_stat & SW_1)) {
        msgs[cur_msg][edit_pos] = prev_char(msgs[cur_msg][edit_pos]);
        draw_edit_screen();
        return;
    }

    // ※SW2 →次の文字へ
    if (sw_mode == SW_INFO_CLICK && (sw_stat & SW_2)) {
        msgs[cur_msg][edit_pos] = next_char(msgs[cur_msg][edit_pos]);
        draw_edit_screen();
        return;
    }

    // ※SW4 →カーソル進む
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_4)
    {
        SW_CLEAR();
        if (edit_pos < MSG_LEN - 2) // フラッシュページ64Bのためヌル込み最大63文字
        {
            edit_pos++;
            if (edit_pos >= edit_len)
            {
                msgs[cur_msg][edit_pos] = ' ';
                edit_len = edit_pos + 1;
                msgs[cur_msg][edit_len] = '\0';
            }
            adjust_edit_view();
        }
        draw_edit_screen();
    }

    // ※SW3 →カーソル戻い
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_3 && edit_pos > 0)
    {
        SW_CLEAR();
        edit_pos--;
        adjust_edit_view();
        draw_edit_screen();
    }
}

//==========================================
//  6px送り文字列描画（8x8フォント、文字間を詰める）
//==========================================
static void drawstr_6px(uint8_t x, uint8_t y, const char *str, uint8_t color)
{
    while (*str) {
        ssd1306_drawchar_sz(x, y, *str++, color, fontsize_8x8);
        x += 6;
    }
}

//==========================================
//  #6 設定画面描画（5項目・スクロール対応）
//==========================================
void draw_setup_screen(void)
{
    static const char* labels[5] = {
        "PDL SWAP", "SIDETONE", "TONE FREQ", "REPEAT INT", "STANDBY"
    };
    static const char* tone_strs[3]   = {"976Hz", "651Hz", "488Hz"};
    static const char* rpt_strs[6]    = {" 1s", " 3s", " 5s", "10s", "30s", "60s"};
    static const char* stby_strs[4]   = {"10s", "30s", "60s", "---"};

    const char* values[5];
    values[0] = paddle_swap      ? " ON" : "OFF";
    values[1] = sidetone_enabled ? " ON" : "OFF";
    values[2] = tone_strs[tone_freq_idx];
    values[3] = rpt_strs[repeat_interval_idx];
    values[4] = stby_strs[standby_time_idx];

    ssd1306_setbuf(0);
    drawstr_6px(0, 0, "SETUP", 1);
    ssd1306_drawFastHLine(0, 10, 128, 1);

    for (int i = 0; i < 3; i++) {
        int idx = setup_scroll + i;
        if (idx >= 5) break;
        int y = 14 + i * 14;
        drawstr_6px(0,  y, (setup_cursor == idx ? ">" : " "), 1);
        drawstr_6px(7,  y, labels[idx], 1);
        drawstr_6px(74, y, values[idx],  1);
    }

    // スクロールインジケータ
    if (setup_scroll > 0)
        drawstr_6px(122, 12, "^", 1);
    if (setup_scroll + 3 < 5)
        drawstr_6px(122, 40, "v", 1);

    // ボタンガイド（F1:↑ F2:↓ F3:CHG F4:SAV）
    ssd1306_fillRect(3,  56, 27, 8, 1);
    ssd1306_fillRect(35, 56, 27, 8, 1);
    ssd1306_fillRect(67, 56, 27, 8, 1);
    ssd1306_fillRect(99, 56, 27, 8, 1);

    const char arrow_up[]   = { 0x04, '\0' };
    const char arrow_down[] = { 0x01, '\0' };
    drawstr_6px(13,  56, (const char*)arrow_up,   0);
    drawstr_6px(45,  56, (const char*)arrow_down, 0);
    drawstr_6px(72,  56, "CHG", 0);
    drawstr_6px(102, 56, "SAV", 0);

    ssd1306_refresh();
}

//==========================================
//  #6 設定モード処理（5項目・スクロール対応）
//  F1:カーソル上  F2:カーソル下  F3:値切替  F4:保存して終了
//==========================================
void handle_setup_mode(void)
{
    update_switch_status();

    // F1クリック → カーソル上
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_1)
    {
        SW_CLEAR();
        if (setup_cursor > 0) {
            setup_cursor--;
            if (setup_cursor < setup_scroll) setup_scroll = setup_cursor;
        }
        draw_setup_screen();
        return;
    }

    // F2クリック → カーソル下
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_2)
    {
        SW_CLEAR();
        if (setup_cursor < 4) {
            setup_cursor++;
            if (setup_cursor >= setup_scroll + 3) setup_scroll = setup_cursor - 2;
        }
        draw_setup_screen();
        return;
    }

    // F3クリック → 選択中の設定値を切り替え
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_3)
    {
        SW_CLEAR();
        switch (setup_cursor) {
        case 0: paddle_swap         = !paddle_swap;                              break;
        case 1: sidetone_enabled    = !sidetone_enabled;                         break;
        case 2:
            tone_freq_idx = (tone_freq_idx + 2) % 3;
            tone_div_val  = (uint8_t)(tone_freq_idx + 2);
            break;
        case 3: repeat_interval_idx = (repeat_interval_idx + 1) % 6;            break;
        case 4: standby_time_idx    = (standby_time_idx    + 1) % 4;            break;
        }
        draw_setup_screen();
        return;
    }

    // F4クリック → フラッシュに保存してキーヤー画面へ戻る
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_4)
    {
        SW_CLEAR();
        save_settings_to_flash();
        mode = MODE_KEYER;
        draw_keyer_screen();
        last_wpm = -999;
        update_speed_from_adc();
        ssd1306_refresh();
        return;
    }
}

//==========================================
//  #1 スタート画面
//==========================================
void draw_startup_screen(void)
{
    update_battery_status();

    ssd1306_drawstr_sz(0, 10, "KEYER DS", 1, fontsize_16x16);
    drawstr_6px(0, 30, "Powered by", 1);
    drawstr_6px(40, 40, "UIAPduino", 1);
    ssd1306_drawFastHLine(0, 50, 128, 1);
    drawstr_6px(0, 52, "Version 0.4", 1);
    ssd1306_refresh();
}

//==========================================
//  #2 キーヤー画面
//==========================================

void draw_keyer_screen(void)
{
    ssd1306_setbuf(0); // 0=黒, 1=白
    drawstr_6px(0, 0, "KEYER", 1);
    draw_wpm_value();
    draw_bat_value();
    ssd1306_drawFastHLine(0, 10, 128, 1);
}

//==========================================
//  #3 録音するメモリーを選択する画面
//==========================================
void draw_edit_select(void)
{
    ssd1306_setbuf(0); // 0=黒, 1=白
    drawstr_6px(0, 0, "EDIT MSG", 1);
    ssd1306_drawFastHLine(0, 10, 128, 1);
    drawstr_6px(0, 16, "F1-F4:SELECT MSG", 1);
    drawstr_6px(0, 25, "F1+F2:Return", 1);
    ssd1306_fillRect(3, 48, 27, 16, 1);
    ssd1306_fillRect(35, 48, 27, 16, 1);
    ssd1306_fillRect(67, 48, 27, 16, 1);
    ssd1306_fillRect(99, 48, 27, 16, 1);    
    ssd1306_drawstr_sz(12, 48, "1", 0, fontsize_16x16);  
    ssd1306_drawstr_sz(44, 48, "2", 0, fontsize_16x16);
    ssd1306_drawstr_sz(76, 48, "3", 0, fontsize_16x16);
    ssd1306_drawstr_sz(108, 48, "4", 0, fontsize_16x16);
    //printf("Select Message\r\n");
    ssd1306_refresh();
    // oled_refreshed_this_frame = true;
}

//==========================================
//  #4 メモリー編集
//==========================================
void draw_edit_screen(void)
{
    char buf[32];

    if (edit_pos >= edit_len && msgs[cur_msg][edit_pos] != '\0') {
        edit_len = edit_pos + 1;
        msgs[cur_msg][edit_len] = '\0';
    }

    ssd1306_setbuf(0);

    /* ===== タイトル ===== */
    buf[0]='E'; buf[1]='D'; buf[2]='I'; buf[3]='T'; buf[4]=' ';
    buf[5]='M'; buf[6]='S'; buf[7]='G'; buf[8]='0'+(cur_msg+1); buf[9]='\0';
    drawstr_6px(0, 0, buf, 1);

    /* ===== カーソル位置表示 (XX/63) ===== */
    {
        uint8_t pos = edit_pos + 1;
        buf[0] = '0' + pos / 10;
        buf[1] = '0' + pos % 10;
        buf[2] = '/';
        buf[3] = '0' + (MSG_LEN - 1) / 10;
        buf[4] = '0' + (MSG_LEN - 1) % 10;
        buf[5] = '\0';
    }
    drawstr_6px(80, 0, buf, 1);

    ssd1306_drawFastHLine(0, 10, 128, 1);

/* ===== メッセージ表示用フォント幅の定義 ===== */
#define EDIT_FONT_W 12

    /* ===== メッセージ表示用文字ウィンドウ ===== */
    for (uint8_t i = 0; i < DISP_COLS; i++)
    {
        uint8_t idx = edit_view_left + i;
        char c = (idx < edit_len) ? msgs[cur_msg][idx] : ' ';
        ssd1306_drawchar_sz(i * EDIT_FONT_W, 14, c, 1, fontsize_16x16);
    }

/* ===== カーソル ===== */
    int cursor_col = edit_pos - edit_view_left;
    if (cursor_col < 0)
        cursor_col = 0;
    if (cursor_col >= DISP_COLS)
        cursor_col = DISP_COLS - 1;

    int x = cursor_col * EDIT_FONT_W+EDIT_FONT_W/2 - 4; // カーソルを文字の中央に配置
    ssd1306_drawchar_sz(x, 32, '^', 1, fontsize_8x8);

    /* ===== 操作説明 ===== */

    ssd1306_fillRect(3, 37, 27, 8, 1);
    ssd1306_fillRect(35, 37, 27, 8, 1);
    ssd1306_fillRect(67, 37, 27, 8, 1);
    ssd1306_fillRect(99, 37, 27, 8, 1);

    const char arrow_up[]    = { 0x04, '\0' };
    const char arrow_down[]  = { 0x01, '\0' };
    const char arrow_right[]  = { 0x02, '\0' };
    const char arrow_left[] = { 0x03, '\0' };
    drawstr_6px(13, 37, (const char*)arrow_up, 0);
    drawstr_6px(45, 37, (const char*)arrow_down, 0);
    drawstr_6px(77, 37, (const char*)arrow_left, 0);
    drawstr_6px(109, 37, (const char*)arrow_right, 0);

    drawstr_6px(3, 48, "Press >1sec", 1);
    ssd1306_fillRect(3, 56, 27, 8, 1);
    ssd1306_fillRect(35, 56, 27, 8, 1);
    ssd1306_fillRect(67, 56, 27, 8, 1);
    ssd1306_fillRect(99, 56, 27, 8, 1);

    drawstr_6px(8, 56, "DEL", 0);
    drawstr_6px(104, 56, "END", 0);
    ssd1306_refresh();
    // oled_refreshed_this_frame = true;
}


//==========================================
//  タイマー割り込み
//==========================================
void TIM1_UP_IRQHandler(void)
{
    /* 割り込みフラグクリア */
    TIM1->INTFR &= (uint16_t)~TIM_IT_Update;

    /* キー処理ここで1回だけ！ */
    service_keyer();

    /* スイッチ処理分周 */
    if (++sw_div_cnt >= SW_SCAN_DIV)
    {
        sw_div_cnt = 0;
        sw_check();
    }

    /* IRQ tick（デバッグ用ならOK）*/
    tim1_tick256++;

    /* トーン制御 */
    if (tone_on)
    {
        if (++tone_div >= tone_div_val)
        {
            tone_div = 0;
            toggle_tone_pin();
        }
    }
    else
    {
        GPIO_digitalWrite(PIN_TONE, low);
    }
}

//==========================================
//  スタンバイモードへ入る
//==========================================
void enter_standby(void)
{
    const uint32_t wake_lines =
        exti_line_from_pin(PIN_SW1) | exti_line_from_pin(PIN_SW2) |
        exti_line_from_pin(PIN_SW3) | exti_line_from_pin(PIN_SW4) |
        exti_line_from_pin(PIN_DOT) | exti_line_from_pin(PIN_DASH) |
        exti_line_from_pin(PIN_ST); 

    tone_on = false;
    GPIO_digitalWrite(PIN_TONE, low);
    GPIO_digitalWrite(PIN_KEYOUT, low);

    TIM1->DMAINTENR &= ~TIM_IT_Update;
    TIM1->CTLR1 &= ~TIM_CEN;
    // 割り込みを無効化（有効化ではなく無効化！E
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

    GPIO_ADC_set_power(0);
    ADC1->CTLR1 = 0;
    ADC1->CTLR2 = 0;

    // --- OLED OFF（最重要）---
    ssd1306_cmd(SSD1306_DISPLAYOFF);   // あなたのSSD1306ライブラリに合わせて
    I2C1->CTLR1 &= ~I2C_CTLR1_PE;

    prepare_gpio_for_standby();

    RCC->APB1PCENR &= ~(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2);
    RCC->APB2PCENR &= ~(RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_USART1);

    // --- PWR クロック有効化 ---
    RCC->APB1PCENR |= RCC_APB1Periph_PWR;
    RCC->APB2PCENR |= RCC_APB2Periph_AFIO;

    // --- SLEEPDEEP = 1 ---
    exti_select_port(PIN_SW1 & 0x0f, 3);   // PD0
    exti_select_port(PIN_SW2 & 0x0f, 2);   // PC3
    exti_select_port(PIN_SW3 & 0x0f, 3);   // PD2
    exti_select_port(PIN_SW4 & 0x0f, 2);   // PC4
    exti_select_port(PIN_DOT & 0x0f, 2);   // PC5
    exti_select_port(PIN_DASH & 0x0f, 2);  // PC6
    exti_select_port(PIN_ST & 0x0f, 0);    // PA1

    EXTI->INTENR &= ~wake_lines;
    EXTI->EVENR = (EXTI->EVENR & ~0x7fu) | wake_lines;
    EXTI->RTENR &= ~wake_lines;
    EXTI->FTENR = (EXTI->FTENR & ~0x7fu) | wake_lines;
    EXTI->INTFR = wake_lines;

    NVIC->SCTLR |= (1 << 2);

    // --- Standby モード選択 ---
    PWR->CTLR |= PWR_CTLR_PDDS;

    // --- 復帰イベント設定！ SW1〜SW4, DOT, DASH, ST ---
    // 例：PA1, PA2, PA3, PA4, PC1, PC2, PC3 など
    // --- Standby へ ---
    __WFE();   // →IRQ ではないEVT で復帰
    restore_after_standby();
}

//==========================================
//  ループ処理
//==========================================
void loop(void)
{
    static uint16_t sec_cnt = 0;
    static uint32_t last_tick = 0;
    static const uint32_t standby_tbl[4] = {39062, 117188, 234375, 0}; // 10s,30s,60s,なし

    /* ==== 10ms周期(IRQ基準: 256us ×40 ≒10ms) ==== */
    if ((tim1_tick256 - last_tick) >= 40)
    {
        last_tick += 40; // advance by 40 ticks
        edit_tick_10ms = true;

        /* ==== EDIT中以外のWPM更新 ==== */
        if (mode != MODE_EDIT)
        {
            update_speed_from_adc();
        }

        /* ==== 1秒周期 ==== */
        if (++sec_cnt >= 100)
        { // 100 ×10ms = 1s
            sec_cnt = 0;
            // 電池電圧更新
            update_battery_status();
            // LOW BAT → 電圧表示を点滅（キーヤー画面のみ）
            if (mode == MODE_KEYER) {
                if (bat_low_warn) {
                    bat_blink = !bat_blink;
                    ssd1306_fillRect(96, 0, 32, 8, 0);
                    if (bat_blink) draw_bat_value();
                    ssd1306_refresh();
                } else {
                    draw_bat_value();
                    ssd1306_refresh();
                }
            }
#if DEBUG_MODE_PRINT
            //printf("[MODE] %s\r\n", mode_to_str(mode));
#endif
        }
    }

    /* ==== モード処理 ==== */
    switch (mode)
    {
    case MODE_KEYER:
        handle_keyer_mode();
        break;
    case MODE_PLAY:
        handle_play_mode();
        break;
    case MODE_EDIT_SELECT:
        handle_edit_select();
        break;
    case MODE_EDIT:
        handle_edit_mode();
        break;
    case MODE_SETUP:
        handle_setup_mode();
        break;
    }

    cw_decode_task();

    // ==== 無音タイムアウト（最後の文字確定用） ====
    // ※メモリ再生中は自動デコード確定を無効化
    if (!key_state && !flush_done && !auto_mode) {

        if (key_off_ticks >= WORD_GAP_MIN) {
            cw_push(EV_WORD_GAP);
            flush_done = true;
        }

    }
    else if (key_state) {
        flush_done = false;
    }

    // 再生中はキーON中のみ描画+リフレッシュ（ギャップ中はTIM1が正常に動くよう行わない）
    // 再生中以外は常時実行
    if (oled_dirty && (key_state || mode != MODE_PLAY)) {
        oled_dirty = false;
        redraw_lines();
        ssd1306_refresh();
    }

    // 無操作でスタンバイ（standby_time_idxで時間設定、3=なし）
    uint32_t now = tim1_tick256;
    uint32_t sb_th = standby_tbl[standby_time_idx];
    if (sb_th > 0 &&
        !key_state &&
        !auto_mode &&
        mode != MODE_PLAY &&
        mode != MODE_EDIT_SELECT &&
        mode != MODE_EDIT &&
        mode != MODE_SETUP &&
        (now - last_activity_tick) > sb_th)
    {

        standby_magic = STANDBY_MAGIC_VALUE;   // ※復帰フラグ
        enter_standby();
        // // OLED OFF
        // ssd1306_cmd(SSD1306_DISPLAYOFF);

        // wake_flag = false;

        // // ※WFI ループ（割り込みで wake_flag が立つ）
        // while (!wake_flag) {
        //     __WFI();
        // }

        // // 復帰処理
        // ssd1306_cmd(SSD1306_DISPLAYON);
        // ssd1306_refresh();

        // last_activity_tick = tim1_tick256;
    }

}

/* ===============================
 * PLAY / SAVE 仮実装 ダミー
 * =============================== */

void start_play(uint8_t msg)
{
    cur_msg = msg;
    last_activity_tick = tim1_tick256;

    auto_msg = msgs[msg]; // ※追加：生成する文字列を指定
    sys_msg_active = false;
    keyout_enabled = true;

    // 再生開始時に未確定データをクリアして、前回の残りで誤デコードされないようにする
    morse_len = 0;    // 未確定のモールス符号を破棄
    cw_r = cw_w;      // CWイベントバッファをクリア
    key_off_ticks = 0;
    key_on_ticks = 0;
    flush_done = true; // 無音タイムアウト処理を一時的に抑止

    auto_mode = true;
    req_reset_auto = true;
    mode = MODE_PLAY;
}

void stop_play(void)
{
    auto_mode = false;     // 自動送信OFF
    req_reset_auto = true; // job_auto の状態を初期化

    // パドル入力を無視（停止操作が音声にならないようにするため）
    ignore_paddle_input = true;

    // スイッチマスクをセット：停止操作の入力を無視！
    sw_mask = 1;

    // ===== 追加 =====
    morse_len = 0;         // →未確定文字を破棄

    // ※ここで「次の1回は必要な描き直し」とマーク
    last_wpm = -999;
    update_speed_from_adc();
    ssd1306_refresh();

}

//==========================================
//  main loop
//==========================================
int main()
{

    // 初期化（最初に実行する）
    SystemInit();
    // RCC->APB1PCENR |= RCC_APB1Periph_PWR; // PWR クロック有効化
    // bool from_standby = (PWR->CSR & PWR_CSR_SBF); // 復帰元がスタンバイかどうか
    // PWR->CTLR |= PWR_CTLR_CWUF | PWR_CTLR_CSBF; // 復帰フラグクリア
    ssd1306_i2c_init();
    ssd1306_init();
    //ssd1306_cmd(SSD1306_DISPLAYOFF);
    //__WFI(); // 割り込み取得
    GPIO_setup(); // gpio Setup;
    GPIO_ADCinit();

    // ssd1306_refreshはTIM1割り込みを一時無効化するため、スタンバイ復帰時のOLED再初期化を
    // tim1_int_init()より前に完了させる（復帰パドルの最初の長短点が正確になる）
    bool resumed_from_standby = (standby_magic == STANDBY_MAGIC_VALUE);
    if (resumed_from_standby) {
        standby_magic = 0;  // 次回のためにクリア
        update_battery_status();
        reset_decoded_display();
        ssd1306_cmd(SSD1306_DISPLAYOFF);
        ssd1306_init();
        for (int i = 0; i < 4; i++) {     // 複数回クリアして確実に初期化
            ssd1306_setbuf(0);
            ssd1306_refresh();
        }
        draw_keyer_screen();
        update_speed_from_adc();  // WPM描画・リフレッシュもTIM1開始前に完了
        ssd1306_refresh();
    }

    // タイマー開始前にkey_spdをADCから正しく設定する（通常起動時も含む）
    {
        int adc = GPIO_analogRead(GPIO_Ain0_A2);
        wpm = map(adc, 0, 1023, WPM_MAX, WPM_MIN);
        dit_est = (1200UL * 1000) / (wpm * 256);
        key_spd = dit_est;
    }
    init_flash_messages();
    tim1_int_init(); //
    // tim2_pwm_init();             // TIM2 PWM Setup

    if (!resumed_from_standby) {
        // 通常起動時はフラグを初期化
        standby_magic = 0;
        reset_decoded_display();
        Delay_Ms(1000);
        draw_startup_screen(); // スタート画面
        play_sys_msg("OK", 20);
        Delay_Ms(1000);
        draw_keyer_screen(); // キーヤー画面
        // ※起動直後に WPM を強制描画して refresh
        update_speed_from_adc();
        ssd1306_refresh();
    }

    // ループ処理
    while (1)
    {
        loop();
    }
}
