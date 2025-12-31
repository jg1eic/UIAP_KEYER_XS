//==================================================================//
// UIAP_keyer_for_ch32fun
// BASE Software from : https://www.gejigeji.com/?page_id=1045 
// Modified by Kimio Ohe JA9OIR/JA1AOQ
//	- port to ch32fun library
//==================================================================
#include "ch32v003fun.h"
#include "ch32v003_GPIO_branchless.h"
#include "keyer_hal.h"
#include <stdio.h>
#include <stdint.h>
#define SSD1306_128X64
#include "ssd1306_i2c.h"
#include "ssd1306.h"
#include "flash_eep.h"

/* ==== FLASH EEPROM ==== */
FLASH_EEP eep;

//==========================================
//常数・マクロ
//==========================================
//スクイズ状態（半ディット単位の状態機械）
#define SQZ_FREE 0
#define SQZ_SPC0 1
#define SQZ_SPC  2
#define SQZ_DOT0 3
#define SQZ_DOT  4
#define SQZ_DAH_CONT0 5
#define SQZ_DAH_CONT1 6
#define SQZ_DAH_CONT  7
#define SQZ_DASH 8

//パドル状態
#define PDL_DOT  1
#define PDL_DASH 2
#define PDL_FREE 0

#define SQUEEZE_TYPE 0  //スクイーズモード
#define PDL_RATIO 4     //短点・長点比率

#define WPM_MAX 40      //最大速度
#define WPM_MIN 5       //最小速度

//トーン設定
#define TONE_DIV 3            // 周波数調整
// 2-->976Hz
// 3-->651Hz
// 4-->488Hz

// スイッチ設定
#define SW_SCAN_DIV 20    // 0.256ms × 20 ≒ 5.12ms
#define SW_PRESS_TH     127      // 長押し判定（5ms × 128 = 0.64秒）      
#define SW_PUSH_TH      5        
#define SW_1 ( 1 << 3 )   
#define SW_2 ( 1 << 2 )
#define SW_3 ( 1 << 1 )
#define SW_4 ( 1 << 0 )
#define SW_INFO_CLICK        0x10
#define SW_INFO_PRESS        0x20
#define SW_INFO_DOUBLE       0x40
#define SW_CLEAR()       ( sw_mask = 0b00001111 )	// 一度スイッチを離すまでカウントをしない
#define MASK_MODE	0xf0

// EDIT操作のリピート設定（体感調整済み）
#define EDIT_REPEAT_START   15   // 15 × 10ms = 150ms
#define EDIT_REPEAT_SPEED   5    // 5 × 10ms = 50ms


// FLASH記録用
#define MSG_COUNT     2

#define PAGE_MSG1     0
#define PAGE_MSG2     1

//メッセージ用変数
#define MSG_NUM  2     // メモリ数（SW1 / SW2）
#define MSG_LEN  64   // 1メッセージの最大文字数
#define EDIT_TABLE_LEN (sizeof(edit_table) - 1)

//==========================================
//構造体定義
//==========================================

//モード定義
typedef enum {
    MODE_KEYER = 0,       // 通常キーイング
    MODE_PLAY,            // メモリ再生中
    MODE_EDIT_SELECT,     // 編集メモリ選択
    MODE_EDIT,            // 編集モード
    MODE_SETUP            // 設定（将来）
} keyer_mode_t;

//文字編集用
typedef enum {
    EDIT_CHAR_SELECT = 0,   // 文字選択中
    EDIT_POS_MOVE         // カーソル移動中（将来拡張）
} edit_state_t;

// static edit_state_t edit_state = EDIT_CHAR_SELECT;

//文字編集時にパドル検出用
typedef struct {
    bool dot;
    bool dash;
} paddle_release_t;

volatile paddle_release_t pad_rel = {0};

//==========================================
//グローバル変数
//==========================================

volatile uint8_t tone_div = 0; // トーン分周カウンタ
volatile bool tone_on = false;  // トーン出力中フラグ
volatile bool edit_tick_10ms = false;   // EDIT用10msタイマーフラグ

// char msgs[MSG_NUM][MSG_LEN+1] = {
//     "CQ TEST JO1YGK",   // メモリ1
//     "5NN 13M BK"    // メモリ2
// };
char msgs[MSG_NUM][MSG_LEN+1]; // メッセージバッファ

const char default_msgs[MSG_NUM][MSG_LEN] = {
    "CQ TEST JO1YGK",
    "5NN 13M BK"
}; // デフォルトメッセージ

static uint8_t cur_msg = 0;      // 編集中メモリ番号
static uint8_t edit_pos = 0;     // カーソル位置
// static uint8_t edit_char_index = 0;

static const char edit_table[] =
    " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/?.";

#define DISP_COLS 16   // 画面に表示する文字数

static uint8_t edit_view_left = 0;  // 表示ウィンドウ先頭
static uint8_t edit_len = 0;        // 現在の文字数
volatile bool edit_tick = false;

//extern char msgs[2][MSG_LEN];

// キーイング用変数
int  key_spd = 1000;
int  wpm = 20;
bool tone_enabled = false;
int squeeze = 0;
int paddle = PDL_FREE;
int paddle_old = PDL_FREE;

volatile uint8_t sw_div_cnt = 0;
volatile uint32_t tim1_tick256 = 0;

//編集用カウンタ
static keyer_mode_t mode = MODE_KEYER;

//編集時パドル状態記録
//static bool dot_prev  = false;
//static bool dash_prev = false;

// EDIT用パドル状態
static uint16_t edit_dot_cnt  = 0;
static uint16_t edit_dash_cnt = 0;
static uint8_t edit_first = 1;   // ★ 追加：EDIT初回フラグ

const char *mode_to_str(keyer_mode_t m)
{
    switch (m) {
        case MODE_KEYER:        return "KEYER";
        case MODE_PLAY:         return "PLAY";
        case MODE_EDIT_SELECT:  return "EDIT_SELECT";
        case MODE_EDIT:         return "EDIT";
        case MODE_SETUP:        return "SETUP";
        default:                return "UNKNOWN";
    }
}

/* ==== 割り込み → main loop 共有 ==== */
volatile bool in_dot  = false;
volatile bool in_dash = false;

// ==== 自動送信制御 ====
// 起動時は何もしない
volatile bool auto_mode  = false; // 今、自動送信中か
volatile bool auto_armed = false; // SWAを一度でも押したか（trueで “自動送信機能が有効”）
volatile bool req_start_auto = false;
volatile bool req_reset_auto = false;

// 停止理由（ラッチ停止）
typedef enum {
  STOP_NONE   = 0,
  STOP_PADDLE = 1, // DOT/DASHで停止（ラッチ）
  STOP_SWB    = 2, // SWBで停止（ラッチ）
} StopReason;

volatile StopReason stop_reason = STOP_NONE;

// スイッチの判定
uint8_t sw_mask = 0; // スイッチ押しっぱなしをカウントしないためのマスク
uint8_t sw_clicked = 0;
uint8_t count_sw[4]; // スイッチ長押しとかカウント
volatile uint8_t sw_stat;
volatile uint8_t sw_mode;

// ==== 汎用 CW メッセージ再生 ====
const char *auto_msg = NULL;   // 実際に再生する文字列
bool sys_msg_active = false;   // システムメッセージか？
//static uint8_t sys_msg_wpm = 20;
volatile bool keyout_enabled = true;   // 通常はON


//==========================================
//関数プロトタイプ
//==========================================
static const char* morseForChar(char c);
static void printAsc(int8_t asciinumber);
static void printAscii(int8_t c);
void dump_msgs(void);
uint8_t job_paddle(void);
uint8_t job_auto(void); 
void startTone(void); 
void stopTone(void); 
void update_speed_from_adc(void);
void start_play(uint8_t msg);
void stop_play(void);
void save_msgs(void);
void clear_sw_rel(void);
void edit_clear_after_cursor(void);
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
void draw_keyer_screen(void);
void draw_startup_screen(void);
void loop(void);
void TIM1_UP_IRQHandler(void);
void init_flash_messages(void);
void save_current_message_to_flash(void);
void draw_sys_message(const char *msg);

//==========================================
// Morse テーブル 
//==========================================
static const char* morseForChar(char c) {
  // 小文字 b は “BT”(-...-) 扱い（区切り用途）
  if (c == 'b') return "-...-";
	if (c == 'a') return ".-.-.";
  if (c == 'k') return "-.--.";
  if (c == 'v') return "...-.-";
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

  switch (c) {
    // Letters
    case 'A': return ".-";
    case 'B': return "-...";
    case 'C': return "-.-.";
    case 'D': return "-..";
    case 'E': return ".";
    case 'F': return "..-.";
    case 'G': return "--.";
    case 'H': return "....";
    case 'I': return "..";
    case 'J': return ".---";
    case 'K': return "-.-";
    case 'L': return ".-..";
    case 'M': return "--";
    case 'N': return "-.";
    case 'O': return "---";
    case 'P': return ".--.";
    case 'Q': return "--.-";
    case 'R': return ".-.";
    case 'S': return "...";
    case 'T': return "-";
    case 'U': return "..-";
    case 'V': return "...-";
    case 'W': return ".--";
    case 'X': return "-..-";
    case 'Y': return "-.--";
    case 'Z': return "--..";

    // Digits
    case '0': return "-----";
    case '1': return ".----";
    case '2': return "..---";
    case '3': return "...--";
    case '4': return "....-";
    case '5': return ".....";
    case '6': return "-....";
    case '7': return "--...";
    case '8': return "---..";
    case '9': return "----.";

    // Punctuation (必要そうなのだけ)
    case '.': return ".-.-.-";
    case ',': return "--..--";
    case '?': return "..--..";
    case '/': return "-..-.";
    case '=': return "-...-";
    case '+': return ".-.-.";
    case '-': return "-....-";
    case '@': return ".--.-.";

    default: return nullptr;
  }
}

#define FONT_WIDTH 12
#define FONT_COLOR 1
#define LINE_HEIGHT 16
#define FONT_SCALE_16X16 fontsize_16x16
const int colums = 10; /// have to be 16 or 20

int lcdindex = 0;
uint8_t line1[colums];
uint8_t line2[colums];
uint8_t lastChar = 0;

volatile uint8_t flg = 0;

//==========================================
//	printasc : print the ascii code to the lcd
//==========================================
static void printAsc(int8_t asciinumber)
{
	if (lcdindex > colums - 1){
		lcdindex = 0;
		for (int i = 0; i <= colums - 1 ; i++){
			ssd1306_drawchar_sz(i * FONT_WIDTH , LINE_HEIGHT, line2[i], FONT_COLOR, FONT_SCALE_16X16);
			line2[i] = line1[i];
		}
		for (int i = 0; i <= colums - 1 ; i++){
			ssd1306_drawchar_sz(i * FONT_WIDTH , LINE_HEIGHT * 2, line1[i], FONT_COLOR, FONT_SCALE_16X16);
			ssd1306_drawchar_sz(i * FONT_WIDTH , LINE_HEIGHT * 3, 32, FONT_COLOR, FONT_SCALE_16X16);
		}
 	}
	line1[lcdindex] = asciinumber;
	ssd1306_drawchar_sz(lcdindex * FONT_WIDTH , LINE_HEIGHT * 3, asciinumber, FONT_COLOR, FONT_SCALE_16X16);
    ssd1306_refresh();
	lcdindex += 1;
}

//==========================================
//	printascii : print the ascii code to the lcd
//==========================================
static void printAscii(int8_t c)
{
    switch (c) {
        case 'b': // BT
            printAsc('B');
            printAsc('T');
            break;
        case 'a':   // AR
            printAsc('A');
            printAsc('R');
            break;
        case 'k':   // KN
            printAsc('K');
            printAsc('N');
            break;
        case 'v':   // VA
            printAsc('V');
            printAsc('A');
            break;
        default:
            printAsc(c);
            break;
    }

        }

//トーン出力トグル
static inline void toggle_tone_pin(void)
{
    GPIOC->OUTDR ^= (1 << 7);  // PC7 の例
}

//==========================================
//メッセージ出力
//==========================================
void dump_msgs(void)
{
    for (int i = 0; i < MSG_NUM; i++) {
        printf("MSG%d: ", i + 1);
        for (int j = 0; j < MSG_LEN; j++) {
            char c = msgs[i][j];
            if (c == '\0') break;
            printf("%c", c);
        }
        printf("\r\n");
    }
}

//==========================================
// フラッシュメモリ読み出しの妥当性確認
//==========================================
static bool is_valid_message(const uint8_t *buf)
{
    bool has_null = false;

    for (int i = 0; i < MSG_LEN; i++) {
        uint8_t c = buf[i];

        if (c == 0xFF) break;      // 消去領域
        if (c == '\0') {
            has_null = true;
            break;
        }
        if (c < 0x20 || c > 0x7E)  // 非ASCII
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

    eep.begin(MSG_COUNT);

    for (int i = 0; i < MSG_COUNT; i++) {
        memset(buf, 0, sizeof(buf));
        eep.read(i, buf);

        if (!is_valid_message(buf)) {
            // Flashが空 or ゴミ → デフォルト投入
            strncpy(msgs[i], default_msgs[i], MSG_LEN - 1);
            msgs[i][MSG_LEN - 1] = '\0';
            printf("MSG%d: default\n", i + 1);
        } else {
            memcpy(msgs[i], buf, MSG_LEN);
            msgs[i][MSG_LEN - 1] = '\0';
            printf("MSG%d: loaded from flash\n", i + 1);
        }
    }
}



//==========================================
//フラッシュメモリ書き込み
//==========================================
void save_current_message_to_flash(void)
{
    uint8_t buf[FLASH_PAGE_SIZE];
    int page = cur_msg;   // MSG番号 = ページ番号

    memset(buf, 0xFF, sizeof(buf));
    strncpy((char *)buf, msgs[cur_msg], MSG_LEN);

    eep.erase(page);
    eep.write(page, buf);

    DEBUG_PRINTF("[FLASH] save MSG%d\n", cur_msg + 1);
}

//==========================================
//手動パドル処理（SWA/SWBは混ぜない） 
//==========================================
uint8_t job_paddle() {
  static uint32_t left_time = 0;
  uint8_t key_dot, key_dash;

  key_dot  = (!GPIO_digitalRead(PIN_DOT));
  key_dash = (!GPIO_digitalRead(PIN_DASH));

  if (left_time != 0) {
    left_time--;
  } else {
    left_time = key_spd / 2;
    if (squeeze != SQZ_FREE) squeeze--;
  }

  if (squeeze != SQZ_FREE) {
    if (paddle_old == PDL_DOT  && key_dash) paddle = PDL_DASH;
    else if (paddle_old == PDL_DASH && key_dot) paddle = PDL_DOT;
  }

  if (SQUEEZE_TYPE == 0) {
    if (squeeze > SQZ_DASH) paddle = PDL_FREE;
  } else {
    if (squeeze > SQZ_SPC)  paddle = PDL_FREE;
  }

  if (squeeze > SQZ_SPC) return 1;
  else if (squeeze == SQZ_SPC || squeeze == SQZ_SPC0) return 0;

  if (paddle == PDL_FREE) {
    if (key_dot) paddle = PDL_DOT;
    else if (key_dash) paddle = PDL_DASH;
  }

  if (paddle == PDL_FREE) return 0;

  if (paddle == PDL_DOT) squeeze = SQZ_DOT;
  else {
    uint8_t dash_len = (SQZ_SPC * PDL_RATIO + 5) / 2;
    squeeze = SQZ_SPC + dash_len;
  }

  left_time = key_spd / 2;
  paddle_old = paddle;
  paddle = PDL_FREE;
  return 1;
}

//==========================================  
//自動送信処理（半ディット単位） 
//========================================== 
uint8_t job_auto() 
{
    static uint32_t left_time = 0;
    static int auto_squeeze = 0;
    static int gap_half = 0;
    static uint8_t msg_i = 0;
    static uint16_t pos = 0;

    static const char* seq = nullptr;
    static uint8_t elem = 0;

    static bool pending_jump = false;
    static uint8_t pending_msg = 0;
    static uint16_t pending_pos = 0;
    static int pending_gap_half = 0;
    static bool pending_stop = false;

    if (req_reset_auto) {
        req_reset_auto = false;
        left_time = 0;
        auto_squeeze = 0;
        gap_half = 0;
        msg_i = cur_msg;
        pos = 0;
        seq = nullptr;
        elem = 0;
        pending_jump = false;
        pending_gap_half = 0;
    }

    if (left_time != 0) {
        left_time--;
    } else {
        left_time = key_spd / 2;
        if (auto_squeeze != 0) auto_squeeze--;
        else if (gap_half > 0) gap_half--;
    }

    uint8_t out = (auto_squeeze > SQZ_SPC) ? 1 : 0;
    if (auto_squeeze != 0 || gap_half > 0) return out;

    if (pending_jump) {

        // ===== メッセージ末尾での自動停止 =====
        /*if (pending_pos == 0 && pending_msg != msg_i) {
            auto_mode = false;
            req_reset_auto = true;
            pending_jump = false;
            return 0;
        }*/

        pending_jump = false;

        gap_half = pending_gap_half;
        pending_gap_half = 0;

        // ★ここを追加
        msg_i = pending_msg;
        pos   = pending_pos;

        seq = nullptr;
        elem = 0;

        if (pending_stop) {
            pending_stop = false;
            auto_mode = false;
            req_reset_auto = true;
        }

        return 0;
    }


    //const char* msg = msgs[msg_i];
    char c = auto_msg[pos];

    //char c = msg[pos];
    if (c == '\0') {
        //msg_i = (uint8_t)((msg_i + 1) % MSG_COUNT);
        //pos = 0;
        //gap_half = 12; // 単語間追加6dit
        //return 0;
        // ===== メッセージ終了 =====
        auto_mode = false;
        req_reset_auto = true;
        auto_msg = NULL;
        sys_msg_active = false;
        keyout_enabled = true;
        mode = MODE_KEYER;

        draw_keyer_screen();   // ★追加（画面復帰）
        return 0;
    }

    if (c == ' ') {
        while (auto_msg[pos] == ' ') pos++;
        gap_half = 12;
        return 0;
    }

    if (seq == nullptr) {
        if (!sys_msg_active) {     // ★追加
            printAscii(c);
        }
        seq = morseForChar(c);
        elem = 0;
        if (seq == nullptr) {
        pos++;
        gap_half = 12;
        return 0;
        }
    }

    char e = seq[elem];
    if (e == '\0') {
        seq = nullptr;
        elem = 0;
        pos++;
        gap_half = 4; // 文字間追加2dit
        return 0;
    }

    // 要素をスケジュール（要素+要素間1dit OFFを含む）
    if (e == '.') auto_squeeze = SQZ_DOT;   // 1dit ON + 1dit OFF
    else          auto_squeeze = SQZ_DASH;  // 3dit ON + 1dit OFF
    elem++;

    // 最後の要素なら、次の文字に応じた追加ギャップを予約
    if (seq[elem] == '\0') {
        uint16_t look = pos + 1;
/*
        if (msg[look] == ' ') {
            printAscii(32);
            while (msg[look] == ' ') look++;
            if (msg[look] == '\0') {
                pending_msg = (uint8_t)((msg_i + 1) % MSG_COUNT);
                pending_pos = 0;
            } else {
                pending_msg = msg_i;
                pending_pos = look;
            }
            pending_gap_half = 12;
            pending_jump = true;
*/
        if (auto_msg[look] == ' ') {
            // 連続スペース数を数える
            uint8_t nsp = 0;
            while (auto_msg[look] == ' ') { nsp++; look++; }

            // LCD表示もスペース数分進めたいならループ（重いので注意）
            // for (uint8_t i = 0; i < nsp; i++) printAscii(32);
            printAscii(32); // とりあえず1回だけ表示（表示は好みで）

            // 次の送信位置（スペースの次へ）
            if (auto_msg[look] == '\0') {
                pending_msg = (uint8_t)((msg_i + 1) % MSG_NUM);
                pending_pos = 0;
            } else {
                pending_msg = msg_i;
                pending_pos = look;
            }

            // ★スペース数ぶん待つ：1スペース=7dit
            // auto_squeezeに含まれる「最後の要素後の1dit OFF」を差し引いて -2(half-dit)
            pending_gap_half = (int)(14 * nsp - 2);   // nsp=1→12, nsp=2→26, nsp=3→40 ...
            pending_jump = true;
        } else if (auto_msg[look] == '\0') {
            //pending_msg = (uint8_t)((msg_i + 1) % MSG_COUNT);
            //pending_pos = 0;
            //pending_gap_half = 12;
            //pending_jump = true;
            // 最後の要素が鳴り終わってから停止させる
            pending_gap_half = 4;   // 最後の文字間
            pending_jump = true;
            pending_stop = true;    // ★ここだけ true
        } else {
        pending_msg = msg_i;
        pending_pos = look;
        pending_gap_half = 4;
        pending_jump = true;
        }
    }

    return out;
}

//==========================================
//トーン制御
//==========================================
void startTone() 
{
    tone_on = true;

    if (keyout_enabled) {
        GPIO_digitalWrite(PIN_KEYOUT, high);  // ★通常のみキーイング
    }
}

void stopTone() 
{
    tone_on = false;

    GPIO_digitalWrite(PIN_TONE, low);

    if (keyout_enabled) {
        GPIO_digitalWrite(PIN_KEYOUT, low);   // ★通常のみキーイング
    }
}

inline void keydown() { startTone(); }
inline void keyup()   { stopTone();  }

static inline long map(long x,
                              long in_min, long in_max,
                              long out_min, long out_max)
{
  // Arduino本家と同じくゼロ除算チェックはしない（in_max == in_min だと未定義）
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//==========================================
// ADCからスピード読み込み
//==========================================
void update_speed_from_adc() {
	int adc = GPIO_analogRead(GPIO_Ain0_A2);        // 0 .. 1023
	wpm = map(adc, 0, 1023, WPM_MIN, WPM_MAX);
	key_spd = 4687 / wpm;  // = (1200/wpm) /0.256  4687.5 -> 4687 
	//printf("WPM=%d, SPD=%d\r\n", wpm, key_spd);

    char buf[16];
    sprintf(buf, "%2d", wpm);
    ssd1306_drawstr_sz(96, 0, buf, 1, fontsize_8x8);
    ssd1306_refresh();

	Delay_Ms(10);
}

//==========================================
// スイッチ状態確認（各ハンドラから呼ぶ)
//==========================================
void update_switch_status(void){
 
    //スイッチ確認
    sw_stat = sw_get_info();
    sw_mode = sw_stat & MASK_MODE;
    sw_stat &= ~MASK_MODE;

    //デバッグ表示
    if (sw_stat) {
        printf("SW mode=%02X stat=%02X mask=%02X\n",
               sw_mode, sw_stat, sw_mask);
    }
}


/**
 * @brief  スイッチカウンタ
 */
uint8_t sw_chatter(uint8_t sw, uint8_t *counter) {
    uint8_t is_clicked = 0;

    if (sw) {
        //カウンタが255に達していなければ、カウンタを1増やす
        *counter += ((*counter != 255) ? 1 : 0); 
    } else {
        is_clicked = ((SW_PUSH_TH < *counter) && (*counter < SW_PRESS_TH)) ? 1 : 0;
        *counter = 0;
    }
    return ( is_clicked);
}

/**
 * @brief  スイッチチェック
 * タイマー割り込みで呼ばれる
 */
void sw_check() {
    uint8_t temp_pin;
    bool s1 = !GPIO_digitalRead(PIN_SW1);
    bool s2 = !GPIO_digitalRead(PIN_SW2);  
    temp_pin = (s1<<3) | (s2 << 2);
    if(sw_mask != 0) { //どれかが押されているとき
        count_sw[0] = count_sw[1] = count_sw[2] = count_sw[3] = 0;
        sw_clicked = 0;
        if ((temp_pin & 0x0F) == 0) {
            // 全部放したらマスクとってSW_MASK = 0;
            sw_mask = 0;
        }
    }

    sw_clicked |= (sw_chatter(temp_pin & SW_1, &count_sw[0])) ? SW_1 : 0;
    sw_clicked |= (sw_chatter(temp_pin & SW_2, &count_sw[1])) ? SW_2 : 0;
    sw_clicked |= (sw_chatter(temp_pin & SW_3, &count_sw[2])) ? SW_3 : 0;
    sw_clicked |= (sw_chatter(temp_pin & SW_4, &count_sw[3])) ? SW_4 : 0;

}

//****************************
// スイッチを操作したら非0
//  メモリ再生の停止用
// ****************************
uint8_t sw_is_pressed() {
    if (sw_mask != 0) {
        return ( 0);
    }

    // なんか押操作された？
    if ((count_sw[0] > SW_PUSH_TH)
            || (count_sw[1] > SW_PUSH_TH)
            || (count_sw[2] > SW_PUSH_TH)
            || (count_sw[3] > SW_PUSH_TH)
            ) {
        return (1);
    }
    return (0);
}


/*****************************************************************************
 スイッチ状態読み込み　(8ビットの情報を返す)
 xxxxdcba
            dcba sw[1,2,3,4] に対応
            xxxx フラグ
                0000 何もなし
                0001 クリック
                0010 長押し
                0100 ダブル押し
 ******************************************************************************/
uint8_t sw_get_info() {
    
    volatile uint8_t count = 0;

    if (sw_mask != 0) { 
    // 1bitでもスイッチマスクがかかっていたら、押されてないことにする
        return (0);
    }

    // 複数押されてる？
    if (count_sw[0] > SW_PUSH_TH) {
        count += 1;
        flg |= SW_1;
    }
    if (count_sw[1] > SW_PUSH_TH) {
        count += 1;
        flg |= SW_2;
    }
    if (count_sw[2] > SW_PUSH_TH) {
        count += 1;
        flg |= SW_3;
    }
    if (count_sw[3] > SW_PUSH_TH) {
        count += 1;
        flg |= SW_4;
    }
    if (count >= 2) {
        return ( SW_INFO_DOUBLE | flg);
    }

    flg = 0;
    // なんか長押しされた？
    if (count_sw[0] > SW_PRESS_TH) {
        flg |= SW_1;
    }
    if (count_sw[1] > SW_PRESS_TH) {
        flg |= SW_2;
    }
    if (count_sw[2] > SW_PRESS_TH) {
        flg |= SW_3;
    }
    if (count_sw[3] > SW_PRESS_TH) {
        flg |= SW_4;
    }
    if (flg != 0) {
        sw_mask = 1;
        return ( SW_INFO_PRESS | flg);
    }

    flg = 0;
    if (sw_clicked != 0) {
    // なんかクリックされてた？
        flg = SW_INFO_CLICK | sw_clicked;
        sw_clicked = 0;
    }
    return (flg);
}


//==========================================
// 編集時の次の文字
//==========================================
char next_char(char c)
{
    const char *p = strchr(edit_table, c);
    if (!p) return edit_table[0];
    p++;
    if (*p == '\0') p = edit_table;
    return *p;
}

//==========================================
// 編集時の前の文字
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
// 編集時にそれ以降の文字を削除
//==========================================
void edit_clear_after_cursor(void)
{
    for (uint8_t i = edit_pos; i < MSG_LEN; i++) {
        msgs[cur_msg][i] = '\0';
    }
    edit_len = edit_pos;
}

//==========================================
// 編集時の表示を調整
//==========================================
void adjust_edit_view(void)
{
    if (edit_pos < edit_view_left) {
        edit_view_left = edit_pos;
    }
    else if (edit_pos >= edit_view_left + DISP_COLS) {
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

    wpm = wpm_val;
    key_spd = 4687 / wpm;

    keyout_enabled = false;   // RFキーイングしない
    req_reset_auto = true;
    auto_mode = true;
    mode = MODE_PLAY;
    //draw_sys_message(msg);
}

void play_mem_msg(uint8_t n)
{
    auto_msg = msgs[n];
    sys_msg_active = false;

    keyout_enabled = true;    // 通常キーイング
    req_reset_auto = true;
    auto_mode = true;
    mode = MODE_PLAY;
}

//==========================================
//  KEY出力処理(割り込みから呼ぶこと)
//==========================================
void service_keyer(void)
{
    static bool key_on = false;
    bool on = false;

    if (mode == MODE_PLAY) {
        on = auto_mode ? job_auto() : false;
    }
    else if (mode == MODE_KEYER) {
        on = job_paddle();
    }

    if (on && !key_on) {
        keydown();
        key_on = true;
    }
    else if (!on && key_on) {
        keyup();
        key_on = false;
    }
}


//==========================================
//  #1 KEYERモード処理
//==========================================
void handle_keyer_mode(void)
{
    // if (sys_msg_active) {
    //     return; // システムメッセージ中は何もしない
    // }

    //スイッチ状態取得
    update_switch_status();
    
    // if(sw_stat != 0){
    //     printf("SW MODE=%02X STAT=%02X\r\n", sw_mode, sw_stat);
    // }

    /* 編集モードへ */
    if (sw_mode == SW_INFO_DOUBLE && sw_stat == (SW_1 | SW_2)) {
        SW_CLEAR();
        mode = MODE_EDIT_SELECT;
        printf("MODE_EDIT_SELECT\r\n");
        dump_msgs();
        draw_edit_select();
        //play_sys_msg("M", 20);
        return;
    }

    /* メモリ再生 */
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_1) {
        SW_CLEAR();
        printf("START PLAY MSG1\r\n");
        mode = MODE_PLAY;
        start_play(0);
    }

    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_2) {
        SW_CLEAR();
        printf("START PLAY MSG2\r\n");
        mode = MODE_PLAY;
        start_play(1);
    }
}

//==========================================
//  #2 PLAYモード（自動リピート防止）
//==========================================
void handle_play_mode(void)
{
    // if (sys_msg_active) {
    //     return; // システムメッセージ中は何もしない
    // }

        /* 何か操作したら止める */
    if (sw_is_pressed() || in_dot || in_dash) {
        SW_CLEAR();
        stop_play();
        printf("Interrupt Message\r\n");
        // sw_rel.sw1=false;
        // sw_rel.sw2=false;
        mode = MODE_KEYER;
        return;
    }

    if (!auto_mode) {
        printf("Finished Message\r\n");
        mode = MODE_KEYER;
    }
}

//==========================================
//  #3 編集モード処理（メモリ選択)
//==========================================
void handle_edit_select(void)
{

    //スイッチ状態取得
    update_switch_status();

    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_1) {
        SW_CLEAR();
        cur_msg = 0;
        edit_pos = 0;
        edit_view_left = 0;
        edit_len = strnlen(msgs[cur_msg], MSG_LEN);
        printf("EDIT MSG1\r\n");
        edit_first    = 1;   // ★ 重要
        mode = MODE_EDIT;
        draw_edit_screen();
        //play_sys_msg("E", 20);
        return;
    }
    
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_2) {
        SW_CLEAR();
        cur_msg = 1;
        edit_pos = 0;
        edit_view_left = 0;
        edit_len = strnlen(msgs[cur_msg], MSG_LEN);
        printf("EDIT MSG2\r\n");
        edit_first    = 1;   // ★ 重要
        mode = MODE_EDIT;
        draw_edit_screen();
        //play_sys_msg("I", 20);
        return;
    }

    if (sw_mode == SW_INFO_DOUBLE && sw_stat == (SW_1 | SW_2)) {
        SW_CLEAR();
        mode = MODE_KEYER;
         draw_keyer_screen();
        printf("BACK TO KEYER\r\n");
        //printf("MODE_EDIT_SELECT\r\n");
        //dump_msgs();
        return;
    }

}

//==========================================
//  #4 編集モード処理（文字選択)
//==========================================
void handle_edit_mode(void)
{
    // 編集初回描画
    if (edit_first) {
        draw_edit_screen();
        edit_first = 0;
    }

    // スイッチ状態取得
    update_switch_status();

    bool dot  = !GPIO_digitalRead(PIN_DOT);
    bool dash = !GPIO_digitalRead(PIN_DASH);

    /* ===== 10ms周期でのみ編集処理 ===== */
    if (edit_tick_10ms) {
        edit_tick_10ms = false;

        /* ---- DOT（戻る）---- */
        if (dot) {
            edit_dot_cnt++;
            if (edit_dot_cnt == EDIT_REPEAT_START ||
                (edit_dot_cnt > EDIT_REPEAT_START &&
                 (edit_dot_cnt - EDIT_REPEAT_START) % EDIT_REPEAT_SPEED == 0)) {

                msgs[cur_msg][edit_pos] =
                    prev_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                draw_edit_screen();
            }
        } else {
            edit_dot_cnt = 0;
        }

        /* ---- DASH（進む）---- */
        if (dash) {
            edit_dash_cnt++;
            if (edit_dash_cnt == EDIT_REPEAT_START ||
                (edit_dash_cnt > EDIT_REPEAT_START &&
                 (edit_dash_cnt - EDIT_REPEAT_START) % EDIT_REPEAT_SPEED == 0)) {

                msgs[cur_msg][edit_pos] =
                    next_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                draw_edit_screen();
            }
        } else {
            edit_dash_cnt = 0;
        }
    }

    /* ===== 以下は従来どおり（即時反応でOK） ===== */

    // EDIT終了（長押し）
    if (sw_mode == SW_INFO_PRESS && sw_stat == SW_2) {
        SW_CLEAR();
        save_current_message_to_flash();
        printf("Message recorded\r\n");
        mode = MODE_KEYER;
        draw_keyer_screen();
        //play_sys_msg("OK", 20);
        return;
    }

    // 以降削除
    if (sw_mode == SW_INFO_PRESS && sw_stat == SW_1) {
        SW_CLEAR();
        edit_clear_after_cursor();
        draw_edit_screen();
        //play_sys_msg("D", 20);
        return;
    }

    /* ---- 確定 ---- */
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_2) {
        SW_CLEAR();
        if (edit_pos < MSG_LEN - 1) {
            edit_pos++;
            if (edit_pos >= edit_len) {
                msgs[cur_msg][edit_pos] = ' ';
                edit_len = edit_pos + 1;
                msgs[cur_msg][edit_len] = '\0';
            }
            adjust_edit_view();
        }
        draw_edit_screen();
    }

    /* ---- カーソル戻し ---- */
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_1 && edit_pos > 0) {
        SW_CLEAR();
        edit_pos--;
        adjust_edit_view();
        draw_edit_screen();
    }
}


//==========================================
//  #6 設定モード
//==========================================
void handle_setup_mode(void)
{

}

//==========================================
//  #1 スタート画面
//==========================================
void draw_startup_screen(void)
{
    printf("UIAP KEYER start\r\n");
    ssd1306_drawstr_sz(0, 30,"UIAP KEYER V0.01", 1, fontsize_8x8);
    ssd1306_refresh();    
}

//==========================================
//  #2 キーヤー画面
//==========================================

void draw_keyer_screen(void)
{
    ssd1306_setbuf(0);    // 0=黒, 1=白
    ssd1306_drawstr_sz(0, 0, "KEYER", 1, fontsize_8x8);
    ssd1306_drawstr_sz(64, 0, "WPM:", 1, fontsize_8x8);
    ssd1306_drawFastHLine(0, 10, 128, 1);
    ssd1306_refresh();
}

//==========================================
//  #3 録音するメモリーを選択する画面
//==========================================
void draw_edit_select(void)
{
    ssd1306_setbuf(0);    // 0=黒, 1=白
    ssd1306_drawstr_sz(0, 0, "EDIT MSG", 1, fontsize_8x8);
    ssd1306_drawFastHLine(0, 10, 128, 1);
    ssd1306_drawstr_sz(0, 16, "SELECT MSG", 1, fontsize_8x8);
    ssd1306_drawstr_sz(0, 24, "SW1: Record MSG1", 1, fontsize_8x8);
    ssd1306_drawstr_sz(0, 32, "SW2: Record MSG2", 1, fontsize_8x8);
    ssd1306_drawstr_sz(0, 40, "SW1+SW2: KEYER", 1, fontsize_8x8);
    printf("Select Message\r\n");
    ssd1306_refresh();
}

//==========================================
//  #4 メモリー編集
//==========================================
void draw_edit_screen(void)
{
    char buf[32];
    char disp[DISP_COLS + 1];

    ssd1306_setbuf(0);

    /* ===== タイトル ===== */
    sprintf(buf, "EDIT MSG%d", cur_msg + 1);
    ssd1306_drawstr_sz(0, 0, buf, 1, fontsize_8x8);

    /* ===== カーソル位置表示 (XX/63) ===== */
    sprintf(buf, "%02d/%d", edit_pos + 1, MSG_LEN-1);
    ssd1306_drawstr_sz(80, 0, buf, 1, fontsize_8x8);

    ssd1306_drawFastHLine(0, 10, 128, 1);

    /* ===== メッセージ表示（16文字ウィンドウ）===== */
    for (uint8_t i = 0; i < DISP_COLS; i++) {
        uint8_t idx = edit_view_left + i;
        disp[i] = (idx < edit_len) ? msgs[cur_msg][idx] : ' ';
    }
    disp[DISP_COLS] = '\0';

    ssd1306_drawstr_sz(0, 14, disp, 1, fontsize_8x8);

    /* ===== カーソル ===== */
    #define EDIT_FONT_W 8
    int cursor_col = edit_pos - edit_view_left;
    if (cursor_col < 0) cursor_col = 0;
    if (cursor_col >= DISP_COLS) cursor_col = DISP_COLS - 1;

    int x = cursor_col * EDIT_FONT_W;
    ssd1306_drawchar_sz(x, 24, '^', 1, fontsize_8x8);

    /* ===== 操作説明 ===== */
    ssd1306_drawstr_sz(0, 36, "SW1:<-,SW2:->", 1, fontsize_8x8);
    ssd1306_drawstr_sz(0, 48, "SW1(>1s):DEL", 1, fontsize_8x8);
    ssd1306_drawstr_sz(0, 56, "SW2(>1s):END", 1, fontsize_8x8);
    ssd1306_refresh();
}

//==========================================
//  #5 システムメッセージ表示
//==========================================
void draw_sys_message(const char *msg)
{
    ssd1306_setbuf(0);                    // ← これだけでOK
    ssd1306_drawstr_sz(0, 16, (char *)msg, 1, fontsize_16x16);
    ssd1306_refresh();
}


//==========================================
//  タイマー割り込み
//==========================================
void TIM1_UP_IRQHandler(void)
{
    /* ==== 割り込みフラグクリア ==== */
    TIM1->INTFR &= (uint16_t)~TIM_IT_Update;

    /* ==== キーイング処理 ==== */
    service_keyer();

    /* ==== スイッチ処理用分周 ==== */
    if (++sw_div_cnt >= SW_SCAN_DIV) {
        sw_div_cnt = 0;
        // call sw_check at configured interval
        sw_check();
    }   

    // increment IRQ tick (approx 256us per interrupt)
    tim1_tick256++;

    //トーン制御
    if (tone_on) {
        if (++tone_div >= TONE_DIV) {
            tone_div = 0;
            toggle_tone_pin();
        }
    } else {
        GPIO_digitalWrite(PIN_TONE, low);
    } 

}

//==========================================
//  ループ処理
//==========================================
void loop(void)
{
    static uint16_t sec_cnt  = 0;
    static uint32_t last_tick = 0;

    /* ==== 10ms周期 (IRQ基準: 256us × 40 ≒ 10ms) ==== */
    if ((tim1_tick256 - last_tick) >= 40) {
        last_tick += 40; // advance by 40 ticks
        edit_tick_10ms = true;

        /* ==== EDIT中はWPM更新しない ==== */
        if (mode == MODE_KEYER || mode == MODE_PLAY) {
            update_speed_from_adc();
        }

        /* ==== 1秒周期 ==== */
        if (++sec_cnt >= 100) {   // 100 × 10ms = 1s
            sec_cnt = 0;
            printf("[MODE] %s\r\n", mode_to_str(mode));
        }
    }

     /* ==== モード処理 ==== */
    switch (mode) {
        case MODE_KEYER:    handle_keyer_mode();    break;
        case MODE_PLAY:     handle_play_mode();     break;
        case MODE_EDIT_SELECT:  handle_edit_select();   break;
        case MODE_EDIT:     handle_edit_mode();     break;
        case MODE_SETUP:    handle_setup_mode();    break;
    }
}


/* ===============================
 * PLAY / SAVE 仮実装（ダミー）
 * =============================== */

void start_play(uint8_t msg)
{
    cur_msg = msg;

    auto_msg = msgs[msg];   // ★ 追加：再生する文字列を指定
    sys_msg_active = false;
    keyout_enabled = true;

    auto_mode = true;
    req_reset_auto = true;
    mode = MODE_PLAY;
}


void stop_play(void)
{
    auto_mode = false;      // 自動送信OFF
    req_reset_auto = true;  // job_auto 内部状態を初期化
}

void save_msgs(void)
{
    // TODO: EEPROM / Flash 保存
}

//==========================================
//  main loop
//==========================================
int main()
{
    //初期化（最初に実行する)
    SystemInit();
  	ssd1306_i2c_init();
	ssd1306_init();
  	GPIO_setup();				// gpio Setup;    
	GPIO_ADCinit();
	tim1_int_init();			//
    init_flash_messages();
	//tim2_pwm_init();             // TIM2 PWM Setup

	Delay_Ms(1000);
    draw_startup_screen();      //スタート画面
    play_sys_msg("OK", 20);
    dump_msgs();
    Delay_Ms(1000);
    draw_keyer_screen();        //キーヤー画面

    //ループ処理
    while (1) {
        loop();
    }
}

