//
// ch32v003 flash eeprom emulation
// 2025/12/20  v0.1  new JA1AOQ
// 2025/12/21  v0.2  API変更: アドレス指定 -> ページ指定 (0..15)
// 
// API ドキュメント
// - コンストラクタ: FLASH_EEP(uint32_t pages = (FLASH_SIZE / FLASH_PAGE_SIZE))
//   - pages: 使用するページ数（64B ページ単位）。指定可能範囲: 1 .. 16
//   - 内部計算: flash_size = pages * 64, flash_base_addr = FLASH_END_ADDR - flash_size + 1
// - メソッド:
//   - int erase(uint32_t page);
//   - int read(uint32_t page, uint8_t *out);
//   - int write(uint32_t page, const uint8_t *in);
//   - page は 0 から (pages-1) の範囲で指定。範囲外の場合は -2 を返す。
// - エラーコード:
//   - -1 : アドレス整列（64B 境界）エラー（主に内部関数で発生）
//   - -2 : 無効なページ／範囲エラー
// - 使用例:
//   FLASH_EEP eep(2); // 2 ページ (128B) を EEPROM として使用
//   uint8_t buf[FLASH_PAGE_SIZE];
//   eep.erase(0);             // ページ 0 を消去
//   eep.write(0, data);       // ページ 0 に書き込み
//   eep.read(0, buf);         // ページ 0 を読み出し

#pragma once
#include <stdint.h>
#include <stdio.h>

/* Debug print macro: define DEBUG to enable debug prints (e.g. -DDEBUG) */
#ifndef DEBUG_PRINTF
#ifdef DEBUG
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) do {} while (0)
#endif
#endif

/* Control inclusion of sample/demo code (flash_test.cpp). Set to 0 to exclude sample code from build to save flash. */
#ifndef SAMPLE_CODE
#define SAMPLE_CODE 1
#endif

#define FLASH_PAGE_SIZE    64u
#define FLASH_BASE_ADDR    0x08003C00u
#define FLASH_SIZE         1024u
#define FLASH_END_ADDR     0x08003FFFu   // CH32V003 16KB

class FLASH_EEP {
public:
    FLASH_EEP();                                    // デフォルトコンストラクタ（軽量）
    int begin(uint32_t pages);                      // 実際の初期化: pages (1..16)
    bool is_initialized() const { return flash_page_count != 0; }

    int erase(uint32_t page);
    int read(uint32_t page, uint8_t *out);
    int write(uint32_t page, const uint8_t *in);
    void dump_state();
private:
    int write_buf(uint32_t addr, const uint8_t *buf);
    int commit(uint32_t addr);
    uint32_t flash_base_addr = 0;   // 初期化前は 0
    uint32_t flash_size = 0;        // 初期化前は 0
    uint32_t flash_page_count = 0;  // 初期化前は 0
    inline int is_64b_aligned(uint32_t a)
    {
        return (a & 0x3F) == 0;
    }

    inline int page_valid(uint32_t p)
    {
        /*
         * Validate that a page number fits within configured page count
         * and within the allowed range 0..15 (max 16 pages)
         */
        if (p > 15u) return 0;
        return (p < flash_page_count);
    }

    inline void flash_wait_busy(void)
    {
        while (FLASH->STATR & FLASH_STATR_BSY) {}
    }

    inline void flash_clear_flags(void)
    {
        // WCH系は STATR に 1 書きでクリア（まとめてクリア）
        FLASH->STATR = 0xFFFFFFFF;
    }

    inline void flash_unlock_all(void)
    {
        // 通常Flash unlock
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
        // Fast erase / page program unlock
        FLASH->MODEKEYR = FLASH_KEY1;
        FLASH->MODEKEYR = FLASH_KEY2;
    }
};
