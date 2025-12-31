//
// ch32v003 flash eeprom emulation
// 2025/12/20  v0.1  new JA1AOQ
//  
#include <stdio.h>
#include <stdint.h>

#include "ch32fun.h"
#include "ch32v003hw.h"
#include "flash_eep.h"

/*
 * 前提：
 *  - 64B erase : FLASH_CTLR_PAGE_ER (0x00020000)
 *  - 64B page program : FLASH_CTLR_PAGE_PG (0x00010000)
 *  - start : FLASH_CTLR_STRT (0x00000040)
 *  - buffer load/reset 使用
 */

/* ============================================================
 * low-level helpers
 * ============================================================ */

/* ============================================================
 * methods
 * ============================================================ */
FLASH_EEP::FLASH_EEP()
{
    // Lightweight ctor: don't touch hardware or compute base/size here.
    flash_base_addr = 0;
    flash_size = 0;
    flash_page_count = 0;
}

int FLASH_EEP::begin(uint32_t pages)
{
    // Validate page count (1 .. 16)
    const uint32_t max_pages = (FLASH_SIZE / FLASH_PAGE_SIZE);
    if (pages < 1u || pages > max_pages) {
        DEBUG_PRINTF("FLASH_EEP::begin invalid pages=%lu, clamping to %lu\n", (unsigned long)pages, (unsigned long)max_pages);
        pages = max_pages;
    }
    flash_page_count = pages;
    flash_size = pages * FLASH_PAGE_SIZE;
    // calculate base address so region ends at FLASH_END_ADDR (inclusive)
    flash_base_addr = FLASH_END_ADDR - flash_size + 1u;

    flash_unlock_all();
    DEBUG_PRINTF("FLASH_EEP::begin this=0x%08lx base=0x%08lx size=%lu pages=%lu\n",
           (unsigned long)(uintptr_t)this, (unsigned long)flash_base_addr, (unsigned long)flash_size, (unsigned long)flash_page_count);
    return 0;
}

int FLASH_EEP::erase(uint32_t page)
{
    if (!page_valid(page)) return -2;
    uint32_t addr = flash_base_addr + (page * FLASH_PAGE_SIZE);

    flash_unlock_all();
    flash_wait_busy();
    flash_clear_flags();

    FLASH->CTLR = FLASH_CTLR_PAGE_ER;   // 64B erase ONLY
    FLASH->ADDR = addr;
    FLASH->CTLR |= FLASH_CTLR_STRT;

    flash_wait_busy();
    flash_clear_flags();

    FLASH->CTLR = 0;    // 後処理（重要）
    return 0;
} 

int FLASH_EEP::read(uint32_t page, uint8_t *out)
{
    if (!page_valid(page)) {
        DEBUG_PRINTF("read: invalid page=%lu base=0x%08lx size=%lu end=0x%08lx\n",
               (unsigned long)page, (unsigned long)flash_base_addr, (unsigned long)flash_size, (unsigned long)FLASH_END_ADDR);
        return -2;
    }
    uint32_t addr = flash_base_addr + (page * FLASH_PAGE_SIZE);

    memcpy(out, (const void *)(uintptr_t)addr, FLASH_PAGE_SIZE);
    return 0;
} 

int FLASH_EEP::write_buf(uint32_t addr, const uint8_t *buf)
{
    if (!is_64b_aligned(addr)) return -1;
    if (addr < flash_base_addr) return -2;
    uint32_t last_valid_start = (flash_size < FLASH_PAGE_SIZE) ? 0 : (flash_base_addr + flash_size - FLASH_PAGE_SIZE);
    if (addr > last_valid_start) return -2;

    flash_unlock_all();
    flash_wait_busy();
    flash_clear_flags();

    // page program + buffer reset
    FLASH->CTLR = FLASH_CTLR_PAGE_PG;
    FLASH->CTLR = FLASH_CTLR_PAGE_PG | FLASH_CTLR_BUF_RST;
    FLASH->ADDR = addr;

    flash_wait_busy();

    volatile uint32_t *dst = (volatile uint32_t *)(uintptr_t)addr;
    uint32_t *in = (uint32_t *)buf;

    for (int i = 0; i <  (int)(FLASH_PAGE_SIZE / 4); i++) {
        dst[i] = in[i];
        FLASH->CTLR = FLASH_CTLR_PAGE_PG | FLASH_CTLR_BUF_LOAD;
        flash_wait_busy();
    }
    return 0;
}

int FLASH_EEP::commit(uint32_t addr)
{
    if (!is_64b_aligned(addr)) return -1;
    if (addr < flash_base_addr) return -2;
    uint32_t last_valid_start = (flash_size < FLASH_PAGE_SIZE) ? 0 : (flash_base_addr + flash_size - FLASH_PAGE_SIZE);
    if (addr > last_valid_start) return -2;

    flash_unlock_all();
    flash_wait_busy();
    flash_clear_flags();

    FLASH->ADDR = addr;
    FLASH->CTLR = FLASH_CTLR_PAGE_PG | FLASH_CTLR_STRT;

    flash_wait_busy();
    flash_clear_flags();

    FLASH->CTLR = 0;
    return 0;
} 

void FLASH_EEP::dump_state()
{
    DEBUG_PRINTF("dump_state this=0x%08lx base=0x%08lx size=%lu pages=%lu\n",
           (unsigned long)(uintptr_t)this, (unsigned long)flash_base_addr, (unsigned long)flash_size, (unsigned long)flash_page_count);
}

int FLASH_EEP::write(uint32_t page, const uint8_t *in)
{
    if (!page_valid(page)) return -2;
    uint32_t addr = flash_base_addr + (page * FLASH_PAGE_SIZE);

    int r = erase(page);
    if (r) return r;

    r = write_buf(addr, in);
    if (r) return r;

    return commit(addr);
} 

