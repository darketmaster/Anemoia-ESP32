/**
 * @file mapper009.cpp
 * @brief MMC2 implementation with FLASH direct pointers and LRU on-demand access
 * @version 2.3 - Fixed LRU stale pointer issue
 */

#include "mapper009.h"
#include "../cartridge.h"
#include <Arduino.h>
#include <cstring>

// Enable verbose logging for debugging (set to 0 for production)
#define LOG_MAPPER009 0

#if LOG_MAPPER009
    #define MAPPER_LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
    #define MAPPER_LOG(fmt, ...)
#endif

struct Mapper009_state {
    Cartridge* cart = nullptr;
    MappedROM* mROM = nullptr;
    ROMBackend backend;

    uint8_t number_PRG_banks;   // 8KB banks
    uint8_t number_CHR_banks;   // 4KB banks

    // Active pointers (only used in FLASH mode for speed)
    // In LRU mode these are not used; instead we call getPRGBank/getCHRBank each time.
    uint8_t* ptr_PRG_bank_8K[4];    // $8000, $A000, $C000, $E000
    uint8_t* ptr_CHR_bank_4K_low;   // $0000–$0FFF
    uint8_t* ptr_CHR_bank_4K_high;  // $1000–$1FFF

    // LRU cache structures
    Bank PRG_banks_8K[MAPPER009_MAX_CACHED_PRG_BANKS];
    Bank CHR_banks_4K[MAPPER009_MAX_CACHED_CHR_BANKS];
    BankCache PRG_cache_8K;
    BankCache CHR_cache_4K;

    // MMC2 registers
    uint8_t prg_bank;
    uint8_t chr_bank0_fd;
    uint8_t chr_bank0_fe;
    uint8_t chr_bank1_fd;
    uint8_t chr_bank1_fe;
    uint8_t latch0;
    uint8_t latch1;
};

// -----------------------------------------------------------------------------
// Bank access helpers (FLASH vs LRU)
// -----------------------------------------------------------------------------

static inline uint8_t* getPRGBankFLASH(Mapper009_state* state, uint8_t index) {
    if (index >= state->number_PRG_banks) return nullptr;
    return (uint8_t*)(state->mROM->prg_base + (index * 8192U));
}

static inline uint8_t* getCHRBankFLASH(Mapper009_state* state, uint8_t bankIndex) {
    if (bankIndex >= state->number_CHR_banks) return nullptr;
    return (uint8_t*)(state->mROM->chr_base + (bankIndex * 4096U));
}

static inline uint8_t* getPRGBankLRU(Mapper009_state* state, uint8_t bankIndex) {
    if (bankIndex >= state->number_PRG_banks) return nullptr;
    uint8_t* ptr = getBank(&state->PRG_cache_8K, bankIndex, RomType::PRG);
    MAPPER_LOG("LRU PRG: requested bank %d, got ptr %p\n", bankIndex, ptr);
    return ptr;
}

static inline uint8_t* getCHRBankLRU(Mapper009_state* state, uint8_t bankIndex) {
    if (bankIndex >= state->number_CHR_banks) return nullptr;
    uint8_t* ptr = getBank(&state->CHR_cache_4K, bankIndex, RomType::CHR);
    MAPPER_LOG("LRU CHR: requested bank %d, got ptr %p\n", bankIndex, ptr);
    return ptr;
}

static inline uint8_t* getPRGBank(Mapper009_state* state, uint8_t bankIndex) {
    if (state->backend == ROMBackend::FLASH)
        return getPRGBankFLASH(state, bankIndex);
    else
        return getPRGBankLRU(state, bankIndex);
}

static inline uint8_t* getCHRBank(Mapper009_state* state, uint8_t bankIndex) {
    if (state->backend == ROMBackend::FLASH)
        return getCHRBankFLASH(state, bankIndex);
    else
        return getCHRBankLRU(state, bankIndex);
}

// -----------------------------------------------------------------------------
// Latch update (MMC2 hardware)
// -----------------------------------------------------------------------------

static inline void mapper009_updateLatch(Mapper009_state* state, uint16_t addr) {
    bool updated = false;
    // Latch 0 (low CHR) – ranges according to MMC2 spec
    if (addr >= 0x0FD0 && addr <= 0x0FDF) {
        if (state->latch0 != 0xFD) {
            state->latch0 = 0xFD;
            if (state->backend == ROMBackend::FLASH) {
                state->ptr_CHR_bank_4K_low = getCHRBankFLASH(state, state->chr_bank0_fd);
            }
            // In LRU we don't cache the pointer; it will be resolved on each read.
            updated = true;
            MAPPER_LOG("LATCH: low = FD (addr 0x%04X)\n", addr);
        }
    } else if (addr >= 0x0FE0 && addr <= 0x0FEF) {
        if (state->latch0 != 0xFE) {
            state->latch0 = 0xFE;
            if (state->backend == ROMBackend::FLASH) {
                state->ptr_CHR_bank_4K_low = getCHRBankFLASH(state, state->chr_bank0_fe);
            }
            updated = true;
            MAPPER_LOG("LATCH: low = FE (addr 0x%04X)\n", addr);
        }
    }
    // Latch 1 (high CHR)
    else if (addr >= 0x1FD0 && addr <= 0x1FDF) {
        if (state->latch1 != 0xFD) {
            state->latch1 = 0xFD;
            if (state->backend == ROMBackend::FLASH) {
                state->ptr_CHR_bank_4K_high = getCHRBankFLASH(state, state->chr_bank1_fd);
            }
            updated = true;
            MAPPER_LOG("LATCH: high = FD (addr 0x%04X)\n", addr);
        }
    } else if (addr >= 0x1FE0 && addr <= 0x1FEF) {
        if (state->latch1 != 0xFE) {
            state->latch1 = 0xFE;
            if (state->backend == ROMBackend::FLASH) {
                state->ptr_CHR_bank_4K_high = getCHRBankFLASH(state, state->chr_bank1_fe);
            }
            updated = true;
            MAPPER_LOG("LATCH: high = FE (addr 0x%04X)\n", addr);
        }
    }
    if (updated) {
        MAPPER_LOG("  -> low ptr=%p, high ptr=%p\n", state->ptr_CHR_bank_4K_low, state->ptr_CHR_bank_4K_high);
    }
}

// -----------------------------------------------------------------------------
// CPU Read/Write
// -----------------------------------------------------------------------------

bool mapper009_cpuRead(Mapper* mapper, uint16_t addr, uint8_t& data) {
    if (addr < 0x8000) return false;

    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) { data = 0xFF; return true; }

    uint8_t* bank_ptr = nullptr;
    uint8_t bank_index;

    if (addr < 0xA000) {
        // $8000–$9FFF: switchable bank
        bank_index = state->prg_bank;
    } else if (addr < 0xC000) {
        // $A000–$BFFF: fixed bank (last - 3)
        bank_index = state->number_PRG_banks - 3;
    } else if (addr < 0xE000) {
        // $C000–$DFFF: fixed bank (last - 2)
        bank_index = state->number_PRG_banks - 2;
    } else {
        // $E000–$FFFF: fixed bank (last - 1)
        bank_index = state->number_PRG_banks - 1;
    }

    // In FLASH mode we use cached pointers for speed; in LRU we resolve each time
    if (state->backend == ROMBackend::FLASH) {
        int slot = (addr - 0x8000) >> 13;
        bank_ptr = state->ptr_PRG_bank_8K[slot];
    } else {
        bank_ptr = getPRGBankLRU(state, bank_index);
    }

    data = bank_ptr ? bank_ptr[addr & 0x1FFF] : 0xFF;
    return true;
}

bool mapper009_cpuWrite(Mapper* mapper, uint16_t addr, uint8_t data) {
    if (addr < 0x8000) return false;
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return true;

    MAPPER_LOG("CPU WRITE: addr=0x%04X, data=0x%02X\n", addr, data);

    if (addr >= 0xA000 && addr < 0xB000) {
        state->prg_bank = data & 0x0F;
        if (state->backend == ROMBackend::FLASH) {
            state->ptr_PRG_bank_8K[0] = getPRGBankFLASH(state, state->prg_bank);
        }
        MAPPER_LOG("PRG bank set to %d\n", state->prg_bank);
    }
    else if (addr >= 0xB000 && addr < 0xC000) {
        state->chr_bank0_fd = data & 0x1F;
        if (state->latch0 == 0xFD) {
            if (state->backend == ROMBackend::FLASH) {
                state->ptr_CHR_bank_4K_low = getCHRBankFLASH(state, state->chr_bank0_fd);
            }
        }
        MAPPER_LOG("CHR low FD bank = %d\n", state->chr_bank0_fd);
    }
    else if (addr >= 0xC000 && addr < 0xD000) {
        state->chr_bank0_fe = data & 0x1F;
        if (state->latch0 == 0xFE) {
            if (state->backend == ROMBackend::FLASH) {
                state->ptr_CHR_bank_4K_low = getCHRBankFLASH(state, state->chr_bank0_fe);
            }
        }
        MAPPER_LOG("CHR low FE bank = %d\n", state->chr_bank0_fe);
    }
    else if (addr >= 0xD000 && addr < 0xE000) {
        state->chr_bank1_fd = data & 0x1F;
        if (state->latch1 == 0xFD) {
            if (state->backend == ROMBackend::FLASH) {
                state->ptr_CHR_bank_4K_high = getCHRBankFLASH(state, state->chr_bank1_fd);
            }
        }
        MAPPER_LOG("CHR high FD bank = %d\n", state->chr_bank1_fd);
    }
    else if (addr >= 0xE000 && addr < 0xF000) {
        state->chr_bank1_fe = data & 0x1F;
        if (state->latch1 == 0xFE) {
            if (state->backend == ROMBackend::FLASH) {
                state->ptr_CHR_bank_4K_high = getCHRBankFLASH(state, state->chr_bank1_fe);
            }
        }
        MAPPER_LOG("CHR high FE bank = %d\n", state->chr_bank1_fe);
    }
    else if (addr >= 0xF000) {
        Cartridge::MIRROR mode = (data & 1) ? Cartridge::HORIZONTAL : Cartridge::VERTICAL;
        state->cart->setMirrorMode(mode);
        MAPPER_LOG("Mirroring set to %s\n", mode == Cartridge::HORIZONTAL ? "HORIZONTAL" : "VERTICAL");
    }
    return true;
}

// -----------------------------------------------------------------------------
// PPU Read (with auto-banking removed, only address-based latching)
// -----------------------------------------------------------------------------

bool mapper009_ppuRead(Mapper* mapper, uint16_t addr, uint8_t& data) {
    if (addr > 0x1FFF) return false;

    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) { data = 0; return true; }

    // Update latch based on PPU address (this is the hardware behavior)
    mapper009_updateLatch(state, addr);

    bool lowBank = (addr < 0x1000);
    uint8_t* bank_ptr = nullptr;

    if (state->backend == ROMBackend::FLASH) {
        // Use cached pointers (fast)
        bank_ptr = lowBank ? state->ptr_CHR_bank_4K_low : state->ptr_CHR_bank_4K_high;
    } else {
        // In LRU mode, resolve the bank each time based on current latch and register values
        uint8_t bank_index;
        if (lowBank) {
            bank_index = (state->latch0 == 0xFD) ? state->chr_bank0_fd : state->chr_bank0_fe;
        } else {
            bank_index = (state->latch1 == 0xFD) ? state->chr_bank1_fd : state->chr_bank1_fe;
        }
        bank_ptr = getCHRBankLRU(state, bank_index);
        MAPPER_LOG("PPU read: addr=0x%04X, latch=%s, bank=%d, ptr=%p\n",
                   addr, lowBank ? (state->latch0==0xFD?"FD":"FE") : (state->latch1==0xFD?"FD":"FE"),
                   bank_index, bank_ptr);
    }

    data = bank_ptr ? bank_ptr[addr & 0x0FFF] : 0;
    return true;
}

bool mapper009_ppuWrite(Mapper* mapper, uint16_t addr, uint8_t data) {
    (void)addr; (void)data;
    return false;
}

uint8_t* mapper009_ppuReadPtr(Mapper* mapper, uint16_t addr) {
    if (addr > 0x1FFF) return nullptr;
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return nullptr;

    // Update latch based on PPU address
    mapper009_updateLatch(state, addr);

    bool lowBank = (addr < 0x1000);
    uint8_t* bank_ptr = nullptr;

    if (state->backend == ROMBackend::FLASH) {
        bank_ptr = lowBank ? state->ptr_CHR_bank_4K_low : state->ptr_CHR_bank_4K_high;
    } else {
        // Resolve bank each time for LRU
        uint8_t bank_index;
        if (lowBank) {
            bank_index = (state->latch0 == 0xFD) ? state->chr_bank0_fd : state->chr_bank0_fe;
        } else {
            bank_index = (state->latch1 == 0xFD) ? state->chr_bank1_fd : state->chr_bank1_fe;
        }
        bank_ptr = getCHRBankLRU(state, bank_index);
        MAPPER_LOG("PPU ptr: addr=0x%04X, bank=%d, ptr=%p\n", addr, bank_index, bank_ptr);
    }

    return bank_ptr ? &bank_ptr[addr & 0x0FFF] : nullptr;
}

// -----------------------------------------------------------------------------
// Reset and state management
// -----------------------------------------------------------------------------

void mapper009_reset(Mapper* mapper) {
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return;

    state->latch0 = 0xFE;
    state->latch1 = 0xFE;
    state->chr_bank0_fd = 0;
    state->chr_bank0_fe = 1;
    state->chr_bank1_fd = 2;
    state->chr_bank1_fe = 3;
    state->prg_bank = 0;

    if (state->backend == ROMBackend::FLASH) {
        // Eager load and cache pointers (fast)
        state->ptr_PRG_bank_8K[0] = getPRGBankFLASH(state, state->prg_bank);
        state->ptr_PRG_bank_8K[1] = getPRGBankFLASH(state, state->number_PRG_banks - 3);
        state->ptr_PRG_bank_8K[2] = getPRGBankFLASH(state, state->number_PRG_banks - 2);
        state->ptr_PRG_bank_8K[3] = getPRGBankFLASH(state, state->number_PRG_banks - 1);
        state->ptr_CHR_bank_4K_low  = getCHRBankFLASH(state, state->chr_bank0_fe);
        state->ptr_CHR_bank_4K_high = getCHRBankFLASH(state, state->chr_bank1_fe);
        MAPPER_LOG("MMC2 RESET: FLASH mode, banks loaded\n");
    } else {
        // LRU mode: no pointer caching; invalidate caches and resolve on each access.
        // Clear pointers to avoid accidental use.
        for (int i = 0; i < 4; i++) state->ptr_PRG_bank_8K[i] = nullptr;
        state->ptr_CHR_bank_4K_low = nullptr;
        state->ptr_CHR_bank_4K_high = nullptr;
        // Optionally invalidate caches to start fresh
        invalidateCache(&state->PRG_cache_8K);
        invalidateCache(&state->CHR_cache_4K);
        MAPPER_LOG("MMC2 RESET: LRU mode, caches invalidated\n");
    }

    state->cart->setMirrorMode(Cartridge::VERTICAL);
    MAPPER_LOG("MMC2 RESET: prg_banks %d, chr_banks %d\n",
               state->number_PRG_banks, state->number_CHR_banks);
}

void mapper009_dumpState(Mapper* mapper, File& stateFile) {
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return;
    stateFile.write((uint8_t*)&state->latch0, sizeof(state->latch0));
    stateFile.write((uint8_t*)&state->latch1, sizeof(state->latch1));
    stateFile.write((uint8_t*)&state->chr_bank0_fd, sizeof(state->chr_bank0_fd));
    stateFile.write((uint8_t*)&state->chr_bank0_fe, sizeof(state->chr_bank0_fe));
    stateFile.write((uint8_t*)&state->chr_bank1_fd, sizeof(state->chr_bank1_fd));
    stateFile.write((uint8_t*)&state->chr_bank1_fe, sizeof(state->chr_bank1_fe));
    stateFile.write((uint8_t*)&state->prg_bank, sizeof(state->prg_bank));
    MAPPER_LOG("State dumped\n");
}

void mapper009_loadState(Mapper* mapper, File& stateFile) {
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return;
    stateFile.read((uint8_t*)&state->latch0, sizeof(state->latch0));
    stateFile.read((uint8_t*)&state->latch1, sizeof(state->latch1));
    stateFile.read((uint8_t*)&state->chr_bank0_fd, sizeof(state->chr_bank0_fd));
    stateFile.read((uint8_t*)&state->chr_bank0_fe, sizeof(state->chr_bank0_fe));
    stateFile.read((uint8_t*)&state->chr_bank1_fd, sizeof(state->chr_bank1_fd));
    stateFile.read((uint8_t*)&state->chr_bank1_fe, sizeof(state->chr_bank1_fe));
    stateFile.read((uint8_t*)&state->prg_bank, sizeof(state->prg_bank));

    // Invalidate LRU caches and reinitialize for current state
    if (state->backend == ROMBackend::LRU) {
        invalidateCache(&state->PRG_cache_8K);
        invalidateCache(&state->CHR_cache_4K);
        // Clear cached pointers (they'll be resolved on next access)
        for (int i = 0; i < 4; i++) state->ptr_PRG_bank_8K[i] = nullptr;
        state->ptr_CHR_bank_4K_low = nullptr;
        state->ptr_CHR_bank_4K_high = nullptr;
    } else {
        // FLASH: re-eager load all banks
        state->ptr_PRG_bank_8K[0] = getPRGBankFLASH(state, state->prg_bank);
        state->ptr_PRG_bank_8K[1] = getPRGBankFLASH(state, state->number_PRG_banks - 3);
        state->ptr_PRG_bank_8K[2] = getPRGBankFLASH(state, state->number_PRG_banks - 2);
        state->ptr_PRG_bank_8K[3] = getPRGBankFLASH(state, state->number_PRG_banks - 1);
        state->ptr_CHR_bank_4K_low = getCHRBankFLASH(state,
            (state->latch0 == 0xFD) ? state->chr_bank0_fd : state->chr_bank0_fe);
        state->ptr_CHR_bank_4K_high = getCHRBankFLASH(state,
            (state->latch1 == 0xFD) ? state->chr_bank1_fd : state->chr_bank1_fe);
    }

    MAPPER_LOG("State loaded (cache invalidated for LRU, re-loaded for FLASH)\n");
}

// -----------------------------------------------------------------------------
// Factory
// -----------------------------------------------------------------------------

Mapper createMapper009(uint8_t PRG_banks, uint8_t CHR_banks, ROMBackend backend, Cartridge* cart) {
    Mapper mapper;
    Mapper009_state* state = new Mapper009_state();
    memset(state, 0, sizeof(Mapper009_state));

    state->backend = backend;
    state->number_PRG_banks = PRG_banks * 2;
    state->number_CHR_banks = CHR_banks * 2;
    state->cart = cart;

    if (backend == ROMBackend::FLASH) {
        state->mROM = &cart->mROM;
        MAPPER_LOG("MMC2: FLASH mode, PRG=%d, CHR=%d\n", state->number_PRG_banks, state->number_CHR_banks);
    } else {
        bankInit(&state->PRG_cache_8K, state->PRG_banks_8K, MAPPER009_MAX_CACHED_PRG_BANKS, 8192, cart);
        bankInit(&state->CHR_cache_4K, state->CHR_banks_4K, MAPPER009_MAX_CACHED_CHR_BANKS, 4096, cart);
        MAPPER_LOG("MMC2: LRU mode, PRG=%d, CHR=%d, cache PRG=%d, CHR=%d\n",
                   state->number_PRG_banks, state->number_CHR_banks,
                   MAPPER009_MAX_CACHED_PRG_BANKS, MAPPER009_MAX_CACHED_CHR_BANKS);
    }

    mapper.state = state;
    return mapper;
}