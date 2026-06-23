/**
 * @file mapper009.cpp
 * @brief MMC2 implementation with eager loading for LRU mode
 * @version 2.2
 */

#include "mapper009.h"
#include "../cartridge.h"
#include <Arduino.h>
#include <cstring>

// Enable verbose logging for debugging
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

    // Active pointers (current banks)
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
    return (state->backend == ROMBackend::FLASH)
        ? getPRGBankFLASH(state, bankIndex)
        : getPRGBankLRU(state, bankIndex);
}

static inline uint8_t* getCHRBank(Mapper009_state* state, uint8_t bankIndex) {
    return (state->backend == ROMBackend::FLASH)
        ? getCHRBankFLASH(state, bankIndex)
        : getCHRBankLRU(state, bankIndex);
}

// -----------------------------------------------------------------------------
// Latch update (MMC2 hardware)
// -----------------------------------------------------------------------------

static inline void mapper009_updateLatch(Mapper009_state* state, uint16_t addr) {
    bool updated = false;
    if (addr >= 0x0FD0 && addr <= 0x0FDF) {
        if (state->latch0 != 0xFD) {
            state->latch0 = 0xFD;
            state->ptr_CHR_bank_4K_low = getCHRBank(state, state->chr_bank0_fd);
            updated = true;
            MAPPER_LOG("LATCH: low = FD (addr 0x%04X)\n", addr);
        }
    } else if (addr >= 0x0FE0 && addr <= 0x0FEF) {
        if (state->latch0 != 0xFE) {
            state->latch0 = 0xFE;
            state->ptr_CHR_bank_4K_low = getCHRBank(state, state->chr_bank0_fe);
            updated = true;
            MAPPER_LOG("LATCH: low = FE (addr 0x%04X)\n", addr);
        }
    } else if (addr >= 0x1FD0 && addr <= 0x1FDF) {
        if (state->latch1 != 0xFD) {
            state->latch1 = 0xFD;
            state->ptr_CHR_bank_4K_high = getCHRBank(state, state->chr_bank1_fd);
            updated = true;
            MAPPER_LOG("LATCH: high = FD (addr 0x%04X)\n", addr);
        }
    } else if (addr >= 0x1FE0 && addr <= 0x1FEF) {
        if (state->latch1 != 0xFE) {
            state->latch1 = 0xFE;
            state->ptr_CHR_bank_4K_high = getCHRBank(state, state->chr_bank1_fe);
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

    int bank_idx = (addr - 0x8000) >> 13;
    data = state->ptr_PRG_bank_8K[bank_idx][addr & 0x1FFF];
    return true;
}

bool mapper009_cpuWrite(Mapper* mapper, uint16_t addr, uint8_t data) {
    if (addr < 0x8000) return false;
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return true;

    MAPPER_LOG("CPU WRITE: addr=0x%04X, data=0x%02X\n", addr, data);

    if (addr >= 0xA000 && addr < 0xB000) {
        state->prg_bank = data & 0x0F;
        state->ptr_PRG_bank_8K[0] = getPRGBank(state, state->prg_bank);
        MAPPER_LOG("PRG bank set to %d\n", state->prg_bank);
    }
    else if (addr >= 0xB000 && addr < 0xC000) {
        state->chr_bank0_fd = data & 0x1F;
        if (state->latch0 == 0xFD) {
            state->ptr_CHR_bank_4K_low = getCHRBank(state, state->chr_bank0_fd);
        }
        MAPPER_LOG("CHR low FD bank = %d\n", state->chr_bank0_fd);
    }
    else if (addr >= 0xC000 && addr < 0xD000) {
        state->chr_bank0_fe = data & 0x1F;
        if (state->latch0 == 0xFE) {
            state->ptr_CHR_bank_4K_low = getCHRBank(state, state->chr_bank0_fe);
        }
        MAPPER_LOG("CHR low FE bank = %d\n", state->chr_bank0_fe);
    }
    else if (addr >= 0xD000 && addr < 0xE000) {
        state->chr_bank1_fd = data & 0x1F;
        if (state->latch1 == 0xFD) {
            state->ptr_CHR_bank_4K_high = getCHRBank(state, state->chr_bank1_fd);
        }
        MAPPER_LOG("CHR high FD bank = %d\n", state->chr_bank1_fd);
    }
    else if (addr >= 0xE000 && addr < 0xF000) {
        state->chr_bank1_fe = data & 0x1F;
        if (state->latch1 == 0xFE) {
            state->ptr_CHR_bank_4K_high = getCHRBank(state, state->chr_bank1_fe);
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
// PPU Read (with auto-banking)
// -----------------------------------------------------------------------------

static inline void mapper009_autoBank(Mapper009_state* state, bool lowBank, uint8_t data) {
    if (data == 0xFD || data == 0xFE) {
        if (lowBank) {
            if (data == 0xFD) {
                state->latch0 = 0xFD;
                state->ptr_CHR_bank_4K_low = getCHRBank(state, state->chr_bank0_fd);
            } else {
                state->latch0 = 0xFE;
                state->ptr_CHR_bank_4K_low = getCHRBank(state, state->chr_bank0_fe);
            }
            MAPPER_LOG("AUTO-BANK low: %s -> bank %d\n", data==0xFD?"FD":"FE",
                       data==0xFD? state->chr_bank0_fd : state->chr_bank0_fe);
        } else {
            if (data == 0xFD) {
                state->latch1 = 0xFD;
                state->ptr_CHR_bank_4K_high = getCHRBank(state, state->chr_bank1_fd);
            } else {
                state->latch1 = 0xFE;
                state->ptr_CHR_bank_4K_high = getCHRBank(state, state->chr_bank1_fe);
            }
            MAPPER_LOG("AUTO-BANK high: %s -> bank %d\n", data==0xFD?"FD":"FE",
                       data==0xFD? state->chr_bank1_fd : state->chr_bank1_fe);
        }
    }
}

bool mapper009_ppuRead(Mapper* mapper, uint16_t addr, uint8_t& data) {
    if (addr > 0x1FFF) return false;

    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) { data = 0; return true; }

    mapper009_updateLatch(state, addr);

    bool lowBank = (addr < 0x1000);
    uint8_t* bank = lowBank ? state->ptr_CHR_bank_4K_low : state->ptr_CHR_bank_4K_high;
    data = bank[addr & 0x0FFF];

    mapper009_autoBank(state, lowBank, data);
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

    mapper009_updateLatch(state, addr);

    bool lowBank = (addr < 0x1000);
    uint8_t* bank = lowBank ? state->ptr_CHR_bank_4K_low : state->ptr_CHR_bank_4K_high;
    if (!bank) return nullptr;

    // Read data for auto-banking (same as ppuRead)
    uint8_t data = bank[addr & 0x0FFF];
    mapper009_autoBank(state, lowBank, data);

    return &bank[addr & 0x0FFF];
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

    // Eager load all banks
    state->ptr_PRG_bank_8K[0] = getPRGBank(state, state->prg_bank);
    state->ptr_PRG_bank_8K[1] = getPRGBank(state, state->number_PRG_banks - 3);
    state->ptr_PRG_bank_8K[2] = getPRGBank(state, state->number_PRG_banks - 2);
    state->ptr_PRG_bank_8K[3] = getPRGBank(state, state->number_PRG_banks - 1);

    state->ptr_CHR_bank_4K_low = getCHRBank(state, state->chr_bank0_fe);
    state->ptr_CHR_bank_4K_high = getCHRBank(state, state->chr_bank1_fe);

    state->cart->setMirrorMode(Cartridge::VERTICAL);
    MAPPER_LOG("MMC2 RESET: eager loading, prg_banks %d, chr_banks %d\n",
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

    // Invalidate LRU cache and eager reload
    if (state->backend == ROMBackend::LRU) {
        invalidateCache(&state->PRG_cache_8K);
        invalidateCache(&state->CHR_cache_4K);
    }

    // Eager load all banks from restored registers
    state->ptr_PRG_bank_8K[0] = getPRGBank(state, state->prg_bank);
    state->ptr_PRG_bank_8K[1] = getPRGBank(state, state->number_PRG_banks - 3);
    state->ptr_PRG_bank_8K[2] = getPRGBank(state, state->number_PRG_banks - 2);
    state->ptr_PRG_bank_8K[3] = getPRGBank(state, state->number_PRG_banks - 1);

    state->ptr_CHR_bank_4K_low = getCHRBank(state,
        (state->latch0 == 0xFD) ? state->chr_bank0_fd : state->chr_bank0_fe);
    state->ptr_CHR_bank_4K_high = getCHRBank(state,
        (state->latch1 == 0xFD) ? state->chr_bank1_fd : state->chr_bank1_fe);

    MAPPER_LOG("State loaded (cache invalidated, eager reload)\n");
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
