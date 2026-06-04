// mapper009.cpp

#include "mapper009.h"
#include "../cartridge.h"
#include <Arduino.h>
#include <cstring>

// =====================================
// Logging
// =====================================

#define LOG_MAPPER009 1

#if LOG_MAPPER009
    #define MAPPER_LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
    #define MAPPER_LOG(fmt, ...)
#endif

// =====================================
// CHR helpers
// =====================================

static inline uint8_t* getCHRBankFLASH(Mapper009_state* state, uint8_t bankIndex)
{
    if (bankIndex >= state->number_CHR_banks) { return nullptr; }
    return (uint8_t*)(state->mROM->chr_base + (bankIndex * 4096U));
}

static inline uint8_t* getCHRBankLRU(Mapper009_state* state, uint8_t bankIndex, bool lowBank)
{
    if (bankIndex >= state->number_CHR_banks) { return nullptr; }
    BankCache* cache = lowBank ? &state->CHR_cache_low : &state->CHR_cache_high;
    return getBank(cache, bankIndex, RomType::CHR);
}

// =====================================
// Unified CHR getter
// =====================================

static inline uint8_t* getCHRBank(Mapper009_state* state, uint8_t bankIndex, bool lowBank)
{
    if (state->backend == ROMBackend::FLASH)
        return getCHRBankFLASH(state, bankIndex);
    else
        return getCHRBankLRU(state, bankIndex, lowBank);
}

// =====================================
// PRG loader
// =====================================

static void loadPRGBank8K(Mapper009_state* state, uint8_t** bankPtr, uint8_t bankIndex,
                          const char* name)
{
    if (bankIndex >= state->number_PRG_banks)
    {
        MAPPER_LOG("MMC2 invalid PRG bank %d\n", bankIndex);
        return;
    }

    if (state->backend == ROMBackend::FLASH)
    {
        *bankPtr = (uint8_t*)(state->mROM->prg_base + (bankIndex * 8192U));
        // MAPPER_LOG("MMC2 FLASH PRG %s -> bank %d ptr=%p\n", name, bankIndex, *bankPtr);
    }
    else
    {
        if (!(*bankPtr))
        {
            *bankPtr = (uint8_t*)malloc(8192);
            if (!(*bankPtr))
            {
                MAPPER_LOG("MMC2 malloc failed for %s\n", name);
                return;
            }
        }
        state->cart->loadPRGBank(*bankPtr, 8192, bankIndex * 8192U);
        MAPPER_LOG("MMC2 LRU PRG %s -> bank %d\n", name, bankIndex);
    }
}

// =====================================
// MMC2 latch update (por dirección PPU)
// =====================================

static inline void mapper009_updateLatch(Mapper009_state* state, uint16_t addr)
{
    // LOW CHR (sprites) - rangos completos según documentación
    if (addr >= 0x0FD0 && addr <= 0x0FDF)
    {
        state->latch0 = 0xFD;
        uint8_t* bank = getCHRBank(state, state->chr_bank0_fd, true);
        if (bank)
        {
            state->ptr_CHR_bank_4K_low = bank;
            //MAPPER_LOG("MMC2 latch0 FD by addr 0x%04X bank=%d\n", addr, state->chr_bank0_fd);
        }
        else
        {
            //MAPPER_LOG("MMC2 latch0 FD: getCHRBank returned NULL for bank %d\n", state->chr_bank0_fd);
        }
    }
    else if (addr >= 0x0FE0 && addr <= 0x0FEF)
    {
        state->latch0 = 0xFE;
        uint8_t* bank = getCHRBank(state, state->chr_bank0_fe, true);
        if (bank)
        {
            state->ptr_CHR_bank_4K_low = bank;
            //MAPPER_LOG("MMC2 latch0 FE by addr 0x%04X bank=%d\n", addr, state->chr_bank0_fe);
        }
        else
        {
            //MAPPER_LOG("MMC2 latch0 FE: getCHRBank returned NULL for bank %d\n", state->chr_bank0_fe);
        }
    }
    // HIGH CHR (background)
    else if (addr >= 0x1FD0 && addr <= 0x1FDF)
    {
        state->latch1 = 0xFD;
        uint8_t* bank = getCHRBank(state, state->chr_bank1_fd, false);
        if (bank)
        {
            state->ptr_CHR_bank_4K_high = bank;
            //MAPPER_LOG("MMC2 latch1 FD by addr 0x%04X bank=%d\n", addr, state->chr_bank1_fd);
        }
        else
        {
            //MAPPER_LOG("MMC2 latch1 FD: getCHRBank returned NULL for bank %d\n", state->chr_bank1_fd);
        }
    }
    else if (addr >= 0x1FE0 && addr <= 0x1FEF)
    {
        state->latch1 = 0xFE;
        uint8_t* bank = getCHRBank(state, state->chr_bank1_fe, false);
        if (bank)
        {
            state->ptr_CHR_bank_4K_high = bank;
            //MAPPER_LOG("MMC2 latch1 FE by addr 0x%04X bank=%d\n", addr, state->chr_bank1_fe);
        }
        else
        {
            //MAPPER_LOG("MMC2 latch1 FE: getCHRBank returned NULL for bank %d\n", state->chr_bank1_fe);
        }
    }
}

// =====================================
// CPU READ
// =====================================

bool mapper009_cpuRead(Mapper* mapper, uint16_t addr, uint8_t& data)
{
    if (addr < 0x8000) return false;

    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state)
    {
        data = 0xFF;
        return true;
    }

    if (addr < 0xA000)
        data = state->PRG_bank_8000[addr & 0x1FFF];
    else if (addr < 0xC000)
        data = state->PRG_bank_A000[addr & 0x1FFF];
    else if (addr < 0xE000)
        data = state->PRG_bank_C000[addr & 0x1FFF];
    else
        data = state->PRG_bank_E000[addr & 0x1FFF];

    return true;
}

// =====================================
// CPU WRITE
// =====================================

bool mapper009_cpuWrite(Mapper* mapper, uint16_t addr, uint8_t data)
{
    if (addr < 0x8000) return false;
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return true;

    // Log de escrituras a registros CHR/PRG
    // if (addr >= 0xA000)
    // {
    //     MAPPER_LOG("CHR reg: addr=0x%04X, data=0x%02X, bank=%d, latch=%s\n", addr, data, data & 0x1F,
    //        (addr >= 0xB000 && addr < 0xC000) ? "LOW FD" :
    //        (addr >= 0xC000 && addr < 0xD000) ? "LOW FE" :
    //        (addr >= 0xD000 && addr < 0xE000) ? "HIGH FD" :
    //        (addr >= 0xE000 && addr < 0xF000) ? "HIGH FE" : "PRG/MIRROR");
    // }

    // =================================
    // $A000-$AFFF : PRG BANK
    // =================================
    if (addr >= 0xA000 && addr < 0xB000)
    {
        uint8_t bank = data & 0x0F;
        loadPRGBank8K(state, &state->PRG_bank_8000, bank, "$8000");
    }

    // =================================
    // $B000-$BFFF : LOW FD
    // =================================
    else if (addr >= 0xB000 && addr < 0xC000)
    {
        state->chr_bank0_fd = data & 0x1F;
        if (state->latch0 == 0xFD)
        {
            uint8_t* bank = getCHRBank(state, state->chr_bank0_fd, true);
            if (bank) state->ptr_CHR_bank_4K_low = bank;
        }
    }

    // =================================
    // $C000-$CFFF : LOW FE
    // =================================
    else if (addr >= 0xC000 && addr < 0xD000)
    {
        state->chr_bank0_fe = data & 0x1F;
        if (state->latch0 == 0xFE)
        {
            uint8_t* bank = getCHRBank(state, state->chr_bank0_fe, true);
            if (bank) state->ptr_CHR_bank_4K_low = bank;
        }
    }

    // =================================
    // $D000-$DFFF : HIGH FD
    // =================================
    else if (addr >= 0xD000 && addr < 0xE000)
    {
        state->chr_bank1_fd = data & 0x1F;
        if (state->latch1 == 0xFD)
        {
            uint8_t* bank = getCHRBank(state, state->chr_bank1_fd, false);
            if (bank) state->ptr_CHR_bank_4K_high = bank;
        }
    }

    // =================================
    // $E000-$EFFF : HIGH FE
    // =================================
    else if (addr >= 0xE000 && addr < 0xF000)
    {
        state->chr_bank1_fe = data & 0x1F;
        if (state->latch1 == 0xFE)
        {
            uint8_t* bank = getCHRBank(state, state->chr_bank1_fe, false);
            if (bank) state->ptr_CHR_bank_4K_high = bank;
        }
    }

    // =================================
    // $F000-$FFFF : MIRRORING
    // =================================
    else if (addr >= 0xF000)
    {
        state->cart->setMirrorMode((data & 1) ? Cartridge::HORIZONTAL : Cartridge::VERTICAL);
    }

    return true;
}

// =====================================
// PPU READ (con auto-banking por dirección y por valor)
// =====================================

bool mapper009_ppuRead(Mapper* mapper, uint16_t addr, uint8_t& data)
{
    if (addr > 0x1FFF) return false;

    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) { data = 0; return true; }

    // 1. Actualizar latches basado en la dirección PPU
    mapper009_updateLatch(state, addr);

    // 2. Obtener el banco actual
    uint8_t* bank = (addr < 0x1000) ? state->ptr_CHR_bank_4K_low : state->ptr_CHR_bank_4K_high;
    if (!bank) { data = 0; return true; }

    // 3. Leer el dato
    data = bank[addr & 0x0FFF];

    // 4. Auto-banking por valor (si el tile leído es 0xFD o 0xFE)
    if (data == 0xFD || data == 0xFE)
    {
        bool isLowPattern = (addr < 0x1000);
        uint8_t newBank = 0;

        if (isLowPattern)
        {
            if (data == 0xFD)
            {
                state->latch0 = 0xFD;
                newBank = state->chr_bank0_fd;
            }
            else
            {
                state->latch0 = 0xFE;
                newBank = state->chr_bank0_fe;
            }
            uint8_t* newBankPtr = getCHRBank(state, newBank, true);
            if (newBankPtr)
            {
                state->ptr_CHR_bank_4K_low = newBankPtr;
                MAPPER_LOG("MMC2 auto-bank LOW by value 0x%02X -> bank %d\n", data, newBank);
            }
            else
            {
                MAPPER_LOG("MMC2 auto-bank LOW: getCHRBank NULL for bank %d\n", newBank);
            }
        }
        else
        {
            if (data == 0xFD)
            {
                state->latch1 = 0xFD;
                newBank = state->chr_bank1_fd;
            }
            else
            {
                state->latch1 = 0xFE;
                newBank = state->chr_bank1_fe;
            }
            uint8_t* newBankPtr = getCHRBank(state, newBank, false);
            if (newBankPtr)
            {
                state->ptr_CHR_bank_4K_high = newBankPtr;
                MAPPER_LOG("MMC2 auto-bank HIGH by value 0x%02X -> bank %d\n", data, newBank);
            }
            else
            {
                MAPPER_LOG("MMC2 auto-bank HIGH: getCHRBank NULL for bank %d\n", newBank);
            }
        }
    }

    return true;
}

// =====================================
// PPU WRITE (no usado)
// =====================================

bool mapper009_ppuWrite(Mapper* mapper, uint16_t addr, uint8_t data)
{
    return false;
}

// =====================================
// PPU READ PTR
// =====================================

uint8_t* mapper009_ppuReadPtr(Mapper* mapper, uint16_t addr)
{
    if (addr > 0x1FFF) return nullptr;

    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return nullptr;

    mapper009_updateLatch(state, addr);

    uint8_t* bank = (addr < 0x1000) ? state->ptr_CHR_bank_4K_low : state->ptr_CHR_bank_4K_high;
    if (!bank) return nullptr;

    return &bank[addr & 0x0FFF];
}

// =====================================
// RESET
// =====================================

void mapper009_reset(Mapper* mapper)
{
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return;

    memset(state->dummy_chr, 0, sizeof(state->dummy_chr));

    // =================================
    // Initial latches (según documentación = 0xFE)
    // =================================
    state->latch0 = 0xFE;
    state->latch1 = 0xFE;

    // =================================
    // Initial CHR banks (valores por defecto)
    // =================================
    state->chr_bank0_fd = 0;
    state->chr_bank0_fe = 1;
    state->chr_bank1_fd = 2;
    state->chr_bank1_fe = 3;

    // =================================
    // PRG setup: $8000=0, $A000=-3, $C000=-2, $E000=-1
    // =================================
    loadPRGBank8K(state, &state->PRG_bank_8000, 0, "$8000");
    loadPRGBank8K(state, &state->PRG_bank_A000, state->number_PRG_banks - 3, "$A000");
    loadPRGBank8K(state, &state->PRG_bank_C000, state->number_PRG_banks - 2, "$C000");
    loadPRGBank8K(state, &state->PRG_bank_E000, state->number_PRG_banks - 1, "$E000");

    // Mostrar código en dirección de reset
    if (state->PRG_bank_E000)
    {
        uint16_t reset_addr = 0xFEF0;
        MAPPER_LOG("Código en 0x%04X: ", reset_addr);
        for (int i = 0; i < 16; i++)
        {
            MAPPER_LOG("%02X ", state->PRG_bank_E000[(reset_addr & 0x1FFF) + i]);
        }
        MAPPER_LOG("\n");
    }

    MAPPER_LOG("CHR banks: %d, chr_base=%p\n", state->number_CHR_banks, state->mROM->chr_base);
    if (state->mROM->chr_base)
    {
        uint8_t* chr0 = (uint8_t*)state->mROM->chr_base;
        MAPPER_LOG("CHR0 primeros 16 bytes: ");
        for (int i = 0; i < 16; i++) MAPPER_LOG("%02X ", chr0[i]);
        MAPPER_LOG("\n");
    }

    // === LOG ADICIONAL: contenido del banco CHR 4 (usado en latch FE) ===
    uint8_t* bank4 = getCHRBank(state, 4, false);
    if (bank4)
    {
        MAPPER_LOG("Banco CHR 4 primeros 16 bytes: ");
        for (int i = 0; i < 16; i++) MAPPER_LOG("%02X ", bank4[i]);
        MAPPER_LOG("\n");
    }
    else
    {
        MAPPER_LOG("Banco CHR 4 es NULL (no se pudo obtener)\n");
    }

    // =================================
    // CHR initial (usar los valores por defecto, banco 0 y 2)
    // =================================
    state->ptr_CHR_bank_4K_low = getCHRBank(state, state->chr_bank0_fd, true);
    state->ptr_CHR_bank_4K_high = getCHRBank(state, state->chr_bank1_fd, false);

    if (!state->ptr_CHR_bank_4K_low) state->ptr_CHR_bank_4K_low = state->dummy_chr;
    if (!state->ptr_CHR_bank_4K_high) state->ptr_CHR_bank_4K_high = state->dummy_chr;

    // =================================
    // Mirroring: vertical por defecto
    // =================================
    state->cart->setMirrorMode(Cartridge::VERTICAL);

    // =================================
    // Debug reset vector
    // =================================
    uint8_t lo = state->PRG_bank_E000[0x1FFC];
    uint8_t hi = state->PRG_bank_E000[0x1FFD];
    uint16_t resetVector = lo | (hi << 8);
    MAPPER_LOG("MMC2 RESET VECTOR = %04X\n", resetVector);
}

// =====================================
// SAVE STATE
// =====================================

void mapper009_dumpState(Mapper* mapper, File& stateFile)
{
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    stateFile.write((uint8_t*)&state->latch0, sizeof(state->latch0));
    stateFile.write((uint8_t*)&state->latch1, sizeof(state->latch1));
    stateFile.write((uint8_t*)&state->chr_bank0_fd, sizeof(state->chr_bank0_fd));
    stateFile.write((uint8_t*)&state->chr_bank0_fe, sizeof(state->chr_bank0_fe));
    stateFile.write((uint8_t*)&state->chr_bank1_fd, sizeof(state->chr_bank1_fd));
    stateFile.write((uint8_t*)&state->chr_bank1_fe, sizeof(state->chr_bank1_fe));
}

void mapper009_loadState(Mapper* mapper, File& stateFile)
{
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    stateFile.read((uint8_t*)&state->latch0, sizeof(state->latch0));
    stateFile.read((uint8_t*)&state->latch1, sizeof(state->latch1));
    stateFile.read((uint8_t*)&state->chr_bank0_fd, sizeof(state->chr_bank0_fd));
    stateFile.read((uint8_t*)&state->chr_bank0_fe, sizeof(state->chr_bank0_fe));
    stateFile.read((uint8_t*)&state->chr_bank1_fd, sizeof(state->chr_bank1_fd));
    stateFile.read((uint8_t*)&state->chr_bank1_fe, sizeof(state->chr_bank1_fe));
}

// =====================================
// FACTORY
// =====================================

Mapper createMapper009(uint8_t PRG_banks, uint8_t CHR_banks, ROMBackend backend, Cartridge* cart)
{
    Mapper mapper;
    Mapper009_state* state = new Mapper009_state;
    memset(state, 0, sizeof(Mapper009_state));

    state->backend = backend;
    state->number_PRG_banks = PRG_banks * 2;
    state->number_CHR_banks = CHR_banks * 2;
    state->cart = cart;

    if (backend == ROMBackend::FLASH)
    {
        state->mROM = &cart->mROM;
    }
    else
    {
        bankInit(&state->CHR_cache_low, state->CHR_banks_low, MAPPER009_MAX_CACHED_BANKS, 4096, cart);
        bankInit(&state->CHR_cache_high, state->CHR_banks_high, MAPPER009_MAX_CACHED_BANKS, 4096, cart);
    }

    mapper.state = state;
    return mapper;
}