#ifndef MAPPER009_H
#define MAPPER009_H

#include "../mapper.h"

#define MAPPER009_MAX_CACHED_BANKS 4

struct Mapper009_state
{
    Cartridge* cart = nullptr;
    MappedROM* mROM = nullptr;
    ROMBackend backend;
    uint8_t number_PRG_banks;
    uint8_t number_CHR_banks;

    // Dos bancos PRG conmutables de 8KB
    uint8_t* PRG_bank_low = nullptr;   // $8000-$9FFF
    uint8_t* PRG_bank_high = nullptr;  // $A000-$BFFF
    uint8_t* PRG_bank_fixed = nullptr; // $C000-$FFFF (16KB)

    // Latches y bancos CHR (igual que antes)
    uint8_t latch0;
    uint8_t latch1;
    uint8_t chr_bank0_fd;
    uint8_t chr_bank0_fe;
    uint8_t chr_bank1_fd;
    uint8_t chr_bank1_fe;

    uint8_t* ptr_CHR_bank_4K_low;
    uint8_t* ptr_CHR_bank_4K_high;

    Bank CHR_banks_4K[MAPPER009_MAX_CACHED_BANKS];
    BankCache CHR_cache_low;
    BankCache CHR_cache_high;

    uint8_t dummy_chr[4096];
};

Mapper createMapper009(uint8_t PRG_banks, uint8_t CHR_banks, ROMBackend backend, Cartridge* cart);

bool mapper009_cpuRead(Mapper* mapper, uint16_t addr, uint8_t& data);
bool mapper009_cpuWrite(Mapper* mapper, uint16_t addr, uint8_t data);
bool mapper009_ppuRead(Mapper* mapper, uint16_t addr, uint8_t& data);
bool mapper009_ppuWrite(Mapper* mapper, uint16_t addr, uint8_t data);
uint8_t* mapper009_ppuReadPtr(Mapper* mapper, uint16_t addr);
void mapper009_reset(Mapper* mapper);
void mapper009_dumpState(Mapper* mapper, File& state);
void mapper009_loadState(Mapper* mapper, File& state);

#endif