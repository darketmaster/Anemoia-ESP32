#ifndef MAPPER009_H
#define MAPPER009_H

#include "../mapper.h"

struct Mapper009_state
{
    Cartridge* cart;
    uint8_t prg_bank;
    uint8_t chr_fd_0, chr_fe_0;
    uint8_t chr_fd_1, chr_fe_1;
    uint8_t latch0, latch1; 
    uint32_t prg_offsets[4]; 
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