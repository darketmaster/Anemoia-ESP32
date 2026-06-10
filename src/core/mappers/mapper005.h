#ifndef MAPPER005_H
#define MAPPER005_H

#include "../mapper.h"

#define MAPPER005_EXRAM_SIZE 1024
#define MAPPER005_PRG_RAM_SIZE (64 * 1024)

struct Mapper005_state {
    Cartridge* cart = nullptr;
    MappedROM* mROM = nullptr;
    ROMBackend backend;

    uint8_t num_prg_banks;      // bancos PRG de 8KB (header * 2)
    uint8_t num_chr_banks_1k;   // bancos CHR de 1KB (header * 8)

    // PRG
    uint8_t prg_mode;           // $5100
    uint8_t prg_ram_protect[2]; // $5102, $5103
    uint8_t prg_ram_bank;       // $5113
    uint8_t prg_banks[5];       // $6000,$8000,$A000,$C000,$E000
    bool    prg_use_ram[5];
    uint8_t* prg_ram;

    // CHR
    uint8_t chr_mode;           // $5101
    uint8_t chr_high_bits;      // $5130
    uint8_t chr_a_regs[8];      // $5120-$5127
    uint8_t chr_b_regs[4];      // $5128-$512B
    uint8_t last_chr_set;       // 0=A,1=B
    bool    sprite_8x16;        // true si PPU en modo 8x16

    // ExRAM
    uint8_t exram_mode;         // $5104
    uint8_t mirroring;          // $5105
    uint8_t fill_tile;          // $5106
    uint8_t fill_attr;          // $5107
    uint8_t exram[MAPPER005_EXRAM_SIZE];

    // Split screen
    uint8_t split_control;      // $5200
    uint8_t split_y_scroll;     // $5201
    uint8_t split_chr_page;     // $5202

    // IRQ
    uint8_t irq_target;         // $5203
    bool    irq_enabled;        // bit7 de $5204
    bool    irq_pending;
    bool    in_frame;
    uint8_t irq_counter;

    // Multiplicador
    uint8_t mul_a, mul_b;
    uint16_t mul_result;
};

Mapper createMapper005(uint8_t PRG_banks, uint8_t CHR_banks, ROMBackend backend, Cartridge* cart);

bool mapper005_cpuRead(Mapper* mapper, uint16_t addr, uint8_t& data);
bool mapper005_cpuWrite(Mapper* mapper, uint16_t addr, uint8_t data);
bool mapper005_ppuRead(Mapper* mapper, uint16_t addr, uint8_t& data);
bool mapper005_ppuWrite(Mapper* mapper, uint16_t addr, uint8_t data);
uint8_t* mapper005_ppuReadPtr(Mapper* mapper, uint16_t addr);
void mapper005_scanline(Mapper* mapper);
void mapper005_set_sprite_mode(Mapper* mapper, bool is8x16);
void mapper005_reset(Mapper* mapper);
void mapper005_dumpState(Mapper* mapper, File& state);
void mapper005_loadState(Mapper* mapper, File& state);

#endif