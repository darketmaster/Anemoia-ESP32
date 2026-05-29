#include "mapper009.h"
#include "../cartridge.h"

#define GET_STATE Mapper009_state* s = (Mapper009_state*)mapper->state

void mapper009_sync_prg(Mapper* mapper) {
    GET_STATE;
    // En Anemoia, el tamaño está en mROM.prg_size
    uint32_t prg_size = s->cart->mROM.prg_size;
    
    // Banco conmutable $8000
    s->prg_offsets[0] = (s->prg_bank * 0x2000) % prg_size;
    // Bancos fijos $A000, $C000, $E000 (últimos 24KB de la ROM)
    s->prg_offsets[1] = prg_size - 0x6000;
    s->prg_offsets[2] = prg_size - 0x4000;
    s->prg_offsets[3] = prg_size - 0x2000;
}

Mapper createMapper009(uint8_t PRG_banks, uint8_t CHR_banks, ROMBackend backend, Cartridge* cart) {
    Mapper mapper;
    Mapper009_state* s = (Mapper009_state*)malloc(sizeof(Mapper009_state));
    memset(s, 0, sizeof(Mapper009_state));

    s->cart = cart;
    s->prg_bank = 0;
    s->latch0 = 0xFE;
    s->latch1 = 0xFE;

    mapper.state = s;
    mapper009_sync_prg(&mapper);
    return mapper;
}

bool mapper009_cpuRead(Mapper* mapper, uint16_t addr, uint8_t& data) {
    if (addr < 0x8000) return false;
    GET_STATE;
    
    uint8_t bank_idx = (addr >> 13) & 0x03; // Determina cuál de los 4 bancos de 8KB es
    uint32_t internal_addr = s->prg_offsets[bank_idx] + (addr & 0x1FFF);
    
    // Acceso usando prg_base que es el puntero real a la memoria mapeada
    data = s->cart->mROM.prg_base[internal_addr];
    return true;
}

bool mapper009_cpuWrite(Mapper* mapper, uint16_t addr, uint8_t data) {
    if (addr < 0xA000) return false;
    GET_STATE;

    switch (addr & 0xF000) {
        case 0xA000: 
            s->prg_bank = data & 0x0F; 
            mapper009_sync_prg(mapper);
            break;
        case 0xB000: s->chr_fd_0 = data & 0x1F; break;
        case 0xC000: s->chr_fe_0 = data & 0x1F; break;
        case 0xD000: s->chr_fd_1 = data & 0x1F; break;
        case 0xE000: s->chr_fe_1 = data & 0x1F; break;
        case 0xF000: 
            s->cart->mirror = (data & 0x01) ? Cartridge::HORIZONTAL : Cartridge::VERTICAL; 
            break;
    }
    return true;
}

uint8_t* mapper009_ppuReadPtr(Mapper* mapper, uint16_t addr) {
    if (addr >= 0x2000) return nullptr;
    GET_STATE;

    // MMC2 Latch Logic
    // Al leer estas direcciones el hardware cambia el banco de CHR automáticamente
    if (addr == 0x0FD8) s->latch0 = 0xFD;
    else if (addr == 0x0FE8) s->latch0 = 0xFE;
    else if ((addr & 0x1FF8) == 0x1FD8) s->latch1 = 0xFD;
    else if ((addr & 0x1FF8) == 0x1FE8) s->latch1 = 0xFE;

    uint32_t bank;
    if (addr < 0x1000)
        bank = (s->latch0 == 0xFD) ? s->chr_fd_0 : s->chr_fe_0;
    else
        bank = (s->latch1 == 0xFD) ? s->chr_fd_1 : s->chr_fe_1;

    uint32_t chr_addr = (bank * 0x1000) + (addr & 0x0FFF);
    
    // Retornamos el puntero a chr_base con cast a uint8_t* (porque chr_base es const)
    return (uint8_t*)&s->cart->mROM.chr_base[chr_addr % s->cart->mROM.chr_size];
}

bool mapper009_ppuRead(Mapper* mapper, uint16_t addr, uint8_t& data) {
    uint8_t* ptr = mapper009_ppuReadPtr(mapper, addr);
    if (ptr) {
        data = *ptr;
        return true;
    }
    return false;
}

void mapper009_reset(Mapper* mapper) {
    GET_STATE;
    s->prg_bank = 0;
    s->latch0 = 0xFE;
    s->latch1 = 0xFE;
    mapper009_sync_prg(mapper);
}

bool mapper009_ppuWrite(Mapper* mapper, uint16_t addr, uint8_t data) {
    return false; // MMC2 es CHR-ROM
}

void mapper009_dumpState(Mapper* mapper, File& state) {
    GET_STATE;
    state.write((uint8_t*)s, sizeof(Mapper009_state));
}

void mapper009_loadState(Mapper* mapper, File& state) {
    GET_STATE;
    state.read((uint8_t*)s, sizeof(Mapper009_state));
    mapper009_sync_prg(mapper);
}