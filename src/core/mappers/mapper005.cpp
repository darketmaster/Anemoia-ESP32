#include "mapper005.h"
#include "../cartridge.h"
#include <Arduino.h>
#include <cstring>

#define LOG_MAPPER005 1
#if LOG_MAPPER005
    #define MAPPER_LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
    #define MAPPER_LOG(fmt, ...)
#endif

static int chr_log_counter = 0;

// --------------------------------------------------------------
// PRG access
// --------------------------------------------------------------
static inline uint8_t* getPRGPtr(Mapper005_state* state, int slot, uint16_t addr_offset) {
    if (slot == 0) return nullptr;
    uint8_t bank = state->prg_banks[slot];
    if (state->prg_use_ram[slot]) return nullptr;
    if (bank >= state->num_prg_banks) bank = state->num_prg_banks - 1;
    return (uint8_t*)(state->mROM->prg_base + bank * 8192) + (addr_offset & 0x1FFF);
}

// --------------------------------------------------------------
// CHR helpers (con parche para fondo forzado a conjunto B)
// --------------------------------------------------------------
static inline uint8_t* getCHRBankPtr(Mapper005_state* state, uint16_t addr) {
    uint8_t mode = state->chr_mode;
    uint8_t high = state->chr_high_bits & 0x03;
    bool isBg = (addr >= 0x1000);
    
    // En modo 8x8 sprites, forzamos el fondo a usar conjunto B
    uint8_t* regs;
    if (state->sprite_8x16) {
        regs = isBg ? state->chr_b_regs : state->chr_a_regs;
    } else {
        // PARCHE: fondo usa conjunto B, sprites usan conjunto A (independientemente de last_chr_set)
        regs = isBg ? state->chr_b_regs : state->chr_a_regs;
    }
    
    int slot;
    uint8_t reg_value;
    switch (mode) {
        case 0: slot = 0; reg_value = regs[0]; break;
        case 1: slot = (addr >> 12) & 1; reg_value = regs[slot]; break;
        case 2: slot = (addr >> 11) & 3; reg_value = regs[slot]; break;
        case 3:
            if (!isBg || (state->sprite_8x16 && isBg)) {
                slot = (addr >> 10) & 7;
                reg_value = regs[slot];
            } else {
                slot = (addr >> 10) & 3;
                reg_value = regs[slot];
            }
            break;
        default: return nullptr;
    }
    
    uint16_t bank = (high << 8) | reg_value;
    if (bank >= state->num_chr_banks_1k) bank = state->num_chr_banks_1k - 1;
    uint32_t offset = bank * 1024 + (addr & 0x3FF);
    
    if (chr_log_counter++ % 1000 == 0) {
        MAPPER_LOG("CHR: addr=0x%04X, regs=%s, slot=%d, val=0x%02X, bank=%d\n",
                   addr, (regs == state->chr_a_regs) ? "A" : "B", slot, reg_value, bank);
    }
    return (uint8_t*)(state->mROM->chr_base + offset);
}

// --------------------------------------------------------------
// ExRAM modo Ex1 simplificado (sin depender de PPU)
// --------------------------------------------------------------
static uint8_t getEx1BgTile(Mapper005_state* state, uint16_t addr) {
    // Usamos addr & 0x3FF como índice en ExRAM (simplificado)
    uint8_t exram_byte = state->exram[addr & 0x3FF];
    uint8_t high = state->chr_high_bits & 0x03;
    uint16_t bank_4k = (high << 6) | (exram_byte & 0x3F);
    uint32_t offset = bank_4k * 4096 + (addr & 0xFFF);
    if (offset < state->num_chr_banks_1k * 1024) {
        return state->mROM->chr_base[offset];
    }
    return 0;
}

// --------------------------------------------------------------
// CPU READ
// --------------------------------------------------------------
bool mapper005_cpuRead(Mapper* mapper, uint16_t addr, uint8_t& data) {
    Mapper005_state* state = (Mapper005_state*)mapper->state;
    if (!state) { data = 0xFF; return true; }

    if (addr >= 0x5C00 && addr <= 0x5FFF) {
        if (state->exram_mode == 2 || state->exram_mode == 3) data = state->exram[addr & 0x3FF];
        else data = 0xFF;
        return true;
    }
    if (addr == 0x5205) { data = state->mul_result & 0xFF; return true; }
    if (addr == 0x5206) { data = (state->mul_result >> 8) & 0xFF; return true; }
    if (addr == 0x5204) {
        data = (state->irq_pending ? 0x40 : 0x00) | (state->in_frame ? 0x80 : 0x00);
        state->irq_pending = false;
        return true;
    }
    if (addr >= 0x6000 && addr <= 0xFFFF) {
        int slot = (addr < 0x8000) ? 0 : (addr < 0xA000) ? 1 : (addr < 0xC000) ? 2 : (addr < 0xE000) ? 3 : 4;
        uint8_t* ptr = getPRGPtr(state, slot, addr);
        data = ptr ? *ptr : 0xFF;
        return true;
    }
    return false;
}

// --------------------------------------------------------------
// CPU WRITE
// --------------------------------------------------------------
bool mapper005_cpuWrite(Mapper* mapper, uint16_t addr, uint8_t data) {
    Mapper005_state* state = (Mapper005_state*)mapper->state;
    if (!state) return true;

    static int write_counter = 0;
    if (write_counter++ % 100000 == 0) MAPPER_LOG("cpuWrite addr=0x%04X data=0x%02X\n", addr, data);

    if (addr >= 0x6000 && addr < 0x8000) {
        bool write_enabled = (state->prg_ram_protect[0] == 0x02 && state->prg_ram_protect[1] == 0x01);
        if (write_enabled && state->prg_ram) {
            uint32_t bank8k = state->prg_ram_bank & 0x07;
            state->prg_ram[bank8k * 8192 + (addr & 0x1FFF)] = data;
        }
        return true;
    }

    if (addr >= 0x5000 && addr <= 0x5015) {
        MAPPER_LOG("MMC5 audio write: 0x%04X=0x%02X\n", addr, data);
        return true;
    }

    if (addr >= 0x5C00 && addr <= 0x5FFF) {
        bool writable = (state->exram_mode == 2) || (state->exram_mode == 0);
        if (writable) state->exram[addr & 0x3FF] = data;
        return true;
    }

    if (addr >= 0x5000 && addr <= 0x5FFF) {
        switch (addr) {
            case 0x5100: state->prg_mode = data & 0x03; break;
            case 0x5101: state->chr_mode = data & 0x03; MAPPER_LOG("CHR mode=%d\n", state->chr_mode); break;
            case 0x5102: state->prg_ram_protect[0] = data; break;
            case 0x5103: state->prg_ram_protect[1] = data; break;
            case 0x5104: state->exram_mode = data & 0x03; MAPPER_LOG("ExRAM mode=%d\n", state->exram_mode); break;
            case 0x5105: {
                state->mirroring = data;
                uint8_t a=data&3, b=(data>>2)&3, c=(data>>4)&3, d=(data>>6)&3;
                if (a==0&&b==0&&c==0&&d==0) state->cart->setMirrorMode(Cartridge::ONESCREEN_LOW);
                else if (a==1&&b==1&&c==1&&d==1) state->cart->setMirrorMode(Cartridge::ONESCREEN_HIGH);
                else if (a==0&&b==0&&c==1&&d==1) state->cart->setMirrorMode(Cartridge::VERTICAL);
                else if (a==0&&b==1&&c==0&&d==1) state->cart->setMirrorMode(Cartridge::HORIZONTAL);
                break;
            }
            case 0x5106: state->fill_tile = data; break;
            case 0x5107: state->fill_attr = data & 0x03; break;
            case 0x5113: state->prg_ram_bank = data; break;
            case 0x5114: state->prg_banks[1] = data & 0x7F; state->prg_use_ram[1] = ((data&0x80)==0); break;
            case 0x5115: state->prg_banks[2] = data & 0x7F; state->prg_use_ram[2] = ((data&0x80)==0); break;
            case 0x5116: state->prg_banks[3] = data & 0x7F; state->prg_use_ram[3] = ((data&0x80)==0); break;
            case 0x5117: state->prg_banks[4] = data & 0x7F; state->prg_use_ram[4] = false; break;
            case 0x5120 ... 0x5127:
                state->chr_a_regs[addr-0x5120] = data;
                state->last_chr_set = 0;
                MAPPER_LOG("CHR A[%d]=0x%02X\n", addr-0x5120, data);
                break;
            case 0x5128 ... 0x512B:
                state->chr_b_regs[addr-0x5128] = data;
                state->last_chr_set = 1;
                MAPPER_LOG("CHR B[%d]=0x%02X\n", addr-0x5128, data);
                break;
            case 0x5130: state->chr_high_bits = data & 0x03; break;
            case 0x5200: state->split_control = data; break;
            case 0x5201: state->split_y_scroll = data; break;
            case 0x5202: state->split_chr_page = data; break;
            case 0x5203: state->irq_target = data; break;
            case 0x5204: state->irq_enabled = (data & 0x80) != 0; break;
            case 0x5205: state->mul_a = data; state->mul_result = state->mul_a * state->mul_b; break;
            case 0x5206: state->mul_b = data; state->mul_result = state->mul_a * state->mul_b; break;
            default: break;
        }
        return true;
    }

    if (addr >= 0x8000 && addr <= 0xFFFF) {
        uint8_t bank = data & 0x7F;
        switch (state->prg_mode) {
            case 0: // 32K mode
                if (addr >= 0x8000 && addr < 0xA000) {
                    state->prg_banks[1] = bank;
                    state->prg_banks[2] = bank + 1;
                    state->prg_banks[3] = bank + 2;
                    state->prg_banks[4] = bank + 3;
                }
                break;
            case 1: // 16K mode
                if (addr >= 0x8000 && addr < 0xA000) {
                    state->prg_banks[1] = bank;
                    state->prg_banks[2] = bank;
                    state->prg_banks[3] = bank + 1;
                    state->prg_banks[4] = bank + 1;
                } else if (addr >= 0xC000 && addr < 0xE000) {
                    state->prg_banks[3] = bank;
                    state->prg_banks[4] = bank;
                }
                break;
            case 3: // 8K mode
                if (addr >= 0x8000 && addr < 0xA000) state->prg_banks[1] = bank;
                else if (addr >= 0xA000 && addr < 0xC000) state->prg_banks[2] = bank;
                else if (addr >= 0xC000 && addr < 0xE000) state->prg_banks[3] = bank;
                else if (addr >= 0xE000 && addr <= 0xFFFF) state->prg_banks[4] = bank;
                break;
        }
        return true;
    }
    return false;
}

// --------------------------------------------------------------
// PPU READ (con soporte ExRAM simplificado)
// --------------------------------------------------------------
bool mapper005_ppuRead(Mapper* mapper, uint16_t addr, uint8_t& data) {
    if (addr > 0x1FFF) return false;
    Mapper005_state* state = (Mapper005_state*)mapper->state;
    if (!state) { data = 0; return true; }

    // Modo ExRAM 1: atributos extendidos para fondo
    if (state->exram_mode == 1 && addr >= 0x1000) {
        data = getEx1BgTile(state, addr);
        return true;
    }
    // Modo normal de CHR
    uint8_t* ptr = getCHRBankPtr(state, addr);
    data = ptr ? *ptr : 0;
    return true;
}

bool mapper005_ppuWrite(Mapper* mapper, uint16_t addr, uint8_t data) { return false; }

uint8_t* mapper005_ppuReadPtr(Mapper* mapper, uint16_t addr) {
    if (addr > 0x1FFF) return nullptr;
    Mapper005_state* state = (Mapper005_state*)mapper->state;
    if (!state) return nullptr;
    uint8_t* ptr = getCHRBankPtr(state, addr);
    return ptr ? &ptr[addr & 0x3FF] : nullptr;
}

// --------------------------------------------------------------
// IRQ scanline
// --------------------------------------------------------------
void mapper005_scanline(Mapper* mapper) {
    Mapper005_state* state = (Mapper005_state*)mapper->state;
    if (!state) return;
    if (!state->in_frame) {
        state->in_frame = true;
        state->irq_counter = 0;
        state->irq_pending = false;
    } else {
        state->irq_counter++;
        if (state->irq_counter == state->irq_target && state->irq_target != 0) {
            state->irq_pending = true;
            if (state->irq_enabled) {
                state->cart->IRQ();
            }
        }
    }
}

void mapper005_set_sprite_mode(Mapper* mapper, bool is8x16) {
    Mapper005_state* state = (Mapper005_state*)mapper->state;
    if (state) {
        state->sprite_8x16 = is8x16;
        MAPPER_LOG("MMC5: sprite mode = %s\n", is8x16 ? "8x16" : "8x8");
    }
}

// --------------------------------------------------------------
// RESET
// --------------------------------------------------------------
void mapper005_reset(Mapper* mapper) {
    Mapper005_state* state = (Mapper005_state*)mapper->state;
    if (!state) return;

    if (!state->prg_ram) {
        state->prg_ram = (uint8_t*)malloc(MAPPER005_PRG_RAM_SIZE);
        if (state->prg_ram) memset(state->prg_ram, 0, MAPPER005_PRG_RAM_SIZE);
        else MAPPER_LOG("MMC5: PRG-RAM allocation failed!\n");
    }

    state->prg_mode = 0x03;
    state->prg_ram_protect[0] = state->prg_ram_protect[1] = 0;
    state->prg_ram_bank = 0;
    state->prg_banks[0] = 0;
    state->prg_banks[1] = 0;
    state->prg_banks[2] = 1;
    state->prg_banks[3] = 2;
    state->prg_banks[4] = state->num_prg_banks - 1;
    state->prg_use_ram[0] = true;
    for (int i=1; i<5; i++) state->prg_use_ram[i] = false;

    // PARCHE: Forzar modo ExRAM = 1 (atributos extendidos)
    state->exram_mode = 1;
    state->chr_mode = 0x03;
    state->chr_high_bits = 0;
    // Inicializar registros CHR con valores básicos (bancos 0-7)
    for (int i=0; i<8; i++) state->chr_a_regs[i] = i;
    for (int i=0; i<4; i++) state->chr_b_regs[i] = i + 4;
    state->last_chr_set = 0;
    state->sprite_8x16 = true;

    state->mirroring = 0;
    state->fill_tile = 0;
    state->fill_attr = 0;
    memset(state->exram, 0, MAPPER005_EXRAM_SIZE);
    // Rellenar ExRAM con valores que mapeen a bancos 0-63 (para que se vea algo)
    for (int i=0; i<1024; i++) state->exram[i] = i & 0x3F;

    state->split_control = 0;
    state->split_y_scroll = 0;
    state->split_chr_page = 0;
    state->irq_target = 0;
    state->irq_enabled = false;
    state->irq_pending = false;
    state->in_frame = false;
    state->irq_counter = 0;
    state->mul_a = state->mul_b = 0;
    state->mul_result = 0;

    MAPPER_LOG("MMC5 reset completo (PRG_8K=%d, CHR_1K=%d) - ExRAM forzado a 1\n", 
               state->num_prg_banks, state->num_chr_banks_1k);
}

void mapper005_dumpState(Mapper* mapper, File& stateFile) {}
void mapper005_loadState(Mapper* mapper, File& stateFile) {}

// --------------------------------------------------------------
// FACTORY
// --------------------------------------------------------------
Mapper createMapper005(uint8_t PRG_banks, uint8_t CHR_banks, ROMBackend backend, Cartridge* cart) {
    Mapper mapper;
    Mapper005_state* state = new Mapper005_state;
    memset(state, 0, sizeof(Mapper005_state));
    state->backend = backend;
    state->cart = cart;
    state->num_prg_banks = PRG_banks * 2;
    state->num_chr_banks_1k = CHR_banks * 8;
    if (backend == ROMBackend::FLASH) state->mROM = &cart->mROM;
    MAPPER_LOG("MMC5 modo FLASH: PRG_8K=%d, CHR_1K=%d\n", state->num_prg_banks, state->num_chr_banks_1k);
    mapper.state = state;
    return mapper;
}