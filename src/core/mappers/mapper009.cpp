#include "mapper009.h"
#include "../cartridge.h"
#include <Arduino.h>
#include <cstring>

// Habilitar logs solo para eventos importantes
#define LOG_MAPPER 1
#if LOG_MAPPER
#define MAPPER_LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define MAPPER_LOG(fmt, ...)
#endif

// ==================== Funciones auxiliares ====================

static inline uint8_t* getCHRBankLRU(Mapper009_state* state, uint8_t bankIndex, bool lowBank) {
    if (bankIndex >= state->number_CHR_banks) return nullptr;
    BankCache* cache = lowBank ? &state->CHR_cache_low : &state->CHR_cache_high;
    return getBank(cache, bankIndex, RomType::CHR);
}

static inline uint8_t* getCHRBankFLASH(Mapper009_state* state, uint8_t bankIndex) {
    if (bankIndex >= state->number_CHR_banks) return nullptr;
    return (uint8_t*)(state->mROM->chr_base + bankIndex * 4096U);
}

static void loadPRGBank8K(Mapper009_state* state, uint8_t** bankPtr, uint8_t bankIndex, const char* name) {
    if (state->backend == ROMBackend::LRU) {
        if (!(*bankPtr)) {
            *bankPtr = (uint8_t*)malloc(8192);
            MAPPER_LOG("loadPRGBank8K: allocated 8KB for %s at %p\n", name, *bankPtr);
        }
        if (*bankPtr) {
            uint32_t offset = bankIndex * 8192;
            MAPPER_LOG("loadPRGBank8K: loading %s bank %d from offset %u\n", name, bankIndex, offset);
            state->cart->loadPRGBank(*bankPtr, 8192, offset);
        } else {
            MAPPER_LOG("loadPRGBank8K: malloc failed for %s\n", name);
        }
    } else {
        *bankPtr = (uint8_t*)(state->mROM->prg_base + bankIndex * 8192);
        MAPPER_LOG("loadPRGBank8K: FLASH %s at %p\n", name, *bankPtr);
    }
}

static void loadFixedPRGBank(Mapper009_state* state) {
    if (state->number_PRG_banks < 2) return;
    uint16_t size = 16 * 1024;
    uint32_t offset = (state->number_PRG_banks - 2) * 8192;
    if (state->backend == ROMBackend::LRU) {
        if (!state->PRG_bank_fixed) {
            state->PRG_bank_fixed = (uint8_t*)malloc(size);
            MAPPER_LOG("loadFixedPRGBank: malloc'd 16KB at %p\n", state->PRG_bank_fixed);
        }
        if (state->PRG_bank_fixed) {
            MAPPER_LOG("loadFixedPRGBank: loading from offset %u\n", offset);
            state->cart->loadPRGBank(state->PRG_bank_fixed, size, offset);
            uint8_t lo = state->PRG_bank_fixed[0xFFFC & 0x3FFF];
            uint8_t hi = state->PRG_bank_fixed[0xFFFD & 0x3FFF];
            uint16_t reset_addr = lo | (hi << 8);
            MAPPER_LOG("loadFixedPRGBank: reset vector = 0x%04X\n", reset_addr);
        } else {
            MAPPER_LOG("loadFixedPRGBank: malloc failed\n");
            static uint8_t dummy[16384] = {0};
            state->PRG_bank_fixed = dummy;
        }
    } else {
        state->PRG_bank_fixed = (uint8_t*)(state->mROM->prg_base + offset);
        MAPPER_LOG("loadFixedPRGBank: FLASH ptr = %p\n", state->PRG_bank_fixed);
        uint8_t lo = state->PRG_bank_fixed[0xFFFC & 0x3FFF];
        uint8_t hi = state->PRG_bank_fixed[0xFFFD & 0x3FFF];
        uint16_t reset_addr = lo | (hi << 8);
        MAPPER_LOG("loadFixedPRGBank: reset vector = 0x%04X\n", reset_addr);
    }
}

// ==================== Funciones del mapper ====================

bool mapper009_cpuRead(Mapper* mapper, uint16_t addr, uint8_t& data) {
    if (addr < 0x8000) return false;
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) { data = 0xFF; return true; }

    if (addr < 0xA000) {
        if (state->PRG_bank_low) data = state->PRG_bank_low[addr & 0x1FFF];
        else data = 0xFF;
    } else if (addr < 0xC000) {
        if (state->PRG_bank_high) data = state->PRG_bank_high[addr & 0x1FFF];
        else data = 0xFF;
    } else {
        if (state->PRG_bank_fixed) data = state->PRG_bank_fixed[addr & 0x3FFF];
        else data = 0xFF;
    }
    // Solo loguear lecturas de vectores para no saturar
    if (addr >= 0xFFFA && addr <= 0xFFFF) {
        MAPPER_LOG("cpuRead: vector read at 0x%04X = 0x%02X\n", addr, data);
    }
    return true;
}

bool mapper009_cpuWrite(Mapper* mapper, uint16_t addr, uint8_t data) {
    if (addr < 0x8000) return false;
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return true;

    MAPPER_LOG("cpuWrite: addr=0x%04X, data=0x%02X\n", addr, data);

    if (addr >= 0xA000 && addr < 0xB000) {
        state->chr_bank0_fd = data & 0x1F;
        if (state->latch0 == 0xFD) {
            uint8_t* newBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, state->chr_bank0_fd, true) : getCHRBankFLASH(state, state->chr_bank0_fd);
            if (newBank) state->ptr_CHR_bank_4K_low = newBank;
        }
    } else if (addr >= 0xB000 && addr < 0xC000) {
        state->chr_bank0_fe = data & 0x1F;
        if (state->latch0 == 0xFE) {
            uint8_t* newBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, state->chr_bank0_fe, true) : getCHRBankFLASH(state, state->chr_bank0_fe);
            if (newBank) state->ptr_CHR_bank_4K_low = newBank;
        }
    } else if (addr >= 0xC000 && addr < 0xD000) {
        state->chr_bank1_fd = data & 0x1F;
        if (state->latch1 == 0xFD) {
            uint8_t* newBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, state->chr_bank1_fd, false) : getCHRBankFLASH(state, state->chr_bank1_fd);
            if (newBank) state->ptr_CHR_bank_4K_high = newBank;
        }
    } else if (addr >= 0xD000 && addr < 0xE000) {
        state->chr_bank1_fe = data & 0x1F;
        if (state->latch1 == 0xFE) {
            uint8_t* newBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, state->chr_bank1_fe, false) : getCHRBankFLASH(state, state->chr_bank1_fe);
            if (newBank) state->ptr_CHR_bank_4K_high = newBank;
        }
    } else if (addr >= 0xE000 && addr < 0xF000) {
        uint8_t bank = data & 0x0F;
        loadPRGBank8K(state, &state->PRG_bank_low, bank, "low");
    } else if (addr >= 0xF000) {
        if (addr < 0xF100) {
            uint8_t bank = data & 0x0F;
            loadPRGBank8K(state, &state->PRG_bank_high, bank, "high");
        } else {
            state->cart->setMirrorMode((data & 0x01) ? Cartridge::HORIZONTAL : Cartridge::VERTICAL);
        }
    }
    return true;
}

bool mapper009_ppuRead(Mapper* mapper, uint16_t addr, uint8_t& data) {
    if (addr > 0x1FFF) return false;
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) { data = 0; return true; }

    uint8_t* bank = (addr < 0x1000) ? state->ptr_CHR_bank_4K_low : state->ptr_CHR_bank_4K_high;
    if (!bank) { data = 0; return true; }

    data = bank[addr & 0xFFF];

    // Auto-banking MMC2
    if (addr == 0x0FD8) {
        state->latch0 = 0xFD;
        uint8_t* newBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, state->chr_bank0_fd, true) : getCHRBankFLASH(state, state->chr_bank0_fd);
        if (newBank) state->ptr_CHR_bank_4K_low = newBank;
        MAPPER_LOG("ppuRead: auto-bank low $FD, new bank = %d\n", state->chr_bank0_fd);
    } else if (addr == 0x0FE8) {
        state->latch0 = 0xFE;
        uint8_t* newBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, state->chr_bank0_fe, true) : getCHRBankFLASH(state, state->chr_bank0_fe);
        if (newBank) state->ptr_CHR_bank_4K_low = newBank;
        MAPPER_LOG("ppuRead: auto-bank low $FE, new bank = %d\n", state->chr_bank0_fe);
    } else if (addr == 0x1FD8) {
        state->latch1 = 0xFD;
        uint8_t* newBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, state->chr_bank1_fd, false) : getCHRBankFLASH(state, state->chr_bank1_fd);
        if (newBank) state->ptr_CHR_bank_4K_high = newBank;
        MAPPER_LOG("ppuRead: auto-bank high $FD, new bank = %d\n", state->chr_bank1_fd);
    } else if (addr == 0x1FE8) {
        state->latch1 = 0xFE;
        uint8_t* newBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, state->chr_bank1_fe, false) : getCHRBankFLASH(state, state->chr_bank1_fe);
        if (newBank) state->ptr_CHR_bank_4K_high = newBank;
        MAPPER_LOG("ppuRead: auto-bank high $FE, new bank = %d\n", state->chr_bank1_fe);
    }
    return true;
}

bool mapper009_ppuWrite(Mapper* mapper, uint16_t addr, uint8_t data) { return false; }

uint8_t* mapper009_ppuReadPtr(Mapper* mapper, uint16_t addr) {
    if (addr > 0x1FFF) return nullptr;
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return nullptr;
    uint8_t* bank = (addr < 0x1000) ? state->ptr_CHR_bank_4K_low : state->ptr_CHR_bank_4K_high;
    if (!bank) return nullptr;
    return &bank[addr & 0xFFF];
}

void mapper009_reset(Mapper* mapper) {
    MAPPER_LOG("mapper009_reset: entry\n");
    Mapper009_state* state = (Mapper009_state*)mapper->state;
    if (!state) return;

    state->latch0 = 0xFD;
    state->latch1 = 0xFD;

    memset(state->dummy_chr, 0, sizeof(state->dummy_chr));

    // Inicializar bancos PRG: bajo = 0, alto = 0 (cambio crítico)
    loadPRGBank8K(state, &state->PRG_bank_low, 0, "low (reset)");
    loadPRGBank8K(state, &state->PRG_bank_high, 0, "high (reset)");
    loadFixedPRGBank(state);
	
	// Después de loadFixedPRGBank(state);
	if (state->PRG_bank_fixed) {
		Serial.printf("Código en 0xFEF0: ");
		for (int i = 0; i < 16; i++) {
			Serial.printf("%02X ", state->PRG_bank_fixed[(0xFEF0 & 0x3FFF) + i]);
		}
		Serial.printf("\n");
	}

    state->cart->setMirrorMode(Cartridge::VERTICAL);

    // Cargar bancos CHR iniciales (banco 0)
    uint8_t* lowBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, 0, true) : getCHRBankFLASH(state, 0);
    uint8_t* highBank = (state->backend == ROMBackend::LRU) ? getCHRBankLRU(state, 0, false) : getCHRBankFLASH(state, 0);
    state->ptr_CHR_bank_4K_low = lowBank ? lowBank : state->dummy_chr;
    state->ptr_CHR_bank_4K_high = highBank ? highBank : state->dummy_chr;

    MAPPER_LOG("reset: low PRG=%p, high PRG=%p, fixed=%p\n", state->PRG_bank_low, state->PRG_bank_high, state->PRG_bank_fixed);
    MAPPER_LOG("reset: low CHR=%p, high CHR=%p\n", state->ptr_CHR_bank_4K_low, state->ptr_CHR_bank_4K_high);
    MAPPER_LOG("mapper009_reset: exit\n");
}

void mapper009_dumpState(Mapper* mapper, File& state) {
    Mapper009_state* s = (Mapper009_state*)mapper->state;
    state.write((uint8_t*)&s->latch0, sizeof(s->latch0));
    state.write((uint8_t*)&s->latch1, sizeof(s->latch1));
    state.write((uint8_t*)&s->chr_bank0_fd, sizeof(s->chr_bank0_fd));
    state.write((uint8_t*)&s->chr_bank0_fe, sizeof(s->chr_bank0_fe));
    state.write((uint8_t*)&s->chr_bank1_fd, sizeof(s->chr_bank1_fd));
    state.write((uint8_t*)&s->chr_bank1_fe, sizeof(s->chr_bank1_fe));
}

void mapper009_loadState(Mapper* mapper, File& state) {
    Mapper009_state* s = (Mapper009_state*)mapper->state;
    state.read((uint8_t*)&s->latch0, sizeof(s->latch0));
    state.read((uint8_t*)&s->latch1, sizeof(s->latch1));
    state.read((uint8_t*)&s->chr_bank0_fd, sizeof(s->chr_bank0_fd));
    state.read((uint8_t*)&s->chr_bank0_fe, sizeof(s->chr_bank0_fe));
    state.read((uint8_t*)&s->chr_bank1_fd, sizeof(s->chr_bank1_fd));
    state.read((uint8_t*)&s->chr_bank1_fe, sizeof(s->chr_bank1_fe));
    if (s->backend == ROMBackend::LRU) {
        s->ptr_CHR_bank_4K_low = (s->latch0 == 0xFD) ? getCHRBankLRU(s, s->chr_bank0_fd, true) : getCHRBankLRU(s, s->chr_bank0_fe, true);
        s->ptr_CHR_bank_4K_high = (s->latch1 == 0xFD) ? getCHRBankLRU(s, s->chr_bank1_fd, false) : getCHRBankLRU(s, s->chr_bank1_fe, false);
    } else {
        s->ptr_CHR_bank_4K_low = (s->latch0 == 0xFD) ? getCHRBankFLASH(s, s->chr_bank0_fd) : getCHRBankFLASH(s, s->chr_bank0_fe);
        s->ptr_CHR_bank_4K_high = (s->latch1 == 0xFD) ? getCHRBankFLASH(s, s->chr_bank1_fd) : getCHRBankFLASH(s, s->chr_bank1_fe);
    }
    if (!s->ptr_CHR_bank_4K_low) s->ptr_CHR_bank_4K_low = s->dummy_chr;
    if (!s->ptr_CHR_bank_4K_high) s->ptr_CHR_bank_4K_high = s->dummy_chr;
}

Mapper createMapper009(uint8_t PRG_banks, uint8_t CHR_banks, ROMBackend backend, Cartridge* cart) {
    // Hack para Punch-Out!! (CRC BE939FCE) que necesita 16 bancos PRG
    if (cart->CRC32 == 0xBE939FCE && PRG_banks == 8) {
        Serial.printf("createMapper009: Forzando PRG banks de 8 a 16 para Punch-Out!!\n");
        PRG_banks = 16;
    }
    Serial.printf("createMapper009: PRG banks=%d, CHR banks=%d, backend=%d\n", PRG_banks, CHR_banks, backend);
    Mapper mapper;
    Mapper009_state* state = new Mapper009_state;
    memset(state, 0, sizeof(Mapper009_state));
    state->backend = backend;
    state->number_PRG_banks = PRG_banks;
    state->number_CHR_banks = CHR_banks;
    state->cart = cart;

    if (backend == ROMBackend::FLASH) {
        state->mROM = &cart->mROM;
    } else {
        bankInit(&state->CHR_cache_low, state->CHR_banks_4K, MAPPER009_MAX_CACHED_BANKS, 4096, cart);
        bankInit(&state->CHR_cache_high, state->CHR_banks_4K, MAPPER009_MAX_CACHED_BANKS, 4096, cart);
    }

    state->latch0 = 0xFD;
    state->latch1 = 0xFD;
    state->chr_bank0_fd = 0;
    state->chr_bank0_fe = 0;
    state->chr_bank1_fd = 0;
    state->chr_bank1_fe = 0;
    state->ptr_CHR_bank_4K_low = nullptr;
    state->ptr_CHR_bank_4K_high = nullptr;
    state->PRG_bank_low = nullptr;
    state->PRG_bank_high = nullptr;
    state->PRG_bank_fixed = nullptr;
    memset(state->dummy_chr, 0, sizeof(state->dummy_chr));

    mapper.state = state;
    return mapper;
}