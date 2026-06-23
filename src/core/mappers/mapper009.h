/**
 * @file mapper009.h
 * @brief Mapper 009 (MMC2) implementation for Anemoia-ESP32 NES emulator.
 * 
 * This mapper is used by Mike Tyson's Punch-Out!! and other titles.
 * Features 8KB PRG banking and 4KB CHR banking with latch-based tile switching ($FD/$FE).
 * 
 * Memory optimisation: uses eager loading with LRU cache to stay within ESP32 heap.
 * 
 * @version 2.0
 * @author darketmaster
 * @date 2026
 */

#ifndef MAPPER009_H
#define MAPPER009_H

#include "../mapper.h"

// -----------------------------------------------------------------------------
// Cache limits for LRU mode (to avoid heap exhaustion)
// -----------------------------------------------------------------------------

/** @brief Maximum number of 8KB PRG banks kept in RAM simultaneously (64KB total). */
#define MAPPER009_MAX_CACHED_PRG_BANKS 8

/** @brief Maximum number of 4KB CHR banks kept in RAM simultaneously (32KB total). */
#define MAPPER009_MAX_CACHED_CHR_BANKS 8

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

/**
 * @brief Factory function to create a new Mapper 009 instance.
 * @param PRG_banks Number of 16KB PRG-ROM chunks from the iNES header.
 * @param CHR_banks Number of 8KB CHR-ROM chunks from the iNES header.
 * @param backend   Memory backend: FLASH (direct) or LRU (cached).
 * @param cart      Pointer to the parent cartridge object.
 * @return Mapper structure with initialised state.
 */
Mapper createMapper009(uint8_t PRG_banks, uint8_t CHR_banks, ROMBackend backend, Cartridge* cart);

bool mapper009_cpuRead(Mapper* mapper, uint16_t addr, uint8_t& data);
bool mapper009_cpuWrite(Mapper* mapper, uint16_t addr, uint8_t data);
bool mapper009_ppuRead(Mapper* mapper, uint16_t addr, uint8_t& data);
bool mapper009_ppuWrite(Mapper* mapper, uint16_t addr, uint8_t data);
uint8_t* mapper009_ppuReadPtr(Mapper* mapper, uint16_t addr);
void mapper009_reset(Mapper* mapper);
void mapper009_dumpState(Mapper* mapper, File& state);
void mapper009_loadState(Mapper* mapper, File& state);

#endif // MAPPER009_H