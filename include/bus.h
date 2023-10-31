#pragma once

#include "cpu.h"

/**
 * @brief Implementation of a 16-bit address bus.
 * */

class Bus_16bit {
public:
    Bus_16bit() = default;
    ~Bus_16bit() = default;

    /**
     * @brief Read a byte from the specified memory address
     *
     * @param addr The memory address to read from
     * @return The value read from the address
     */
    uint16_t read(uint16_t addr);

    /**
     * @brief Write a byte to the specified memory address
     *
     * @param addr The memory address to write to
     * @param data The data to write
     */
    void write(uint16_t addr, uint8_t data);
};
