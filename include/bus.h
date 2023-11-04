#pragma once

#include "cpu.h"
#include <array>
#include <memory>

/**
 * @brief Implementation of a 16-bit address bus.
 * */

class Bus_16bit {
public:
    Bus_16bit();
    ~Bus_16bit();

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

    void connectCpu(std::shared_ptr<Cpu6502> c);

private:
    std::shared_ptr<Cpu6502> cpu;

    std::array<uint8_t, 0x10000> memory = {};
};
