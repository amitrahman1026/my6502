#include "bus.h"
#include "cpu.h"
#include <memory>

Bus_16bit::Bus_16bit() {}
Bus_16bit::~Bus_16bit() {}

void Bus_16bit::connectCpu(std::shared_ptr<Cpu6502> c) {
    cpu = c;
}

uint16_t Bus_16bit::read(uint16_t addr) { return memory[addr]; }

void Bus_16bit::write(uint16_t addr, uint8_t data) { memory[addr] = data; }
