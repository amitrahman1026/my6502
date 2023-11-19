#include "cpu.h"
#include "instructions.h"
#include <_types/_uint16_t.h>
#include <_types/_uint8_t.h>

InstructionFactory::InstructionFactory(std::shared_ptr<Cpu6502> c) { cpu = c; }

// Usage
// std::shared_ptr<Cpu6502> cpu = std::make_shared<Cpu6502>();
// InstructionFactory factory(cpu);

bool InstructionFactory::isLegalInstruction(uint8_t addr) {
    return (cpu->opcodeTable[addr].opcode != "???");
}

void InstructionFactory::setCustomInstruction(uint8_t addr,
                                              Cpu6502::Instruction6502 in) {
    if (!InstructionFactory::isLegalInstruction(addr)) {
#warning                                                                       \
    "Overriding legal 6502 instruction set. May cause unxpected behaviour in emulation"
    }
    cpu->opcodeTable[addr] = in;
}

uint8_t Cpu6502::fetch() {
    if (opcodeTable[opcode].address_mode != &Cpu6502::IMP &&
        opcodeTable[opcode].address_mode != &Cpu6502::ACC) {
        operand = read(addr_abs);
    }
    return operand;
}

// Addressing modes
// Return values used to track addicitonal cycles required

// Accumulator Addressing Mode
// Instructions typically use the accumulator register as the operand
uint8_t Cpu6502::ACC() {
    operand = a;
    return 0;
}

// Implied Addressing Mode
// No additionial operand is used
uint8_t Cpu6502::IMP() { return 0; }

// Immediate Addressing Mode
// Immediate addressing allows the programmer to directly specify an 8 bit
// constant within the instruction
uint8_t Cpu6502::IMM() {
    addr_abs = pc++;
    return 0;
}

// Absolute Addressing
// Instructions using absolute addressing contain a full 16 bit address to
// identify the target location
uint8_t Cpu6502::ABS() {
    addr_lo = read(pc++);
    addr_hi = read(pc++);
    addr_abs = (addr_hi << 8) | addr_lo;
    return 0;
}

// Absolute, X-Indexed Addressing Mode
uint8_t Cpu6502::ABX() {
    addr_lo = read(pc++);
    addr_hi = read(pc++);
    addr_abs = (addr_hi << 8) | addr_lo;
    addr_abs += x;
    return ((addr_abs & 0xFF00) != (addr_hi << 8)) ? 1 : 0;
}

// Absolute, Y-Indexed Adressing Mode
uint8_t Cpu6502::ABY() {
    addr_lo = read(pc++);
    addr_hi = read(pc++);
    addr_abs = (addr_hi << 8) | addr_lo;
    addr_abs += y;
    return ((addr_abs & 0xFF00) != (addr_hi << 8)) ? 1 : 0;
}

// Indirect Addressing Mode
uint8_t Cpu6502::IND() {
    uint16_t ptr_lo = read(pc++);
    uint16_t ptr_hi = read(pc++);
    uint16_t ptr = (ptr_hi << 8) | ptr_lo;
    if (ptr_lo == 0x00FF) {
        addr_abs = (read(ptr & 0xFF00) << 8) | read(ptr + 0);
    } else {
        addr_abs = (read(ptr + 1) << 8) | read(ptr + 0);
    }
    return 0;
}

// Indirect, Pre X-Indexed Addressing Mode
uint8_t Cpu6502::IZX() {
    uint16_t ptr_lo = read(pc++);
    addr_lo = read((uint16_t)(ptr_lo + (uint16_t)x) & 0x00FF);
    addr_hi = read((uint16_t)(ptr_lo + (uint16_t)x + 1) & 0x00FF);
    addr_abs = (addr_hi << 8) | addr_lo;
    return 0;
}
// Indirect, Post Y-Indirect Addressing Mode
uint8_t Cpu6502::IZY() {
    uint16_t idx = read(pc++);
    addr_lo = read(idx & 0x00FF);
    addr_hi = read((idx + 1) & 0x00FF);
    addr_abs = (addr_hi << 8) | addr_lo;
    addr_abs += y;
    return ((addr_abs & 0xFF00) != (addr_hi << 8)) ? 1 : 0;
}

// Relative Addressing Mode
uint8_t Cpu6502::REL() {
    addr_rel = read(pc++);
    if (addr_rel & 0x80) addr_rel |= 0xFF00;
    return 0;
}

// Zeropage Addressing Mode
uint8_t Cpu6502::ZP0() {
    addr_abs = read(pc++);
    addr_abs &= 0x00FF;
    return 0;
}

// Zeropage X-Offset Addressing Mode
uint8_t Cpu6502::ZPX() {
    addr_abs = read(pc++);
    addr_abs += x;
    addr_abs &= 0x00FF;
    return 0;
}

// Zeropage Y-Offset Addressing Mode
uint8_t Cpu6502::ZPY() {
    addr_abs = read(pc++);
    addr_abs += y;
    addr_abs &= 0x00FF;
    return 0;
}

// Instructions functions
