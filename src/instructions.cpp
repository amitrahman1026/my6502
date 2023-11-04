#include "cpu.h"
#include "instructions.h"

InstructionFactory::InstructionFactory(std::shared_ptr<Cpu6502> c) { cpu = c; }

// Usage
// std::shared_ptr<Cpu6502> cpu = std::make_shared<Cpu6502>();
// InstructionFactory factory(cpu);

void InstructionFactory::setCustomInstruction(uint8_t addr,
                                              Cpu6502::Instruction6502 in) {
    if (cpu->opcodeTable[addr].opcode != "???") {
#warning "Overriding legal 6502 instruction set. May cause unxpected behaviour in emulation"
    }
    cpu->opcodeTable[addr] = in;
}
