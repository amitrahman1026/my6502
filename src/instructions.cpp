#include "cpu.h"
#include "instructions.h"

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

 
