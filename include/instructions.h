#pragma once
#include "cpu.h"
#include <memory>

class InstructionFactory {
public:
    InstructionFactory(std::shared_ptr<Cpu6502> c);
    ~InstructionFactory();

    void setCustomInstruction(uint8_t opcode, Cpu6502::Instruction6502 in);

private:
    std::shared_ptr<Cpu6502> cpu;
    bool isLegalInstruction(uint8_t addr);
};
