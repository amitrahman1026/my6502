#include "bus.h"
#include "cpu.h"
#include <memory>

Cpu6502::Cpu6502() {
    using In = Instruction6502;
    using C = Cpu6502;
    opcodeTable =
        // clang-format off
{
/*              0                                1                                2                                3                                4                                5                                6                                7                                8                                9                                A                                B                                C                                D                                E                                F                             */
/*0x0*/         In{"BRK", &C::BRK, &C::IMP, 7 }, In{"ORA", &C::ORA, &C::IZX, 6 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"???", &C::XXX, &C::IMP, 3 }, In{"ORA", &C::ORA, &C::ZP0, 3 }, In{"ASL", &C::ASL, &C::ZP0, 5 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"PHP", &C::PHP, &C::IMP, 3 }, In{"ORA", &C::ORA, &C::IMM, 2 }, In{"ASL", &C::ASL, &C::ACC, 2 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"ORA", &C::ORA, &C::ABS, 4 }, In{"ASL", &C::ASL, &C::ABS, 6 }, In{"???", &C::XXX, &C::IMP, 6 },

/*0x1*/         In{"BPL", &C::BPL, &C::REL, 2 }, In{"ORA", &C::ORA, &C::IZY, 5 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"ORA", &C::ORA, &C::ZPX, 4 }, In{"ASL", &C::ASL, &C::ZPX, 6 }, In{"???", &C::XXX, &C::IMP, 6 }, In{"CLC", &C::CLC, &C::IMP, 2 }, In{"ORA", &C::ORA, &C::ABY, 4 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 7 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"ORA", &C::ORA, &C::ABX, 4 }, In{"ASL", &C::ASL, &C::ABX, 7 }, In{"???", &C::XXX, &C::IMP, 7 },

/*0x2*/         In{"JSR", &C::JSR, &C::ABS, 6 }, In{"AND", &C::AND, &C::IZX, 6 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"BIT", &C::BIT, &C::ZP0, 3 }, In{"AND", &C::AND, &C::ZP0, 3 }, In{"ROL", &C::ROL, &C::ZP0, 5 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"PLP", &C::PLP, &C::IMP, 4 }, In{"AND", &C::AND, &C::IMM, 2 }, In{"ROL", &C::ROL, &C::ACC, 2 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"BIT", &C::BIT, &C::ABS, 4 }, In{"AND", &C::AND, &C::ABS, 4 }, In{"ROL", &C::ROL, &C::ABS, 6 }, In{"???", &C::XXX, &C::IMP, 6 },

/*0x3*/         In{"BMI", &C::BMI, &C::REL, 2 }, In{"AND", &C::AND, &C::IZY, 5 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"AND", &C::AND, &C::ZPX, 4 }, In{"ROL", &C::ROL, &C::ZPX, 6 }, In{"???", &C::XXX, &C::IMP, 6 }, In{"SEC", &C::SEC, &C::IMP, 2 }, In{"AND", &C::AND, &C::ABY, 4 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 7 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"AND", &C::AND, &C::ABX, 4 }, In{"ROL", &C::ROL, &C::ABX, 7 }, In{"???", &C::XXX, &C::IMP, 7 },

/*0x4*/         In{"RTI", &C::RTI, &C::IMP, 6 }, In{"EOR", &C::EOR, &C::IZX, 6 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"???", &C::XXX, &C::IMP, 3 }, In{"EOR", &C::EOR, &C::ZP0, 3 }, In{"LSR", &C::LSR, &C::ZP0, 5 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"PHA", &C::PHA, &C::IMP, 3 }, In{"EOR", &C::EOR, &C::IMM, 2 }, In{"LSR", &C::LSR, &C::ACC, 2 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"JMP", &C::JMP, &C::ABS, 3 }, In{"EOR", &C::EOR, &C::ABS, 4 }, In{"LSR", &C::LSR, &C::ABS, 6 }, In{"???", &C::XXX, &C::IMP, 6 },

/*0x5*/         In{"BVC", &C::BVC, &C::REL, 2 }, In{"EOR", &C::EOR, &C::IZY, 5 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"EOR", &C::EOR, &C::ZPX, 4 }, In{"LSR", &C::LSR, &C::ZPX, 6 }, In{"???", &C::XXX, &C::IMP, 6 }, In{"CLI", &C::CLI, &C::IMP, 2 }, In{"EOR", &C::EOR, &C::ABY, 4 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 7 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"EOR", &C::EOR, &C::ABX, 4 }, In{"LSR", &C::LSR, &C::ABX, 7 }, In{"???", &C::XXX, &C::IMP, 7 },

/*0x6*/         In{"RTS", &C::RTS, &C::IMP, 6 }, In{"ADC", &C::ADC, &C::IZX, 6 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"???", &C::XXX, &C::IMP, 3 }, In{"ADC", &C::ADC, &C::ZP0, 3 }, In{"ROR", &C::ROR, &C::ZP0, 5 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"PLA", &C::PLA, &C::IMP, 4 }, In{"ADC", &C::ADC, &C::IMM, 2 }, In{"ROR", &C::ROR, &C::ACC, 2 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"JMP", &C::JMP, &C::IND, 5 }, In{"ADC", &C::ADC, &C::ABS, 4 }, In{"ROR", &C::ROR, &C::ABS, 6 }, In{"???", &C::XXX, &C::IMP, 6 },

/*0x7*/         In{"BVS", &C::BVS, &C::REL, 2 }, In{"ADC", &C::ADC, &C::IZY, 5 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"ADC", &C::ADC, &C::ZPX, 4 }, In{"ROR", &C::ROR, &C::ZPX, 6 }, In{"???", &C::XXX, &C::IMP, 6 }, In{"SEI", &C::SEI, &C::IMP, 2 }, In{"ADC", &C::ADC, &C::ABY, 4 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 7 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"ADC", &C::ADC, &C::ABX, 4 }, In{"ROR", &C::ROR, &C::ABX, 7 }, In{"???", &C::XXX, &C::IMP, 7 },

/*0x8*/         In{"???", &C::XXX, &C::IMP, 2 }, In{"STA", &C::STA, &C::IZX, 6 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 6 }, In{"STY", &C::STY, &C::ZP0, 3 }, In{"STA", &C::STA, &C::ZP0, 3 }, In{"STX", &C::STX, &C::ZP0, 3 }, In{"???", &C::XXX, &C::IMP, 3 }, In{"DEY", &C::DEY, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"TXA", &C::TXA, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"STY", &C::STY, &C::ABS, 4 }, In{"STA", &C::STA, &C::ABS, 4 }, In{"STX", &C::STX, &C::ABS, 4 }, In{"???", &C::XXX, &C::IMP, 4 },

/*0x9*/         In{"BCC", &C::BCC, &C::REL, 2 }, In{"STA", &C::STA, &C::IZY, 6 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 6 }, In{"STY", &C::STY, &C::ZPX, 4 }, In{"STA", &C::STA, &C::ZPX, 4 }, In{"STX", &C::STX, &C::ZPY, 4 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"TYA", &C::TYA, &C::IMP, 2 }, In{"STA", &C::STA, &C::ABY, 5 }, In{"TXS", &C::TXS, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"STA", &C::STA, &C::ABX, 5 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"???", &C::XXX, &C::IMP, 5 },

/*0xA*/         In{"LDY", &C::LDY, &C::IMM, 2 }, In{"LDA", &C::LDA, &C::IZX, 6 }, In{"LDX", &C::LDX, &C::IMM, 2 }, In{"???", &C::XXX, &C::IMP, 6 }, In{"LDY", &C::LDY, &C::ZP0, 3 }, In{"LDA", &C::LDA, &C::ZP0, 3 }, In{"LDX", &C::LDX, &C::ZP0, 3 }, In{"???", &C::XXX, &C::IMP, 3 }, In{"TAY", &C::TAY, &C::IMP, 2 }, In{"LDA", &C::LDA, &C::IMM, 2 }, In{"TAX", &C::TAX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"LDY", &C::LDY, &C::ABS, 4 }, In{"LDA", &C::LDA, &C::ABS, 4 }, In{"LDX", &C::LDX, &C::ABS, 4 }, In{"???", &C::XXX, &C::IMP, 4 },

/*0xB*/         In{"BCS", &C::BCS, &C::REL, 2 }, In{"LDA", &C::LDA, &C::IZY, 5 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"LDY", &C::LDY, &C::ZPX, 4 }, In{"LDA", &C::LDA, &C::ZPX, 4 }, In{"LDX", &C::LDX, &C::ZPY, 4 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"CLV", &C::CLV, &C::IMP, 2 }, In{"LDA", &C::LDA, &C::ABY, 4 }, In{"TSX", &C::TSX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"LDY", &C::LDY, &C::ABX, 4 }, In{"LDA", &C::LDA, &C::ABX, 4 }, In{"LDX", &C::LDX, &C::ABY, 4 }, In{"???", &C::XXX, &C::IMP, 4 },

/*0xC*/         In{"CPY", &C::CPY, &C::IMM, 2 }, In{"CMP", &C::CMP, &C::IZX, 6 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"CPY", &C::CPY, &C::ZP0, 3 }, In{"CMP", &C::CMP, &C::ZP0, 3 }, In{"DEC", &C::DEC, &C::ZP0, 5 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"INY", &C::INY, &C::IMP, 2 }, In{"CMP", &C::CMP, &C::IMM, 2 }, In{"DEX", &C::DEX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"CPY", &C::CPY, &C::ABS, 4 }, In{"CMP", &C::CMP, &C::ABS, 4 }, In{"DEC", &C::DEC, &C::ABS, 6 }, In{"???", &C::XXX, &C::IMP, 6 },

/*0xD*/         In{"BNE", &C::BNE, &C::REL, 2 }, In{"CMP", &C::CMP, &C::IZY, 5 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"CMP", &C::CMP, &C::ZPX, 4 }, In{"DEC", &C::DEC, &C::ZPX, 6 }, In{"???", &C::XXX, &C::IMP, 6 }, In{"CLD", &C::CLD, &C::IMP, 2 }, In{"CMP", &C::CMP, &C::ABY, 4 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 7 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"CMP", &C::CMP, &C::ABX, 4 }, In{"DEC", &C::DEC, &C::ABX, 7 }, In{"???", &C::XXX, &C::IMP, 7 },

/*0xE*/         In{"CPX", &C::CPX, &C::IMM, 2 }, In{"SBC", &C::SBC, &C::IZX, 6 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"CPX", &C::CPX, &C::ZP0, 3 }, In{"SBC", &C::SBC, &C::ZP0, 3 }, In{"INC", &C::INC, &C::ZP0, 5 }, In{"???", &C::XXX, &C::IMP, 5 }, In{"INX", &C::INX, &C::IMP, 2 }, In{"SBC", &C::SBC, &C::IMM, 2 }, In{"NOP", &C::NOP, &C::IMP, 2 }, In{"???", &C::SBC, &C::IMP, 2 }, In{"CPX", &C::CPX, &C::ABS, 4 }, In{"SBC", &C::SBC, &C::ABS, 4 }, In{"INC", &C::INC, &C::ABS, 6 }, In{"???", &C::XXX, &C::IMP, 6 },

/*0xF*/         In{"BEQ", &C::BEQ, &C::REL, 2 }, In{"SBC", &C::SBC, &C::IZY, 5 }, In{"???", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 8 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"SBC", &C::SBC, &C::ZPX, 4 }, In{"INC", &C::INC, &C::ZPX, 6 }, In{"???", &C::XXX, &C::IMP, 6 }, In{"SED", &C::SED, &C::IMP, 2 }, In{"SBC", &C::SBC, &C::ABY, 4 }, In{"NOP", &C::XXX, &C::IMP, 2 }, In{"???", &C::XXX, &C::IMP, 7 }, In{"???", &C::XXX, &C::IMP, 4 }, In{"SBC", &C::SBC, &C::ABX, 4 }, In{"INC", &C::INC, &C::ABX, 7 }, In{"???", &C::XXX, &C::IMP, 7 }

};
    // clang-format on
    reset();
};

void Cpu6502::connectBus(std::shared_ptr<Bus_16bit> b) { bus = b; }

uint8_t Cpu6502::getFlag(Cpu6502::Flags6502 f) { return sr & f; }

void Cpu6502::setFlag(Cpu6502::Flags6502 f) { sr |= f; }

void Cpu6502::clearFlag(Cpu6502::Flags6502 f) { sr &= ~f; }

uint8_t Cpu6502::read(uint16_t addr) { return bus->read(addr); }

void Cpu6502::write(uint16_t addr, uint8_t data) {
    return bus->write(addr, data);
}

void Cpu6502::reset() {
    addr_abs = 0xFFFC;
    addr_lo = read(addr_abs);
    addr_hi = read(addr_abs + 1);

    // clear registers, set program counter to 0xFFFC, stack pointer to 0xFD
    a = 0;
    x = 0;
    y = 0;
    sr = 0x00;
    sp = 0xFD;
    pc = (addr_hi << 8) | addr_lo;

    setFlag(U);

    cycles = 8;
    opcode = 0x00;
    addr_abs = 0x0000;
    addr_rel = 0x0000;
    addr_lo = 0x0000;
    addr_hi = 0x0000;
    decoded = 0x00;
}

void Cpu6502::irq() {
    if (!getFlag(I)) { // Interrupt disable bit check

        write(0x100 + sp, (pc >> 8) & 0x00FF);
        sp--;
        write(0x100 + sp, pc & 0x00FF);
        sp--;

        // push status register to stack
        clearFlag(B);
        setFlag(U);
        setFlag(I);
        write(0x100 + sp, sr);
        sp--;

        // read new program counter
        addr_abs = 0xFFFE;
        addr_lo = read(addr_abs);
        addr_hi = read(addr_abs + 1);
        pc = (addr_hi << 8) | addr_lo;

        cycles = 7;
    }
}

void Cpu6502::nmi() {
    write(0x100 + sp, (pc >> 8) & 0x00FF);
    sp--;
    write(0x100 + sp, pc & 0x00FF);
    sp--;

    // push status register to stack
    clearFlag(B);
    setFlag(U);
    setFlag(I);
    write(0x100 + sp, sr);
    sp--;

    // read new program counter
    addr_abs = 0xFFFA;
    addr_lo = read(addr_abs);
    addr_hi = read(addr_abs + 1);
    pc = (addr_hi << 8) | addr_lo;

    cycles = 8;
}

/**
 * @brief Executes a single clock cycle in the 6502 CPU emulation.
 *
 * Each instruction requires a variable number of clock cycles to execute. In
 * behavioral emulation, calculations are performed instantly, and the clock
 * cycles are separately delayed to emulate the execution time. This delay is
 * implemented by counting down the cycles required by the instruction.
 * When the cycle count reaches 0, the instruction is complete, and the next one
 * is ready to be executed.
 */
void Cpu6502::clock() {
    // If the cycles have expired, fetch the next instruction.
    if (cycles == 0) {
        // Read instruction byte
        opcode = read(pc);
        // Set ununsed flag
        setFlag(U);

        cycles = opcodeTable[opcode].cycles;
        uint8_t additional_cycles_addr =
            (this->*opcodeTable[opcode].address_mode)();
        uint8_t additional_cycles2_inst =
            (this->*opcodeTable[opcode].instruction_type)();

        cycles += (additional_cycles_addr & additional_cycles2_inst);
    }
    cycles--;
}
