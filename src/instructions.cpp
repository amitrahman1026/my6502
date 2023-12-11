#include "cpu.h"
#include <cstdint>
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
/*
    enum class Flags6502 {
        C = 1 << 0, // Carry Bit
        Z = 1 << 1, // Zero
        I = 1 << 2, // Disable Interrupts
        D = 1 << 3, // Decimal Mode
        B = 1 << 4, // Break
        U = 1 << 5, // Unused
        V = 1 << 6, // Overflow
        N = 1 << 7  // Negative
    };
 * */

// Add Memory to Accumulator with Carry
uint8_t Cpu6502::ADC() {
    fetch();
    uint16_t temp = (uint16_t)a + (uint16_t)operand + (uint16_t)getFlag(Flags6502::C);
    (temp > 255) ? setFlag(Flags6502::C) : clearFlag(Flags6502::C);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    ((~((uint16_t)a ^ (uint16_t)operand) & ((uint16_t)a ^ (uint16_t)temp)) &
     0x0080)
        ? setFlag(Flags6502::V)
        : clearFlag(Flags6502::V);
    (temp & 0x80) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    a = temp & 0x00FF;
    return 1;
}

// AND Memory with Accumulator
uint8_t Cpu6502::AND() {
    fetch();
    a &= operand;
    a == 0x00 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    a & 0x80 ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 1;
}

// Shift Left One Bit (Memory or Accumulator);
uint8_t Cpu6502::ASL() {
    fetch();
    uint16_t temp = (uint16_t)operand << 1;
    (temp & 0xFF00) > 0 ? setFlag(Flags6502::C) : clearFlag(Flags6502::C);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (temp & 0x80) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    if (opcodeTable[opcode].address_mode == &Cpu6502::ACC) a = temp & 0x00FF;
    else write(addr_abs, temp & 0x00FF);
    return 0;
}

// Branch on Carry Clear
uint8_t Cpu6502::BCC() {
    if (getFlag(Flags6502::C) == 0) {
        cycles++;
        addr_abs = pc + addr_rel;
        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}

// Branch on Carry Set
uint8_t Cpu6502::BCS() {
    if (getFlag(Flags6502::C) == 1) {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}

// Branch on Result Zero
uint8_t Cpu6502::BEQ() {
    if (getFlag(Flags6502::Z) == 1) {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}

// Test Bits in Memory with Accumulator
uint8_t Cpu6502::BIT() {
    fetch();
    uint16_t temp = a & operand;
    (temp & 0x00FF) == 0x00 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    operand & (1 << 7) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    operand & (1 << 6) ? setFlag(Flags6502::V) : clearFlag(Flags6502::V);
    return 0;
}

// Branch on Result Minus
uint8_t Cpu6502::BMI() {
    if (getFlag(Flags6502::N) == 1) {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}

// Branch on Result Not Zero
uint8_t Cpu6502::BNE() {
    if (getFlag(Flags6502::Z) == 0) {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}

// Branch on Result Plus
uint8_t Cpu6502::BPL() {
    if (getFlag(Flags6502::N) == 0) {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}

// Force Break
uint8_t Cpu6502::BRK() {
    pc++;

    setFlag(Flags6502::I);
    write(0x0100 + sp, (pc >> 8) & 0x00FF);
    sp--;
    write(0x0100 + sp, pc & 0x00FF);
    sp--;

    setFlag(Flags6502::B);
    write(0x0100 + sp, sr);
    sp--;
    clearFlag(Flags6502::B);

    pc = (uint16_t)read(0xFFFE) | ((uint16_t)read(0xFFFF) << 8);
    return 0;
}

// Branch on Overflow Clear
uint8_t Cpu6502::BVC() {
    if (getFlag(Flags6502::V) == 0) {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}

// Brach on Overflow Set
uint8_t Cpu6502::BVS() {
    if (getFlag(Flags6502::V) == 1) {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00)) cycles++;

        pc = addr_abs;
    }
    return 0;
}

// Clear Carry Flag
uint8_t Cpu6502::CLC() {
    clearFlag(Flags6502::C);
    return 0;
}

// Clear Decimal Mode
uint8_t Cpu6502::CLD() {
    clearFlag(Flags6502::D);
    return 0;
}

// Clear Interrupt Disable Bit
uint8_t Cpu6502::CLI() {
    clearFlag(Flags6502::I);
    return 0;
}

// Clear Overflow Flag
uint8_t Cpu6502::CLV() {
    clearFlag(Flags6502::V);
    return 0;
}

// Compare Memory with Accumulator
uint8_t Cpu6502::CMP() {
    fetch();
    uint16_t temp = (uint16_t)a - (uint16_t)operand;
    (a >= operand) ? setFlag(Flags6502::C) : clearFlag(Flags6502::C);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (temp & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 1;
}

// Compare Memory with Index X
uint8_t Cpu6502::CPX() {
    fetch();
    uint16_t temp = (uint16_t)x - (uint16_t)operand;
    (x >= operand) ? setFlag(Flags6502::C) : clearFlag(Flags6502::C);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (temp & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Compare Memry with Index Y
uint8_t Cpu6502::CPY() {
    fetch();
    uint16_t temp = (uint16_t)y - (uint16_t)operand;
    (y >= operand) ? setFlag(Flags6502::C) : clearFlag(Flags6502::C);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (temp & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Decrement Memory
uint8_t Cpu6502::DEC() {
    fetch();
    uint16_t temp = operand - 1;
    write(addr_abs, temp & 0x00FF);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (temp & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Decrement Index X
uint8_t Cpu6502::DEX() {
    x--;
    x == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (x & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Decrement Index Y
uint8_t Cpu6502::DEY() {
    y--;
    y == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (y & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Exclusive OR with Accumulator
uint8_t Cpu6502::EOR() {
    fetch();
    a = a ^ operand;
    a == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (a & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 1;
}

// Increment Memory
uint8_t Cpu6502::INC() {
    fetch();
    uint16_t temp = operand + 1;
    write(addr_abs, temp & 0x00FF);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (temp & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Increment Index X
uint8_t Cpu6502::INX() {
    x++;
    x == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (x & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Increment Index Y
uint8_t Cpu6502::INY() {
    y++;
    y == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (y & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Jump to New Location
uint8_t Cpu6502::JMP() {
    pc = addr_abs;
    return 0;
}

// Jump to New Location with Saving Return Address
uint8_t Cpu6502::JSR() {
    pc--;

    write(0x0100 + sp, (pc >> 8) & 0x00FF);
    sp--;
    write(0x0100 + sp, pc & 0x00FF);
    sp--;

    pc = addr_abs;
    return 0;
}

// Load Accumulator with Memory
uint8_t Cpu6502::LDA() {
    fetch();
    a = operand;
    a == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (a & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 1;
}

// Load Index X with Memory
uint8_t Cpu6502::LDX() {
    fetch();
    x = operand;
    x == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (x & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 1;
}

// Load Intex Y with Memory
uint8_t Cpu6502::LDY() {
    fetch();
    y = operand;
    y == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (y & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 1;
}

// Shift One Bit Right (Memory or Accumulator)
uint8_t Cpu6502::LSR() {
    fetch();
    (operand & 0x0001) ? setFlag(Flags6502::C) : clearFlag(Flags6502::C);
    uint16_t temp = operand >> 1;
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (temp & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    if (opcodeTable[opcode].address_mode == &Cpu6502::IMP) a = temp & 0x00FF;
    else write(addr_abs, temp & 0x00FF);
    return 0;
}

// No Operation
uint8_t Cpu6502::NOP() {
    // potential to add in illegal opcode's effects here
    return 0;
}

// OR Memeory with Accumulator
uint8_t Cpu6502::ORA() {
    fetch();
    a = a | operand;
    a == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (a & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 1;
}

// Push Accumulator on Stack
uint8_t Cpu6502::PHA() {
    write(0x0100 + sp, a);
    sp--;
    return 0;
}

// Push Processor Status on Stack
uint8_t Cpu6502::PHP() {
    write(0x0100 + sp, (sr | (uint8_t)Flags6502::B | (uint8_t)Flags6502::U));
    clearFlag(Flags6502::B);
    clearFlag(Flags6502::U);
    sp--;
    return 0;
}

// Pull Accumulator from Stack
uint8_t Cpu6502::PLA() {
    sp++;
    a = read(0x0100 + sp);
    a == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (a & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Pull Processor Status from Stack
uint8_t Cpu6502::PLP() {
    sp++;
    sr = read(0x0100 + sp);
    setFlag(Flags6502::U);
    return 0;
}

// Rotate One Bit Left (Memory or Accumulator)
uint8_t Cpu6502::ROL() {
    fetch();
    uint16_t temp = (uint16_t)(operand << 1) | getFlag(Flags6502::C);
    (temp & 0xFF00) ? setFlag(Flags6502::C) : clearFlag(Flags6502::C);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (temp & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    if (opcodeTable[opcode].address_mode == &Cpu6502::IMP) a = temp & 0x00FF;
    else write(addr_abs, temp & 0x00FF);
    return 0;
}

// Rotate One Bit Right (Memory or Accumulator)
uint8_t Cpu6502::ROR() {
    fetch();
    uint16_t temp = (uint16_t)(getFlag(Flags6502::C) << 7) | (operand >> 1);
    (operand & 0x01) ? setFlag(Flags6502::C) : clearFlag(Flags6502::C);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    (temp & 0x0080) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    if (opcodeTable[opcode].address_mode == &Cpu6502::IMP) a = temp & 0x00FF;
    else write(addr_abs, temp & 0x00FF);
    return 0;
}

// Return from Interrupt
uint8_t Cpu6502::RTI() {
    sp++;
    sr = read(0x0100 + sp);
    sr &= ~(uint8_t)Flags6502::B;
    sr &= ~(uint8_t)Flags6502::U;

    sp++;
    pc = (uint16_t)read(0x0100 + sp);
    sp++;
    pc |= (uint16_t)read(0x0100 + sp) << 8;
    return 0;
}

// Return from Subroutine
uint8_t Cpu6502::RTS() {
    sp++;
    pc = (uint16_t)read(0x0100 + sp);
    sp++;
    pc |= (uint16_t)read(0x0100 + sp) << 8;

    pc++;
    return 0;
}

// Subtract Memory from Accumulator with Borrow
uint8_t Cpu6502::SBC() {
    fetch();
    uint16_t temp =
        (uint16_t)a + (uint16_t)operand ^ 0x00FF + (uint16_t)getFlag(Flags6502::C);
    (temp & 0xFF00) ? setFlag(Flags6502::C) : clearFlag(Flags6502::C);
    (temp & 0x00FF) == 0 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    ((~((uint16_t)a ^ (uint16_t)operand) & ((uint16_t)a ^ (uint16_t)temp)) &
     0x0080)
        ? setFlag(Flags6502::V)
        : clearFlag(Flags6502::V);
    (temp & 0x80) ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    a = temp & 0x00FF;
    return 1;
}

// Set Carry Flag
uint8_t Cpu6502::SEC() {
    setFlag(Flags6502::C);
    return 0;
}

// Set Decimal Flag
uint8_t Cpu6502::SED() {
    setFlag(Flags6502::D);
    return 0;
}

// Set Interrupt Disable Status
uint8_t Cpu6502::SEI() {
    setFlag(Flags6502::I);
    return 0;
}

// Store Accumulator in Memory
uint8_t Cpu6502::STA() {
    write(addr_abs, a);
    return 0;
}

// Store Index X in Memory
uint8_t Cpu6502::STX() {
    write(addr_abs, x);
    return 0;
}

// Store Index Y in Memory
uint8_t Cpu6502::STY() {
    write(addr_abs, y);
    return 0;
}

// Transfer Accumulator to Index X
uint8_t Cpu6502::TAX() {
    x = a;
    x == 0x00 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    x & 0x80 ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Transfer Accumulator to Index Y
uint8_t Cpu6502::TAY() {
    y = a;
    y == 0x00 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    y & 0x80 ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Transfer Stack Pointer to Index X
uint8_t Cpu6502::TSX() {
    x = sp;
    x == 0x00 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    x & 0x80 ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Transfer Index X to Accumulator
uint8_t Cpu6502::TXA() {
    a = x;
    a == 0x00 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    a == 0x80 ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Transfer Index X to Stack Register
uint8_t Cpu6502::TXS() {
    sp = x;
    return 0;
}

// Transfer Index Y to Accumulator
uint8_t Cpu6502::TYA() {
    a = y;
    a == 0x00 ? setFlag(Flags6502::Z) : clearFlag(Flags6502::Z);
    a == 0x80 ? setFlag(Flags6502::N) : clearFlag(Flags6502::N);
    return 0;
}

// Illegal opcodes
uint8_t Cpu6502::XXX() { return 0; }
