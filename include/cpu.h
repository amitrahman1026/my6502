#pragma once

#include <_types/_uint8_t.h>
#include <array>
#include <cstdint>
#include <string>

// Forward declarations of all components of 6502 processor to prevent circular
// dependencies

class Bus_16bit;

class Cpu6502 {
public:
    Cpu6502();
    ~Cpu6502() = default;

    /**
     * @brief All the core registers in the 6502
     * */
    uint8_t a = 0x00;     // A Register
    uint8_t x = 0x00;     // X Register
    uint8_t y = 0x00;     // Y Register
    uint8_t sr = 0x00;    // Status Register
    uint16_t sp = 0x0000; // Stack Pointer
    uint16_t pc = 0x0000; // Program Counter

    /**
     * @brief Masks for manipulating 6502 flag register
     * */
    enum class Flags6502 : uint8_t {
        C = 1 << 0, // Carry Bit
        Z = 1 << 1, // Zero
        I = 1 << 2, // Disable Interrupts
        D = 1 << 3, // Decimal Mode
        B = 1 << 4, // Break
        U = 1 << 5, // Unused
        V = 1 << 6, // Overflow
        N = 1 << 7  // Negative
    };

    /**
     * @brief Events and interrupts
     * */
    void reset(); // Reset cpu to known state
    void clock(); // Update by one clock cycle
    void irq();   // Interrupt request
    void nmi();   // Non-maskable interrupt
    void ret();   // instruction is complete

    /**
     * @brief 16 Bit address but and utilities
     * */
    Bus_16bit *bus = nullptr;

    /**
     * @brief Instructions,lookup table, addressing modes, opcodes
     * https://www.masswerk.at/6502/6502_instruction_set.html
     * */
    struct Instruction6502 {
        std::string opcode;
        uint8_t (Cpu6502::*address_mode)();
        uint8_t (Cpu6502::*instruction_type)();
        uint8_t cycles = 0;
    };

    std::array<Instruction6502, 0x100> opcodeTable;

    // Instructions types
    uint8_t ADC(); // Add Memory to Accumulator with Carry
    uint8_t AND(); // AND Memory with Accumulator
    uint8_t ASL(); // Shift Left One Bit (Memory or Accumulator)

    uint8_t BCC(); // Branch on Carry Clear
    uint8_t BCS(); // Branch on Carry Set
    uint8_t BEQ(); // Branch on Result Zero
    uint8_t BIT(); // Test Bits in Memory with Accumulator
    uint8_t BMI(); // Branch on Result Minus
    uint8_t BNE(); // Branch on Result not Zero
    uint8_t BPL(); // Branch on Result Plus
    uint8_t BRK(); // Force Break
    uint8_t BVC(); // Branch on Overflow Clear
    uint8_t BVS(); // Branch on Overflow Set

    uint8_t CLC(); // Clear Carry Flag
    uint8_t CLD(); // Clear Decimal Mode
    uint8_t CLI(); // Clear Interrupt Disable Bit
    uint8_t CLV(); // Clear Overflow Flag
    uint8_t CMP(); // Compare Memory with Accumulator
    uint8_t CPX(); // Compare Memory and Index X
    uint8_t CPY(); // Compare Memory and Index Y

    uint8_t DEC(); // Decrement Memory by 1
    uint8_t DEX(); // Decrement Index X by 1
    uint8_t DEY(); // Decrement Index Y by 1

    uint8_t EOR(); // Exclusive OR Memory with Accumulator

    uint8_t INC(); // Increment Memory by 1
    uint8_t INX(); // Increment Index X by 1
    uint8_t INY(); // Increment Index Y by 1

    uint8_t JMP(); // Jump to New Location
    uint8_t JSR(); // Jump to New Location Saving Return Address

    uint8_t LDA(); // Load Accumulator with Memory
    uint8_t LDX(); // Load Index X with Memory
    uint8_t LDY(); // Load Index Y with Memory
    uint8_t LSR(); // Shift One Bit Right (Memory or Accumulator

    uint8_t NOP(); // No Operation

    uint8_t ORA(); // OR Memory with Accumulator

    uint8_t PHA(); // Push Accumulator on Stack
    uint8_t PHP(); // Push Processor Status on Stack
    uint8_t PLA(); // Pull Accumulator from Stack
    uint8_t PLP(); // Pull Processor Status from Stack

    uint8_t ROL(); // Rotate One Bit Left (Memory or Accumulator)
    uint8_t ROR(); // Rotate One Bit Right (Memory or Accumulator)
    uint8_t RTI(); // Return from Interrupt
    uint8_t RTS(); // Return from Subroutine

    uint8_t SBC(); // Subtract Memory from Accumulator with Borrow
    uint8_t SEC(); // Set Carry Flag
    uint8_t SED(); // Set Decimal Flag
    uint8_t SEI(); // Set Interrupt Disable Status
    uint8_t STA(); // Store Accumulator in Memory
    uint8_t STX(); // Store Index X in Memory
    uint8_t STY(); // Store Index Y in Memory

    uint8_t TAX(); // Transfer Accumulator to Index X
    uint8_t TAY(); // Transfer Accumulator to Index Y
    uint8_t TSX(); // Transfer Stack Pointer to Index X
    uint8_t TXA(); // Transfer Stack Pointer to Accumulator
    uint8_t TXS(); // Transfer Index X to Stack Register
    uint8_t TYA(); // Transfer Index Y to Accumulator

    uint8_t XXX(); // Illegal opcode will be captured in this

    // Address modes
    uint8_t ACC(); // Accumulator
    uint8_t ABS(); // Absolute
    uint8_t ABX(); // Absolute, X-Indexed
    uint8_t ABY(); // Absolute, Y-Indexed
    uint8_t IMM(); // Immediate
    uint8_t IMP(); // Implied
    uint8_t IND(); // Indirect
    uint8_t IZX(); // X-Indexed, Indirect
    uint8_t IZY(); // Indirect, Y-Indexed
    uint8_t REL(); // Relative
    uint8_t ZP0(); // Zeropage
    uint8_t ZPX(); // Zeropage, X-Indexed
    uint8_t ZPY(); // Zeropage, Y-Indexed

private:
    /**
     * @brief
     * */
};
