#ifndef CPU_HPP
#define CPU_HPP

/**********************************************************
 * This class implements the MOS 6502 CPU.                *
 * NES CPU is Ricoh 2A03, and it's based on the MOS CPU.  *
 * ****************************************************** *
 * The CPU is Little Endian.                              *
 * ****************************************************** *
 * The CPU has 6 registers:                               *
 * - Accumulator - doing arithemtic and logic.            *
 * - Index Registers - used for Indirect Addressing and   *
 *   Counters and Indexes.                                *
 * - Stack Pointer - stores the LSB ( Byte ) of the top   *
 *   of the stack.                                        *
 * - Processor Status - each bit contains a different     *
 *   state that executed.                                 *
 * ****************************************************** *
 * Memory Map layout:                                     *
 * - $0000-$00FF - used by the zero page instructions.    *
 * - $0100-$01FF - reserved for the stack.                *
 * - $0200-$FFF9 - unspecified.                           *
 * - $FFFA-$FFFB - contains address of NMI handler.       *
 * - $FFFC-$FFFD - contains address of reset location.    *
 * - $FFFE-$FFFF - contains address of BRK/IRQ handler.   *
 * ****************************************************** *
 * The CPU has 13 Addressing Modes:                       *
 * - Implicit - the source and destination of the         *
 *   information to be manipulated is implied directly    *
 *   by the function of the instruction itself and no     *
 *   further operand needs to be specified.               *
 * - Accumulator - some instructions have an option to    *
 *   operate directly upon the accumulator.               *
 * - Immediate - allows the programmer to directly        *
 *   specify an 8 bit constant within the instruction.    *
 * - Zero Page - an instruction using zero page           *
 *   addressing mode has only an 8 bit address operand.   *
 * - Zero Page X - same as Zero Page but with addition    *
 *   of the X register.                                   *
 * - Zero Page Y - same as Zero Page but with addition    *
 *   of the Y register.                                   *
 * - Relative - used by branch instructions.              *
 * - Absolute - contain a full 16 bit address to identify *
 *   the target location.                                 *
 * - Absolute X - same as Absolute but with an addition   *
 *   of the X register.                                   *
 * - Absolute Y - same as Absolute but with an addition   *
 *   of the Y register.                                   *
 * - Indirect - contains a 16bit address which identifies *
 *   the location of the least significant byte of        *
 *   another 16bit memory address which is the real       *
 *   target of the instruction.                           *
 * - Indexed Indirect - used in conjunction with a table  *
 *   of address held on zero page.                        *
 * - Indirect Indexed - most common indirection mode used *
 *   on the 6502.                                         *
 * ****************************************************** *
 * All of the instructions are more explained in the      *
 * translation unit.                                      *
 * ****************************************************** *
 * http://www.obelisk.me.uk/6502/reference.html           *
 **********************************************************/

#include <memory>
#include <vector>

// include the bus header to handle the CPU
// #include "bus.hpp"

#if __cplusplus >= 201703L
#   define MOS_NODISCARD    [[nodiscard]]
#else
#   if defined(__GNUC__) || defined(__clang__)
#       define MOS_NODISCARD    __attribute__((warn_unused_result))
#   elif defined(_MSC_VER >= 1700) // >= VS 2012 
#       define MOS_NODISCARD    _Check_return_
#   else
#       define MOS_NODISCARD
#   endif
#endif

class CPU
{
private:
    static constexpr auto SIZE = 0x800;
    static constexpr auto STACK_LOCATION = 0x0100;

public:
    CPU(/*Bus* bus_ptr*/);
    ~CPU();

    // execute the current instruction
    void execute() noexcept;

    void IRQ()   noexcept;
    void NMI()   noexcept;
    void reset() noexcept;

private:
    // --- Stack --- //
    void push_stack(uint8_t val);
    uint8_t pop_stack();

    // --- Addressing Modes --- //
    uint16_t implicit();
    uint16_t accumulator();
    uint16_t immediate();
    uint16_t zero_page();
    uint16_t zero_page_x();
    uint16_t zero_page_y();
    uint16_t relative();
    uint16_t absolute();
    uint16_t absolute_x();
    uint16_t absolute_y();
    uint16_t indirect();
    uint16_t indexed_indirect();
    uint16_t indirect_indexed();

    // --- Instructions --- //
    // --- Load Operations ---
    uint8_t LDA(uint16_t addr); // Load Accumulator
    uint8_t LDX(uint16_t addr); // Load X Register
    uint8_t LDY(uint16_t addr); // Load Y Register

    // --- Store Operations ---
    uint8_t STA(uint16_t addr); // Store Accumulator
    uint8_t STX(uint16_t addr); // Store X Register
    uint8_t STY(uint16_t addr); // Store Y Register

    // --- Register Transfers ---
    uint8_t TAX(uint16_t addr); // Transfer accumulator to X
    uint8_t TAY(uint16_t addr); // Transfer accumulator to Y
    uint8_t TXA(uint16_t addr); // Transfer X to accumulator
    uint8_t TYA(uint16_t addr); // Transfer Y to accumulator

    // --- Stack Operations ---
    uint8_t TSX(uint16_t addr); // Transfer stack pointer to X
    uint8_t TXS(uint16_t addr); // Transfer X to stack pointer
    uint8_t PHA(uint16_t addr); // Push accumulator on stack
    uint8_t PHP(uint16_t addr); // Push processor status on stack
    uint8_t PLA(uint16_t addr); // Pull accumulator from stack
    uint8_t PLP(uint16_t addr); // Pull processor status from stack

    // --- Logical ---
    uint8_t AND(uint16_t addr); // Logical AND
    uint8_t EOR(uint16_t addr); // Exclusive OR
    uint8_t ORA(uint16_t addr); // Logical Inclusive OR
    uint8_t BIT(uint16_t addr); // Bit Test

    // --- Arithmetic ---
    uint8_t ADC(uint16_t addr); // Add with Carry
    uint8_t SBC(uint16_t addr); // Subtract with Carry
    uint8_t CMP(uint16_t addr); // Compare accumulator
    uint8_t CPX(uint16_t addr); // Compare X register
    uint8_t CPY(uint16_t addr); // Compare Y register

    // --- Increments and Decrements ---
    uint8_t INC(uint16_t addr); // Increment a memory location
    uint8_t INX(uint16_t addr); // Increment the X register
    uint8_t INY(uint16_t addr); // Increment the Y register
    uint8_t DEC(uint16_t addr); // Decrement a memory location
    uint8_t DEX(uint16_t addr); // Decrement the X register
    uint8_t DEY(uint16_t addr); // Decrement the Y register

    // --- Shifts ---
    uint8_t ASL(uint16_t addr); // Arithmetic Shift Left
    uint8_t LSR(uint16_t addr); // Logical Shift Right
    uint8_t ROL(uint16_t addr); // Rotate Left
    uint8_t ROR(uint16_t addr); // Rotate Right

    // --- Jumps and Calls ---
    uint8_t JMP(uint16_t addr); // Jump to another location
    uint8_t JSR(uint16_t addr); // Jump to a subroutine
    uint8_t RTS(uint16_t addr); // Return from subroutine

    // --- Branches ---
    uint8_t BCC(uint16_t addr); // Branch if carry flag clear
    uint8_t BCS(uint16_t addr); // Branch if carry flag set
    uint8_t BEQ(uint16_t addr); // Branch if zero flag set
    uint8_t BMI(uint16_t addr); // Branch if negative flag set
    uint8_t BNE(uint16_t addr); // Branch if zero flag clear
    uint8_t BPL(uint16_t addr); // Branch if negative flag clear
    uint8_t BVC(uint16_t addr); // Branch if overflow flag clear
    uint8_t BVS(uint16_t addr); // Branch if overflow flag set

    // --- Status Flag Changes ---
    uint8_t CLC(uint16_t addr); // Clear carry flag
    uint8_t CLD(uint16_t addr); // Clear decimal mode flag
    uint8_t CLI(uint16_t addr); // Clear interrupt disable flag
    uint8_t CLV(uint16_t addr); // Clear overflow flag
    uint8_t SEC(uint16_t addr); // Set carry flag
    uint8_t SED(uint16_t addr); // Set decimal mode flag
    uint8_t SEI(uint16_t addr); // Set interrupt disable flag

    // --- System Functions ---
    uint8_t BRK(uint16_t addr); // Force an interrupt
    uint8_t NOP(uint16_t addr); // No Operation
    uint8_t RTI(uint16_t addr); // Return from Interrup

    // --- Checkers --- //
    MOS_NODISCARD
    constexpr bool check_page_crossed_with_offset(uint16_t addr, uint8_t offset) const 
    { 
        return (addr & 0xFF00) != ((addr - offset) & 0xFF00); 
    }

    MOS_NODISCARD
    constexpr bool check_branch_succeeds(uint16_t addr) const 
    {
        return (addr & 0xFF00) != (PC & 0xFF00); 
    }

private:
    // PIMPL idiom
    struct Flags_impl;
    struct Opcode_impl;

    using Flags_ptr  = std::unique_ptr<Flags_impl>;
    using Opcode_ptr = std::unique_ptr<Opcode_impl>;

    uint8_t   A;  // accumulator
    uint8_t   X;  // X register
    uint8_t   Y;  // Y register
    uint8_t   SP; // stack pointer
    uint16_t  PC; // program counter
    Flags_ptr P;  // processing status

    // Opcode table
    std::array<Opcode_ptr, /*max cpu opcodes*/ 0xFF> opcode_table;
    Opcode_impl* current_opcode;

    // CPU clocks
    size_t clocks;

    // Linker between components
    // Bus* bus;
    // friend class Bus;

}; // CPU

#endif // CPU_HPP
