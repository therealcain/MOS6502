// Author: Eviatar Mor, 2019

#pragma once

#include "bus.hpp"

#include <array>
#include <memory>
#include <vector>

class CPU {
private:
	static constexpr uint32_t STACK_LOCATION = 0x0100;

public:
	CPU( Bus& rBus );
	~CPU();

	// execute the current instruction
	void execute() noexcept;

	void IRQ() noexcept;
	void NMI() noexcept;
	void reset() noexcept;

private:
	// --- Stack --- //
	void	push_stack( uint8_t val );
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
	uint8_t LDA( [[maybe_unused]] uint16_t addr ); // Load Accumulator
	uint8_t LDX( [[maybe_unused]] uint16_t addr ); // Load X Register
	uint8_t LDY( [[maybe_unused]] uint16_t addr ); // Load Y Register

	// --- Store Operations ---
	uint8_t STA( [[maybe_unused]] uint16_t addr ); // Store Accumulator
	uint8_t STX( [[maybe_unused]] uint16_t addr ); // Store X Register
	uint8_t STY( [[maybe_unused]] uint16_t addr ); // Store Y Register

	// --- Register Transfers ---
	uint8_t TAX( [[maybe_unused]] uint16_t addr ); // Transfer accumulator to X
	uint8_t TAY( [[maybe_unused]] uint16_t addr ); // Transfer accumulator to Y
	uint8_t TXA( [[maybe_unused]] uint16_t addr ); // Transfer X to accumulator
	uint8_t TYA( [[maybe_unused]] uint16_t addr ); // Transfer Y to accumulator

	// --- Stack Operations ---
	uint8_t TSX( [[maybe_unused]] uint16_t addr ); // Transfer stack pointer to X
	uint8_t TXS( [[maybe_unused]] uint16_t addr ); // Transfer X to stack pointer
	uint8_t PHA( [[maybe_unused]] uint16_t addr ); // Push accumulator on stack
	uint8_t PHP( [[maybe_unused]] uint16_t addr ); // Push processor status on stack
	uint8_t PLA( [[maybe_unused]] uint16_t addr ); // Pull accumulator from stack
	uint8_t PLP( [[maybe_unused]] uint16_t addr ); // Pull processor status from stack

	// --- Logical ---
	uint8_t AND( [[maybe_unused]] uint16_t addr ); // Logical AND
	uint8_t EOR( [[maybe_unused]] uint16_t addr ); // Exclusive OR
	uint8_t ORA( [[maybe_unused]] uint16_t addr ); // Logical Inclusive OR
	uint8_t BIT( [[maybe_unused]] uint16_t addr ); // Bit Test

	// --- Arithmetic ---
	uint8_t ADC( [[maybe_unused]] uint16_t addr ); // Add with Carry
	uint8_t SBC( [[maybe_unused]] uint16_t addr ); // Subtract with Carry
	uint8_t CMP( [[maybe_unused]] uint16_t addr ); // Compare accumulator
	uint8_t CPX( [[maybe_unused]] uint16_t addr ); // Compare X register
	uint8_t CPY( [[maybe_unused]] uint16_t addr ); // Compare Y register

	// --- Increments and Decrements ---
	uint8_t INC( [[maybe_unused]] uint16_t addr ); // Increment a memory location
	uint8_t INX( [[maybe_unused]] uint16_t addr ); // Increment the X register
	uint8_t INY( [[maybe_unused]] uint16_t addr ); // Increment the Y register
	uint8_t DEC( [[maybe_unused]] uint16_t addr ); // Decrement a memory location
	uint8_t DEX( [[maybe_unused]] uint16_t addr ); // Decrement the X register
	uint8_t DEY( [[maybe_unused]] uint16_t addr ); // Decrement the Y register

	// --- Shifts ---
	uint8_t ASL( [[maybe_unused]] uint16_t addr ); // Arithmetic Shift Left
	uint8_t LSR( [[maybe_unused]] uint16_t addr ); // Logical Shift Right
	uint8_t ROL( [[maybe_unused]] uint16_t addr ); // Rotate Left
	uint8_t ROR( [[maybe_unused]] uint16_t addr ); // Rotate Right

	// --- Jumps and Calls ---
	uint8_t JMP( [[maybe_unused]] uint16_t addr ); // Jump to another location
	uint8_t JSR( [[maybe_unused]] uint16_t addr ); // Jump to a subroutine
	uint8_t RTS( [[maybe_unused]] uint16_t addr ); // Return from subroutine

	// --- Branches ---
	uint8_t BCC( [[maybe_unused]] uint16_t addr ); // Branch if carry flag clear
	uint8_t BCS( [[maybe_unused]] uint16_t addr ); // Branch if carry flag set
	uint8_t BEQ( [[maybe_unused]] uint16_t addr ); // Branch if zero flag set
	uint8_t BMI( [[maybe_unused]] uint16_t addr ); // Branch if negative flag set
	uint8_t BNE( [[maybe_unused]] uint16_t addr ); // Branch if zero flag clear
	uint8_t BPL( [[maybe_unused]] uint16_t addr ); // Branch if negative flag clear
	uint8_t BVC( [[maybe_unused]] uint16_t addr ); // Branch if overflow flag clear
	uint8_t BVS( [[maybe_unused]] uint16_t addr ); // Branch if overflow flag set

	// --- Status Flag Changes ---
	uint8_t CLC( [[maybe_unused]] uint16_t addr ); // Clear carry flag
	uint8_t CLD( [[maybe_unused]] uint16_t addr ); // Clear decimal mode flag
	uint8_t CLI( [[maybe_unused]] uint16_t addr ); // Clear interrupt disable flag
	uint8_t CLV( [[maybe_unused]] uint16_t addr ); // Clear overflow flag
	uint8_t SEC( [[maybe_unused]] uint16_t addr ); // Set carry flag
	uint8_t SED( [[maybe_unused]] uint16_t addr ); // Set decimal mode flag
	uint8_t SEI( [[maybe_unused]] uint16_t addr ); // Set interrupt disable flag

	// --- System Functions ---
	uint8_t BRK( [[maybe_unused]] uint16_t addr ); // Force an interrupt
	uint8_t NOP( [[maybe_unused]] uint16_t addr ); // No Operation
	uint8_t RTI( [[maybe_unused]] uint16_t addr ); // Return from Interrup

	// --- Checkers --- //
	inline bool check_page_crossed_with_offset( uint16_t addr, uint8_t offset ) const { return ( addr & 0xFF00 ) != ( ( addr - offset ) & 0xFF00 ); }
	inline bool check_branch_succeeds( uint16_t addr ) const { return ( addr & 0xFF00 ) != ( PC & 0xFF00 ); }

private:
	// PIMPL idiom
	struct FlagsImpl;
	struct OpcodeImpl;

	using pFlags = std::unique_ptr<FlagsImpl>;
	using pOpcode = std::unique_ptr<OpcodeImpl>;

	uint8_t	 A; // accumulator
	uint8_t	 X; // X register
	uint8_t	 Y; // Y register
	uint8_t	 SP; // stack pointer
	uint16_t PC; // program counter
	pFlags	 P; // processing status

	// Opcode table
	std::array<pOpcode, /*max cpu opcodes*/ 0xFF> opcode_table;
	OpcodeImpl*									  current_opcode;

	// CPU clocks
	size_t clocks;

	// Linker between components
	Bus& bus;
}; // CPU
