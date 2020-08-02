#include "cpu.hpp"

#include <bitset>
#include <algorithm>
#include <iostream>

#if __cplusplus >= 201703L
#   define MOS_UNUSED   MOS_UNUSED
#else
#   if defined(__GNUC__) || defined(__clang__)
#       define MOS_UNUSED    __attribute__((gnu::unused))
#   else
#       define MOS_UNUSED
#   endif
#endif

#define MOS_EXECUTE_CLASS_FUNCTION_POINTER(func)         (this->*func)()
#define MOS_EXECUTE_CLASS_FUNCTION_POINTER(func, param)  (this->*func)(param)

// PIMPL idiom
struct CPU::Flags_impl
{
public:
    Flags_impl(uint8_t value) { bits = value; }
    Flags_impl() = default;

public:
    enum 
    {
        CARRY     = (1 << 0), // 0x1
        ZERO      = (1 << 1), // 0x2
        INTERRUPT = (1 << 2), // 0x4
        DECIMAL   = (1 << 3), // 0x8
        BREAK     = (1 << 4), // 0x10
        UNUSED    = (1 << 5), // 0x20
        OVERFLOW  = (1 << 6), // 0x40
        NEGATIVE  = (1 << 7)  // 0x80
    };

    MOS_NODISCARD
    constexpr bool get_carry()     const { return bits[0]; }

    MOS_NODISCARD
    constexpr bool get_zero()      const { return bits[1]; }

    MOS_NODISCARD
    constexpr bool get_interrupt() const { return bits[2]; }

    MOS_NODISCARD
    constexpr bool get_decimal()   const { return bits[3]; }

    MOS_NODISCARD
    constexpr bool get_break()     const { return bits[4]; }

    MOS_NODISCARD
    constexpr bool get_unused()    const { return bits[5]; }

    MOS_NODISCARD
    constexpr bool get_overflow()  const { return bits[6]; }

    MOS_NODISCARD
    constexpr bool get_negative()  const { return bits[7]; } 

    inline void set_carry(bool turn)     { bits[0] = turn; }
    inline void set_zero(bool turn)      { bits[1] = turn; }
    inline void set_interrupt(bool turn) { bits[2] = turn; }
    inline void set_decimal(bool turn)   { bits[3] = turn; }
    inline void set_break(bool turn)     { bits[4] = turn; }
    inline void set_unused(bool turn)    { bits[5] = turn; }
    inline void set_overflow(bool turn)  { bits[6] = turn; }
    inline void set_negative(bool turn)  { bits[7] = turn; } 

    MOS_NODISCARD
    constexpr uint8_t get_value() const { return static_cast<uint8_t>(bits.to_ulong()); }
    
    inline void operator=(uint8_t val)  { bits = val;  }
    inline void operator&=(uint8_t val) { bits &= val; }

private: 
    std::bitset<8> bits;
}; // Flags_impl

// --------------------------------------------------------------------------- //

struct CPU::Opcode_impl
{
private:
    // function pointers
    typedef uint16_t (CPU::*funcptr_addrmode) ();
    typedef uint8_t  (CPU::*funcptr_instr) (uint16_t);

public:
    funcptr_instr    instruction_ptr = nullptr; // returns how many clocks to add, expecting an address
    funcptr_addrmode address_ptr     = nullptr; // returns the address in the memory to work with
    uint8_t          clock;                     // clocks for the CPU

    const std::string instruction_name;
    const std::string address_name;

    Opcode_impl(funcptr_instr inst, funcptr_addrmode addr, uint8_t cycle,
                const std::string& inst_name, const std::string& addr_name) // for debugging
        : instruction_ptr(inst),
          address_ptr(addr),
          clock(cycle),
          instruction_name(inst_name),
          address_name(addr_name)
    {}
}; // Opcode_impl

// --------------------------------------------------------------------------- //

CPU::CPU(/*Bus* bus_ptr*/) 
    // initializing registers
    : P(std::make_unique<Flags_impl>()),
      current_opcode(nullptr),
      clocks(0)
    //   ,bus(bus_ptr)
{
    // filling the entire array with nullptr
    std::fill(opcode_table.begin(), opcode_table.end(), nullptr);

    opcode_table[0x69] = std::make_unique<CPU::Opcode_impl>(&CPU::ADC, &CPU::immediate       , 2, "ADC", "Immediate"  );
    opcode_table[0x65] = std::make_unique<CPU::Opcode_impl>(&CPU::ADC, &CPU::zero_page       , 3, "ADC", "Zero Page"  );
    opcode_table[0x75] = std::make_unique<CPU::Opcode_impl>(&CPU::ADC, &CPU::zero_page_x     , 4, "ADC", "Zero Page X");
    opcode_table[0x6D] = std::make_unique<CPU::Opcode_impl>(&CPU::ADC, &CPU::absolute        , 4, "ADC", "Absolute"   );
    opcode_table[0x7D] = std::make_unique<CPU::Opcode_impl>(&CPU::ADC, &CPU::absolute_x      , 4, "ADC", "Absolute X" );
    opcode_table[0x79] = std::make_unique<CPU::Opcode_impl>(&CPU::ADC, &CPU::absolute_y      , 4, "ADC", "Absolute Y" );
    opcode_table[0x61] = std::make_unique<CPU::Opcode_impl>(&CPU::ADC, &CPU::indexed_indirect, 6, "ADC", "Indirect X" );
    opcode_table[0x71] = std::make_unique<CPU::Opcode_impl>(&CPU::ADC, &CPU::indirect_indexed, 5, "ADC", "Indirect Y" );

    opcode_table[0x29] = std::make_unique<CPU::Opcode_impl>(&CPU::AND, &CPU::immediate       , 2, "AND", "Immediate"  );
    opcode_table[0x25] = std::make_unique<CPU::Opcode_impl>(&CPU::AND, &CPU::zero_page       , 3, "AND", "Zero Page"  );
    opcode_table[0x35] = std::make_unique<CPU::Opcode_impl>(&CPU::AND, &CPU::zero_page_x     , 4, "AND", "Zero Page X");
    opcode_table[0x2D] = std::make_unique<CPU::Opcode_impl>(&CPU::AND, &CPU::absolute        , 4, "AND", "Absolute"   );
    opcode_table[0x3D] = std::make_unique<CPU::Opcode_impl>(&CPU::AND, &CPU::absolute_x      , 4, "AND", "Absolute X" );
    opcode_table[0x39] = std::make_unique<CPU::Opcode_impl>(&CPU::AND, &CPU::absolute_y      , 4, "AND", "Absolute Y" );
    opcode_table[0x21] = std::make_unique<CPU::Opcode_impl>(&CPU::AND, &CPU::indexed_indirect, 6, "AND", "Indirect X" );
    opcode_table[0x31] = std::make_unique<CPU::Opcode_impl>(&CPU::AND, &CPU::indirect_indexed, 5, "AND", "Indirect Y" );

    opcode_table[0x0A] = std::make_unique<CPU::Opcode_impl>(&CPU::ASL, &CPU::accumulator, 5, "ASL", "Accumulator");
    opcode_table[0x06] = std::make_unique<CPU::Opcode_impl>(&CPU::ASL, &CPU::zero_page  , 5, "ASL", "Zero Page"  );
    opcode_table[0x16] = std::make_unique<CPU::Opcode_impl>(&CPU::ASL, &CPU::zero_page_x, 5, "ASL", "Zero Page X");
    opcode_table[0x0E] = std::make_unique<CPU::Opcode_impl>(&CPU::ASL, &CPU::absolute   , 6, "ASL", "Absolute"   );
    opcode_table[0x1E] = std::make_unique<CPU::Opcode_impl>(&CPU::ASL, &CPU::absolute_x , 7, "ASL", "Absolute X" );

    opcode_table[0x90] = std::make_unique<CPU::Opcode_impl>(&CPU::BCC, &CPU::relative, 2, "BCC", "Relative");
    opcode_table[0xB0] = std::make_unique<CPU::Opcode_impl>(&CPU::BCS, &CPU::relative, 2, "BCS", "Relative");
    opcode_table[0xF0] = std::make_unique<CPU::Opcode_impl>(&CPU::BEQ, &CPU::relative, 2, "BEQ", "Relative");

    opcode_table[0x24] = std::make_unique<CPU::Opcode_impl>(&CPU::BIT, &CPU::zero_page, 3, "BIT", "Zero Page");
    opcode_table[0x2C] = std::make_unique<CPU::Opcode_impl>(&CPU::BIT, &CPU::absolute , 4, "BIT", "Absolute" );

    opcode_table[0x30] = std::make_unique<CPU::Opcode_impl>(&CPU::BMI, &CPU::relative, 2, "BMI", "Relative");
    opcode_table[0xD0] = std::make_unique<CPU::Opcode_impl>(&CPU::BNE, &CPU::relative, 2, "BNE", "Relative");
    opcode_table[0x10] = std::make_unique<CPU::Opcode_impl>(&CPU::BPL, &CPU::relative, 2, "BPL", "Relative");

    opcode_table[0x00] = std::make_unique<CPU::Opcode_impl>(&CPU::BRK, &CPU::implicit, 2, "BRK", "Implicit");

    opcode_table[0x50] = std::make_unique<CPU::Opcode_impl>(&CPU::BVC, &CPU::relative, 2, "BVC", "Relative");
    opcode_table[0x70] = std::make_unique<CPU::Opcode_impl>(&CPU::BVS, &CPU::relative, 2, "BVS", "Relative");

    opcode_table[0x18] = std::make_unique<CPU::Opcode_impl>(&CPU::CLC, &CPU::implicit, 2, "CLC", "Implicit");
    opcode_table[0xD8] = std::make_unique<CPU::Opcode_impl>(&CPU::CLD, &CPU::implicit, 2, "CLD", "Implicit");
    opcode_table[0x58] = std::make_unique<CPU::Opcode_impl>(&CPU::CLI, &CPU::implicit, 2, "CLI", "Implicit");
    opcode_table[0xB8] = std::make_unique<CPU::Opcode_impl>(&CPU::CLV, &CPU::implicit, 2, "CLV", "Implicit");

    opcode_table[0xC9] = std::make_unique<CPU::Opcode_impl>(&CPU::CMP, &CPU::immediate       , 2, "CMP", "Immediate"  );
    opcode_table[0xC5] = std::make_unique<CPU::Opcode_impl>(&CPU::CMP, &CPU::zero_page       , 3, "CMP", "Zero Page"  );
    opcode_table[0xD5] = std::make_unique<CPU::Opcode_impl>(&CPU::CMP, &CPU::zero_page_x     , 4, "CMP", "Zero Page X");
    opcode_table[0xCD] = std::make_unique<CPU::Opcode_impl>(&CPU::CMP, &CPU::absolute        , 4, "CMP", "Absolute"   );
    opcode_table[0xDD] = std::make_unique<CPU::Opcode_impl>(&CPU::CMP, &CPU::absolute_x      , 4, "CMP", "Absolute X" );
    opcode_table[0xD9] = std::make_unique<CPU::Opcode_impl>(&CPU::CMP, &CPU::absolute_y      , 4, "CMP", "Absolute Y" );
    opcode_table[0xC1] = std::make_unique<CPU::Opcode_impl>(&CPU::CMP, &CPU::indexed_indirect, 6, "CMP", "Indirect X" );
    opcode_table[0xD1] = std::make_unique<CPU::Opcode_impl>(&CPU::CMP, &CPU::indirect_indexed, 5, "CMP", "Indirect Y" );

    opcode_table[0xE0] = std::make_unique<CPU::Opcode_impl>(&CPU::CPX, &CPU::immediate, 2, "CPX", "Immediate");
    opcode_table[0xE4] = std::make_unique<CPU::Opcode_impl>(&CPU::CPX, &CPU::zero_page, 3, "CPX", "Zero Page");
    opcode_table[0xEC] = std::make_unique<CPU::Opcode_impl>(&CPU::CPX, &CPU::absolute , 4, "CPX", "Absolute" );

    opcode_table[0xC0] = std::make_unique<CPU::Opcode_impl>(&CPU::CPY, &CPU::immediate, 2, "CPY", "Immediate");
    opcode_table[0xC4] = std::make_unique<CPU::Opcode_impl>(&CPU::CPY, &CPU::zero_page, 3, "CPY", "Zero Page");
    opcode_table[0xCC] = std::make_unique<CPU::Opcode_impl>(&CPU::CPY, &CPU::absolute , 4, "CPY", "Absolute" );

    opcode_table[0xC6] = std::make_unique<CPU::Opcode_impl>(&CPU::DEC, &CPU::zero_page  , 5, "DEC", "Zero Page"  );
    opcode_table[0xD6] = std::make_unique<CPU::Opcode_impl>(&CPU::DEC, &CPU::zero_page_x, 6, "DEC", "Zero Page X");
    opcode_table[0xCE] = std::make_unique<CPU::Opcode_impl>(&CPU::DEC, &CPU::absolute   , 6, "DEC", "Absolute"   );
    opcode_table[0xDE] = std::make_unique<CPU::Opcode_impl>(&CPU::DEC, &CPU::absolute_x , 7, "DEC", "Absolute X" );

    opcode_table[0xCA] = std::make_unique<CPU::Opcode_impl>(&CPU::DEX, &CPU::implicit, 2, "DEX", "Implicit");
    opcode_table[0x88] = std::make_unique<CPU::Opcode_impl>(&CPU::DEY, &CPU::implicit, 2, "DEY", "Implicit");

    opcode_table[0x49] = std::make_unique<CPU::Opcode_impl>(&CPU::EOR, &CPU::immediate       , 2, "EOR", "Immediate"  );
    opcode_table[0x45] = std::make_unique<CPU::Opcode_impl>(&CPU::EOR, &CPU::zero_page       , 3, "EOR", "Zero Page"  );
    opcode_table[0x55] = std::make_unique<CPU::Opcode_impl>(&CPU::EOR, &CPU::zero_page_x     , 4, "EOR", "Zero Page X");
    opcode_table[0x4D] = std::make_unique<CPU::Opcode_impl>(&CPU::EOR, &CPU::absolute        , 4, "EOR", "Absolute"   );
    opcode_table[0x5D] = std::make_unique<CPU::Opcode_impl>(&CPU::EOR, &CPU::absolute_x      , 4, "EOR", "Absolute X" );
    opcode_table[0x59] = std::make_unique<CPU::Opcode_impl>(&CPU::EOR, &CPU::absolute_y      , 4, "EOR", "Absolute Y" );
    opcode_table[0x41] = std::make_unique<CPU::Opcode_impl>(&CPU::EOR, &CPU::indexed_indirect, 6, "EOR", "Indirect X" );
    opcode_table[0x51] = std::make_unique<CPU::Opcode_impl>(&CPU::EOR, &CPU::indirect_indexed, 5, "EOR", "Indirect Y" );

    opcode_table[0xE6] = std::make_unique<CPU::Opcode_impl>(&CPU::INC, &CPU::zero_page  , 5, "INC", "Zero Page"  );
    opcode_table[0xF6] = std::make_unique<CPU::Opcode_impl>(&CPU::INC, &CPU::zero_page_x, 6, "INC", "Zero Page X");
    opcode_table[0xEE] = std::make_unique<CPU::Opcode_impl>(&CPU::INC, &CPU::absolute   , 6, "INC", "Absolute"   );
    opcode_table[0xFE] = std::make_unique<CPU::Opcode_impl>(&CPU::INC, &CPU::absolute_x , 7, "INC", "Absolute X" );

    opcode_table[0xE8] = std::make_unique<CPU::Opcode_impl>(&CPU::INX, &CPU::implicit, 2, "INX", "Implicit");
    opcode_table[0xC8] = std::make_unique<CPU::Opcode_impl>(&CPU::INY, &CPU::implicit, 2, "INY", "Implicit");

    opcode_table[0x4C] = std::make_unique<CPU::Opcode_impl>(&CPU::JMP, &CPU::absolute, 3, "JMP", "Absolute");
    opcode_table[0x6C] = std::make_unique<CPU::Opcode_impl>(&CPU::JMP, &CPU::indirect, 5, "JMP", "Indirect");

    opcode_table[0x20] = std::make_unique<CPU::Opcode_impl>(&CPU::JSR, &CPU::absolute, 6, "JSR", "Absolute");

    opcode_table[0xA9] = std::make_unique<CPU::Opcode_impl>(&CPU::LDA, &CPU::immediate       , 2, "LDA", "Immediate"  );
    opcode_table[0xA5] = std::make_unique<CPU::Opcode_impl>(&CPU::LDA, &CPU::zero_page       , 3, "LDA", "Zero Page"  );
    opcode_table[0xB5] = std::make_unique<CPU::Opcode_impl>(&CPU::LDA, &CPU::zero_page_x     , 4, "LDA", "Zero Page X");
    opcode_table[0xAD] = std::make_unique<CPU::Opcode_impl>(&CPU::LDA, &CPU::absolute        , 4, "LDA", "Absolute"   );
    opcode_table[0xBD] = std::make_unique<CPU::Opcode_impl>(&CPU::LDA, &CPU::absolute_x      , 4, "LDA", "Absolute X" );
    opcode_table[0xB9] = std::make_unique<CPU::Opcode_impl>(&CPU::LDA, &CPU::absolute_y      , 4, "LDA", "Absolute Y" );
    opcode_table[0xA1] = std::make_unique<CPU::Opcode_impl>(&CPU::LDA, &CPU::indexed_indirect, 6, "LDA", "Indirect X" );
    opcode_table[0xB1] = std::make_unique<CPU::Opcode_impl>(&CPU::LDA, &CPU::indirect_indexed, 5, "LDA", "Indirect Y" );

    opcode_table[0xA2] = std::make_unique<CPU::Opcode_impl>(&CPU::LDX, &CPU::immediate  , 2, "LDX", "Immediate"  );
    opcode_table[0xA6] = std::make_unique<CPU::Opcode_impl>(&CPU::LDX, &CPU::zero_page  , 3, "LDX", "Zero Page"  );
    opcode_table[0xB6] = std::make_unique<CPU::Opcode_impl>(&CPU::LDX, &CPU::zero_page_y, 4, "LDX", "Zero Page Y");
    opcode_table[0xAE] = std::make_unique<CPU::Opcode_impl>(&CPU::LDX, &CPU::absolute   , 4, "LDX", "Absolute"   );
    opcode_table[0xBE] = std::make_unique<CPU::Opcode_impl>(&CPU::LDX, &CPU::absolute_y , 4, "LDX", "Absolute Y" );

    opcode_table[0xA0] = std::make_unique<CPU::Opcode_impl>(&CPU::LDY, &CPU::immediate  , 2, "LDY", "Immediate"  );
    opcode_table[0xA4] = std::make_unique<CPU::Opcode_impl>(&CPU::LDY, &CPU::zero_page  , 3, "LDY", "Zero Page"  );
    opcode_table[0xB4] = std::make_unique<CPU::Opcode_impl>(&CPU::LDY, &CPU::zero_page_x, 4, "LDY", "Zero Page X");
    opcode_table[0xAC] = std::make_unique<CPU::Opcode_impl>(&CPU::LDY, &CPU::absolute   , 4, "LDY", "Absolute"   );
    opcode_table[0xBC] = std::make_unique<CPU::Opcode_impl>(&CPU::LDY, &CPU::absolute_x , 4, "LDY", "Absolute X" );

    opcode_table[0x4A] = std::make_unique<CPU::Opcode_impl>(&CPU::LSR, &CPU::accumulator, 2, "LSR", "Accumulator");
    opcode_table[0x46] = std::make_unique<CPU::Opcode_impl>(&CPU::LSR, &CPU::zero_page  , 5, "LSR", "Zero Page"  );
    opcode_table[0x56] = std::make_unique<CPU::Opcode_impl>(&CPU::LSR, &CPU::zero_page_x, 6, "LSR", "Zero Page X");
    opcode_table[0x4E] = std::make_unique<CPU::Opcode_impl>(&CPU::LSR, &CPU::absolute   , 6, "LSR", "Absolute"   );
    opcode_table[0x5E] = std::make_unique<CPU::Opcode_impl>(&CPU::LSR, &CPU::absolute_x , 7, "LSR", "Absolute X" );

    opcode_table[0xEA] = std::make_unique<CPU::Opcode_impl>(&CPU::NOP, &CPU::implicit, 2, "NOP", "Implicit");

    opcode_table[0x09] = std::make_unique<CPU::Opcode_impl>(&CPU::ORA, &CPU::immediate       , 2, "ORA", "Immediate"  );
    opcode_table[0x05] = std::make_unique<CPU::Opcode_impl>(&CPU::ORA, &CPU::zero_page       , 3, "ORA", "Zero Page"  );
    opcode_table[0x15] = std::make_unique<CPU::Opcode_impl>(&CPU::ORA, &CPU::zero_page_x     , 4, "ORA", "Zero Page X");
    opcode_table[0x0D] = std::make_unique<CPU::Opcode_impl>(&CPU::ORA, &CPU::absolute        , 4, "ORA", "Absolute"   );
    opcode_table[0x1D] = std::make_unique<CPU::Opcode_impl>(&CPU::ORA, &CPU::absolute_x      , 4, "ORA", "Absolute X" );
    opcode_table[0x19] = std::make_unique<CPU::Opcode_impl>(&CPU::ORA, &CPU::absolute_y      , 4, "ORA", "Absolute Y" );
    opcode_table[0x01] = std::make_unique<CPU::Opcode_impl>(&CPU::ORA, &CPU::indexed_indirect, 6, "ORA", "Indirect X" );
    opcode_table[0x11] = std::make_unique<CPU::Opcode_impl>(&CPU::ORA, &CPU::indirect_indexed, 5, "ORA", "Indirect Y" );

    opcode_table[0x48] = std::make_unique<CPU::Opcode_impl>(&CPU::PHA, &CPU::implicit, 3, "PHA", "Implicit");
    opcode_table[0x08] = std::make_unique<CPU::Opcode_impl>(&CPU::PHP, &CPU::implicit, 3, "PHP", "Implicit");
    opcode_table[0x68] = std::make_unique<CPU::Opcode_impl>(&CPU::PLA, &CPU::implicit, 4, "PLA", "Implicit");
    opcode_table[0x28] = std::make_unique<CPU::Opcode_impl>(&CPU::PLP, &CPU::implicit, 4, "PLP", "Implicit");

    opcode_table[0x2A] = std::make_unique<CPU::Opcode_impl>(&CPU::ROL, &CPU::accumulator, 2, "ROL", "Accumulator");
    opcode_table[0x26] = std::make_unique<CPU::Opcode_impl>(&CPU::ROL, &CPU::zero_page  , 5, "ROL", "Zero Page"  );
    opcode_table[0x36] = std::make_unique<CPU::Opcode_impl>(&CPU::ROL, &CPU::zero_page_x, 6, "ROL", "Zero Page X");
    opcode_table[0x2E] = std::make_unique<CPU::Opcode_impl>(&CPU::ROL, &CPU::absolute   , 6, "ROL", "Absolute"   );
    opcode_table[0x3E] = std::make_unique<CPU::Opcode_impl>(&CPU::ROL, &CPU::absolute_x , 7, "ROL", "Absolute, X");

    opcode_table[0x6A] = std::make_unique<CPU::Opcode_impl>(&CPU::ROR, &CPU::accumulator, 2, "ROR", "Accumulator");
    opcode_table[0x66] = std::make_unique<CPU::Opcode_impl>(&CPU::ROR, &CPU::zero_page  , 5, "ROR", "Zero Page"  );
    opcode_table[0x76] = std::make_unique<CPU::Opcode_impl>(&CPU::ROR, &CPU::zero_page_x, 6, "ROR", "Zero Page X");
    opcode_table[0x6E] = std::make_unique<CPU::Opcode_impl>(&CPU::ROR, &CPU::absolute   , 6, "ROR", "Absolute"   );
    opcode_table[0x7E] = std::make_unique<CPU::Opcode_impl>(&CPU::ROR, &CPU::absolute_x , 7, "ROR", "Absolute X" );

    opcode_table[0x40] = std::make_unique<CPU::Opcode_impl>(&CPU::RTI, &CPU::implicit, 6, "RTI", "Implicit");
    opcode_table[0x60] = std::make_unique<CPU::Opcode_impl>(&CPU::RTS, &CPU::implicit, 6, "RTS", "Implicit");

    opcode_table[0xE9] = std::make_unique<CPU::Opcode_impl>(&CPU::SBC, &CPU::immediate       , 2, "SBC", "Immediate"  );
    opcode_table[0xE5] = std::make_unique<CPU::Opcode_impl>(&CPU::SBC, &CPU::zero_page       , 3, "SBC", "Zero Page"  );
    opcode_table[0xF5] = std::make_unique<CPU::Opcode_impl>(&CPU::SBC, &CPU::zero_page_x     , 4, "SBC", "Zero Page X");
    opcode_table[0xED] = std::make_unique<CPU::Opcode_impl>(&CPU::SBC, &CPU::absolute        , 4, "SBC", "Absolute"   );
    opcode_table[0xFD] = std::make_unique<CPU::Opcode_impl>(&CPU::SBC, &CPU::absolute_x      , 4, "SBC", "Absolute X" );
    opcode_table[0xF9] = std::make_unique<CPU::Opcode_impl>(&CPU::SBC, &CPU::absolute_y      , 4, "SBC", "Absolute Y" );
    opcode_table[0xE1] = std::make_unique<CPU::Opcode_impl>(&CPU::SBC, &CPU::indexed_indirect, 6, "SBC", "Indirect X" );
    opcode_table[0xF1] = std::make_unique<CPU::Opcode_impl>(&CPU::SBC, &CPU::indirect_indexed, 5, "SBC", "Indirect Y" );

    opcode_table[0x38] = std::make_unique<CPU::Opcode_impl>(&CPU::SEC, &CPU::implicit, 2, "SEC", "Implicit");
    opcode_table[0xF8] = std::make_unique<CPU::Opcode_impl>(&CPU::SED, &CPU::implicit, 2, "SED", "Implicit");
    opcode_table[0x78] = std::make_unique<CPU::Opcode_impl>(&CPU::SEI, &CPU::implicit, 2, "SEI", "Implicit");

    opcode_table[0x85] = std::make_unique<CPU::Opcode_impl>(&CPU::STA, &CPU::zero_page       , 3, "STA", "Zero Page"  );
    opcode_table[0x95] = std::make_unique<CPU::Opcode_impl>(&CPU::STA, &CPU::zero_page_x     , 4, "STA", "Zero Page X");
    opcode_table[0x8D] = std::make_unique<CPU::Opcode_impl>(&CPU::STA, &CPU::absolute        , 4, "STA", "Absolute"   );
    opcode_table[0x9D] = std::make_unique<CPU::Opcode_impl>(&CPU::STA, &CPU::absolute_x      , 5, "STA", "Absolute X" );
    opcode_table[0x99] = std::make_unique<CPU::Opcode_impl>(&CPU::STA, &CPU::absolute_y      , 5, "STA", "Absolute Y" );
    opcode_table[0x81] = std::make_unique<CPU::Opcode_impl>(&CPU::STA, &CPU::indexed_indirect, 6, "STA", "Indirect X" );
    opcode_table[0x91] = std::make_unique<CPU::Opcode_impl>(&CPU::STA, &CPU::indirect_indexed, 6, "STA", "Indirect Y" );

    opcode_table[0x86] = std::make_unique<CPU::Opcode_impl>(&CPU::STX, &CPU::zero_page  , 3, "STX", "Zero Page"  );
    opcode_table[0x96] = std::make_unique<CPU::Opcode_impl>(&CPU::STX, &CPU::zero_page_y, 4, "STX", "Zero Page Y");
    opcode_table[0x8E] = std::make_unique<CPU::Opcode_impl>(&CPU::STX, &CPU::absolute   , 4, "STX", "Absolute"   );

    opcode_table[0x84] = std::make_unique<CPU::Opcode_impl>(&CPU::STY, &CPU::zero_page  , 3, "STY", "Zero Page"  );
    opcode_table[0x94] = std::make_unique<CPU::Opcode_impl>(&CPU::STY, &CPU::zero_page_x, 4, "STY", "Zero Page X");
    opcode_table[0x8C] = std::make_unique<CPU::Opcode_impl>(&CPU::STY, &CPU::absolute   , 4, "STY", "Absolute"   );

    opcode_table[0xAA] = std::make_unique<CPU::Opcode_impl>(&CPU::TAX, &CPU::implicit, 2, "TAX", "Implicit");
    opcode_table[0xA8] = std::make_unique<CPU::Opcode_impl>(&CPU::TAY, &CPU::implicit, 2, "TAY", "Implicit");
    opcode_table[0xBA] = std::make_unique<CPU::Opcode_impl>(&CPU::TSX, &CPU::implicit, 2, "TSX", "Implicit");
    opcode_table[0x8A] = std::make_unique<CPU::Opcode_impl>(&CPU::TXA, &CPU::implicit, 2, "TXA", "Implicit");
    opcode_table[0x9A] = std::make_unique<CPU::Opcode_impl>(&CPU::TXS, &CPU::implicit, 2, "TXS", "Implicit");
    opcode_table[0x98] = std::make_unique<CPU::Opcode_impl>(&CPU::TYA, &CPU::implicit, 2, "TYA", "Implicit");
}

CPU::~CPU() = default;

// --------------------------------------------------------------------------- //

void CPU::execute() noexcept 
{
    // fetch
    uint16_t opcode = bus->cpu_read(PC);

    ++PC;

    current_opcode = opcode_table[opcode].get();
    if(current_opcode != nullptr)
    {
        P->set_unused(1);

        // decode
        uint16_t addr = MOS_EXECUTE_CLASS_FUNCTION_POINTER(current_opcode->address_ptr);

        // execute
        uint8_t additional_clocks = MOS_EXECUTE_CLASS_FUNCTION_POINTER(current_opcode->instruction_ptr, addr)

        // imitating the clocks delay in the microprocessor
        uint8_t temp_clocks = additional_clocks + current_opcode->clock;
        clocks += temp_clocks;

        // count down clocks
        // while(temp_clocks --> 0);
    }
    else
    {
        std::cerr << "MOS WRONG OPCODE: "     
                  << opcode->instruction_name << " " 
                  << opcode->address_name     << std::endl;
    }
}

void CPU::IRQ() noexcept
{
    if(P->get_interrupt() == false)
    {
        push_stack((PC >> 8) & 0x00FF);
        push_stack(PC & 0x00FF);

        P->set_break(0);
        P->set_unused(1);
        P->set_interrupt(1);

        push_stack(P->get_value());

        uint8_t low  = bus->cpu_read(0xFFFE);
        uint8_t high = bus->cpu_read(0xFFFF);
        
        PC = (high << 8) | low;

        clocks = 7;
    }
}

void CPU::NMI() noexcept
{
    push_stack((PC >> 8) & 0x00FF);
    push_stack(PC & 0x00FF);

    P->set_break(0);
    P->set_unused(1);
    P->set_interrupt(1);

    push_stack(P->get_value());

    uint8_t low  = bus->cpu_read(0xFFFA);
    uint8_t high = bus->cpu_read(0xFFFB);
    
    PC = (high << 8) | low;

    clocks = 8;
}

void CPU::reset() noexcept
{
    uint8_t low  = bus->cpu_read(0xFFFC);
    uint8_t high = bus->cpu_read(0xFFFD);

    PC = (high << 8) | low;
    
    A = 0;
    X = 0;
    Y = 0;
    SP = 0xFD;
    *P = 0x34;

    clocks = 7;
}

// --------------------------------------------------------------------------- //
// --- Stack --- //
void CPU::push_stack(uint8_t val)
{
    // 0x0100 is the starting position of the stack
    bus->cpu_write(STACK_LOCATION + SP, val);
    --SP;
}

uint8_t CPU::pop_stack()
{
    // 0x0100 is the starting position of the stack
    ++SP;
    return bus->cpu_read(STACK_LOCATION + SP);
}

// --------------------------------------------------------------------------- //
// --- Addressing Modes --- //
MOS_NODISCARD
uint16_t CPU::implicit()
{ 
    return 0;
}

MOS_NODISCARD
uint16_t CPU::accumulator()
{
    return A;
}

MOS_NODISCARD
uint16_t CPU::immediate()
{
    return PC++;
}

MOS_NODISCARD
uint16_t CPU::zero_page()
{
    uint8_t addr = bus->cpu_read(PC);
    ++PC;

    return addr;
}

MOS_NODISCARD
uint16_t CPU::zero_page_x()
{
    return (zero_page() + X) % 256;
}

MOS_NODISCARD
uint16_t CPU::zero_page_y()
{
    return (zero_page() + Y) % 256;
}

MOS_NODISCARD
uint16_t CPU::relative()
{
    uint16_t offset = bus->cpu_read(PC);
    ++PC;

    if(offset & (1 << 7))
        offset |= 0xFF00;

    return PC + offset;
}

MOS_NODISCARD
uint16_t CPU::absolute()
{
    uint8_t low = bus->cpu_read(PC);
    ++PC;

    uint8_t high = bus->cpu_read(PC);
    ++PC;

    // combine both
    return (high << 8) | low;
}

MOS_NODISCARD
uint16_t CPU::absolute_x()
{
    return absolute() + X;
}

MOS_NODISCARD
uint16_t CPU::absolute_y()
{
    return absolute() + Y;
}

MOS_NODISCARD
uint16_t CPU::indirect()
{
    uint16_t temp;

    uint8_t ptr_low = bus->cpu_read(PC);
    ++PC;
    
    {
        uint8_t high = bus->cpu_read(PC);
        ++PC;

        temp = (high << 8) | ptr_low;
    }

    uint16_t low = bus->cpu_read(temp);
    uint16_t high;

    // bug in the hardware
    if(ptr_low == 0xFF)
        high = bus->cpu_read((temp & 0xFF00) + ((temp + 1) & 0x00FF));
    else
        high = bus->cpu_read(temp + 1);

    return low + 0x100 * high;
}

MOS_NODISCARD
uint16_t CPU::indexed_indirect()
{
    uint8_t temp = bus->cpu_read(PC);
    ++PC;

    uint8_t low  = bus->cpu_read((temp + X) % 256);
    uint8_t high = bus->cpu_read((temp + X + 1) % 256);

    return (high << 8) | low;
}

MOS_NODISCARD
uint16_t CPU::indirect_indexed()
{
    uint8_t temp = bus->cpu_read(PC);
    ++PC;

    uint8_t low  = bus->cpu_read(temp);
    uint8_t high = bus->cpu_read((temp + 1) % 256);

    return ((high << 8) | low) + Y;
}

// --------------------------------------------------------------------------- //
// --- Instructions --- //
// --- Load Operations ---
// Loads a byte of memory into the accumulator setting 
// the zero and negative flags as appropriate.
MOS_NODISCARD
uint8_t CPU::LDA(uint16_t addr) 
{
    A = bus->cpu_read(addr);

    P->set_zero(A == 0);
    P->set_negative(A & P->NEGATIVE);

    // Check if page crossed
    if(current_opcode->address_ptr == &CPU::absolute_x)
        return check_page_crossed_with_offset(addr, X);
    else if(current_opcode->address_ptr == &CPU::absolute_y)
        return check_page_crossed_with_offset(addr, Y);
    else if(current_opcode->address_ptr == &CPU::indirect_indexed)
        return check_page_crossed_with_offset(addr, Y);

    return 0;
}

// Loads a byte of memory into the X register setting 
// the zero and negative flags as appropriate.
MOS_NODISCARD
uint8_t CPU::LDX(uint16_t addr) 
{
    X = bus->cpu_read(addr);

    P->set_zero(X == 0);
    P->set_negative(X & P->NEGATIVE);

    // Check if page crossed
    if(current_opcode->address_ptr == &CPU::absolute_y)
        return check_page_crossed_with_offset(addr, Y);

    return 0;
}

// Loads a byte of memory into the Y register setting 
// the zero and negative flags as appropriate.
MOS_NODISCARD
uint8_t CPU::LDY(uint16_t addr) 
{
    Y = bus->cpu_read(addr);

    P->set_zero(Y == 0);
    P->set_negative(Y & P->NEGATIVE);

    // Check if page crossed
    if(current_opcode->address_ptr == &CPU::absolute_y)
        return check_page_crossed_with_offset(addr, Y);

    return 0;
}

// --- Store Operations ---
// Stores the contents of the accumulator into memory.
MOS_NODISCARD
uint8_t CPU::STA(uint16_t addr) 
{
    bus->cpu_write(addr, A);
    
    return 0;
}

// Stores the contents of the X register into memory.
MOS_NODISCARD
uint8_t CPU::STX(uint16_t addr) 
{
    bus->cpu_write(addr, X);

    return 0;
} 

// Stores the contents of the Y register into memory.
MOS_NODISCARD
uint8_t CPU::STY(uint16_t addr) 
{
    bus->cpu_write(addr, Y);

    return 0;
} 

// --- Register Transfers ---
// Copies the current contents of the accumulator into 
// the X register and sets the zero and negative flags 
// as appropriate.
MOS_NODISCARD
uint8_t CPU::TAX(MOS_UNUSED uint16_t addr) 
{
    X = A;

    P->set_zero(X == 0);
    P->set_negative(X & P->NEGATIVE);

    return 0;
}

// Copies the current contents of the accumulator into 
// the Y register and sets the zero and negative flags 
// as appropriate.
MOS_NODISCARD
uint8_t CPU::TAY(MOS_UNUSED uint16_t addr) 
{
    Y = A;

    P->set_zero(Y == 0);
    P->set_negative(Y & P->NEGATIVE);

    return 0;
} 

// Copies the current contents of the X register into 
// the accumulator and sets the zero and negative flags 
// as appropriate.
MOS_NODISCARD
uint8_t CPU::TXA(MOS_UNUSED uint16_t addr) 
{
    A = X;

    P->set_zero(A == 0);
    P->set_negative(A & P->NEGATIVE);

    return 0;
} 

// Copies the current contents of the Y register into 
// the accumulator and sets the zero and negative flags 
// as appropriate.
MOS_NODISCARD
uint8_t CPU::TYA(MOS_UNUSED uint16_t addr) 
{
    A = Y;

    P->set_zero(A == 0);
    P->set_negative(A & P->NEGATIVE);

    return 0;
} 

// --- Stack Operations ---
// Copies the current contents of the stack register into 
// the X register and sets the zero and negative flags as 
// appropriate.
MOS_NODISCARD
uint8_t CPU::TSX(MOS_UNUSED uint16_t addr) 
{
    X = SP;

    P->set_zero(X == 0);
    P->set_negative(X & P->NEGATIVE);

    return 0;
}

// Copies the current contents of the X register into the stack register.
MOS_NODISCARD
uint8_t CPU::TXS(MOS_UNUSED uint16_t addr) 
{
    SP = X;

    return 0;
} 

// Pushes a copy of the accumulator on to the stack.
MOS_NODISCARD
uint8_t CPU::PHA(MOS_UNUSED uint16_t addr) 
{
    push_stack(A);

    return 0;
}

// Pushes a copy of the status flags on to the stack.
MOS_NODISCARD
uint8_t CPU::PHP(MOS_UNUSED uint16_t addr) 
{
    push_stack(P->get_value());

    return 0;
} 

// Pulls an 8 bit value from the stack and into the accumulator. 
// The zero and negative flags are set as appropriate.
MOS_NODISCARD
uint8_t CPU::PLA(MOS_UNUSED uint16_t addr) 
{
    A = pop_stack();

    P->set_zero(A == 0);
    P->set_negative(A & P->NEGATIVE);

    return 0;
} 

// Pulls an 8 bit value from the stack and into the processor flags. 
// The flags will take on new states as determined by the value pulled.
MOS_NODISCARD
uint8_t CPU::PLP(MOS_UNUSED uint16_t addr) 
{
    *P = pop_stack();

    return 0;
} 

// --- Logical ---
// A logical AND is performed, bit by bit, on the accumulator 
// contents using the contents of a byte of memory.
MOS_NODISCARD
uint8_t CPU::AND(uint16_t addr) 
{
    uint8_t src = bus->cpu_read(addr);
    A = A & src;

    P->set_zero(A == 0);
    P->set_negative(A & P->NEGATIVE);

    // check crossed page
    if(current_opcode->address_ptr == &CPU::absolute_x)
        return check_page_crossed_with_offset(addr, X);
    else if(current_opcode->address_ptr == &CPU::absolute_y)
        return check_page_crossed_with_offset(addr, Y);
    else if(current_opcode->address_ptr == &CPU::indirect_indexed)
        return check_page_crossed_with_offset(addr, Y);

    return 0;
}

// An exclusive OR is performed, bit by bit, on the accumulator 
// contents using the contents of a byte of memory.
MOS_NODISCARD
uint8_t CPU::EOR(uint16_t addr) 
{
    uint8_t src = bus->cpu_read(addr);
    A = A ^ src;

    P->set_zero(A == 0);
    P->set_negative(A & P->NEGATIVE);

    // check crossed page
    if(current_opcode->address_ptr == &CPU::absolute_x)
        return check_page_crossed_with_offset(addr, X);
    else if(current_opcode->address_ptr == &CPU::absolute_y)
        return check_page_crossed_with_offset(addr, Y);
    else if(current_opcode->address_ptr == &CPU::indirect_indexed)
        return check_page_crossed_with_offset(addr, Y);

    return 0;
} 

// An inclusive OR is performed, bit by bit, on the accumulator 
// contents using the contents of a byte of memory.
MOS_NODISCARD
uint8_t CPU::ORA(uint16_t addr) 
{
    uint8_t src = bus->cpu_read(addr);
    A = A | src;

    P->set_zero(A == 0);
    P->set_negative(A & P->NEGATIVE);

    // check crossed page
    if(current_opcode->address_ptr == &CPU::absolute_x)
        return check_page_crossed_with_offset(addr, X);
    else if(current_opcode->address_ptr == &CPU::absolute_y)
        return check_page_crossed_with_offset(addr, Y);
    else if(current_opcode->address_ptr == &CPU::indirect_indexed)
        return check_page_crossed_with_offset(addr, Y);

    return 0;
} 

// This instructions is used to test if one or more bits are set in a 
// target memory location. The mask pattern in A is ANDed with the value 
// in memory to set or clear the zero flag, but the result is not kept. 
// Bits 7 and 6 of the value from memory are copied into the N and V flags.
MOS_NODISCARD
uint8_t CPU::BIT(uint16_t addr) 
{
    uint8_t src = bus->cpu_read(addr);

    P->set_zero((A & src) == 0);
    P->set_overflow(src & P->OVERFLOW);
    P->set_negative(src & P->NEGATIVE);

    return 0;
} 

// --- Arithmetic ---
// This instruction adds the contents of a memory location to the 
// accumulator together with the carry bit. If overflow occurs the 
// carry bit is set, this enables multiple byte addition to be performed.
MOS_NODISCARD
uint8_t CPU::ADC(uint16_t addr) 
{
    uint16_t src  = bus->cpu_read(addr);
    uint16_t temp = A + src + P->get_carry();

    P->set_carry(temp > 255);
    P->set_zero((temp & 0x00FF) == 0);
    P->set_overflow((~(A ^ src) & (A ^ temp)) & 0x0080);
    P->set_negative(temp & P->NEGATIVE);

    A = temp;

    // check crossed page
    if(current_opcode->address_ptr == &CPU::absolute_x)
        return check_page_crossed_with_offset(addr, X);
    else if(current_opcode->address_ptr == &CPU::absolute_y)
        return check_page_crossed_with_offset(addr, Y);
    else if(current_opcode->address_ptr == &CPU::indirect_indexed)
        return check_page_crossed_with_offset(addr, Y);

    return 0;
}

// This instruction subtracts the contents of a memory location to the 
// accumulator together with the not of the carry bit. If overflow 
// occurs the carry bit is clear, this enables multiple byte subtraction 
// to be performed.
// This is literally the inverse of ADC, another common way to to do this
// is by doing something like this:
// return ADC(~addr->content);
MOS_NODISCARD
uint8_t CPU::SBC(uint16_t addr) 
{
    uint16_t src   = bus->cpu_read(addr);
    uint16_t value = src ^ 0x00FF;
    uint16_t temp  = A + value + P->get_carry();

    P->set_carry(temp & 0xFF00);
    P->set_zero((temp & 0x00FF) == 0);
    P->set_overflow((temp ^ A) & (temp ^ value) & 0x0080);
    P->set_negative(temp & P->NEGATIVE);

    A = temp;

    // check crossed page
    if(current_opcode->address_ptr == &CPU::absolute_x)
        return check_page_crossed_with_offset(addr, X);
    else if(current_opcode->address_ptr == &CPU::absolute_y)
        return check_page_crossed_with_offset(addr, Y);
    else if(current_opcode->address_ptr == &CPU::indirect_indexed)
        return check_page_crossed_with_offset(addr, Y);

    return 0;
} 

// This instruction compares the contents of the accumulator with another 
// memory held value and sets the zero and carry flags as appropriate.
MOS_NODISCARD
uint8_t CPU::CMP(uint16_t addr) 
{
    uint16_t src  = bus->cpu_read(addr);
    uint16_t temp = A - src;

    P->set_carry(A >= temp);
    P->set_zero((temp & 0x00FF) == 0);
    P->set_negative(temp & P->NEGATIVE);

    // check crossed page
    if(current_opcode->address_ptr == &CPU::absolute_x)
        return check_page_crossed_with_offset(addr, X);
    else if(current_opcode->address_ptr == &CPU::absolute_y)
        return check_page_crossed_with_offset(addr, Y);
    else if(current_opcode->address_ptr == &CPU::indirect_indexed)
        return check_page_crossed_with_offset(addr, Y);

    return 0;
} 

// This instruction compares the contents of the X register with another 
// memory held value and sets the zero and carry flags as appropriate.
MOS_NODISCARD
uint8_t CPU::CPX(uint16_t addr) 
{
    uint16_t src  = bus->cpu_read(addr);
    uint16_t temp = X - src;

    P->set_carry(X >= temp);
    P->set_zero((temp & 0x00FF) == 0);
    P->set_negative(temp & P->NEGATIVE);

    return 0;
} 

// This instruction compares the contents of the Y register with another 
// memory held value and sets the zero and carry flags as appropriate.
MOS_NODISCARD
uint8_t CPU::CPY(uint16_t addr) 
{
    uint16_t src  = bus->cpu_read(addr);
    uint16_t temp = Y - src;

    P->set_carry(Y >= temp);
    P->set_zero((temp & 0x00FF) == 0);
    P->set_negative(temp & P->NEGATIVE);

    return 0;
}

// --- Increments and Decrements ---
// Adds one to the value held at a specified memory location 
// setting the zero and negative flags as appropriate.
MOS_NODISCARD
uint8_t CPU::INC(uint16_t addr) 
{
    uint8_t src = bus->cpu_read(addr);
    ++src;

    P->set_zero(src == 0);
    P->set_negative(src & P->NEGATIVE);

    bus->cpu_write(addr, src);

    return 0;
}

// Adds one to the X register setting the zero and negative 
// flags as appropriate.
MOS_NODISCARD
uint8_t CPU::INX(MOS_UNUSED uint16_t addr) 
{
    ++X;

    P->set_zero(X == 0);
    P->set_negative(X & P->NEGATIVE);

    return 0;
}

// Adds one to the Y register setting the zero and negative 
// flags as appropriate.
MOS_NODISCARD
uint8_t CPU::INY(MOS_UNUSED uint16_t addr) 
{
    ++Y;

    P->set_zero(Y == 0);
    P->set_negative(Y & P->NEGATIVE);

    return 0;
}

// Subtracts one from the value held at a specified memory 
// location setting the zero and negative flags as 
// appropriate.
MOS_NODISCARD
uint8_t CPU::DEC(uint16_t addr) 
{   
    uint8_t src = bus->cpu_read(addr);
    --src;

    P->set_zero(src == 0);
    P->set_negative(src & P->NEGATIVE);

    bus->cpu_write(addr, src);

    return 0;
}

// Subtracts one from the X register setting the zero and 
// negative flags as appropriate.
MOS_NODISCARD
uint8_t CPU::DEX(MOS_UNUSED uint16_t addr) 
{
    --X;

    P->set_zero(X == 0);
    P->set_negative(X & P->NEGATIVE);

    return 0;
} 

// Subtracts one from the Y register setting the zero and 
// negative flags as appropriate.
MOS_NODISCARD
uint8_t CPU::DEY(MOS_UNUSED uint16_t addr)
{
    --Y;

    P->set_zero(Y == 0);
    P->set_negative(Y & P->NEGATIVE);

    return 0;
} 

// --- Shifts ---
// This operation shifts all the bits of the accumulator or 
// memory contents one bit left. Bit 0 is set to 0 and bit 7 
// is placed in the carry flag. The effect of this operation 
// is to multiply the memory contents by 2 
// (ignoring 2's complement considerations), setting the carry
// if the result will not fit in 8 bits.
MOS_NODISCARD
uint8_t CPU::ASL(uint16_t addr) 
{
    uint16_t src;

    if(current_opcode->address_ptr == &CPU::accumulator)
        src = addr;
    else
        src = bus->cpu_read(addr);

    src <<= 1;

    P->set_carry((src & 0xFF00) > 0);
    P->set_zero((src & 0x00FF) == 0);
    P->set_negative(src & P->NEGATIVE);

    if(current_opcode->address_ptr == &CPU::accumulator)
        A = src & 0x00FF;
    else
        bus->cpu_write(addr, src & 0x00FF);

    return 0;
}

// Each of the bits in A or M is shift one place to the right. 
// The bit that was in bit 0 is shifted into the carry flag. 
// Bit 7 is set to zero.
MOS_NODISCARD
uint8_t CPU::LSR(uint16_t addr) 
{
    uint16_t src;

    if(current_opcode->address_ptr == &CPU::accumulator)
        src = addr;
    else
        src = bus->cpu_read(addr);

    P->set_carry(src & P->CARRY);

    src >>= 1;

    P->set_zero((src & 0x00FF) == 0);
    P->set_negative(src & P->NEGATIVE);

    if(current_opcode->address_ptr == &CPU::accumulator)
        A = src & 0x00FF;
    else
        bus->cpu_write(addr, src & 0x00FF);

    return 0;
}

// Move each of the bits in either A or M one place to the left. 
// Bit 0 is filled with the current value of the carry flag 
// whilst the old bit 7 becomes the new carry flag value.
MOS_NODISCARD
uint8_t CPU::ROL(uint16_t addr) 
{
    uint16_t src;

    if(current_opcode->address_ptr == &CPU::accumulator)
        src = addr;
    else
        src = bus->cpu_read(addr);

    src = (src << 1) | P->get_carry();

    P->set_carry(src & 0xFF00);
    P->set_zero((src & 0x00FF) == 0);
    P->set_negative(src & P->NEGATIVE);

    if(current_opcode->address_ptr == &CPU::accumulator)
        A = src & 0x00FF;
    else
        bus->cpu_write(addr, src & 0x00FF);

    return 0;
}

// Move each of the bits in either A or M one place to the right. 
// Bit 7 is filled with the current value of the carry flag whilst 
// the old bit 0 becomes the new carry flag value.
MOS_NODISCARD
uint8_t CPU::ROR(uint16_t addr) 
{
    uint16_t src;

    if(current_opcode->address_ptr == &CPU::accumulator)
        src = addr;
    else
        src = bus->cpu_read(addr);

    if(P->get_carry() == true)
        src |= (1 << 8);

    P->set_carry(src & P->CARRY);
    src >>= 1;

    P->set_zero((src & 0x00FF) == 0);
    P->set_negative(src & P->NEGATIVE);

    if(current_opcode->address_ptr == &CPU::accumulator)
        A = src & 0x00FF;
    else
        bus->cpu_write(addr, src & 0x00FF);

    return 0;
}

// --- Jumps and Calls ---
// Sets the program counter to the address specified by the operand.
MOS_NODISCARD
uint8_t CPU::JMP(uint16_t addr) 
{
    PC = addr;

    return 0;
}

// The JSR instruction pushes the address (minus one) of the return 
// point on to the stack and then sets the program counter to the 
// target memory address.
MOS_NODISCARD
uint8_t CPU::JSR(MOS_UNUSED uint16_t addr) 
{
    --PC;

    push_stack((PC >> 8) & 0xFF);
    push_stack(PC & 0xFF);

    PC = addr;

    return 0;
}

// The RTS instruction is used at the end of a subroutine to return 
// to the calling routine. It pulls the program counter (minus one) 
// from the stack.
MOS_NODISCARD
uint8_t CPU::RTS(MOS_UNUSED uint16_t addr) 
{
    PC = pop_stack();
    PC |= pop_stack() << 8;

    ++PC;

    return 0;
}

// --- Branches ---
// If the carry flag is clear then add the relative displacement to 
// the program counter to cause a branch to a new location.
MOS_NODISCARD
uint8_t CPU::BCC(uint16_t addr) 
{
    uint8_t cycles = 0;

    if(P->get_carry() == false)
    {
        ++cycles += check_branch_succeeds(addr);

        PC = addr;
    }

    return cycles;
}

// If the carry flag is set then add the relative displacement to 
// the program counter to cause a branch to a new location.
MOS_NODISCARD
uint8_t CPU::BCS(uint16_t addr) 
{
    uint8_t cycles = 0;

    if(P->get_carry() == true)
    {
        ++cycles += check_branch_succeeds(addr);

        PC = addr;
    }

    return cycles;
}

// If the zero flag is set then add the relative displacement to 
// the program counter to cause a branch to a new location.
MOS_NODISCARD
uint8_t CPU::BEQ(uint16_t addr) 
{
    uint8_t cycles = 0;

    if(P->get_zero() == true)
    {
        ++cycles += check_branch_succeeds(addr);

        PC = addr;
    }

    return cycles;
} 

// If the negative flag is set then add the relative displacement 
// to the program counter to cause a branch to a new location.
MOS_NODISCARD
uint8_t CPU::BMI(uint16_t addr) 
{
    uint8_t cycles = 0;

    if(P->get_negative() == true)
    {
        ++cycles += check_branch_succeeds(addr);

        PC = addr;
    }

    return cycles;
}

// If the zero flag is clear then add the relative displacement to 
// the program counter to cause a branch to a new location.
MOS_NODISCARD
uint8_t CPU::BNE(uint16_t addr) 
{
    uint8_t cycles = 0;

    if(P->get_zero() == false)
    {
        ++cycles += check_branch_succeeds(addr);

        PC = addr;
    }

    return cycles;
}

// If the negative flag is clear then add the relative displacement 
// to the program counter to cause a branch to a new location.
MOS_NODISCARD
uint8_t CPU::BPL(uint16_t addr) 
{
    uint8_t cycles = 0;

    if(P->get_negative() == false)
    {
        ++cycles += check_branch_succeeds(addr);

        PC = addr;
    }

    return cycles;
} 

// If the overflow flag is clear then add the relative displacement 
// to the program counter to cause a branch to a new location.
MOS_NODISCARD
uint8_t CPU::BVC(uint16_t addr) 
{
    uint8_t cycles = 0;

    if(P->get_overflow() == false)
    {
        ++cycles += check_branch_succeeds(addr);

        PC = addr;
    }

    return cycles;
} 

// If the overflow flag is set then add the relative displacement 
// to the program counter to cause a branch to a new location.
MOS_NODISCARD
uint8_t CPU::BVS(uint16_t addr) 
{
    uint8_t cycles = 0;

    if(P->get_overflow() == true)
    {
        ++cycles += check_branch_succeeds(addr);

        PC = addr;
    }

    return cycles;
}

// --- Status Flag Changes ---
// Set the carry flag to zero.
MOS_NODISCARD
uint8_t CPU::CLC(MOS_UNUSED uint16_t addr) 
{
    P->set_carry(0);

    return 0;
} 

// Sets the decimal mode flag to zero.
MOS_NODISCARD
uint8_t CPU::CLD(MOS_UNUSED uint16_t addr) 
{
    P->set_decimal(0);
    
    return 0;
} 

// Clears the interrupt disable flag allowing normal interrupt 
// requests to be serviced.
MOS_NODISCARD
uint8_t CPU::CLI(MOS_UNUSED uint16_t addr) 
{
    P->set_interrupt(0);
    
    return 0;
}

// Clears the overflow flag.
MOS_NODISCARD
uint8_t CPU::CLV(MOS_UNUSED uint16_t addr) 
{
    P->set_overflow(0);
    
    return 0;
} 

// Set the carry flag to one.
MOS_NODISCARD
uint8_t CPU::SEC(MOS_UNUSED uint16_t addr) 
{
    P->set_carry(1);

    return 0;
} 

// Set the decimal mode flag to one.
MOS_NODISCARD
uint8_t CPU::SED(MOS_UNUSED uint16_t addr) 
{
    P->set_decimal(1);
    
    return 0;
}

// Set the interrupt disable flag to one.
MOS_NODISCARD
uint8_t CPU::SEI(MOS_UNUSED uint16_t addr) 
{
    P->set_interrupt(1);
    
    return 0;
} 

// --- System Functions ---
// The BRK instruction forces the generation of an interrupt request. 
// The program counter and processor status are pushed on the stack 
// then the IRQ interrupt vector at $FFFE/F is loaded into the PC 
// and the break flag in the status set to one.
MOS_NODISCARD
uint8_t CPU::BRK(MOS_UNUSED uint16_t addr) 
{
    ++PC;

    P->set_interrupt(1);

    push_stack((PC >> 8) & 0x00FF);
    push_stack(PC & 0x00FF);

    P->set_break(1);
    push_stack(P->get_value());
    P->set_break(0);

    PC = bus->cpu_read(0xFFFE) | (bus->cpu_read(0xFFFF) << 8);

    return 0;
}

// The NOP instruction causes no changes to the processor other than 
// the normal incrementing of the program counter to the next 
// instruction.
MOS_NODISCARD
uint8_t CPU::NOP(MOS_UNUSED uint16_t addr) 
{
    return 0;
}

// The RTI instruction is used at the end of an interrupt processing 
// routine. It pulls the processor flags from the stack followed by 
// the program counter.
MOS_NODISCARD
uint8_t CPU::RTI(MOS_UNUSED uint16_t addr) 
{
    *P = pop_stack();
    *P &= ~P->BREAK;
    *P &= ~P->UNUSED;

    PC = pop_stack();
    PC |= pop_stack() << 8;

    return 0;
}
