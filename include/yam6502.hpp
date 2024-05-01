/*
** yam6502.hpp
** Main implementation for yam::M6502 - a 6502 emulator
**
** Copyright(c) 2021 Marisa Heit
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*/

#include <concepts>
#include <stdint.h>
#include <cassert>
#include <string>
#include <functional>

namespace yam {
	// Addressing modes
	enum class am : uint8_t {
		imp,				// implicit
		acc,				// accumulator

		// Internal execution on memory data
		imm,				// #$00
		zp,					// $00
		zpx,				// $00,X
		zpy,				// $00,Y
		izx,				// ($00,X)
		izy,				// ($00),Y
		abs,				// $0000
		abx,				// $0000,X
		aby,				// $0000,Y

		// Store operations (STA/STX/STY)
		zp_sto,				// $00
		abs_sto,			// $0000
		izx_sto,			// ($00,X)
		abx_sto,			// $0000,X
		aby_sto,			// $0000,Y
		zpx_sto,			// $00,X
		zpy_sto,			// $00,Y
		izy_sto,			// ($00),Y

		// Read-Modify-Write operations
		zp_rmw,				// $00
		abs_rmw,			// $0000
		zpx_rmw,			// $00,X
		abx_rmw,			// $0000,X
		aby_rmw,			// $0000,Y only used by illegal opcodes
		izx_rmw,			// ($00,X) only used by illegal opcodes
		izy_rmw,			// ($00),Y only used by illegal opcodes

		// Miscellaneous Operations
		push,				// Push to stack
		pull,				// Pull from stack
		jmp,				// JMP $0000
		jmp_ind,			// JMP ($0000)
		call,				// JSR $0000
		rts,				// Return from Subroutine
		rel,				// PC-relative (branch)
		brk,				// BRK/IRQ/NMI/RESET sequence,
		rti,				// Return from Interrupt
		halt,				// Halt the processor by never transitioning back to T0

		COUNT
	};

	// Operations
	enum class op : uint8_t {

		ADC,
		AND,
		ASL,
		BCC,
		BCS,
		BEQ,
		BIT,
		BMI,
		BNE,
		BPL,
		BRK,
		BVC,
		BVS,
		CLC,
		CLD,
		CLI,
		CLV,
		CMP,
		CPX,
		CPY,
		DEC,
		DEX,
		DEY,
		EOR,
		INC,
		INX,
		INY,
		JMP,
		JSR,
		LDA,
		LDX,
		LDY,
		LSR,
		NOP,
		ORA,
		PHA,
		PHP,
		PLA,
		PLP,
		ROL,
		ROR,
		RTI,
		RTS,
		SBC,
		SEC,
		SED,
		SEI,
		STA,
		STX,
		STY,
		TAX,
		TAY,
		TSX,
		TXA,
		TXS,
		TYA,
		// Illegal operations
		AHX,
		ALR,
		ANC,
		ARR,
		AXS,
		DCP,
		ISC,
		KIL,	// aka JAM/HLT/CIM/CRP
		LAS,
		LAX,
		LAX_IMM,
		RLA,
		RRA,
		SAX,
		SHX,
		SHY,
		SLO,
		SRE,
		TAS,
		XAA,

		COUNT,

		// "Fake" operations
		IRQ,
		NMI,
		RESET,
	};

	// Opcode information for execution
	struct BaseOpcode {
		op Op;
		am Mode;
	};

	// Opcode information for disassembly
	struct DisasmInfo {
		uint8_t OperandBytes;	// # of bytes of operand data
		const char Format[11];	// printf specifier for operand
	};

	extern const char OpNames[static_cast<int>(op::COUNT)][4];	// Operation names for disassembly
	extern const DisasmInfo ModeTable[static_cast<int>(am::COUNT)];
	extern const BaseOpcode Opc6502[256];

	namespace flag {
		static constexpr inline uint8_t C = 0x01;
		static constexpr inline uint8_t Z = 0x02;
		static constexpr inline uint8_t I = 0x04;
		static constexpr inline uint8_t D = 0x08;
		static constexpr inline uint8_t B = 0x10;
		static constexpr inline uint8_t RESERVED = 0x20;
		static constexpr inline uint8_t V = 0x40;
		static constexpr inline uint8_t N = 0x80;
	}

	// Concepts to check for optional features of the bus
	//
	//  Check for processor interrupt pins. Any that the bus does not
	//  provide a function to read are treated as being held high/true:
	//   bus->GetRDY()  - return status of RDY line
	//   bus->GetIRQB() - return status of IRQB line
	//   bus->GetNMIB() - return status of NMIB line
	//   bus->GetRESB() - return status of RESB line
	//
	//  Opportunities to inject custom behavior:
	//   bus->SyncHandler() - Called just before an instruction is fetched
	//   bus->BreakHandler() - Called when the BRK instruction is executed
	//   bus->ReadNoSideEffects() - Used by the disassembler to read memory. Useful, e.g.,
	//                              for reading I/O space without triggering I/O actions.
	template<typename T>
	concept HasGetRDY = requires (T bus) {
		{ bus->GetRDY() } -> std::convertible_to<bool>;
	};
	template<typename T>
	concept HasGetIRQB = requires (T bus) {
		{ bus->GetIRQB() } -> std::convertible_to<bool>;
	};
	template<typename T>
	concept HasGetNMIB = requires (T bus) {
		{ bus->GetNMIB() } -> std::convertible_to<bool>;
	};
	template<typename T>
	concept HasGetRESB = requires (T bus) {
		{ bus->GetRESB() } -> std::convertible_to<bool>;
	};
	template<typename T, typename U>
	concept HasSyncHandler = requires (T bus, U & cpu, uint16_t addr) {
		{ bus->SyncHandler(cpu, addr) };
	};
	template<typename T, typename U>
	concept HasBreakHandler = requires (T bus, U &cpu, uint16_t addr) {
		{ bus->BreakHandler(cpu, addr) } -> std::convertible_to<bool>;
	};
	template<typename T>
	concept HasReadNoSideEffects = requires (T bus, uint16_t addr) {
		{ bus->ReadNoSideEffects(addr) } -> std::convertible_to<uint8_t>;
	};

	// T = A pointer type to the bus used to read and write.
	// trapcode = A trap opcode, if nonnegative.
	template<typename T, int trap_code=-1>
	class M6502 {
	public:
		using type = M6502<T, trap_code>;

		explicit M6502(T bus) : Bus(bus) {}
		M6502() = delete;
		M6502(const M6502 &) = default;
		M6502(M6502 &&) = default;
		M6502 &operator=(const M6502 &) = default;
		M6502 &operator=(M6502 &&) = default;

		// Set to true to emulate these related BRK/NMI bugs:
		//
		// 1. When an NMI is signalled during the beginning of a BRK sequence, the
		//    NMI vector will be used instead of the IRQ/BRK vector.
		// 2. An NMI that goes low during the vector read and goes high before the
		//    final BRK cycle will be "lost".
		//
		// As these are unlikely to be useful behaviors except for 100% accurate
		// emulation, this can be turned off.
		//
		// This class will not change this flag.
		bool EmulateNMIBRKBug = true;

		// Getters
		[[nodiscard]] uint8_t getA() const { return A; }
		[[nodiscard]] uint8_t getX() const { return X; }
		[[nodiscard]] uint8_t getY() const { return Y; }
		[[nodiscard]] uint16_t getSP() const { return 0x100 | SP; }
		[[nodiscard]] uint16_t getPC() const { return PC; }
		[[nodiscard]] uint8_t getP() const { return static_cast<uint8_t>(P); }

		// Setters
		void setA(uint8_t val) { A = val; }
		void setX(uint8_t val) { X = val; }
		void setY(uint8_t val) { Y = val; }
		void setSP(uint16_t val) { SP = val & 0xFF; }
		void setPC(uint16_t val) { PC = val; State = &type::execFetch_T1; }
		void setP(uint8_t val) { P = val; }

		// Begin the processor's reset sequence.
		void Reset()
		{
			BaseOp = op::RESET;
			State = &M6502::execBreak_T2;
			NextIrqPending = IrqPending = false;
			NmiPending = false;
			NmiMemory = CheckNMIB() ? ~0 : 0;
			InterruptGen = false;
		}

		// Execute one cycle.
		void Tick()
		{
			assert(State != nullptr);
			CheckInterruptPins();
			State = std::invoke(State, this);
		}

		// Disassemble the opcode at addr.
		std::string DisasmOp(uint16_t addr, bool full_line)
		{
			return DisasmStep(addr, full_line);
		}

		// Disassemble the opcode at addr, and advance addr to the next opcode.
		std::string DisasmStep(uint16_t &addr, bool full_line)
		{
			// I wanted to play with std::format, but it doesn't exist
			// in the standard library yet, so back to the C library I go.
			char buffer[48];
			uint8_t opbyte[4] = { ReadNoSideEffectsIfPossible(addr) };
			const auto &base = Opc6502[opbyte[0]];
			const auto &info = ModeTable[static_cast<int>(base.Mode)];
			unsigned operand = 0;

			const auto buffsize = std::size(buffer);
			int buffpos = 0;

			// Get operand bytes
			if (info.OperandBytes >= 1) {
				operand = opbyte[1] = ReadNoSideEffectsIfPossible(addr + 1);
				if (info.OperandBytes > 1) {
					opbyte[2] = ReadNoSideEffectsIfPossible(addr + 2);
					operand |= opbyte[2] << 8;
				}
			}

			if (full_line) {
				// Address of this instruction
				buffpos += snprintf(buffer, buffsize, "%04X-   ", addr);
				// Opcode bytes or blank space
				for (int i = 0; i < 4; ++i) {
					buffpos += snprintf(buffer + buffpos, buffsize - buffpos,
						(i <= info.OperandBytes) ? "%02X " : "   ",
						opbyte[i]);
				}
			}
			// Instruction neumonic
			buffpos += snprintf(buffer + buffpos, buffsize - buffpos, "%s", OpNames[static_cast<int>(base.Op)]);

			// Operand
			if (base.Mode == am::rel) {
				operand = static_cast<uint16_t>(addr + 2 + static_cast<int8_t>(opbyte[1]));
			}
			if (info.Format[0]) {
				// Three spaces for a full-line disassembly; one space for just the instruction
				buffpos += snprintf(buffer + buffpos, buffsize - buffpos, "%*c", full_line ? 3 : 1, ' ');
				buffpos += snprintf(buffer + buffpos, buffsize - buffpos, info.Format, operand);
			}
			if (base.Mode == am::rel) {
				buffpos += snprintf(buffer + buffpos, buffsize - buffpos, "   [%+d]", static_cast<int8_t>(opbyte[1]));
			}
			if constexpr (trap_code >= 0) {
				if (opbyte[0] == trap_code) {
					buffpos += snprintf(buffer + buffpos, buffsize - buffpos, "%*c[TRAP]",
						full_line ? 38 - buffpos : 1, ' ');
				}
			}
			addr += 1 + info.OperandBytes;
			return std::string(buffer, buffpos);
		}

		// Returns the address of the vector associated with an "operation"
		// RESET and NMI are treated as operations only to distinguish them
		// from BRK.
		[[nodiscard]] constexpr uint16_t opToVector(op operation) const
		{
			switch (operation) {
			case op::RESET: return 0xFFFC;
			case op::NMI: return 0xFFFA;
			default: return 0xFFFE;
			}
		}

		// Address of the stack page
		static constexpr inline uint16_t STACK_PAGE = 0x100;

	private:
		class StatusFlags {
		public:
			// Set individual flags.
			constexpr void setN(uint8_t val) { N = val; }
			constexpr void setV(uint8_t val) { V = val; }
			constexpr void setD(bool flag) { if (flag) { DI |= flag::D; } else { DI &= ~flag::D; } }
			constexpr void setI(bool flag) { if (flag) { DI |= flag::I; } else { DI &= ~flag::I; } }
			constexpr void setZ(bool flag) { Z = flag; }
			constexpr void setC(bool flag) { C = flag; }

			// Set the (N)egative and (Z)ero flag according to the value passed in.
			constexpr void setNZ(uint8_t from) { setN(from); setZ(!from); }

			[[nodiscard]] constexpr uint8_t getN() const { return N & flag::N; }
			[[nodiscard]] constexpr uint8_t getV() const { return V & flag::V; }
			[[nodiscard]] constexpr uint8_t getD() const { return DI & flag::D; }
			[[nodiscard]] constexpr uint8_t getI() const { return DI & flag::I; }
			[[nodiscard]] constexpr bool getZ() const { return Z; }
			[[nodiscard]] constexpr bool getC() const { return C; }

			// Combine the individual status flags into the unified flag word
			// exposed by the processor to the outside world. The B flag is always
			// set because the 6502 always sets it when exposing the processor flags
			// to code except when pushing it to the stack during the IRQ and NMI sequences.
			[[nodiscard]] constexpr explicit operator uint8_t() const {
				return getN() | getV() | flag::RESERVED | flag::B | DI | (Z << 1) | (uint8_t)C;
			}

			// Split the combined flag word into separate pieces.
			constexpr StatusFlags &operator=(uint8_t word) {
				N = word;
				V = word;
				DI = word & (flag::D | flag::I);
				Z = (word & flag::Z) >> 1;
				C = word & flag::C;
				return *this;
			}

		private:
			uint8_t N = 0;		// All but bit 7 are ignored
			uint8_t V = 0;		// All but bit 6 are ignored
			bool Z = false;
			bool C = false;
			uint8_t DI = 0;		// All but bits 2 and 3 must be zero
		};

		// The "bus" this processor is connected to.
		T Bus;

		// Processor registers
		uint8_t A = 0;			// Accumulator
		uint8_t X = 0;			// X index register
		uint8_t Y = 0;			// Y index register
		uint8_t SP = 0;			// Stack Pointer
		uint16_t PC = 0;		// Program Counter
		StatusFlags P;			// Processor status flags

		uint16_t TempAddr = 0;	// 16-bit working storage for instruction execution
		uint8_t TempData = 0;	// 8-bit working storage for instruction execution

		bool IrqPending = false;	// 
		bool NextIrqPending = false;// Delay IRQ action one cycle.
		bool NmiPending = false;	// Latched when the NMI line transitions low in the preceding cycle.
		uint8_t NmiMemory = ~0;		// 8 bits of history for NMI. Bit 0 is most recent state, 7 is oldest.
		bool InterruptGen = false;	// Substitute BRK for next instruction?

		// Helper functions to access optional pins on the bus
		[[nodiscard]] constexpr bool CheckRDY() const
		{
			if constexpr (HasGetRDY<T>) {
				return Bus->GetRDY();
			}
			else {
				return true;
			}
		}
		[[nodiscard]] constexpr bool CheckIRQB() const
		{
			if constexpr (HasGetIRQB<T>) {
				return Bus->GetIRQB();
			}
			else {
				return true;
			}
		}
		[[nodiscard]] constexpr bool CheckNMIB() const
		{
			if constexpr (HasGetNMIB<T>) {
				return Bus->GetNMIB();
			}
			else {
				return true;
			}
		}
		[[nodiscard]] constexpr bool CheckRESB() const
		{
			if constexpr (HasGetRESB<T>) {
				return Bus->GetRESB();
			}
			else {
				return true;
			}
		}
		[[nodiscard]] uint8_t ReadNoSideEffectsIfPossible(uint16_t addr)
		{
			if constexpr (HasReadNoSideEffects<T>) {
				return Bus->ReadNoSideEffects(addr);
			}
			else {
				return Bus->ReadAddr(addr);
			}
		}

		// State machine using pointers to member functions which return pointers
		// to the next member function require wrapping the return type in a struct
		// to avoid a recursive type definition.
		struct ExecPtrRet;
		using ExecPtr = ExecPtrRet (type::*)();
		struct ExecPtrRet {
			ExecPtrRet(ExecPtr pp) : p(pp) {}
			operator ExecPtr() { return p; }
			ExecPtr p;
		};
		using DoOpPtr = uint8_t (M6502::*)(uint8_t);

		ExecPtr State = &type::execBreak_T2;
		op BaseOp = op::RESET;

		// Fetch the next instruction and setup to begin executing it on the next cycle.
		[[nodiscard]] ExecPtrRet NextInstr()
		{
			if (InterruptGen) {
				Bus->ReadAddr(PC);	// Dummy read of pre-injection opcode
				BaseOp = NmiPending ? op::NMI : op::IRQ;
				return ModeExec[static_cast<int>(am::brk)];
			}
			else {
				// The sync handler (if present) is passed this CPU instance and the
				// address of the instruction about to be fetched.
				if constexpr (HasSyncHandler<T, M6502>) {
					Bus->SyncHandler(*this, PC);
				}

				auto op = Bus->ReadAddr(PC++);
				const BaseOpcode &base = Opc6502[op];
				BaseOp = base.Op;

				// Trap handlers (if present) are called when the opcode matches the
				// trap opcode defined by this template's instatiation. Passed a
				// reference to this CPU instance and the address of the opcode, the
				// handler may return true if it handled the opcode or false to
				// allow the opcode to execute normally. The actual PC points to the
				// byte after the opcode.
				if constexpr (trap_code >= 0) {
					if (op == trap_code) {
						if (Bus->Trap(*this, PC - 1)) {
							// Force the next cycle to call NextInstr() again instead
							// of executing whatever this instruction was supposed to
							// start executing.
							return &type::execFetch_T1;
						}
					}
				}
				return ModeExec[static_cast<int>(base.Mode)];
			}
		}

		// Check the interrupt pins and update state accordingly. This is called
		// by Tick() just before the current state is executed, so happens every
		// cycle. For reference:
		// http://visual6502.org/wiki/index.php?title=6502_Interrupt_Recognition_Stages_and_Tolerances
		constexpr void CheckInterruptPins()
		{
			// When IRQ changes, that change will be reflected in IRQP in the next
			// cycle. If IRQP is high during T0, then INTG will go high, causing the
			// next cycle (T1) to substitute BRK for the instruction, but only if
			// the interrupt disable bit is clear.
			IrqPending = NextIrqPending;
			NextIrqPending = !CheckIRQB();

			// NMI triggers when the NMIB line goes from high to low. Unlike IRQB,
			// it does not need to still be low when T0 is reached to be serviced.
			//
			// However, if NMIB is low only during the two cycles when BRK fetches
			// the vector address, then the NMI will be "lost" because the
			// processor explicitly blocks reading of the NMIB line during these
			// cycles to avoid a mixed vector read for both IRQ and NMI (optionally
			// emulated via the EmulateNMIBRKBug flag).
			NmiMemory = (NmiMemory << 1) | static_cast<decltype(NmiMemory)>(CheckNMIB());

			// If NMI was high 2 cycles ago but low in the previous cycle, the next
			// call to IntCheckT0() should trigger an NMI.
			NmiPending |= ((NmiMemory & 6) == 4);
		}

		// IntCheckT0 checks if the next instruction fetch should substitute a BRK.
		// It is called mostly from T0 states. The exception is branch statements
		// which only do this in their T0 if the branch is taken and crosses a page
		// boundary. Branch instructions are also the only ones to check from their
		// T2 states, and do so always.
		constexpr void IntCheckT0()
		{
			if (NmiPending || (!P.getI() && IrqPending)) {
				InterruptGen = true;
			}
		}

		/**************** Opcode state definitions past this point ****************/
		// Note: State numbers (Tn) match those from Visual 6502, not those from the
		// hardware manual.


		// Implicit ======================================================= 2 cycles
		// All instructions read from the byte after the opcode. Implicit ones throw
		// it away.

		ExecPtrRet execImplicit_T02()	// Dummy read of nonexistant operand
		{
			Bus->ReadAddr(PC);
			IntCheckT0();
			return &type::execCommon_T1;
		}

		// Accumulator ==================================================== 2 cycles

		ExecPtrRet execAccumulator_T02() // Dummy read while the ALU does its thing
		{
			Bus->ReadAddr(PC);
			IntCheckT0();
			return &type::execAccumulator_T1;
		}
		ExecPtrRet execAccumulator_T1()	// Perform operation + fetch next instruction
		{
			A = std::invoke(DoOp[static_cast<int>(BaseOp)], this, A);
			return NextInstr();
		}


		/* Internal Execution on Memory Data **************************************/


		// Common T0 and T1 ========================================================

		ExecPtrRet execCommon_T0()		// Read operand
		{
			TempData = Bus->ReadAddr(TempAddr);
			IntCheckT0();
			return &type::execCommon_T1;
		}
		ExecPtrRet execCommon_T1()		// Perform operation + fetch next instruction
		{
			std::invoke(DoOp[static_cast<int>(BaseOp)], this, TempData);
			return NextInstr();
		}

		// Immediate   #$00 =============================================== 2 cycles

		ExecPtrRet execImmediate_T02()	// Read operand
		{
			TempData = Bus->ReadAddr(PC++);
			IntCheckT0();
			return &type::execCommon_T1;
		}

		// Zero page   $00 ================================================ 3 cycles

		ExecPtrRet execZP_T2()			// Fetch effective address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execCommon_T0;
		}

		// Absolute   $0000 =============================================== 4 cycles

		ExecPtrRet execAbs_T2()			// Fetch low byte of effective address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execAbs_T3;
		}
		ExecPtrRet execAbs_T3()			// Fetch high byte of effective address
		{
			TempAddr |= Bus->ReadAddr(PC++) << 8;
			return &type::execCommon_T0;
		}

		// Indirect, X   ($00,X) ========================================== 6 cycles

		ExecPtrRet execIZPX_T2()		// Fetch ZP base address
		{
			TempData = Bus->ReadAddr(PC++);
			return &type::execIZPX_T3;
		}
		ExecPtrRet execIZPX_T3()		// Dummy fetch while add happens
		{
			Bus->ReadAddr(TempData);
			TempData += X;
			return &type::execIZPX_T4;
		}
		ExecPtrRet execIZPX_T4()		// Fetch low byte of effective address
		{
			TempAddr = Bus->ReadAddr(TempData);
			return &type::execIZPX_T5;
		}
		ExecPtrRet execIZPX_T5()		// Fetch high byte of effective address
		{
			TempAddr |= Bus->ReadAddr((TempData + 1) & 0xFF) << 8;
			return &type::execCommon_T0;
		}

		// Absolute, X   $0000,X ===================================== 4 or 5 cycles

		ExecPtrRet execAbsX_T2()		// Fetch low byte of base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			TempData = X;
			return &type::execAbsXY_T3;
		}
		ExecPtrRet execAbsXY_T3()		// Fetch high byte of base address
		{
			uint8_t pre_hi = Bus->ReadAddr(PC++);
			TempAddr |= pre_hi << 8;
			TempAddr += TempData;		// T2 stored X or Y in TempData for us
			TempData = pre_hi;
			return (TempAddr >> 8) != pre_hi ? &type::execAbsXY_T4 : &type::execCommon_T0;
		}
		ExecPtrRet execAbsXY_T4()		// Dummy read while performing carry
		{
			Bus->ReadAddr((TempData << 8) | (TempAddr & 0x00FF));
			return &type::execCommon_T0;
		}

		// Absolute, Y   $0000,Y ===================================== 4 or 5 cycles

		ExecPtrRet execAbsY_T2()		// Fetch low byte of base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			TempData = Y;
			return &type::execAbsXY_T3;
		}

		// Zero Page, X   $00, X ========================================== 4 cycles

		ExecPtrRet execZPX_T2()			// Fetch base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execZPX_T3;
		}
		ExecPtrRet execZPX_T3()			// Dummy read while performing add
		{
			Bus->ReadAddr(TempAddr);
			TempAddr = (TempAddr + X) & 0x00FF;
			return &type::execCommon_T0;
		}

		// Zero Page, Y   $00, Y ========================================== 4 cycles

		ExecPtrRet execZPY_T2()			// Fetch base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execZPY_T3;
		}
		ExecPtrRet execZPY_T3()			// Dummy read while performing add
		{
			Bus->ReadAddr(TempAddr);
			TempAddr = (TempAddr + Y) & 0x00FF;
			return &type::execCommon_T0;
		}

		// Indirect, Y   ($00),Y ===================================== 5 or 6 cycles

		ExecPtrRet execIZPY_T2()		// Fetch indirect address
		{
			TempData = Bus->ReadAddr(PC++);
			return &type::execIZPY_T3;
		}
		ExecPtrRet execIZPY_T3()		// Fetch low byte of base address
		{
			TempAddr = Bus->ReadAddr(TempData);
			return &type::execIZPY_T4;
		}
		ExecPtrRet execIZPY_T4()		// Fetch high byte of base address
		{
			TempAddr |= Bus->ReadAddr((TempData + 1) & 0x00FF) << 8;
			TempData = TempAddr >> 8;
			TempAddr += Y;
			return (TempAddr >> 8) != TempData ? &type::execAbsXY_T4 : &type::execCommon_T0;
		}


		/* Store Operations *******************************************************/


		// Common T0 and T1 for Store/RMW ==========================================

		ExecPtrRet execCommon_Sto_T0()	// Write register/modified data to memory
		{
			// AHX/SHX/SHY/TAS can potentially change the target address, so
			// the operator must be invoked separately from the WriteAddr call.
			uint8_t output = std::invoke(DoOp[static_cast<int>(BaseOp)], this, TempData);
			Bus->WriteAddr(TempAddr, output);
			IntCheckT0();
			return &type::execFetch_T1;
		}
		ExecPtrRet execFetch_T1()	// Fetch next instruction
		{
			return NextInstr();
		}

		// Zero page   STx $00 ============================================ 3 cycles

		ExecPtrRet execZP_Sto_T2()		// Fetch effective address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execCommon_Sto_T0;
		}

		// Absolute   STx $0000 =========================================== 4 cycles

		ExecPtrRet execAbs_Sto_T2()		// Fetch low byte of effective address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execAbs_Sto_T3;
		}
		ExecPtrRet execAbs_Sto_T3()		// Fetch high byte of effective address
		{
			TempAddr |= Bus->ReadAddr(PC++) << 8;
			return &type::execCommon_Sto_T0;
		}

		// Indirect, X   STx ($00,X) ====================================== 6 cycles

		ExecPtrRet execIZPX_Sto_T2()	// Fetch ZP base address
		{
			TempData = Bus->ReadAddr(PC++);
			return &type::execIZPX_Sto_T3;
		}
		ExecPtrRet execIZPX_Sto_T3()	// Dummy fetch while add happens
		{
			Bus->ReadAddr(TempData);
			TempData += X;
			return &type::execIZPX_Sto_T4;
		}
		ExecPtrRet execIZPX_Sto_T4()	// Fetch low byte of effective address
		{
			TempAddr = Bus->ReadAddr(TempData);
			return &type::execIZPX_Sto_T5;
		}
		ExecPtrRet execIZPX_Sto_T5()	// Fetch high byte of effective address
		{
			TempAddr |= Bus->ReadAddr((TempData + 1) & 0xFF) << 8;
			return &type::execCommon_Sto_T0;
		}

		// Absolute, X   Stx $0000,X ====================================== 5 cycles

		ExecPtrRet execAbsX_Sto_T2()	// Fetch low byte of base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execAbsX_Sto_T3;
		}
		ExecPtrRet execAbsX_Sto_T3()	// Fetch high byte of base address
		{
			TempAddr |= Bus->ReadAddr(PC++) << 8;
			return &type::execAbsX_Sto_T4;
		}
		ExecPtrRet execAbsX_Sto_T4()	// Dummy read, maybe performing carry
		{
			uint16_t pre_hi = TempAddr & 0xFF00;
			TempAddr += X;
			Bus->ReadAddr(pre_hi | (TempAddr & 0x00FF));
			TempData = uint8_t(pre_hi >> 8);	// Record for AHX/SHX/SHY/TAS
			return &type::execCommon_Sto_T0;
		}

		// Absolute, Y   STx $0000,Y ====================================== 5 cycles

		ExecPtrRet execAbsY_Sto_T2()	// Fetch low byte of base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execAbsY_Sto_T3;
		}
		ExecPtrRet execAbsY_Sto_T3()	// Fetch high byte of base address
		{
			TempAddr |= Bus->ReadAddr(PC++) << 8;
			return &type::execAbsY_Sto_T4;
		}
		ExecPtrRet execAbsY_Sto_T4()	// Dummy read, maybe performing carry
		{
			uint16_t pre_hi = TempAddr & 0xFF00;
			TempAddr += Y;
			Bus->ReadAddr(pre_hi | (TempAddr & 0x00FF));
			TempData = uint8_t(pre_hi >> 8);	// Record for AHX/SHX/SHY/TAS
			return &type::execCommon_Sto_T0;
		}

		// Zero Page, X   Stx $00, X ====================================== 4 cycles

		ExecPtrRet execZPX_Sto_T2()		// Fetch base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execZPX_Sto_T3;
		}
		ExecPtrRet execZPX_Sto_T3()		// Dummy read while performing add
		{
			Bus->ReadAddr(TempAddr);
			TempAddr = (TempAddr + X) & 0x00FF;
			return &type::execCommon_Sto_T0;
		}

		// Zero Page, Y   Stx $00, Y ====================================== 4 cycles

		ExecPtrRet execZPY_Sto_T2()		// Fetch base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execZPY_Sto_T3;
		}
		ExecPtrRet execZPY_Sto_T3()		// Dummy read while performing add
		{
			Bus->ReadAddr(TempAddr);
			TempAddr = (TempAddr + Y) & 0x00FF;
			return &type::execCommon_Sto_T0;
		}

		// Indirect, Y   Stx ($00),Y ====================================== 6 cycles

		ExecPtrRet execIZPY_Sto_T2()	// Fetch indirect address
		{
			TempData = Bus->ReadAddr(PC++);
			return &type::execIZPY_Sto_T3;
		}
		ExecPtrRet execIZPY_Sto_T3()	// Fetch low byte of base address
		{
			TempAddr = Bus->ReadAddr(TempData);
			return &type::execIZPY_Sto_T4;
		}
		ExecPtrRet execIZPY_Sto_T4()	// Fetch high byte of base address
		{
			TempAddr |= Bus->ReadAddr((TempData + 1) & 0xFF) << 8;
			return &type::execAbsY_Sto_T4;
		}


		/* Read-Modify-Write Operations *******************************************/


		// Common cycles for RMW ===================================================

		ExecPtrRet execCommon_RMW_SD1()	// Fetch data
		{
			TempData = Bus->ReadAddr(TempAddr);
			return &type::execCommon_RMW_SD2;
		}
		ExecPtrRet execCommon_RMW_SD2()	// Dummy write
		{
			Bus->WriteAddr(TempAddr, TempData);
			return &type::execCommon_Sto_T0;
		}

		// Zero page   RMW $00 ============================================ 5 cycles

		ExecPtrRet execZP_RMW_T2()		// Fetch effective address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execCommon_RMW_SD1;
		}

		// Absolute   RMW $0000 =========================================== 6 cycles

		ExecPtrRet execAbs_RMW_T2()		// Fetch low byte of effective address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execAbs_RMW_T3;
		}
		ExecPtrRet execAbs_RMW_T3()		// Fetch high byte of effective address
		{
			TempAddr |= Bus->ReadAddr(PC++) << 8;
			return &type::execCommon_RMW_SD1;
		}

		// Zero Page, X   RMW $00, X ====================================== 6 cycles

		ExecPtrRet execZPX_RMW_T2()		// Fetch base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execZPX_RMW_T3;
		}
		ExecPtrRet execZPX_RMW_T3()		// Dummy read while performing add
		{
			Bus->ReadAddr(TempAddr);
			TempAddr = (TempAddr + X) & 0xFF;
			return &type::execCommon_RMW_SD1;
		}

		// Absolute, X   RMW $0000,X ====================================== 7 cycles

		ExecPtrRet execAbsX_RMW_T2()	// Fetch low byte of base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execAbsX_RMW_T3;
		}
		ExecPtrRet execAbsX_RMW_T3()	// Fetch high byte of base address
		{
			TempAddr |= Bus->ReadAddr(PC++) << 8;
			return &type::execAbsX_RMW_T4;
		}
		ExecPtrRet execAbsX_RMW_T4()	// Dummy read, maybe performing carry
		{
			uint16_t pre_hi = TempAddr & 0xFF00;
			TempAddr += X;
			Bus->ReadAddr(pre_hi | (TempAddr & 0x00FF));
			return &type::execCommon_RMW_SD1;
		}

		// Absolute, Y   RMW $0000,Y ==== Illegal ========================= 7 cycles

		ExecPtrRet execAbsY_RMW_T2()	// Fetch low byte of base address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execAbsY_RMW_T3;
		}
		ExecPtrRet execAbsY_RMW_T3()	// Fetch high byte of base address
		{
			TempAddr |= Bus->ReadAddr(PC++) << 8;
			return &type::execAbsY_RMW_T4;
		}
		ExecPtrRet execAbsY_RMW_T4()	// Dummy read, maybe performing carry
		{
			uint16_t pre_hi = TempAddr & 0xFF00;
			TempAddr += Y;
			Bus->ReadAddr(pre_hi | (TempAddr & 0x00FF));
			return &type::execCommon_RMW_SD1;
		}

		// Indirect, X   RMW ($00,X) ==== Illegal ========================= 8 cycles

		ExecPtrRet execIZPX_RMW_T2()	// Fetch ZP base address
		{
			TempData = Bus->ReadAddr(PC++);
			return &type::execIZPX_RMW_T3;
		}
		ExecPtrRet execIZPX_RMW_T3()	// Dummy fetch while add happens
		{
			Bus->ReadAddr(TempData);
			TempData += X;
			return &type::execIZPX_RMW_T4;
		}
		ExecPtrRet execIZPX_RMW_T4()	// Fetch low byte of effective address
		{
			TempAddr = Bus->ReadAddr(TempData);
			return &type::execIZPX_RMW_T5;
		}
		ExecPtrRet execIZPX_RMW_T5()	// Fetch high byte of effective address
		{
			TempAddr |= Bus->ReadAddr((TempData + 1) & 0xFF) << 8;
			return &type::execCommon_RMW_SD1;
		}

		// Indirect, Y   RMW ($00),Y ==== Illegal ========================= 8 cycles

		ExecPtrRet execIZPY_RMW_T2()	// Fetch indirect address
		{
			TempData = Bus->ReadAddr(PC++);
			return &type::execIZPY_RMW_T3;
		}
		ExecPtrRet execIZPY_RMW_T3()	// Fetch low byte of base address
		{
			TempAddr = Bus->ReadAddr(TempData);
			return &type::execIZPY_RMW_T4;
		}
		ExecPtrRet execIZPY_RMW_T4()	// Fetch high byte of base address
		{
			TempAddr |= Bus->ReadAddr((TempData + 1) & 0xFF) << 8;
			return &type::execAbsY_RMW_T4;
		}


		/* Miscellaneous Operations ***********************************************/


		// Push Operation ================================================= 3 cycles

		ExecPtrRet execPush_T2()		// Dummy read
		{
			Bus->ReadAddr(PC);
			return &type::execPush_T0;
		}
		ExecPtrRet execPush_T0()		// Write register to stack
		{
			Bus->WriteAddr(STACK_PAGE | SP, std::invoke(DoOp[static_cast<int>(BaseOp)], this, 0));
			--SP;
			IntCheckT0();
			return &type::execFetch_T1;
		}

		// Pull Operation ================================================= 4 cycles

		ExecPtrRet execPull_T2()		// Dummy read from PC+1
		{
			Bus->ReadAddr(PC);
			return &type::execPull_T3;
		}
		ExecPtrRet execPull_T3()		// Dummy read from SP
		{
			Bus->ReadAddr(STACK_PAGE | SP);
			++SP;
			return &type::execPull_T0;
		}
		ExecPtrRet execPull_T0()		// Fetch data from stack
		{
			TempData = Bus->ReadAddr(STACK_PAGE | SP);
			IntCheckT0();
			return &type::execCommon_T1;
		}

		// Jump Absolute   JMP $0000 ====================================== 3 cycles

		ExecPtrRet execJump_T2()		// Fetch low byte of target address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execJump_T0;
		}
		ExecPtrRet execJump_T0()		// Fetch high byte of target address
		{
			TempAddr |= Bus->ReadAddr(PC) << 8;
			IntCheckT0();
			return &type::execJump_T1;
		}
		ExecPtrRet execJump_T1()		// Fetch next instruction
		{
			PC = TempAddr;
			return NextInstr();
		}

		// Jump Indirect   JMP ($0000) ==================================== 5 cycles

		ExecPtrRet execJumpInd_T2()		// Fetch low byte of indirect address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execJumpInd_T3;
		}
		ExecPtrRet execJumpInd_T3()		// Fetch high byte of indirect address
		{
			TempAddr |= Bus->ReadAddr(PC++) << 8;
			return &type::execJumpInd_T4;
		}
		ExecPtrRet execJumpInd_T4()		// Fetch low byte of target address
		{
			TempData = Bus->ReadAddr(TempAddr);
			return &type::execJumpInd_T0;
		}
		ExecPtrRet execJumpInd_T0()		// Fetch high byte of target address
		{
			// On the NMOS 6502, a page boundary is not crossed here
			uint16_t addr = (TempAddr & 0xFF00) | ((TempAddr + 1) & 0x00FF);
			PC = TempData | (Bus->ReadAddr(addr) << 8);
			IntCheckT0();
			return &type::execJumpInd_T1;
		}
		ExecPtrRet execJumpInd_T1()		// Fetch next instruction
		{
			return NextInstr();
		}

		// Jump to Subroution   JSR $0000 ================================= 6 cycles

		ExecPtrRet execCall_T2()		// Fetch low byte of target address
		{
			TempAddr = Bus->ReadAddr(PC++);
			return &type::execCall_T3;
		}
		ExecPtrRet execCall_T3()		// Dummy read from stack
		{
			Bus->ReadAddr(STACK_PAGE | SP);
			return &type::execCall_T4;
		}
		ExecPtrRet execCall_T4()		// Push high byte of PC
		{
			Bus->WriteAddr(STACK_PAGE | SP--, PC >> 8);
			return &type::execCall_T5;
		}
		ExecPtrRet execCall_T5()		// Push low byte of PC
		{
			Bus->WriteAddr(STACK_PAGE | SP--, PC & 0xFF);
			return &type::execCall_T0;
		}
		ExecPtrRet execCall_T0()		// Fetch high byte of target address
		{
			TempAddr |= Bus->ReadAddr(PC) << 8;
			IntCheckT0();
			return &type::execJump_T1;
		}

		// Return from Subroutine   RTS =================================== 6 cycles

		ExecPtrRet execRTS_T2()			// Dummy read of byte after opcode
		{
			Bus->ReadAddr(PC);
			return &type::execRTS_T3;
		}
		ExecPtrRet execRTS_T3()			// Dummy read from stack
		{
			Bus->ReadAddr(STACK_PAGE | SP);
			return &type::execRTS_T4;
		}
		ExecPtrRet execRTS_T4()			// Pull PCL from stack
		{
			TempAddr = Bus->ReadAddr(STACK_PAGE | ++SP);
			return &type::execRTS_T5;
		}
		ExecPtrRet execRTS_T5()			// Pull PCH from stack
		{
			TempAddr |= Bus->ReadAddr(STACK_PAGE | ++SP) << 8;
			return &type::execRTS_T0;
		}
		ExecPtrRet execRTS_T0()			// Dummy read to increment PC
		{
			PC = TempAddr;
			Bus->ReadAddr(PC++);
			IntCheckT0();
			return &type::execFetch_T1;
		}

		// BRK/IRQ/NMI/RESET ============================================== 7 cycles
		// The BRK instruction sequence is reused for BRK/IRQ/NMI/RESET. When used
		// for RESET, the writes to the stack are turned into reads.

		ExecPtrRet execBreak_T2()		// Dummy read of PC
		{
			// If present, the break handler will be called for a software interrupt
			// (i.e. the BRK instruction is executing). It is passed a reference to
			// this CPU instance and the address of the BRK opcode. Returning true
			// will cancel the rest of the interrupt sequence and continue execution
			// at PC, which, unless changed by the handler, points one byte past BRK
			// (not two bytes past as would be the case if the sequence continued
			// and RTI was used to return to this point).
			if constexpr (HasBreakHandler<T, M6502>) {
				if (BaseOp == op::BRK && Bus->BreakHandler(*this, PC - 1)) {
					return NextInstr();
				}
			}
			Bus->ReadAddr(PC);
			// Software break increments PC; Hardware break does not
			if (BaseOp == op::BRK) {
				++PC;
			}
			return &type::execBreak_T3;
		}
		ExecPtrRet execBreak_T3()		// Push high order byte of PC to stack
		{
			const uint16_t addr = STACK_PAGE | SP--;
			if (BaseOp != op::RESET) {
				Bus->WriteAddr(addr, PC >> 8);
			} else {
				Bus->ReadAddr(addr);
			}
			return &type::execBreak_T4;
		}
		ExecPtrRet execBreak_T4()		// Push low order byte of PC to stack
		{
			const uint16_t addr = STACK_PAGE | SP--;
			if (BaseOp != op::RESET) {
				Bus->WriteAddr(addr, PC & 0xFF);
			} else {
				Bus->ReadAddr(addr);
			}
			return &type::execBreak_T5;
		}
		ExecPtrRet execBreak_T5()		// Push status register to stack
		{
			const uint16_t addr = STACK_PAGE | SP--;
			if (BaseOp != op::RESET) {
				auto status = static_cast<uint8_t>(P);
				if (BaseOp != op::BRK) {
					status &= ~0x10;	// HW interrupts force the BRK flag to 0
				}
				Bus->WriteAddr(addr, status);
			} else {
				Bus->ReadAddr(addr);
			}

			if (EmulateNMIBRKBug) {
				if (NmiPending) {
					// Force servicing of the NMI, even if this was a BRK.
					BaseOp = op::NMI;
				}
				// Copy bit 1 to bit 0 to prevent NMI detection during vector fetch
				NmiMemory = (NmiMemory & ~1) | ((NmiMemory >> 1) & 1);
			}
			return &type::execBreak_T6;
		}
		ExecPtrRet execBreak_T6()		// Fetch low order byte of interrupt vector
		{
			InterruptGen = false;
			P.setI(true);
			TempAddr = Bus->ReadAddr(opToVector(BaseOp));

			if (EmulateNMIBRKBug) {
				// Copy bit 1 to bit 0 to prevent NMI detection during vector fetch
				NmiMemory = (NmiMemory & ~1) | ((NmiMemory >> 1) & 1);
			}
			return &type::execBreak_T0;
		}
		ExecPtrRet execBreak_T0()		// Fetch high order byte of interrupt vector
		{
			PC = TempAddr | (Bus->ReadAddr(opToVector(BaseOp) + 1) << 8);
			if (BaseOp == op::NMI) {
				NmiPending = false;
			}
			IntCheckT0();
			return &type::execFetch_T1;
		}

		// Return from Interrupt ========================================== 6 cycles

		ExecPtrRet execRTI_T2()			// Dummy fetch of PC+1
		{
			Bus->ReadAddr(PC);
			return &type::execRTI_T3;
		}
		ExecPtrRet execRTI_T3()			// Dummy fetch from stack
		{
			Bus->ReadAddr(STACK_PAGE | SP++);
			return &type::execRTI_T4;
		}
		ExecPtrRet execRTI_T4()			// Pull P from stack
		{
			P = Bus->ReadAddr(STACK_PAGE | SP++);
			return &type::execRTI_T5;
		}
		ExecPtrRet execRTI_T5()			// Pull PCL from stack
		{
			TempAddr = Bus->ReadAddr(STACK_PAGE | SP++);
			return &type::execRTI_T0;
		}
		ExecPtrRet execRTI_T0()			// Pull PCH from stack
		{
			PC = TempAddr | (Bus->ReadAddr(STACK_PAGE | SP) << 8);
			IntCheckT0();
			return &type::execFetch_T1;
		}

		// Branches ============================================== 2, 3, or 4 cycles

		ExecPtrRet execBranch_T2()	// Read offset
		{
			TempData = Bus->ReadAddr(PC++);
			IntCheckT0();
			return &type::execBranch_T3;
		}
		ExecPtrRet execBranch_T3()	// Check condition
		{
			if (!std::invoke(DoOp[static_cast<int>(BaseOp)], this, 0)) {
				// No branch, so do next instruction
				return NextInstr();
			} else {
				// Branching, but we still need to read the instruction
				// that would have been executed if we didn't branch.
				Bus->ReadAddr(PC);
				return &type::execBranch_T0;
			}
		}
		ExecPtrRet execBranch_T0()	// Take branch, if no carry
		{
			uint16_t newPC = PC + (int8_t)TempData;
			if ((newPC ^ PC) & 0xFF00) {
				// Carry needed, so dummy read from incomplete PC sum
				PC = (PC & 0xFF00) | (newPC & 0xFF);
				Bus->ReadAddr(PC);
				// Complete branch in next cycle
				TempAddr = newPC;
				// This cycle behaves as a T0 for interrupt purposes
				// when the branch crosses a page boundary.
				IntCheckT0();
				return &type::execBranch_T1;
			}
			else {
				// No carry needed, so do real instruction fetch now
				PC = newPC;
				return NextInstr();
			}
		}
		ExecPtrRet execBranch_T1()	// Take branch, carry complete
		{
			PC = TempAddr;
			return NextInstr();
		}

		// Halt / Kill / Jam / Crap ======================================== Forever

		ExecPtrRet execHalt_T2() // Dummy read of PC+1
		{
			Bus->ReadAddr(PC++);
			return &type::execHalt_T3;
		}
		ExecPtrRet execHalt_T3() // Put $FFFF on the address bus
		{
			Bus->ReadAddr(0xFFFF);
			return &type::execHalt_T4;
		}
		ExecPtrRet execHalt_T4() // Put $FFFE on the address bus
		{
			Bus->ReadAddr(0xFFFE);
			return &type::execHalt_T5;
		}
		ExecPtrRet execHalt_T5() // Put $FFFE on the address bus
		{
			Bus->ReadAddr(0xFFFE);
			return &type::execHalt_TX;
		}
		ExecPtrRet execHalt_TX() // Put $FFFF on the address bus until reset
		{
			Bus->ReadAddr(0xFFFF);
			return &type::execHalt_TX;
		}

		/******************************* Operations *******************************/

		/* There are different classes of operations, but for the convenience of
		 * being able to use a single table to hold them all, they all share the
		 * same function signature:
		 * 
		 * 1. Retrieve some bit of internal data for the addressing mode to work
		 *    with. These functions ignore their parameter but return a value.
		 *    For instance, branch operations return the branch condition and
		 *    store operations return the register contents.
		 *
		 * 2. Operations that modify some internal state based on memory data.
		 *    These operations use their parameter, but their return value is
		 *    ignored. For instance, math operators like ADC only use their
		 *    parameter to modify the accumulator, so their return value is
		 *    meaningless.
		 * 
		 * 3. Operations that implicitly perform some internal state change.
		 *    Their parameter and return value are both ignored. This includes
		 *    processor flag changing instructions like SEI or register transfer
		 *    instructions like TXA.
		 * 
		 * 4. Operations that can modify either a memory location or the accumulator
		 *    depending on their addressing mode. Their parameter is the original
		 *    value and they return the modified value. This includes all the RMW
		 *    instructions like DEC and ASL.
		 */

		// Branch on Carry Clear
		uint8_t getBCC(uint8_t)
		{
			return !P.getC();
		}

		// Branch on Carry Set
		uint8_t getBCS(uint8_t)
		{
			return P.getC();
		}

		// Branch on Result not Zero
		uint8_t getBNE(uint8_t)
		{
			return !P.getZ();
		}

		// Branch on Result Zero
		uint8_t getBEQ(uint8_t)
		{
			return P.getZ();
		}

		// Branch on Result Plus
		uint8_t getBPL(uint8_t)
		{
			return !P.getN();
		}

		// Branch on Result Minus
		uint8_t getBMI(uint8_t)
		{
			return P.getN();
		}

		// Branch on Overflow Clear
		uint8_t getBVC(uint8_t)
		{
			return !P.getV();
		}

		// Branch on Overflow Set
		uint8_t getBVS(uint8_t)
		{
			return P.getV();
		}

		uint8_t addBinary(uint8_t data)
		{
			uint16_t added = A + data + P.getC();
			auto new_a = static_cast<uint8_t>(added);
			// The overflow flag is set IFF the inputs both had
			// the same sign but the output has a different sign.
			// In this case, this is expressed as both input bits
			// are different from the output bit.
			P.setV(((A ^ added) & (data ^ added)) >> 1);	// >>1 to get it into bit 6
			P.setC(static_cast<bool>(added >> 8));
			P.setNZ(new_a);
			return new_a;
		}

		// Algorithms for decimal mode addition/subtraction are described at
		// http://www.6502.org/tutorials/decimal_mode.html

		// Add Memory to Accumulator with Carry
		uint8_t doADC(uint8_t data)
		{
			if (!P.getD()) {
				// Regular binary addition
				A = addBinary(data);
			}
			else {
				// Decimal addition flags are weird.
				// Z flag is based on the binary addition.
				P.setZ(!((A + data + P.getC()) & 0xFF));

				uint8_t lo = (A & 0x0F) + (data & 0x0F) + P.getC();
				if (lo > 9) {
					lo = ((lo + 6) & 0x0F) + 0x10;
				}
				uint16_t r = (A & 0xF0) + (data & 0xF0) + lo;
				// N and V are calculated before doing the decimal adjust
				// on the high nibble.
				P.setN(uint8_t(r));
				P.setV(((A ^ r) & (data ^ r)) >> 1);
				if (r >= 0xA0) {
					r += 0x60;
				}
				// C is the only flag that works "as expected".
				P.setC(r >= 256);
				A = uint8_t(r & 0xFF);
			}
			return 0;
		}

		// Subtract Memory from Accumulator with Borrow
		uint8_t doSBC(uint8_t data)
		{
			if (!P.getD()) {
				// The 6502 implements binary subtraction by adding the 1's
				// complement of the data value to the accumulator.
				A = addBinary(~data);
			}
			else {
				// Decimal mode subtraction.
				int8_t lo = (A & 0x0F) - (data & 0x0F) + P.getC() - 1;
				if (lo < 0) {
					lo = ((lo - 6) & 0x0F) - 0x10;
				}
				int16_t r = (A & 0xF0) - (data & 0xF0) + lo;
				if (r < 0) {
					r -= 0x60;
				}
				// In the 6502 (but not 65C02), subtraction always sets the
				// flags as if the subtraction had been done in binary.
				addBinary(~data);
				A = uint8_t(r & 0xFF);
			}
			return 0;
		}

		// "AND" Memory with Accumulator
		uint8_t doAND(uint8_t data)
		{
			A &= data;
			P.setNZ(A);
			return 0;
		}

		// Shift left One Bit (Memory or Accumulator)
		uint8_t doASL(uint8_t data)
		{
			P.setC(!!(data & 0x80));
			data <<= 1;
			P.setNZ(data);
			return data;
		}

		// Test Bits in Memory with Accumulator
		uint8_t doBIT(uint8_t data)
		{
			P.setZ(!(A & data));
			P.setN(data);
			P.setV(data);
			return 0;
		}

		// Clear Carry Flag
		uint8_t doCLC(uint8_t)
		{
			P.setC(false);
			return 0;
		}

		// Clear Decimal Mode
		uint8_t doCLD(uint8_t)
		{
			P.setD(false);
			return 0;
		}

		// Clear Interrupt Disable Bit
		uint8_t doCLI(uint8_t)
		{
			P.setI(false);
			return 0;
		}

		// Clear Overflow Flag
		uint8_t doCLV(uint8_t)
		{
			P.setV(0);
			return 0;
		}

		constexpr uint16_t doCompare(uint8_t a, uint8_t b)
		{
			uint16_t diff = a - b;
			P.setNZ(uint8_t(diff));
			P.setC(!(diff & 0xFF00));
			return diff;	// Returned for AXS
		}

		// Compare Memory with Accumulator
		uint8_t doCMP(uint8_t data)
		{
			doCompare(A, data);
			return 0;
		}

		// Compare Memory and Index X
		uint8_t doCPX(uint8_t data)
		{
			doCompare(X, data);
			return 0;
		}

		// Compare Memory and Index Y
		uint8_t doCPY(uint8_t data)
		{
			doCompare(Y, data);
			return 0;
		}

		// Decrement Memory by One
		uint8_t doDEC(uint8_t data)
		{
			P.setNZ(--data);
			return data;
		}

		// Decrement Index X by One
		uint8_t doDEX(uint8_t)
		{
			P.setNZ(--X);
			return 0;
		}

		// Decrement Index Y by One
		uint8_t doDEY(uint8_t)
		{
			P.setNZ(--Y);
			return 0;
		}

		// "Exclusive-Or" Memory with Accumulator
		uint8_t doEOR(uint8_t data)
		{
			A ^= data;
			P.setNZ(A);
			return 0;
		}

		// Increment Memory by One
		uint8_t doINC(uint8_t data)
		{
			data += 1;
			P.setNZ(data);
			return data;
		}

		// Increment Index X by One
		uint8_t doINX(uint8_t)
		{
			P.setNZ(++X);
			return 0;
		}

		// Increment Index Y by One
		uint8_t doINY(uint8_t)
		{
			P.setNZ(++Y);
			return 0;
		}

		// Load Accumulator with Memory
		uint8_t doLDA(uint8_t data)
		{
			A = data;
			P.setNZ(A);
			return 0;
		}

		// Load Index X with Memory
		uint8_t doLDX(uint8_t data)
		{
			X = data;
			P.setNZ(X);
			return 0;
		}

		// Load Index Y with Memory
		uint8_t doLDY(uint8_t data)
		{
			Y = data;
			P.setNZ(Y);
			return 0;
		}

		// Shift Right One Bit (Memory or Accumulator)
		uint8_t doLSR(uint8_t data)
		{
			P.setC(bool(data & 1));
			data >>= 1;
			P.setZ(!data);
			P.setN(false);
			return data;
		}

		// No Operation
		uint8_t doNOP(uint8_t)
		{
			return 0;
		}

		// "OR" Memory with Accumulator
		uint8_t doORA(uint8_t data)
		{
			A |= data;
			P.setNZ(A);
			return 0;
		}

		// PHA - Push Accumulator on Stack
		// STA - Store Accumulator in Memory
		uint8_t doGetA(uint8_t)
		{
			return A;
		}

		// Push Processor Status on Stack
		uint8_t doPHP(uint8_t)
		{
			return uint8_t(P);
		}

		// Pull Accumulator from Stack
		uint8_t doPLA(uint8_t data)
		{
			A = data;
			P.setNZ(A);
			return 0;
		}

		// Pull Processor Status from Stack
		uint8_t doPLP(uint8_t data)
		{
			P = data;
			return 0;
		}

		// Rotate One Bit Left (Memory or Accumulator)
		uint8_t doROL(uint8_t data)
		{
			uint8_t shift_in = P.getC();
			P.setC(bool(data >> 7));
			data = (data << 1) | shift_in;
			P.setNZ(data);
			return data;
		}

		// Rotate One Bit Right (Memory or Accumulator)
		uint8_t doROR(uint8_t data)
		{
			uint8_t shift_in = P.getC() << 7;
			P.setC(bool(data & 1));
			data = (data >> 1) | shift_in;
			P.setNZ(data);
			return data;
		}

		// Set Carry Flag
		uint8_t doSEC(uint8_t)
		{
			P.setC(true);
			return 0;
		}

		// Set Decimal Mode
		uint8_t doSED(uint8_t)
		{
			P.setD(true);
			return 0;
		}

		// Set Interrupt Disable Status
		uint8_t doSEI(uint8_t)
		{
			P.setI(true);
			return 0;
		}

		// STX - Store Index X in Memory
		uint8_t doGetX(uint8_t)
		{
			return X;
		}

		// STY - Store Index Y in Memory
		uint8_t doGetY(uint8_t)
		{
			return Y;
		}

		// Transfer Accumulator to Index X
		uint8_t doTAX(uint8_t)
		{
			X = A;
			P.setNZ(X);
			return 0;
		}

		// Transfer Accumulator to Index Y
		uint8_t doTAY(uint8_t)
		{
			Y = A;
			P.setNZ(Y);
			return 0;
		}

		// Transfer Stack Pointer to Index X
		uint8_t doTSX(uint8_t)
		{
			X = SP;
			P.setNZ(X);
			return 0;
		}

		// Transfer Index X to Accumulator
		uint8_t doTXA(uint8_t)
		{
			A = X;
			P.setNZ(A);
			return 0;
		}

		// Transfer Index X to Stack Pointer
		uint8_t doTXS(uint8_t)
		{
			SP = X;
			// Unlike TSX, does not modify condition codes
			return 0;
		}

		// Transfer Index Y to Accumulator
		uint8_t doTYA(uint8_t)
		{
			A = Y;
			P.setNZ(A);
			return 0;
		}

		// Illegal operations !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//
		// NMOS 6510 Unintended Opcodes "No More Secrets" v0.95 - 24/12/20
		// used as reference for these. <https://csdb.dk/release/?id=198357>

		// ALR: AND + LSR
		//   A = (A & #{imm}) / 2
		uint8_t doALR(uint8_t data)
		{
			A = doLSR(A & data);
			return 0;
		}

		// ANC: AND + ASL/ROL
		//   A = A & #{imm}
		uint8_t doANC(uint8_t data)
		{
			A &= data;
			P.setNZ(A);
			P.setC(bool(A >> 7));
			return 0;
		}

		// ARR: AND/ADC + ROR
		//   A = (A & #{imm}) / 2
		uint8_t doARR(uint8_t data)
		{
			data &= A;
			if (!P.getD()) {
				uint8_t rored = (data >> 1) | (P.getC() << 7);
				P.setC(!!(data & 0x80));	// C = input bit 7
				P.setV(rored ^ data);		// V = input bit 7 xor input bit 6
				P.setNZ(rored);
				A = rored;
			}
			else {
				// Decimal mode funiness
				uint8_t tmp = (data >> 1) | (P.getC() << 7);
				P.setV(data ^ tmp);			// V = input bit 7 xor input bit 6
				P.setNZ(tmp);
				if ((data & 0x0F) >= 0x05) {
					tmp = (tmp & 0xF0) | ((tmp + 6) & 0x0F);
				}
				if (data >= 0x50) {
					tmp += 0x60;
					P.setC(true);			// C = high nibble was decimal adjusted
				}
				else {
					P.setC(false);
				}
				A = tmp;
			}
			return 0;
		}

		// AXS: CMP + DEX					(aka SBX, SAX, XMA)
		//   X = A & X - #{imm}
		uint8_t doAXS(uint8_t data)
		{
			auto diff = doCompare(A & X, data);
			X = uint8_t(diff);
			return 0;
		}

		// DCP: DEC + CMP
		//   {addr} = {addr} - 1   A cmp {addr}
		uint8_t doDCP(uint8_t data)
		{
			--data;
			doCompare(A, data);
			return data;
		}

		// ISC: INC + SBC
		//   {addr} = {addr} + 1   A = A - {addr}
		uint8_t doISC(uint8_t data)
		{
			++data;
			doSBC(data);
			return data;
		}

		// LAS: STA/TXS + LDA/TSX (maybe unstable?)		(aka LAR)
		//   A,X,SP = {addr} & SP
		uint8_t doLAS(uint8_t data)
		{
			A = X = SP = data &= SP;
			P.setNZ(data);
			return 0;
		}

		// LAX: LDA + LDX
		uint8_t doLAX(uint8_t data)
		{
			A = data;
			X = data;
			P.setNZ(data);
			return 0;
		}

		// RLA: ROL + AND
		//   {addr} = rol {addr}   A = A and {addr}
		uint8_t doRLA(uint8_t data)
		{
			data = doROL(data);
			A &= data;
			P.setNZ(A);
			return data;
		}

		// RRA: ROR + ADC
		//   {addr} = ror {addr}   A = A adc {addr}
		uint8_t doRRA(uint8_t data)
		{
			data = doROR(data);
			doADC(data);
			return data;
		}

		// SAX: STA + STX
		//   {addr} = A & X
		uint8_t doSAX(uint8_t)
		{
			return A & X;
		}

		// SLO: ASL + ORA
		//   {addr} = {addr} * 2   A = A or {addr}
		uint8_t doSLO(uint8_t data)
		{
			data = doASL(data);
			A |= data;
			P.setNZ(A);
			return data;
		}

		// SRE: LSR + EOR
		//   {addr} = {addr} / 2   A = A eor {addr}
		uint8_t doSRE(uint8_t data)
		{
			data = doLSR(data);
			A ^= data;
			P.setNZ(A);
			return data;
		}

		// Unstable address high byte opcodes --------------------------------------
		//
		// If a page boundary is crossed, the high byte of the target address is
		// ANDed with the value stored. execAbsX_Sto_T4 and execAbsY_Sto_T4 store
		// the preindexed address high byte in TempData, which gets passed to us
		// as prehi by execCommon_Sto_T0. If RDY is pulled low during T0 (which is
		// when these functions are called), then the & {H+1} is left off of the
		// stored value.

		uint8_t doUnstableHi(uint8_t value, uint8_t prehi)
		{
			if (prehi != uint8_t(TempAddr >> 8)) {
				// Page boundary was crossed
				TempAddr &= (uint16_t(value) << 8) | 0xFF;
			}
			if (CheckRDY()) {
				value &= prehi + 1;
			}
			return value;
		}

		// AHX: STA/STX/STY						(aka SHA, AXA, TEA)
		//   {addr} = A & X & {H+1}
		uint8_t doAHX(uint8_t prehi)
		{
			return doUnstableHi(A & X, prehi);
		}

		// SHX: STA/STX/STY						(aka A11, SXA, XAS, TEX)
		//   {addr} = X & {H+1}
		uint8_t doSHX(uint8_t prehi)
		{
			return doUnstableHi(X, prehi);
		}

		// SHY: STA/STX/STY						(aka A11, SYA, SAY, TEY)
		//   {addr} = Y & {H+1}
		uint8_t doSHY(uint8_t prehi)
		{
			return doUnstableHi(Y, prehi);
		}

		// TAS: STA/TXS , LDA/TSX				(aka XAS, SHS)
		//   SP = A & X   {addr} = A & X & {H+1}
		uint8_t doTAS(uint8_t prehi)
		{
			SP = A & X;
			return doUnstableHi(SP, prehi);
		}

		// "Magic constant" unstable opcodes ---------------------------------------
		// Operation depends on a "magic" value that is dependent on analog
		// properties of the CPU and its operating environment. They cannot be
		// modelled 100% accurately, but you can pick magic constants that will
		// work for the majority of cases.

		static constexpr inline uint8_t LAX_MAGIC = 0xEE;

		// LAX #imm: LDA + LDX + TAX			(aka ATX, LXA, OAL, ANX)
		//   A,X = (A | {CONST}) & #{imm}
		// LAX #imm is in this group because it wires up the Accumulator as input
		// and output to the Special Bus at the same time, while the other versions
		// of LAX do not.
		uint8_t doLAX_IMM(uint8_t data)
		{
			X = A = (A | LAX_MAGIC) & data;
			P.setNZ(A);
			return 0;
		}

		static constexpr inline uint8_t XAA_MAGIC = 0xEF;		// for RDY high
		static constexpr inline uint8_t XAA_MAGIC_RDY = 0xEE;	// for RDY low

		// XAA									(axa ANE, AXM)
		//   A = (A | {CONST}) & X & #imm
		uint8_t doXAA(uint8_t data)
		{
			A = (A | (CheckRDY() ? XAA_MAGIC : XAA_MAGIC_RDY)) & X & data;
			P.setNZ(A);
			return 0;
		}

		/***************************** Opcode tables ******************************/

		// ModeExec is a jump table to the first state of each addressing mode.
		static inline const ExecPtr ModeExec[] = {
			&type::execImplicit_T02,	// implicit
			&type::execAccumulator_T02,	// accumulator

			&type::execImmediate_T02,	// #$00
			&type::execZP_T2,			// $00
			&type::execZPX_T2,			// $00,X
			&type::execZPY_T2,			// $00,Y
			&type::execIZPX_T2,			// ($00,X)
			&type::execIZPY_T2,			// ($00),Y
			&type::execAbs_T2,			// $0000
			&type::execAbsX_T2,			// $0000,X
			&type::execAbsY_T2,			// $0000,Y

			// Internal execution on memory data
			&type::execZP_Sto_T2,		// STx $00
			&type::execAbs_Sto_T2,		// STx $0000
			&type::execIZPX_Sto_T2,		// STx ($00,X)
			&type::execAbsX_Sto_T2,		// STx $0000,X
			&type::execAbsY_Sto_T2,		// STx $0000,Y
			&type::execZPX_Sto_T2,		// STx $00,X
			&type::execZPY_Sto_T2,		// STx $00,Y
			&type::execIZPY_Sto_T2,		// STx ($00),Y

			// Read-Modify-Write operations
			&type::execZP_RMW_T2,		// RMW $00
			&type::execAbs_RMW_T2,		// RMW $0000
			&type::execZPX_RMW_T2,		// RMW $00,X
			&type::execAbsX_RMW_T2,		// RMW $0000,X
			&type::execAbsY_RMW_T2,		// RMW $0000,Y [illegal]
			&type::execIZPX_RMW_T2,		// RMW ($00,X) [illegal]
			&type::execIZPY_RMW_T2,		// RMW ($00),Y [illegal]

			// Miscellaneous Operations
			&type::execPush_T2,			// Push to stack
			&type::execPull_T2,			// Pull from stack
			&type::execJump_T2,			// JMP $0000
			&type::execJumpInd_T2,		// JMP ($0000)
			&type::execCall_T2,			// JSR $0000
			&type::execRTS_T2,			// Return from Subroutine
			&type::execBranch_T2,		// PC-relative
			&type::execBreak_T2,		// BRK/IRQ/NMI/RESET sequence
			&type::execRTI_T2,			// Return from Interrupt
			&type::execHalt_T2,			// Halt the processor
		};

		// DoOp is a jump table to the functions that execute each operation.
		static inline const DoOpPtr DoOp[] = {
			&type::doADC,
			&type::doAND,
			&type::doASL,
			&type::getBCC,
			&type::getBCS,
			&type::getBEQ,
			&type::doBIT,
			&type::getBMI,
			&type::getBNE,
			&type::getBPL,
			&type::doNOP,	// BRK
			&type::getBVC,
			&type::getBVS,
			&type::doCLC,
			&type::doCLD,
			&type::doCLI,
			&type::doCLV,
			&type::doCMP,
			&type::doCPX,
			&type::doCPY,
			&type::doDEC,
			&type::doDEX,
			&type::doDEY,
			&type::doEOR,
			&type::doINC,
			&type::doINX,
			&type::doINY,
			&type::doNOP,	// JMP
			&type::doNOP,	// JSR
			&type::doLDA,
			&type::doLDX,
			&type::doLDY,
			&type::doLSR,
			&type::doNOP,
			&type::doORA,
			&type::doGetA,	// PHA
			&type::doPHP,
			&type::doPLA,
			&type::doPLP,
			&type::doROL,
			&type::doROR,
			&type::doNOP,	// RTI
			&type::doNOP,	// RTS
			&type::doSBC,
			&type::doSEC,
			&type::doSED,
			&type::doSEI,
			&type::doGetA,	// STA
			&type::doGetX,	// STX
			&type::doGetY,	// STY
			&type::doTAX,
			&type::doTAY,
			&type::doTSX,
			&type::doTXA,
			&type::doTXS,
			&type::doTYA,

			// "Illegal" operations
			&type::doAHX,
			&type::doALR,
			&type::doANC,
			&type::doARR,
			&type::doAXS,
			&type::doDCP,
			&type::doISC,
			&type::doNOP,	// KIL/HLT/CIM/CRP
			&type::doLAS,
			&type::doLAX,
			&type::doLAX_IMM,
			&type::doRLA,
			&type::doRRA,
			&type::doSAX,
			&type::doSHX,
			&type::doSHY,
			&type::doSLO,
			&type::doSRE,
			&type::doTAS,
			&type::doXAA,
		};
	};
}
