#include <concepts>
#include <stdint.h>
#include <cassert>
#include <string>
#include <functional>

#include "yam6502ops.h"

namespace m65xx {
	enum : uint8_t {
		FLAG_C = 0x01,
		FLAG_Z = 0x02,
		FLAG_I = 0x04,
		FLAG_D = 0x08,
		FLAG_B = 0x10,
		FLAG_RESERVED = 0x20,
		FLAG_V = 0x40,
		FLAG_N = 0x80
	};

	template<typename T>
	concept HasGetRDY = requires (T bus) {
		{ bus->getRDY() } -> std::convertible_to<bool>;
	};
	template<typename T>
	concept HasGetIRQB = requires (T bus) {
		{ bus->getIRQB() } -> std::convertible_to<bool>;
	};
	template<typename T>
	concept HasGetNMIB = requires (T bus) {
		{ bus->getNMIB() } -> std::convertible_to<bool>;
	};
	template<typename T>
	concept HasGetRESB = requires (T bus) {
		{ bus->getRESB() } -> std::convertible_to<bool>;
	};

	/*
	template<typename T>
	concept is_system_bus =
	requires(T bus, unsigned addr, uint8_t data) {
		{ bus->ReadAddr(addr) } -> std::same_as<uint8_t>;
		bus->WriteAddr(addr, data);
	};
	*/

	// T = A pointer type to the bus used to read and write.
	// trapcode = A trap opcode, if nonnegative.
	template<typename T, int trap_code=-1>
	class M6502 {
	public:
		using type = M6502<T, trap_code>;

		M6502(T bus) : Bus(bus) {}
		M6502() = delete;
		M6502(const M6502 &) = default;
		M6502(M6502 &&) = default;
		M6502 &operator=(const M6502 &) = default;
		M6502 &operator=(M6502 &&) = default;

		// Getters
		[[nodiscard]] constexpr uint8_t getA() const { return A; }
		[[nodiscard]] constexpr uint8_t getX() const { return X; }
		[[nodiscard]] constexpr uint8_t getY() const { return Y; }
		[[nodiscard]] constexpr uint16_t getSP() const { return 0x100 | SP; }
		[[nodiscard]] constexpr uint16_t getPC() const { return PC; }
		[[nodiscard]] constexpr uint8_t getP() const { return static_cast<uint8_t>(P); }
		[[nodiscard]] constexpr uint8_t getIR() const { return IR; }
		[[nodiscard]] constexpr bool getSync() const { return Sync; }

		// Setters
		void setA(uint8_t val) { A = val; }
		void setX(uint8_t val) { X = val; }
		void setY(uint8_t val) { Y = val; }
		void setSP(uint16_t val) { SP = val & 0xFF; }
		void setPC(uint16_t val) { PC = val; State = &type::execFetch_T1; }
		void setP(uint8_t val) { P = val; }

		// Do something
		void reset()
		{
			IR = static_cast<uint8_t>(op::BRK);
			AddrMode = am::brk;
			BaseOp = op::RESET;
			State = &M6502<T>::execBreak_T2;
			NextIrqPending = IrqPending = false;
			NextNmiPending = NmiPending = false;
			LastNMIB = checkNMIB();
			InterruptGen = false;
		}

		void tick(int cycles = 1)
		{
			ExecPtr next = State;
			while (cycles-- > 0) {
				assert(next != nullptr);
				Sync = false;
				checkInterruptPins();
				next = std::invoke(next, this);
			}
			State = next;
		}

		// Disassemble the opcode at addr
		std::string disasmOp(uint16_t addr, bool full_line)
		{
			// I wanted to play with std::format, but it doesn't exist
			// in the standard library yet, so back to the C library I go.
			char buffer[48];
			uint8_t opbyte[4] = { Bus->readAddr(addr) };
			const auto &base = Opc6502[opbyte[0]];
			const auto &info = ModeTable[static_cast<int>(base.Mode)];
			unsigned operand = 0;

			const auto buffsize = std::size(buffer);
			int buffpos = 0;

			// Get operand bytes
			if (info.OperandBytes >= 1) {
				operand = opbyte[1] = Bus->readAddr(addr + 1);
				if (info.OperandBytes > 1) {
					opbyte[2] = Bus->readAddr(addr + 2);
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
			const char *opname = OpNames[base.Op];
			buffpos += snprintf(buffer + buffpos, buffsize - buffpos, "%s", OpNames[base.Op]);

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
			return std::string(buffer, buffpos);
		}

		[[nodiscard]] constexpr uint16_t opToVector(op operation) const
		{
			switch (operation) {
			case op::RESET: return 0xFFFC;
			case op::NMI: return 0xFFFA;
			default: return 0xFFFE;
			}
		}

		enum : uint16_t { STACK_PAGE = 0x100 };

	private:
		class StatusFlags {
		public:
			constexpr void setN(uint8_t val) { N = val; }
			constexpr void setV(uint8_t val) { V = val; }
			constexpr void setD(bool flag) { DI = (DI & ~FLAG_D) | (flag ? FLAG_D : 0); }
			constexpr void setI(bool flag) { DI = (DI & ~FLAG_I) | (flag ? FLAG_I : 0); }
			constexpr void setZ(bool flag) { Z = flag; }
			constexpr void setC(bool flag) { C = flag; }


			constexpr void setNZ(uint8_t from) { setN(from); setZ(!from); }

			[[nodiscard]] constexpr uint8_t getN() const { return N & FLAG_N; }
			[[nodiscard]] constexpr uint8_t getV() const { return V & FLAG_V; }
			[[nodiscard]] constexpr uint8_t getD() const { return DI & FLAG_D; }
			[[nodiscard]] constexpr uint8_t getI() const { return DI & FLAG_I; }
			[[nodiscard]] constexpr bool getZ() const { return Z; }
			[[nodiscard]] constexpr bool getC() const { return C; }

			[[nodiscard]] constexpr explicit operator uint8_t() const {
				return getN() | getV() | FLAG_RESERVED | FLAG_B | DI | (Z << 1) | (uint8_t)C;
			}
			constexpr StatusFlags &operator=(uint8_t word) {
				N = word;
				V = word;
				DI = word & (FLAG_D | FLAG_I);
				Z = (word & FLAG_Z) >> 1;
				C = word & FLAG_C;
				return *this;
			}

		private:
			uint8_t N = 0;
			uint8_t V = 0;
			bool Z = false;
			bool C = false;
			uint8_t DI = 0;
		};

		T Bus;

		uint8_t A = 0;
		uint8_t X = 0;
		uint8_t Y = 0;
		uint8_t SP = 0;
		uint16_t PC = 0;
		uint16_t TempAddr = 0;
		uint8_t TempData = 0;
		uint8_t IR = 0;
		StatusFlags P;
		bool Sync = false;
		bool IrqPending = false;
		bool NextIrqPending = false;
		bool NmiPending = false;
		bool NextNmiPending = false;
		bool LastNMIB = true;
		bool InterruptGen = false;

		// Helper functions to access optional pins on the bus
		[[nodiscard]] constexpr bool checkRDY() const
		{
			if constexpr (HasGetRDY<T>) {
				return Bus->getRDY();
			}
			else {
				return true;
			}
		}
		[[nodiscard]] constexpr bool checkIRQB() const
		{
			if constexpr (HasGetIRQB<T>) {
				return Bus->getIRQB();
			}
			else {
				return true;
			}
		}
		[[nodiscard]] constexpr bool checkNMIB() const
		{
			if constexpr (HasGetNMIB<T>) {
				return Bus->getNMIB();
			}
			else {
				return true;
			}
		}
		[[nodiscard]] constexpr bool checkRESB() const
		{
			if constexpr (HasGetRESB<T>) {
				return Bus->getRESB();
			}
			else {
				return true;
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
		using ExecOpPtr = uint8_t (M6502::*)(uint8_t);

		ExecPtr State = &type::execBreak_T2;
		am AddrMode = am::brk;
		op BaseOp = op::RESET;

		[[nodiscard]] ExecPtrRet nextInstr()
		{
			Sync = true;
			auto op = Bus->readAddr(PC);
			if (InterruptGen) {
				op = 0;
				BaseOp = NmiPending ? op::NMI : op::IRQ;
			}
			else {
				++PC;
			}
			IR = op;
			const BaseOpcode &base = Opc6502[IR];
			AddrMode = base.Mode;
			if (!InterruptGen) {
				if constexpr (trap_code >= 0) {
					if (op == trap_code) {
						// The trap handler is called with a reference to this
						// CPU and the address of the trap opcode. PC points to
						// the byte after the opcode. The handler should return
						// true if it handled the opcode or false to let default
						// handling continue.
						if (Bus->trap(*this, PC - 1)) {
							return &type::execFetch_T1;
						}
					}
				}
				BaseOp = base.Op;
			}
			return ModeTable[static_cast<int>(base.Mode)].Exec;
		}

		// Check the interrupt pins and update state accordingly. For reference:
		// http://visual6502.org/wiki/index.php?title=6502_Interrupt_Recognition_Stages_and_Tolerances
		constexpr void checkInterruptPins()
		{
			// When IRQ changes, that change will be reflected in IRQP in the next
			// cycle. If IRQP is high during T0, then INTG will go high, causing the
			// next cycle (T1) to substitute BRK for the instruction, but only if
			// the interrupt disable bit is clear.
			IrqPending = NextIrqPending;
			NextIrqPending = !checkIRQB();

			// NMI triggers when the NMIB line goes from high to low. Unlike IRQB,
			// it does not need to still be low when T0 is reached to be serviced.
			// However, if NMIB is low only during the two cycles when BRK fetches
			// the vector address, then the NMI will be "lost" because the
			// processor explicitly blocks reading of the NMIB line during these
			// cycles to avoid a mixed vector read for both IRQ and NMI (not
			// emulated).
			bool nmib = checkNMIB();
			if (!NextNmiPending && LastNMIB && !nmib) {
				NextNmiPending = true;
			}
			else if (NextNmiPending) {
				NmiPending = true;
				NextNmiPending = false;
			}
			LastNMIB = nmib;
		}

		// For all T0 and also T2 of branches, check if the next instruction fetch
		// should substitute a BRK.
		constexpr void intCheckT0()
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
			Bus->readAddr(PC);
			intCheckT0();
			return &type::execCommon_T1;
		}

		// Accumulator ==================================================== 2 cycles

		ExecPtrRet execAccumulator_T02() // Dummy read while the ALU does its thing
		{
			Bus->readAddr(PC);
			intCheckT0();
			return &type::execAccumulator_T1;
		}
		ExecPtrRet execAccumulator_T1()	// Perform operation + fetch next instruction
		{
			A = std::invoke(ExecOp[static_cast<int>(BaseOp)], this, A);
			return nextInstr();
		}


		/* Internal Execution on Memory Data **************************************/


		// Common T0 and T1 ========================================================

		ExecPtrRet execCommon_T0()		// Read operand
		{
			TempData = Bus->readAddr(TempAddr);
			intCheckT0();
			return &type::execCommon_T1;
		}
		ExecPtrRet execCommon_T1()		// Perform operation + fetch next instruction
		{
			std::invoke(ExecOp[static_cast<int>(BaseOp)], this, TempData);
			return nextInstr();
		}

		// Immediate   #$00 =============================================== 2 cycles

		ExecPtrRet execImmediate_T02()	// Read operand
		{
			TempData = Bus->readAddr(PC++);
			intCheckT0();
			return &type::execCommon_T1;
		}

		// Zero page   $00 ================================================ 3 cycles

		ExecPtrRet execZP_T2()			// Fetch effective address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execCommon_T0;
		}

		// Absolute   $0000 =============================================== 4 cycles

		ExecPtrRet execAbs_T2()			// Fetch low byte of effective address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execAbs_T3;
		}
		ExecPtrRet execAbs_T3()			// Fetch high byte of effective address
		{
			TempAddr |= Bus->readAddr(PC++) << 8;
			return &type::execCommon_T0;
		}

		// Indirect, X   ($00,X) ========================================== 6 cycles

		ExecPtrRet execIZPX_T2()		// Fetch ZP base address
		{
			TempData = Bus->readAddr(PC++);
			return &type::execIZPX_T3;
		}
		ExecPtrRet execIZPX_T3()		// Dummy fetch while add happens
		{
			Bus->readAddr(TempData);
			TempData += X;
			return &type::execIZPX_T4;
		}
		ExecPtrRet execIZPX_T4()		// Fetch low byte of effective address
		{
			TempAddr = Bus->readAddr(TempData);
			return &type::execIZPX_T5;
		}
		ExecPtrRet execIZPX_T5()		// Fetch high byte of effective address
		{
			TempAddr |= Bus->readAddr((TempData + 1) & 0xFF) << 8;
			return &type::execCommon_T0;
		}

		// Absolute, X   $0000,X ===================================== 4 or 5 cycles

		ExecPtrRet execAbsX_T2()		// Fetch low byte of base address
		{
			TempAddr = Bus->readAddr(PC++);
			TempData = X;
			return &type::execAbsXY_T3;
		}
		ExecPtrRet execAbsXY_T3()		// Fetch high byte of base address
		{
			TempAddr |= Bus->readAddr(PC++) << 8;
			uint8_t pre_hi = TempAddr >> 8;
			TempAddr += TempData;		// T2 stored X or Y in TempData for us
			TempData = pre_hi;
			return (TempAddr >> 8) != pre_hi ? &type::execAbsXY_T4 : &type::execCommon_T0;
		}
		ExecPtrRet execAbsXY_T4()		// Dummy read while performing carry
		{
			Bus->readAddr((TempData << 8) | (TempAddr & 0x00FF));
			return &type::execCommon_T0;
		}

		// Absolute, Y   $0000,Y ===================================== 4 or 5 cycles

		ExecPtrRet execAbsY_T2()		// Fetch low byte of base address
		{
			TempAddr = Bus->readAddr(PC++);
			TempData = Y;
			return &type::execAbsXY_T3;
		}

		// Zero Page, X   $00, X ========================================== 4 cycles

		ExecPtrRet execZPX_T2()			// Fetch base address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execZPX_T3;
		}
		ExecPtrRet execZPX_T3()			// Dummy read while performing add
		{
			Bus->readAddr(TempAddr);
			TempAddr = (TempAddr + X) & 0x00FF;
			return &type::execCommon_T0;
		}

		// Zero Page, Y   $00, Y ========================================== 4 cycles

		ExecPtrRet execZPY_T2()			// Fetch base address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execZPY_T3;
		}
		ExecPtrRet execZPY_T3()			// Dummy read while performing add
		{
			Bus->readAddr(TempAddr);
			TempAddr = (TempAddr + Y) & 0x00FF;
			return &type::execCommon_T0;
		}

		// Indirect, Y   ($00),Y ===================================== 5 or 6 cycles

		ExecPtrRet execIZPY_T2()		// Fetch indirect address
		{
			TempData = Bus->readAddr(PC++);
			return &type::execIZPY_T3;
		}
		ExecPtrRet execIZPY_T3()		// Fetch low byte of base address
		{
			TempAddr = Bus->readAddr(TempData);
			return &type::execIZPY_T4;
		}
		ExecPtrRet execIZPY_T4()		// Fetch high byte of base address
		{
			TempAddr |= Bus->readAddr((TempData + 1) & 0x00FF) << 8;
			TempData = TempAddr >> 8;
			TempAddr += Y;
			return (TempAddr >> 8) != TempData ? &type::execAbsXY_T4 : &type::execCommon_T0;
		}


		/* Store Operations *******************************************************/


		// Common T0 and T1 for Store/RMW ==========================================

		ExecPtrRet execCommon_Sto_T0()	// Write register/modified data to memory
		{
			// AHX/SHX/SHY/TAS can potentially change the target address, so
			// the operator must be invoked separately from the writeAddr call.
			uint8_t output = std::invoke(ExecOp[static_cast<int>(BaseOp)], this, TempData);
			Bus->writeAddr(TempAddr, output);
			intCheckT0();
			return &type::execFetch_T1;
		}
		ExecPtrRet execFetch_T1()	// Fetch next instruction
		{
			return nextInstr();
		}

		// Zero page   STx $00 ============================================ 3 cycles

		ExecPtrRet execZP_Sto_T2()		// Fetch effective address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execCommon_Sto_T0;
		}

		// Absolute   STx $0000 =========================================== 4 cycles

		ExecPtrRet execAbs_Sto_T2()		// Fetch low byte of effective address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execAbs_Sto_T3;
		}
		ExecPtrRet execAbs_Sto_T3()		// Fetch high byte of effective address
		{
			TempAddr |= Bus->readAddr(PC++) << 8;
			return &type::execCommon_Sto_T0;
		}

		// Indirect, X   STx ($00,X) ====================================== 6 cycles

		ExecPtrRet execIZPX_Sto_T2()	// Fetch ZP base address
		{
			TempData = Bus->readAddr(PC++);
			return &type::execIZPX_Sto_T3;
		}
		ExecPtrRet execIZPX_Sto_T3()	// Dummy fetch while add happens
		{
			Bus->readAddr(TempData);
			TempData += X;
			return &type::execIZPX_Sto_T4;
		}
		ExecPtrRet execIZPX_Sto_T4()	// Fetch low byte of effective address
		{
			TempAddr = Bus->readAddr(TempData);
			return &type::execIZPX_Sto_T5;
		}
		ExecPtrRet execIZPX_Sto_T5()	// Fetch high byte of effective address
		{
			TempAddr |= Bus->readAddr((TempData + 1) & 0xFF) << 8;
			return &type::execCommon_Sto_T0;
		}

		// Absolute, X   Stx $0000,X ====================================== 5 cycles

		ExecPtrRet execAbsX_Sto_T2()	// Fetch low byte of base address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execAbsX_Sto_T3;
		}
		ExecPtrRet execAbsX_Sto_T3()	// Fetch high byte of base address
		{
			TempAddr |= Bus->readAddr(PC++) << 8;
			return &type::execAbsX_Sto_T4;
		}
		ExecPtrRet execAbsX_Sto_T4()	// Dummy read, maybe performing carry
		{
			uint16_t pre_hi = TempAddr & 0xFF00;
			TempAddr += X;
			Bus->readAddr(pre_hi | (TempAddr & 0x00FF));
			TempData = uint8_t(pre_hi >> 8);	// Record for AHX/SHX/SHY/TAS
			return &type::execCommon_Sto_T0;
		}

		// Absolute, Y   STx $0000,Y ====================================== 5 cycles

		ExecPtrRet execAbsY_Sto_T2()	// Fetch low byte of base address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execAbsY_Sto_T3;
		}
		ExecPtrRet execAbsY_Sto_T3()	// Fetch high byte of base address
		{
			TempAddr |= Bus->readAddr(PC++) << 8;
			return &type::execAbsY_Sto_T4;
		}
		ExecPtrRet execAbsY_Sto_T4()	// Dummy read, maybe performing carry
		{
			uint16_t pre_hi = TempAddr & 0xFF00;
			TempAddr += Y;
			Bus->readAddr(pre_hi | (TempAddr & 0x00FF));
			TempData = uint8_t(pre_hi >> 8);	// Record for AHX/SHX/SHY/TAS
			return &type::execCommon_Sto_T0;
		}

		// Zero Page, X   Stx $00, X ====================================== 4 cycles

		ExecPtrRet execZPX_Sto_T2()		// Fetch base address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execZPX_Sto_T3;
		}
		ExecPtrRet execZPX_Sto_T3()		// Dummy read while performing add
		{
			Bus->readAddr(TempAddr);
			TempAddr = (TempAddr + X) & 0x00FF;
			return &type::execCommon_Sto_T0;
		}

		// Zero Page, Y   Stx $00, Y ====================================== 4 cycles

		ExecPtrRet execZPY_Sto_T2()		// Fetch base address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execZPY_Sto_T3;
		}
		ExecPtrRet execZPY_Sto_T3()		// Dummy read while performing add
		{
			Bus->readAddr(TempAddr);
			TempAddr = (TempAddr + Y) & 0x00FF;
			return &type::execCommon_Sto_T0;
		}

		// Indirect, Y   Stx ($00),Y ====================================== 6 cycles

		ExecPtrRet execIZPY_Sto_T2()	// Fetch indirect address
		{
			TempData = Bus->readAddr(PC++);
			return &type::execIZPY_Sto_T3;
		}
		ExecPtrRet execIZPY_Sto_T3()	// Fetch low byte of base address
		{
			TempAddr = Bus->readAddr(TempData);
			return &type::execIZPY_Sto_T4;
		}
		ExecPtrRet execIZPY_Sto_T4()	// Fetch high byte of base address
		{
			TempAddr |= Bus->readAddr((TempData + 1) & 0xFF) << 8;
			return &type::execAbsY_Sto_T4;
		}


		/* Read-Modify-Write Operations *******************************************/


		// Common cycles for RMW ===================================================

		ExecPtrRet execCommon_RMW_SD1()	// Fetch data
		{
			TempData = Bus->readAddr(TempAddr);
			return &type::execCommon_RMW_SD2;
		}
		ExecPtrRet execCommon_RMW_SD2()	// Dummy write
		{
			Bus->writeAddr(TempAddr, TempData);
			return &type::execCommon_Sto_T0;
		}

		// Zero page   RMW $00 ============================================ 5 cycles

		ExecPtrRet execZP_RMW_T2()		// Fetch effective address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execCommon_RMW_SD1;
		}

		// Absolute   RMW $0000 =========================================== 6 cycles

		ExecPtrRet execAbs_RMW_T2()		// Fetch low byte of effective address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execAbs_RMW_T3;
		}
		ExecPtrRet execAbs_RMW_T3()		// Fetch high byte of effective address
		{
			TempAddr |= Bus->readAddr(PC++) << 8;
			return &type::execCommon_RMW_SD1;
		}

		// Zero Page, X   RMW $00, X ====================================== 6 cycles

		ExecPtrRet execZPX_RMW_T2()		// Fetch base address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execZPX_RMW_T3;
		}
		ExecPtrRet execZPX_RMW_T3()		// Dummy read while performing add
		{
			Bus->readAddr(TempAddr);
			TempAddr = (TempAddr + X) & 0xFF;
			return &type::execCommon_RMW_SD1;
		}

		// Absolute, X   RMW $0000,X ====================================== 7 cycles

		ExecPtrRet execAbsX_RMW_T2()	// Fetch low byte of base address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execAbsX_RMW_T3;
		}
		ExecPtrRet execAbsX_RMW_T3()	// Fetch high byte of base address
		{
			TempAddr |= Bus->readAddr(PC++) << 8;
			return &type::execAbsX_RMW_T4;
		}
		ExecPtrRet execAbsX_RMW_T4()	// Dummy read, maybe performing carry
		{
			uint16_t pre_hi = TempAddr & 0xFF00;
			TempAddr += X;
			Bus->readAddr(pre_hi | (TempAddr & 0x00FF));
			return &type::execCommon_RMW_SD1;
		}

		// Absolute, Y   RMW $0000,Y ==== Illegal ========================= 7 cycles

		ExecPtrRet execAbsY_RMW_T2()	// Fetch low byte of base address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execAbsY_RMW_T3;
		}
		ExecPtrRet execAbsY_RMW_T3()	// Fetch high byte of base address
		{
			TempAddr |= Bus->readAddr(PC++) << 8;
			return &type::execAbsY_RMW_T4;
		}
		ExecPtrRet execAbsY_RMW_T4()	// Dummy read, maybe performing carry
		{
			uint16_t pre_hi = TempAddr & 0xFF00;
			TempAddr += Y;
			Bus->readAddr(pre_hi | (TempAddr & 0x00FF));
			return &type::execCommon_RMW_SD1;
		}

		// Indirect, X   RMW ($00,X) ==== Illegal ========================= 8 cycles

		ExecPtrRet execIZPX_RMW_T2()	// Fetch ZP base address
		{
			TempData = Bus->readAddr(PC++);
			return &type::execIZPX_RMW_T3;
		}
		ExecPtrRet execIZPX_RMW_T3()	// Dummy fetch while add happens
		{
			Bus->readAddr(TempData);
			TempData += X;
			return &type::execIZPX_RMW_T4;
		}
		ExecPtrRet execIZPX_RMW_T4()	// Fetch low byte of effective address
		{
			TempAddr = Bus->readAddr(TempData);
			return &type::execIZPX_RMW_T5;
		}
		ExecPtrRet execIZPX_RMW_T5()	// Fetch high byte of effective address
		{
			TempAddr |= Bus->readAddr((TempData + 1) & 0xFF) << 8;
			return &type::execCommon_RMW_SD1;
		}

		// Indirect, Y   RMW ($00),Y ==== Illegal ========================= 8 cycles

		ExecPtrRet execIZPY_RMW_T2()	// Fetch indirect address
		{
			TempData = Bus->readAddr(PC++);
			return &type::execIZPY_RMW_T3;
		}
		ExecPtrRet execIZPY_RMW_T3()	// Fetch low byte of base address
		{
			TempAddr = Bus->readAddr(TempData);
			return &type::execIZPY_RMW_T4;
		}
		ExecPtrRet execIZPY_RMW_T4()	// Fetch high byte of base address
		{
			TempAddr |= Bus->readAddr((TempData + 1) & 0xFF) << 8;
			return &type::execAbsY_RMW_T4;
		}


		/* Miscellaneous Operations ***********************************************/


		// Push Operation ================================================= 3 cycles

		ExecPtrRet execPush_T2()		// Dummy read
		{
			Bus->readAddr(PC);
			return &type::execPush_T0;
		}
		ExecPtrRet execPush_T0()		// Write register to stack
		{
			Bus->writeAddr(STACK_PAGE | SP, std::invoke(ExecOp[static_cast<int>(BaseOp)], this, 0));
			--SP;
			intCheckT0();
			return &type::execFetch_T1;
		}

		// Pull Operation ================================================= 4 cycles

		ExecPtrRet execPull_T2()		// Dummy read from PC+1
		{
			Bus->readAddr(PC);
			return &type::execPull_T3;
		}
		ExecPtrRet execPull_T3()		// Dummy read from SP
		{
			Bus->readAddr(STACK_PAGE | SP);
			++SP;
			return &type::execPull_T0;
		}
		ExecPtrRet execPull_T0()		// Fetch data from stack
		{
			TempData = Bus->readAddr(STACK_PAGE | SP);
			intCheckT0();
			return &type::execCommon_T1;
		}

		// Jump Absolute   JMP $0000 ====================================== 3 cycles

		ExecPtrRet execJump_T2()		// Fetch low byte of target address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execJump_T0;
		}
		ExecPtrRet execJump_T0()		// Fetch high byte of target address
		{
			TempAddr |= Bus->readAddr(PC) << 8;
			intCheckT0();
			return &type::execJump_T1;
		}
		ExecPtrRet execJump_T1()		// Fetch next instruction
		{
			PC = TempAddr;
			return nextInstr();
		}

		// Jump Indirect   JMP ($0000) ==================================== 5 cycles

		ExecPtrRet execJumpInd_T2()		// Fetch low byte of indirect address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execJumpInd_T3;
		}
		ExecPtrRet execJumpInd_T3()		// Fetch high byte of indirect address
		{
			TempAddr |= Bus->readAddr(PC++) << 8;
			return &type::execJumpInd_T4;
		}
		ExecPtrRet execJumpInd_T4()		// Fetch low byte of target address
		{
			TempData = Bus->readAddr(TempAddr);
			return &type::execJumpInd_T0;
		}
		ExecPtrRet execJumpInd_T0()		// Fetch high byte of target address
		{
			// On the NMOS 6502, a page boundary is not crossed here
			uint16_t addr = (TempAddr & 0xFF00) | ((TempAddr + 1) & 0x00FF);
			PC = TempData | (Bus->readAddr(addr) << 8);
			intCheckT0();
			return &type::execJumpInd_T1;
		}
		ExecPtrRet execJumpInd_T1()		// Fetch next instruction
		{
			return nextInstr();
		}

		// Jump to Subroution   JSR $0000 ================================= 6 cycles

		ExecPtrRet execCall_T2()		// Fetch low byte of target address
		{
			TempAddr = Bus->readAddr(PC++);
			return &type::execCall_T3;
		}
		ExecPtrRet execCall_T3()		// Dummy read from stack
		{
			Bus->readAddr(STACK_PAGE | SP);
			return &type::execCall_T4;
		}
		ExecPtrRet execCall_T4()		// Push high byte of PC
		{
			Bus->writeAddr(STACK_PAGE | SP--, PC >> 8);
			return &type::execCall_T5;
		}
		ExecPtrRet execCall_T5()		// Push low byte of PC
		{
			Bus->writeAddr(STACK_PAGE | SP--, PC & 0xFF);
			return &type::execCall_T0;
		}
		ExecPtrRet execCall_T0()		// Fetch high byte of target address
		{
			TempAddr |= Bus->readAddr(PC) << 8;
			intCheckT0();
			return &type::execJump_T1;
		}

		// Return from Subroutine   RTS =================================== 6 cycles

		ExecPtrRet execRTS_T2()			// Dummy read of byte after opcode
		{
			Bus->readAddr(PC);
			return &type::execRTS_T3;
		}
		ExecPtrRet execRTS_T3()			// Dummy read from stack
		{
			Bus->readAddr(STACK_PAGE | SP);
			return &type::execRTS_T4;
		}
		ExecPtrRet execRTS_T4()			// Pull PCL from stack
		{
			TempAddr = Bus->readAddr(STACK_PAGE | ++SP);
			return &type::execRTS_T5;
		}
		ExecPtrRet execRTS_T5()			// Pull PCH from stack
		{
			TempAddr |= Bus->readAddr(STACK_PAGE | ++SP) << 8;
			return &type::execRTS_T0;
		}
		ExecPtrRet execRTS_T0()			// Dummy read to increment PC
		{
			PC = TempAddr;
			Bus->readAddr(PC++);
			intCheckT0();
			return &type::execFetch_T1;
		}

		// BRK/IRQ/NMI/RESET ============================================== 7 cycles
		// The BRK instruction sequence is reused for BRK/IRQ/NMI/RESET. When used
		// for RESET, the writes to the stack are turned into reads.

		ExecPtrRet execBreak_T2()		// Dummy read of PC
		{
			Bus->readAddr(PC);
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
				Bus->writeAddr(addr, PC >> 8);
			} else {
				Bus->readAddr(addr);
			}
			return &type::execBreak_T4;
		}
		ExecPtrRet execBreak_T4()		// Push low order byte of PC to stack
		{
			const uint16_t addr = STACK_PAGE | SP--;
			if (BaseOp != op::RESET) {
				Bus->writeAddr(addr, PC & 0xFF);
			} else {
				Bus->readAddr(addr);
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
				Bus->writeAddr(addr, status);
			} else {
				Bus->readAddr(addr);
			}
			return &type::execBreak_T6;
		}
		ExecPtrRet execBreak_T6()		// Fetch low order byte of interrupt vector
		{
			InterruptGen = false;
			P.setI(true);
			TempAddr = Bus->readAddr(opToVector(BaseOp));
			return &type::execBreak_T0;
		}
		ExecPtrRet execBreak_T0()		// Fetch high order byte of interrupt vector
		{
			PC = TempAddr | (Bus->readAddr(opToVector(BaseOp) + 1) << 8);
			if (BaseOp == op::NMI) {
				NmiPending = false;
			}
			intCheckT0();
			return &type::execFetch_T1;
		}

		// Return from Interrupt ========================================== 6 cycles

		ExecPtrRet execRTI_T2()			// Dummy fetch of PC+1
		{
			Bus->readAddr(PC);
			return &type::execRTI_T3;
		}
		ExecPtrRet execRTI_T3()			// Dummy fetch from stack
		{
			Bus->readAddr(STACK_PAGE | SP++);
			return &type::execRTI_T4;
		}
		ExecPtrRet execRTI_T4()			// Pull P from stack
		{
			P = Bus->readAddr(STACK_PAGE | SP++);
			return &type::execRTI_T5;
		}
		ExecPtrRet execRTI_T5()			// Pull PCL from stack
		{
			TempAddr = Bus->readAddr(STACK_PAGE | SP++);
			return &type::execRTI_T0;
		}
		ExecPtrRet execRTI_T0()			// Pull PCH from stack
		{
			PC = TempAddr | (Bus->readAddr(STACK_PAGE | SP) << 8);
			intCheckT0();
			return &type::execFetch_T1;
		}

		// Branches ============================================== 2, 3, or 4 cycles

		ExecPtrRet execBranch_T2()	// Read offset
		{
			TempData = Bus->readAddr(PC++);
			intCheckT0();
			return &type::execBranch_T3;
		}
		ExecPtrRet execBranch_T3()	// Check condition
		{
			if (!std::invoke(ExecOp[static_cast<int>(BaseOp)], this, 0)) {
				// No branch, so do next instruction
				return nextInstr();
			} else {
				// Branching, but we still need to read the instruction
				// that would have been executed if we didn't branch.
				Bus->readAddr(PC);
				return &type::execBranch_T4;
			}
		}
		ExecPtrRet execBranch_T4()	// Take branch, if no carry
		{
			uint16_t newPC = PC + (int8_t)TempData;
			if ((newPC ^ PC) & 0xFF00) {
				// Carry needed, so dummy read from incomplete PC sum
				PC = (PC & 0xFF00) | (newPC & 0xFF);
				Bus->readAddr(PC);
				// Complete branch in next cycle
				TempAddr = newPC;
				return &type::execBranch_T0;
			}
			else {
				// No carry needed, so do real instruction fetch now
				PC = newPC;
				return nextInstr();
			}
		}
		ExecPtrRet execBranch_T0()	// Take branch, carry complete
		{
			PC = TempAddr;
			intCheckT0();
			return nextInstr();
		}

		// Halt / Kill ===================================================== Forever

		ExecPtrRet execHalt_T2() // Dummy read of PC+1
		{
			Bus->readAddr(PC++);
			return &type::execHalt_T3;
		}
		ExecPtrRet execHalt_T3() // Put $FFFF on the address bus
		{
			Bus->readAddr(0xFFFF);
			return &type::execHalt_T4;
		}
		ExecPtrRet execHalt_T4() // Put $FFFE on the address bus
		{
			Bus->readAddr(0xFFFE);
			return &type::execHalt_T5;
		}
		ExecPtrRet execHalt_T5() // Put $FFFE on the address bus
		{
			Bus->readAddr(0xFFFE);
			return &type::execHalt_TX;
		}
		ExecPtrRet execHalt_TX() // Put $FFFF on the address bus until reset
		{
			Bus->readAddr(0xFFFF);
			return &type::execHalt_TX;
		}

		/******************************* Operations *******************************/

		uint8_t execBCC(uint8_t)
		{
			return !P.getC();
		}

		uint8_t execBCS(uint8_t)
		{
			return P.getC();
		}

		uint8_t execBNE(uint8_t)
		{
			return !P.getZ();
		}

		uint8_t execBEQ(uint8_t)
		{
			return P.getZ();
		}

		uint8_t execBPL(uint8_t)
		{
			return !P.getN();
		}

		uint8_t execBMI(uint8_t)
		{
			return P.getN();
		}

		uint8_t execBVC(uint8_t)
		{
			return !P.getV();
		}

		uint8_t execBVS(uint8_t)
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
			P.setV(((A ^ added) & (data ^ added)) >> 1);
			P.setC(bool(added >> 8));
			P.setNZ(new_a);
			return new_a;
		}

		// Algorithms for decimal mode addition/subtraction are described at
		// http://www.6502.org/tutorials/decimal_mode.html

		uint8_t execADC(uint8_t data)
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

		uint8_t execSBC(uint8_t data)
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

		uint8_t execAND(uint8_t data)
		{
			A &= data;
			P.setNZ(A);
			return 0;
		}

		uint8_t execASL(uint8_t data)
		{
			P.setC(!!(data & 0x80));
			data <<= 1;
			P.setNZ(data);
			return data;
		}

		uint8_t execBIT(uint8_t data)
		{
			P.setZ(!(A & data));
			P.setN(data);
			P.setV(data);
			return 0;
		}

		uint8_t execCLC(uint8_t)
		{
			P.setC(false);
			return 0;
		}

		uint8_t execCLD(uint8_t)
		{
			P.setD(false);
			return 0;
		}

		uint8_t execCLI(uint8_t)
		{
			P.setI(false);
			return 0;
		}

		uint8_t execCLV(uint8_t)
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

		uint8_t execCMP(uint8_t data)
		{
			doCompare(A, data);
			return 0;
		}

		uint8_t execCPX(uint8_t data)
		{
			doCompare(X, data);
			return 0;
		}

		uint8_t execCPY(uint8_t data)
		{
			doCompare(Y, data);
			return 0;
		}

		uint8_t execDEC(uint8_t data)
		{
			P.setNZ(--data);
			return data;
		}

		uint8_t execDEX(uint8_t)
		{
			P.setNZ(--X);
			return 0;
		}

		uint8_t execDEY(uint8_t)
		{
			P.setNZ(--Y);
			return 0;
		}

		uint8_t execEOR(uint8_t data)
		{
			A ^= data;
			P.setNZ(A);
			return 0;
		}

		uint8_t execINC(uint8_t data)
		{
			data += 1;
			P.setNZ(data);
			return data;
		}

		uint8_t execINX(uint8_t)
		{
			P.setNZ(++X);
			return 0;
		}

		uint8_t execINY(uint8_t)
		{
			P.setNZ(++Y);
			return 0;
		}

		uint8_t execLDA(uint8_t data)
		{
			A = data;
			P.setNZ(A);
			return 0;
		}

		uint8_t execLDX(uint8_t data)
		{
			X = data;
			P.setNZ(X);
			return 0;
		}

		uint8_t execLDY(uint8_t data)
		{
			Y = data;
			P.setNZ(Y);
			return 0;
		}

		uint8_t execLSR(uint8_t data)
		{
			P.setC(bool(data & 1));
			data >>= 1;
			P.setZ(!data);
			P.setN(false);
			return data;
		}

		uint8_t execNOP(uint8_t)
		{
			return 0;
		}

		uint8_t execORA(uint8_t data)
		{
			A |= data;
			P.setNZ(A);
			return 0;
		}

		uint8_t execPHA(uint8_t)
		{
			return A;
		}

		uint8_t execPHP(uint8_t)
		{
			return uint8_t(P);
		}

		uint8_t execPLA(uint8_t data)
		{
			A = data;
			P.setNZ(A);
			return 0;
		}

		uint8_t execPLP(uint8_t data)
		{
			P = data;
			return 0;
		}

		uint8_t execROL(uint8_t data)
		{
			uint8_t shift_in = P.getC();
			P.setC(bool(data >> 7));
			data = (data << 1) | shift_in;
			P.setNZ(data);
			return data;
		}

		uint8_t execROR(uint8_t data)
		{
			uint8_t shift_in = P.getC() << 7;
			P.setC(bool(data & 1));
			data = (data >> 1) | shift_in;
			P.setNZ(data);
			return data;
		}

		uint8_t execSEC(uint8_t)
		{
			P.setC(true);
			return 0;
		}

		uint8_t execSED(uint8_t)
		{
			P.setD(true);
			return 0;
		}

		uint8_t execSEI(uint8_t)
		{
			P.setI(true);
			return 0;
		}

		uint8_t execSTA(uint8_t)
		{
			return A;
		}

		uint8_t execSTX(uint8_t)
		{
			return X;
		}

		uint8_t execSTY(uint8_t)
		{
			return Y;
		}

		uint8_t execTAX(uint8_t)
		{
			X = A;
			P.setNZ(X);
			return 0;
		}

		uint8_t execTAY(uint8_t)
		{
			Y = A;
			P.setNZ(Y);
			return 0;
		}

		uint8_t execTSX(uint8_t)
		{
			X = SP;
			P.setNZ(X);
			return 0;
		}

		uint8_t execTXA(uint8_t)
		{
			A = X;
			P.setNZ(A);
			return 0;
		}

		uint8_t execTXS(uint8_t)
		{
			SP = X;
			// Unlike TSX, does not modify condition codes
			return 0;
		}

		uint8_t execTYA(uint8_t)
		{
			A = Y;
			P.setNZ(A);
			return 0;
		}

		// Illegal operations !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//
		// NMOS 6510 Unintended Opcodes "No More Secrets" v0.95 - 24/12/20
		// used as reference for these.

		// ALR: AND + LSR
		//   A = (A & #{imm}) / 2
		uint8_t execALR(uint8_t data)
		{
			A = execLSR(A & data);
			return 0;
		}

		// ANC: AND + ASL/ROL
		//   A = A & #{imm}
		uint8_t execANC(uint8_t data)
		{
			A &= data;
			P.setNZ(A);
			P.setC(bool(A >> 7));
			return 0;
		}

		// ARR: AND/ADC + ROR
		//   A = (A & #{imm}) / 2
		uint8_t execARR(uint8_t data)
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

		// AXS: CMP + DEX
		//   X = A & X - #{imm}
		uint8_t execAXS(uint8_t data)
		{
			auto diff = doCompare(A & X, data);
			X = uint8_t(diff);
			return 0;
		}

		// DCP: DEC + CMP
		//   {addr} = {addr} - 1   A cmp {addr}
		uint8_t execDCP(uint8_t data)
		{
			--data;
			doCompare(A, data);
			return data;
		}

		// ISC: INC + SBC
		//   {addr} = {addr} + 1   A = A - {addr}
		uint8_t execISC(uint8_t data)
		{
			++data;
			execSBC(data);
			return data;
		}

		uint8_t execKIL(uint8_t)
		{
			return 0;
		}

		// LAS: STA/TXS + LDA/TSX (maybe unstable?)
		//   A,X,SP = {addr} & SP
		uint8_t execLAS(uint8_t data)
		{
			A = X = SP = data &= SP;
			P.setNZ(data);
			return 0;
		}

		// LAX: LDA + LDX
		uint8_t execLAX(uint8_t data)
		{
			A = data;
			X = data;
			P.setNZ(data);
			return 0;
		}

		// RLA: ROL + AND
		//   {addr} = rol {addr}   A = A and {addr}
		uint8_t execRLA(uint8_t data)
		{
			data = execROL(data);
			A &= data;
			P.setNZ(A);
			return data;
		}

		// RRA: ROR + ADC
		//   {addr} = ror {addr}   A = A adc {addr}
		uint8_t execRRA(uint8_t data)
		{
			data = execROR(data);
			execADC(data);
			return data;
		}

		// SAX: STA + STX
		//   {addr} = A & X
		uint8_t execSAX(uint8_t)
		{
			return A & X;
		}

		// SLO: ASL + ORA
		//   {addr} = {addr} * 2   A = A or {addr}
		uint8_t execSLO(uint8_t data)
		{
			data = execASL(data);
			A |= data;
			P.setNZ(A);
			return data;
		}

		// SRE: LSR + EOR
		//   {addr} = {addr} / 2   A = A eor {addr}
		uint8_t execSRE(uint8_t data)
		{
			data = execLSR(data);
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
			if (checkRDY()) {
				value &= prehi + 1;
			}
			return value;
		}

		// AHX: STA/STX/STY
		//   {addr} = A & X & {H+1}
		uint8_t execAHX(uint8_t prehi)
		{
			return doUnstableHi(A & X, prehi);
		}

		// SHX: STA/STX/STY
		//   {addr} = X & {H+1}
		uint8_t execSHX(uint8_t prehi)
		{
			return doUnstableHi(X, prehi);
		}

		// SHY: STA/STX/STY
		//   {addr} = Y & {H+1}
		uint8_t execSHY(uint8_t prehi)
		{
			return doUnstableHi(Y, prehi);
		}

		// TAS: STA/TXS , LDA/TSX
		//   SP = A & X   {addr} = A & X & {H+1}
		uint8_t execTAS(uint8_t prehi)
		{
			SP = A & X;
			return doUnstableHi(SP, prehi);
		}

		// "Magic constant" unstable opcodes ---------------------------------------
		// Operation depends on a "magic" value that is dependent on analog
		// properties of the CPU and its operating environment. They cannot be
		// modelled 100% accurately, but you can pick magic constants that wil
		// work for the majority of cases.

		enum { LAX_MAGIC = 0xEE };

		// LAX #imm: LDA + LDX + TAX
		//   A,X = (A | {CONST}) & #{imm}
		// LAX #imm is in this group because it wires up the Accumulator as input
		// and output to the Special Bus at the same time, while the other versions
		// of LAX do not.
		uint8_t execLAX_IMM(uint8_t data)
		{
			X = A = (A | LAX_MAGIC) & data;
			P.setNZ(A);
			return 0;
		}

		enum { XAA_MAGIC = 0xEF, XAA_MAGIC_RDY = 0xEE };

		// XAA
		//   A = (A | {CONST}) & X & #imm
		uint8_t execXAA(uint8_t data)
		{
			A = (A | (!checkRDY() ? XAA_MAGIC_RDY : XAA_MAGIC)) & X & data;
			P.setNZ(A);
			return 0;
		}

		/***************************** Opcode tables ******************************/

		struct AddrModeInfo {
			ExecPtr Exec;
			uint8_t OperandBytes;
			const char Format[11];
		};

		static inline const AddrModeInfo ModeTable[] = {
			{ &type::execImplicit_T02, 0, "" },	// implicit
			{ &type::execAccumulator_T02, 0, "A" },	// accumulator

			{ &type::execImmediate_T02, 1, "#$%02X" },	// #$00
			{ &type::execZP_T2, 1, "$%02X" },			// $00
			{ &type::execZPX_T2, 1, "$%02X,X" },		// $00,X
			{ &type::execZPY_T2, 1, "$%02X,Y" },		// $00,Y
			{ &type::execIZPX_T2, 1, "($%02X,X)" },		// ($00,X)
			{ &type::execIZPY_T2, 1, "($%02X),Y" },		// ($00),Y
			{ &type::execAbs_T2, 2, "$%04X" },			// $0000
			{ &type::execAbsX_T2, 2, "$%04X,X" },		// $0000,X
			{ &type::execAbsY_T2, 2, "$%04X,Y" },		// $0000,Y

			// Internal execution on memory data
			{ &type::execZP_Sto_T2, 1, "$%02X" },		// STx $00
			{ &type::execAbs_Sto_T2, 2, "$%04X" },		// STx $0000
			{ &type::execIZPX_Sto_T2, 1, "($%02X,X)" },	// STx ($00,X)
			{ &type::execAbsX_Sto_T2, 2, "$%04X,X" },	// STx $0000,X
			{ &type::execAbsY_Sto_T2, 2, "$%04X,Y" },	// STx $0000,Y
			{ &type::execZPX_Sto_T2, 1, "$%02X,X" },	// STx $00,X
			{ &type::execZPY_Sto_T2, 1, "$%02X,Y" },	// STx $00,Y
			{ &type::execIZPY_Sto_T2, 1, "($%02X),Y" },	// STx ($00),Y

			// Read-Modify-Write operations
			{ &type::execZP_RMW_T2, 1, "$%02X" },		// RMW $00
			{ &type::execAbs_RMW_T2, 2, "$%04X" },		// RMW $0000
			{ &type::execZPX_RMW_T2, 1, "$%02X,X" },	// RMW $00,X
			{ &type::execAbsX_RMW_T2, 2, "$%04X,X" },	// RMW $0000,X
			{ &type::execAbsY_RMW_T2, 2, "$%04X,Y" },	// RMW $0000,Y [illegal]
			{ &type::execIZPX_RMW_T2, 1, "($%02X,X)" },	// RMW ($00,X) [illegal]
			{ &type::execIZPY_RMW_T2, 1, "($%02X),Y" },	// RMW ($00),Y [illegal]

			// Miscellaneous Operations
			{ &type::execPush_T2, 0, "" },				// Push to stack
			{ &type::execPull_T2, 0, "" },				// Pull from stack
			{ &type::execJump_T2, 2, "$%04X" },			// JMP $0000
			{ &type::execJumpInd_T2, 2, "($%04X)" },	// JMP ($0000)
			{ &type::execCall_T2, 2, "$%04X" },			// JSR $0000
			{ &type::execRTS_T2, 0, "" },				// Return from Subroutine
			{ &type::execBranch_T2, 1, "$%04X" },		// PC-relative
			{ &type::execBreak_T2, 1, "#$%02X" },		// BRK/IRQ/NMI/RESET sequence
			{ &type::execRTI_T2, 0, "" },				// Return from Interrupt
			{ &type::execHalt_T2, 0, "" },				// Halt the processor
		};
		static inline const ExecOpPtr ExecOp[] = {
			&type::execADC,
			&type::execAND,
			&type::execASL,
			&type::execBCC,
			&type::execBCS,
			&type::execBEQ,
			&type::execBIT,
			&type::execBMI,
			&type::execBNE,
			&type::execBPL,
			&type::execNOP,	// BRK
			&type::execBVC,
			&type::execBVS,
			&type::execCLC,
			&type::execCLD,
			&type::execCLI,
			&type::execCLV,
			&type::execCMP,
			&type::execCPX,
			&type::execCPY,
			&type::execDEC,
			&type::execDEX,
			&type::execDEY,
			&type::execEOR,
			&type::execINC,
			&type::execINX,
			&type::execINY,
			&type::execNOP,	// JMP
			&type::execNOP,	// JSR
			&type::execLDA,
			&type::execLDX,
			&type::execLDY,
			&type::execLSR,
			&type::execNOP,
			&type::execORA,
			&type::execPHA,
			&type::execPHP,
			&type::execPLA,
			&type::execPLP,
			&type::execROL,
			&type::execROR,
			&type::execNOP,	// RTI
			&type::execNOP,	// RTS
			&type::execSBC,
			&type::execSEC,
			&type::execSED,
			&type::execSEI,
			&type::execSTA,
			&type::execSTX,
			&type::execSTY,
			&type::execTAX,
			&type::execTAY,
			&type::execTSX,
			&type::execTXA,
			&type::execTXS,
			&type::execTYA,

			// Illegal operations
			&type::execAHX,
			&type::execALR,
			&type::execANC,
			&type::execARR,
			&type::execAXS,
			&type::execDCP,
			&type::execISC,
			&type::execNOP,	// KIL
			&type::execLAS,
			&type::execLAX,
			&type::execLAX_IMM,
			&type::execRLA,
			&type::execRRA,
			&type::execSAX,
			&type::execSHX,
			&type::execSHY,
			&type::execSLO,
			&type::execSRE,
			&type::execTAS,
			&type::execXAA,
		};
	};
}
