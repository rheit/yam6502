#include <fstream>
#include <iostream>
#include <cmath>
#include <cstring>

#include "yam6502.h"
#include "test.h"

// A minimal CIA emulation with timers and IRQ generation, which is enough to get the
// timing tests working. This is based on cia6526.cpp from
// <https://www.commodore.ca/manuals/funet/cbm/documents/chipdata/cia6526.zip>, which
// as far as I know, is public domain.
class MiniCIA {
public:
	// Control register bits
	static constexpr inline uint8_t START		= 0x01;
	//static constexpr inline uint8_t PBON		= 0x02;
	//static constexpr inline uint8_t OUTMODE	= 0x04;
	static constexpr inline uint8_t ONESHOT		= 0x08;
	static constexpr inline uint8_t LOAD		= 0x10;
	//static constexpr inline uint8_t INMODE	= 0x20;		// only phi2 (0) supported
	//static constexpr inline uint8_t SPMODE	= 0x40;
	//static constexpr inline uint8_t TODIN		= 0x80;

	// Different bits for Control Register B
	static constexpr inline uint8_t INMODEB		= 0x40;
	//static constexpr inline uint8_t ALARM		= 0x80;

	[[nodiscard]] uint8_t ReadReg(int reg)
	{
		switch (reg) {
		case 4:		return TimerA.Counter & 0xFF;
		case 5:		return TimerA.Counter >> 8;
		case 6:		return TimerB.Counter & 0xFF;
		case 7:		return TimerB.Counter >> 8;
		case 13:	return ReadIrqData();
		case 14:	return TimerA.Control;
		case 15:	return TimerB.Control;
		}
		return 0;
	}

	void WriteReg(int reg, uint8_t data)
	{
		switch (reg) {
		case 4:		TimerA.SetLo(data); break;
		case 5:		TimerA.SetHi(data); break;
		case 6:		TimerB.SetLo(data); break;
		case 7:		TimerB.SetHi(data); break;
		case 13:	WriteIrqMask(data); break;
		case 14:	TimerA.WriteControl(data); break;
		case 15:	TimerB.WriteControl(data); break;
		}
	}

	void Tick()
	{
		bool a_out = TimerA.Tick(*this, 1);
		// If timer B in cascade mode, stick the output of timer A
		// into its pipeline.
		if (a_out && (TimerB.Control & INMODEB)) {
			TimerB.Delay |= Count1;
		}
		TimerB.Tick(*this, 2);

		// Set interrupt line
		if (IrqDelay & 2) {
			Interrupt = true;
		}

		IrqDelay <<= 1;
	}

	void Reset()
	{
		TimerA.Reset();
		TimerB.Reset();
		IrqData = 0;
		IrqMask = 0x81;
		IrqDelay = 0;
		Interrupt = false;
	}

	[[nodiscard]] uint16_t GetTimerA() const;
	[[nodiscard]] uint16_t GetTimerB() const;
	[[nodiscard]] uint8_t GetTimerADelay() const;
	[[nodiscard]] uint8_t GetTimerBDelay() const;

	[[nodiscard]] bool GetIRQB() const
	{
		return !Interrupt;
	}

	[[nodiscard]] uint8_t GetICR() const;

private:
	uint8_t IrqData = 0;
	uint8_t IrqMask = 0x81;		// As set by the C64 KERNAL
	uint8_t IrqDelay = 0;
	bool Interrupt = false;

	static constexpr inline uint8_t Count0 = 0x01;
	static constexpr inline uint8_t Count1 = 0x02;
	static constexpr inline uint8_t Count2 = 0x04;
	static constexpr inline uint8_t Count3 = 0x08;
	static constexpr inline uint8_t Load0 = 0x10;
	static constexpr inline uint8_t Load1 = 0x20;
	static constexpr inline uint8_t OneShot0 = 0x40;
	static constexpr inline uint8_t DelayMask = ~(Count0 | Load0 | OneShot0);

	struct Timer
	{
		uint16_t Latch = 0xFFFF;
		uint16_t Counter = 0;
		uint8_t Control = 0;
		uint8_t Delay = 0, Feed = 0;

		void Reset()
		{
			Latch = 0xFFFF;
			Counter = 0;
			Control = 0;
			Delay = Feed = 0;
		}

		void SetLo(uint8_t data)
		{
			Latch = (Latch & 0xFF00) | data;
		}

		void SetHi(uint8_t data)
		{
			Latch = (Latch & 0x00FF) | (data << 8);
			// Load counter if timer is stopped
			if (!(Control & START)) {
				Delay |= Load0;
			}
		}

		void WriteControl(uint8_t data)
		{
			// Start/stop timer
			if (data & START && !(data & INMODEB)) {
				Delay |= Count1 | Count0;
				Feed |= Count0;
			}
			else {
				Delay &= ~(Count1 | Count0);
				Feed &= ~Count0;
			}

			// Set/clear one shot mode
			if (data & ONESHOT) {
				Feed |= OneShot0;
			}
			else {
				Feed &= ~OneShot0;
			}

			// Set force load
			if (data & LOAD) {
				Delay |= Load0;
			}

			Control = data;
		}

		bool Tick(MiniCIA &cia, const uint8_t icrbit)
		{
			// Assume no underflow
			bool out = false;

			if (Delay) {
				// Decrement counter
				if (Delay & Count3) {
					Counter--;
				}

				// Check counter underflow
				if (Counter == 0 && (Delay & Count2)) {
					// Signal underflow
					cia.IrqData |= icrbit;

					// Underflow interrupt in next clock
					if (cia.IrqMask & icrbit) {
						cia.IrqDelay |= 1;
					}

					// Stop timer in one shot mode
					if ((Delay | Feed) & OneShot0) {
						Control &= ~START;
						Delay &= ~(Count2 | Count1 | Count0);
						Feed &= ~Count0;
					}

					// Signal underflow externally
					out = true;

					// Reload counter
					Delay |= Load1;
				}

				// Load counter
				if (Delay & Load1) {
					Counter = Latch;

					// Don't decrement counter in next clock
					Delay &= ~Count2;
				}
			}
			// Advance pipeline for next clock
			Delay = ((Delay << 1) & DelayMask) | Feed;

			return out;
		}
	};

	Timer TimerA, TimerB;

	[[nodiscard]] uint8_t ReadIrqData()
	{
		uint8_t icr = IrqData | (Interrupt << 7);

		// Clear interrupt and discard any pending
		Interrupt = false;
		IrqDelay = 0;

		IrqData = 0;
		return icr;
	}

	void WriteIrqMask(uint8_t mask)
	{
		if (mask & 0x80) {
			IrqMask |= mask & 0x1F;
		}
		else {
			IrqMask &= ~(mask & 0x1F);
		}
		// Raise an interrupt in the next cycle if condition matches
		if ((IrqMask & IrqData) && !Interrupt) {
			IrqDelay |= 1;
		}
	}
};

[[nodiscard]] uint16_t MiniCIA::GetTimerA() const
{
	return TimerA.Counter;
}

[[nodiscard]] uint16_t MiniCIA::GetTimerB() const
{
	return TimerB.Counter;
}

[[nodiscard]] uint8_t MiniCIA::GetTimerADelay() const
{
	return TimerA.Delay;
}

[[nodiscard]] uint8_t MiniCIA::GetTimerBDelay() const
{
	return TimerB.Delay;
}

[[nodiscard]] uint8_t MiniCIA::GetICR() const
{
	return IrqData | (Interrupt << 7);
}


struct MiniC64Bus {
	constexpr static inline int TRAP = 2;

	constexpr static inline uint16_t FACHO = 0x62;		// Floating-point accumulator #1: Mantissa (5 bytes)

	constexpr static inline uint16_t FNLEN = 0xB7;		// Length of Current File Name
	constexpr static inline uint16_t SA = 0xB9;			// Secondary address for IO
	constexpr static inline uint16_t FNADR = 0xBB;		// Pointer: Current File Name
	constexpr static inline uint16_t PNTR = 0xD3;		// Cursor column on current line

	constexpr static inline uint16_t CBINV = 0x0316;	// BRK interrupt vector

	constexpr static inline uint16_t READY = 0xA474;	// Enter BASIC immediate mode

	constexpr static inline uint16_t STROUT = 0xAB1E;	// Print 0-terminated string at (Y,A)
	constexpr static inline uint16_t FLOATC = 0xBC49;	// Float unsigned value in FAC+1,2
	constexpr static inline uint16_t FMULT = 0xBA28;	// Multiply FP accum with memory
	constexpr static inline uint16_t MOVAF = 0xBC0C;	// Move FP accum to FP arg
	constexpr static inline uint16_t FADDT = 0xB86A;	// Add FP arg to FP accum
	constexpr static inline uint16_t LINPRNT = 0xBDCD;	// Print XA as unsigned integer
	constexpr static inline uint16_t FOUT = 0xBDDD;		// FP accum to string at bottom of stack

	constexpr static inline uint16_t SCROLY = 0xD011;	// VIC Control Register

	constexpr static inline uint16_t CIA2CRA = 0xDD0E;	// CIA 2 control register A
	constexpr static inline uint16_t CIA2CRB = 0xDD0F;	// CIA 2 control register B

	constexpr static inline uint16_t TIMB = 0xFE66;		// Kernal BRK handler
	constexpr static inline uint16_t PULS = 0xFF48;		// Kernal IRQ handler
	constexpr static inline uint16_t SETNAM = 0xFDF9;	// Set filename parameters

	// Kernal jump table routines
	constexpr static inline uint16_t VEC_IOINIT = 0xFF84;	// Initialize I/O devices
	constexpr static inline uint16_t VEC_RESTOR = 0xFF8A;	// Restore default system and interrupt vectors
	constexpr static inline uint16_t VEC_SETLFS = 0xFFBA;	// Set up a logical file
	constexpr static inline uint16_t VEC_SETNAM = 0xFFBD;	// Set up file name
	constexpr static inline uint16_t VEC_GETIN = 0xFFE4;	// Get a character
	constexpr static inline uint16_t VEC_CHROUT = 0xFFD2;	// Output a character
	constexpr static inline uint16_t VEC_LOAD = 0xFFD5;		// Load RAM from a device

	using cputype = yam::M6502<MiniC64Bus *, TRAP>;

	uint8_t memory[65536]{};
	MiniCIA CIA1;
	MiniCIA CIA2;
	double fp_accum = 0;
	double fp_arg = 0;
	bool DontTrapBreak = false;
	unsigned long long clocks = 0;

	enum class State {
		Running,
		Passed,
		Failed
	} state = State::Running;

	void Tick();
	void Init(const cputype &cpu);
	void StartTest(cputype &cpu, uint16_t loadaddr);
	int LoadTest(std::string_view testname, bool reloc, uint16_t loadaddr);
	static std::string pet2ascii(uint8_t pet);
	bool BreakHandler(cputype &cpu, uint16_t addr);
	void SyncHandler([[maybe_unused]] cputype &cpu, [[maybe_unused]] uint16_t pc) const;
	bool Trap(cputype &cpu, uint16_t addr);
	[[nodiscard]] uint16_t ReadWord(uint16_t addr) const;
	void WriteWord(uint16_t addr, uint16_t data);
	uint8_t ReadAddr(uint16_t addr);
	void WriteAddr(uint16_t addr, uint8_t data);

	[[nodiscard]] uint8_t ReadNoSideEffects(uint16_t addr) const
	{
		return memory[addr];
	}

	[[nodiscard]] bool GetIRQB() const
	{
		return CIA1.GetIRQB();
	}
	[[nodiscard]] bool GetNMIB() const
	{
		return CIA2.GetIRQB();
	}
};

void MiniC64Bus::Tick()
{
	CIA1.Tick();
	CIA2.Tick();
	++clocks;
}

void MiniC64Bus::Init(const cputype &cpu)
{
	CIA1.Reset();
	CIA2.Reset();

	// Set traps for several ROM routines that we will implement natively
	for (auto traploc :
		{ TIMB, READY, VEC_RESTOR, VEC_LOAD, VEC_CHROUT,
			FLOATC, FMULT, MOVAF, FADDT, FOUT, STROUT, LINPRNT }) {
		memory[traploc] = TRAP;
		memory[traploc + 1] = 0x60;	// RTS
	}

	memory[VEC_IOINIT] = 0x60;		// RTS because not relevant here

	// The secondary address, which indicates whether to relocate
	// the load, is the only thing we care about for SETLFS.
	memory[VEC_SETLFS] = 0x84;		// STY zp
	memory[VEC_SETLFS + 1] = SA;
	memory[VEC_SETLFS + 2] = 0x60;	// RTS

	// Make GETIN always return a space character.
	memory[VEC_GETIN] = 0xA9;		// LDA #
	memory[VEC_GETIN + 1] = ' ';
	memory[VEC_GETIN + 2] = 0x60;	// RTS

	// Set up SETNAM routine.
	memory[VEC_SETNAM] = 0x4C;		// JMP
	WriteWord(VEC_SETNAM + 1, SETNAM);
	
	static const uint8_t setnam[] = {
		0x85, FNLEN,		// STA FNLEN
		0x86, FNADR,		// STX FNADR
		0x84, FNADR + 1,	// STY FNADR+1
		0x60				// RTS
	};
	std::copy(setnam, setnam + sizeof(setnam), &memory[SETNAM]);

	// Setup vectors
	WriteWord(cpu.opToVector(yam::op::IRQ), PULS);
	WriteWord(CBINV, TIMB);

	// Copy IRQ handler
	static const uint8_t puls[] = {
		0x48,				// PHA			; push accumulator
		0x8A,				// TXA			; push X register
		0x48,				// PHA
		0x98,				// TYA			; push Y register
		0x48,				// PHA
		0xBA,				// TSX			; check status of B bit
		0xBD, 0x04, 0x01,	// LDA $0104,X
		0x29, 0x10,			// AND #$10
		0xF0, 0x03,			// BEQ $FF58
		0x6C, 0x16, 0x03,	// JMP ($0316)	; jump to break vector if set
		0x6C, 0x14, 0x03,	// JMP ($0314)	; jump to IRQ vector if clear
	};
	std::copy(puls, puls + sizeof(puls), &memory[PULS]);
}

void MiniC64Bus::StartTest(cputype &cpu, uint16_t loadaddr)
{
	Init(cpu);

	// Set up top of stack so RTS goes to READY.
	WriteWord(0x1FE, READY - 1);
	cpu.setSP(0x1FD);
	cpu.setP(0);

	// Find the real program immediately after the BASIC stub.
	// BASIC program are stored in a single-linked list of program lines,
	// with a NULL pointer indicating the end of the program.
	auto addr = ReadWord(loadaddr);
	while (auto next = ReadWord(addr)) {
		addr = next;
	}
	// Start of routine is the first non-0 byte.
	addr += 2;
	while (memory[addr] == 0) {
		++addr;
	}
	cpu.setPC(addr);
}

// Returns address file was loaded to, or -1 on failure.
int MiniC64Bus::LoadTest(std::string_view testname, bool reloc, uint16_t loadaddr)
{
	std::filesystem::path path{ TESTS_BIN };
	// Convert file name from PETSCII to ASCII. In practice, this
	// amounts to converting from uppercase to lowercase.
	std::string name_ascii;
	name_ascii.reserve(testname.size());
	for (auto c : testname) {
		name_ascii += pet2ascii(c);
	}
	path /= "lorenz";
	path /= name_ascii;
	path += ".prg";

	// The TRAP* tests expect to get a BRK sequence.
	DontTrapBreak = testname.compare(0, 4, "TRAP", 4) == 0;

	std::ifstream input(path, std::ios::binary | std::ios::in);
	if (!input.is_open()) {
		pfileerror(path, "Could not open file");
		return -1;
	}

	// Read base address of file
	uint8_t loadbase[2] = { 0x01, 0x08 };
	if (!input.read(reinterpret_cast<char *>(loadbase), 2)) {
		pfileerror(path, "Could not read file");
		return -1;
	}
	// But only use the base address if we are not relocating the file.
	if (!reloc) {
		loadaddr = (static_cast<uint16_t>(loadbase[1]) << 8) | loadbase[0];
	}
	// Try to read as much as possible
	input.read(reinterpret_cast<char *>(&memory[loadaddr]), static_cast<std::streamsize>(65536) - loadaddr);
	if (input.gcount() == 0 || input.bad()) {
		pfileerror(path, "Could not read file");
		return -1;
	}
	return loadaddr;
}

std::string MiniC64Bus::pet2ascii(uint8_t pet)
{
	if (pet == 147/* Clear screen */ || pet == 14/* Switch to lower case */) {
		return "";
	}
	if (pet == 145) {	// Cursor up
		return "\33[A";	// This is the ANSI escape sequence to move the cursor up
	}
	if (pet >= 'A' && pet <= 'Z') {
		return std::string(1, pet + ('a' - 'A'));
	}
	if (pet >= 'a' && pet <= 'z') {
		return std::string(1, pet + ('A' - 'a'));
	}
	if (pet >= 0xC1 && pet <= 0xDA) {
		return std::string(1, pet - (0xC1 - 'A'));
	}
	if (pet == '\r') {
		return "\n";
	}
	return std::string(1, pet);
}

// Debug helper for tracing the test execution.
void MiniC64Bus::SyncHandler([[maybe_unused]] cputype &cpu, [[maybe_unused]] uint16_t pc) const
{
#if 0
	if (
		//	!(pc >= 0xa80 && pc < 0xa8f) // inside savestack (irq.prg)
		//	&& !(pc >= 0xa90 && pc < 0xaaf) // inside restorestack (irq.prg)
		!(pc >= 0xa4b && pc < 0xa5a) // inside savestack (nmi.prg)
		&& !(pc >= 0xa5b && pc < 0xa7a) // inside restorestack (nmi.prg)
		) {
		printf("A=%02X X=%02X Y=%02X S=%02X ",
			cpu.getA(), cpu.getX(), cpu.getY(), cpu.getSP());
		print_p(cpu.getP());
		printf(" T=[{%02x}%-5u {%02x}%-5u {%02x}%-5u {%02x}%-5u] %s\n",
			CIA1.GetTimerADelay(),
			CIA1.GetTimerA(),
			CIA1.GetTimerBDelay(),
			CIA1.GetTimerB(),
			CIA2.GetTimerADelay(),
			CIA2.GetTimerA(),
			CIA2.GetTimerBDelay(),
			CIA2.GetTimerB(),
			cpu.DisasmOp(pc, true).c_str());
	}
#endif
}

// This was used to catch unimplemented routines that were needed to get the different
// tests running. At this point, all the tests run, so this routine isn't needed anymore,
// but keeping it in ensures that there is one path where the relevant code in M6502
// is actually compiled.
bool MiniC64Bus::BreakHandler([[maybe_unused]] cputype &cpu, uint16_t addr)
{
	if (!DontTrapBreak && addr >= 0xA000) {
		fprintf(stderr, "Unhandled routine at $%04X\n", addr);
		fprintf(stderr, "Zero page dump:\n");
		dump_mem(stderr, memory, 0, 256);
		state = State::Failed;
		return true;
	}
	return false;
}

// Implement various ROM routines needed to get the tests to run.
bool MiniC64Bus::Trap(cputype &cpu, uint16_t addr)
{
	switch (addr) {
	// Kernal routines used by the tests.

	case TIMB:			// Should never be reached if everything is working properly.
		state = State::Failed;
		return true;

	case READY:			// Returning to BASIC means the tests are over.
		state = State::Passed;
		return true;

	case VEC_RESTOR:	// Restore state for the next test.
		Init(cpu);
		return true;

	case VEC_LOAD: {	// Load a test.
		std::string_view test{ reinterpret_cast<char *>(&memory[ReadWord(FNADR)]), memory[FNLEN] };
		if (LoadTest(test, !!memory[SA], (cpu.getX() << 8) | cpu.getY()) < 0) {
			std::cerr << "Failed loading " << test << '\n';
			state = State::Failed;
		}
		return true;
	}

	case VEC_CHROUT:	// Write out a PETSCII character
		std::cout << pet2ascii(cpu.getA());
		return true;


	// Various BASIC routines used to print 16- and 32-bit decimal numbers

	case FLOATC:		// Convert unsigned word to floating point
		// FACHO[0] and FACHO[1] contain an unsigned 16-bit integer
		// in big-endian order. X contains the MBF (Microsoft Binary
		// Format) exponent. C is set for a positive number, clear for
		// a negative number. We store the result in a private
		// accumulator in the host's native floating point format
		// rather than the zero page MBF accumulator, since none of
		// the tests actually read it.
		fp_accum = ((memory[FACHO] << 8) | memory[FACHO + 1]) * std::exp2(cpu.getX() - (128 + 16));
		if (!(cpu.getP() & yam::FLAG_C)) {
			fp_accum = -fp_accum;
		}
		return true;

	case FMULT: {		// Floating point multiply
		static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
		// A is the low byte of the memory arg's address
		// Y is the high byte of the memory arg's address
		//
		// On the C64, this is a 5-byte MBF floating point number,
		// which is very similar to IEEE-754. The first byte is the
		// exponent, 128 biased. The next bit is the sign. Then the next
		// 31 bits are are the mantissa, in big-endian format. There is an
		// implicit 1 just after the radix point (compared to just before
		// it for IEEE-754).
		const uint8_t *mbf = &memory[(cpu.getY() << 8) | cpu.getA()];
		if (mbf[0] == 0) {
			fp_arg = 0;
		}
		else {
			uint64_t fpbits =
				(uint64_t(mbf[1] & 0x80) << 56)
				| (uint64_t(mbf[0] + (1023 - 129)) << 52)
				| (uint64_t(mbf[1] & 0x7F) << 45)
				| (uint64_t(mbf[2]) << 37)
				| (uint64_t(mbf[3]) << 29)
				| (uint64_t(mbf[4]) << 21);
			std::memcpy(&fp_arg, &fpbits, 8);
		}
		fp_accum *= fp_arg;
		return true;
	}

	case MOVAF:		// Move floating point accumulator to floating point argument.
		fp_arg = fp_accum;
		return true;

	case FADDT:		// Add floating point argument to floating point accumulator.
		fp_accum += fp_arg;
		return true;

	case FOUT:		// Convert floating point to string at bottom of stack space.
		snprintf(reinterpret_cast<char *>(&memory[0x100]), 32, "% .f", fp_accum);
		cpu.setY(1);
		cpu.setA(0);
		return true;

	case LINPRNT:	// Print AX as an unsigned integer
		snprintf(reinterpret_cast<char *>(&memory[0x100]), 32, "%u",
			static_cast<unsigned>((cpu.getA() << 8) | cpu.getX()));
		cpu.setY(1);
		cpu.setA(0);
		[[fallthrough]];

	case STROUT:	// Print the 0-terminated string at YA
		std::cout << reinterpret_cast<char *>(&memory[(cpu.getY() << 8) | cpu.getA()]);
		return true;
	}
	return false;
}

// Read a 16-bit value from RAM
[[nodiscard]] uint16_t MiniC64Bus::ReadWord(uint16_t addr) const
{
	return memory[addr] + (memory[addr + 1] << 8);
}

// Write a 16-bit value to RAM
void MiniC64Bus::WriteWord(uint16_t addr, uint16_t data)
{
	memory[addr] = data & 0xFF;
	memory[addr + 1] = data >> 8;
}

// Read from RAM or I/O
uint8_t MiniC64Bus::ReadAddr(uint16_t addr)
{
	if (addr == SCROLY) {
		// Always say we're inside the border, since we're not emulating any VIC-II DMA.
		// The other bits don't matter, since the tests just read this to ensure that
		// the VIC-II won't pull the RDY line low during the test.
		return 0x80;
	}
	if ((addr & 0xFF00) == 0xDC00) {
		return CIA1.ReadReg(addr & 15);
	}
	if ((addr & 0xFF00) == 0xDD00) {
		return CIA2.ReadReg(addr & 15);
	}
	return memory[addr];
}

// Write to RAM or I/O.
void MiniC64Bus::WriteAddr(uint16_t addr, uint8_t data)
{
	if (addr == PNTR) {
		std::cout << '\r';
		if (data != 0) {
			printf("\33[%uC", data);
		}
	}
	else if ((addr & 0xFF00) == 0xDC00) {
		CIA1.WriteReg(addr & 15, data);
	}
	else if ((addr & 0xFF00) == 0xDD00) {
		CIA2.WriteReg(addr & 15, data);
	}
	else {
		memory[addr] = data;
	}
}

// Start the Lorenz test suite rolling.
void run_lorenz_tests()
{
	printf("Running Wolfgang Lorenz's test suite\n");
	MiniC64Bus bus;
	MiniC64Bus::cputype cpu(&bus);
	if (auto loadaddr = bus.LoadTest("START", false, 0)) {
		bus.StartTest(cpu, loadaddr);
		auto start = std::chrono::steady_clock::now();
		while (bus.state == MiniC64Bus::State::Running) {
			cpu.Tick();
			bus.Tick();
		}
		std::chrono::duration<double> diff = std::chrono::steady_clock::now() - start;
		std::cout << bus.clocks << " cycles in " << diff.count() << " sec ("
			<< bus.clocks / diff.count() / 1000000 << " MHz)\n";
		if (bus.state == MiniC64Bus::State::Failed) {
			std::cout << "  Failed\n";
		}
		else {
			std::cout << "  Ok\n";
		}
		std::cout << "Exit code " << static_cast<int>(bus.memory[0xd7ff]) << '\n';
	}
}
