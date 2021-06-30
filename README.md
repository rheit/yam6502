YAM6502
=======
*Yet another MOS 6502 Emulator*

A per-cycle emulation of the NMOS 6502 microprocessor, complete with
undocumented instruction support and verified against
[Visual6502](http://visual6502.org/) and
[perfect6502](https://github.com/mist64/perfect6502).

About
-----
There are a lot of 6502 emulators out there, most of which emulate 6502
instructions and not 6502 cycles. YAM6502 is different and emulates cycles
instead of instructions. This design has the advantage that it can be
relatively simple to create an accurate emulation of a system powered by a
6502 processor when it is emulated at the cycle level. When instructions are
emulated as discrete units instead, more work may need to be done to take
into account events that occur during the middle of an instruction's
execution.

The drawback is that per-cycle emulation will always be slower than
per-instruction emulation. However, modern systems are thousands of times
faster than any production 6502 system, so this should not pose too much of
a problem.

### Motivation

For a while, YouTube kept recommending javidx9 (aka One Lone Coder)'s [NES
emulation series of videos](https://www.youtube.com/playlist?list=PLrOv9FMX8xJHqMvSGB_9G9nZZ_4IgteYf)
to me. Before long, I decided to start watching them, and he made emulation
look accessible, so here's my 6502 emulator as a result. Eventually, I would
like to use it in an Apple II emulator, since that family of computers (at
least without any add-in cards) is quite simple and I used them extensively
in elementary school, but for now it's just a stand-alone CPU emulator.

Usage
-----
YAM6502 consists of a single source file and a single header file (yam6502.h
and yam6502.cpp) that are intended to be added to any project that uses it.
Although the CMakeLists.txt here generates a static library out of it, that
is not necessary and is done only to assist with sharing it across the
multiple binaries included here.

### Basic example

See example/helloworld.cpp for a full example. This is a minimal usage
example to provide an idea of what it looks like to use this. Full
documentation follows the example.

```C++
struct RAMBus {
    uint8_t ReadAddr(uint16_t addr) const { return Memory[addr]; }
    void WriteAddr(uint16_t addr, uint8_t val) { Memory[addr] = val; }
    uint8_t &operator[](uint16_t addr) { return Memory[addr]; }

    std::array<uint8_t, 65536> Memory = { 0 };
};

RAMBus bus;
yam::M6502<RAMBus *> cpu{&bus};

// Copy some program data from 'program' to the 6502 memory at address
// $200, where 'program' is an array-like object defined elsewhere.
std::copy(std::begin(program), std::end(program), &bus[0x200]);

// Set the Program Counter to $200.
cpu.setPC(0x200);

// Presumably you would check for a termination condition, but this is
// just a simple example.
while(true) {
    cpu.Tick();
    // Delay if desired
}
```

### The 6502 emulator class
`template<typename bustype, int trap_code=-1> class yam::M6502;`

To use the CPU emulator, you instantiate a class of type yam::M6502.
It has one required template parameter, which describes a "bus" object
that it uses to communicate with your code. This parameter must have pointer
semantics. The second template parameter is optional and describes an
opcode which, if encountered, will trigger a trap handler in the bus.

#### Member functions

  * `explicit M6502(bustype bus) : Bus(bus)`   _(constructor)_
    
    _bus_ is a pointer to the bus object, described in more detail below.
    It must be valid for the life of this M6502 instance.

  * `void Tick()`

    Executes one cycle.

  * `void Reset()`

    Initializes some internal state and begins the processor's reset
    sequence. You must call `Tick()` to actually execute the reset sequence,
    which takes seven cycles, just like on a real 6502.

  * `std::string DisasmOp(uint16_t addr, bool full_line)`

    Disassemble the instruction at _addr_. If _full_line_ is false, then
    only the instruction mnemonic and its operand is output. Otherwise, an
    entire line, formatted similar to the Apple II monitor is produced.

    The full line format is as follows:
    ```
    AAAA-   XX YY ZZ    OPC   OPERAND [OFS]
    ```
    Here, the `AAAA` field is the 16-bit address of the instruction. `XX`,
    `YY`, and `ZZ` are the one to three bytes of the instruction. `XX` is
    always present, as it is the instruction opcode. `YY` and `ZZ` are only
    included if the instruction uses them and are filled with spaces if not.
    `OPC` is the mnemonic for the instruction. `OPERAND` is the
    instruction's operand, if used, and is written in the standard format
    for 6502 assembly. `OFS` is only included for branch instructions and is
    the branch offset, in decimal. All other numbers are in hexadecimal.

    As an example, here is one line produced by the included helloworld.cpp,
    which shows off all the fields (except for `ZZ`):
    ```
    0604-   F0 05       BEQ   $060B   [+5]
    ```

  * `std::string DisasmStep(uint16_t &addr, bool full_line)`

    This is like `DisasmOp` above, but `addr` is passed as a reference and
    will be advanced to the location of the following instruction upon
    return. Thus, disassembling a block of code can easily be done in a
    loop by calling `DisasmStep()` repeatedly. e.g.:

    ```C++
    while (addr < stopaddr) {
        std::cout << cpu.DisasmStep(addr, true);
    }
    ```

#### Accessing registers

The processor registers are protected behind getter/setter pairs.

  * `uint8_t getA() const` &mdash; Set the value of the Accumulator  
    `void setA(uint8_t val)` &mdash; Get the value of the Accumulator

  * `uint8_t getX() const` &mdash; Get the value of the X index register  
    `void setX(uint8_t val)` &mdash; Set the value of the X index register

  * `uint8_t getY() const` &mdash; Get the value of the Y index register  
    `void setY(uint8_t val)` &mdash; Set the value of the Y index register

  * `uint8_t getP() const` &mdash; Get the value of the status register  
    `void setP(uint8_t val)` &mdash; Set the value of the status register

  * `uint16_t getSP() const` &mdash; Get the value of the stack pointer  
    `void setSP(uint16_t val)` &mdash; Set the value of the stack pointer

    Note that although the stack pointer is only 8-bits wide, these
    functions use 16-bit values. The stack pointer returned by `getSP()`
    will always be in the range 0x100-0x1FF, and the pointer passed to
    `setSP()` should be as well.

  * `uint16_t getPC() const` &mdash; Get the value of the program counter  
    `void setPC(uint16_t val)` &mdash; Set the value of the program counter

    Note that if the processor is in the middle of executing an instruction
    when `setPC()` is called, that instruction will be terminated and the
    next cycle will begin executing the instruction at the new program
    counter.

### Processor flag definitions

For convenience, the bit values of the different processor flags are defined
in the `yam::flag` namespace:

  * `yam::flag::C` = 0x01 &mdash; The carry flag
  * `yam::flag::Z` = 0x02 &mdash; The zero flag
  * `yam::flag::I` = 0x04 &mdash; The interrupt disable bit
  * `yam::flag::D` = 0x08 &mdash; The decimal mode bit
  * `yam::flag::B` = 0x10 &mdash; The break bit
  * `yam::flag::RESERVED` = 0x20 &mdash; The reserved bit
  * `yam::flag::V` = 0x40 &mdash; The overflow flag
  * `yam::flag::N` = 0x80 &mdash; The negative flag

These can be tested directly against the processor status flag, e.g.:

```C++
if (cpu.getP() & yam::flag::Z) {
    // The last operation produced a zero value
}
```

### The bus class

The bus is a user-defined class that yam::M6502 uses to communicate with
the rest of the system. It encompasses the address bus, data bus, signalling
pins, and some utility functions. Only two functions are required, while
the rest are all optional. C++20 concepts are used so that if you don't
implement a particular optional bus feature, then no code will be generated
for it in the emulator.

#### Required functions

The two required functions are used to read and write to the bus. The 6502
always does one or the other each cycle. Typically, a 6502 system's address
space is some combination of RAM and memory-mapped I/O, possibly with some
bank switching scheme as well. For this reason, yam::M6502 makes no
assumptions about the memory layout and lets the bus object handle it.

  * `uint8_t ReadAddr(uint16_t addr)`

    Return the byte at `addr`.

  * `void WriteAddr(uint16_t addr, uint8_t val)`

    Write the byte `val` to memory mapped at `addr`.

  * `Trap(yam::M6502<> &cpu, uint16_t opcode_addr)`  
    (if trap opcode is defined)

    If you specified a trap opcode when creating this CPU, then this
    function is required and will be called whenever that opcode is fetched.
    It is passed a reference to this CPU instance and the address of the
    opcode.

    If the handler returns false, then this opcode will execute as normally.
    If the handler returns true, then it handled the opcode instead and the
    next cycle will read the instruction at PC. If `Trap()` does not call
    `setPC()` to change the PC, then this will be the byte after
    `opcode_addr`.

#### Optional functions

In addition to the required functions above, there are a number of optional
functions that can be used to further interact with the CPU emulation. For
the input pins connected to the processor, a `false` value represents logic
level low, and a `true` value represents logic level high.

  * `bool GetRDY()`

    Get the status of the RDY line. If this function is not provided, the
    RDY line is assumed to always be high.

    At present, RDY is only implemented for the undefined opcodes that
    change their behavior depending on its state and does not actually
    enable/disable instruction execution.

  * `bool GetIRQB()`

    Get the status of the <span style="text-decoration:overline">IRQ</span>
    line. If this function is not provided, the 
    <span style="text-decoration:overline">IRQ</span> line is assumed to
    always be high.

  * `bool GetNMIB()`

    Get the status of the <span style="text-decoration:overline">NMI</span>
    line. If this function is not provided, the
    <span style="text-decoration:overline">NMI</span> line is assumed to
    always be high.

  * `uint8_t ReadNoSideEffects(uint16_t addr)`

    This function is called by the disassembler to read memory. If not
    present, the disassembler will call `ReadAddr()` instead.

    If your memory map includes I/O space that triggers actions upon a read
    access and you might end up disassembling that address space, you
    probably want to include this function so that disassembling doesn't
    change the state of the emulated system.

  * `void SyncHandler(yam::M6502<> &cpu, uint16_t opcode_addr)`

    Passed a reference to this CPU and the address about to be read for an
    instruction fetch. This serves as an alternative to full SYNC pin
    emulation, since only the cycle when it is high is likely to be of
    interest, and that is the cycle when `SyncHandler()` is called.

  * `bool BreakHandler(yam::M6502<> &cpu, uint16_t opcode_addr)`

    Called at the beginning of the cycle immediately following the one when
    a BRK instruction was read. The handler is passed a reference to this
    CPU and the address of the BRK instruction being executed.

    Your handler can either return false to let the break sequence execute
    as normal, or you can return true to interrupt the sequence and continue
    execution at the current PC. Note that if you interrupt the sequence,
    PC will be one byte after BRK and not two like it would if the
    interrupt was allowed to continue and execution returned here with RTI.
    This is because the dummy read of the break operand has not yet happened
    when the handler is called. It is perfectly valid to call `setPC()`
    inside a break handler to have execution resume somewhere else.
