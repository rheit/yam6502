/*
** test.cpp
** Runs all the 6502 accuracy tests.
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

#include <cstdio>
#include <cerrno>
#include <cstring>
#include <iostream>

#include "yam6502.hpp"
#include "test.h"

void dump_mem(FILE *out, const uint8_t *memory, uint16_t start, uint16_t end)
{
	for (auto i = start; i < end; ++i) {
		if (!(i & 15)) {
			fprintf(out, "\n%04X:", i);
		}
		else if (!(i & 3)) {
			fputc(' ', out);
		}
		fprintf(out, " %02X", memory[i]);
	}
	fputc('\n', out);
}

void print_p(unsigned p)
{
	printf("P=%02X [%c%c%c%c%c%c]",
		p & 0xFF,
		p & yam::flag::N ? 'N' : '.',
		p & yam::flag::V ? 'V' : '.',
		p & yam::flag::D ? 'D' : '.',
		p & yam::flag::I ? 'I' : '.',
		p & yam::flag::Z ? 'Z' : '.',
		p & yam::flag::C ? 'C' : '.'
	);
}

void pfileerror(std::filesystem::path path, const char *errmsg)
{
	puts(TESTS_BIN);
	std::cerr << errmsg << ": " << strerror(errno) << ' ' << path << '\n';
}

int main()
{
	run_functional_test();
	run_decimal_test();
	run_interrupt_test();
	run_lorenz_tests();
	return 0;
}
