#include "fib.h"

static uint16_t fib(uint16_t n)
{
    uint16_t a = 0;
    uint16_t b = 1;
    while (n) {
        const uint16_t next = a + b;
        a = b;
        b = next;
        --n;
    }
    return a;
}

void run(const struct Input* input, struct Output* output)
{
    output->result = fib(input->to);
}
