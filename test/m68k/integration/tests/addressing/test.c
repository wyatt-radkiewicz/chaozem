#include "test.h"

uint8_t a = 0x55;
uint8_t b = 0xAA;

void run(const struct Input* input, struct Output* output)
{
    output->result = 0;
    for (uint16_t i = input->offset; i < input->offset + input->count; i++) {
        output->result += (input->a[i] ^ a) * (input->b[i] ^ b);
        a += input->a[i];
        b += input->b[i];
    }
}
