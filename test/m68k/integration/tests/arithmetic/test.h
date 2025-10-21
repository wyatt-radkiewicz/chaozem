#ifndef TEST_H
#define TEST_H

#include <stdint.h>

struct Input {
    uint16_t a;
    uint16_t b;
};

struct Output {
    uint16_t result;
};

void run(const struct Input* input, struct Output* output);

#endif