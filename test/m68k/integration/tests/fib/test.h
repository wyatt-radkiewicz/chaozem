#ifndef TEST_H
#define TEST_H

#include <stdint.h>

struct Input {
    uint32_t to;
};

struct Output {
    uint32_t result;
};

void run(const struct Input* input, struct Output* output);

#endif
