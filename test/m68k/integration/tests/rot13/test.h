#ifndef TEST_H
#define TEST_H

#include <stdint.h>

struct Input {
    uint8_t str[16];
};

struct Output {
    uint8_t str[16];
};

void run(const struct Input* input, struct Output* output);

#endif
