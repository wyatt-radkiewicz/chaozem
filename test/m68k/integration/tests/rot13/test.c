#include "test.h"

static void rot13(uint8_t len, uint8_t* str)
{
    for (; len != 0; --len, str++) {
        if (*str >= 'A' && *str <= 'Z') {
            *str += 13;
            if (*str > 'Z') {
                *str -= 'Z' - 'A' + 1;
            }
        } else if (*str >= 'a' && *str <= 'z') {
            *str += 13;
            if (*str > 'z') {
                *str -= 'z' - 'a' + 1;
            }
        }
    }
}

void run(const struct Input* input, struct Output* output)
{
    for (int8_t i = 15; i > -1; --i) {
        output->str[i] = input->str[i];
    }
    rot13(16, output->str);
}
