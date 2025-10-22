#include "test.h"

static void rot13(uint8_t len, uint8_t* str)
{
    for (; len != 0; --len, str++) {
        if (*str >= 'A' && *str <= 'Z') {
            // Shift letters in the uppercase range
            *str += 13;
            if (*str > 'Z') {
                // Wrap them
                *str -= 'Z' - 'A' + 1;
            }
        } else if (*str >= 'a' && *str <= 'z') {
            // Shift letters in lowercase range
            *str += 13;
            if (*str > 'z') {
                // Wrap them
                *str -= 'z' - 'a' + 1;
            }
        }
    }
}

void run(const struct Input* input, struct Output* output)
{
    for (int8_t i = sizeof(output->str) - 1; i >= 0; --i) {
        output->str[i] = input->str[i];
    }
    rot13(16, output->str);
}
