#if !defined(__STDC_VERSION__) || __STDC_VERSION__ < 202311L
#include <stdbool.h>
#endif

#include "test.h"

void bubblesort(uint16_t len, uint16_t* arr)
{
    while (true) {
        bool sorted = true;
        for (uint16_t i = 0; i < len - 1; ++i) {
            if (arr[i] <= arr[i + 1]) {
                continue;
            }

            // Swap these if they are out of order
            const uint16_t tmp = arr[i];
            arr[i] = arr[i + 1];
            arr[i + 1] = tmp;
            sorted = false;
        }

        // We're done!
        if (sorted) {
            return;
        }
    }
}

void run(const struct Input* input, struct Output* output)
{
    for (int16_t i = 7; i >= 0; --i) {
        output->arr[i] = input->arr[i];
    }
    bubblesort(8, output->arr);
}
