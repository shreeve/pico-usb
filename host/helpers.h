// Hex dump (mode: 0 = hex; 1 = hex + ascii; 2 = hex + ascii + no newline)
void hexdump(const void* data, size_t size, uint mode) {
    const unsigned char* byte = (const unsigned char *) data;
    size_t i, j;

    for (i = 0; i < size; i += 16) {
        printf("\t│ %08zx │ ", i); // Print the offset

        // Print hex values
        for (j = 0; j < 16; j++) {
            if (i + j < size) {
                printf("%02x ", byte[i + j]);
            } else {
                printf("   "); // Pad if less than 16 bytes in the line
            }
        }

        printf(" │ ");

        if (mode > 1) return;

        // Print ASCII values
        if (mode == 1) {
            for (j = 0; j < 16; j++) {
                if (i + j < size) {
                    unsigned char ch = byte[i + j];
                    printf("%c", (ch >= 32 && ch <= 126) ? ch : '.');
                }
            }
        }

        printf("\n");
    }
}

// Binary dump
void bindump(uint8_t *str, uint32_t val) {
    uint32_t bit = 1 << 31u;
    size_t i;

    printf("%s\t│ ", str);

    for (i = 0; i < 32; i++) {
        printf("%c", val & bit ? '1' : '0');
        bit >>= 1u;
        if (i % 4 == 7) printf(" ");
    }

    printf(" │ 0x%08x\n", val);
}
