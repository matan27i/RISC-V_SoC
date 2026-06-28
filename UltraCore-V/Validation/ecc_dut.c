#include <stdint.h>

#define ECC_BASE 0x4000020
#define ECC_DATA (*(volatile uint32_t *)(ECC_BASE + 0x00))
#define ECC_STATUS (*(volatile uint32_t *)(ECC_BASE + 0x04))
#define ECC_DATA_READY 0
#define ECC_SINGLE_ERROR 1
#define ECC_DOUBLE_ERROR 2

uint8_t ecc_data(uint8_t *err_status)
{
    uint32_t current_status;
    while(!(ECC_STATUS & (1 << ECC_DATA_READY)))
    {
    }   
    current_status = ECC_STATUS;
    if (current_status & (1 << ECC_DOUBLE_ERROR)) {
        *err_status = 2;
    } else if (current_status & (1 << ECC_SINGLE_ERROR)) {
        *err_status = 1;
    } else {
        *err_status = 0;
    }
    return (uint8_t)ECC_DATA;