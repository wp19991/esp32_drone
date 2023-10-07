/**
 * ESP-Drone Firmware
 * cfassert.h - Assert macro
 */

// #include "console.h"

#ifndef __CFASSERT_H__
#define __CFASSERT_H__

/**
 * @brief Assert that verifies that a pointer is pointing at memory that can be
 * used for DMA transfers. There are two types of RAM in the Crazyflie and CCM
 * does not work for DMA. Flash and RAM can be accessed by the DMA.
 *
 * @param[in] PTR : the pointer to verify
 */

#define ASSERT_DMA_SAFE(PTR)

/**
 * Assert handler function
 */
void assertFail(char *exp, char *file, int line);
/**
 * Print assert snapshot data
 */
void printAssertSnapshotData();
/**
 * Store assert snapshot data to be read at startup if a reset is triggered (watchdog)
 */
void storeAssertFileData(const char *file, int line);
/**
 * Store hardfault data to be read at startup if a reset is triggered (watchdog)
 * Line information can be printed using:
 * > make gdb
 * gdb> info line *0x<PC>
 */
void storeAssertHardfaultData(
    unsigned int r0,
    unsigned int r1,
    unsigned int r2,
    unsigned int r3,
    unsigned int r12,
    unsigned int lr,
    unsigned int pc,
    unsigned int psr);

/**
 * Store assert data to be read at startup if a reset is triggered (watchdog)
 */
void storeAssertTextData(const char *text);

/**
 * @brief Check for assert information to indicate that the system was restarted
 * after a failed assert.
 *
 * @return true   If assert information exists
 * @return false  If no assert information exists
 */
bool cfAssertNormalStartTest(void);

#endif //__CFASSERT_H__
