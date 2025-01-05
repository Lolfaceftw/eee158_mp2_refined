/**
 * @file main.c
 * @brief Module 5 Sample: "Keystroke Hexdump"
 *
 * @author Alberto de Villa <alberto.de.villa@eee.upd.edu.ph>
 * @date 28 Oct 2024
 */

// Common include for the XC32 compiler
#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "platform/blink_settings.h"

#include "platform.h"

/////////////////////////////////////////////////////////////////////////////

/*
 * Copyright message printed upon reset
 * 
 * Displaying author information is optional; but as always, must be present
 * as comments at the top of the source file for copyright purposes.
 * 
 * FIXME: Modify this prompt message to account for additional instructions.
 */
#define HOME_KEY 0x1B    // ASCII for Home key
#define CTRL_E 0x05     // ASCII for CTRL+E

volatile int ignore_all = 0;
volatile int reset_timer = 1;
volatile int ready_to_exit = 0;
volatile int present = 1;
volatile int toggle = 0;
volatile int e_stop = 0;

int tcc3_read(void) {
    TCC3_REGS -> TCC_CTRLBSET |= (0x4 << 5);
    while ((TCC3_REGS -> TCC_SYNCBUSY) & (1 << 4) == 1);
    return TCC3_REGS -> TCC_COUNT;
}

int tcc0_read(void) {
    TCC0_REGS -> TCC_CTRLBSET |= (0x4 << 5);
    while ((TCC0_REGS -> TCC_SYNCBUSY) & (1 << 4) == 1);
    return TCC0_REGS -> TCC_COUNT;
}

static char banner_msg[] =
        "\033[1;1H"
        "+--------------------------------------------------------------------+\r\n"
        "| EEE 158: Electrical and Electronics Engineering Laboratory V       |\r\n"
        "|          Academic Year 2024-2025, Semester 1                       |\r\n"
        "|                                                                    |\r\n"
        "| Solution: Machine Problem 2                                        |\r\n"
        "|                                                                    |\r\n"
        "| Author:  Christian Klein C. Ramos // SN 2022 03126                 |\r\n"
        "| Date:    03 Jan 2025                                               |\r\n"
        "+--------------------------------------------------------------------+\r\n"
        "\r\n";
static const char init_banner_msg[] =
        "\033[1;1H"
        "+--------------------------------------------------------------------+\r\n"
        "| EEE 158: Electrical and Electronics Engineering Laboratory V       |\r\n"
        "|          Academic Year 2024-2025, Semester 1                       |\r\n"
        "|                                                                    |\r\n"
        "| Solution: Machine Problem 2                                        |\r\n"
        "|                                                                    |\r\n"
        "| Author:  Christian Klein C. Ramos // SN 2022 03126                 |\r\n"
        "| Date:    03 Jan 2025                                               |\r\n"
        "+--------------------------------------------------------------------+\r\n"
        "\r\n"
        "System Status: [ ACTIVE ]\r\n"
        "Motor Direction: [ STOPPED ]\r\n"
        "Speed: [ 0% ]\r\n";

static const char blank[] =
        "\033[2J";

//////////////////////////////////////////////////////////////////////////////

// Program state machine

typedef struct prog_state_type {
    // Flags for this program
#define PROG_FLAG_BANNER_PENDING	0x0001	// Waiting to transmit the banner
#define PROG_FLAG_UPDATE_PENDING	0x0002	// Waiting to transmit updates
#define PROG_FLAG_GEN_COMPLETE		0x8000	// Message generation has been done, but transmission has not occurred; 32768; 2**15

    uint16_t flags;

    // Transmit stuff
    /*
     * Declares a four element array with the buffer and length of the message.
     */
    platform_usart_tx_bufdesc_t tx_desc[4];

    char tx_buf[64];
    uint16_t tx_blen; // [0, 65535]

    // Receiver stuff
    platform_usart_rx_async_desc_t rx_desc; // Buffer, length, type of completion; if applicable, completion info
    uint16_t rx_desc_blen;
    char rx_desc_buf[16];
} prog_state_t;

/*
 * Initialize the main program state
 * 
 * This style might be familiar to those accustomed to he programming
 * conventions employed by the Arduino platform.
 */
static void prog_setup(prog_state_t *ps) {
    memset(ps, 0, sizeof (*ps));

    platform_init();

    ps->rx_desc.buf = ps->rx_desc_buf;
    ps->rx_desc.max_len = sizeof (ps->rx_desc_buf);

    platform_usart_cdc_rx_async(&ps->rx_desc);
    return;
}

BlinkSetting currentSetting = OFF;

const char *blinkSettingStrings[NUM_SETTINGS] = {
    "Blink Setting: [   OFF  ]\r\n",
    "Blink Setting: [  SLOW  ]\r\n",
    "Blink Setting: [ MEDIUM ]\r\n",
    "Blink Setting: [  FAST  ]\r\n",
    "Blink Setting: [   ON   ]\r\n"
};


static const char CHANGE_MODE[] = "\033[12;1H\033[0K";

static void updateBlinkSetting(prog_state_t *ps, bool increase) {
    if (increase && currentSetting < ON) {
        currentSetting++;
    } else if (!increase && currentSetting > OFF) {
        currentSetting--;
    }

    ps->tx_desc[0].buf = CHANGE_MODE;
    ps->tx_desc[0].len = sizeof (CHANGE_MODE) - 1;
    ps->tx_desc[1].buf = blinkSettingStrings[currentSetting];
    ps->tx_desc[1].len = strlen(blinkSettingStrings[currentSetting]);

    platform_usart_cdc_tx_async(ps->tx_desc, 2);
    // Still doesn't fix the problem!
    platform_blink_modify();
}

/*
 * Do a single loop of the main program
 * 
 * This style might be familiar to those accustomed to he programming
 * conventions employed by the Arduino platform.
 */
int init = 0;
// Add these escape sequences at the top with other static constants
static const char ESC_SEQ_BUTTON_POS[] = "\033[11;1H"; // Position cursor at button state
static const char BUTTON_PRESSED[] = "On-board button: [Pressed] ";
static const char BUTTON_RELEASED[] = "On-board button: [Released]";
static char current_banner[sizeof (banner_msg)];
static const char EMERGENCY[] = "\033[13;1H";
static const char EMERGENCY_ON[] = "EMERGENCY MODE ON\r\n";
int i = 0;
volatile int ignore_init = 1;
volatile int valid_input = 0;
char h[] = "test";
volatile int absent = 0;
static void prog_loop_one(prog_state_t *ps) {
    uint16_t a = 0, b = 0, c = 0;

    // Do one iteration of the platform event loop first.
    platform_do_loop_one();
    platform_blink_modify();

    if (valid_input == 1) {
        TCC0_REGS -> TCC_COUNT = 0; // Reset counter to zero.
        while ((TCC0_REGS -> TCC_SYNCBUSY) & (1 << 4)); // This register is write-synchronized.
        valid_input = 0;
    }
    if (tcc0_read() >= TCC0_REGS->TCC_PER || ((ps -> rx_desc_buf[0] == 0x1B) && (ps -> rx_desc_buf[1] == 0x4F) && (ps -> rx_desc_buf[2] == 0x77))) {
        ps->tx_desc[0].buf = blank;
        ps->tx_desc[0].len = sizeof (blank) - 1;
        platform_usart_cdc_tx_async(&ps->tx_desc[0], 1);
        absent = 1;
        //PORT_SEC_REGS -> GROUP[0].PORT_OUTCLR = (1 << 1);
    }
    // Print out the banner
    if (init == 0) {
        ps->tx_desc[0].buf = init_banner_msg;
        ps->tx_desc[0].len = sizeof (init_banner_msg) - 1;
        platform_usart_cdc_tx_async(&ps->tx_desc[0], 1);
        ps->tx_desc[1].buf = h;
        ps->tx_desc[1].len = sizeof (h) - 1;

        init = 1;
    }
    if (ignore_all == 1 || ignore_init == 1) {
        // Something from the UART?
        if (ps->rx_desc.compl_type == PLATFORM_USART_RX_COMPL_DATA) {

            char received_char = ps->rx_desc_buf[0];
            if (received_char == CTRL_E || (received_char == 0x1B && ps -> rx_desc_buf[2] == 0x48)) {
                ps->flags |= PROG_FLAG_BANNER_PENDING;
                ignore_init = 0;
                absent = 0;
                valid_input = 1;
                e_stop = 0;
            } else {
                valid_input = 0;
                ps->flags |= PROG_FLAG_UPDATE_PENDING;
            }
            ps->rx_desc_blen = ps->rx_desc.compl_info.data_len;
        }
    }
    if ((ignore_all == 0 || ready_to_exit == 1) && ignore_init == 0) {

        // Something happened to the pushbutton?
        if ((a = platform_pb_get_event()) != 0) {
            valid_input = 1;
            if ((a & PLATFORM_PB_ONBOARD_PRESS) != 0) {
                ps->tx_desc[0].buf = ESC_SEQ_BUTTON_POS;
                ps->tx_desc[0].len = sizeof (ESC_SEQ_BUTTON_POS) - 1;
                ps->tx_desc[1].buf = BUTTON_PRESSED;
                ps->tx_desc[1].len = sizeof (BUTTON_PRESSED) - 1;
                platform_usart_cdc_tx_async(&ps->tx_desc[0], 2);
                if (ready_to_exit == 1) {
                    ready_to_exit = 0;
                    ignore_all = 0;
                    e_stop = 0;
                }
            } else if ((a & PLATFORM_PB_ONBOARD_RELEASE) != 0) {
                ps->tx_desc[0].buf = ESC_SEQ_BUTTON_POS;
                ps->tx_desc[0].len = sizeof (ESC_SEQ_BUTTON_POS) - 1;
                ps->tx_desc[1].buf = BUTTON_RELEASED;
                ps->tx_desc[1].len = sizeof (BUTTON_RELEASED) - 1;
                platform_usart_cdc_tx_async(&ps->tx_desc[0], 2);
            }
        }
        // Something from the UART?
        if (ps->rx_desc.compl_type == PLATFORM_USART_RX_COMPL_DATA && ready_to_exit == 0) {
            char received_char = ps->rx_desc_buf[0];

            if (received_char == CTRL_E || (received_char == 0x1B && ps -> rx_desc_buf[2] == 0x48)) {
                ps->flags |= PROG_FLAG_BANNER_PENDING;
                valid_input = 1;
                absent = 0;
            } else {
                ps->flags |= PROG_FLAG_UPDATE_PENDING;
            }
            ps->rx_desc_blen = ps->rx_desc.compl_info.data_len;
        }
    }

    if (reset_timer == 0) {

        if (tcc3_read() >= TCC3_REGS -> TCC_PER) {
            PORT_SEC_REGS -> GROUP[0].PORT_OUTCLR = (1 << 1);
            ready_to_exit = 1;
        }

    } else {
        TCC3_REGS -> TCC_COUNT = 0; // Reset counter to zero.
        while ((TCC3_REGS -> TCC_SYNCBUSY) & (1 << 4)); // This register is write-synchronized.
    }

    ////////////////////////////////////////////////////////////////////

    // Process any pending flags (BANNER)
    do {
        if ((ps->flags & PROG_FLAG_BANNER_PENDING) == 0)
            break;

        if (platform_usart_cdc_tx_busy())
            break;

        if ((ps->flags & PROG_FLAG_GEN_COMPLETE) == 0) {

            ps->tx_desc[0].buf = banner_msg;
            ps->tx_desc[0].len = sizeof (banner_msg) - 1;
            platform_usart_cdc_tx_async(&ps->tx_desc[0], 1);

            ps->flags |= PROG_FLAG_GEN_COMPLETE;
            // Reset receive buffer immediately
            ps->rx_desc.compl_type = PLATFORM_USART_RX_COMPL_NONE;
            platform_usart_cdc_rx_async(&ps->rx_desc);
        }

        if (platform_usart_cdc_tx_async(&ps->tx_desc[0], 1)) {
            ps->flags &= ~(PROG_FLAG_BANNER_PENDING | PROG_FLAG_GEN_COMPLETE);
        }
    } while (0);

    // Process any pending flags (UPDATE)
    do {
        if ((ps->flags & PROG_FLAG_UPDATE_PENDING) == 0)
            break;

        if (platform_usart_cdc_tx_busy())
            break;

        if ((ps->flags & PROG_FLAG_GEN_COMPLETE) == 0) {
            // Message has not been generated.
            // Echo back the received packet as a hex dump.
            if (ignore_all == 1) {
                ps->tx_desc[0].buf = "\033[11;1H";
                ps->tx_desc[0].len = sizeof ("\033[11;1H") - 1;
                ps->tx_desc[1].buf = "E-STOP";
                ps->tx_desc[1].len = sizeof ("E-STOP") - 1;

                platform_usart_cdc_tx_async(&ps->tx_desc[0], 2);
            }
            memset(ps->tx_buf, 0, sizeof (ps->tx_buf));
            if (ps->rx_desc_blen > 0) {
                ps->tx_desc[1].len = 0;
                ps->tx_desc[1].buf = ps->tx_buf;
                for (a = 0, c = 0; a < ps->rx_desc_blen && c < sizeof (ps->tx_buf) - 1; ++a) {
                    b = snprintf(ps->tx_buf + c,
                            sizeof (ps->tx_buf) - c - 1,
                            "%02X ", (char) (ps->rx_desc_buf[a] & 0x00FF)
                            );
                    c += b;
                }
                ps->tx_desc[1].len = c;
            } else {
                ps->tx_desc[1].len = 7;
                ps->tx_desc[1].buf = "<None> ";
            }
            // Echo back the received packet as a hex dump.
            if (ignore_all == 0 && ignore_init == 0) {
                char received_char = ps->rx_desc_buf[0];
                if (received_char == 0x09) {
                    valid_input = 1;
                }
                if (received_char == '\033') {
                    // Escape sequence detected, could be an arrow key
                    if (ps->rx_desc_buf[1] == '[') {
                        switch (ps->rx_desc_buf[2]) {
                            case 'D': // Left arrow
                                TC0_REGS -> COUNT16.TC_COUNT = 0;
                                updateBlinkSetting(ps, false);
                                valid_input = 1;
                                break;
                            case 'C': // Right arrow
                                TC0_REGS -> COUNT16.TC_COUNT = 0;
                                updateBlinkSetting(ps, true);
                                valid_input = 1;
                                break;
                        }
                    }
                } else if (received_char == 0x61 || received_char == 0x41) {
                    TC0_REGS -> COUNT16.TC_COUNT = 0;
                    while (TC0_REGS -> COUNT16.TC_SYNCBUSY & (1 << 4));
                    valid_input = 1;
                    updateBlinkSetting(ps, false);
                } else if (received_char == 'D' || received_char == 'd') {
                    valid_input = 1;
                    TC0_REGS -> COUNT16.TC_COUNT = 0;
                    while (TC0_REGS -> COUNT16.TC_SYNCBUSY & (1 << 4));
                    updateBlinkSetting(ps, true);
                } else {
                    // Handle other inputs as before
                    ps->flags |= PROG_FLAG_UPDATE_PENDING;
                    ps->rx_desc_blen = ps->rx_desc.compl_info.data_len;
                }
            }
            ps->flags |= PROG_FLAG_UPDATE_PENDING;
            ps->rx_desc_blen = ps->rx_desc.compl_info.data_len;
            // Reset receive buffer and wait for completion
            while (platform_usart_cdc_tx_busy()) {
                platform_do_loop_one();
            }

            ps->rx_desc.compl_type = PLATFORM_USART_RX_COMPL_NONE;
            platform_usart_cdc_rx_async(&ps->rx_desc);

            ps->flags |= PROG_FLAG_GEN_COMPLETE;
            ps->rx_desc_blen = 0;
        }

        if (platform_usart_cdc_tx_async(&ps->tx_desc[0], 3)) {
            ps->rx_desc.compl_type = PLATFORM_USART_RX_COMPL_NONE;
            platform_usart_cdc_rx_async(&ps->rx_desc);
            ps->flags &= ~(PROG_FLAG_UPDATE_PENDING | PROG_FLAG_GEN_COMPLETE);
        }


    } while (0);

    // Done
    return;
}

// main() -- the heart of the program

int main(void) {
    prog_state_t ps;

    // Initialization time	
    prog_setup(&ps);

    /*
     * Microcontroller main()'s are supposed to never return (welp, they
     * have none to return to); hence the intentional infinite loop.
     */

    for (;;) {
        prog_loop_one(&ps);
    }

    // This line must never be reached
    return 1;
}