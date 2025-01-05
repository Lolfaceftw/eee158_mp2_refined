#ifndef CLK_H
#define CLK_H

// Enable higher frequencies for higher performance

void raise_perf_level(void) {
    uint32_t tmp_reg = 0;

    /*
     * The chip starts in PL0, which emphasizes energy efficiency over
     * performance. However, we need the latter for the clock frequency
     * we will be using (~24 MHz); hence, switch to PL2 before continuing.
     */
    PM_REGS->PM_INTFLAG = 0x01;
    PM_REGS->PM_PLCFG = 0x02;
    while ((PM_REGS->PM_INTFLAG & 0x01) == 0)
        asm("nop");
    PM_REGS->PM_INTFLAG = 0x01;

    /*
     * Power up the 48MHz DFPLL.
     * 
     * On the Curiosity Nano Board, VDDPLL has a 1.1uF capacitance
     * connected in parallel. Assuming a ~20% error, we have
     * STARTUP >= (1.32uF)/(1uF) = 1.32; as this is not an integer, choose
     * the next HIGHER value.
     */
    NVMCTRL_SEC_REGS->NVMCTRL_CTRLB = (2 << 1);
    SUPC_REGS->SUPC_VREGPLL = 0x00000302;
    while ((SUPC_REGS->SUPC_STATUS & (1 << 18)) == 0)
        asm("nop");

    /*
     * Configure the 48MHz DFPLL.
     * 
     * Start with disabling ONDEMAND...
     */
    OSCCTRL_REGS->OSCCTRL_DFLLCTRL = 0x0000;
    while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
        asm("nop");

    /*
     * ... then writing the calibration values (which MUST be done as a
     * single write, hence the use of a temporary variable)...
     */
    tmp_reg = *((uint32_t*) 0x00806020);
    tmp_reg &= ((uint32_t) (0b111111) << 25);
    tmp_reg >>= 15;
    tmp_reg |= ((512 << 0) & 0x000003ff);
    OSCCTRL_REGS->OSCCTRL_DFLLVAL = tmp_reg;
    while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
        asm("nop");

    // ... then enabling ...
    OSCCTRL_REGS->OSCCTRL_DFLLCTRL |= 0x0002;
    while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
        asm("nop");

    // ... then restoring ONDEMAND.
    //	OSCCTRL_REGS->OSCCTRL_DFLLCTRL |= 0x0080;
    //	while ((OSCCTRL_REGS->OSCCTRL_STATUS & (1 << 24)) == 0)
    //		asm("nop");

    /*
     * Configure GCLK_GEN2 as described; this one will become the main
     * clock for slow/medium-speed peripherals, as GCLK_GEN0 will be
     * stepped up for 24 MHz operation.
     */
    GCLK_REGS->GCLK_GENCTRL[2] = 0x00000105;
    while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 4)) != 0)
        asm("nop");

    // Switch over GCLK_GEN0 to DFLL48M, with DIV=2 to get 24 MHz.
    GCLK_REGS->GCLK_GENCTRL[0] = 0x00020107;
    while ((GCLK_REGS->GCLK_SYNCBUSY & (1 << 2)) != 0)
        asm("nop");

    // Done. We're now at 24 MHz.
    return;
}

void TC0_Init(void) {
    // Enable the TC0 Bus Clock
    GCLK_REGS -> GCLK_PCHCTRL[23] = (1 << 6); // Bit 6 Enable
    while ((GCLK_REGS -> GCLK_PCHCTRL [23] * (1 << 6)) == 0);

    // Setting up the TC 0 -> CTRLA Register
    TC0_REGS -> COUNT16.TC_CTRLA = (1); // Software Reset; Bit 0
    while (TC0_REGS -> COUNT16.TC_SYNCBUSY & (1));
    TC0_REGS -> COUNT16.TC_CC[0] = 23438*2;
    while (TC0_REGS -> COUNT16.TC_SYNCBUSY & (1 << 6));
    TC0_REGS -> COUNT16.TC_CTRLA = (0x0 << 2); // Set to 16 bit mode; Bit[3:2].
    TC0_REGS -> COUNT16.TC_CTRLA = (0x1 << 4); // Reset counter on next prescaler clock Bit[5:4]]
    TC0_REGS -> COUNT16.TC_CTRLA = (0x7 << 8); // Prescaler Factor: 1024 Bit[10:8]]

    // Setting up the WAVE Register
    TC0_REGS -> COUNT16.TC_WAVE = (0x1 << 0); // Use MFRQ Bit [1:0]

    TC0_REGS -> COUNT16.TC_CTRLA |= (1 << 1); // Enable TC0 Peripheral Bit 1
}

void TCC3_Init(void) {
    GCLK_REGS -> GCLK_PCHCTRL[27] |= (1 << 6);
    while ((GCLK_REGS -> GCLK_PCHCTRL [27] & (1 << 6)) == 0); // Wait for synchronization

    /* Reset TCC */
    TCC3_REGS->TCC_CTRLA |= 0x01; // Set SWRST bit to 1 to reset
    while (TCC3_REGS->TCC_SYNCBUSY & ~(1 << 0)); // Wait for synchronization

    /* Clock Prescaler and Mode */
    TCC3_REGS->TCC_CTRLA |= (1 << 12) | (7 << 8); // Precsync = PRESC | Prescaler = 1024
    while (TCC3_REGS->TCC_SYNCBUSY & ~(1 << 0)); // Wait for synchronization

    TCC3_REGS->TCC_WEXCTRL |= TCC_WEXCTRL_OTMX(0UL); // Default configuration
    TCC3_REGS->TCC_WAVE |= (2 << 0) | (0 << 4) | (1 << 17) | (1 << 16) | (1 << 19); // 0x2 NPWM Normal PWM PER TOP/Zero or Single slope PWM
    while (TCC3_REGS->TCC_SYNCBUSY & ~(1 << 6)); // Wait for synchronization
    // RAMP 1 operation (Polarity 1) - bit 16, set at CCx, clear at TOP 

    /* Configure duty cycle values */

    TCC3_REGS->TCC_PER = 93750; // 4 seconds
    while ((TCC3_REGS -> TCC_SYNCBUSY) & (1 << 7));
    /* TCC enable */
    TCC3_REGS->TCC_CTRLA |= (1 << 1); // Enables TCC
    while (TCC3_REGS->TCC_SYNCBUSY & ~(1 << 0)); // Wait for synchronization
}

void TCC0_Init(void) {
    GCLK_REGS -> GCLK_PCHCTRL[25] |= (1 << 6);
    while ((GCLK_REGS -> GCLK_PCHCTRL [25] & (1 << 6)) == 0); // Wait for synchronization

    /* Reset TCC */
    TCC0_REGS->TCC_CTRLA |= 0x01; // Set SWRST bit to 1 to reset
    while (TCC0_REGS->TCC_SYNCBUSY & ~(1 << 0)); // Wait for synchronization

    /* Clock Prescaler and Mode */
    TCC0_REGS->TCC_CTRLA |= (1 << 12) | (7 << 8); // Precsync = PRESC | Prescaler = 1024
    while (TCC0_REGS->TCC_SYNCBUSY & ~(1 << 0)); // Wait for synchronization

    TCC0_REGS->TCC_WEXCTRL |= TCC_WEXCTRL_OTMX(0UL); // Default configuration
    TCC0_REGS->TCC_WAVE |= (2 << 0) | (0 << 4) | (1 << 17) | (1 << 16) | (1 << 19); // 0x2 NPWM Normal PWM PER TOP/Zero or Single slope PWM
    while (TCC0_REGS->TCC_SYNCBUSY & ~(1 << 6)); // Wait for synchronization
    // RAMP 1 operation (Polarity 1) - bit 16, set at CCx, clear at TOP 

    /* Configure duty cycle values */

    TCC0_REGS->TCC_PER = 1054688; // 45 seconds
    //TCC0_REGS->TCC_PER = 70313; // 3 seconds
    while ((TCC0_REGS -> TCC_SYNCBUSY) & (1 << 7));
    /* TCC enable */
    TCC0_REGS->TCC_CTRLA |= (1 << 1); // Enables TCC
    while (TCC0_REGS->TCC_SYNCBUSY & ~(1 << 0)); // Wait for synchronization
}
#endif CLK_H