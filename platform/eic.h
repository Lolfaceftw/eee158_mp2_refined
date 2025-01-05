#ifndef EIC_H
#define EIC_H
extern ignore_all;
extern toggle;
extern reset_timer;
extern e_stop;

/*
 * Configure the EIC peripheral
 * 
 * NOTE: EIC initialization is split into "early" and "late" halves. This is
 *       because most settings within the peripheral cannot be modified while
 *       EIC is enabled.
 */
void EIC_init_early(void) {
    /*
     * Enable the APB clock for this peripheral
     * 
     * NOTE: The chip resets with it enabled; hence, commented-out.
     * 
     * WARNING: Incorrect MCLK settings can cause system lockup that can
     *          only be rectified via a hardware reset/power-cycle.
     */
    // MCLK_REGS->MCLK_APBAMASK |= (1 << 10);

    /*
     * In order for debouncing to work, GCLK_EIC needs to be configured.
     * We can pluck this off GCLK_GEN2, configured for 4 MHz; then, for
     * mechanical inputs we slow it down to around 15.625 kHz. This
     * prescaling is OK for such inputs since debouncing is only employed
     * on inputs connected to mechanical switches, not on those coming from
     * other (electronic) circuits.
     * 
     * GCLK_EIC is at index 4; and Generator 2 is used.
     */
    GCLK_REGS->GCLK_PCHCTRL[4] = 0x00000042;
    while ((GCLK_REGS->GCLK_PCHCTRL[4] & 0x00000042) == 0)
        asm("nop");

    // Reset, and wait for said operation to complete.
    EIC_SEC_REGS->EIC_CTRLA = 0x01;
    while ((EIC_SEC_REGS->EIC_SYNCBUSY & 0x01) != 0)
        asm("nop");

    /*
     * Just set the debounce prescaler for now, and leave the EIC disabled.
     * This is because most settings are not editable while the peripheral
     * is enabled.
     */
    EIC_SEC_REGS->EIC_DPRESCALER = (0b0 << 16) | (0b0000 << 4) |
            (0b1111 << 0);
    return;
}

void EIC_init_late(void) {
    /*
     * Enable the peripheral.
     * 
     * Once the peripheral is enabled, further configuration is almost
     * impossible.
     */
    EIC_SEC_REGS->EIC_CTRLA |= 0x02;
    while ((EIC_SEC_REGS->EIC_SYNCBUSY & 0x02) != 0)
        asm("nop");
    return;
}

void PB_init(void) {
    /*
     * Configure PA23.
     * 
     * NOTE: PORT I/O configuration is never separable from the in-circuit
     *       wiring. Refer to the top of this source file for each PORT
     *       pin assignments.
     */
    /*
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = 0x00800000;
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[23] = 0x03;
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[(23 >> 1)] &= ~(0xF0);*/
    /*PA 23*/

    // 31.7.1
    PORT_SEC_REGS->GROUP[0].PORT_DIRCLR |= (1 << 23); // Set as input.
    // 31.7.14
    PORT_SEC_REGS->GROUP[0].PORT_PINCFG[23] |= 0x7; // Enables PULLEN, INEN, and PMUXEN, input with pull.
    // 31.7.6
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET |= (1 << 0); // Set as internal pull-up.
    // 31.7.13
    PORT_SEC_REGS->GROUP[0].PORT_PMUX[11] |= (0x0 << 4); // A Peripheral for PA23, PMUXO[3:0], required PMUXEN 1
    /*
     * Debounce EIC_EXT2, where PA23 is.
     * 
     * Configure the line for edge-detection only.
     * 
     * NOTE: EIC has been reset and pre-configured by the time this
     *       function is called.
     */
    EIC_SEC_REGS->EIC_DEBOUNCEN |= (1 << 2);
    EIC_SEC_REGS->EIC_CONFIG0 &= ~((uint32_t) (0xF) << 8);
    EIC_SEC_REGS->EIC_CONFIG0 |= ((uint32_t) (0xB) << 8);
    EIC_SEC_REGS -> EIC_CONFIG0 |= (0x3 << 28); // Both Edge Detection
    EIC_SEC_REGS -> EIC_CONFIG0 |= (1 << 31); // Filter Enabled
        EIC_SEC_REGS -> EIC_DEBOUNCEN |= (1 << 7); // Debounce Enabled
    EIC_SEC_REGS -> EIC_INTENSET |= (1 << 7); // Enable EIC
    /*
     * NOTE: Even though interrupts are enabled here, global interrupts
     *       still need to be enabled via NVIC.
     */
    EIC_SEC_REGS->EIC_INTENSET |= (1 << 2);
    return;
}

void PA18_EIC_Init(void){
    

}

/*
 * Configure the NVIC
 * 
 * This must be called last, because interrupts are enabled as soon as
 * execution returns from this function.
 */
void NVIC_init(void) {
    /*
     * Unlike AHB/APB peripherals, the NVIC is part of the Arm v8-M
     * architecture core proper. Hence, it is always enabled.
     */
    __DMB();
    __enable_irq();
    NVIC_SetPriority(EIC_EXTINT_2_IRQn, 3);
    NVIC_SetPriority(EIC_EXTINT_7_IRQn, 3);
    NVIC_SetPriority(SysTick_IRQn, 3);
    NVIC_EnableIRQ(EIC_EXTINT_2_IRQn);
    NVIC_EnableIRQ(EIC_EXTINT_7_IRQn);
    NVIC_EnableIRQ(SysTick_IRQn);
    return;
}

void __attribute__((interrupt())) EIC_EXTINT_7_Handler(void) {
    EIC_SEC_REGS->EIC_INTFLAG |= (1 << 7);
    asm("nop");
    if (toggle == 0){
        // Ignore all commands
        ignore_all = 1;
        reset_timer = 1;
        PORT_SEC_REGS -> GROUP[0].PORT_OUTCLR = (1 << 1); // Turn PA 01 Off
        toggle = 1;
        e_stop = 1;
    } else if (toggle == 1) {
        // Wait 4 seconds
        reset_timer = 0;
        // Wait onboard push button
        PORT_SEC_REGS -> GROUP[0].PORT_OUTSET = (1 << 1); // Turn PA 01 Off
        //ignore_all = 0;
        toggle = 0;
    }
    
}

#endif EIC_H