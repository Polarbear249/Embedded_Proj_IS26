/* Robo Pico: left GP10/11, right GP8/9. No Pico SDK dependency. */
#include "motor.h"
#include <sys/machine.h>

#define S2_PWM_REG(slice, offset) (PWM_BASE + (slice) * 0x14U + (offset))

void motor_stop(void)
{
    /* Atomic pair writes avoid read/modify/write of the shared CC register. */
    out_w(S2_PWM_REG(4U, PWM_CHx_CC), 0U);
    out_w(S2_PWM_REG(5U, PWM_CHx_CC), 0U);
}

ER motor_init(void)
{
    UINT pin, slice;
    ER er;
    for (slice = 4U; slice <= 5U; ++slice) {
        out_w(S2_PWM_REG(slice, PWM_CHx_CSR), 0U);
        out_w(S2_PWM_REG(slice, PWM_CHx_CC), 0U);
        out_w(S2_PWM_REG(slice, PWM_CHx_CTR), 0U);
        /* 125 MHz / 125 / 1000 = 1 kHz, matching Buddy-2 bench tests. */
        out_w(S2_PWM_REG(slice, PWM_CHx_DIV), 125U << 4);
        out_w(S2_PWM_REG(slice, PWM_CHx_TOP), 999U);
    }
    for (pin = 8U; pin <= 11U; ++pin) {
        er = pwm_set_pin(pin);
        if (er < E_OK) { motor_stop(); return er; }
    }
    out_w(S2_PWM_REG(4U, PWM_CHx_CSR), 1U);
    out_w(S2_PWM_REG(5U, PWM_CHx_CSR), 1U);
    return E_OK;
}

ER motor_apply(INT left, INT right)
{
    UW l, r;
    if (left < -200 || left > 200 || right < -200 || right > 200) {
        motor_stop();
        return E_PAR;
    }
    /* Positive = vehicle forward: GP10 left, GP9 right. */
    l = left >= 0 ? (UW)left : ((UW)(-left) << 16);
    r = right >= 0 ? ((UW)right << 16) : (UW)(-right);
    out_w(S2_PWM_REG(5U, PWM_CHx_CC), l);
    out_w(S2_PWM_REG(4U, PWM_CHx_CC), r);
    return E_OK;
}
