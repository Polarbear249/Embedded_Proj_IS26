/* Host-only RP2040 register layout. */
#define PWM_BASE 0x40050000U
#define PWM_CHx_CSR 0U
#define PWM_CHx_DIV 4U
#define PWM_CHx_CTR 8U
#define PWM_CHx_CC 12U
#define PWM_CHx_TOP 16U
/* Reset controller: motor_init() takes PWM out of reset. On the host
 * the fake reset controller reports every block as already out of
 * reset, and remembers which bits were cleared. */
#define RESETS_RESET 0x4000C000U
#define RESETS_RESET_DONE 0x4000C008U
#define RESETS_RESET_PWM 0x00004000U
static UW fake_reset_cleared;
static void clr_w(UW addr, UW bits) { (void)addr; fake_reset_cleared |= bits; }
static UW in_w(UW addr) { (void)addr; return 0xFFFFFFFFU; }
