/* Host-only fake GPIO: the test drives host_gpio[] directly. */
#ifndef HOST_LIBBSP_H
#define HOST_LIBBSP_H
#define GPIO_MODE_IN 0
extern UINT host_gpio[32];
static inline ER gpio_set_pin(UINT no, UINT mode)
{
    (void)no; (void)mode; return E_OK;
}
static inline UINT gpio_get_val(UINT no) { return host_gpio[no]; }
#endif
