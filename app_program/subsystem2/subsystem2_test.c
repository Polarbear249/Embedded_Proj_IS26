#include "subsystem2.h"
#include "../test_report.h"
#if APP_TEST
EXPORT void subsystem2_test(void)
{
    TEST_INIT();
    TEST_CHECK("S2-U01", motion_set_duty(201, 0, 100) == E_PAR,
               "reject out-of-range duty without moving wheels");
    TEST_CHECK("S2-U02", motion_set_duty(0, 0, 0) == E_PAR,
               "reject zero command lease");
    TEST_SUMMARY("subsystem2");
}
#endif
