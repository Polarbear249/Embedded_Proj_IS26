/* Exercise the production C against fake kernel time and PWM registers. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#define APP_H
#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_OBJ -41
#define E_PAR -17
#define E_TMOUT -50
#define TMO_POL 0
#define TA_INHERIT 3
#define IMPORT extern
typedef int ER, ID, INT, BOOL;
typedef unsigned UINT;
typedef uint32_t UW;
typedef struct { int hi; UW lo; } SYSTIM;
typedef struct { int mtxatr; } T_CMTX;
static UW clock_ms, regs[40];
static int busy;
static void out_w(UW addr, UW value) { regs[(addr-0x40050000U)/4] = value; }
static ER pwm_set_pin(UINT pin) { assert(pin >= 8 && pin <= 11); return E_OK; }
static ID tk_cre_mtx(T_CMTX *cfg) { (void)cfg; return 1; }
static ER tk_loc_mtx(ID id, INT timeout) {
    (void)id; (void)timeout; return busy ? E_TMOUT : E_OK;
}
static ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
static ER tk_get_tim(SYSTIM *now) { now->lo=clock_ms; return E_OK; }
static ER tk_dly_tsk(INT ms) { (void)ms; return E_OK; }
#include "../../../app_program/subsystem2/motor.c"
#include "../../../app_program/subsystem2/motion_control.c"
#define LEFT regs[(5*20+12)/4]
#define RIGHT regs[(4*20+12)/4]
int main(void)
{
    MOTION_STATUS status;
    assert(motion_stop()==E_OBJ);
    assert(motion_init()==E_OK);
    assert(LEFT==0 && RIGHT==0);
    assert(regs[(4*20+4)/4]==(125U<<4));
    assert(motion_set_duty(201,0,100)==E_PAR);
    assert(motion_set_duty(0,-201,100)==E_PAR);
    assert(motion_set_duty(100,100,0)==E_PAR);
    assert(motion_set_duty(100,100,1001)==E_PAR);
    assert(motion_set_duty(150,170,300)==E_OK);
    motion_tick(); assert(LEFT==150 && RIGHT==(170U<<16));
    assert(motion_set_duty(-150,-170,300)==E_OK);
    motion_tick(); assert(LEFT==0 && RIGHT==0);
    clock_ms+=10; motion_tick(); assert(LEFT==(150U<<16) && RIGHT==170);
    assert(motion_stop()==E_OK); motion_tick(); assert(LEFT==0 && RIGHT==0);
    assert(motion_set_duty(150,150,100)==E_OK);
    motion_tick(); clock_ms+=100; motion_tick(); assert(LEFT==0 && RIGHT==0);
    assert(motion_get_status(&status)==E_OK && status.expired);
    assert(motion_estop()==E_OK);
    assert(motion_set_duty(150,150,100)==E_OBJ);
    motion_tick(); assert(LEFT==0 && RIGHT==0);
    assert(motion_clear_estop()==E_OK); motion_tick(); assert(LEFT==0 && RIGHT==0);
    clock_ms=UINT32_MAX-50;
    assert(motion_set_duty(100,-100,100)==E_OK);
    motion_tick(); assert(LEFT==100 && RIGHT==100);
    clock_ms=49; motion_tick(); assert(LEFT==0 && RIGHT==0);
    assert(motion_set_duty(100,100,100)==E_OK); motion_tick();
    busy=1; assert(motion_stop()==E_TMOUT); motion_tick(); assert(LEFT==0 && RIGHT==0);
    busy=0;
    assert(motor_apply(999,0)==E_PAR); assert(LEFT==0 && RIGHT==0);
    puts("PASS: startup, limits, mapping, reverse gap, stop, expiry, estop, clock wrap, mutex contention");
}
