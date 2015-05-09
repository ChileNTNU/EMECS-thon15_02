
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"

#include "motor.h"

/* Define PWM frequency value */
//Real freq 50Hz (20ms)
//Top compare value 14 000 000 / 3205 = 4368
#define PWM_FREQ 3205

//Constants for moving the wheels
/*
#define FORWD_VALUE   262 //1.2ms high
#define BACKWRD_VALUE 393 //1,8ms
#define STEADY_VALUE  330 //1.5ms high
 */
static uint32_t forward_value;
static uint32_t backward_value;
static uint32_t steady_value;

volatile uint32_t value_right_wheel;
volatile uint32_t value_left_wheel;

void Move_Right(void){
	value_right_wheel = forward_value;
	value_left_wheel = forward_value;
}

void Move_Left(void){
	value_right_wheel = backward_value;
	value_left_wheel = backward_value;
}

void Move_Forward(void){
	value_right_wheel = forward_value;
	value_left_wheel = backward_value;
}

void Move_Backward(void){
	value_right_wheel = backward_value;
	value_left_wheel = forward_value;
}

void Stop_Robot(void){
	value_right_wheel = steady_value;
	value_left_wheel = steady_value;
}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);

  TIMER_CompareBufSet(TIMER0, 1, value_left_wheel);
  TIMER_CompareBufSet(TIMER0, 2, value_right_wheel);

}
/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int motor_init(void)
{
  uint32_t top_value = 0;

  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);

  /* Set CC1 location 3 pin (PD2) as output */
  //Left wheel
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
  /* Set CC2 location 3 pin (PD3) as output */
  //Right wheel
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);

  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit =
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionNone,
    .cmoa       = timerOutputActionToggle,
    .mode       = timerCCModePWM,
    .filter     = false,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,
  };

  /* Configure CC channel 0 */
  TIMER_InitCC(TIMER0, 1, &timerCCInit);
  TIMER_InitCC(TIMER0, 2, &timerCCInit);

  /* Route CC0 to location 3 (PD1) and enable pin */
  TIMER0->ROUTE |= (TIMER_ROUTE_CC2PEN | TIMER_ROUTE_CC1PEN | TIMER_ROUTE_LOCATION_LOC3);

  /* Set Top Value */
  TIMER_TopSet(TIMER0, CMU_ClockFreqGet(cmuClock_HFPER)/PWM_FREQ);

  top_value = CMU_ClockFreqGet(cmuClock_HFPER)/PWM_FREQ;

  forward_value = ((18*top_value)/200);
  backward_value = ((12*top_value)/200);
  steady_value = ((15*top_value)/200);

  /* Set compare value starting at 0 - it will be incremented in the interrupt handler */
  TIMER_CompareBufSet(TIMER0, 1, 0);
  TIMER_CompareBufSet(TIMER0, 2, 0);

  /* Select timer parameters */
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,
    .debugRun   = true,
    .prescale   = timerPrescale64,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };

  /* Enable overflow interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);

  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);

  /* Configure timer */
  TIMER_Init(TIMER0, &timerInit);

  return 0;
}

