#include "debug/led.h"
#include "thread_init.h"

#include <cmsis_os.h>


#define LED_TIM			htim8
#define LED_MAX			(0xfff)


#define LED_TK_BREAK   100  // [ms] off time in between thread
#define LED_TK_ON      300  // [ms] on time of each thread
#define LED_LOOP_BREAK 500  // [ms] time in between each start of thread sequence
#define LED_INIT_DELAY 1000 // [ms] initial time to display a solid color

#define MAX_N_THREADS 32

volatile int n_threads = 0;
volatile uint16_t r_list[MAX_N_THREADS] = { 0 };
volatile uint16_t g_list[MAX_N_THREADS] = { 0 };
volatile uint16_t b_list[MAX_N_THREADS] = { 0 };

void TK_led_handler(void const *arg)
{
	led_init();

	while (1) {
		for (int i = 0; i < n_threads; i++) {
			led_set_rgb(r_list[i], g_list[i], b_list[i]);
			osDelay(LED_TK_ON);
			led_set_rgb(0, 0, 0);
			osDelay(LED_TK_BREAK);
		}

		osDelay(LED_LOOP_BREAK);
	}
}

// return ID if successfull, -1 otherwise
uint8_t led_register_TK()
{
	uint8_t val = 0;

	portDISABLE_INTERRUPTS();

	if (n_threads < MAX_N_THREADS) {
		val = n_threads++;
	} else {
		return -1;
	}

	portENABLE_INTERRUPTS();

	return val;
}


void led_set_TK_rgb(int tk_id, uint16_t r, uint16_t g, uint16_t b)
{
	if (tk_id >= 0 && tk_id < n_threads) {
		r_list[tk_id] = r;
		g_list[tk_id] = g;
		b_list[tk_id] = b;
	}
}


void led_set_rgb(uint16_t r, uint16_t g, uint16_t b)
{
	LED_TIM.Instance->CCR1 = r;
	LED_TIM.Instance->CCR2 = g;
	LED_TIM.Instance->CCR3 = b;
}

void led_set_r(uint16_t r)
{
	LED_TIM.Instance->CCR1 = r;
}

void led_set_g(uint16_t g)
{
	LED_TIM.Instance->CCR2 = g;
}

void led_set_b(uint16_t b)
{
	LED_TIM.Instance->CCR3 = b;
}

void led_init()
{

	LED_TIM.Instance->ARR = LED_MAX;
	HAL_TIMEx_PWMN_Start(&LED_TIM, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&LED_TIM, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&LED_TIM, TIM_CHANNEL_3);
	led_set_rgb(0, 0, 0);
}
