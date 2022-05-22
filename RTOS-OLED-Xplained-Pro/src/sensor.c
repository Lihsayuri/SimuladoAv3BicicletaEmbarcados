/*
 * sensor.c
 *
 * Created: 12/06/2021 12:13:28
 *  Author: RafaelCF4
 */ 

#include "sensor.h"

static void pin_toggle(Pio *pio, uint32_t mask);

void TC3_Handler(void) {
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Pulso do sensor */
	pio_set(SENSOR_PIO, SENSOR_MASK);
  delay_us(1);
	pio_clear(SENSOR_PIO, SENSOR_MASK);  
}

static void pin_toggle(Pio *pio, uint32_t mask){
  if(pio_get_output_data_status(pio, mask))
  pio_clear(pio, mask);
  else
  pio_set(pio,mask);
}

static void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
  NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void patinete_power(int power) {
  switch (power) {
  case 0:
    tc_stop(TC1, 0);
    break;
  case 1:
    tc_start(TC1, 0);
    TC_init(TC1, ID_TC3, 0, 1);
    break;
  case 2:
    tc_start(TC1, 0);
    TC_init(TC1, ID_TC3, 0, 3);
    break;
  case 3:
    tc_start(TC1, 0);
    TC_init(TC1, ID_TC3, 0, 5);
    break;
  default:
    break;
  }

}

void init_sensor(void) {
  pmc_enable_periph_clk(SENSOR_ID);
  pio_configure(SENSOR_PIO, PIO_OUTPUT_0, SENSOR_MASK, PIO_DEFAULT);
  
  patinete_power(0);
}
