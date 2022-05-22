#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "sensor.h"

#define PI 3.142857
#define RAIO 0.2

/* Botao da placa */
#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_IDX 0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_IDX 30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_IDX 2
#define LED_3_IDX_MASK (1 << LED_3_IDX)

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO      PIOC
#define BUT_2_PIO_ID   ID_PIOC
#define BUT_2_IDX      31
#define BUT_2_IDX_MASK (1 << BUT_2_IDX)
// Botao 3
#define BUT_3_PIO      PIOA
#define BUT_3_PIO_ID   ID_PIOA
#define BUT_3_IDX      19
#define BUT_3_IDX_MASK (1 << BUT_3_IDX)

/* Pino PA21 */
#define PINO21_PIO PIOA
#define PINO21_PIO_ID ID_PIOA
#define PINO21_IDX 21
#define PINO21_IDX_MASK (1 << PINO21_IDX)

/** RTOS  */
#define TASK_MAIN_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MAIN_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void io_init(void);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);

/** globals */
QueueHandle_t xQueueBut;
QueueHandle_t xQueuedT;


volatile int power = 0;
volatile int g_tc_counter = 0;
double dist = 0;
volatile int tempo = 0;
volatile int i = 0;
/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/



void but1_callback(void) {
	int but = 1;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBut, &but, xHigherPriorityTaskWoken);
}

void but2_callback(void) {
	int but = 2;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBut, &but, xHigherPriorityTaskWoken);
	
}

void but3_callback(void) {
	int but = 3;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(xQueueBut, &but, xHigherPriorityTaskWoken);
}

//void pino_21_callback(void){
	//int dt = g_tc_counter;
	//g_tc_counter = 0;
	//BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	//xQueueSendFromISR(xQueueBut, &dt, &xHigherPriorityTaskWoken);
	//
//}

void pino_21_callback(void) {
	int dt = g_tc_counter;
	g_tc_counter = 0;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueuedT, &dt, &xHigherPriorityTaskWoken);
	
}

void update_power(int power){
	patinete_power(power);
	if (power == 0){
		pio_set(LED_1_PIO, LED_1_IDX_MASK);
		pio_set(LED_2_PIO, LED_2_IDX_MASK);
		pio_set(LED_3_PIO, LED_3_IDX_MASK);
	} else if (power == 1){
		pio_clear(LED_1_PIO, LED_1_IDX_MASK);
		pio_set(LED_2_PIO, LED_2_IDX_MASK);
		pio_set(LED_3_PIO, LED_3_IDX_MASK);
	} else if (power == 2){
		pio_clear(LED_1_PIO, LED_1_IDX_MASK);
		pio_clear(LED_2_PIO, LED_2_IDX_MASK);
		pio_set(LED_3_PIO, LED_3_IDX_MASK);		
	} else if (power == 3){
		pio_clear(LED_1_PIO, LED_1_IDX_MASK);
		pio_clear(LED_2_PIO, LED_2_IDX_MASK);
		pio_clear(LED_3_PIO, LED_3_IDX_MASK);
	}
	
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_main(void *pvParameters) {
	gfx_mono_ssd1306_init();
	//gfx_mono_draw_string("Super patinete", 0, 0, &sysfont);
	//gfx_mono_draw_string("20 km/h", 20, 20, &sysfont);
	
	init_sensor();
	io_init();
	int but_pressed;
	int dt;
	char string_dist[20];
	char string_vel[20];
	int contagem = 0;
	int desliga = 0;


	for (;;)  {
		if (!contagem){
			contagem = 1;
			tempo = 0;
			TC_init(TC0, ID_TC0, 0, 1);
			tc_start(TC0, 0);
		}
		
		if (tempo == 12 && !desliga){
			gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
			sprintf(string_vel, "Velocidade: %2.1f", 0.0);
			gfx_mono_draw_string(string_vel, 0, 0, &sysfont);
			sprintf(string_dist, "Distancia: %2.1f", 0.0);
			gfx_mono_draw_string(string_dist, 0, 20, &sysfont);
			power = 0;
			update_power(power);
		}

		
		if (xQueueReceive(xQueueBut, &but_pressed , 10)){
			desliga = 0;
			TC_init(TC0, ID_TC1, 1, 100);
			tc_start(TC0, 1);

			
			if (but_pressed == 1){ // aumenta potência
				power += 1;
				if (power > 3){
					power = 3;
				}
			} else if(but_pressed == 3){
				power -= 1;
				if (power < 0){
					power = 0;
				}
				
			
			} else if(but_pressed = 2){
				TC_init(TC1, ID_TC4, 1, 1);
				tc_start(TC1, 1);
				while (!pio_get(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK)){
					if (i == 10){
						gfx_mono_generic_draw_filled_rect(0, 0, 127, 31, GFX_PIXEL_CLR);
						desliga = 1;
					}
				}
			}
			
			
			if (!desliga){
				printf("O valor da potencia é de: %d\n", power);
				update_power(power);
				contagem = 0;				
			}
						
		} if (xQueueReceive(xQueuedT, &dt, 0)){
			double T = dt*0.01; // periodo em s
			double w = 2*PI/T;  // vel angular
			double v = RAIO*w*3.6; //vel linear em [km/h]
			
			dist += (v*T)/3.6; // tá em km, então divide por 3.6
			double distancia_km = dist;
			
			sprintf(string_vel, "Velocidade: %2.1f", v);
			gfx_mono_draw_string(string_vel, 0, 0, &sysfont);
			sprintf(string_dist, "Distancia: %2.1f", distancia_km);
			gfx_mono_draw_string(string_dist, 0, 20, &sysfont);
			contagem = 0;
		}
		

	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/


void configure_pio_input(Pio *pio, const pio_type_t ul_type, const uint32_t ul_mask, const uint32_t ul_attribute, uint32_t ul_id){
	pmc_enable_periph_clk(ul_id);
	pio_configure(pio, ul_type, ul_mask, ul_attribute);
	pio_set_debounce_filter(pio, ul_mask, 60);
}

void configure_interruption(Pio *pio, uint32_t ul_id, const uint32_t ul_mask,  uint32_t ul_attr, void (*p_handler) (uint32_t, uint32_t), uint32_t priority){
	pio_handler_set(pio, ul_id, ul_mask , ul_attr, p_handler);
	pio_enable_interrupt(pio, ul_mask);
	pio_get_interrupt_status(pio);
	NVIC_EnableIRQ(ul_id);
	NVIC_SetPriority(ul_id, priority);
}


void io_init(void) {
	pmc_enable_periph_clk(LED_1_PIO_ID);
	pmc_enable_periph_clk(LED_2_PIO_ID);
	pmc_enable_periph_clk(LED_3_PIO_ID);

	//pio_configure(LED_1_PIO, PIO_OUTPUT_0, LED_1_IDX_MASK, PIO_DEFAULT);
	//pio_configure(LED_2_PIO, PIO_OUTPUT_0, LED_2_IDX_MASK, PIO_DEFAULT);
	//pio_configure(LED_3_PIO, PIO_OUTPUT_0, LED_3_IDX_MASK, PIO_DEFAULT);
	pio_set_output(LED_1_PIO, LED_1_IDX_MASK, 1, 0, 0);
	pio_set_output(LED_2_PIO, LED_2_IDX_MASK, 1, 0, 0);
	pio_set_output(LED_3_PIO, LED_3_IDX_MASK, 1, 0, 0);
	
	configure_pio_input(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP|PIO_DEBOUNCE, BUT_1_PIO_ID);
	configure_interruption(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE, but1_callback, 4);
	
	configure_pio_input(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP|PIO_DEBOUNCE, BUT_3_PIO_ID);
	configure_interruption(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE, but3_callback, 4);
	
	configure_pio_input(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP|PIO_DEBOUNCE, BUT_2_PIO_ID);
	configure_interruption(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE, but2_callback, 4);
	
	//configure_pio_input(PINO21_PIO, PIO_INPUT, PINO21_IDX_MASK, PIO_PULLUP|PIO_DEBOUNCE, PINO21_PIO_ID);
	//configure_interruption(PINO21_PIO, PINO21_PIO_ID, PINO21_IDX_MASK, PIO_IT_RISE_EDGE, pino_21_callback, 4);
	
	pmc_enable_periph_clk(PINO21_PIO_ID);
    pio_set_input(PINO21_PIO,PINO21_IDX_MASK,PIO_DEFAULT);
    pio_handler_set(PINO21_PIO, PINO21_PIO_ID, PINO21_IDX_MASK, PIO_IT_RISE_EDGE, pino_21_callback);
	pio_enable_interrupt(PINO21_PIO, PINO21_IDX_MASK);
	pio_get_interrupt_status(PINO21_PIO);
    NVIC_EnableIRQ(PINO21_PIO_ID);
    NVIC_SetPriority(PINO21_PIO_ID, 4);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}


void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
	g_tc_counter++;
}

void TC0_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 0);

	/** Muda o estado do LED (pisca) **/
	tempo++;
}

void TC4_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 1);
	i++;
	
}
/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	
	xQueueBut = xQueueCreate(100, sizeof(int));
	if (xQueueBut == NULL)
		printf("falha em criar a queue xQueueBut \n");
		
	xQueuedT = xQueueCreate(1, sizeof(int));
	if (xQueuedT == NULL)
		printf("falha em criar a queue xQueuedT \n");

	/* Create task to control oled */
	if (xTaskCreate(task_main, "main", TASK_MAIN_STACK_SIZE, NULL, TASK_MAIN_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create main task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}