/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <stdlib.h>
#include <string.h>
#include "conf_board.h"
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* IOS                                                                  */
/************************************************************************/

#define BTN_PIO PIOA
#define BTN_PIO_ID ID_PIOA
#define BTN_PIO_PIN 11
#define BTN_PIO_PIN_MASK (1 << BTN_PIO_PIN)

#define PINO_BUZ_PIO PIOC
#define PINO_BUZ_PIO_ID ID_PIOC
#define PINO_BUZ_PIO_IDX 19
#define PINO_BUZ_PIO_IDX_MASK (1 << PINO_BUZ_PIO_IDX)

#define NOTE_E6  1319
#define NOTE_B5  988




/************************************************************************/
/* prototypes and types                                                 */
/************************************************************************/

void btn_init(void);
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
int tone(int freq, int duracao);

/************************************************************************/
/* rtos vars                                                            */
/************************************************************************/

QueueHandle_t xQueueCoins;
SemaphoreHandle_t xBtnSemaphore;

volatile int primeira_vez = 1;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

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

void but_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xBtnSemaphore, &xHigherPriorityTaskWoken);
}


/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_coins(void *pvParameters){
	btn_init();
	RTT_init(1000,0,0);
	int time;
	int coins;
	
	for(;;){

		
		if ((xSemaphoreTake(xBtnSemaphore, 1000))& (primeira_vez)) { //Clicou o botao pela primeira vez
			int ticks = rtt_read_timer_value(RTT);
			time = ticks/1000;
			srand(time);
			
			printf(" \ntime: %d \n", time);
			primeira_vez = 0;
			coins = (rand() % 3) + 1;
			xQueueSend(xQueueCoins, &coins, 0);
			
			
			
		}
		if ((xSemaphoreTake(xBtnSemaphore, 1000))& (!primeira_vez)) { //Clicou o botao pela segunda vez ou mais
			coins = (rand() % 3) + 1;
			xQueueSend(xQueueCoins, &coins, 0);
			
		}

		
		
		
		
	}
}

static void task_play(void *pvParameters){
	btn_init();
	int coins;
	
	
	for(;;){
		
		if (xQueueReceive(xQueueCoins, &(coins), 1000)) {
			printf("Coins: %d \n",coins);
			
			if (coins == 1){
				tone(NOTE_B5,  80);
				tone(NOTE_E6, 640);

			}
			if (coins == 2){
				tone(NOTE_B5,  80);
				tone(NOTE_E6, 640);
				tone(NOTE_B5,  80);
				tone(NOTE_E6, 640);

			}
			if (coins == 3){
				tone(NOTE_B5,  80);
				tone(NOTE_E6, 640);
				tone(NOTE_B5,  80);
				tone(NOTE_E6, 640);
				tone(NOTE_B5,  80);
				tone(NOTE_E6, 640);

			}
			
			
		}
		
	}
}

static void task_debug(void *pvParameters) {
	gfx_mono_ssd1306_init();

	for (;;) {
		gfx_mono_draw_filled_circle(10,10,4,1,GFX_WHOLE);
		vTaskDelay(150);
		gfx_mono_draw_filled_circle(10,10,4,0,GFX_WHOLE);
		vTaskDelay(150);

	}
}



/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

int tone(int freq, int duracao){
	// Recebendo uma frequencia em hz (rotações/ segundo)
	// Recebendo um tempo de toque
	// f = 1/T
	int T = 0;
	int N = 0;
	
	if (freq == 0){
		T = 0;
		N = 0;
	}
	else{
		
		T = 1000000/freq;
		
		N = freq*duracao/1000;
	}

	
	for (int i = 0;i<N;i++){
		pio_set(PINO_BUZ_PIO,PINO_BUZ_PIO_IDX_MASK);
		delay_us(T/2);
		pio_clear(PINO_BUZ_PIO,PINO_BUZ_PIO_IDX_MASK);
		delay_us(T/2);
		
	}
}

void btn_init(void) {
	
	pmc_enable_periph_clk(PINO_BUZ_PIO_ID);
	pio_set_output(PINO_BUZ_PIO, PINO_BUZ_PIO_IDX_MASK, 0, 0, 0);
	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BTN_PIO_ID);

	// Configura PIO para lidar com o pino do botão como entrada
	// com pull-up
	pio_configure(BTN_PIO, PIO_INPUT, BTN_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BTN_PIO, BTN_PIO_PIN_MASK, 60);
	
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BTN_PIO,
	BTN_PIO_ID,
	BTN_PIO_PIN_MASK,
	PIO_IT_FALL_EDGE,
	but_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BTN_PIO, BTN_PIO_PIN_MASK);
	pio_get_interrupt_status(BTN_PIO);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BTN_PIO_ID);
	NVIC_SetPriority(BTN_PIO_ID, 4); // Prioridade 4
}


void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int)(((float)32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT))
		;
		rtt_write_alarm_time(RTT, IrqNPulses + ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
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

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	
	printf("Comecando codigo...");

	/* Initialize the console uart */
	configure_console();
	
	//Semaforos
	
	xBtnSemaphore = xSemaphoreCreateBinary();
	if (xBtnSemaphore == NULL)
	printf("falha em criar o semaforo \n");
	
	// Queues
	
	xQueueCoins = xQueueCreate(100, sizeof(int));
	if (xQueueCoins == NULL)
	printf("falha em criar a queue xQueueCoins \n");

	//Tasks
	
	if (xTaskCreate(task_coins, "coins", TASK_OLED_STACK_SIZE, NULL,
	TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create coins task\r\n");
	}
	
	if (xTaskCreate(task_play, "play", TASK_OLED_STACK_SIZE, NULL,
	(TASK_OLED_STACK_PRIORITY+1), NULL) != pdPASS) {
		printf("Failed to create play task\r\n");
	}
	
	if (xTaskCreate(task_debug, "debug", TASK_OLED_STACK_SIZE, NULL,
	TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create debug task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
