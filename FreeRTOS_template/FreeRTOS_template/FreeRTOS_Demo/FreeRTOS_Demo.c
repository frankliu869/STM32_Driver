#include "FreeRTOS_Demo.h"
#include "delay.h"
#include "queue.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "event_groups.h"

#define event_1 (1<<0)
#define event_2 (1<<1)

#define Start_Task_Size 130
uint32_t Start_Task_Stack[Start_Task_Size];
StaticTask_t start_TCB;

void freeRTOS_demo(void){
	xTaskCreateStatic( (TaskFunction_t) Start_Task,
                             (char *) "Start_Task",
                           (uint32_t) 130,
                             (void *) NULL,
                        (UBaseType_t) 0,
                      (StackType_t *) Start_Task_Stack,
                     (StaticTask_t *) &start_TCB
										);
								 
	vTaskStartScheduler();
}
TaskHandle_t Task1_Handle;
TaskHandle_t Task2_Handle;
TaskHandle_t KEY_Task_Handle;

TimerHandle_t timer1;
TimerHandle_t timer2;

void Timer1_Callback( TimerHandle_t xTimer ){
	static uint8_t num1=0;
	num1++;
	printf("The Timer 1 is running %d\r\n", num1);
}

void Timer2_Callback( TimerHandle_t xTimer ){
	static uint8_t num2=0;
	num2++;
	printf("The Timer 2 is running %d\r\n", num2);
}


void Start_Task(void){
	taskENTER_CRITICAL();
	
	timer1 = xTimerCreate("TIMER1",500,pdTRUE,(void *)1,Timer1_Callback);
	timer2 = xTimerCreate("TIMER2",1000,pdTRUE,(void *)2,Timer2_Callback);
	
	xTaskCreate( (TaskFunction_t				 )Task1,
								 (char *								 )"Task1",
								 (configSTACK_DEPTH_TYPE)200,
								 (void * 							 )NULL,
								 (UBaseType_t					 )2,
								 (TaskHandle_t * 			 )&Task1_Handle);
								 
								 
	xTaskCreate( (TaskFunction_t				 )task2,
							 (char *								 )"task2",
							 (configSTACK_DEPTH_TYPE)130,
							 (void * 							 )NULL,
							 (UBaseType_t					 )2,
							 (TaskHandle_t * 			 )&Task2_Handle);
							 
	/*xTaskCreate( (TaskFunction_t				 )task3,
							 (char *								 )"task3",
							 (configSTACK_DEPTH_TYPE)200,
							 (void * 							 )NULL,
							 (UBaseType_t					 )2,
							 (TaskHandle_t * 			 )&KEY_Task_Handle);*/
	
		 
	vTaskDelete(NULL);
	taskEXIT_CRITICAL();

}


void Task1(void){
	
	while(1){
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==0){
			while(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==0);
			xTimerStart(timer1,portMAX_DELAY);
			xTimerStart(timer2,portMAX_DELAY);
		}
		vTaskDelay(10);
	}
	
}


void task2(void){
	uint32_t rec;
	while(1){
		printf("task is running !\r\n");
		printf("task is running !\r\n");
		vTaskDelay(500);
	}
}


void task3(void){
	
	while(1){
		
	}
	
	
}



void EXTI3_IRQHandler(void)
{
	while(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==0);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
	while(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==0);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}	

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
}

#define staticStack_Size 130
StaticTask_t IdleTaskTCB;
StackType_t IdleTaskStack[staticStack_Size];
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer, StackType_t ** ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	*ppxIdleTaskTCBBuffer = &IdleTaskTCB;
	*ppxIdleTaskStackBuffer = IdleTaskStack;
	*pulIdleTaskStackSize = staticStack_Size;
}


StaticTask_t TimerTCB;
StackType_t TimerStack[staticStack_Size];
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer, StackType_t ** ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
	*ppxTimerTaskTCBBuffer = &TimerTCB;
	*ppxTimerTaskStackBuffer = TimerStack;
	*pulTimerTaskStackSize = staticStack_Size;
}




TIM_HandleTypeDef TIM2_HDER;
TIM_HandleTypeDef TIM3_HDER;

void timer_Init(int count1)
{
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	
	HAL_NVIC_SetPriority(TIM3_IRQn,4,0);    
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	
	HAL_NVIC_SetPriority(TIM2_IRQn,10,0);    
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
	TIM2_HDER.Instance=TIM2;
	TIM2_HDER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	TIM2_HDER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM2_HDER.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	TIM2_HDER.Init.Period = count1-1;
	TIM2_HDER.Init.Prescaler = 1-1;
	
	HAL_TIM_Base_Init(&TIM2_HDER);
	
	
	
	/*TIM3_HDER.Instance=TIM3;
	TIM3_HDER.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	TIM3_HDER.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM3_HDER.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	TIM3_HDER.Init.Period = 9000-1;
	TIM3_HDER.Init.Prescaler = 72000-1;
	
	HAL_TIM_Base_Init(&TIM3_HDER);*/
	
	
	
	HAL_TIM_Base_Start_IT(&TIM2_HDER);
	//HAL_TIM_Base_Start_IT(&TIM3_HDER);
}
uint32_t FreeRTOSRunTimeTicks;
void CONFIGURE_TIMER_FOR_RUN_TIME_STATS(void){
	timer_Init(720);
	FreeRTOSRunTimeTicks = 0;
}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TIM2_HDER);
}

void TIM3_IRQHandler(void)
{
   HAL_TIM_IRQHandler(&TIM3_HDER);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2){
			FreeRTOSRunTimeTicks++;
		}else if(htim->Instance == TIM3){
			printf("TIM3 is running !\r\n");
		}
		
}
																					

