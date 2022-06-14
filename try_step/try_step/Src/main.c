
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"


#define LINE_BUFFER_LENGTH 512

#define DIR_PIN GPIO_PIN_3
#define DIR_PORT GPIOA
#define STEP_PIN GPIO_PIN_2
#define STEP_PORT GPIOA

#define DIR_PIN1 GPIO_PIN_7
#define STEP_PIN1 GPIO_PIN_6

#define DIR_PIN2 GPIO_PIN_4
#define STEP_PIN2 GPIO_PIN_5


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int line_number = 0;

struct point {
	float x;
	float y;
	float z;
};

struct point actuatorPos;

float StepInc = 1;
int penDelay = 50;

uint16_t stepsPerMm = 80;

float Xmin = 0;
float Xmax = 400;
float Ymin = 0;
float Ymax = 400;
float Zmin = 0;
float Zmax = 1;

float Xpos = 0;
float Ypos = 0;
float Zpos = 1; 

int stepDelay = 1000;
int dx, dy;
uint8_t flag = 0;

char line[LINE_BUFFER_LENGTH];
uint16_t Index = 0;
uint8_t c;
int lineIndex = 0;
int line_comment = 0;
int line_semicolon = 0;
float posX, posY;

char home[] = "home";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void microDelay (uint16_t delay);
void print(char *tx_data);
void print1(char *tx_data);
void Received_and_Send();
void clear_buffer();
void process_command(char *line, uint16_t Index);
void drawLine(float x, float y);
void print_real(float x);
void print_number(int a);
void onestep(uint8_t d, uint8_t a);
void penUp();
void penDown();
void homecoming();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_UART_Receive_IT(&huart5, &c, 1); 
	
	/*print("Mini CNC Plotter alive and kicking!\r\n");
  print("X range is from "); 
  print("0"); 
  print(" to "); 
  print("40"); 
  print(" mm.\r\n"); 
  print("Y range is from "); 
  print("0"); 
  print(" to "); 
  print("40"); 
  print(" mm.\r\n"); */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		int i;
		HAL_UART_Receive_IT(&huart5, &c, 1); 
		
		/*HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIR_PORT, DIR_PIN1, GPIO_PIN_SET);
		for (i = 0; i < 3200; i++){
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN1, GPIO_PIN_SET);
			microDelay (stepDelay);
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN1, GPIO_PIN_RESET);
			microDelay (stepDelay);	
		}
		
		HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIR_PORT, DIR_PIN1, GPIO_PIN_RESET);
		for (i = 0; i < 3200; i++){
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN1, GPIO_PIN_SET);
			microDelay (stepDelay);
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN1, GPIO_PIN_RESET);
			microDelay (stepDelay);
		}
		HAL_Delay(5000);*/
		//if(flag ==0){
			//print("M1 X25.89 Y55.59 F3500.0\nG1 X35.34 Y24.45 F3500.00\nenough\n\n");
			//HAL_Delay(1000);
			//flag  =1;
		//}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART5){
		Received_and_Send();
		HAL_UART_Receive_IT(&huart5,&c,1);
	}
}

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

//in ra man hinh chuoi ki tu yeu cau
void print(char *tx_data){
	while(*tx_data != '\0'){
		HAL_UART_Transmit(&huart5, (uint8_t*)tx_data, sizeof(*tx_data),1000);
		tx_data++;
	}
}

void print1(char *tx_data){
	while(*tx_data != '\0'){
		HAL_UART_Transmit(&huart1, (uint8_t*)tx_data, sizeof(*tx_data),1000);
		tx_data++;
	}
}

//nhan ki tu va gui lai
void Received_and_Send(){
	if((c != '\n')&&(c != '\r')){
		if((line_comment) || (line_semicolon) ) {   // Throw away all comment characters
			if( c == ')' )  line_comment = 0;     // End of comment. Resume line.
			} 
    else{
			if ( c <= ' ' ) {                           // Throw away whitepace and control characters
				} 
      else if ( c == '/' ) {                    // Block delete not supported. Ignore character.
        } 
      else if ( c == '(' ) {                    // Enable comments flag and ignore all characters until ')' or EOL.
				line_comment = 1;
        } 
      else if ( c == ';' ) {
				//line_semicolon = 1;
         } 
      else if ( lineIndex >= LINE_BUFFER_LENGTH-1 ) {
				print( "ERROR - lineBuffer overflow\n" );
				line_comment = 0;
				line_semicolon = 0;
         } 
      else if ( c >= 'a' && c <= 'z' ) {        // Upcase lowercase
        line[ lineIndex++ ] = c-'a'+'A';
         }
      else {
        line[ lineIndex++ ] = c;
         }
        }
      }
	else{
		line_number++;
		if ( lineIndex > 0 ) {                        // Line is complete. Then execute! 
			if(!strcmp(line,home)){
				homecoming();
			}
			process_command(line, lineIndex);
			clear_buffer();
			lineIndex = 0;
		}
			line_comment = 0;
			line_semicolon = 0;
			
			print("ok\n");
		}
}
			
	


//xoa bo dem khi gap '\n'
void clear_buffer(){
	for(int i = 0;i<512;i++){
		line[i] = 0;
	}
}

//dieu khien dong co buoc voi G-CODE
void process_command(char *line, uint16_t Index){
	int currentIndex = 0;
	char buffer[64];
	struct point newPos;
	newPos.x = 0.0;
	newPos.y = 0.0;
	while(currentIndex < Index){
		switch(line[currentIndex++]){
			case 'U':
				penUp();
				break;
			case 'D':
				penDown();
				break;
			case 'G':
				switch(line[currentIndex++]){
					case '0':
	//					homecoming();
						break;
					case '1':
						char *indexX = strchr(line,'X');
						char *indexY = strchr(line,'Y');
						newPos.x = atof(indexX + 1);
						newPos.y = atof(indexY + 1);
						posX = newPos.x;
						drawLine(newPos.x, newPos.y);
						actuatorPos.x = newPos.x;
						actuatorPos.y = newPos.y;
						break;
					default:
						break;
				}
				break;
			case 'M':
						buffer[0] = line[ currentIndex++ ];        // /!\ Dirty - Only works with 3 digit commands
						buffer[1] = line[ currentIndex++ ];
						buffer[2] = line[ currentIndex++ ];
						buffer[3] = '\0';
						switch ( atoi( buffer ) ){
							case 3:
								char* indexS = strchr( line+currentIndex, 'S' );
								float Spos = atof( indexS + 1);
								if (Spos == 255) { 
									penDown(); 
								}
								else{ 
									penUp(); 
								}
								break;
							default:
								break;
						}
				}
		}
}

void drawLine(float x1, float y1){
	
	//Destination
	print("x1, y1: ");
  print_real(x1);
  print(",");
  print_real(y1);
  print("\n");
	
	if (x1 >= Xmax) { 
   x1 = Xmax; 
  }
  if (x1 <= Xmin) { 
    x1 = Xmin; 
  }
  if (y1 >= Ymax) { 
    y1 = Ymax; 
  }
  if (y1 <= Ymin) { 
    y1 = Ymin; 
  }
	
	//Current position
	print("Xpos, Ypos: ");
  print_real(Xpos);
  print(",");
  print_real(Ypos);
  print("\n");
	
	//Convert destination to steps
	x1 = (int)(x1*stepsPerMm);
	print("steps_x1: ");
  print_real(x1);
  y1 = (int)(y1*stepsPerMm);
	print(", steps_y1: ");
  print_real(y1);
	print("\n");
	
  float x0 = Xpos;
  float y0 = Ypos;
	
	//Steps to reach destination from current position
	dx = abs(x1-x0);
  dy = abs(y1-y0);
	print("dx, dy:");
  print_number(dx);
  print(",");
  print_number(dy);
  print("\n");
	print("Going to destination (");
  print_number(x1);
  print(",");
  print_number(y1);
  print(")\n");
	
  int sx = x0<x1 ? StepInc : -StepInc;
  int sy = y0<y1 ? StepInc : -StepInc;
	
	long i;
  long over = 0;

  if (dx > dy) {
    for (i=0; i<dx; ++i) {
      onestep(sx,1);					//direct: x
      over+=dy;
      if (over>=dx) {
       over-=dx;
				onestep(sy,0);					//direct: y
      }
    }
  }
  else {
    for (i=0; i<dy; ++i) {
      onestep(sy,0);
      over+=dx;
      if (over>=dy) {
        over-=dy;
        onestep(sx,1);
      }
    }    
  }
	Xpos = x1;
  Ypos = y1;
}

void print_real(float a){
	char str[10];
	sprintf(str,"%0.3f",a);
	print(str);
}

void print_number(int a){
  char str[10];
	sprintf(str,"%d",a);
	print(str);
}

void onestep(uint8_t direct, uint8_t ordinate){			//ordinate = 1: x, direct = 1: forward
		if(ordinate == 1){
			if(direct == 1){	// x - forward
				HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);				
				microDelay (stepDelay);
				HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
				microDelay (stepDelay);	
			}
			else{				//x - backward
				HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);			
				microDelay (stepDelay);
				HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);			
				microDelay (stepDelay);
			}
		}
		else{
			if(direct == 1){ // y - forward
				HAL_GPIO_WritePin(DIR_PORT, DIR_PIN1, GPIO_PIN_SET);
				HAL_GPIO_WritePin(STEP_PORT, STEP_PIN1, GPIO_PIN_SET);
				microDelay (stepDelay);				
				HAL_GPIO_WritePin(STEP_PORT, STEP_PIN1, GPIO_PIN_RESET);
				microDelay (stepDelay);	
			}
			else{			// y - backward
				HAL_GPIO_WritePin(DIR_PORT, DIR_PIN1, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STEP_PORT, STEP_PIN1, GPIO_PIN_SET);
				microDelay (stepDelay);			
				HAL_GPIO_WritePin(STEP_PORT, STEP_PIN1, GPIO_PIN_RESET);
				microDelay (stepDelay);
			}
		}
}

void penUp(){
	int i;
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN2, GPIO_PIN_SET);
	for(i = 0;i < 100; i++){
		HAL_GPIO_WritePin(STEP_PORT, STEP_PIN2, GPIO_PIN_SET);
		microDelay (stepDelay);			
		HAL_GPIO_WritePin(STEP_PORT, STEP_PIN2, GPIO_PIN_RESET);
		microDelay (stepDelay);
	}
}

void penDown(){
	int i;
	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN2, GPIO_PIN_RESET);
	for(i = 0;i < 100; i++){
		HAL_GPIO_WritePin(STEP_PORT, STEP_PIN2, GPIO_PIN_SET);
		microDelay (stepDelay);			
		HAL_GPIO_WritePin(STEP_PORT, STEP_PIN2, GPIO_PIN_RESET);
		microDelay (stepDelay);
	}
}

void homecoming(){
	dx = (int)(Xpos*stepsPerMm);
	dy = (int)(Ypos*stepsPerMm);
	
	int sx = -StepInc;
  int sy = -StepInc;
	
	long i;
  long over = 0;

  if (dx > dy) {
    for (i=0; i<dx; ++i) {
      onestep(sx,1);					//direct: x
      over+=dy;
      if (over>=dx) {
       over-=dx;
				onestep(sy,0);					//direct: y
      }
    }
  }
  else {
    for (i=0; i<dy; ++i) {
      onestep(sy,0);
      over+=dx;
      if (over>=dy) {
        over-=dy;
        onestep(sx,1);
      }
    }    
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
