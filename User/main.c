/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Private variables *******************************************************/
float adcValue1 = 0;
float adcValue2 = 0;
float adcValue3 = 0;
float adcValue4 = 0;
float percentageAdcValue1 = 0;
float percentageAdcValue2 = 0;
float percentageAdcValue3 = 0;
float percentageAdcValue4 = 0;
/* Private Function Prototypes *********************************************/

/* Main Function ***********************************************************/
int main(void){

	/* Initialize Module &  BitzOS */
	Module_Init();

	/* Don't place your code here */
	for(;;){
	}
}

/***************************************************************************/
void UserTask(void *argument){

  AddButton(P2, SWITCH_NO, CLICKED);
  // put your code here, to run repeatedly.
  while (1) {}
}

void buttonClickedCallback(uint8_t port) {
    IND_blink(100);
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
