/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "Hard/USART.h"
#include "Hard/SPI.h"


#include "stm32f10x_tim.h"
#include "stm32f10x.h"
#include "core_cm3.h"

#include "Hard/Timer.h"
#include "BlinkLed.h"
#include "LORA.h"

USART 		USART_COM1;
Timer 		timer;

int usleep(useconds_t __useconds){

	timer.sleep(__useconds/1000);

	return 0;
}

extern "C" void USART1_IRQHandler(void)
{
	USART_COM1.USART1_Interrupt();
}

extern "C" void SysTick_Handler(void)
{
#if defined(USE_HAL_DRIVER)
  HAL_IncTick();
#endif

}

extern "C" void TIM2_IRQHandler(void) {
    if((TIM2->SR & TIM_SR_UIF) != 0)
    {// If update flag is set
		Timer::tick();
		Timer::SysTime++;
    }

    TIM2->SR &= ~TIM_SR_UIF; //clear interrupt flag

}


// Definitions visible only within this translation unit.
namespace
{
  // ----- Timing definitions -------------------------------------------------

  // Keep the LED on for 2/3 of a second.
  constexpr Timer::ticks_t BLINK_ON_TICKS = Timer::FREQUENCY_HZ * 3 / 4;
  constexpr Timer::ticks_t BLINK_OFF_TICKS = Timer::FREQUENCY_HZ
      - BLINK_ON_TICKS;
}

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int
main(int argc, char* argv[])
{
  // Show the program parameters (passed via semihosting).
  // Output is via the semihosting output channel.
  trace_dump_args(argc, argv);

  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello Eduardo!");

  // Send a message to the standard output.
  puts("Standard output message.");

  // Send a message to the standard error.
//  fprintf(stderr, "Standard error message.\n");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  SysTick_Config(SystemCoreClock / 1000u);

  timer.start();

  SPI spi;
  if(!spi.Init(SPI2))
  {
	  trace_printf("SPI fail!\n");
	  return 0;
  }
  trace_printf("SPI ok!\n");

  RadioCallbacks_t CallBack;

  LORA	Lora(&CallBack);
  if(!Lora.Initialize(&spi,915))
  {
	  trace_printf("Lora fail!\n");
	  assert(false);
	  return 0;
  }

  trace_printf("Lora OK!\n");
  BlinkLed blinkLed;

  // Perform all necessary initialisations for the LED.
  blinkLed.powerUp(2,13);

	//Init Serial Communication
	USART_COM1.Setup();
//	setbuf(stdout, NULL);
	USART_COM1.write("Ola mundo\n\r");
	printf("I'm alive\n\r");
	puts("Serial Started");
	Lora.EnableTX();
	usleep(100);


  // Short loop.
  while(true)
    {
      blinkLed.turnOn();
      timer.sleep(BLINK_ON_TICKS);

      blinkLed.turnOff();
      timer.sleep(BLINK_OFF_TICKS);

      uint8_t data[10]={0xFF,0xAA,0,1,2,3,4,5,6,7};
      Lora.SendPayload(data,6,0);
	  timer.sleep(10);
      while(DIO2_IS_HIGH);


      //Command TX done
      uint8_t counter = 10;
      do{
    	  counter--;
    	  usleep(500);
      }while(!Lora.TXDone() && (counter!=0));
    }
  return 0;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
