/*
 * time.cpp
 *
 *  Created on: Jul 26, 2024
 *      Author: agamb
 */

#include "hardware/utils/time.h"

void HAL_Delay_us(__IO uint32_t delay_us)
{
  uint32_t first_value = 0;
  uint32_t current_value = 0;
  uint32_t reload = SysTick ->LOAD;

  uint32_t nus_number = delay_us * ((reload + 1) / 1000);
  uint32_t change_number = 0;

  first_value = SysTick ->VAL;
  while (1)
  {
    current_value = SysTick ->VAL;
    if (current_value != first_value)
    {

      if (current_value < first_value)
      {
        change_number += first_value - current_value;
        //change_number = first_value - current_value + change_number;
      }

      else
      {
        change_number += reload - current_value + first_value;
      }
      first_value = current_value;
      if (change_number >= nus_number)
      {
        break;
      }
    }
  }
}
