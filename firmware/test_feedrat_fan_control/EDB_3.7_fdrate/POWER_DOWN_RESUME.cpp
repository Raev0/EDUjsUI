#include "Marlin.h"
#include "EEPROM.h"
#include "buzzer_sound.h"
#ifdef POWER_DOWN_CHECK
//uint32_t power_off_store[9];
/*
bool POWER_CHECK()
{
// return digitalRead(POWER_0_CHECK)==0 ? true : false;
 return digitalRead(POWER_OFF_CHECK)==0 ? true : false;
}
*/
extern float w_d_x_5_16;
void POWER_DOWN_INTERRUPT()
{
   back_z_value.v=w_d_x_5_16;
   for(int i=0;i<8;i++)
	{
       EEPROM.write(i,power_off_store[i]);
    }   
	for (int i=0;i<4;i++)
	{
		EEPROM.write(i+55,back_z_value.zchar[i]);
	}
  if(!digitalRead(POWER_OFF_CHECK))
	{
	 delay(10);
	 if(!digitalRead(POWER_OFF_CHECK))
     digitalWrite(POWER_OFF,HIGH);
    }

  }

void POWER_DOWN_RESUME_INIT()
{
	 pinMode(POWER_OFF_CHECK,INPUT);
	 digitalWrite(POWER_OFF_CHECK,HIGH);
	 attachInterrupt(0,POWER_DOWN_INTERRUPT,CHANGE);
	 pinMode(POWER_OFF,OUTPUT);
	 digitalWrite(POWER_OFF,LOW);

 // pinMode(POWER_0_CHECK,INPUT);
 // digitalWrite(POWER_0_CHECK,HIGH);
}

#endif










