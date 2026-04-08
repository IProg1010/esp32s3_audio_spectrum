#include "di_do_control.h"
#include "ch32v20x_gpio.h"
#include "debug.h"

/*struct for set state time duration*/

typedef struct
{
	enum InFilter filt_st;
	uint8_t filt_cnt;
	uint8_t curr_filt_cnt;
	enum PinState curr_st;
	enum PinActive active;
} InPinProp;

typedef struct
{
	enum OutType type;
	uint16_t duration_ms;
    uint16_t period_ms;
	uint16_t curr_duration_ms;
    uint16_t curr_period_ms;
	enum PinActive active;
	enum PinState future_state;
} OutPinProp;

typedef struct
{
	enum PinType type;
	uint8_t ch_num;
	enum PinState state;
	uint16_t pin;
	GPIO_TypeDef* port;
	InPinProp* in_pin;
	OutPinProp* out_pin;
} IOPin;

static uint8_t io_pin_cnt = 0;
IOPin io_pins[30];

static uint8_t out_pin_cnt = 0;
OutPinProp out_pins[30];

static uint8_t in_pin_cnt = 0;
InPinProp in_pins[30];


static uint8_t curr_pin_cnt = 0;
IOPin* curr_pin_control[30] = { NULL }; 


void setIOpin(uint16_t pin, enum Port port, uint8_t ch_num, enum PinType type, enum InMode in_type, enum OutMode out_type)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_TypeDef* port_base;
	if(port == pA)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		port_base = GPIOA;
	}
	if(port == pB)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		port_base = GPIOB;
	}
	if(port == pC)	
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		port_base = GPIOC;
	}
	if(port == pD)	
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
		port_base = GPIOD;
	}
	if(port == pE)	
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
		port_base = GPIOE;
	}

	GPIO_InitStructure.GPIO_Pin = pin;
	
	if(type == tInput)
	{
		if(in_type = iFL)
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  		else if(in_type = iPD)
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  		else if(in_type = iPU)
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	}
	if(type == tOutput)
	{
		if(out_type == oOD)
  			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
		else if (out_type == oPP)
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	}
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port_base, &GPIO_InitStructure);


	io_pins[io_pin_cnt].type = type;
	io_pins[io_pin_cnt].ch_num = ch_num;
	io_pins[io_pin_cnt].pin = pin;
	io_pins[io_pin_cnt].port = port_base;

	if(io_pins[io_pin_cnt].type == tOutput)
	{
		io_pins[io_pin_cnt].out_pin = (out_pins + out_pin_cnt);
		out_pin_cnt++;
	}
	else
	{
		io_pins[io_pin_cnt].in_pin = (in_pins + in_pin_cnt);
		in_pin_cnt++;
	}

	io_pin_cnt++;
}

void resetIOpin(uint8_t ch_num)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		
	}
}

enum PinState inversePinState(enum PinState st)
{
	if(st == stOn)
		return stOff;
	else if(st == stOff)
		return stOn;
	else
		return stUnknow;
}

BitAction bitActionFromState(enum PinState state)
{
	BitAction st;
	if(state  == stOn)
	{
		st = Bit_SET;
	}
	else if(state  == stOff)
	{
		st = Bit_RESET;
	}
	else
	{
		st = Bit_RESET;
	}

	return st;
}

enum PinState getInputCurrentState(uint8_t channel_num)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			return io_pins[i].state;		
		}
	}

	return stUnknow;
}

enum PinState getInputCurrentStateQuickly(uint8_t channel_num)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			uint8_t retVal = GPIO_ReadInputDataBit(io_pins[i].port, io_pins[i].pin);
			if(retVal == 0)
			{
				return stOff;
			}	
			else
			{
				return stOn;
			}		
		}
	}

	return stUnknow;
}

void setInputFilter(uint8_t channel_num, enum InFilter filt, uint8_t filter_cnt)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].in_pin->filt_st = filt;
			io_pins[i].in_pin->filt_cnt = filter_cnt;
			io_pins[i].in_pin->active = actOn;
		}
	}
}


enum PinState getCurrentOutputState(uint8_t channel_num)
{
	
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			return io_pins[i].state;		
		}
	}

	return stUnknow;
}

void setOutputStateQuickly(uint8_t channel_num, enum PinState state)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			GPIO_WriteBit(io_pins[i].port, io_pins[i].pin, bitActionFromState(state));
			io_pins[i].state = state;

			io_pins[i].out_pin->type = otSet;
			io_pins[i].out_pin->active = actOff;
			io_pins[i].out_pin->future_state = state;
			io_pins[i].out_pin->period_ms = 0;
			io_pins[i].out_pin->duration_ms = 0;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;

			printf("setOutputStateQuickly %d\r\n", channel_num);
			return;
		}
	}
}

void setOutputState(uint8_t channel_num, enum PinState state)
{
	
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otSet;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->future_state = state;
			io_pins[i].out_pin->period_ms = 0;
			io_pins[i].out_pin->duration_ms = 0;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;
			
			return;
		}
	}
}

void setOutputStateForTime(uint8_t channel_num, enum PinState state, uint16_t duration_ms)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otDurationSet;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->future_state = state;
			io_pins[i].out_pin->period_ms = 0;
			io_pins[i].out_pin->duration_ms = duration_ms;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;
			
			return;
		}
	}
}

void inverseOutputStateQuickly(uint8_t channel_num)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			GPIO_WriteBit(io_pins[i].port, io_pins[i].pin, bitActionFromState(inversePinState(io_pins[i].state)));
			io_pins[i].state = inversePinState(io_pins[i].state);

			io_pins[i].out_pin->type = otInverse;
			io_pins[i].out_pin->active = actOff;
			io_pins[i].out_pin->future_state = io_pins[i].state;
			io_pins[i].out_pin->period_ms = 0;
			io_pins[i].out_pin->duration_ms = 0;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;

			return;
		}
	}
}

void inverseOutputState(uint8_t channel_num)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otInverse;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->period_ms = 0;
			io_pins[i].out_pin->duration_ms = 0;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;
			
			return;
		}
	}
}

void inverseOutputStateForTime(uint8_t channel_num, uint16_t duration_ms)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otDurationImpulse;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->period_ms = 0;
			io_pins[i].out_pin->duration_ms = duration_ms;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;
			
			return;
		}
	}
}


void impulseOutputStateQuickly(uint8_t channel_num, uint16_t period_ms)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otImpulse;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->period_ms = period_ms;
			io_pins[i].out_pin->duration_ms = 0;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;
			
			
			GPIO_WriteBit(io_pins[i].port, io_pins[i].pin, bitActionFromState(inversePinState(io_pins[i].state)));

			io_pins[i].out_pin->future_state = inversePinState(io_pins[i].state);
			io_pins[i].state = inversePinState(io_pins[i].state);

			printf("impulseOutputStateQuickly %d\r\n", channel_num);
			return;
		}
	}
}

void impulseOutputState(uint8_t channel_num, uint16_t period_ms)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otImpulse;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->period_ms = period_ms;
			io_pins[i].out_pin->duration_ms = 0;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;

			io_pins[i].out_pin->future_state = inversePinState(io_pins[i].state);
			
			return;
		}
	}
}

void impulseOutputStateForTime(uint8_t channel_num, uint16_t period_ms, uint16_t duration_ms)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otDurationImpulse;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->period_ms = period_ms;
			io_pins[i].out_pin->duration_ms = 0;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;
			
			io_pins[i].out_pin->future_state = inversePinState(io_pins[i].state);

			printf("impulseOutputStateForTime %d\r\n", channel_num);
			return;
		}
	}
}

void blinkOutputStateQuickly(uint8_t channel_num, uint16_t period_ms)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otBlink;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->period_ms = period_ms;
			io_pins[i].out_pin->duration_ms = 0;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;
			
			GPIO_WriteBit(io_pins[i].port, io_pins[i].pin, bitActionFromState(stOn));
			io_pins[i].state = stOn;

			return;
		}
	}
}

void blinkOutputState(uint8_t channel_num, uint16_t period_ms)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otBlink;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->period_ms = period_ms;
			io_pins[i].out_pin->duration_ms = 0;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;

			printf("blinkOutputState %d\r\n", channel_num);
			
			return;
		}
	}
}

void blinkOutputStateForDuration(uint8_t channel_num, uint16_t period_ms, uint16_t duration_ms, enum PinState end_state)
{
	for(int i = 0; i < io_pin_cnt; i++)
	{
		if(io_pins[i].ch_num == channel_num)
		{
			io_pins[i].out_pin->type = otDurationBlink;
			io_pins[i].out_pin->active = actOn;
			io_pins[i].out_pin->period_ms = period_ms;
			io_pins[i].out_pin->duration_ms = duration_ms;
			io_pins[i].out_pin->curr_period_ms = 0;
			io_pins[i].out_pin->curr_duration_ms = 0;
			io_pins[i].out_pin->future_state = end_state;

			printf("blinkOutputStateForDuration %d\r\n", channel_num);
			
			return;
		}
	}
}

void updateForTimerMs()
{
	IOPin* pin;
	for(int i = 0; i < io_pin_cnt; i++)
	{
		pin = &io_pins[i]; 
		if(pin->type == tInput)
		{
			uint8_t retVal = GPIO_ReadInputDataBit(pin->port, pin->pin);
			if(pin->in_pin->filt_st == fOn)
			{
				if(retVal == 0)
				{
					if(pin->in_pin->curr_filt_cnt > 0)
						pin->in_pin->curr_filt_cnt--;
				}
				else
				{
					if(pin->in_pin->curr_filt_cnt < pin->in_pin->filt_cnt)
						pin->in_pin->curr_filt_cnt++;
				}
				pin->in_pin->curr_st = retVal;

				if(pin->in_pin->curr_filt_cnt >= pin->in_pin->filt_cnt)
				{
					pin->state = stOn;
				}
				else if(pin->in_pin->curr_filt_cnt == 0)
				{
					pin->state = stOff;
				}
			}
			else
			{
				if(retVal == 1)
				{
					pin->state = stOn;
				}
				else
				{
					pin->state = stOff;
				}
			}
		}
		else
		{
			if(pin->out_pin->active == actOn)
			{
				pin->out_pin->curr_period_ms++;
				pin->out_pin->curr_duration_ms++;

				if(pin->out_pin->type == otBlink)
				{
					if(pin->out_pin->period_ms < pin->out_pin->curr_period_ms)
					{
						//printf("otBlink %d\r\n", pin->ch_num);
						GPIO_WriteBit(pin->port, pin->pin, bitActionFromState(inversePinState(pin->state)));
						pin->state = inversePinState(pin->state);
						pin->out_pin->curr_period_ms = 0;
						pin->out_pin->curr_duration_ms = 0;
					}
				}	
				if(pin->out_pin->type == otDurationBlink)
				{

					if(pin->out_pin->duration_ms > pin->out_pin->curr_duration_ms)
					{
						if(pin->out_pin->period_ms < pin->out_pin->curr_period_ms)
						{
							//printf("otDurationBlink %d\r\n", pin->ch_num);
							GPIO_WriteBit(pin->port, pin->pin, bitActionFromState(inversePinState(pin->state)));
							pin->state = inversePinState(pin->state);
							pin->out_pin->curr_period_ms = 0;
						}
					}
					else
					{
						GPIO_WriteBit(pin->port, pin->pin, bitActionFromState(pin->out_pin->future_state));
						pin->state = pin->out_pin->future_state;
						pin->out_pin->curr_duration_ms = 0;
						pin->out_pin->curr_period_ms = 0;
						pin->out_pin->active = actOff;
					}
				}	
				else if(pin->out_pin->type == otInverse)
				{
					GPIO_WriteBit(pin->port, pin->pin, bitActionFromState(inversePinState(pin->state)));
					pin->state = inversePinState(pin->state);
					pin->out_pin->active = actOff;
				}	
				else if(pin->out_pin->type == otDurationInverse)
				{
					if(pin->out_pin->duration_ms > pin->out_pin->curr_duration_ms)
					{
						GPIO_WriteBit(pin->port, pin->pin, bitActionFromState(inversePinState(pin->state)));
						pin->state = inversePinState(pin->state);
						pin->out_pin->curr_duration_ms = 0;
						pin->out_pin->active = actOff;
					}
				}	
				else if(pin->out_pin->type == otSet)
				{
					GPIO_WriteBit(pin->port, pin->pin, bitActionFromState(pin->out_pin->future_state));
					pin->state = pin->out_pin->future_state;
					pin->out_pin->active = actOff;
				}
				else if(pin->out_pin->type == otDurationSet)
				{
					if(pin->out_pin->duration_ms > pin->out_pin->curr_duration_ms)
					{
						GPIO_WriteBit(pin->port, pin->pin, bitActionFromState(pin->out_pin->future_state));
						pin->state = pin->out_pin->future_state;
						pin->out_pin->curr_duration_ms = 0;
						pin->out_pin->active = actOff;
					}
				}
				else if(pin->out_pin->type == otImpulse)
				{
					if(pin->out_pin->future_state != pin->state)
					{
						GPIO_WriteBit(pin->port, pin->pin, bitActionFromState(pin->out_pin->future_state));
						pin->state = pin->out_pin->future_state;
						pin->out_pin->curr_period_ms = 0;
					}
					else
					{
						if(pin->out_pin->period_ms < pin->out_pin->curr_period_ms)
						{
							GPIO_WriteBit(pin->port, pin->pin, inversePinState(pin->state));
							pin->state = inversePinState(pin->state);
							pin->out_pin->curr_period_ms = 0;
							pin->out_pin->curr_duration_ms = 0;
							pin->out_pin->active = actOff;
						}
					}
				}
				else if(pin->out_pin->type == otDurationImpulse)
				{
					if(pin->out_pin->duration_ms < pin->out_pin->curr_duration_ms)
					{
						if(pin->out_pin->future_state != pin->state)
						{
							GPIO_WriteBit(pin->port, pin->pin, bitActionFromState(pin->out_pin->future_state));
							pin->state = pin->out_pin->future_state;
							pin->out_pin->curr_period_ms = 0;
						}
						else
						{
							if(pin->out_pin->period_ms < pin->out_pin->curr_period_ms)
							{
								GPIO_WriteBit(pin->port, pin->pin, inversePinState(pin->state));
								pin->state = inversePinState(pin->state);
								pin->out_pin->curr_period_ms = 0;
								pin->out_pin->curr_duration_ms = 0;
								pin->out_pin->active = actOff;
							}
						}
					}
				}
			}
		}
	}
}