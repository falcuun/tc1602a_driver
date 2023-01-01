#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"

#define SYSTICK_LOAD_VAL 16000
#define SYSTICK_CTRL_ENABLE (1U<<0)
#define SYSTICK_CTRL_CLKSRC (1U<<2)
#define SYSTICK_CTRL_COUNTFLAG (1U<<16)

#define GPIOAEN (1U<<0)
#define GPIOBEN (1U<<1)
#define GPIOCEN (1U<<2)
#define GPIODEN (1U<<3)
#define GPIOEEN (1U<<4)

#define LCD_RS(_signal) DIGITAL_WRITE(GPIOA, PIN10, _signal)
#define LCD_RW(_signal) DIGITAL_WRITE(GPIOB, PIN3, _signal)
#define LCD_E(_signal) DIGITAL_WRITE(GPIOB, PIN5, _signal)
#define LCD_DB0(_signal) DIGITAL_WRITE(GPIOB, PIN4, _signal)
#define LCD_DB1(_signal) DIGITAL_WRITE(GPIOB, PIN10, _signal)
#define LCD_DB2(_signal) DIGITAL_WRITE(GPIOA, PIN8, _signal)
#define LCD_DB3(_signal) DIGITAL_WRITE(GPIOA, PIN9, _signal)
#define LCD_DB4(_signal) DIGITAL_WRITE(GPIOC, PIN7, _signal)
#define LCD_DB5(_signal) DIGITAL_WRITE(GPIOB, PIN6, _signal)
#define LCD_DB6(_signal) DIGITAL_WRITE(GPIOA, PIN7, _signal)
#define LCD_DB7(_signal) DIGITAL_WRITE(GPIOA, PIN6, _signal)

void delay_ms(int ms)
{
	SysTick->LOAD = SYSTICK_LOAD_VAL;

	SysTick->VAL = 0;

	SysTick->CTRL = SYSTICK_CTRL_ENABLE | SYSTICK_CTRL_CLKSRC;

	for(int i = 0; i < ms; i++)
	{
		while((SysTick->CTRL & SYSTICK_CTRL_COUNTFLAG) == 0){}
	}
	SysTick->CTRL = 0;
}

void delay_us(uint32_t us) {
  // Enable SysTick timer
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

  // Set reload value for the desired delay
  SysTick->LOAD = (us * (SystemCoreClock / 1000000)) - 1;

  // Reset counter value
  SysTick->VAL = 0;

  // Wait for countdown to complete
  while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

  // Disable SysTick timer
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

typedef enum {
INPUT =  0,
OUTPUT = 1,
}MODE;

typedef enum {
	LOW = 	0,
	HIGH = 	1,
}SIGNAL;

typedef enum {
PORT_A = 0,
PORT_B = 1,
PORT_C = 2,
PORT_D = 3,
PORT_E = 4,
}PORT_LETTER;

typedef enum{
PIN0 =  0,
PIN1 =  1,
PIN2 =  2,
PIN3 =  3,
PIN4 =  4,
PIN5 =  5,
PIN6 =  6,
PIN7 =  7,
PIN8 =  8,
PIN9 =  9,
PIN10 = 10,
PIN11 = 11,
PIN12 = 12,
PIN13 = 13,
PIN14 = 14,
PIN15 = 15,
}PIN_NUMBER;

void PIN_MODE(PORT_LETTER port_num, PIN_NUMBER pin_num, MODE mode)
{

	int pin_bit1 = pin_num * 2;
	int pin_bit2 = pin_bit1 + 1;


	RCC->AHB1ENR |= (1U<<port_num); // Enable Clock access to the Port

	GPIO_TypeDef *GPIO_PORT;
	switch(port_num)
	{
	case PORT_A: GPIO_PORT = GPIOA; break;
	case PORT_B: GPIO_PORT = GPIOB; break;
	case PORT_C: GPIO_PORT = GPIOC; break;
	case PORT_D: GPIO_PORT = GPIOD; break;
	case PORT_E: GPIO_PORT = GPIOE; break;
	}

	if(mode == INPUT)
	{
		GPIO_PORT->MODER &=~(1U<<pin_bit1);
		GPIO_PORT->MODER &=~(1U<<pin_bit2);
	}
	else if (mode == OUTPUT)
	{
		GPIO_PORT->MODER |= (1U<<pin_bit1);
		GPIO_PORT->MODER &=~(1U<<pin_bit2);
	}

}

void DIGITAL_WRITE(GPIO_TypeDef *port, PIN_NUMBER pin, SIGNAL signal)
{
	if(signal == LOW)
	{
		/* BR(y) BIT RESET, When this bit is set to 1, it will
		 * RESET the corresponding ODRx Bit
		 * When set to 0, nothing happens.*/
		port->BSRR = (1U<<(pin + 16));
	}
	else if (signal == HIGH)
	{
		/* BS(y) BIT SET, When this bit is set to 1, it will
		 * SET the corresponding ODRx Bit
		 * When set to 0, nothing happens.*/
		port->BSRR = (1U<<pin);
	}
}

int DIGITAL_READ(GPIO_TypeDef *port, PIN_NUMBER pin)
{
	return (port->IDR & (1U<<pin));
}

void peripherals_init()
{
	RCC->AHB1ENR |= GPIOAEN;
	RCC->AHB1ENR |= GPIOBEN;
	RCC->AHB1ENR |= GPIOCEN;
	RCC->AHB1ENR |= GPIODEN;


	PIN_MODE(PORT_A, PIN10, OUTPUT);
	PIN_MODE(PORT_B, PIN3, OUTPUT);
	PIN_MODE(PORT_B, PIN5, OUTPUT);
	PIN_MODE(PORT_B, PIN4, OUTPUT);
	PIN_MODE(PORT_B, PIN10, OUTPUT);
	PIN_MODE(PORT_A, PIN8, OUTPUT);
	PIN_MODE(PORT_A, PIN9, OUTPUT);
	PIN_MODE(PORT_C, PIN7, OUTPUT);
	PIN_MODE(PORT_B, PIN6, OUTPUT);
	PIN_MODE(PORT_A, PIN7, OUTPUT);
	PIN_MODE(PORT_A, PIN6, OUTPUT);

}

void send(SIGNAL rs, uint8_t value)
{
	LCD_RS(rs);
	LCD_DB0((value >> 0)& 0x01);
	LCD_DB1((value >> 1)& 0x01);
	LCD_DB2((value >> 2)& 0x01);
	LCD_DB3((value >> 3)& 0x01);
	LCD_DB4((value >> 4)& 0x01);
	LCD_DB5((value >> 5)& 0x01);
	LCD_DB6((value >> 6)& 0x01);
	LCD_DB7((value >> 7)& 0x01);

	LCD_E(1);
	LCD_E(0);
	delay_ms(1);
}

void send_command(uint8_t value)
{
	send(LOW, value);
}

void send_data(char *value)
{
	const char *ptr = value;
	size_t index = 0;
	while(*ptr != '\0')
	{
		send(HIGH, (uint8_t)value[index]);
		ptr++;
		index++;
	}
}

void set_cursor(int row, int col)
{
	int row1 = 0x00;
	int row2 = 0x40;
	switch(row)
	{
	case 1:
		send_command(0x80 | (row1 + col));
		break;
	case 2:
		send_command(0x80 | (row2 + col));
		break;
	}
}

void clear_screen()
{
	send_command(0x01);
	delay_ms(2);
}

int x_pos = 1;
int max_x = 15;
int min_x = 0;
int direction = 0;

void scroll_text(char *text)
{
		set_cursor(1,x_pos);
		clear_screen();
		set_cursor(1,x_pos);
		send_data(text);
		direction == 0 ? ++x_pos : --x_pos;
		if(x_pos > max_x && direction == 0)
		{
			direction = 1;
		}
		if(x_pos < min_x && direction == 1)
		{
			direction = 0;
		}

		delay_ms(300);
}

int main(void)
{
	peripherals_init();

	delay_ms(100);

	send_command(0x3C);
	send_command(0xE);
	send_command(0x06);
	send_command(0x01);


	delay_ms(1);

//	set_cursor(1,-1);
//	send_data("Hello World");
	while(1)
	{
		 scroll_text("Happy New Year!");
	}
}
