#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define set_bit(reg, pin)    (reg |= (1<<pin))
#define clr_bit(reg, pin)    (reg &= ~(1<<pin))
#define toggle_bit(reg, pin) (reg ^= (1<<pin))
#define tst_bit(reg, pin)    (reg & (1<<pin))


#define DISPLAY1 PC5
#define DISPLAY2 PC4
#define DISPLAY3 PC3
#define DISPLAY4 PC2

void setup(void);
void loop(void);
void f_timers(void);
void read_keyb(void);
void mux_display(void);
void change_display(void);
void cpwm(void);

uint8_t cpwm1 = 0, cpwm2 = 0, display1 = 0, display2 = 0, display3 = 0, display4 = 0;
int16_t duty_cicle1 = 0, duty_cicle2 = 0;

uint8_t hex_numbers[] = 
{
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01100111, // 9
};

ISR(PCINT1_vect) //houve alteracao de nivel logico
{
  read_keyb();
}

ISR(TIMER1_OVF_vect)
{
  TCNT1 = 65536;
  f_timers();
}

int main()
{
  setup();

  while(1) loop();
}

void setup(void)
{
  cli(); //desabilta a chave geral de interrupcao
  DDRD   = 0b11111111;  //PD2 e PD3 como entrada
  DDRB   = 0b00111110; 
  DDRC = 0b01111100; //PC0 e PC1 como entradas
  PORTD  = 0;  //inicia apagado
  PORTB  = 0;
  PORTC = 0;
  TCNT1 = 65536;
  TIMSK1 = 0x01; //habilita a interrupcao do timer 1
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011001; //PWM fast mode de 10 bits comprescaler de 8
  PCICR = 0x02; //00000010
  PCMSK1 = (1<<PC0) | (1<<PC1); //habilito a interrupcao na porta PC0
  ICR1 = 3999;
  sei(); //habilta a chave geral de interrupcao
}

void loop(void)
{
}

void f_timers(void)
{
  static uint16_t counter0 = 1, counter1 = 1, counter2 = 1;

  if(counter0 < 10)
  {
    counter0++;
  }

  else
  {
    cpwm();
    counter0 = 1;
  }

  if(counter1 < 10)
  {
    counter1++;
  }

  else
  {
    change_display();
    counter1 = 1;
  }

  if(counter2 < 10)
  {
    counter2++;
  }

  else
  {
    mux_display();
    counter2 = 1;
  }
}

void read_keyb(void)
{
  static uint8_t memory_button1 = 1, button1 = 0, memory_button2 = 1, button2 = 0;

  if(tst_bit(PINC, PC0))
  {
      button1 = 1;
  }

  else
  {
      button1 = 0;
  }

  if(tst_bit(PINC, PC1))
  {
      button2 = 1;
  }

  else
  {
      button2 = 0;
  }

  if(button1 < memory_button1)
  {   
      cpwm1 ^= 1; 
  }

  if(button2 < memory_button2)
  {   
      cpwm2 ^= 1; 
  }

  memory_button1 = button1;
  memory_button2 = button2;
}

void change_display(void)
{

}

void cpwm(void) 
{
  if(cpwm1) 
  {
    duty_cicle1++;
    if (duty_cicle1 > ICR1) 
    duty_cicle1 = 0;
  }

  if(cpwm2) 
  {
    duty_cicle2++;
    if (duty_cicle2 > ICR1) 
    duty_cicle2 = 0;
  }
}

void mux_display(void)
{
  static uint8_t mux = 0;

    switch (mux)
    {
      case 0:
        PORTD = hex_numbers[display1];
        set_bit(PORTC, DISPLAY1);
        clr_bit(PORTC, DISPLAY4);
        mux = 1;
        break;
      
      case 1:
        PORTD = hex_numbers[display2];
        set_bit(PORTC, DISPLAY2);
        clr_bit(PORTC, DISPLAY1);
        mux = 2;
        break;

      case 2:
        PORTD = hex_numbers[display3];
        set_bit(PORTC, DISPLAY3);
        clr_bit(PORTC, DISPLAY2);
        mux = 3;
        break;

      case 3:
        PORTD = hex_numbers[display4];
        set_bit(PORTC, DISPLAY4);
        clr_bit(PORTC, DISPLAY3);
        mux = 0;
        break;
    }
}