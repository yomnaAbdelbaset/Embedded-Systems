#include <mega16.h>
#include <delay.h>
#include <alcd.h>

// some defines for LCD conditions ..

#define MOVING 0
#define COUNTING 1 
#define WEIGHT 2

//Global variables 

unsigned char w;
float weights[50];
int i=0;
unsigned char Flag=MOVING;

// External Interrupt 0 service routine

interrupt [EXT_INT0] void ext_int0_isr(void)
{
     if(PIND.2==0) //BACKAGE DETECTOR SWITCH PRESSED
     {
       PORTD.1=1;  //MOTOR 2 (FRUITS) ON
       PORTD.0=0;  //MOTOR 1 (PACKAGES) OFF
     }
     lcd_clear(); // clear LCD after "Moving Package"
     Flag=COUNTING; // start counting on the LCD
     
}
 
// Voltage Reference: AVCC pin
#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (0<<ADLAR))

// Read the AD conversion result

unsigned int read_adc(unsigned char adc_input)
{
  
  ADMUX=adc_input | ADC_VREF_TYPE;
  
  // Delay needed for the stabilization of the ADC input voltage
     delay_us(10);
  // Wait for the AD conversion to complete

    while ((ADCSRA & (1<<ADIF))==0);
    ADCSRA|=(1<<ADIF);
    return ADCW;
}                             

// Timer 0 output compare interrupt service routine
interrupt [TIM0_COMP] void timer0_comp_isr(void)
{        
      w=read_adc(0); // read from ADC to convert it later to Kg
      weights[i]=( (w*25.0)/256) + (25.0/256);   //STORE THE WEIGHT OF PACKAGE IN AN ARRAY   
      i++;
      lcd_clear(); // clear the LCD after counting 
      Flag=WEIGHT;  // start weighting
      PORTD.1=0;  // MOTOR 2 (FRIUTS) OFF
}


void main(void)
{
// Input/Output Ports initialization

// Port A initialization

// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRA=(0<<DDA7) | (0<<DDA6) | (0<<DDA5) | (0<<DDA4) | (0<<DDA3) | (1<<DDA2) | (0<<DDA1) | (0<<DDA0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTA=(0<<PORTA7) | (0<<PORTA6) | (0<<PORTA5) | (0<<PORTA4) | (0<<PORTA3) | (0<<PORTA2) | (0<<PORTA1) | (0<<PORTA0);

// Port B initialization

// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRB=(0<<DDB7) | (0<<DDB6) | (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=P 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (1<<PORTB0);

// Port C initialization

// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRC=(1<<DDC7) | (1<<DDC6) | (1<<DDC5) | (1<<DDC4) | (1<<DDC3) | (1<<DDC2) | (1<<DDC1) | (1<<DDC0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTC=(0<<PORTC7) | (0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization


// Function: Bit7=In Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=Out Bit0=Out 
DDRD=(0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (1<<DDD1) | (1<<DDD0);
// State: Bit7=T Bit6=T Bit5=T Bit4=T Bit3=T Bit2=P Bit1=0 Bit0=0 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (1<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: T0 pin Falling Edge
// Mode: CTC top=OCR0
// OC0 output: Disconnected
  
  TCCR0=(0<<WGM00) | (0<<COM01) | (0<<COM00) | (1<<WGM01) | (1<<CS02) | (1<<CS01) | (0<<CS00);
  TCNT0=0x00;
  OCR0=0x14;  // OCR0=20 
  
  PORTD.0=1; // MOTOR 1 (Packages) is ON initially 
  
// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (1<<OCIE0) | (0<<TOIE0);

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Falling Edge

GICR|=(0<<INT1) | (1<<INT0) | (0<<INT2);
MCUCR=(0<<ISC11) | (0<<ISC10) | (1<<ISC01) | (0<<ISC00);
MCUCSR=(0<<ISC2);
GIFR=(0<<INTF1) | (1<<INTF0) | (0<<INTF2);

// USART initialization
// USART disabled
UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);

// ADC initialization
// ADC Clock frequency: 500.000 kHz
// ADC Voltage Reference: AVCC pin
// ADC Auto Trigger Source: Timer0 Compare Match
ADMUX=ADC_VREF_TYPE;
ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADATE) | (0<<ADIF) | (0<<ADIE) | (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
SFIOR=(0<<ADTS2) | (1<<ADTS1) | (1<<ADTS0);


// Alphanumeric LCD initialization
// Connections are specified in the
// Project|Configure|C Compiler|Libraries|Alphanumeric LCD menu:
// RS - PORTC Bit 0
// RD - PORTC Bit 1
// EN - PORTC Bit 2
// D4 - PORTC Bit 4
// D5 - PORTC Bit 5
// D6 - PORTC Bit 6
// D7 - PORTC Bit 7
// Characters/line: 16

lcd_init(16);

// Globally enable interrupts
#asm("sei")

      lcd_puts("Moving package"); 
while (1)
      {

      if(Flag==COUNTING)
        {  
            lcd_gotoxy(0, 0);
            lcd_puts("Fruits = ");
            lcd_putchar( (TCNT0/10)+48 );
            lcd_putchar( (TCNT0%10)+48 );
        }
      else if(Flag==WEIGHT)
        {
            lcd_clear();   
            lcd_gotoxy(0, 0);
            lcd_puts("pack weight = ");
            lcd_putchar( ((int)weights[i-1]/10)+48);
            lcd_putchar( ((int)weights[i-1]%10)+48);
            delay_ms(2000);
            lcd_clear(); 
            PORTD.0=1;          //MOTOR 1 (PACKAGES) ON
            Flag=MOVING;
            lcd_puts("Moving package");
        }
      
      }
}
