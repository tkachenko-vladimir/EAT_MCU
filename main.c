#include "stm8s.h"
#include "iostm8s003f3.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "osa.h"
#include "stm8s_adc1.h"
#include "stm8s_tim2.h"
#include "stm8s_tim4.h"

#define OUT1 PD_ODR_bit.ODR4
#define OUT2 PD_ODR_bit.ODR3
#define OUT3 PD_ODR_bit.ODR2
#define OUT4 PC_ODR_bit.ODR7
#define SIMOFF PC_ODR_bit.ODR4
#define SIMBTN PC_ODR_bit.ODR3
#define SIMSTA PA_IDR_bit.IDR3
#define RFIN PA_IDR_bit.IDR1

void    Task1(void);
void    Task2(void);
void    Task3(void);
void    Task4(void);

char ib[30], tmp_buf[100];
char in_byte;
char * ibp1, * ibp2, * ibp3, * ibp4, * ibp1r, * ibp2r;
unsigned char ibp;
unsigned char gsm_busy = 0, simcom_on = 0, gsm_reg = 0, status = 0x0F;
unsigned char ret_err = 0, simof_cnt1 = 0, gsm_fun = 0, sendsms_f = 0, offsim_f = 0;
unsigned char menu_mode = 0;
char * CMD2, * ANS2, * cmd;
unsigned char WaitAnsTf = 0, SendCmdTf = 0, cnt1, cnt2, cnt3;
unsigned int TM2, sms_nbr;
volatile unsigned int timer1 = 0, timer2 = 0, timer3 = 0, timer4 = 0, timer5 = 0;
volatile unsigned int offsim_cnt = 0;
volatile unsigned char smsread_cnt = 0, netchk_cnt = 0, blink1 = 0;
unsigned char simof_cnt3 = 0, simof_cnt2 = 0, senderror_cnt = 0;
unsigned char inputs = 0, rssi = 0;
unsigned int i, Vcc, Vbat, tmp_cnt = 0;
signed char Tm = 0;
unsigned char Vbat_tmp;
volatile unsigned char in_sms = 0, in_rng = 0, dtmf_cmd = 0;
volatile unsigned char outputs = 0, sim800_ok = 0, offon = 0;
unsigned char outputs1 = 0, outputs_pos = 0;
u32 rf_cmd = 0, rf_cmd1a = 0, rf_cmd1b = 0;
volatile u8 rf_cmd1 = 0;
u8 bit_cnt = 0, byte_cnt = 0;

#define SIMCOM_OFF {if(SIMSTA){SIMBTN=1;OS_Delay(4000);SIMBTN=0;}SIMOFF = 0;}

#pragma location=0x004000
__no_init unsigned char eeprom_flag;
#pragma location=0x004001
__no_init volatile unsigned long outputs_s[40];

void uart1_sendbyte(char data)
{
    while(!UART1_SR_bit.TXE);
    UART1_DR = data;
}

void
set_outputs()
{
  if(outputs & 1)
    OUT1 = 1;
  else
    OUT1 = 0;
  if(outputs & 2)
    OUT2 = 1;
  else
    OUT2 = 0;
  if(outputs & 4)
    OUT3 = 1;
  else
    OUT3 = 0;
  if(outputs & 8)
    OUT4 = 1;
  else
    OUT4 = 0;
}

void init_gpio(void)
{
  PA_DDR_bit.DDR1 = 0;
  PA_CR1_bit.C11 = 0;
  PA_CR2_bit.C21 = 1;
  EXTI_CR1_PAIS = 3;      //  Interrupt on falling edge.
  EXTI_CR2_TLIS = 0;      //  Falling edge only.
  
  PA_DDR_bit.DDR2 = 1;
  PA_CR1_bit.C12 = 1;
  PA_CR2_bit.C22 = 1;

  PA_DDR_bit.DDR3 = 0;//STATUS
  PA_CR1_bit.C13 = 0;
  PA_CR2_bit.C23 = 0;

  PB_DDR_bit.DDR4 = 1;
  PB_CR1_bit.C14 = 1;
  PB_CR2_bit.C24 = 1;

  PB_DDR_bit.DDR5 = 1;
  PB_CR1_bit.C15 = 1;
  PB_CR2_bit.C25 = 1;
  
  PC_DDR_bit.DDR3 = 1;//PWRKEY
  PC_CR1_bit.C13 = 1;
  PC_CR2_bit.C23 = 1;

  PC_DDR_bit.DDR4 = 1;//SIM_OFF
  PC_CR1_bit.C14 = 1;
  PC_CR2_bit.C24 = 1;

  PC_DDR_bit.DDR5 = 1;
  PC_CR1_bit.C15 = 1;
  PC_CR2_bit.C25 = 1;

  PC_DDR_bit.DDR6 = 1;
  PC_CR1_bit.C16 = 1;
  PC_CR2_bit.C26 = 1;

  PC_DDR_bit.DDR7 = 1;//OUT4
  PC_CR1_bit.C17 = 1;
  PC_CR2_bit.C27 = 1;

  PD_DDR_bit.DDR1 = 1;
  PD_CR1_bit.C11 = 1;
  PD_CR2_bit.C21 = 1;

  PD_DDR_bit.DDR2 = 1;//OUT3
  PD_CR1_bit.C12 = 1;
  PD_CR2_bit.C22 = 1;

  PD_DDR_bit.DDR3 = 1;//OUT2
  PD_CR1_bit.C13 = 1;
  PD_CR2_bit.C23 = 1;

  PD_DDR_bit.DDR4 = 1;//OUT1
  PD_CR1_bit.C14 = 1;
  PD_CR2_bit.C24 = 1;

  PD_DDR_bit.DDR5 = 1;//TX
  PD_CR1_bit.C15 = 1;
  PD_CR2_bit.C25 = 1;

  PD_DDR_bit.DDR6 = 0;//RX
  PD_CR1_bit.C16 = 0;
  PD_CR2_bit.C26 = 0;
}

void init_timers(void)
{
/*
  TIM2_PSCR = 0x04; // Делитель 16
  TIM2_ARRH = 0xc3; // Старший байт числа 50000
  TIM2_ARRL = 0x50; // Младший байт числа 50000
  TIM2_IER_bit.UIE = 1; //Включаем флаг обновления таймера
  TIM2_CR1_bit.CEN = 1; //И окончательно включаем таймер

  TIM4_SR_bit.UIF = 0;
  TIM4_PSCR = 7;
  TIM4_ARR = 124;
  TIM4_IER_bit.UIE = 1;
  TIM4_CR1_bit.CEN = 1;
*/
  TIM2_TimeBaseInit(TIM4_PRESCALER_128, 249);//2mS
  TIM2_ClearFlag(TIM2_FLAG_UPDATE);
  TIM2_UpdateRequestConfig(TIM2_UPDATESOURCE_REGULAR);
  TIM2_ITConfig(TIM2_IT_UPDATE,ENABLE);
  TIM2_Cmd(ENABLE);

  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 124);//1mS
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  TIM4_UpdateRequestConfig(TIM4_UPDATESOURCE_REGULAR);
  TIM4_ITConfig(TIM4_IT_UPDATE,ENABLE);  
  TIM4_Cmd(ENABLE);
}

void Init_UART1()
{
  UART1_BRR2 = 0x03; //скорость 9600bps
  UART1_BRR1 = 0x68;
//  UART1_BRR2 = 0x0B; //скорость 115200bps
//  UART1_BRR1 = 0x08;
  UART1_CR2_bit.REN = 1;
  UART1_CR2_bit.TEN = 1;
  UART1_CR2_RIEN = 1;
}

void InitialiseIWDG()
{
  IWDG_KR = 0xCC;         //  Start the independent watchdog.
  IWDG_KR = 0x55;         //  Allow the IWDG registers to be programmed.
  IWDG_PR = 0x06;         //  Prescaler is 2 => each count is 250uS
  IWDG_RLR = 0xFF;        //  Reload counter.
  IWDG_KR = 0xAA;         //  Reset the counter.
}

int
main(void)
{
  CLK_DeInit();
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSI, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);;
//  CLK_ECKR_bit.HSEEN = 1;
//  CLK_SWCR_bit.SWEN=1;
//  while(CLK_ECKR_bit.HSERDY != 1);
//  CLK_CKDIVR = 0;
//  CLK_SWR = 0xB4;
//  while(CLK_SWCR_bit.SWIF != 1);

  init_gpio();

  SIMOFF = 0;
  SIMBTN = 0;

  OS_Init();
  init_timers();
  Init_UART1();

  FLASH_DUKR = 0xAE;
  FLASH_DUKR = 0x56; 
  if(eeprom_flag != 0xA2)
  {
    eeprom_flag = 0xA2;
    outputs = 0;
    outputs_pos = 0;
    outputs_s[0] = 0;outputs_s[1] = 0;outputs_s[2] = 0;outputs_s[3] = 0;outputs_s[4] = 0;
    outputs_s[5] = 0;outputs_s[6] = 0;outputs_s[7] = 0;outputs_s[8] = 0;outputs_s[9] = 0;
    outputs_s[10] = 0;outputs_s[11] = 0;outputs_s[12] = 0;outputs_s[13] = 0;outputs_s[14] = 0;
    outputs_s[15] = 0;outputs_s[16] = 0;outputs_s[17] = 0;outputs_s[18] = 0;outputs_s[19] = 0;
    outputs_s[20] = 0;outputs_s[21] = 0;outputs_s[22] = 0;outputs_s[23] = 0;outputs_s[24] = 0;
    outputs_s[25] = 0;outputs_s[26] = 0;outputs_s[27] = 0;outputs_s[28] = 0;outputs_s[29] = 0;
    outputs_s[30] = 0;outputs_s[31] = 0;outputs_s[32] = 0;outputs_s[33] = 0;outputs_s[34] = 0;
    outputs_s[35] = 0;outputs_s[36] = 0;outputs_s[37] = 0;outputs_s[38] = 0;outputs_s[39] = 0;
  }
  else
  {
    outputs_pos = 0;
    while(outputs_s[outputs_pos+20] == 100000)
      outputs_pos++;
    outputs = outputs_s[outputs_pos];
  }
  set_outputs();

  __enable_interrupt();

  OS_Task_Create(0, Task1);
  OS_Task_Create(0, Task2);
  OS_Task_Create(0, Task3);
  OS_Task_Create(0, Task4);
//  InitialiseIWDG();
  
  while(1)
  {
    OS_Run();
  };
}

void Task1(void)
{
  for(;;)
  {
    if(!(SIMSTA) || (offon == 1))
    {
      offon = 0;
      SIMOFF = 0;
      OS_Delay(3000);
      SIMOFF = 1;
      OS_Delay(1000);
      timer1 = 5000;
      SIMBTN = 1;
      OS_Delay(4000);
      SIMBTN = 0;
    }
    OS_Delay(20000);
  }
}

void Task2(void)
{
  for(;;)
  {
    sim800_ok = 0;
    OS_Delay(20000);
    if(SIMSTA && (sim800_ok == 0))
    {
      SIMCOM_OFF
    }
    OS_Yield();
  }
}

void Task3(void)
{
  for(;;)
  {
    IWDG_KR = 0xAA;
    OS_Delay(20);
  }
}

void Task4(void)
{
  for(;;)
  {
    uart1_sendbyte('\n');
    uart1_sendbyte('A');
    uart1_sendbyte('1');
    uart1_sendbyte('2');
    uart1_sendbyte('3');
    uart1_sendbyte('\n');
    OS_Delay(1000);
  }
}

INTERRUPT_HANDLER(TIM2_handler, ITC_IRQ_TIM2_OVF)
{
  TIM2_SR1_bit.UIF = 0;
  rf_cmd = 0;
  bit_cnt = 0;
}

INTERRUPT_HANDLER(TIM4_handler, ITC_IRQ_TIM4_OVF)
{
  TIM4_SR_bit.UIF = 0;
  OS_Timer();
  if(timer1 != 0)
    timer1--;
  if(timer2 != 0)
    timer2--;
  if(timer3 != 0)
    timer3--;
  if(timer4 != 0)
    timer4--;
  if(timer5 != 0)
    timer5--;
  tmp_cnt++;
  if(tmp_cnt == 1000)
  {
    tmp_cnt = 0;
  }
}

INTERRUPT_HANDLER(UART1_RX, 18)
{
  if(UART1_SR_bit.OR_LHE)
  {
    UART1_SR_bit.OR_LHE = 0;
    UART1_SR_bit.RXNE = 0;
  }
  else
  {
    in_byte = UART1_DR;
    if(ibp == 30)
      ibp = 0;
    if(in_byte > 0x1F)
    {
      ib[ibp++] = in_byte;
    }
    else
    {
      if(in_byte == 0x0D)
      {
        if(ibp != 0)
        {
          ib[ibp++] = 0;
          ibp = 0;
          if(ib[0] == 'O')
          {
            sim800_ok = 1;
            outputs = atoi(&ib[1]);
            outputs1 = outputs;
            if(outputs_s[outputs_pos] != outputs1)
            {
              outputs_s[outputs_pos+20]++;
              if(outputs_s[outputs_pos+20] == 100000)
                outputs_pos++;
              outputs_s[outputs_pos] = outputs;
            }
            if(outputs & 1)
              OUT1 = 1;
            else
              OUT1 = 0;
            if(outputs & 2)
              OUT2 = 1;
            else
              OUT2 = 0;
            if(outputs & 4)
              OUT3 = 1;
            else
              OUT3 = 0;
            if(outputs & 8)
              OUT4 = 1;
            else
              OUT4 = 0;
          }
          if(ib[0] == 'R')
          {
            offon = 1;
          }
        }
      }
    }
  }
}
INTERRUPT_HANDLER(EXTI_PORTA, ITC_IRQ_PORTA)
{
  if(RFIN == 0)
  {
    rf_cmd = rf_cmd << 1;
    if(TIM2_GetCounter() > 50)
      rf_cmd++;
    bit_cnt++;
  }
  if(bit_cnt == 24)
  {
    bit_cnt = 0;
    switch(byte_cnt)
    {
    case 0:
      {
        rf_cmd1a = rf_cmd;
        byte_cnt++;
        break;
      }
    case 1:
      {
        rf_cmd1b = rf_cmd;
        if((rf_cmd1a == rf_cmd1b) && (rf_cmd != 0))
          rf_cmd1 = (u8)rf_cmd1a;
        byte_cnt = 0;
      }
    }
  }
  TIM2_SetCounter(0);
}
/*
#pragma vector = TIM1_OVR_UIF_vector
__interrupt void TIM1_OVR_UIF_handler(void)
{
  // Проверка, что же вызвало прерывание
  if (TIM1_SR1_UIF==1)
  {
    TIM1_SR1_UIF = 0;             // Очистка флага прерывания по обновлению
  }  
}
*/
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    nop();
  }
}
#endif