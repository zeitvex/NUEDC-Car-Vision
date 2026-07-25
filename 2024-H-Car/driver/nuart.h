#ifndef __NUART_H
#define __NUART_H



void UART_SendBytes(UART_Regs *port,uint8_t *ubuf, uint32_t len);
void UART_SendByte(UART_Regs *port,uint8_t data);
int fputc(int ch, FILE *f);
void usart_irq_config(void);
void usart1_send_bytes(unsigned char *buf, int len);


#endif





//void UART_1_INST_IRQHandler(void)
//{
//  switch (DL_UART_Main_getPendingInterrupt(UART_1_INST))
//  {
//    case DL_UART_MAIN_IIDX_RX:
//    {
//      uint8_t ch = DL_UART_Main_receiveData(UART_1_INST);
//      //DL_UART_Main_transmitData(UART_1_INST, ch);
//    }
//    break;

//    default:
//      break;
//  }
//}

//void UART_2_INST_IRQHandler(void)
//{
//  switch (DL_UART_Main_getPendingInterrupt(UART0))
//  {
//    case DL_UART_MAIN_IIDX_RX:
//    {
//      uint8_t ch = DL_UART_Main_receiveData(UART_2_INST);
//      //DL_UART_Main_transmitData(UART_2_INST, ch);
//    }
//    break;

//    default:
//      break;
//  }
//}

//void UART_3_INST_IRQHandler(void)
//{
//  switch (DL_UART_Main_getPendingInterrupt(UART_3_INST))
//  {
//    case DL_UART_MAIN_IIDX_RX:
//    {
//      uint8_t ch = DL_UART_Main_receiveData(UART_3_INST);
//      //DL_UART_Main_transmitData(UART_3_INST, ch);
//    }
//    break;

//    default:
//      break;
//  }
//}



