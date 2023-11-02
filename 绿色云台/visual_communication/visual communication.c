#include "visual communication.h"
#include "usart.h"
uint8_t visual_data[4];
Visual_Data_ MY_;
extern DMA_HandleTypeDef hdma_uart4_rx;
void Visual_init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart4.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart4_rx);
    {
        __HAL_DMA_DISABLE(&hdma_uart4_rx);
    }
            hdma_uart4_rx.Instance->PAR = (uint32_t) & (UART4->DR);

    //memory buffer 1
    hdma_uart4_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //data length
    //数据长度
    hdma_uart4_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer

    __HAL_DMA_ENABLE(&hdma_uart4_rx);

}
void visual_receive(void)
{
	if(visual_data[0]==0xEE&&visual_data[3]==0xFF)
	{
		MY_.YAW=visual_data[1];
		MY_.PITCH=visual_data[2];
	}
	else
	{
		MY_.YAW=0;
		MY_.PITCH=0;
	}
}

/*视觉数据接收*/
void visual_data_receive(void)
{
	  uint8_t ucTemp;	
    __HAL_UART_CLEAR_PEFLAG(&huart4);
    __HAL_DMA_DISABLE(&hdma_uart4_rx);
	  hdma_uart4_rx.Instance->NDTR = 8;
		__HAL_DMA_ENABLE(&hdma_uart4_rx);
		visual_receive();

}













