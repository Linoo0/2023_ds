#include "visual communication.h"
#include "usart.h"
uint8_t visual_data[12];
uint8_t visual_data2[14];
Visual_Data_ A_,B_,C_,D_,MY_;
Visual_Data_ MID_1,A_1,B_1,C_1,D_1,MY_1;
extern uint8_t KEY0,KEY1,KEY2,KEY3,KEY3_2,KEY3_Stop,KEY_RED,KEY_REDsign;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern uint8_t Txbuf_v[1];
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
extern uint8_t Follow_SIGN;
void visual_receive(void)
{
	if(visual_data[0]==0xEE&&visual_data[11]==0xFF)
	{
		if(Follow_SIGN==1)
		{
			A_.YAW=visual_data[1]+2;
			A_.PITCH=visual_data[2]+1;
			B_.YAW=visual_data[3]-3;
			B_.PITCH=visual_data[4]+2;
			C_.YAW=visual_data[5]-3;
			C_.PITCH=visual_data[6]-5;
			D_.YAW=visual_data[7]+3;
			D_.PITCH=visual_data[8]-3;
			MY_.YAW=visual_data[9];
			MY_.PITCH=visual_data[10];
		}
		else if(Follow_SIGN==2)
		{
			A_.YAW=visual_data[1];
			A_.PITCH=visual_data[2];
			B_.YAW=visual_data[3];
			B_.PITCH=visual_data[4];
			C_.YAW=visual_data[5];
			C_.PITCH=visual_data[6];
			D_.YAW=visual_data[7];
			D_.PITCH=visual_data[8];
			MY_.YAW=visual_data[9];
			MY_.PITCH=visual_data[10];
		}
		else
		{
			A_.YAW=visual_data[1];
			A_.PITCH=visual_data[2];
			B_.YAW=visual_data[3];
			B_.PITCH=visual_data[4];
			C_.YAW=visual_data[5];
			C_.PITCH=visual_data[6];
			D_.YAW=visual_data[7];
			D_.PITCH=visual_data[8];
			MY_.YAW=visual_data[9];
			MY_.PITCH=visual_data[10];
		}
	}

}
//void visual_receive2(void)
//{
//	if(visual_data2[0]==0xEE&&visual_data2[13]==0xFF)
//	{
//			MID_.YAW=visual_data[1];
//			MID_.PITCH=visual_data[2];
//	}
//}
/*视觉数据接收*/
void visual_data_receive(void)
{
	  uint8_t ucTemp;	
    __HAL_UART_CLEAR_PEFLAG(&huart4);
    __HAL_DMA_DISABLE(&hdma_uart4_rx);
	  hdma_uart4_rx.Instance->NDTR = 12;
		__HAL_DMA_ENABLE(&hdma_uart4_rx);
		visual_receive();
}

void TT(void)
{
//	if(KEY_RED==0&&KEY_REDsign==1)
//		{
//			MID_1.YAW=MY_.YAW;
//			MID_1.PITCH=MY_.PITCH;
//		}
//		else if(KEY_RED==1&&KEY_REDsign==1)
//		{
//			A_1.YAW=MY_.YAW;
//			A_1.PITCH=MY_.PITCH;
//		}
//		else if(KEY_RED==2&&KEY_REDsign==1)
//		{
//			B_1.YAW=MY_.YAW;
//			B_1.PITCH=MY_.PITCH;
//		}
//		else if(KEY_RED==3&&KEY_REDsign==1)
//		{
//			C_1.YAW=MY_.YAW;
//			C_1.PITCH=MY_.PITCH;
//		}
//		else if(KEY_RED==4&&KEY_REDsign==1)
//		{
//			D_1.YAW=MY_.YAW;
//			D_1.PITCH=MY_.PITCH;
//		}
		MID_1.YAW=113;
		MID_1.PITCH=115;
		A_1.YAW=38;
			A_1.PITCH=37;
		B_1.YAW=187;
			B_1.PITCH=46;
		C_1.YAW=182;
			C_1.PITCH=182;
		D_1.YAW=39;
			D_1.PITCH=187;
}











