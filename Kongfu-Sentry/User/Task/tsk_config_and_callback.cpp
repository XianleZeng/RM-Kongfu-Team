

//#include "tsk_config_and_callback.h"
//#include "usart.h"	


//extern UART_HandleTypeDef huart3;
//extern DMA_HandleTypeDef hdma_usart3_rx;
//extern RC_ctrl_t rc_ctrl;
//extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

///**
//  * @brief          remote control protocol resolution
//  * @param[in]      sbus_buf: raw data point
//  * @param[out]     rc_ctrl: remote control data struct point
//  * @retval         none
//  */
///**
//  * @brief          遥控器协议解析
//  * @param[in]      sbus_buf: 原生数据指针
//  * @param[out]     rc_ctrl: 遥控器数据指
//  * @retval         none
//  */
//static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);


////串口中断
//void UART3_Callback(uint8_t *Buffer, uint16_t Length)
//{
//    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart3);
//    }
//    else if(USART3->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart3);

//        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */
//    
//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart3_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//            //set memory buffer 1
//            //设定缓冲区1
//            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart3_rx);

//            if(this_time_rx_len == RC_FRAME_LENGTH)
//            {
////                HAL_UART_Transmit_IT(&huart1, sbus_rx_buf[0], 25);
//								sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
//							
//							
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart3_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//            //set memory buffer 0
//            //设定缓冲区0
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart3_rx);

//            if(this_time_rx_len == RC_FRAME_LENGTH)
//            {
//                //处理遥控器数据
////								HAL_UART_Transmit_DMA(&huart1, sbus_rx_buf[0], 25);
//                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
//							
//            }
//        }
//    }
//}

///**
//  * @brief          remote control protocol resolution
//  * @param[in]      sbus_buf: raw data point
//  * @param[out]     rc_ctrl: remote control data struct point
//  * @retval         none
//  */
///**
//  * @brief          遥控器协议解析
//  * @param[in]      sbus_buf: 原生数据指针
//  * @param[out]     rc_ctrl: 遥控器数据指
//  * @retval         none
//  */
//static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
//{
//    if (sbus_buf == NULL || rc_ctrl == NULL)
//    {
//        return;
//    }
//		rc_ctrl->rc.ch[0] = ((sbus_buf[1]|sbus_buf[2]<< 8) & 0x07FF);
//		rc_ctrl->rc.ch[1] = ((sbus_buf[2]>>3|sbus_buf[3]<<5) & 0x07FF);
//		rc_ctrl->rc.ch[2] = ((sbus_buf[3]>>6|sbus_buf[4]<<2|sbus_buf[5]<<10) & 0x07FF);
//		rc_ctrl->rc.ch[3] = ((sbus_buf[5]>>1|sbus_buf[6]<<7) & 0x07FF);
//		rc_ctrl->rc.ch[4] = ((sbus_buf[6]>>4|sbus_buf[7]<<4) & 0x07FF);

//    rc_ctrl->rc.s[0] = ((sbus_buf[7]>>7|sbus_buf[8]<<1|sbus_buf[9]<<9) & 0x07FF);                  //!< Switch left
//    rc_ctrl->rc.s[1] = ((sbus_buf[9]>>2|sbus_buf[10]<<6) & 0x07FF);                    //!< Switch right
//}




///**
// * @brief 初始化任务
// *
// */
//void Task_Init()
//{
//    UART_Init(&huart1, UART3_Callback, 25);
//    UART_Init(&huart2, Serialplot_UART2_Callback, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH);
//    UART_Init(&huart6, Referee_UART6_Callback, 128);
//    UART_Init(&huart7, AHRS_UART7_Callback, 11);
//}