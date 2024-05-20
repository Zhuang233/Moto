#include "uart_zbw.h"
#include "usart.h"
#include <stdbool.h>
// 同步数据
/*
瓴控前三轴位置uint16_t
横移位置int32_t 
前伸位置int32_t 
*/


// AC板通信协议
/*
帧头 0x55
中间8字节数据
帧尾 0xaa
*/




typedef struct {
    uint8_t buffer[FRAME_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
} RingBuffer;


RingBuffer uart_ring_buffer;
uint8_t data_rev_byte = 0;
uint8_t sync_data_to_a[SYNC_TO_A_SIZE] = {0};
DataUnion sync_data_from_a;




void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
		for(int i=0; i<FRAME_SIZE-2;i++){
			rb->buffer[i] = 0;
		}
		
}

bool RingBuffer_IsFull(RingBuffer *rb) {
    return rb->count == FRAME_SIZE;
}

bool RingBuffer_IsEmpty(RingBuffer *rb) {
    return rb->count == 0;
}

bool RingBuffer_Put(RingBuffer *rb, uint8_t data) {
    rb->buffer[rb->tail] = data;
    rb->tail = (rb->tail + 1) % FRAME_SIZE;
    rb->count++;
		rb->count = rb->count % (FRAME_SIZE+1);
    return true;
}

bool RingBuffer_Get(RingBuffer *rb, uint8_t *data) {
    if (RingBuffer_IsEmpty(rb)) {
        return false; // 缓冲区空
    }
    *data = rb->buffer[rb->head];
    rb->head = (rb->head + 1) % FRAME_SIZE;
    rb->count--;
    return true;
}

void decode_uart_rev_data(){
	uint8_t test_byte = 0;
	RingBuffer_Put(&uart_ring_buffer,data_rev_byte);
	if(data_rev_byte == FRAME_TAIL && uart_ring_buffer.count == FRAME_SIZE-1){
		RingBuffer_Get(&uart_ring_buffer,&test_byte);
		if(test_byte == FRAME_HEAD){
			for(int i=0; i<FRAME_SIZE-2;i++){
				RingBuffer_Get(&uart_ring_buffer,(sync_data_from_a.bytes+i));
			}
			RingBuffer_Get(&uart_ring_buffer,&test_byte);
		}
		else{
			RingBuffer_Init(&uart_ring_buffer);
		}
		
	}
}

void data_sync_uart(){
	uint8_t head = FRAME_HEAD;
	uint8_t tail = FRAME_TAIL;
	
	HAL_UART_Transmit(&huart1, &head, 1, 10);
	HAL_UART_Transmit(&huart1, sync_data_to_a, SYNC_TO_A_SIZE, 10);
	HAL_UART_Transmit(&huart1, &tail, 1, 10);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
	if(huart->Instance==USART1)
	{
		decode_uart_rev_data();
		HAL_UART_Receive_IT(&huart1,&data_rev_byte,1);//恢复接受中断
	}
}

void usart_init(){
	HAL_UART_Receive_IT(&huart1,&data_rev_byte,1);
	RingBuffer_Init(&uart_ring_buffer);
}