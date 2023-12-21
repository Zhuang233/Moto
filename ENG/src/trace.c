#include "trace.h"
#include "trcRecorder.h"


#define CAN1_RX0_ISR_NAME "MotoAheadRevISR"	// 中断名称，用于在上位机显示
#define CAN1_RX0_ISR_PRIORITY 5							// 中断优先级
TraceISRHandle_t Can1Rx0ISRHandle;


#define MotoCurrentChannel "Moto current"
traceString MotoChannel;

void trace_init(void){
	
	vTraceEnable(TRC_START);// 使能Trace,必须在硬件完成初始化之后，第一个内核对象创建之前
	
	
	// 注册中断追踪（注意需要在vTraceEnable之后，否则相关结构体未初始化，上位名字显示如ISR #1的名字）
	xTraceISRRegister(CAN1_RX0_ISR_NAME, CAN1_RX0_ISR_PRIORITY, &Can1Rx0ISRHandle);
	
	MotoChannel = xTraceRegisterString(MotoCurrentChannel);
}

void trace_current(int current){
	vTracePrintF(MotoChannel, "Motor Current = %d", current);
}
