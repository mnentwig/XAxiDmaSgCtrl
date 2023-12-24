#include "XAxiDmaSgCtrl.h"

#include "xtime_l.h"
#include <stdio.h>
#include <iostream>
int main(void){
#ifdef XPAR_INTC_0_DEVICE_ID
	const u32 txIntrId = XPAR_INTC_0_AXIDMA_0_MM2S_INTROUT_VEC_ID;
	const u32 rxIntrId = XPAR_INTC_0_AXIDMA_0_S2MM_INTROUT_VEC_ID;
#else
	const u32 txIntrId = XPAR_FABRIC_AXIDMA_0_MM2S_INTROUT_VEC_ID;
	const u32 rxIntrId = XPAR_FABRIC_AXIDMA_0_S2MM_INTROUT_VEC_ID;
#endif
	xil_printf("SG DMA test\r\n");
	dmaFeed::installGlobalIrqExceptionHandler();
	dmaFeedConfig c(XPAR_AXIDMA_0_DEVICE_ID, txIntrId, rxIntrId);
	//c.maxPacketSize = 4;
	dmaFeedSimple d(c);

	int n = 10000000;
	u32* txBuf = (u32*)malloc(n*sizeof(u32)); assert(txBuf);
	u32* rxBuf = (u32*)malloc(n*sizeof(u32)); assert(rxBuf);

	for (int ix = 0; ix < n; ++ix)
		txBuf[ix] = ix;

	unsigned int nTxBytes = n*sizeof(u32);
	unsigned int nRxBytes = n*sizeof(u32);

	u64 t1, t2;
	XTime_GetTime(&t1);
	d.runStart((char*)txBuf, nTxBytes, (char*)rxBuf, nRxBytes);
	dmaFeed::run_poll_e status;
	while (true){
		status = d.run_poll();
		if (status != dmaFeed::DMAFEED_BUSY)
			break;
		//usleep(1);
	}

	if (status == dmaFeed::DMAFEED_IDLE){
		XTime_GetTime(&t2);

		double t_s = (double)(t2-t1)/COUNTS_PER_SECOND;
		printf("dummy slave: %.3f us for %i bytes\r\n", 1e6*t_s, nTxBytes);
		double throughput_GBps = nTxBytes / t_s / 1e9;
		printf("dummy slave: throughput %.3f gigabytes per second\r\n", throughput_GBps);
		xil_printf("completed\r\n");
		for (int ix = 0; ix < n; ++ix)
			if (rxBuf[ix] != txBuf[ix])
				xil_printf("verify error at position %i: expected %08x got %08x\r\n", ix, txBuf[ix], rxBuf[ix]);
	} else if (status == dmaFeed::DMAFEED_IDLE_ERROR){
		xil_printf("completed with DMA error\r\n");
	}
	xil_printf("exiting\r\n");
	while (1){}

	return 0;
}
