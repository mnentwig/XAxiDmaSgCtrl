#include "xtime_l.h" // for throughput calculation
#include <stdio.h> // using full-featured printf (large!)

#include "dmaFeedBasic.h"
int main(void){
	// identify interrupts (need DMA0 configured with interrupts, connected to PS via concat)
#ifdef XPAR_INTC_0_DEVICE_ID
	const u32 txIntrId = XPAR_INTC_0_AXIDMA_0_MM2S_INTROUT_VEC_ID;
	const u32 rxIntrId = XPAR_INTC_0_AXIDMA_0_S2MM_INTROUT_VEC_ID;
#else
	const u32 txIntrId = XPAR_FABRIC_AXIDMA_0_MM2S_INTROUT_VEC_ID;
	const u32 rxIntrId = XPAR_FABRIC_AXIDMA_0_S2MM_INTROUT_VEC_ID;
#endif
	printf("=== scatter-gather DMA wrapper test===\n");

	// top-level exception handler (all interrupts)
	dmaFeedBase::installGlobalIrqExceptionHandler();

	// memory for test
	int n = 10000000;
	u32* txBuf = (u32*)malloc(n*sizeof(u32)); assert(txBuf);
	u32* rxBuf = (u32*)malloc(n*sizeof(u32)); assert(rxBuf);
	for (int ix = 0; ix < n; ++ix)
		txBuf[ix] = ix;

	for (int testcase = 0; testcase < 11; ++testcase){
		// overwrite Rx buffer to detect data error
		memset(rxBuf, /*value*/0, /*nBytes*/n*sizeof(u32));

		int nTest = n;
		int packetSizeTest = 1 << 13;
		switch (testcase){
			case 0: default:
				packetSizeTest <<= 0;
				break;
			case 1:
				packetSizeTest >>= 1;
				break;
			case 2:
				packetSizeTest >>= 2;
				break;
			case 3:
				packetSizeTest >>= 3;
				break;
			case 4:
				packetSizeTest >>= 4;
				break;
			case 5:
				packetSizeTest >>= 5;
				break;
			case 6:
				packetSizeTest >>= 6;
				nTest >>= 1;
				break;
			case 7:
				packetSizeTest >>= 7;
				nTest >>= 2;
				break;
			case 8:
				packetSizeTest >>= 8;
				nTest >>= 3;
				break;
			case 9:
				packetSizeTest >>= 9;
				nTest >>= 4;
				break;
			case 10:
				packetSizeTest >>= 10;
				nTest >>= 5;
				break;
		} // switch

		// set up DMA wrapper for testing
		dmaFeedBasicConfig c(XPAR_AXIDMA_0_DEVICE_ID, txIntrId, rxIntrId);
		c.maxPacketSize = packetSizeTest;
		dmaFeedBasic d(c);

		unsigned int nTxBytesTest = nTest*sizeof(u32);
		unsigned int nRxBytesTest = nTest*sizeof(u32);

		u64 t1, t2;
		XTime_GetTime(&t1);
		d.runStart((char*)txBuf, nTxBytesTest, (char*)rxBuf, nRxBytesTest);
		dmaFeedBase::run_poll_e status;
		while (true){
			status = d.run_poll();
			if (status != dmaFeedBase::DMAFEED_BUSY)
				break;
			//usleep(1);
		} // while busy

		if (status == dmaFeedBase::DMAFEED_IDLE){
	 		XTime_GetTime(&t2);

			double t_s = (double)(t2-t1)/COUNTS_PER_SECOND;
			printf("%.3f us for %i bytes using packet size %u\n", 1e6*t_s, nTxBytesTest, packetSizeTest);
			double throughput_GBps = nTxBytesTest / t_s / 1e9;
			printf("throughput %.3f gigabytes per second\n", throughput_GBps);
			for (int ix = 0; ix < nTest; ++ix)
				if (rxBuf[ix] != txBuf[ix])
					printf("verify error at position %i: expected %08lx got %08lx\n", ix, txBuf[ix], rxBuf[ix]);
		} else if (status == dmaFeedBase::DMAFEED_IDLE_ERROR){
			printf("completed with DMA error\n");
		}
	}

	printf("Done\r\n");
	while (1){}

	return 0;
}
