#include "dmaFeedBasic.h"
dmaFeedBasic::dmaFeedBasic(const dmaFeedBasicConfig& config) : dmaFeedBase(config), maxPacketSize(config.maxPacketSize){
	// note: config fields added by this custom class (currently only one) are
}

unsigned int dmaFeedBasic::bytesToBufs(unsigned int nBytes, unsigned int nBufAvailable) const {
	u32 nBuf = nBytes / maxPacketSize;
	u32 nBytesRem = nBytes - nBuf * maxPacketSize;
	if (nBytesRem > 0)
		++nBuf;
	return (nBuf > nBufAvailable) ? nBufAvailable : nBuf;
}

void dmaFeedBasic::collectTx()/*override*/{
	// === get completed BDs ===
	// note: this will be empty on startup
	XAxiDma_Bd *firstBdPtr; // first buffer descriptor in returned set
	int nBd = XAxiDma_BdRingFromHw(txRingPtr, /*no limit to the number of returned BDs*/XAXIDMA_ALL_BDS, &firstBdPtr);
	XAxiDma_Bd *itBdPtr = firstBdPtr;
	//xil_printf("tx int callback status: %08x with %i BDs\r\n", irqStatus, nBd);

	// === count transmitted bytes ===
	unsigned int numNewBytesTransmitted = 0;
	int bdCount = nBd;
	while (bdCount--){
		u32 bdStatus = XAxiDma_BdGetSts(itBdPtr);
		assert(!(bdStatus & XAXIDMA_BD_STS_ALL_ERR_MASK));
	    assert(bdStatus & XAXIDMA_BD_STS_COMPLETE_MASK);
	    numNewBytesTransmitted += XAxiDma_BdGetLength(itBdPtr, txRingPtr->MaxTransferLen);
		itBdPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(txRingPtr, itBdPtr);
	}

	// === return completed BDs to pool ===
	int s = XAxiDma_BdRingFree(txRingPtr, nBd, firstBdPtr); assert(s == XST_SUCCESS && "XAxiDma_BdRingFree(Tx) failed");

	// === detect end of (fixed length) transmission ===
	assert(numNewBytesTransmitted <= nTxBytesRemainingToComplete);
	nTxBytesRemainingToComplete -= numNewBytesTransmitted;

	if (!nTxBytesRemainingToComplete){
		txDone = true;
		if (txDone && rxDone)
			done();
	}
}

void dmaFeedBasic::collectRx()/*override*/{
	// === get completed BDs ===
	// note: this will be empty on startup
	XAxiDma_Bd *firstBdPtr; // first buffer descriptor in returned set
	int nBd = XAxiDma_BdRingFromHw(rxRingPtr, /*no limit to the number of returned BDs*/XAXIDMA_ALL_BDS, &firstBdPtr);
	XAxiDma_Bd *itBdPtr = firstBdPtr;

	// === count received bytes ===
	unsigned int numNewBytesReceived = 0;
	int bdCount = nBd;
	while (bdCount--){
		u32 bdStatus = XAxiDma_BdGetSts(itBdPtr);
		assert(!(bdStatus & XAXIDMA_BD_STS_ALL_ERR_MASK));
	    assert(bdStatus & XAXIDMA_BD_STS_COMPLETE_MASK);
	    numNewBytesReceived += XAxiDma_BdGetLength(itBdPtr, rxRingPtr->MaxTransferLen);
		itBdPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(rxRingPtr, itBdPtr);
	}

	// === return completed BDs to pool ===
	int s = XAxiDma_BdRingFree(rxRingPtr, nBd, firstBdPtr); assert(s == XST_SUCCESS && "XAxiDma_BdRingFree(Rx) failed");

	// === detect end of reception ===
	assert(numNewBytesReceived <= nRxBytesRemainingToComplete);
	nRxBytesRemainingToComplete -= numNewBytesReceived;

	if (!nRxBytesRemainingToComplete){
		rxDone = true;
		if (txDone && rxDone)
			done();
	}
}

void dmaFeedBasic::queue(bool txEvent, bool rxEvent)/*override*/{
	if (txEvent)
		queueTx();
	if (rxEvent)
		queueRx();
}

void dmaFeedBasic::queueTx(){
	// number of available (idle) buffers
	int nFreeBd = XAxiDma_BdRingGetFreeCnt(txRingPtr); assert(nFreeBd && "queueTx() should never encounter zero idle buffers");
	// number of buffers to queue
	unsigned int nBufsToQueue = bytesToBufs(nTxBytesRemainingToQueue, /*limit to*/nFreeBd);
	//xil_printf("queueTx got %i free Bds need %i\r\n", nFreeBd, nBufsToQueue);

	int s;
	if (nBufsToQueue > 0){
		XAxiDma_Bd *firstBdPtr; // first buffer descriptor in allocated set
		s = XAxiDma_BdRingAlloc(txRingPtr, nBufsToQueue, &firstBdPtr); assert (s == XST_SUCCESS && "DMA queueTx: BdRingAlloc() failed");
		XAxiDma_Bd* itBdPtr = firstBdPtr; // buffer descriptor iterating over allocated set

		unsigned int count = nBufsToQueue;
		bool isFirstBd = true;
		while (count--){
			bool isLastBd = !count;
			int thisBufNBytes = maxPacketSize < nTxBytesRemainingToQueue ? maxPacketSize : nTxBytesRemainingToQueue;
			assert (thisBufNBytes); // nBufsToQueue calculation makes certain this never becomes 0

			// === assign next chunk of Tx data to Bd ===
			s = XAxiDma_BdSetBufAddr(itBdPtr, (UINTPTR)txPtr); assert(s == XST_SUCCESS && "DMA feedTx: BdSetBufAddr() failed");
			s = XAxiDma_BdSetLength(itBdPtr, thisBufNBytes, txRingPtr->MaxTransferLen); assert(s == XST_SUCCESS && "DMA feedTx: BdSetLength() failed");

			// === flag first and last BD ===
			u32 crBits = 0;
			if (isFirstBd){
				isFirstBd = false;
				crBits |= XAXIDMA_BD_CTRL_TXSOF_MASK;
			}
			if (isLastBd){
				crBits |= XAXIDMA_BD_CTRL_TXEOF_MASK;
			}
			XAxiDma_BdSetCtrl(itBdPtr, crBits);

			// === set arbitrary ID ===
			XAxiDma_BdSetId(itBdPtr, txPtr);

			// === next ... ===
			itBdPtr = (XAxiDma_Bd*)XAxiDma_BdRingNext(txRingPtr, itBdPtr); assert(itBdPtr);

			txPtr += thisBufNBytes;
			nTxBytesRemainingToQueue -= thisBufNBytes;
		} // for all bufs to queue

		// === submit to hardware ===
		s = XAxiDma_BdRingToHw(txRingPtr, nBufsToQueue, firstBdPtr); assert (s == XST_SUCCESS && "DMA queueTx: BdRingToHw() failed");
	} // if bufs to queue
}

void dmaFeedBasic::queueRx(){
	int nFreeBd = XAxiDma_BdRingGetFreeCnt(rxRingPtr); assert(nFreeBd && "queueRx() should never encounter zero idle buffers");

	// number of buffers to queue
	unsigned int nBufsToQueue = bytesToBufs(nRxBytesRemainingToQueue, /*limit to*/nFreeBd);
	//xil_printf("queueRx got %i free Bds need %i\r\n", nFreeBd, nBufsToQueue);

	int s;
	if (nBufsToQueue > 0){
		XAxiDma_Bd* firstBdPtr;
		s = XAxiDma_BdRingAlloc(rxRingPtr, nBufsToQueue, &firstBdPtr); assert (s == XST_SUCCESS && "DMA queueRx: BdRingAlloc() failed");
		XAxiDma_Bd* itBdPtr = firstBdPtr;

		for (unsigned int ix = 0; ix < nBufsToQueue; ++ix){
			s = XAxiDma_BdSetBufAddr(itBdPtr, (UINTPTR)rxPtr); assert (s == XST_SUCCESS && "DMA queueRx: BdSetBufAddr() failed");
			int n = maxPacketSize < nRxBytesRemainingToQueue ? maxPacketSize : nRxBytesRemainingToQueue;
			s = XAxiDma_BdSetLength(itBdPtr, n, rxRingPtr->MaxTransferLen); assert (s == XST_SUCCESS && "DMA queueRx: BdSetLength() failed");

			XAxiDma_BdSetCtrl(itBdPtr, 0); // unnecessary (HW will set)
			XAxiDma_BdSetId(itBdPtr, rxPtr); // assign arbitrary ID

			// === next ... ===
			rxPtr += n;
			nRxBytesRemainingToQueue -= n;
			itBdPtr = (XAxiDma_Bd*)XAxiDma_BdRingNext(rxRingPtr, itBdPtr); assert(itBdPtr);
		}

		s = XAxiDma_BdRingToHw(rxRingPtr, nBufsToQueue, firstBdPtr); assert (s == XST_SUCCESS && "DMA queueRx: BdRingToHw() failed");
	} // if bufs to queue
}

void dmaFeedBasic::runStart(char* txBuf, u32 numTxBytes, char* rxBuf, u32 numRxBytes){
	assert(((uintptr_t)txBuf & 3) == 0); // check alignment
	assert(((uintptr_t)rxBuf & 3) == 0); // check alignment
	txPtr = txBuf;
	rxPtr = rxBuf;
	nTxBytesRemainingToQueue = numTxBytes;
	nRxBytesRemainingToQueue = numRxBytes;
	nTxBytesRemainingToComplete = numTxBytes;
	nRxBytesRemainingToComplete = numRxBytes;

	// DMA doesn't go through cache => must flush
	Xil_DCacheFlushRange((INTPTR)txBuf, numTxBytes);
	Xil_DCacheFlushRange((INTPTR)rxBuf, numRxBytes);
	dmaFeedBase::runStart();
}
