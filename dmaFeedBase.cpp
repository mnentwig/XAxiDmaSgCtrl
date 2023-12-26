#include "dmaFeedBase.h"

#ifndef DEBUG
extern void xil_printf(const char *format, ...);
#endif

#ifdef DMAFEED_HAS_INTC
	XIntc dmaFeedBase::iIntc;
#endif
#ifdef DMAFEED_HAS_SCUGIC
	XScuGic dmaFeedBase::iIntc;
#endif

dmaFeedBase::dmaFeedBase(const dmaFeedBaseConfig& config) : config(config){
	// === allocate memory for buffer descriptor rings ===

#ifdef __aarch64__
	nBytesAllocTxBd = 0x10000; // DMA SG sample code uses 64k. TODO: Should use 1M
	nBytesAllocRxBd = 0x10000; // DMA SG sample code uses 64k. TODO: Should use 1M

	// need to disable cache, as DMA is not IO coherent (https://docs.xilinx.com/r/en-US/ug1085-zynq-ultrascale-trm/Full-Coherency)
	// translation table resolution is 2 MB for the first 32 bits, then 4 GB (https://docs.xilinx.com/r/2021.1-English/oslib_rm/Xil_SetTlbAttributes)
	// So we reserve an aligned 2 MB chunk prevent that unrelated data gets alloc'd into the same region with cache disabled
	u32 twoMB =  0x200000;
	assert(twoMB >= nAllocTxBD + nAllocRxBD);
	bufferDescriptorSpace = aligned_alloc(/*alignment*/twoMB, /*size*/twoMB);
	assert(((uintptr_t)bufferDescriptorSpace < 0x100000000) && "aligned_alloc returned memory beyond 32 bit address space. Refusing to disable cache for a whole gigabyte section...");
	Xil_SetTlbAttributes(bufferDescriptorSpace, /*MARK_UNCACHEABLE*/0x701);
#else
	nBytesAllocTxBd = 0x10000; // DMA SG sample code uses 64k
	nBytesAllocRxBd = 0x10000; // DMA SG sample code uses 64k

	// use consistent approach with single free() as in aarch64 above.
	// Correct alignment already here isn't strictly necessary.
	bufferDescriptorSpace = aligned_alloc(XAXIDMA_BD_MINIMUM_ALIGNMENT, nBytesAllocTxBd+nBytesAllocRxBd); // space for Tx and Rx. Only this gets free()d
	assert(bufferDescriptorSpace && "aligned_alloc failed");
#endif
	// split single alloc into Tx and Rx space
	txBdBufSpace = (void*)((char*)bufferDescriptorSpace + /*byte (aka char) offset*/0 );
	rxBdBufSpace = (void*)((char*)bufferDescriptorSpace + /*byte (aka char) offset*/nBytesAllocTxBd);

	acquireBDRings();
}

void dmaFeedBase::acquireBDRings(){
	if (BDRingsAreUp)
		return;

	int s; // status return value
	XAxiDma_Bd bdTemplate;
	XAxiDma_BdClear(&bdTemplate);

	// calculate number of BDs that fit into space
	u32 nTxBd = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT, nBytesAllocTxBd);
	u32 nRxBd = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT, nBytesAllocRxBd);

	XAxiDma_Config* dmaConf = XAxiDma_LookupConfig(config.dmaDevId);
	assert(dmaConf && "failed to locate DMA");

	XAxiDma_CfgInitialize(&iDma, dmaConf);
	assert(XAxiDma_HasSg(&iDma) && "need scatter-gather DMA, got simple mode\r\n");

	txRingPtr = XAxiDma_GetTxRing(&iDma);
	rxRingPtr = XAxiDma_GetRxRing(&iDma);

	interruptsDmaOnOff(false); // sample code explicitly disables BdRing interrupts before config

	// === Tx ===
	s = XAxiDma_BdRingCreate(txRingPtr, /*phys. address*/(UINTPTR)txBdBufSpace, /*virt. address*/(UINTPTR)txBdBufSpace, XAXIDMA_BD_MINIMUM_ALIGNMENT, nTxBd);
	assert (s == XST_SUCCESS && "DMA Tx BdRingCreate() failed");

	s = XAxiDma_BdRingClone(txRingPtr, &bdTemplate);
	assert (s == XST_SUCCESS && "DMA Tx BdRingClone() failed");

	// === RX ===
	s = XAxiDma_BdRingCreate(rxRingPtr, /*phys. address*/(UINTPTR)rxBdBufSpace, /*virt. address*/(UINTPTR)rxBdBufSpace, XAXIDMA_BD_MINIMUM_ALIGNMENT, nRxBd);
	assert (s == XST_SUCCESS && "DMA Rx BdRingCreate() failed");

	s = XAxiDma_BdRingClone(rxRingPtr, &bdTemplate);
	assert (s == XST_SUCCESS && "DMA Rx BdRingClone() failed");
	BDRingsAreUp = true;
}

void dmaFeedBase::installGlobalIrqExceptionHandler(){
	Xil_ExceptionInit();
#ifdef DMAFEED_HAS_INTC
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XIntc_InterruptHandler, (void *)&iIntc);
#endif
#ifdef DMAFEED_HAS_SCUGIC
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, (void *)&iIntc);
#endif
	Xil_ExceptionEnable();
}

void dmaFeedBase::done(){
	assert(!doneFlag);

	XAxiDma_Bd *firstBdPtr;
	assert(!XAxiDma_BdRingFromHw(txRingPtr, /*no limit to the number of returned BDs*/XAXIDMA_ALL_BDS, &firstBdPtr) && "done() called with uncollected Tx RBs in DMA hardware"); // BdRingFromHw is free of side effects as long as it returns 0
	assert(!XAxiDma_BdRingFromHw(rxRingPtr, /*no limit to the number of returned BDs*/XAXIDMA_ALL_BDS, &firstBdPtr) && "done() called with uncollected Rx RBs in DMA hardware"); // BdRingFromHw is free of side effects as long as it returns 0

	doneFlag = true;
}

void dmaFeedBase::interruptsOnOff(bool newState){
	interruptsDmaOnOff(newState);
	interruptsIrcOnOff(newState);
}

void dmaFeedBase::interruptsDmaOnOff(bool newState){
	if (newState == DMAsideInterruptsAreUp)
		return;
	int s; // generic status
	if (newState){
		// configure DMA end IRQ coalescing
		s = XAxiDma_BdRingSetCoalesce(txRingPtr, config.coalesceNTxInterrupts, config.coalesceDelayTx); assert (s == XST_SUCCESS && "DMA Tx BdRingSetCoalesce() failed");
		s = XAxiDma_BdRingSetCoalesce(rxRingPtr, config.coalesceNRxInterrupts, config.coalesceDelayRx); assert (s == XST_SUCCESS && "DMA Rx BdRingSetCoalesce() failed");

		// enable interrupts on DMA end
		// - config leaves them disabled
		// - reset() in error case disables all interrupts
		XAxiDma_BdRingIntEnable(txRingPtr, XAXIDMA_IRQ_ALL_MASK);
		XAxiDma_BdRingIntEnable(rxRingPtr, XAXIDMA_IRQ_ALL_MASK);
	} else /* if (!newState) */{
		XAxiDma_BdRingIntDisable(txRingPtr, XAXIDMA_IRQ_ALL_MASK);
		XAxiDma_BdRingIntDisable(rxRingPtr, XAXIDMA_IRQ_ALL_MASK);
	}
	DMAsideInterruptsAreUp = newState;
}

#ifdef DMAFEED_HAS_INTC
void dmaFeedBase::interruptsIrcOnOff(bool newState){
	if (newState == IRCSideInterruptsAreUp)
		return;
	if (newState){
		s = XIntc_Initialize(&iIntCtrl, DMAFEED_INTC_DEVICE_ID); assert(s == XST_SUCCESS && "XIntc_Initialize() failed");

		s = XIntc_Connect(&iIntCtrl, config.txIntrId, (XInterruptHandler) txInterruptCallback, /*payload arg*/this); assert(s == XST_SUCCESS && "XIntc_Connect(Tx) failed");
		s = XIntc_Connect(&iIntCtrl, config.rxIntrId, (XInterruptHandler) rxInterruptCallback, /*payload arg*/this); assert(s == XST_SUCCESS && "XIntc_Connect(Rx) failed");

		s = XIntc_Start(&iIntCtrl, XIN_REAL_MODE); assert(s == XST_SUCCESS && "XIntc_Start() failed");

		XIntc_Enable(&iIntCtrl, config.txIntrId);
		XIntc_Enable(&iIntCtrl, config.rxIntrId);
	} else /* if (!newState) */{
		XIntc_Disconnect(IntcInstancePtr, config.txIntrId);
		XIntc_Disconnect(IntcInstancePtr, config.rxIntrId);
	}
	IRCsideInterruptsAreUp = newState;
}
#endif

#ifdef DMAFEED_HAS_SCUGIC
void dmaFeedBase::interruptsIrcOnOff(bool newState){
	if (newState == IRCsideInterruptsAreUp)
		return;
	int s; // generic state
	if (newState){
		XScuGic_Config *intcConfig = XScuGic_LookupConfig(DMAFEED_INTC_DEVICE_ID); assert(intcConfig && "DMA: XScuGic_LookupConfig() failed");
		s = XScuGic_CfgInitialize(&iIntc, intcConfig, intcConfig->CpuBaseAddress); assert(s == XST_SUCCESS && "DMA: XScuGic_CfgInitialize() failed");

		XScuGic_SetPriorityTriggerType(&iIntc, config.txIntrId, /*prio*/0xA0, /*rising edge*/0x3); // prio value from sample code
		XScuGic_SetPriorityTriggerType(&iIntc, config.rxIntrId, /*prio*/0xA0, /*rising edge*/0x3);

		s = XScuGic_Connect(&iIntc, config.txIntrId, (Xil_InterruptHandler)txInterruptCallback, /*payload arg*/this); assert(s == XST_SUCCESS && "DMA: XScuGic_Connect() failed for txInt");
		s = XScuGic_Connect(&iIntc, config.rxIntrId, (Xil_InterruptHandler)rxInterruptCallback, /*payload arg*/this); assert(s == XST_SUCCESS && "DMA: XScuGic_Connect() failed for rxInt");

		XScuGic_Enable(&iIntc, config.txIntrId);
		XScuGic_Enable(&iIntc, config.rxIntrId);
	} else /* if (!newState) */{
		XScuGic_Disconnect(&iIntc, config.txIntrId);
		XScuGic_Disconnect(&iIntc, config.rxIntrId);
	}
	IRCsideInterruptsAreUp = newState;
}
#endif

void dmaFeedBase::runStart(){
	doneFlag = false;
	dmaError = false;

	acquireBDRings();

	// === enable callback ===
	// note: edge sensitive => must enable before starting
	interruptsOnOff(true);

	// === queue first BDs ===
	queue(/*txEvent*/true, /*rxEvent*/true); // both Tx and Rx RBs are available

	// === start transfer ===
	int s; // generic state
	s = XAxiDma_BdRingStart(txRingPtr);	assert(s == XST_SUCCESS && "DMA Tx: BdRingStart() failed");
	s = XAxiDma_BdRingStart(rxRingPtr); assert(s == XST_SUCCESS && "DMA Rx: BdRingStart() failed");
}

dmaFeedBase::run_poll_e dmaFeedBase::run_poll(){
	if (dmaError){
		// Dma_Reset(): "Any DMA transaction in progress will finish gracefully before engine starts reset.
		// Any other transactions that have been submitted to hardware will be discarded by the hardware."

		// reset() disables interrupts on DMA end => need to re-enable on next transaction
		DMAsideInterruptsAreUp = false;

		// "all transactions finished" implies all buffers are free - partly complete - after reset.
		BDRingsAreUp = false; // let's not trust this and rebuild BD rings ...

		// sample code implies reset may fail. Not sure what to do about this...
		for (int retry = 0; retry < 5; ++retry){
			XAxiDma_Reset(&iDma);

			for (unsigned int ix = 0; ix < 10000; ++ix)
				if (XAxiDma_ResetIsDone(&iDma))
					goto doneWithError;
		}
		assert(0 && "failed to reset DMA"); // -DNDEBUG will continue like the sample code
	doneWithError:
		// run_poll() reports an error only once
		dmaError = false;
		return DMAFEED_IDLE_ERROR;
	} // if dmaError

	return doneFlag ? DMAFEED_IDLE : DMAFEED_BUSY;
}

dmaFeedBase::~dmaFeedBase(){
	// disable interrupts
	interruptsOnOff(false);
	free(bufferDescriptorSpace); // from aligned_alloc; free(NULL) is safe
}

void dmaFeedBase::txInterruptCallback(dmaFeedBase* self){
	// === get and acknowledge IRQ status ===
	u32 irqStatus = XAxiDma_BdRingGetIrq(self->txRingPtr);
	XAxiDma_BdRingAckIrq(self->txRingPtr, irqStatus);

	if (!(irqStatus & XAXIDMA_IRQ_ALL_MASK))
		return; // nothing to do (shortcut)

	if (self->doneFlag)
		return; // possible delay interrupt after user code has flagged completion, suppress callbacks

	if ((irqStatus & XAXIDMA_IRQ_ERROR_MASK))
		self->dmaError = true;

	if (self->dmaError)
		return; // no user code callbacks in error state, pending reset

	if (!(irqStatus & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK)))
		return;

	self->collectTx(); // continue with implementation-specific code to free (and optionally, process) Tx BDs completed by the DMA hardware
	self->queue(/*txEvent*/true, /*rxEvent*/false);
}

void dmaFeedBase::rxInterruptCallback(dmaFeedBase* self){
	// === get and acknowledge IRQ status ===
	u32 irqStatus = XAxiDma_BdRingGetIrq(self->rxRingPtr);
	XAxiDma_BdRingAckIrq(self->rxRingPtr, irqStatus);

	if (!(irqStatus & XAXIDMA_IRQ_ALL_MASK))
		return; // nothing to do (shortcut)

	if (self->doneFlag)
		return; // possible delay interrupt after user code has flagged completion, suppress callbacks

	if ((irqStatus & XAXIDMA_IRQ_ERROR_MASK))
		self->dmaError = true;

	if (self->dmaError)
		return; // no callbacks in error state, pending reset

	if (!(irqStatus & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK)))
		return;

	self->collectRx(); // continue with implementation-specific code to free (and optionally, process) Rx BDs completed by the DMA hardware
	self->queue(/*txEvent*/false, /*rxEvent*/true);
}
