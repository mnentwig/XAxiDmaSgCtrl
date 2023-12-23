#ifndef XAXIDMASGCTRL_H
#define XAXIDMASGCTRL_H
#include "xaxidma.h"

#include "xparameters.h"
#include "xil_exception.h"

#include <cassert>
#include "xscugic.h"

// === determine type of interrupt controller ===
#ifdef XPAR_INTC_0_DEVICE_ID
#	define DMAFEED_HAS_INTC
#	define DMAFEED_INTC_DEVICE_ID          XPAR_INTC_0_DEVICE_ID
#	include "xscugic.h" // TBD update
#else
#	define DMAFEED_HAS_SCUGIC
#	define DMAFEED_INTC_DEVICE_ID          XPAR_SCUGIC_SINGLE_DEVICE_ID
#	include "xscugic.h"
#endif

// all fields may be optionally configured before passing to dmaFeed constructor
class dmaFeedConfig{
public:
	dmaFeedConfig(unsigned int dmaDevId, unsigned int txIntrId, unsigned int rxIntrId) :
		dmaDevId(dmaDevId), txIntrId(txIntrId), rxIntrId(rxIntrId){}
	u32 dmaDevId;
	unsigned int txIntrId; // Tx side interrupt ID
	unsigned int rxIntrId; // Rx side interrupt ID
	u32 maxPacketSize = 1 << 13; // up to configured width of DMA length register e.g. XPAR_AXI_DMA_0_SG_LENGTH_WIDTH
	// number of Tx interrupts required for callback
	unsigned int coalesceNTxInterrupts = XAXIDMA_NO_CHANGE;
	// number of Tx interrupts required for callback
	unsigned int coalesceNRxInterrupts = XAXIDMA_NO_CHANGE;
	// delay for Tx interrupt when coalesce count is not reached in time in units of C_DLYTMR_RESOLUTION clock cycles (DMA option)
	unsigned int coalesceDelayTx = XAXIDMA_NO_CHANGE;
	// delay for Rx interrupt when coalesce count is not reached in time in units of C_DLYTMR_RESOLUTION clock cycles (DMA option)
	unsigned int coalesceDelayRx = XAXIDMA_NO_CHANGE;
};

// dmaFeed sends and receives a predetermined amount of data on a given DMA channel
class dmaFeed{
public:
	dmaFeed(const dmaFeedConfig& config);
	~dmaFeed();
	void runStart(char* txBuf, u32 numTxBytes, char* rxBuf, u32 numRxBytes);
	typedef enum {
		// freshly initialized or transaction has completed
		DMAFEED_IDLE=0,

		// transaction is ongoing
		DMAFEED_BUSY,

		// same as IDLE but last transaction resulted in (DMA) error. Clears error state => next poll after ERROR returns IDLE.
		DMAFEED_IDLE_ERROR} run_poll_e;
	run_poll_e run_poll();

	// installs the global library-default exception handler for interrupts (once per application if not done elsewhere)
	static void installGlobalIrqExceptionHandler();

protected: // custom derived class would override queueTx(), queueRx(), needs access to those:
	// parameters that can be externally configured
	dmaFeedConfig config;
	// queues idle buffers to DMA, up to the total amount of remaining Tx data. Called at startup and by interrupt.
	void queueTx(unsigned int nNewBytesTransmitted);
	// queues idle buffers to DMA, up to the total amount of remaining Rx data. Called at startup and by interrupt.
	void queueRx(unsigned int nNewBytesReceived);
	// DMA Tx buffer descriptor ring from XAxiDma_GetTxRing();
	XAxiDma_BdRing *txRingPtr = NULL;
	// DMA Rx buffer descriptor ring from XAxiDma_GetRxRing();
	XAxiDma_BdRing *rxRingPtr = NULL;
private:
	// enables / disables interrupt generation and reception
	void interruptsOnOff(bool newState);

	// enables / disables interrupt generation
	void interruptsDmaOnOff(bool newState);
	// whether interrupt generation on DMA end is currently enabled
	bool DMAsideInterruptsAreUp = true; // true here causes possibly redundant disable on DMA end before ring buffer config (see sample code)

	// enables / disables interrupt reception
	void interruptsIrcOnOff(bool newState);
	// whether interrupt reception (callbacks) at IRC end is currently enabled
	bool IRCsideInterruptsAreUp = false;

	// creates buffer descriptor rings and assigns to DMA HW (setup, repeatedly, if sharing DMA channel between multiple instances)
	void acquireBDRings();
	// whether BD rings are valid and configured on DMA
	bool BDRingsAreUp = false;

	XAxiDma iDma; // DMA hardware block "instance"

	// tx interrupt callback
	static void txInterruptCallback(dmaFeed* self);
	// rx interrupt callback
	static void rxInterruptCallback(dmaFeed* self);
	// need retval buffers to transmit given nr. bytes (not exceeding nBufAvailable)
	unsigned int bytesToBufs(unsigned int nBytes, unsigned int nBufAvailable) const;

	// running pointer into outbound data
	char* txPtr = NULL;
	// running pointer into inbound data
	char* rxPtr = NULL;
	// remaining number of outbound bytes to queue
	u32 nTxBytesRemainingToQueue = 0;
	// remaining number of inbound bytes to queue
	u32 nRxBytesRemainingToQueue = 0;
	// remaining number of outbound bytes pending completion
	u32 nTxBytesRemainingToComplete = 0;
	// remaining number of inbound bytes pending completion
	u32 nRxBytesRemainingToComplete = 0;
	// queueTx sets this flag when all Tx data has been queued
	volatile bool txDone = false;
	// queueRx sets this flag when all Rx data has been received
	volatile bool rxDone = false;
	volatile bool dmaError = false;
	// all memory allocated for buffer descriptors
	void* bufferDescriptorSpace = NULL;
	// memory for tx buffer descriptors (subsection in bufferDescriptorSpace for Tx)
	void* txBdBufSpace = NULL;
	// buffer for rx buffer descriptors (subsection in bufferDescriptorSpace for Rx)
	void* rxBdBufSpace = NULL;
	u32 nBytesAllocTxBd = 0;
	u32 nBytesAllocRxBd = 0;

#ifdef DMAFEED_HAS_INTC
	static XIntc iIntc; // interrupt controller "instance"
#endif
#ifdef DMAFEED_HAS_SCUGIC
	static XScuGic iIntc; // interrupt controller "instance"
#endif
};
#endif
