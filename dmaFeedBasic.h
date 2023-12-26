#ifndef DMAFEEDBASIC_H
#define DMAFEEDBASIC_H
#include "dmaFeedBase.h"

// all fields may be optionally configured before passing to dmaFeed constructor
class dmaFeedBasicConfig: public dmaFeedBaseConfig{
public:
	dmaFeedBasicConfig(unsigned int dmaDevId, unsigned int txIntrId, unsigned int rxIntrId) : dmaFeedBaseConfig(dmaDevId, txIntrId, rxIntrId){
	}
	u32 maxPacketSize = 1 << 13; // up to configured width of DMA length register e.g. XPAR_AXI_DMA_0_SG_LENGTH_WIDTH
};

// sends and receives a predetermined amount of data from and to memory
class dmaFeedBasic: public dmaFeedBase{
public:
	dmaFeedBasic(const dmaFeedBasicConfig& config);
	void runStart(char* txBuf, u32 numTxBytes, char* rxBuf, u32 numRxBytes);
private:
	void collectTx() override final;
	void collectRx() override final;
	void queue(bool txFlag, bool rxFlag) override final;
	void queueTx(); // splitting queue() in two for readability
	void queueRx(); // splitting queue() in two for readability

	// remaining number of outbound bytes to queue
	u32 nTxBytesRemainingToQueue = 0;

	// remaining number of inbound bytes to queue
	u32 nRxBytesRemainingToQueue = 0;

	// remaining number of outbound bytes pending completion
	u32 nTxBytesRemainingToComplete = 0;

	// remaining number of inbound bytes pending completion
	u32 nRxBytesRemainingToComplete = 0;

	// running pointer into outbound data
	char* txPtr = NULL;

	// running pointer into inbound data
	char* rxPtr = NULL;

	// txHandler sets this flag when all Tx data has been queued
	volatile bool txDone = false;

	// rxHandler sets this flag when all Rx data has been received
	volatile bool rxDone = false;

	// need retval buffers to transmit given nr. bytes (not exceeding nBufAvailable)
	unsigned int bytesToBufs(unsigned int nBytes, unsigned int nBufAvailable) const;

	const unsigned int maxPacketSize;
};
#endif
