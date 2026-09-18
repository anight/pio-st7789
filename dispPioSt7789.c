//(c) 2023 Dmitry Grinberg  https://dmitry.gr
//Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//	Redistributions of source code must retain the above copyright notice, this list
//		of conditions and the following disclaimer.
//	Redistributions in binary form must reproduce the above copyright notice, this
//		list of conditions and the following disclaimer in the documentation and/or
//		other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//	IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
//	INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
//	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//	WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//	POSSIBILITY OF SUCH DAMAGE.[

#include <string.h>
#include <stdio.h>

#include "pico.h"
#include "pico/time.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/resets.h"
#include "hardware/regs/pio.h"
#include "hardware/regs/dma.h"
#include "hardware/regs/dreq.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/resets.h"
#include "hardware/structs/pio.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/dma.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#include "dispPioSt7789.h"

#define SIDE_SET_HAS_ENABLE_BIT				1
#define SIDE_SET_NUM_BITS					2
#define DEFINE_PIO_INSTRS
#include "pioAsm.h"


static uint16_t __attribute__((aligned(512))) mClut[256];		//MUST be 512 bytes aligned

/* Panel bit depth: 8 (indexed through mClut) or 16 (RGB565 straight from the
 * caller's buffer). Ported from Dmitry Grinberg's original driver, which
 * supported 1/2/4/8/16 and which this fork had reduced to 8 alone.
 *
 * The two differ only in how a pixel reaches SM1. SM1 is the SPI shifter and is
 * the same program in both: autopull at 16, OSR shifting left, one OUT per bit.
 * At 8bpp a byte goes to SM0, which turns it into a CLUT address, and a DMA pair
 * fetches the RGB565 entry into SM1. At 16bpp the framebuffer already holds what
 * SM1 wants, so the data DMA writes it there itself and SM0 and the lookup chain
 * are not used at all.
 *
 * A halfword write to a 32-bit FIFO register is replicated across both halves of
 * the bus, so a 16-bit push lands in OSR as 0xCCCCCCCC-style duplicate; shifting
 * left from bit 31 therefore emits the correct 16 bits and the pull threshold
 * refills after exactly those. That replication is what makes 16bpp need no
 * repacking on the CPU - and dispDrawOneColor() has always relied on it.
 *
 * Fixed at dispInit(). The original driver let an application change depth while
 * running, which cost it a dispSetDepth() that had to tear down and rebuild the
 * DMA wiring safely; nothing here wants that. A client knows at start-up whether
 * it draws indexed or direct colour, and one of the two is dead code for it. */
static uint8_t mDepth = 8;

/* How this panel is wired, as dispInit() was given it. A copy, so the caller
 * may pass a stack local, and the driver's only source of pin numbers. */
static struct dispPinout mPins;

static struct Rect mClipArea = {0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT};


//these manual spi pieces are only used at start-up. We could use the SPI unit, but why bother?
static uint8_t spiBit(uint8_t bit)
{
	if (bit)
		sio_hw->gpio_set = 1 << mPins.mosi;
	else
		sio_hw->gpio_clr = 1 << mPins.mosi;
	asm volatile("dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy");
	sio_hw->gpio_togl = 1 << mPins.sck;
	asm volatile("dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy");
	bit = ((sio_hw->gpio_in >> mPins.miso) & 1);
	sio_hw->gpio_togl = 1 << mPins.sck;
	asm volatile("dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy");
	
	return bit;
}

static uint8_t spiByte(uint8_t val)
{
	uint_fast8_t i;
	
	for (i = 0; i < 8; i++)
		val = val * 2 + spiBit(val >> 7);
	
	return val;
}

static void lcdPrvWriteByte(uint8_t val)
{
	sio_hw->gpio_clr = 1 << mPins.cs;
	spiByte(val);
	sio_hw->gpio_set = 1 << mPins.cs;
}

static void lcdPrvWriteCmd(uint8_t val)
{
	sio_hw->gpio_clr = 1 << mPins.dnc;
	lcdPrvWriteByte(val);
}

static void lcdPrvWriteData(uint8_t val)
{
	sio_hw->gpio_set = 1 << mPins.dnc;
	lcdPrvWriteByte(val);
}


/*
 * SM1: the SPI shifter, and the one piece both depths share.
 *
 * Takes 16-bit words, shifts them out MSB first on MOSI with the clock on
 * sideset. Autopull at 16 bits means one OUT per bit is all the program needs;
 * X counts a chunk of pixels and the wrap re-initialises it, so the machine
 * sustains itself for as long as data keeps arriving.
 *
 * Who supplies that data is the whole difference between 8bpp and 16bpp, and it
 * is a property of the DMA wiring rather than of this program - which is why
 * this function is identical in both and the depth never reaches PIO.
 */
static uint_fast8_t dispPrvPioSm1SpiProgram(uint_fast8_t pc, uint_fast8_t *startPC, uint_fast8_t *endPC)
{
	uint_fast8_t lblPullNgo, lblMoreBits;

	*startPC = pc;
	pio0_hw->instr_mem[pc++] = I_SET(0, 4, SET_DST_X, 0x0f);
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_ISR, MOV_OP_COPY, MOV_SRC_X);
	pio0_hw->instr_mem[pc++] = I_IN(0, 0, IN_SRC_ZEROES, 9);
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_X, MOV_OP_COPY, MOV_SRC_ISR);
	lblPullNgo = pc;
	pio0_hw->instr_mem[pc++] = I_SET(0, 0, SET_DST_Y, 15);
	lblMoreBits = pc;
	pio0_hw->instr_mem[pc++] = I_OUT(0, 4, OUT_DST_PINS, 1);
	pio0_hw->instr_mem[pc++] = I_JMP(0, 6, JMP_Y_POSTDEC, lblMoreBits);	//1 cy delay after here no matter if we jumped
	pio0_hw->instr_mem[pc++] = I_JMP(0, 4, JMP_X_POSTDEC, lblPullNgo);
	*endPC = pc - 1;	//that was the last instr

	return pc;
}

static void dispPrvPioSm1Configure(uint_fast8_t sm1StartPC, uint_fast8_t sm1EndPC)
{
	pio0_hw->sm[1].clkdiv = (1 << PIO_SM0_CLKDIV_INT_LSB);	//full speed
	pio0_hw->sm[1].execctrl = (pio0_hw->sm[1].execctrl &~ (PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS | PIO_SM2_EXECCTRL_SIDE_EN_BITS)) | (sm1EndPC << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) | (sm1StartPC << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) | (SIDE_SET_HAS_ENABLE_BIT ? PIO_SM2_EXECCTRL_SIDE_EN_BITS : 0);
	pio0_hw->sm[1].shiftctrl = (pio0_hw->sm[1].shiftctrl &~ (PIO_SM1_SHIFTCTRL_PULL_THRESH_BITS | PIO_SM1_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS)) | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS | (16 << PIO_SM1_SHIFTCTRL_PULL_THRESH_LSB);
	pio0_hw->sm[1].pinctrl = (SIDE_SET_BITS_USED << PIO_SM1_PINCTRL_SIDESET_COUNT_LSB) | (1 << PIO_SM1_PINCTRL_OUT_COUNT_LSB) | (mPins.miso << PIO_SM1_PINCTRL_IN_BASE_LSB) | (mPins.cs << PIO_SM1_PINCTRL_SIDESET_BASE_LSB) | (mPins.mosi << PIO_SM1_PINCTRL_OUT_BASE_LSB);
}

static void dispPrvPioProgram8bpp(void)
{
	/*  For one-shot mode:
	    cpu: send fb address to ch2 read address reg
	    ch2: send fb byte by byte to sm0
	    sm0: convert each 8bpp into 16bpp address using clut table and send to ch1
	    ch1: read 16bpp address and send it to ch0
	    ch0: read 16bpp data and send it to sm1
	    sm1: spi 16bpp pixel
	*/

	uint_fast8_t pc = 0, lblMore, lblPullNgo, lblMoreBits, sm0StartPC, sm0EndPC, sm1StartPC, sm1EndPC;
	
	//SM0 expand and request palette entry. basically input byte ??, output CLUT_BASE + (?? * 2), where CLUT_BASE is preset in register X
	//expects X to be clut addr >> 9. input shift shifts right, output shifts right. autopush at 32, autopull at 32
	//waits for IRQ for pushback from second SM
	sm0StartPC = pc;
	pio0_hw->instr_mem[pc++] = I_OUT(0, 0, OUT_DST_Y, 8);
	pio0_hw->instr_mem[pc++] = I_OUT(0, 0, OUT_DST_NULL, 24);
	pio0_hw->instr_mem[pc++] = I_IN(0, 0, IN_SRC_ZEROES, 1);
	pio0_hw->instr_mem[pc++] = I_IN(0, 0, IN_SRC_Y, 8);
	pio0_hw->instr_mem[pc++] = I_IN(0, 0, IN_SRC_X, 32 - 9);
	sm0EndPC = pc - 1;	//that was the last instr

	//SM1 program: SPI the data out. input 16 bit words - from the CLUT-lookup
	//DMA pair at 8bpp, straight from the caller's framebuffer at 16bpp.
	pc = dispPrvPioSm1SpiProgram(pc, &sm1StartPC, &sm1EndPC);

	printf("LCD: PIO programs created. %u instrs\n", pc);
	printf("LCD: PIO prog 0 is %u..%u, 1 is %u..%u\n", sm0StartPC, sm0EndPC, sm1StartPC, sm1EndPC);
	
	//configure sm0
	pio0_hw->sm[0].clkdiv = (1 << PIO_SM0_CLKDIV_INT_LSB);	//full speed
	pio0_hw->sm[0].execctrl = (pio0_hw->sm[0].execctrl &~ (PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS | PIO_SM2_EXECCTRL_SIDE_EN_BITS)) | (sm0EndPC << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) | (sm0StartPC << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) | (SIDE_SET_HAS_ENABLE_BIT ? PIO_SM2_EXECCTRL_SIDE_EN_BITS : 0);
	pio0_hw->sm[0].shiftctrl = (pio0_hw->sm[0].shiftctrl &~ (PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS | PIO_SM1_SHIFTCTRL_PUSH_THRESH_BITS)) | PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS;
	
	//give SM0 the clut address
	pio0_hw->txf[0] = ((uintptr_t)mClut) >> 9;
	pio0_hw->sm[0].instr = I_PULL(0, 0, 0, 0);
	pio0_hw->sm[0].instr = I_OUT(0, 0, OUT_DST_X, 32);	
	
	dispPrvPioSm1Configure(sm1StartPC, sm1EndPC);
	
	//start sm0..sm2
	pio0_hw->sm[0].instr = I_JMP(0, 0, JMP_ALWAYS, sm0StartPC);
	pio0_hw->sm[1].instr = I_JMP(0, 0, JMP_ALWAYS, sm1StartPC);
	pio0_hw->ctrl |= (3 << PIO_CTRL_SM_ENABLE_LSB);  // Enable SM0, SM1 only

	//ch1 (first) RXes a word from SM0s output and writes to ch0's source reg. then triggers ch0. ch0 then DMAs a single 16 bit CLUT value to SM1's input, triggers ch1 again
	dma_hw->ch[0].write_addr = (uintptr_t)&pio0_hw->txf[1];
	dma_hw->ch[0].transfer_count = 1;
	dma_hw->ch[0].al1_ctrl = (DREQ_PIO0_TX1 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (1 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS;
	
	dma_hw->ch[1].read_addr = (uintptr_t)&pio0_hw->rxf[0];
	dma_hw->ch[1].write_addr = (uintptr_t)&dma_hw->ch[0].al3_read_addr_trig;
	dma_hw->ch[1].transfer_count = 1;
	dma_hw->ch[1].ctrl_trig = (DREQ_PIO0_RX0 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (1 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS; 
}

/*
 * 16bpp: RGB565 straight from the caller's buffer.
 *
 * Grinberg's note on this mode in the original driver is "things are simple
 * here", and they are: the panel is already in RGB565, so the framebuffer holds
 * exactly the bytes SM1 shifts out. SM0 and the two lookup channels that make
 * 8bpp work have nothing to do, so they are simply not started - the data DMA in
 * dispDrawBuffer16() writes halfwords to SM1's FIFO itself.
 *
 * Leaving SM0 stopped matters rather than being tidiness. Its 8bpp program ends
 * by pushing a CLUT address to its RX FIFO, and ch1 is armed on DREQ_PIO0_RX0;
 * an SM0 left running with stale TX data would hand ch1 an address, ch1 would
 * hand it to ch0, and ch0 would write a CLUT entry into the middle of our pixel
 * stream. The one-line fix is to not enable it.
 */
static void dispPrvPioProgram16bpp(void)
{
	uint_fast8_t pc = 0, sm1StartPC, sm1EndPC;

	pc = dispPrvPioSm1SpiProgram(pc, &sm1StartPC, &sm1EndPC);

	printf("LCD: PIO program created (16bpp). %u instrs\n", pc);
	printf("LCD: PIO prog 1 is %u..%u\n", sm1StartPC, sm1EndPC);

	dispPrvPioSm1Configure(sm1StartPC, sm1EndPC);

	pio0_hw->sm[1].instr = I_JMP(0, 0, JMP_ALWAYS, sm1StartPC);
	pio0_hw->ctrl |= (2 << PIO_CTRL_SM_ENABLE_LSB);  // Enable SM1 only

	/* ch0/ch1 are the CLUT lookup pair and stay disarmed here. dispPrvPioSetup()
	 * resets PIO0 but not the DMA, so clear them explicitly: a depth switch must
	 * not leave a channel armed on a DREQ that is about to mean something else. */
	dma_hw->ch[0].al1_ctrl = 0;
	dma_hw->ch[1].al1_ctrl = 0;
	dma_hw->abort = (1 << 0) | (1 << 1);
	while (dma_hw->abort);
	while (dma_channel_is_busy(0) || dma_channel_is_busy(1));
}

static void dipPrvPinsSetup(bool forPio)		//uses SM0. only safe while SM0 is stopped
{
	const uint8_t mPinsForDir[] = {mPins.sck, mPins.mosi, mPins.cs}; //first in others out
	uint_fast8_t j;
	
	for (j = 0; j < sizeof(mPinsForDir) / sizeof(*mPinsForDir); j++) {
		
		uint32_t pin = mPinsForDir[j];
		
		//seems that only PIO can set directions when pins are in PIO mode, so do that, one at a time
		
		pio0_hw->sm[0].pinctrl = (pio0_hw->sm[0].pinctrl &~ (PIO_SM1_PINCTRL_SET_BASE_BITS | PIO_SM1_PINCTRL_SET_COUNT_BITS)) | (pin << PIO_SM1_PINCTRL_SET_BASE_LSB) | (1 << PIO_SM1_PINCTRL_SET_COUNT_LSB);
		pio0_hw->sm[0].instr = I_SET(0, 0, SET_DST_PINDIRS, 1);
		pio0_hw->sm[0].instr = I_SET(0, 0, SET_DST_PINS, j >= 2);

		/* gpio_set_function, not a raw FUNCSEL poke: on RP2350 that also
		 * clears the pad isolation latch, and SIO's FUNCSEL constant was
		 * renamed (SIO_0 -> SIOB_PROC_0). GPIO_FUNC_PIO0 / GPIO_FUNC_SIO
		 * are the same numbers on both chips. */
		gpio_set_function(pin, forPio ? GPIO_FUNC_PIO0 : GPIO_FUNC_SIO);
	}
}

static void dispPrvPioSetup(void)
{
	uint_fast8_t i;
	
	//reset PIO0
	resets_hw->reset |= RESETS_RESET_PIO0_BITS;		//this is correct... there seems no other way to re-init the PIO properly. "Restart" doesn't do enough
	resets_hw->reset |= RESETS_RESET_PIO0_BITS;
	resets_hw->reset &=~ RESETS_RESET_PIO0_BITS;
	resets_hw->reset &=~ RESETS_RESET_PIO0_BITS;
	resets_hw->reset &=~ RESETS_RESET_PIO0_BITS;
	while (!(resets_hw->reset_done & RESETS_RESET_PIO0_BITS));
	
	//stop SMs
	pio0_hw->ctrl &=~ (7 << PIO_CTRL_SM_ENABLE_LSB);
	
	//reset SMs
	pio0_hw->ctrl = (7 << PIO_CTRL_SM_RESTART_LSB);
	
	dipPrvPinsSetup(true);
	
	if (mDepth == 16)
		dispPrvPioProgram16bpp();
	else
		dispPrvPioProgram8bpp();
}

static void dispPrvLcdInit(void)
{
	//high bit means command
	static const uint16_t mInitSeq[] = {
		0x8011,          // Sleep out
		0x803a, 0x0055,  // Interface Pixel Format
		0x8036, 0x00a0,  // Memory Data Access Control
		0x8020,          // Display Inversion Off
		0x8013,          // Normal Display Mode On
		0x8029,          // Display On
	};
	uint_fast8_t i;
	
	//reset
	sio_hw->gpio_clr = 1 << mPins.reset;
	sleep_ms(50);
	sio_hw->gpio_set = 1 << mPins.reset;
	sleep_ms(50);

	sio_hw->gpio_set = 1 << mPins.dnc;
	sio_hw->gpio_set = 1 << mPins.cs;

	for (i = 0; i < sizeof(mInitSeq) / sizeof(*mInitSeq); i++) {
		if (mInitSeq[i] >> 15)
			lcdPrvWriteCmd(mInitSeq[i]);
		else
			lcdPrvWriteData(mInitSeq[i]);
	}
	
}

static void dispPrvLcdSetDrawArea(uint_fast16_t topLeftCol, uint_fast16_t topLeftRow, uint_fast16_t width, uint_fast16_t height)	//and issue write command
{
	uint_fast16_t endCol = topLeftCol + width - 1, endRow = topLeftRow + height - 1;
	
	lcdPrvWriteCmd(0x2a);
	lcdPrvWriteData(topLeftCol >> 8);
	lcdPrvWriteData(topLeftCol);
	lcdPrvWriteData(endCol >> 8);
	lcdPrvWriteData(endCol);
	
	lcdPrvWriteCmd(0x2b);
	lcdPrvWriteData(topLeftRow >> 8);
	lcdPrvWriteData(topLeftRow);
	lcdPrvWriteData(endRow >> 8);
	lcdPrvWriteData(endRow);
	
	lcdPrvWriteCmd(0x2c);		//"all that follows is data"
}

static void dispPrvTurnOff(void)
{
	uint_fast8_t i, numDmaChannels = 6;
	
	dma_hw->inte0 &=~ (1 << 5);
	for (i = 0; i < numDmaChannels; i++)
		dma_hw->ch[i].al1_ctrl &=~ DMA_CH0_CTRL_TRIG_EN_BITS;
		
	dma_hw->abort = ((1 << numDmaChannels) - 1);		//abort them all
	while (dma_hw->abort);

	for (i = 0; i < numDmaChannels; i++) {
		while (dma_hw->ch[i].al1_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS);
		dma_hw->ch[i].al1_ctrl = 0;
	}

	dipPrvPinsSetup(false);
	sio_hw->gpio_set = (1 << mPins.cs);
	sio_hw->gpio_clr = (1 << mPins.sck) | (1 << mPins.mosi);
}

static void dispPrvTurnOn(void)
{
	dispPrvLcdInit();

	dispPrvPioSetup();
		
	dispDmaTransferWaitFinish(dispDrawOneColor(0x0000, &mClipArea));
}

void dispSetClut(int32_t firstIdx, uint32_t numEntries, const struct ClutEntry *entries)
{
	uint32_t i;
	
	for (i = 0; i < numEntries; i++, entries++) {
		
		uint32_t effectiveIdx = firstIdx + i, r, g, b;
		
		if (effectiveIdx >= sizeof(mClut) / sizeof(*mClut))
			break;
		
		r = entries->r * 31 / 255;
		g = entries->g * 63 / 255;
		b = entries->b * 31 / 255;
		
		mClut[effectiveIdx] = (r << 11) + (g << 5) + b;
	}
}

bool dispSetClipArea(const struct Rect *rect)
{
	if (rect->x < 0 || rect->y < 0 || rect->width == 0 || rect->height == 0 || rect->x + rect->width > DISPLAY_WIDTH || rect->y + rect->height > DISPLAY_HEIGHT) {
		return false;
	}
	mClipArea = *rect;
	return true;
}

void dispDebugPrintStatus(void)
{
	// Capture snapshot of all status values first (before slow printf)
	bool dma_ch0_busy = dma_channel_is_busy(0);
	bool dma_ch1_busy = dma_channel_is_busy(1);
	bool dma_ch2_busy = dma_channel_is_busy(2);
	bool dma_ch3_busy = dma_channel_is_busy(3);

	uint32_t sm_enabled = pio0_hw->ctrl & 0xF;
	uint32_t flevel = pio0_hw->flevel;
	uint32_t fstat = pio0_hw->fstat;
	
	// Now print the captured snapshot
	printf("DMA Status:\n");
	printf("  CH0: %s", dma_ch0_busy ? "BUSY" : "idle");
	printf("  CH1: %s", dma_ch1_busy ? "BUSY" : "idle");
	printf("  CH2: %s", dma_ch2_busy ? "BUSY" : "idle");
	printf("  CH3: %s\n", dma_ch3_busy ? "BUSY" : "idle");
	
	printf("PIO State Machines:\n");
	printf("  SM0: %s", (sm_enabled & (1 << 0)) ? "ENABLED" : "disabled");
	printf("  SM1: %s", (sm_enabled & (1 << 1)) ? "ENABLED" : "disabled");
	printf("  SM2: %s\n", (sm_enabled & (1 << 2)) ? "ENABLED" : "disabled");
	
	printf("FIFO Levels:\n");
	printf("  SM0: TX=%lu RX=%lu", (flevel >> 0) & 0xF, (flevel >> 4) & 0xF);
	printf("  SM1: TX=%lu RX=%lu", (flevel >> 8) & 0xF, (flevel >> 12) & 0xF);
	printf("  SM2: TX=%lu RX=%lu\n", (flevel >> 16) & 0xF, (flevel >> 20) & 0xF);
	
	printf("FSTAT: 0x%08lx\n", fstat);
}

struct dmaTransfer {
	bool is_working;
	uintptr_t ch3_end_read_addr;
	uintptr_t ch2_end_read_addr;
};

static struct dmaTransfer dmaTransfer = {0};

/* Row start addresses for a push whose source stride is wider than the rectangle,
 * walked by ch3 and terminated by a zero. Shared by both depths because only one
 * transfer is ever in flight - every entry point waits for the previous one. */
static uintptr_t bufs[DISPLAY_HEIGHT + 1];

bool clipArea(const struct Rect *rect, struct Rect *clipRect)
{
	// Clip the draw area to mClipArea
	int16_t clipLeft = mClipArea.x;
	int16_t clipTop = mClipArea.y;
	int16_t clipRight = clipLeft + mClipArea.width;
	int16_t clipBottom = clipTop + mClipArea.height;
	
	int16_t drawRight = rect->x + rect->width;
	int16_t drawBottom = rect->y + rect->height;
	
	// Check if rectangles overlap
	if (rect->x >= clipRight || rect->y >= clipBottom || drawRight <= clipLeft || drawBottom <= clipTop)
		return false;  // No overlap, nothing to draw

	// Calculate clipped pixels from left and top
	int16_t clippedLeft = (rect->x < clipLeft) ? (clipLeft - rect->x) : 0;
	int16_t clippedTop = (rect->y < clipTop) ? (clipTop - rect->y) : 0;

	// Calculate intersection rectangle
	int16_t newX = (rect->x < clipLeft) ? clipLeft : rect->x;
	int16_t newY = (rect->y < clipTop) ? clipTop : rect->y;
	int16_t newRight = (drawRight > clipRight) ? clipRight : drawRight;
	int16_t newBottom = (drawBottom > clipBottom) ? clipBottom : drawBottom;

	clipRect->x = newX;
	clipRect->y = newY;
	clipRect->width = newRight - newX;
	clipRect->height = newBottom - newY;

	return true;
}

struct dmaTransfer *dispDrawBuffer(void* framebuffer, uint32_t size, const struct Rect *rect, uint16_t stride)
{
	if (dmaTransfer.is_working) {
		dispDmaTransferWaitFinish(&dmaTransfer);
	}

	struct Rect clipRect;
	if (!clipArea(rect, &clipRect)) {
		return NULL;
	}

	// Adjust framebuffer pointer to skip clipped pixels (assuming 8bpp = 1 byte per pixel)
	uint8_t* adjustedFb = (uint8_t*)framebuffer + ((clipRect.y - rect->y) * stride) + (clipRect.x - rect->x);
	
	// Calculate new size
	uint32_t newSize = clipRect.height * clipRect.width;

        bool sourceStrideMismatch = clipRect.width != stride;

        dipPrvPinsSetup(false);
	dispPrvLcdSetDrawArea(clipRect.x, clipRect.y, clipRect.width, clipRect.height);
	sio_hw->gpio_set = 1 << mPins.dnc;	//data from now on
	dipPrvPinsSetup(true);

	if (sourceStrideMismatch) {
		uint32_t i;
		uintptr_t addr = (uintptr_t)adjustedFb;
		for (i = 0; i < clipRect.height; i++, addr += stride) {
			bufs[i] = addr;
		}
		bufs[i] = 0;
		dma_hw->ch[2].transfer_count = clipRect.width;
		dmaTransfer.ch2_end_read_addr = 0;
		dmaTransfer.ch3_end_read_addr = (uintptr_t)&bufs[clipRect.height+1];
	} else {
		bufs[0] = (uintptr_t)adjustedFb;
		bufs[1] = 0;
		dma_hw->ch[2].transfer_count = newSize;
		dmaTransfer.ch2_end_read_addr = 0;
		dmaTransfer.ch3_end_read_addr = (uintptr_t)&bufs[2];
	}

	dma_hw->ch[2].write_addr = (uintptr_t)&pio0_hw->txf[0];
	dma_hw->ch[2].al1_ctrl = (DREQ_PIO0_TX0 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (3 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_BYTE << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;

	dma_hw->ch[3].read_addr = (uintptr_t)bufs;
	dma_hw->ch[3].transfer_count = 1;
	dma_hw->ch[3].write_addr = (uintptr_t)&dma_hw->ch[2].al3_read_addr_trig;
	dma_hw->ch[3].ctrl_trig = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (3 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;

	dmaTransfer.is_working = true;
	return &dmaTransfer;
}

/*
 * Push a rectangle of an RGB565 buffer.
 *
 * The 16bpp counterpart of dispDrawBuffer(), and the reason the depth exists: a
 * renderer that already works in RGB565 - which is what the panel wants - hands
 * its framebuffer over untouched and no CPU cycle is spent on a pixel. At 8bpp
 * the same picture would have to be quantised to 256 colours first.
 *
 * `stride` is in pixels, not bytes, which is the unit a caller with a uint16_t*
 * already has. The rest is dispDrawBuffer(): clip, set the draw area, then let
 * ch2 move the data and ch3 walk the row addresses when the source is wider than
 * the rectangle.
 *
 * The one difference in the DMA is where the data goes. At 8bpp ch2 feeds SM0,
 * which produces a CLUT address that another pair of channels turns into a
 * colour. Here ch2 writes halfwords straight to SM1's FIFO - the same place
 * dispDrawOneColor() has always written - so the lookup chain is bypassed
 * entirely rather than being fed something it would misread.
 */
struct dmaTransfer *dispDrawBuffer16(void* framebuffer, uint32_t size, const struct Rect *rect, uint16_t stride)
{
	(void)size;

	if (mDepth != 16)
		return NULL;

	if (dmaTransfer.is_working) {
		dispDmaTransferWaitFinish(&dmaTransfer);
	}

	struct Rect clipRect;
	if (!clipArea(rect, &clipRect)) {
		return NULL;
	}

	//stride and the clip offsets are in pixels here; one pixel is one uint16_t
	uint16_t* adjustedFb = (uint16_t*)framebuffer + ((clipRect.y - rect->y) * (uint32_t)stride) + (clipRect.x - rect->x);

	uint32_t newSize = clipRect.height * clipRect.width;

	bool sourceStrideMismatch = clipRect.width != stride;

	dipPrvPinsSetup(false);
	dispPrvLcdSetDrawArea(clipRect.x, clipRect.y, clipRect.width, clipRect.height);
	sio_hw->gpio_set = 1 << mPins.dnc;	//data from now on
	dipPrvPinsSetup(true);

	if (sourceStrideMismatch) {
		uint32_t i;
		uintptr_t addr = (uintptr_t)adjustedFb;
		for (i = 0; i < clipRect.height; i++, addr += (uint32_t)stride * sizeof(uint16_t)) {
			bufs[i] = addr;
		}
		bufs[i] = 0;
		dma_hw->ch[2].transfer_count = clipRect.width;
		dmaTransfer.ch2_end_read_addr = 0;
		dmaTransfer.ch3_end_read_addr = (uintptr_t)&bufs[clipRect.height+1];
	} else {
		bufs[0] = (uintptr_t)adjustedFb;
		bufs[1] = 0;
		dma_hw->ch[2].transfer_count = newSize;
		dmaTransfer.ch2_end_read_addr = 0;
		dmaTransfer.ch3_end_read_addr = (uintptr_t)&bufs[2];
	}

	dma_hw->ch[2].write_addr = (uintptr_t)&pio0_hw->txf[1];
	dma_hw->ch[2].al1_ctrl = (DREQ_PIO0_TX1 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (3 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;

	dma_hw->ch[3].read_addr = (uintptr_t)bufs;
	dma_hw->ch[3].transfer_count = 1;
	dma_hw->ch[3].write_addr = (uintptr_t)&dma_hw->ch[2].al3_read_addr_trig;
	dma_hw->ch[3].ctrl_trig = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (3 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;

	dmaTransfer.is_working = true;
	return &dmaTransfer;
}

void dispDmaTransferWaitFinish(struct dmaTransfer *dmaTransfer)
{
	while (dma_hw->ch[3].read_addr != dmaTransfer->ch3_end_read_addr);
	while (dma_channel_is_busy(3));
	if (dmaTransfer->ch2_end_read_addr != (uintptr_t)-1) {
	        while (dma_hw->ch[2].read_addr != dmaTransfer->ch2_end_read_addr);
	        while (dma_channel_is_busy(2));
	}
	dmaTransfer->is_working = false;
}

struct dmaTransfer *dispDrawOneColor(uint16_t color, const struct Rect *rect)
{
	if (dmaTransfer.is_working) {
		dispDmaTransferWaitFinish(&dmaTransfer);
	}

	struct Rect clipRect;
	if (!clipArea(rect, &clipRect)) {
		return NULL;
	}

	uint32_t numPixels = clipRect.width * clipRect.height;

	// Store the color value
	static volatile uint16_t mColorValue;
	mColorValue = color;

	// Setup drawing area for full screen
	dipPrvPinsSetup(false);
	dispPrvLcdSetDrawArea(clipRect.x, clipRect.y, clipRect.width, clipRect.height);
	sio_hw->gpio_set = 1 << mPins.dnc;	//data from now on
	dipPrvPinsSetup(true);
	
	// Configure DMA channel 3 to send color directly to sm[1]
	dma_hw->ch[3].read_addr = (uintptr_t)&mColorValue;
	dma_hw->ch[3].write_addr = (uintptr_t)&pio0_hw->txf[1];
	dma_hw->ch[3].transfer_count = numPixels;
	dma_hw->ch[3].ctrl_trig = (DREQ_PIO0_TX1 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | 
	                           (3 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | 
	                           (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | 
	                           DMA_CH0_CTRL_TRIG_EN_BITS;

	dmaTransfer.ch3_end_read_addr = (uintptr_t)&mColorValue;
	dmaTransfer.ch2_end_read_addr = (uintptr_t)-1;
	dmaTransfer.is_working = true;

	return &dmaTransfer;
}

bool dispInit(const struct dispPinout *pins, uint8_t bpp)
{
	if (!pins)
		return false;

	if (bpp != 8 && bpp != 16)
		return false;

	const uint8_t all[] = {pins->dnc, pins->cs, pins->sck,
	                       pins->mosi, pins->miso, pins->reset};
	uint_fast8_t i;

	//every pin is addressed as 1 << pin through SIO's low bank, and the PIO
	//base fields are 5 bits wide, so nothing above 31 can work here
	for (i = 0; i < sizeof(all) / sizeof(*all); i++) {
		if (all[i] > 31)
			return false;
	}

	//SM1 side-sets two bits based at CS, which makes SCK structurally CS + 1.
	//Wired otherwise the panel is simply dark, with nothing to read: worth a
	//refusal rather than a debugging session.
	if (pins->sck != pins->cs + 1)
		return false;

	mPins = *pins;
	mDepth = bpp;

	printf("Init: display is %u x %u, %ubpp\n", DISPLAY_WIDTH, DISPLAY_HEIGHT, bpp);
	dispPrvTurnOn();

	return true;
}
