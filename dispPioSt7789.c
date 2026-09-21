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
#include "hardware/pio.h"

#include "dispPioSt7789.h"

#define SIDE_SET_HAS_ENABLE_BIT				1
#define SIDE_SET_NUM_BITS					2
#define DEFINE_PIO_INSTRS
#include "pioAsm.h"


#if DISP_COLOR_DEPTH == 8
static uint16_t __attribute__((aligned(512))) mClut[256];		//MUST be 512 bytes aligned
#endif

/*
 * The two depths differ only in how a pixel reaches the SPI shifter, which is
 * the same program in both: autopull at 16, OSR shifting left, one OUT per bit.
 * At 8bpp a byte goes to the expander machine, which turns it into a CLUT
 * address, and a DMA pair fetches the RGB565 entry into the shifter. At 16bpp
 * the framebuffer already holds what the shifter wants, so the data DMA writes
 * it there itself and neither the expander nor the lookup chain exists.
 *
 * A halfword write to a 32-bit FIFO register is replicated across both halves of
 * the bus, so a 16-bit push lands in OSR as 0xCCCCCCCC-style duplicate; shifting
 * left from bit 31 therefore emits the correct 16 bits and the pull threshold
 * refills after exactly those. That replication is what makes 16bpp need no
 * repacking on the CPU - and dispDrawOneColor() has always relied on it.
 *
 * DISP_COLOR_DEPTH, in the header, picks which of the two is built.
 */

/* How this panel is wired, as dispInit() was given it. A copy, so the caller
 * may pass a stack local, and the driver's only source of pin numbers. */
static struct dispPinout mPins;

/*
 * The PIO block and the two state machines, held rather than hardcoded.
 *
 * Everything here goes through the SDK - pio_claim_unused_sm(), pio_add_program(),
 * pio_sm_init() - so the allocator knows what is taken and another driver can be
 * given what is left. Writing the registers directly means the SDK believes all
 * four machines and all 32 instruction slots are free, and hands a second driver
 * one of these.
 *
 * mSmExpand is claimed at both depths even though only 8bpp runs a program on it.
 * It is also the machine that executes the SET PINDIRS behind pin setup, and that
 * has to be a machine other than the SPI shifter: pio_sm_set_*_with_mask()
 * overwrites PINCTRL for the duration, and doing that to the shifter mid-frame
 * would point its OUT and sideset at the wrong pins for a few cycles.
 */
static PIO  mPio = pio0;
static uint mSmExpand;      /* 8bpp CLUT address generator; scratch at 16bpp */
static uint mSmSpi;         /* the SPI shifter, both depths                  */
static uint mOffExpand;
static uint mOffSpi;

/*
 * The DMA channels, claimed from the SDK for the same reason the state machines
 * are: a driver that takes channels without claiming them is invisible to the
 * allocator, and the next driver to ask for one is handed a channel already in
 * use.
 *
 * Two chains, and the numbers have to be known rather than fixed because each
 * chain's second channel disables its own chaining by pointing CHAIN_TO at
 * itself, which is how the hardware spells "do not chain".
 *
 *   mDmaData -> mDmaWalk    the data chain. mDmaData moves pixels to a state
 *                           machine; mDmaWalk loads the next row address into
 *                           mDmaData and re-triggers it, which is what lets a
 *                           source wider than the rectangle be pushed a row at
 *                           a time. mDmaWalk also pushes a solid colour on its
 *                           own, with no chain at all.
 *
 *   mDmaClutFetch <-> mDmaClutAddr   the 8bpp lookup ring, and only 8bpp has it.
 *                           mDmaClutAddr takes a CLUT address from the expander
 *                           machine and writes it into mDmaClutFetch's trigger;
 *                           mDmaClutFetch sends that entry to the shifter and
 *                           chains back to re-arm mDmaClutAddr.
 */
static uint mDmaData;
static uint mDmaWalk;
#if DISP_COLOR_DEPTH == 8
static uint mDmaClutFetch;
static uint mDmaClutAddr;
#endif

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
#define SPI_PROG_LEN      8
#define EXPAND_PROG_LEN   5

/*
 * Jump targets are program-relative, not absolute PCs.
 *
 * pio_add_program() picks the offset and adds it to every JMP as it copies the
 * program in, so a label here counts from the start of the array. That is the
 * whole reason the programs can go anywhere, and the reason the SDK can be told
 * what they occupy.
 */
#define SPI_LBL_PULL_N_GO   4
#define SPI_LBL_MORE_BITS   5

static const uint16_t mSpiProgInstrs[SPI_PROG_LEN] = {
	I_SET(0, 4, SET_DST_X, 0x0f),
	I_MOV(0, 0, MOV_DST_ISR, MOV_OP_COPY, MOV_SRC_X),
	I_IN(0, 0, IN_SRC_ZEROES, 9),
	I_MOV(0, 0, MOV_DST_X, MOV_OP_COPY, MOV_SRC_ISR),
	I_SET(0, 0, SET_DST_Y, 15),                             /* SPI_LBL_PULL_N_GO */
	I_OUT(0, 4, OUT_DST_PINS, 1),                           /* SPI_LBL_MORE_BITS */
	I_JMP(0, 6, JMP_Y_POSTDEC, SPI_LBL_MORE_BITS),          /* 1 cy delay after here either way */
	I_JMP(0, 4, JMP_X_POSTDEC, SPI_LBL_PULL_N_GO),
};

static const struct pio_program mSpiProg = {
	.instructions = mSpiProgInstrs,
	.length       = SPI_PROG_LEN,
	.origin       = -1,
	.pio_version  = 0,
#if PICO_PIO_VERSION > 0
	.used_gpio_ranges = 0x1,        /* every pin this drives is below 16 */
#endif
};

#if DISP_COLOR_DEPTH == 8
/*
 * SM0: turn a byte into a CLUT address. In 8bpp only.
 *
 * Input byte ??, output CLUT_BASE + ?? * 2, with CLUT_BASE >> 9 preloaded into X.
 * Shifts right both ways, autopush and autopull at 32.
 */
static const uint16_t mExpandProgInstrs[EXPAND_PROG_LEN] = {
	I_OUT(0, 0, OUT_DST_Y, 8),
	I_OUT(0, 0, OUT_DST_NULL, 24),
	I_IN(0, 0, IN_SRC_ZEROES, 1),
	I_IN(0, 0, IN_SRC_Y, 8),
	I_IN(0, 0, IN_SRC_X, 32 - 9),
};

static const struct pio_program mExpandProg = {
	.instructions = mExpandProgInstrs,
	.length       = EXPAND_PROG_LEN,
	.origin       = -1,
	.pio_version  = 0,
#if PICO_PIO_VERSION > 0
	.used_gpio_ranges = 0,          /* drives no pins at all */
#endif
};

#endif /* DISP_COLOR_DEPTH == 8 */

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
 * this is identical in both and the depth never reaches PIO.
 *
 * SIDE_SET_BITS_USED is three: two clock bits and the enable bit that makes
 * side-setting optional per instruction, which is what `optional` means here.
 */
static void dispPrvSpiSmConfigure(void)
{
	pio_sm_config c = pio_get_default_sm_config();

	sm_config_set_wrap(&c, mOffSpi, mOffSpi + SPI_PROG_LEN - 1);
	sm_config_set_sideset(&c, SIDE_SET_BITS_USED, true, false);
	sm_config_set_sideset_pins(&c, mPins.cs);
	sm_config_set_out_pins(&c, mPins.mosi, 1);
	sm_config_set_in_pins(&c, mPins.miso);
	/* Shift left, autopull at 16: a halfword written to the 32-bit FIFO is
	 * replicated across both halves, so shifting from bit 31 emits the right
	 * 16 bits and the threshold refills after exactly those. */
	sm_config_set_out_shift(&c, false, true, 16);
	sm_config_set_in_shift(&c, false, false, 32);
	sm_config_set_clkdiv_int_frac(&c, 1, 0);

	pio_sm_init(mPio, mSmSpi, mOffSpi, &c);
}

#if DISP_COLOR_DEPTH == 8
static void dispPrvExpandSmConfigure(void)
{
	pio_sm_config c = pio_get_default_sm_config();

	sm_config_set_wrap(&c, mOffExpand, mOffExpand + EXPAND_PROG_LEN - 1);
	sm_config_set_out_shift(&c, true, true, 32);
	sm_config_set_in_shift(&c, true, true, 32);
	sm_config_set_clkdiv_int_frac(&c, 1, 0);

	pio_sm_init(mPio, mSmExpand, mOffExpand, &c);

	/* Hand it the CLUT base. pio_sm_init() has just cleared the FIFOs, so this
	 * has to follow it, and the two exec'd instructions move the value into X
	 * without disturbing the PC that init set. */
	pio_sm_put(mPio, mSmExpand, ((uintptr_t)mClut) >> 9);
	pio_sm_exec(mPio, mSmExpand, pio_encode_pull(false, false));
	pio_sm_exec(mPio, mSmExpand, pio_encode_out(pio_x, 32));
}

#endif /* DISP_COLOR_DEPTH == 8 */

#if DISP_COLOR_DEPTH == 8
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

	printf("LCD: PIO programs at %u..%u (expand) and %u..%u (spi), SM%u and SM%u\n",
	       mOffExpand, mOffExpand + EXPAND_PROG_LEN - 1,
	       mOffSpi, mOffSpi + SPI_PROG_LEN - 1, mSmExpand, mSmSpi);

	dispPrvExpandSmConfigure();
	dispPrvSpiSmConfigure();

	/* Both at once, so neither runs against a half-configured partner. */
	pio_set_sm_mask_enabled(mPio, (1u << mSmExpand) | (1u << mSmSpi), true);

	//ch1 (first) RXes a word from SM0s output and writes to ch0's source reg. then triggers ch0. ch0 then DMAs a single 16 bit CLUT value to SM1's input, triggers ch1 again
	dma_hw->ch[mDmaClutFetch].write_addr = (uintptr_t)&mPio->txf[mSmSpi];
	dma_hw->ch[mDmaClutFetch].transfer_count = 1;
	dma_hw->ch[mDmaClutFetch].al1_ctrl = (pio_get_dreq(mPio, mSmSpi, true) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (mDmaClutAddr << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS;
	
	dma_hw->ch[mDmaClutAddr].read_addr = (uintptr_t)&mPio->rxf[mSmExpand];
	dma_hw->ch[mDmaClutAddr].write_addr = (uintptr_t)&dma_hw->ch[mDmaClutFetch].al3_read_addr_trig;
	dma_hw->ch[mDmaClutAddr].transfer_count = 1;
	dma_hw->ch[mDmaClutAddr].ctrl_trig = (pio_get_dreq(mPio, mSmExpand, false) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (mDmaClutAddr << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS; 
}

#endif /* DISP_COLOR_DEPTH == 8 */

#if DISP_COLOR_DEPTH == 16
/*
 * 16bpp: RGB565 straight from the caller's buffer.
 *
 * Grinberg's note on this mode in the original driver is "things are simple
 * here", and they are: the panel is already in RGB565, so the framebuffer holds
 * exactly the bytes SM1 shifts out. SM0 and the two lookup channels that make
 * 8bpp work have nothing to do, so they are simply not started - the data DMA in
 * dispDrawBuffer16() writes halfwords to SM1's FIFO itself.
 *
 * Leaving the expander machine stopped matters rather than being tidiness. Its
 * 8bpp program ends by pushing a CLUT address to its RX FIFO, and ch1 is armed
 * on that machine's RX DREQ; left running with stale TX data it would hand ch1
 * an address, ch1 would hand it to ch0, and ch0 would write a CLUT entry into
 * the middle of the pixel stream. The one-line fix is to not enable it.
 */
static void dispPrvPioProgram16bpp(void)
{
	printf("LCD: PIO program at %u..%u (spi, 16bpp), SM%u\n",
	       mOffSpi, mOffSpi + SPI_PROG_LEN - 1, mSmSpi);

	dispPrvSpiSmConfigure();
	pio_sm_set_enabled(mPio, mSmSpi, true);

	/* No lookup ring at this depth: the two channels it would use are never
	 * claimed, so there is nothing here to disarm and nothing of ours to
	 * clear. */
}

#endif /* DISP_COLOR_DEPTH == 16 */

/*
 * Hand the three SPI pins to PIO, or back to SIO for the start-up bit-bang.
 *
 * All three are outputs. CS idles high and the other two low, which is what the
 * panel expects between transfers.
 *
 * Directions are PIO's own state, held per pin inside the block and not cleared
 * by handing the pad to SIO and back, so they are set once when the pins first
 * become PIO's and not on every switch.
 *
 * The SET PINDIRS behind pio_sm_set_*_with_mask() is executed by the expander
 * machine, never the shifter. Those helpers overwrite PINCTRL and restore it,
 * and doing that to the shifter while a frame is going out would aim its OUT and
 * sideset at the wrong pins for as long as it took.
 */
static void dispPrvPinsSetup(bool forPio)
{
	const uint8_t pins[] = { mPins.sck, mPins.mosi, mPins.cs };
	const uint32_t mask  = (1u << mPins.sck) | (1u << mPins.mosi) | (1u << mPins.cs);
	uint_fast8_t j;

	for (j = 0; j < sizeof(pins) / sizeof(*pins); j++) {
		if (forPio) {
			/* pio_gpio_init, not a raw FUNCSEL poke: on RP2350 it also
			 * clears the pad isolation latch, and it picks the right
			 * function for whichever block mPio is. */
			pio_gpio_init(mPio, pins[j]);
		} else {
			gpio_set_function(pins[j], GPIO_FUNC_SIO);
		}
	}
}

/* Once, while every machine is still stopped. */
static void dispPrvPinDirsSetup(void)
{
	const uint32_t mask = (1u << mPins.sck) | (1u << mPins.mosi) | (1u << mPins.cs);

	pio_sm_set_pindirs_with_mask(mPio, mSmExpand, mask, mask);
	pio_sm_set_pins_with_mask(mPio, mSmExpand, 1u << mPins.cs, mask);
}

/*
 * Take what this driver needs and nothing else.
 *
 * Claiming the machines and adding the programs through the SDK is what lets a
 * second driver share the block: the allocator can only avoid what it knows
 * about. This used to reset the whole PIO block - which wiped every machine and
 * all 32 slots, co-tenant or not - and then write CTRL whole, which cleared
 * SM_ENABLE for machines it did not own. pio_sm_init() restarts the machine,
 * clears its FIFOs and sets its PC, which is what the reset was there to do.
 */
static void dispPrvPioSetup(void)
{
	mSmSpi  = (uint)pio_claim_unused_sm(mPio, true);
	mOffSpi = (uint)pio_add_program(mPio, &mSpiProg);

#if DISP_COLOR_DEPTH == 8
	mSmExpand  = (uint)pio_claim_unused_sm(mPio, true);
	mOffExpand = (uint)pio_add_program(mPio, &mExpandProg);
#endif

	/* Two channels at 16bpp, four at 8bpp: the lookup ring exists only where
	 * there is a CLUT to look things up in. */
	mDmaData = (uint)dma_claim_unused_channel(true);
	mDmaWalk = (uint)dma_claim_unused_channel(true);
#if DISP_COLOR_DEPTH == 8
	mDmaClutFetch = (uint)dma_claim_unused_channel(true);
	mDmaClutAddr  = (uint)dma_claim_unused_channel(true);
#endif

	dispPrvPinsSetup(true);
	dispPrvPinDirsSetup();

#if DISP_COLOR_DEPTH == 8
	dispPrvPioProgram8bpp();
#else
	dispPrvPioProgram16bpp();
#endif
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
	/* This driver's channels and no others. Aborting by a fixed range would
	 * reach whatever else the SDK has handed out - the I2S driver's pair, for
	 * one - and stop a transfer that has nothing to do with the panel. */
	const uint mine[] = {
		mDmaData, mDmaWalk,
#if DISP_COLOR_DEPTH == 8
		mDmaClutFetch, mDmaClutAddr,
#endif
	};
	uint32_t mask = 0;
	uint_fast8_t i;

	for (i = 0; i < count_of(mine); i++)
		mask |= 1u << mine[i];

	for (i = 0; i < count_of(mine); i++)
		dma_hw->ch[mine[i]].al1_ctrl &=~ DMA_CH0_CTRL_TRIG_EN_BITS;

	dma_hw->abort = mask;
	while (dma_hw->abort);

	for (i = 0; i < count_of(mine); i++) {
		while (dma_hw->ch[mine[i]].al1_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS);
		dma_hw->ch[mine[i]].al1_ctrl = 0;
	}

	dispPrvPinsSetup(false);
	sio_hw->gpio_set = (1 << mPins.cs);
	sio_hw->gpio_clr = (1 << mPins.sck) | (1 << mPins.mosi);
}

static void dispPrvTurnOn(void)
{
	dispPrvLcdInit();

	dispPrvPioSetup();
		
	dispDmaTransferWaitFinish(dispDrawOneColor(0x0000, &mClipArea));
}

#if DISP_COLOR_DEPTH == 8
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

#endif /* DISP_COLOR_DEPTH == 8 */

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
	bool data_busy = dma_channel_is_busy(mDmaData);
	bool walk_busy = dma_channel_is_busy(mDmaWalk);
#if DISP_COLOR_DEPTH == 8
	bool clut_fetch_busy = dma_channel_is_busy(mDmaClutFetch);
	bool clut_addr_busy  = dma_channel_is_busy(mDmaClutAddr);
#endif

	uint32_t sm_enabled = mPio->ctrl & 0xF;
	uint32_t flevel = mPio->flevel;
	uint32_t fstat = mPio->fstat;
	
	// Now print the captured snapshot
	printf("DMA Status:\n");
	printf("  data (ch%u): %s", mDmaData, data_busy ? "BUSY" : "idle");
	printf("  walk (ch%u): %s\n", mDmaWalk, walk_busy ? "BUSY" : "idle");
#if DISP_COLOR_DEPTH == 8
	printf("  clut fetch (ch%u): %s", mDmaClutFetch, clut_fetch_busy ? "BUSY" : "idle");
	printf("  clut addr (ch%u): %s\n", mDmaClutAddr, clut_addr_busy ? "BUSY" : "idle");
#endif
	
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
	uintptr_t walk_end_read_addr;
	uintptr_t data_end_read_addr;
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

#if DISP_COLOR_DEPTH == 8
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

        dispPrvPinsSetup(false);
	dispPrvLcdSetDrawArea(clipRect.x, clipRect.y, clipRect.width, clipRect.height);
	sio_hw->gpio_set = 1 << mPins.dnc;	//data from now on
	dispPrvPinsSetup(true);

	if (sourceStrideMismatch) {
		uint32_t i;
		uintptr_t addr = (uintptr_t)adjustedFb;
		for (i = 0; i < clipRect.height; i++, addr += stride) {
			bufs[i] = addr;
		}
		bufs[i] = 0;
		dma_hw->ch[mDmaData].transfer_count = clipRect.width;
		dmaTransfer.data_end_read_addr = 0;
		dmaTransfer.walk_end_read_addr = (uintptr_t)&bufs[clipRect.height+1];
	} else {
		bufs[0] = (uintptr_t)adjustedFb;
		bufs[1] = 0;
		dma_hw->ch[mDmaData].transfer_count = newSize;
		dmaTransfer.data_end_read_addr = 0;
		dmaTransfer.walk_end_read_addr = (uintptr_t)&bufs[2];
	}

	dma_hw->ch[mDmaData].write_addr = (uintptr_t)&mPio->txf[mSmExpand];
	dma_hw->ch[mDmaData].al1_ctrl = (pio_get_dreq(mPio, mSmExpand, true) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (mDmaWalk << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_BYTE << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;

	dma_hw->ch[mDmaWalk].read_addr = (uintptr_t)bufs;
	dma_hw->ch[mDmaWalk].transfer_count = 1;
	dma_hw->ch[mDmaWalk].write_addr = (uintptr_t)&dma_hw->ch[mDmaData].al3_read_addr_trig;
	dma_hw->ch[mDmaWalk].ctrl_trig = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (mDmaWalk << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;

	dmaTransfer.is_working = true;
	return &dmaTransfer;
}

#endif /* DISP_COLOR_DEPTH == 8 */

#if DISP_COLOR_DEPTH == 16

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

	dispPrvPinsSetup(false);
	dispPrvLcdSetDrawArea(clipRect.x, clipRect.y, clipRect.width, clipRect.height);
	sio_hw->gpio_set = 1 << mPins.dnc;	//data from now on
	dispPrvPinsSetup(true);

	if (sourceStrideMismatch) {
		uint32_t i;
		uintptr_t addr = (uintptr_t)adjustedFb;
		for (i = 0; i < clipRect.height; i++, addr += (uint32_t)stride * sizeof(uint16_t)) {
			bufs[i] = addr;
		}
		bufs[i] = 0;
		dma_hw->ch[mDmaData].transfer_count = clipRect.width;
		dmaTransfer.data_end_read_addr = 0;
		dmaTransfer.walk_end_read_addr = (uintptr_t)&bufs[clipRect.height+1];
	} else {
		bufs[0] = (uintptr_t)adjustedFb;
		bufs[1] = 0;
		dma_hw->ch[mDmaData].transfer_count = newSize;
		dmaTransfer.data_end_read_addr = 0;
		dmaTransfer.walk_end_read_addr = (uintptr_t)&bufs[2];
	}

	dma_hw->ch[mDmaData].write_addr = (uintptr_t)&mPio->txf[mSmSpi];
	dma_hw->ch[mDmaData].al1_ctrl = (pio_get_dreq(mPio, mSmSpi, true) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (mDmaWalk << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;

	dma_hw->ch[mDmaWalk].read_addr = (uintptr_t)bufs;
	dma_hw->ch[mDmaWalk].transfer_count = 1;
	dma_hw->ch[mDmaWalk].write_addr = (uintptr_t)&dma_hw->ch[mDmaData].al3_read_addr_trig;
	dma_hw->ch[mDmaWalk].ctrl_trig = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (mDmaWalk << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;

	dmaTransfer.is_working = true;
	return &dmaTransfer;
}

/*
 * Has this transfer finished, without waiting for it?
 *
 * The same four conditions dispDmaTransferWaitFinish() spins on, tested once. A
 * caller cycling two framebuffers uses this to find out whether the panel is
 * still reading one of them, and can spend the time on something else instead of
 * blocking - which is the only thing the blocking version lets it do.
 *
 * Clears is_working once the transfer is done, so a later wait returns at once.
 */
#endif /* DISP_COLOR_DEPTH == 16 */

bool dispDmaTransferBusy(struct dmaTransfer *dmaTransfer)
{
	if (!dmaTransfer->is_working)
		return false;

	if (dma_hw->ch[mDmaWalk].read_addr != dmaTransfer->walk_end_read_addr)
		return true;
	if (dma_channel_is_busy(mDmaWalk))
		return true;

	if (dmaTransfer->data_end_read_addr != (uintptr_t)-1) {
		if (dma_hw->ch[mDmaData].read_addr != dmaTransfer->data_end_read_addr)
			return true;
		if (dma_channel_is_busy(mDmaData))
			return true;
	}

	dmaTransfer->is_working = false;
	return false;
}

void dispDmaTransferWaitFinish(struct dmaTransfer *dmaTransfer)
{
	while (dma_hw->ch[mDmaWalk].read_addr != dmaTransfer->walk_end_read_addr);
	while (dma_channel_is_busy(mDmaWalk));
	if (dmaTransfer->data_end_read_addr != (uintptr_t)-1) {
	        while (dma_hw->ch[mDmaData].read_addr != dmaTransfer->data_end_read_addr);
	        while (dma_channel_is_busy(mDmaData));
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
	dispPrvPinsSetup(false);
	dispPrvLcdSetDrawArea(clipRect.x, clipRect.y, clipRect.width, clipRect.height);
	sio_hw->gpio_set = 1 << mPins.dnc;	//data from now on
	dispPrvPinsSetup(true);
	
	// Configure DMA channel 3 to send color directly to sm[1]
	dma_hw->ch[mDmaWalk].read_addr = (uintptr_t)&mColorValue;
	dma_hw->ch[mDmaWalk].write_addr = (uintptr_t)&mPio->txf[mSmSpi];
	dma_hw->ch[mDmaWalk].transfer_count = numPixels;
	dma_hw->ch[mDmaWalk].ctrl_trig = (pio_get_dreq(mPio, mSmSpi, true) << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | 
	                           (mDmaWalk << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | 
	                           (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | 
	                           DMA_CH0_CTRL_TRIG_EN_BITS;

	dmaTransfer.walk_end_read_addr = (uintptr_t)&mColorValue;
	dmaTransfer.data_end_read_addr = (uintptr_t)-1;
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

	/* The depth is compiled in; the argument only has to agree with it. A
	 * caller built against the other depth would otherwise call functions this
	 * driver does not have, or feed the wrong DMA chain. */
	if (bpp != DISP_COLOR_DEPTH)
		return false;

	mPins = *pins;

	printf("Init: display is %u x %u, %ubpp\n",
	       DISPLAY_WIDTH, DISPLAY_HEIGHT, DISP_COLOR_DEPTH);
	dispPrvTurnOn();

	return true;
}
