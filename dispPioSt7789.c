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
#include "hardware/irq.h"

#include "dispPioSt7789.h"
#include "pinout.h"

// Define system clock frequency (default for RP2040)
#ifndef TICKS_PER_SECOND
#define TICKS_PER_SECOND 125000000
#endif

// Define debug print function
#define pr printf

#define SIDE_SET_HAS_ENABLE_BIT				1
#define SIDE_SET_NUM_BITS					2
#define DEFINE_PIO_INSTRS
#include "pioAsm.h"


#if MAX_SUPPORTED_BPP >= 8
	static uint16_t __attribute__((aligned(512))) mClut[256];		//MUST be 512 bytes aligned
#endif


#ifndef NO_TOUCH
static volatile uint32_t mTouchCommands[] = {0b01000000000001011, 0b00110000000000010, 0b00000000000000000};
static volatile uint16_t mTouchCoords[4];
#endif
static uint32_t mFramebufBytes;
static bool mDispOn = false;
static bool mContinuousRefresh = true;
static uint8_t mCurDepth;
static void* mFb;


//these manual spi pieces are only used at start-up. We could use the SPI unit, but why bother?
static uint8_t spiBit(uint8_t bit)
{
	if (bit)
		sio_hw->gpio_set = 1 << PIN_SPI_MOSI;
	else
		sio_hw->gpio_clr = 1 << PIN_SPI_MOSI;
	asm volatile("dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy");
	sio_hw->gpio_togl = 1 << PIN_SPI_CLK;
	asm volatile("dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy;dsb sy");
	bit = ((sio_hw->gpio_in >> PIN_SPI_MISO) & 1);
	sio_hw->gpio_togl = 1 << PIN_SPI_CLK;
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
	sio_hw->gpio_clr = 1 << PIN_LCD_CS;
	spiByte(val);
	sio_hw->gpio_set = 1 << PIN_LCD_CS;
}

static void lcdPrvWriteCmd(uint8_t val)
{
	sio_hw->gpio_clr = 1 << PIN_LCD_DnC;
	lcdPrvWriteByte(val);
}

static void lcdPrvWriteData(uint8_t val)
{
	sio_hw->gpio_set = 1 << PIN_LCD_DnC;
	lcdPrvWriteByte(val);
}


/*
1,2,4 BPP:

	idea: use multiple state machines to make up for lack of regs. DMA between them
	problem is that we want backpressure both ways (either stalls if channel is not ready)
		this cannot be done with JUST DMA so we also use IRQs. consumer sends an IRQ to producer  to produce a sample
		DMA is driven by producer. this way producer will halt if consumer not ready (due to irq) but will also halt due to lack of input data
		lack of data will halt consumer. one nit is that we need to PRIME this to avoid stalls. we send one irq manually at start
	
	touch and screen share the same SPI bus. they work at different speeds. our sending SM will occasionally (4-5 times per screen) stop, deselect the
		screen and give time to another SM to go sample the touch panel. we do this signalling using an IRQ as well
	
	screen is configured in RGB444 mode where it literally takes 4 bits of red then 4 of green, then 4 of blue for each pixel. it still needs complete bytes though
		so we cannot send an odd number of bits per nCS period.
	
	SM0: expands data (on demand) we feed screen data to this one. autopush at 12 bits
	more:
		out bpp -> Y
		mov y, y, invert			//we want 0 = white, but lcd uses 0 = black
		set 12 / bpp - 1 -> X		//replicate "bpp" bits of y 3 times for 4bpp, 6 for 2bpp, 12 for 1bpp. this wors becuase for 4 BPP y is the sample and we want that to go into R, G, B. for 2bpp, screen is still in RGB444 mode, so to upcovert from 2bpp to 4bpp we duplicate the sample. this is correct!. same applies to 1bpp
	again:
		in X, bpp
		goto again if X--
		
		wait irq 4
		jump more, if osr_not_empty
		LOOP_TO_START
	
	[!!!] at setup time, we signal irq once ourselves to prime the system
	
	SM1: sends data and times touch sampling, side set controls CS and CLK, with enable. FED data from the above. each sample of 12 bits in a word bit reversded, right aligned, ready to be shifted out to the right in correct order
	setup:
		// X <- num pixels to do minus 1 90xa01 for us
		set x, 0x0a, ncs low
		in x, 4
		in NULL, 8
		mov isr -> x
		jump $+1, if x--  (x needs to be odd so that we send an even number of pixels which makes it an integer number of bytes)
		
	pull_n_go:
		signal irq 4
		pull
		set y, 11
	more:
		out pins, 1, CLK LOW
		jump more, if y--, CLK HI
		jump pull_n_go, if x--, CLK LO
		
		irq send 5, ncs high, wait for ack			//give touch time to sample 
		irq wait t					//wait for touch to be done
		
		jmp setup
		
		it is important to note that there is a footnote that WAIT gets RE_EXECUTED while waiting, which means a non-optional sideset will get re-xecuted too.
		this is why i had to make side-set optional here
	
	SM2 handles touch

		cmdY = 0b10010000_
		cmdX = 0b11010000_
		cmdZ = 0b11000000_
		
		
		WE CAN DO:
		TX		cmdZcmdZ             cmdYcmdY             cmdXcmdX
		BUSY	        X                    X                    X
		RX		         DDDDDDDDDDDD         DDDDDDDDDDDD         DDDDDDDDDDDD
		
		input shift data left
		output shift data left, command left justified, autopull at 21 bits, autopush at 21 bits
		DMA in the command words !!!: cmdx, cmdy, cmdx
		DMA out the tree samples each in a word
		63 clock ticks total
		
		remembee that side set takes place on the first cycle of any instruction, even if it stalls or has a wait
		wait takes place AFTER instruction (even if it already stalls)
		
		side set controls nCS, out controls mosi, in is from miso, SET controls nCS
		
		start:
			irq wait 5
			set nCS low
			set x, 2
		outer:
			set y, 20, wait 15
		doIO:
			out pins, 1, wait 15, clock low
			in pins, 1, wait 15, clock high
			jump doIO, if y--, clock low
			jump outer, if x--
		
			set nCS high
			irq send 5

	but, we can improve to a better version:

		51 clock tick total
		
		---------------------------------------------??????    output data (51 bits total, issued as 3x17 bit words)
		110100000000000100100000000000110000000000000000000
		CMDCMDCM       CMDCMDCM       CMDCMDCM
		        B              B              B
		         DATADATADATA   DATADATADATA   DATADATADATA
		******---------------------------------------------	input samples (4x15, first pre-stuffed with 9 bits and entirely ignored)
		
		SEND DATA:
			01000000000001011
			00110000000000010
			00000000000000000

		we pre-set X to the number of bits we'll do before starting the SM, then:
		
		mov x -> y
		in 9 zeroes
		wait_for_irq 5
		set nCS low
	loop:
		out pins 1
		in pins 1
		jmp loop if y--
		set nCS high
		send irq 5
	
	there is one more complication. the touch chip lowers nPENIRQ while sampling, and also randomly at other times,
	so using it as a CPU irq is no good - we just analize the data in an irq handler - it is fast

8BPP:
	screen is configured for RGB565, the CLUT is full of RGB565 entries MUST be512-byte aligned. Why? Because of how we do things
	
	SM0 is preconfigured to have (clut_address >> 9) in register X. it input screen data one word (4 pixels) at a time
	and outputs ... RAM addresses of CLUT entries that each pixel needs. This is simple:
	
	loop:
		out 8 bits -> Y
		in 1 <- ZERO
		in 8 <- Y
		WAIT (sync with consumer, same as above)
		in 23 <- X
		goto loop
	
	this will produce a 32-bt value per pixel. our consumer DMA reads one sample, triggers the second channel and writes this value as its "read_address" reg
	its "write_address_ is SM1's input. transfer size is a word. This means that SM1 gets 16-bit data as input and can just shift it out. This is why CLUT needs to be 512-byte aligned
	PIO lacks ability to ADD, we just place those values there.
	
	SM1 juts shifts out 16 bits at a time, leftward,s
	
	Touch done same as above.

16BPP:
	
	things are simple here. screen is configured for RGB565, we DMA data here as halfwords and shift
	them to the left (MSB first). bus replication makes it work. touch is done the same way as in all cases
*/


#ifndef NO_TOUCH
static uint_fast8_t dispPrvPioSm2touchProgram(uint_fast8_t pc)
{
	uint_fast8_t lblOuter, lblDoIO;
	
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_Y, MOV_OP_COPY, MOV_SRC_X);
	pio0_hw->instr_mem[pc++] = I_IN(0, 0, IN_SRC_ZEROES, 9);
	pio0_hw->instr_mem[pc++] = I_WAIT(0, 0, 1, WAIT_FOR_IRQ, 5);
	pio0_hw->instr_mem[pc++] = I_SET(0, 0, SET_DST_PINS, 0);			//nCS low
	lblDoIO = pc;
	pio0_hw->instr_mem[pc++] = I_OUT(1, 5, OUT_DST_PINS, 1);
	pio0_hw->instr_mem[pc++] = I_IN(2, 7, IN_SRC_PINS, 1);
	pio0_hw->instr_mem[pc++] = I_JMP(0, 5, JMP_Y_POSTDEC, lblDoIO);
	pio0_hw->instr_mem[pc++] = I_SET(0, 0, SET_DST_PINS, 1);			//nCS high
	pio0_hw->instr_mem[pc++] = I_IRQ(0, 0, 0, 1, 5);					//sgnal irq and wait for ack
	
	return pc;
}

static void dispPrvPioSm2touchSmConfigure(uint_fast8_t sm2StartPC, uint_fast8_t sm2EndPC)
{
	//touch controller is 2.5MHz max as per spec, do not overspeed it
	uint32_t desiredRate = 2500000, divM256 = (TICKS_PER_SECOND * 256ull / 6 + 2500000 - 1) / 2500000;
	
	//configure sm2 (it uses side enable)
	pio0_hw->sm[2].clkdiv = (divM256 << PIO_SM0_CLKDIV_FRAC_LSB);
	pio0_hw->sm[2].execctrl = (pio0_hw->sm[2].execctrl &~ (PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS | PIO_SM2_EXECCTRL_SIDE_EN_BITS)) | (sm2EndPC << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) | (sm2StartPC << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) | (SIDE_SET_HAS_ENABLE_BIT ? PIO_SM2_EXECCTRL_SIDE_EN_BITS : 0);
	pio0_hw->sm[2].shiftctrl = (pio0_hw->sm[2].shiftctrl &~ (PIO_SM1_SHIFTCTRL_PULL_THRESH_BITS | PIO_SM1_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS)) | (17 << PIO_SM1_SHIFTCTRL_PULL_THRESH_LSB) | (15 << PIO_SM1_SHIFTCTRL_PUSH_THRESH_LSB) | PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS;
	pio0_hw->sm[2].pinctrl = (SIDE_SET_BITS_USED << PIO_SM1_PINCTRL_SIDESET_COUNT_LSB) | (1 << PIO_SM1_PINCTRL_OUT_COUNT_LSB) | (PIN_SPI_MISO << PIO_SM1_PINCTRL_IN_BASE_LSB) | (PIN_LCD_CS << PIO_SM1_PINCTRL_SIDESET_BASE_LSB) | (PIN_SPI_MOSI << PIO_SM1_PINCTRL_OUT_BASE_LSB) | (1 << PIO_SM1_PINCTRL_SET_COUNT_LSB) | (PIN_TOUCH_CS << PIO_SM1_PINCTRL_SET_BASE_LSB);
	
	//give it the value for total number of bits minus one
	pio0_hw->txf[2] = 50;
	pio0_hw->sm[2].instr = I_PULL(0, 0, 0, 0);
	pio0_hw->sm[2].instr = I_OUT(0, 0, OUT_DST_X, 21);
}

void DMA0_IRQHandler(void);

static void dispPrvPioSm2touchDmaConfigure(void)
{
	//touch DMAs have a whole order, and it works thanks to PIOs having 4-word FIFOs
	// * we send the touch constants
	// * we read the touch replies
	// * we re configure ch5 to re-do this again
	struct DmaDescr {
		uint32_t readAddr, writeAddr, xferCt, ctrl;
	};
	static volatile uint32_t mDescrsAddr;
	static volatile struct DmaDescr mDescrs[] = {
		{	//send DATA to touch SM
			.ctrl = (DREQ_PIO0_TX2 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (5 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS,
			.readAddr = (uintptr_t)mTouchCommands,
			.writeAddr = (uintptr_t)&pio0_hw->txf[2],
			.xferCt = sizeof(mTouchCommands) / sizeof(*mTouchCommands),
		},
		{	//get DATA from touch SM
			.ctrl = (DREQ_PIO0_RX2 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (5 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS | DMA_CH0_CTRL_TRIG_EN_BITS,
			.readAddr = (uintptr_t)&pio0_hw->rxf[2],
			.writeAddr = (uintptr_t)mTouchCoords,
			.xferCt = sizeof(mTouchCoords) / sizeof(*mTouchCoords),
		},
		{	//re-configure ch5
			.ctrl = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (4 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS,
			.readAddr = (uintptr_t)&mDescrsAddr,
			.writeAddr = (uintptr_t)&dma_hw->ch[5].al3_read_addr_trig,
			.xferCt = 1,
		},
	};
	mDescrsAddr = (uintptr_t)mDescrs;
	
	//increments on read and write. write wrapped every 16 bytes so it will keep reconfiguring dma4
	//so instead we write the config reg that triggers it
	dma_hw->ch[5].read_addr = mDescrsAddr;
	dma_hw->ch[5].write_addr = (uintptr_t)&dma_hw->ch[4].read_addr;
	dma_hw->ch[5].transfer_count = sizeof(struct DmaDescr) / sizeof(uint32_t);
	dma_hw->ch[5].ctrl_trig = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (5 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_RING_SEL_BITS | (4 << DMA_CH0_CTRL_TRIG_RING_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;
	
	dma_hw->ints0 = 1 << 5;
	dma_hw->inte0 |= 1 << 5;
	irq_set_exclusive_handler(DMA_IRQ_0, DMA0_IRQHandler);
	irq_set_enabled(DMA_IRQ_0, true);
}
#endif // NO_TOUCH

#ifdef MODE_421BPP
static void dispPrvPioProgram421bpp(uint_fast8_t bpp)
{
	uint_fast8_t pc = 0, lblAgain, lblMore, lblPullNgo, lblMoreBits, sm0StartPC, sm0EndPC, sm1StartPC, sm1EndPC;
#ifndef NO_TOUCH
	uint_fast8_t sm2StartPC, sm2EndPC;
#endif
	uint8_t chain_target;
	static volatile uint32_t mDmaVal;
	
	//SM0 program: reorder and expand. input 32 bit words of 8 pixels. output 12 bit words of expanded pixel val, one pixel per word in TOP bits, ready to be shifted out to the left
	//OSR shifts right, ISR also right
	sm0StartPC = pc;
	
	lblMore = pc;
	pio0_hw->instr_mem[pc++] = I_OUT(0, 0, OUT_DST_Y, bpp);
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_Y, MOV_OP_INVERT, MOV_SRC_Y);			//we want 0 = white, 15 = black, as G&W LCDs do
	pio0_hw->instr_mem[pc++] = I_SET(0, 4, SET_DST_X, 12 / bpp - 1);
	lblAgain = pc;
	pio0_hw->instr_mem[pc++] = I_IN(0, 0, IN_SRC_Y, bpp);
	pio0_hw->instr_mem[pc++] = I_JMP(0, 0, JMP_X_POSTDEC, lblAgain);
	//pio0_hw->instr_mem[pc++] = I_PUSH(0, 0, 0, 1);
	pio0_hw->instr_mem[pc++] = I_WAIT(0, 0, 1, WAIT_FOR_IRQ, 4);
	pio0_hw->instr_mem[pc++] = I_JMP(0, 0, JMP_OSR_NE, lblMore);
	sm0EndPC = pc - 1;	//that was the last instr

	//SM1 program: SPI the data out. input 12 bit words
	//OSR shifts left, ISR shifts left, sideset used for clock, data output to MOSI using OUT
	
	sm1StartPC = pc;
	pio0_hw->instr_mem[pc++] = I_SET(0, 4, SET_DST_X, 0x0a);
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_ISR, MOV_OP_COPY, MOV_SRC_X);
	pio0_hw->instr_mem[pc++] = I_IN(0, 0, IN_SRC_ZEROES, 10);
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_X, MOV_OP_COPY, MOV_SRC_ISR);
	pio0_hw->instr_mem[pc] = I_JMP(0, 0, JMP_X_POSTDEC, pc + 1);	//we need the number of iterations ot be even, which means X needs to be odd, thus we subtract one from it
	pc++;
	lblPullNgo = pc;
	pio0_hw->instr_mem[pc++] = I_IRQ(0, 0, 0, 1, 4);					//wait here makes sure irq does not get lost
	pio0_hw->instr_mem[pc++] = I_SET(0, 0, SET_DST_Y, 11);
	pio0_hw->instr_mem[pc++] = I_PULL(0, 0, 0, 1);
	lblMoreBits = pc;
	pio0_hw->instr_mem[pc++] = I_OUT(0, 4, OUT_DST_PINS, 1);
	pio0_hw->instr_mem[pc++] = I_JMP(0, 6, JMP_Y_POSTDEC, lblMoreBits);	//1 cy delay after here no matter if we jumped
	pio0_hw->instr_mem[pc++] = I_JMP(0, 4, JMP_X_POSTDEC, lblPullNgo);

#ifndef NO_TOUCH
	//done with this chunk of screen - signal touch to go do its thing	
	pio0_hw->instr_mem[pc++] = I_IRQ(0, 5, 0, 1, 5);					//signal irq and wait for ack, release nCS
	pio0_hw->instr_mem[pc++] = I_WAIT(0, 0, 1, WAIT_FOR_IRQ, 5);		//wait for reply irq, our loop start will lower nCS	
#endif
	
	sm1EndPC = pc - 1;	//that was the last instr

#ifndef NO_TOUCH
	sm2StartPC = pc;
	pc = dispPrvPioSm2touchProgram(pc);
	sm2EndPC = pc - 1;	//that was the last instr
#endif

	pr("LCD: PIO programs created. %u instrs\n", pc);
#ifndef NO_TOUCH
	pr("LCD: PIO prog 0 is %u..%u, 1 is %u..%u, 2 is %u..%u\n", sm0StartPC, sm0EndPC, sm1StartPC, sm1EndPC, sm2StartPC, sm2EndPC);
#else
	pr("LCD: PIO prog 0 is %u..%u, 1 is %u..%u\n", sm0StartPC, sm0EndPC, sm1StartPC, sm1EndPC);
#endif
	
	//configure sm0
	pio0_hw->sm[0].clkdiv = (1 << PIO_SM0_CLKDIV_INT_LSB);	//full speed
	pio0_hw->sm[0].execctrl = (pio0_hw->sm[0].execctrl &~ (PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS | PIO_SM2_EXECCTRL_SIDE_EN_BITS)) | (sm0EndPC << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) | (sm0StartPC << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) | (SIDE_SET_HAS_ENABLE_BIT ? PIO_SM2_EXECCTRL_SIDE_EN_BITS : 0);
	pio0_hw->sm[0].shiftctrl = (pio0_hw->sm[0].shiftctrl &~ (PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS | PIO_SM1_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS)) | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS | PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS | (12 << PIO_SM1_SHIFTCTRL_PUSH_THRESH_LSB);
	
	//configure sm1
	pio0_hw->sm[1].clkdiv = (1 << PIO_SM0_CLKDIV_INT_LSB);	//full speed
	pio0_hw->sm[1].execctrl = (pio0_hw->sm[1].execctrl &~ (PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS | PIO_SM2_EXECCTRL_SIDE_EN_BITS)) | (sm1EndPC << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) | (sm1StartPC << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) | (SIDE_SET_HAS_ENABLE_BIT ? PIO_SM2_EXECCTRL_SIDE_EN_BITS : 0);
	pio0_hw->sm[1].shiftctrl = (pio0_hw->sm[1].shiftctrl &~ (PIO_SM1_SHIFTCTRL_PULL_THRESH_BITS | PIO_SM1_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS));
	pio0_hw->sm[1].pinctrl = (SIDE_SET_BITS_USED << PIO_SM1_PINCTRL_SIDESET_COUNT_LSB) | (1 << PIO_SM1_PINCTRL_OUT_COUNT_LSB) | (PIN_SPI_MISO << PIO_SM1_PINCTRL_IN_BASE_LSB) | (PIN_LCD_CS << PIO_SM1_PINCTRL_SIDESET_BASE_LSB) | (PIN_SPI_MOSI << PIO_SM1_PINCTRL_OUT_BASE_LSB);
	
#ifndef NO_TOUCH
	dispPrvPioSm2touchSmConfigure(sm2StartPC, sm2EndPC);
#endif
	
	//start sm0..sm2 (jump tailored to what state GPIOs need to be in)
	pio0_hw->sm[0].instr = I_JMP(0, 0, JMP_ALWAYS, sm0StartPC);
	pio0_hw->sm[1].instr = I_JMP(0, 0, JMP_ALWAYS, sm1StartPC);
#ifndef NO_TOUCH
	pio0_hw->sm[2].instr = I_JMP(0, 0, JMP_ALWAYS, sm2StartPC);
	pio0_hw->ctrl |= (7 << PIO_CTRL_SM_ENABLE_LSB);  // Enable SM0, SM1, SM2
	pr("LCD: sm0..2 up\n");
#else
	pio0_hw->ctrl |= (3 << PIO_CTRL_SM_ENABLE_LSB);  // Enable SM0, SM1 only
	pr("LCD: sm0..1 up\n");
#endif
	
	//set up dma from sm0 to sm1 uding DMA ch0 to do the work and DMA ch1 to restart it. ch1 is used simply as a chain link, we do not care what it transfers and where from
	//we start ch1, which should trigger ch0
	dma_hw->ch[0].read_addr = (uintptr_t)&pio0_hw->rxf[0];
	dma_hw->ch[0].write_addr = (uintptr_t)&pio0_hw->txf[1];
	dma_hw->ch[0].transfer_count = 0x10000000;	//DO verify that it works with a smaller number here (it should)
	dma_hw->ch[0].al1_ctrl = (DREQ_PIO0_RX0 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (1 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS; //pio0_rx0 trigger
	
	dma_hw->ch[1].read_addr = (uintptr_t)&mDmaVal;
	dma_hw->ch[1].write_addr = (uintptr_t)&mDmaVal;
	dma_hw->ch[1].transfer_count = 1;
	dma_hw->ch[1].ctrl_trig = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (0 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS;
	
	//prime irq 4
	pio0_hw->irq_force = 1 << 4;
	pr("LCD: irq primed\n");
	
	//set up dma to send data to SM0. ch 2 to do it, ch3 to restart it (in continuous mode)
	dma_hw->ch[2].write_addr = (uintptr_t)&pio0_hw->txf[0];
	dma_hw->ch[2].transfer_count = mFramebufBytes / sizeof(uint32_t);
	dma_hw->ch[2].al1_ctrl = (DREQ_PIO0_TX0 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (3 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS; //pio0_tx0 trigger
	
	dma_hw->ch[3].read_addr = (uintptr_t)&mFb;
	dma_hw->ch[3].write_addr = (uintptr_t)&dma_hw->ch[2].al3_read_addr_trig;
	dma_hw->ch[3].transfer_count = 1;
	// Chain to self (3) for continuous mode, or no chain (0xF) for one-shot mode
	chain_target = mContinuousRefresh ? 3 : 0xF;
	dma_hw->ch[3].ctrl_trig = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (chain_target << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS;
}
#endif

#ifdef MODE_8BPP
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

#if MAX_SUPPORTED_BPP >= 8
	uint_fast8_t pc = 0, lblMore, lblPullNgo, lblMoreBits, sm0StartPC, sm0EndPC, sm1StartPC, sm1EndPC;
#ifndef NO_TOUCH
	uint_fast8_t sm2StartPC, sm2EndPC;
#endif
	
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

	//SM1 program: SPI the data out. input 16 bit words (from DMA used for CLUT lookup)
	//OSR shifts left, ISR shifts left, sideset used for clock, data output to MOSI using OUT
	
	sm1StartPC = pc;
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

#ifndef NO_TOUCH
	//done with this chunk of screen - signal touch to go do its thing	
	pio0_hw->instr_mem[pc++] = I_IRQ(0, 5, 0, 1, 5);					//signal irq and wait for ack, release nCS
	pio0_hw->instr_mem[pc++] = I_WAIT(0, 0, 1, WAIT_FOR_IRQ, 5);		//wait for reply irq, our loop start will lower nCS	
#endif
	
	sm1EndPC = pc - 1;	//that was the last instr

#ifndef NO_TOUCH
	sm2StartPC = pc;
	pc = dispPrvPioSm2touchProgram(pc);
	sm2EndPC = pc - 1;	//that was the last instr
#endif

	pr("LCD: PIO programs created. %u instrs\n", pc);
#ifndef NO_TOUCH
	pr("LCD: PIO prog 0 is %u..%u, 1 is %u..%u, 2 is %u..%u\n", sm0StartPC, sm0EndPC, sm1StartPC, sm1EndPC, sm2StartPC, sm2EndPC);
#else
	pr("LCD: PIO prog 0 is %u..%u, 1 is %u..%u\n", sm0StartPC, sm0EndPC, sm1StartPC, sm1EndPC);
#endif
	
	//configure sm0
	pio0_hw->sm[0].clkdiv = (1 << PIO_SM0_CLKDIV_INT_LSB);	//full speed
	pio0_hw->sm[0].execctrl = (pio0_hw->sm[0].execctrl &~ (PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS | PIO_SM2_EXECCTRL_SIDE_EN_BITS)) | (sm0EndPC << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) | (sm0StartPC << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) | (SIDE_SET_HAS_ENABLE_BIT ? PIO_SM2_EXECCTRL_SIDE_EN_BITS : 0);
	pio0_hw->sm[0].shiftctrl = (pio0_hw->sm[0].shiftctrl &~ (PIO_SM0_SHIFTCTRL_PULL_THRESH_BITS | PIO_SM1_SHIFTCTRL_PUSH_THRESH_BITS)) | PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS;
	
	//give SM0 the clut address
	pio0_hw->txf[0] = ((uintptr_t)mClut) >> 9;
	pio0_hw->sm[0].instr = I_PULL(0, 0, 0, 0);
	pio0_hw->sm[0].instr = I_OUT(0, 0, OUT_DST_X, 32);	
	
	//configure sm1
	pio0_hw->sm[1].clkdiv = (1 << PIO_SM0_CLKDIV_INT_LSB);	//full speed
	pio0_hw->sm[1].execctrl = (pio0_hw->sm[1].execctrl &~ (PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS | PIO_SM2_EXECCTRL_SIDE_EN_BITS)) | (sm1EndPC << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) | (sm1StartPC << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) | (SIDE_SET_HAS_ENABLE_BIT ? PIO_SM2_EXECCTRL_SIDE_EN_BITS : 0);
	pio0_hw->sm[1].shiftctrl = (pio0_hw->sm[1].shiftctrl &~ (PIO_SM1_SHIFTCTRL_PULL_THRESH_BITS | PIO_SM1_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS)) | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS | (16 << PIO_SM1_SHIFTCTRL_PULL_THRESH_LSB);
	pio0_hw->sm[1].pinctrl = (SIDE_SET_BITS_USED << PIO_SM1_PINCTRL_SIDESET_COUNT_LSB) | (1 << PIO_SM1_PINCTRL_OUT_COUNT_LSB) | (PIN_SPI_MISO << PIO_SM1_PINCTRL_IN_BASE_LSB) | (PIN_LCD_CS << PIO_SM1_PINCTRL_SIDESET_BASE_LSB) | (PIN_SPI_MOSI << PIO_SM1_PINCTRL_OUT_BASE_LSB);
	
#ifndef NO_TOUCH
	dispPrvPioSm2touchSmConfigure(sm2StartPC, sm2EndPC);
#endif
	
	//start sm0..sm2
	pio0_hw->sm[0].instr = I_JMP(0, 0, JMP_ALWAYS, sm0StartPC);
	pio0_hw->sm[1].instr = I_JMP(0, 0, JMP_ALWAYS, sm1StartPC);
#ifndef NO_TOUCH
	pio0_hw->sm[2].instr = I_JMP(0, 0, JMP_ALWAYS, sm2StartPC);
	pio0_hw->ctrl |= (7 << PIO_CTRL_SM_ENABLE_LSB);  // Enable SM0, SM1, SM2
#else
	pio0_hw->ctrl |= (3 << PIO_CTRL_SM_ENABLE_LSB);  // Enable SM0, SM1 only
#endif

	//ch1 (first) RXes a word from SM0s output and writes to ch0's source reg. then triggers ch0. ch0 then DMAs a single 16 bit CLUT value to SM1's input, triggers ch1 again
	dma_hw->ch[0].write_addr = (uintptr_t)&pio0_hw->txf[1];
	dma_hw->ch[0].transfer_count = 1;
	dma_hw->ch[0].al1_ctrl = (DREQ_PIO0_TX1 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (1 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS;
	
	dma_hw->ch[1].read_addr = (uintptr_t)&pio0_hw->rxf[0];
	dma_hw->ch[1].write_addr = (uintptr_t)&dma_hw->ch[0].al3_read_addr_trig;
	dma_hw->ch[1].transfer_count = 1;
	dma_hw->ch[1].ctrl_trig = (DREQ_PIO0_RX0 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (1 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS; 
	
	//set up dma to send data to SM0. ch 2 to do it, ch3 to restart it (in continuous mode)
	dma_hw->ch[2].write_addr = (uintptr_t)&pio0_hw->txf[0];
	dma_hw->ch[2].al1_ctrl = (DREQ_PIO0_TX0 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (2 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_BYTE << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;

	if (mContinuousRefresh) {
		dma_hw->ch[3].read_addr = (uintptr_t)&mFb;
		dma_hw->ch[3].write_addr = (uintptr_t)&dma_hw->ch[2].al3_read_addr_trig;
		dma_hw->ch[3].transfer_count = 1;
		dma_hw->ch[3].ctrl_trig = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (3 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS;
	}
#endif
}
#endif

#ifdef MODE_16BPP
static void dispPrvPioProgram16bpp(void)
{
#if MAX_SUPPORTED_BPP >= 8
	uint_fast8_t pc = 0, lblMore, lblPullNgo, lblMoreBits, sm0StartPC, sm0EndPC;
#ifndef NO_TOUCH
	uint_fast8_t sm2StartPC, sm2EndPC;
#endif
	uint8_t chain_target;

	//SM0 program: SPI the data out. input 16 bit words
	//OSR shifts left, ISR shifts left, sideset used for clock, data output to MOSI using OUT, autopull
	
	sm0StartPC = pc;
	pio0_hw->instr_mem[pc++] = I_SET(0, 4, SET_DST_X, 0x09);
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_ISR, MOV_OP_COPY, MOV_SRC_X);
	pio0_hw->instr_mem[pc++] = I_IN(0, 0, IN_SRC_ZEROES, 10);
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_X, MOV_OP_COPY, MOV_SRC_ISR);
	lblPullNgo = pc;
	pio0_hw->instr_mem[pc++] = I_PULL(0, 0, 0, 1);
	pio0_hw->instr_mem[pc++] = I_SET(0, 0, SET_DST_Y, 15);
	lblMoreBits = pc;
	pio0_hw->instr_mem[pc++] = I_OUT(0, 4, OUT_DST_PINS, 1);
	pio0_hw->instr_mem[pc++] = I_JMP(0, 6, JMP_Y_POSTDEC, lblMoreBits);	//1 cy delay after here no matter if we jumped
	pio0_hw->instr_mem[pc++] = I_JMP(0, 4, JMP_X_POSTDEC, lblPullNgo);

#ifndef NO_TOUCH
	//done with this chunk of screen - signal touch to go do its thing	
	pio0_hw->instr_mem[pc++] = I_IRQ(0, 5, 0, 1, 5);					//signal irq and wait for ack, release nCS
	pio0_hw->instr_mem[pc++] = I_WAIT(0, 0, 1, WAIT_FOR_IRQ, 5);		//wait for reply irq, our loop start will lower nCS	
#endif
	
	sm0EndPC = pc - 1;	//that was the last instr

#ifndef NO_TOUCH
	sm2StartPC = pc;
	pc = dispPrvPioSm2touchProgram(pc);
	sm2EndPC = pc - 1;	//that was the last instr
#endif

	pr("LCD: PIO programs created. %u instrs\n", pc);
#ifndef NO_TOUCH
	pr("LCD: PIO prog 0 is %u..%u, 2 is %u..%u\n", sm0StartPC, sm0EndPC, sm2StartPC, sm2EndPC);
#else
	pr("LCD: PIO prog 0 is %u..%u\n", sm0StartPC, sm0EndPC);
#endif

	//configure sm0
	pio0_hw->sm[0].clkdiv = (1 << PIO_SM0_CLKDIV_INT_LSB);	//full speed
	pio0_hw->sm[0].execctrl = (pio0_hw->sm[1].execctrl &~ (PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS | PIO_SM2_EXECCTRL_SIDE_EN_BITS)) | (sm0EndPC << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) | (sm0StartPC << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) | (SIDE_SET_HAS_ENABLE_BIT ? PIO_SM2_EXECCTRL_SIDE_EN_BITS : 0);
	pio0_hw->sm[0].shiftctrl = (pio0_hw->sm[1].shiftctrl &~ (PIO_SM1_SHIFTCTRL_PULL_THRESH_BITS | PIO_SM1_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS));
	pio0_hw->sm[0].pinctrl = (SIDE_SET_BITS_USED << PIO_SM1_PINCTRL_SIDESET_COUNT_LSB) | (1 << PIO_SM1_PINCTRL_OUT_COUNT_LSB) | (PIN_SPI_MISO << PIO_SM1_PINCTRL_IN_BASE_LSB) | (PIN_LCD_CS << PIO_SM1_PINCTRL_SIDESET_BASE_LSB) | (PIN_SPI_MOSI << PIO_SM1_PINCTRL_OUT_BASE_LSB);
	
#ifndef NO_TOUCH
	dispPrvPioSm2touchSmConfigure(sm2StartPC, sm2EndPC);
#endif
	
	//start sm0 & sm2
	pio0_hw->sm[0].instr = I_JMP(0, 0, JMP_ALWAYS, sm0StartPC);
#ifndef NO_TOUCH
	pio0_hw->sm[2].instr = I_JMP(0, 0, JMP_ALWAYS, sm2StartPC);
	pio0_hw->ctrl |= (5 << PIO_CTRL_SM_ENABLE_LSB);  // Enable SM0, SM2
	pr("LCD: sm0, sm2 up\n");
#else
	pio0_hw->ctrl |= (1 << PIO_CTRL_SM_ENABLE_LSB);  // Enable SM0 only
	pr("LCD: sm0 up\n");
#endif

	//prime irq 4
	pio0_hw->irq_force = 1 << 4;
	pr("LCD: irq primed\n");
	
	//set up dma to send data to SM0
	dma_hw->ch[0].write_addr = (uintptr_t)&pio0_hw->txf[0];
	dma_hw->ch[0].transfer_count = mFramebufBytes / sizeof(uint16_t);
	dma_hw->ch[0].al1_ctrl = (DREQ_PIO0_TX0 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (1 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;
	
	dma_hw->ch[1].read_addr = (uintptr_t)&mFb;
	dma_hw->ch[1].write_addr = (uintptr_t)&dma_hw->ch[0].al3_read_addr_trig;
	dma_hw->ch[1].transfer_count = 1;
	// Chain to self (1) for continuous mode, or no chain (0xF) for one-shot mode
	chain_target = mContinuousRefresh ? 1 : 0xF;
	dma_hw->ch[1].ctrl_trig = (DREQ_FORCE << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (chain_target << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS;
#endif
}
#endif

static void dipPrvPinsSetup(bool forPio)		//uses SM0. only safe while SM0 is stopped
{
#ifndef NO_TOUCH
	const uint8_t mPinsForDir[] = {PIN_SPI_CLK, PIN_SPI_MOSI, PIN_LCD_CS, PIN_TOUCH_CS}; //first in others out
#else
	const uint8_t mPinsForDir[] = {PIN_SPI_CLK, PIN_SPI_MOSI, PIN_LCD_CS}; //first in others out
#endif
	uint_fast8_t j;
	
	for (j = 0; j < sizeof(mPinsForDir) / sizeof(*mPinsForDir); j++) {
		
		uint32_t pin = mPinsForDir[j];
		
		//seems that only PIO can set directions when pins are in PIO mode, so do that, one at a time
		
		pio0_hw->sm[0].pinctrl = (pio0_hw->sm[0].pinctrl &~ (PIO_SM1_PINCTRL_SET_BASE_BITS | PIO_SM1_PINCTRL_SET_COUNT_BITS)) | (pin << PIO_SM1_PINCTRL_SET_BASE_LSB) | (1 << PIO_SM1_PINCTRL_SET_COUNT_LSB);
		pio0_hw->sm[0].instr = I_SET(0, 0, SET_DST_PINDIRS, 1);
		pio0_hw->sm[0].instr = I_SET(0, 0, SET_DST_PINS, j >= 2);
		
		iobank0_hw->io[pin].ctrl = (iobank0_hw->io[pin].ctrl &~ IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) | ((forPio ? IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PIO0_0 : IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_SIO_0) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB);
	}
}

static void dispPrvPioSetup(uint_fast8_t bpp)
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
	
	if (bpp < 8)
		dispPrvPioProgram421bpp(bpp);
	else if (bpp == 8)
		dispPrvPioProgram8bpp();
	else if (bpp == 16)
		dispPrvPioProgram16bpp();
	
#ifndef NO_TOUCH
	dispPrvPioSm2touchDmaConfigure();
#endif
}

static bool dispPrvLcdInit(uint_fast8_t depth)
{
	//high bit means command
	static const uint16_t mInitSeq[] = {
		0x8011,          // Sleep out
		0x803a, 0x0055,  // Interface Pixel Format
		0x8036, 0x00a0,  // Memory Data Access Control
//		0x802a, 0x0000, 0x0000, DISPLAY_WIDTH >> 8, DISPLAY_WIDTH & 0xff,    // Column Address Set
//		0x802b, 0x0000, 0x0000, DISPLAY_HEIGHT >> 8, DISPLAY_HEIGHT & 0xff,  // Row Address Set
		0x8020,          // Display Inversion Off
		0x8013,          // Normal Display Mode On
		0x8029,          // Display On
	};
	uint_fast8_t i;
	
	//reset
	sio_hw->gpio_clr = 1 << PIN_LCD_RESET;
	pr("display in reset\n");
	sleep_ms(50);
	sio_hw->gpio_set = 1 << PIN_LCD_RESET;
	pr("display out of reset\n");
	sleep_ms(50);

	sio_hw->gpio_clr = 1 << PIN_LCD_CS;
	sio_hw->gpio_clr = 1 << PIN_LCD_DnC;
	spiByte(0x04);
	sio_hw->gpio_set = 1 << PIN_LCD_DnC;
	if ((i = spiByte(0)) != 0x42) {
		pr("LCD: %s ID byte is unexpected: %02xh!\n", "first", i);
		//we could bail out here, but some displays do have different IDs and docs do not list all the valid values
		//sio_hw->gpio_set = 1 << PIN_LCD_CS;
		//return false;
	}
	if ((i = spiByte(0)) != 0xc2) {
		pr("LCD: %s ID byte is unexpected: %02xh!\n", "second", i);
		//we could bail out here, but some displays do have different IDs and docs do not list all the valid values
		//sio_hw->gpio_set = 1 << PIN_LCD_CS;
		//return false;
	}
	if ((i = spiByte(0)) != 0xa9) {
		pr("LCD: %s ID byte is unexpected: %02xh!\n", "third", i);
		//we could bail out here, but some displays do have different IDs and docs do not list all the valid values
		//sio_hw->gpio_set = 1 << PIN_LCD_CS;
		//return false;
	}
	sio_hw->gpio_set = 1 << PIN_LCD_CS;

	for (i = 0; i < sizeof(mInitSeq) / sizeof(*mInitSeq); i++) {
		if (mInitSeq[i] >> 15)
			lcdPrvWriteCmd(mInitSeq[i]);
		else
			lcdPrvWriteData(mInitSeq[i]);
	}
	
	return true;
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

#ifndef NO_TOUCH
static void dispPrvTouchRead(uint32_t *dstP)
{
	uint_fast8_t i;
	
	do {
		
		for (i = 0; i < 3; i++)
			dstP[i] = mTouchCoords[i + 1];
		for (i = 0; i < 3 && dstP[i] == mTouchCoords[i + 1]; i++);
		
	} while (i != 3);
}

static void dispPrvPenHandle(int16_t x, int16_t y)
{
	#define FIRST_TOSS		20	//first this many points are tossed
	#define LAST_TOSS		8	//then this many are averaged (and dropped at end)
	
	static uint8_t mFirstCollected, mLastCollected, mArrPtr;
	static uint16_t buf[LAST_TOSS][2];
	static uint32_t avgX, avgY;
	
	if (x < 0 || y < 0) {	//pen is up
		
		if (mLastCollected == LAST_TOSS)	//we were already reporting data, just report pen up
			dispExtTouchReport(-1, -1);
		else if (mLastCollected) {			//we did not collect enough to report. Report last point and an up
			mArrPtr = (mArrPtr + LAST_TOSS - 1) % LAST_TOSS;
			dispExtTouchReport(buf[mArrPtr][0], buf[mArrPtr][1]);
			dispExtTouchReport(-1, -1);
		}
		//else we did not collect enough for FIRST toss - and had reported nothing
		
		mFirstCollected = 0;
		mLastCollected = 0;
		avgX = 0;
		avgY = 0;
	}
	else if (mFirstCollected < FIRST_TOSS) {	//start toss
		
		mFirstCollected++;
	}
	else {
		if (mLastCollected == LAST_TOSS) {
			
			dispExtTouchReport(avgX / LAST_TOSS, avgY / LAST_TOSS);
			avgX -= buf[mArrPtr][0];
			avgY -= buf[mArrPtr][1];
		}
		else
			mLastCollected++;
		buf[mArrPtr][0] = x;
		buf[mArrPtr][1] = y;
		avgX += x;
		avgY += y;
		mArrPtr = (mArrPtr + 1) % LAST_TOSS;
	}
}

void __attribute__((used)) DMA0_IRQHandler(void)
{
	uint32_t sample[3], zThresh = 0xf40;
	static bool wasDown = false;
	
	dma_hw->ints0 = 1 << 5;
	dispPrvTouchRead(sample);
	
	if (sample[2] <= zThresh) {
		wasDown = true;
		dispPrvPenHandle(sample[0], sample[1]);
	}
	else if (wasDown) {
		wasDown = false;
		dispPrvPenHandle(-1, -1);
	}
}
#endif // NO_TOUCH

static bool dispPrvTurnOff(void)
{
	uint_fast8_t i, numDmaChannels = 6;
	
	if (!mDispOn)
		return true;
	mDispOn = false;
	
	pr("DISP: display off start\n");
	
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
#ifndef NO_TOUCH
	sio_hw->gpio_set = (1 << PIN_LCD_CS) | (1 << PIN_TOUCH_CS);
#else
	sio_hw->gpio_set = (1 << PIN_LCD_CS);
#endif
	sio_hw->gpio_clr = (1 << PIN_SPI_CLK) | (1 << PIN_SPI_MOSI);
	
	pr("DISP: display off end\n");
	return true;
}

static bool dispPrvTurnOn(uint_fast8_t depth)
{
	pr("DISP: depth %u\n", depth);
	
	if (!dispPrvLcdInit(depth)) {
		pr("DISP INIT FAIL\n");
		return false;
	}

        //fill the whole screen with blackness (no matter the current bit depth)
	dispPrvLcdSetDrawArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        for (uint32_t i = 0; i < DISPLAY_WIDTH * DISPLAY_HEIGHT * 2; i++) {
                lcdPrvWriteData(0);
	}

	dispPrvPioSetup(depth);
		
	mDispOn = true;
	return true;
}

void dispSetClut(int32_t firstIdx, uint32_t numEntries, const struct ClutEntry *entries)
{
#if MAX_SUPPORTED_BPP >= 8
	
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
	//it is effective immediately
#endif
}

void dispSetContinuousRefresh(bool enabled)
{
	mContinuousRefresh = enabled;
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

bool dispDrawBuffer(void* framebuffer, uint32_t size, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t stride)
{
	mFb = framebuffer;
	mFramebufBytes = size;
        dipPrvPinsSetup(false);
	dispPrvLcdSetDrawArea(x, y, width, height);
	sio_hw->gpio_set = 1 << PIN_LCD_DnC;	//data from now on
	dipPrvPinsSetup(true);

	dma_hw->ch[2].transfer_count = mFramebufBytes;
        dma_hw->ch[2].al3_read_addr_trig = (uintptr_t)mFb;

	return true;
}

bool dispDrawWaitFinish(void)
{
	while (dma_hw->ch[2].read_addr != (uintptr_t)mFb + mFramebufBytes);
	while (dma_channel_is_busy(2));
	
	return true;
}

bool dispInit(uint8_t bitDepth)
{
	pr("Init: display is %u x %u\n", DISPLAY_WIDTH, DISPLAY_HEIGHT);
	mCurDepth = bitDepth;
	return dispPrvTurnOn(bitDepth);
}
