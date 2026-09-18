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


#ifndef _DISP_PIO_ST7789_H_
#define _DISP_PIO_ST7789_H_

#include <stdbool.h>
#include <stdint.h>

#define DISPLAY_WIDTH                                                   320		//actual size of the LCD display
#define DISPLAY_HEIGHT                                                  240

//structs
struct ClutEntry {
	uint8_t r, g, b;	
};

struct Rect {
	int16_t x;
	int16_t y;
	uint16_t width;
	uint16_t height;
};

//How the panel is wired. Passed to dispInit(), which keeps a copy - so a stack
//local is fine - and is the driver's only source of pin numbers. Two rules:
//every pin must be 0..31, and SCK must be CS + 1, because SM1 side-sets two
//bits based at CS. dispInit() returns false rather than lighting nothing.
struct dispPinout {
	uint8_t dnc;		//data / command select
	uint8_t cs;		//active low; also the PIO sideset base
	uint8_t sck;		//must be cs + 1
	uint8_t mosi;
	uint8_t miso;		//configured, never read: the panel is write-only here
	uint8_t reset;		//active low
};

struct dmaTransfer;

//`bpp` picks how pixels reach the panel and is fixed for the life of the driver:
//  8   one byte per pixel, expanded through the CLUT by the PIO - dispDrawBuffer()
//  16  RGB565 as-is, no lookup and no CPU in the path - dispDrawBuffer16()
//Either way the panel itself runs in RGB565. False on a bad pinout (see above) or
//a bpp that is neither 8 nor 16.
bool dispInit(const struct dispPinout *pins, uint8_t bpp);

//Only meaningful at 8bpp, but always safe to call: it writes plain memory.
void dispSetClut(int32_t firstIdx, uint32_t numEntries, const struct ClutEntry *entries);

bool dispSetClipArea(const struct Rect *rect);

//8bpp. `stride` is in bytes. NULL if the rectangle clips away entirely.
struct dmaTransfer *dispDrawBuffer(void* framebuffer, uint32_t size, const struct Rect *rect, uint16_t stride);

//16bpp. RGB565 handed to the panel untouched, no CLUT and no CPU in the path.
//`stride` is in PIXELS, not bytes. NULL if the rectangle clips away entirely, or
//if dispInit() was not given 16.
struct dmaTransfer *dispDrawBuffer16(void* framebuffer, uint32_t size, const struct Rect *rect, uint16_t stride);
struct dmaTransfer *dispDrawOneColor(uint16_t color, const struct Rect *rect);
//Has the transfer finished? Tests the same conditions as the wait below, once,
//without spinning. Clears the transfer's busy flag when it has.
bool dispDmaTransferBusy(struct dmaTransfer *dmaTransfer);

void dispDmaTransferWaitFinish(struct dmaTransfer *dmaTransfer);
void dispDebugPrintStatus(void);

#endif