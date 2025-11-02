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


#define MAX_SUPPORTED_BPP						16
#define DISP_WIDTH								320		//the size of the display you want (drawn in the center of real display)
#define DISP_HEIGHT								240

#define LCD_REAL_WIDTH							320		//actual size of the LCD display
#define LCD_REAL_HEIGHT							240


//externally defined
#ifndef NO_TOUCH
void dispExtTouchReport(int16_t x, int16_t y);	//negative on pen up
#endif

//structs
struct ClutEntry {
	uint8_t r, g, b;	
};

//defined here

void dispSetDepth(uint8_t depth);
void dispSetClut(int32_t firstIdx, uint32_t numEntries, const struct ClutEntry *entries);
bool dispInit(void* framebuffer, uint8_t depth);
bool dispOn(void);
bool dispOff(void);

// Refresh control API
// 
// By default, the display runs in continuous refresh mode where the framebuffer is
// automatically copied to the display continuously. For applications that update the
// framebuffer infrequently, you can switch to one-shot mode for power savings.
//
// Example usage (continuous mode - default):
//   dispInit(framebuffer, 16);
//   // Display auto-refreshes continuously
//
// Example usage (one-shot mode - synchronous):
//   dispInit(framebuffer, 16);
//   dispSetContinuousRefresh(false);  // Switch to one-shot mode
//   // Update framebuffer...
//   dispRefreshStart();               // Start copying framebuffer to display
//   dispRefreshWaitFinish();          // Wait for transfer to complete
//
// Example usage (one-shot mode - asynchronous):
//   dispInit(framebuffer, 16);
//   dispSetContinuousRefresh(false);  // Switch to one-shot mode
//   // Update framebuffer...
//   dispRefreshStart();               // Start copying framebuffer to display
//   // Do other work while DMA transfer is in progress...
//   doSomeWork();
//   dispRefreshWaitFinish();          // Wait for transfer to complete before next update
//
void dispSetContinuousRefresh(bool enabled);  // Enable/disable automatic continuous refresh
bool dispRefreshStart(void);                  // Start a one-shot framebuffer copy (non-blocking)
bool dispRefreshWaitFinish(void);             // Wait for one-shot framebuffer copy to complete

// Debug function to print current status of DMA channels and state machines
void dispDebugPrintStatus(void);

#endif