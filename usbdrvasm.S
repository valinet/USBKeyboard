/* Name: usbdrvasm.S
 * Project: V-USB, virtual USB port for Atmel's(r) AVR(r) microcontrollers
 * Author: Christian Starkjohann
 * Creation Date: 2007-06-13
 * Tabsize: 4
 * Copyright: (c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * Revision: $Id: usbdrvasm.S 761 2009-08-12 16:30:23Z cs $
 */

/*
General Description:
This module is the assembler part of the USB driver. This file contains
general code (preprocessor acrobatics and CRC computation) and then includes
the file appropriate for the given clock rate.
*/

#define __SFR_OFFSET 0      /* used by avr-libc's register definitions */
#include "usbportability.h"
#include "usbdrv.h"         /* for common defs */

/* register names */
#define x1      r16
#define x2      r17
#define shift   r18
#define cnt     r19
#define x3      r20
#define x4      r21
#define x5		r22
#define bitcnt  x5
#define phase   x4
#define leap    x4

/* Some assembler dependent definitions and declarations: */

#ifdef __IAR_SYSTEMS_ASM__
    extern  usbRxBuf, usbDeviceAddr, usbNewDeviceAddr, usbInputBufOffset
    extern  usbCurrentTok, usbRxLen, usbRxToken, usbTxLen
    extern  usbTxBuf, usbTxStatus1, usbTxStatus3
#   if USB_COUNT_SOF
        extern usbSofCount
#   endif
    public  usbCrc16
    public  usbCrc16Append

    COMMON  INTVEC
#   ifndef USB_INTR_VECTOR
        ORG     INT0_vect
#   else /* USB_INTR_VECTOR */
        ORG     USB_INTR_VECTOR
#       undef   USB_INTR_VECTOR
#   endif /* USB_INTR_VECTOR */
#   define  USB_INTR_VECTOR usbInterruptHandler
    rjmp    USB_INTR_VECTOR
    RSEG    CODE

#else /* __IAR_SYSTEMS_ASM__ */

#   ifndef USB_INTR_VECTOR /* default to hardware interrupt INT0 */
#       define USB_INTR_VECTOR  SIG_INTERRUPT0
#   endif
    .text
    .global USB_INTR_VECTOR
    .type   USB_INTR_VECTOR, @function
    .global usbCrc16
    .global usbCrc16Append
#endif /* __IAR_SYSTEMS_ASM__ */


#if USB_INTR_PENDING < 0x40 /* This is an I/O address, use in and out */
#   define  USB_LOAD_PENDING(reg)   in reg, USB_INTR_PENDING
#   define  USB_STORE_PENDING(reg)  out USB_INTR_PENDING, reg
#else   /* It's a memory address, use lds and sts */
#   define  USB_LOAD_PENDING(reg)   lds reg, USB_INTR_PENDING
#   define  USB_STORE_PENDING(reg)  sts USB_INTR_PENDING, reg
#endif

#define usbTxLen1   usbTxStatus1
#define usbTxBuf1   (usbTxStatus1 + 1)
#define usbTxLen3   usbTxStatus3
#define usbTxBuf3   (usbTxStatus3 + 1)


;----------------------------------------------------------------------------
; Utility functions
;----------------------------------------------------------------------------

#ifdef __IAR_SYSTEMS_ASM__
/* Register assignments for usbCrc16 on IAR cc */
/* Calling conventions on IAR:
 * First parameter passed in r16/r17, second in r18/r19 and so on.
 * Callee must preserve r4-r15, r24-r29 (r28/r29 is frame pointer)
 * Result is passed in r16/r17
 * In case of the "tiny" memory model, pointers are only 8 bit with no
 * padding. We therefore pass argument 1 as "16 bit unsigned".
 */
RTMODEL "__rt_version", "3"
/* The line above will generate an error if cc calling conventions change.
 * The value "3" above is valid for IAR 4.10B/W32
 */
#   define argLen   r18 /* argument 2 */
#   define argPtrL  r16 /* argument 1 */
#   define argPtrH  r17 /* argument 1 */

#   define resCrcL  r16 /* result */
#   define resCrcH  r17 /* result */

#   define ptrL     ZL
#   define ptrH     ZH
#   define ptr      Z
#   define byte     r22
#   define bitCnt   r19
#   define polyL    r20
#   define polyH    r21
#   define scratch  r23

#else  /* __IAR_SYSTEMS_ASM__ */ 
/* Register assignments for usbCrc16 on gcc */
/* Calling conventions on gcc:
 * First parameter passed in r24/r25, second in r22/23 and so on.
 * Callee must preserve r1-r17, r28/r29
 * Result is passed in r24/r25
 */
#   define argLen   r22 /* argument 2 */
#   define argPtrL  r24 /* argument 1 */
#   define argPtrH  r25 /* argument 1 */

#   define resCrcL  r24 /* result */
#   define resCrcH  r25 /* result */

#   define ptrL     XL
#   define ptrH     XH
#   define ptr      x
#   define byte     r18
#   define bitCnt   r19
#   define polyL    r20
#   define polyH    r21
#   define scratch  r23

#endif

#if USB_USE_FAST_CRC

; This implementation is faster, but has bigger code size
; Thanks to Slawomir Fras (BoskiDialer) for this code!
; It implements the following C pseudo-code:
; unsigned table(unsigned char x)
; {
; unsigned    value;
; 
;     value = (unsigned)x << 6;
;     value ^= (unsigned)x << 7;
;     if(parity(x))
;         value ^= 0xc001;
;     return value;
; }
; unsigned usbCrc16(unsigned char *argPtr, unsigned char argLen)
; {
; unsigned crc = 0xffff;
; 
;     while(argLen--)
;         crc = table(lo8(crc) ^ *argPtr++) ^ hi8(crc);
;     return ~crc;
; }

; extern unsigned usbCrc16(unsigned char *argPtr, unsigned char argLen);
;   argPtr  r24+25 / r16+r17
;   argLen  r22 / r18
; temp variables:
;   byte    r18 / r22
;   scratch r23
;   resCrc  r24+r25 / r16+r17
;   ptr     X / Z
usbCrc16:
    mov     ptrL, argPtrL
    mov     ptrH, argPtrH
    ldi     resCrcL, 0xFF
    ldi     resCrcH, 0xFF
    rjmp    usbCrc16LoopTest
usbCrc16ByteLoop:
    ld      byte, ptr+
    eor     resCrcL, byte   ; resCrcL is now 'x' in table()
    mov     byte, resCrcL   ; compute parity of 'x'
    swap    byte
    eor     byte, resCrcL
    mov     scratch, byte
    lsr     byte
    lsr     byte
    eor     byte, scratch
    inc     byte
    lsr     byte
    andi    byte, 1         ; byte is now parity(x)
    mov     scratch, resCrcL
    mov     resCrcL, resCrcH
    eor     resCrcL, byte   ; low byte of if(parity(x)) value ^= 0xc001;
    neg     byte
    andi    byte, 0xc0
    mov     resCrcH, byte   ; high byte of if(parity(x)) value ^= 0xc001;
    clr     byte
    lsr     scratch
    ror     byte
    eor     resCrcH, scratch
    eor     resCrcL, byte
    lsr     scratch
    ror     byte
    eor     resCrcH, scratch
    eor     resCrcL, byte
usbCrc16LoopTest:
    subi    argLen, 1
    brsh    usbCrc16ByteLoop
    com     resCrcL
    com     resCrcH
    ret

#else   /* USB_USE_FAST_CRC */

; This implementation is slower, but has less code size
;
; extern unsigned usbCrc16(unsigned char *argPtr, unsigned char argLen);
;   argPtr  r24+25 / r16+r17
;   argLen  r22 / r18
; temp variables:
;   byte    r18 / r22
;   bitCnt  r19
;   poly    r20+r21
;   scratch r23
;   resCrc  r24+r25 / r16+r17
;   ptr     X / Z
usbCrc16:
    mov     ptrL, argPtrL
    mov     ptrH, argPtrH
    ldi     resCrcL, 0
    ldi     resCrcH, 0
    ldi     polyL, lo8(0xa001)
    ldi     polyH, hi8(0xa001)
    com     argLen      ; argLen = -argLen - 1: modified loop to ensure that carry is set
    ldi     bitCnt, 0   ; loop counter with starnd condition = end condition
    rjmp    usbCrcLoopEntry
usbCrcByteLoop:
    ld      byte, ptr+
    eor     resCrcL, byte
usbCrcBitLoop:
    ror     resCrcH     ; carry is always set here (see brcs jumps to here)
    ror     resCrcL
    brcs    usbCrcNoXor
    eor     resCrcL, polyL
    eor     resCrcH, polyH
usbCrcNoXor:
    subi    bitCnt, 224 ; (8 * 224) % 256 = 0; this loop iterates 8 times
    brcs    usbCrcBitLoop
usbCrcLoopEntry:
    subi    argLen, -1
    brcs    usbCrcByteLoop
usbCrcReady:
    ret
; Thanks to Reimar Doeffinger for optimizing this CRC routine!

#endif /* USB_USE_FAST_CRC */

; extern unsigned usbCrc16Append(unsigned char *data, unsigned char len);
usbCrc16Append:
    rcall   usbCrc16
    st      ptr+, resCrcL
    st      ptr+, resCrcH
    ret

#undef argLen
#undef argPtrL
#undef argPtrH
#undef resCrcL
#undef resCrcH
#undef ptrL
#undef ptrH
#undef ptr
#undef byte
#undef bitCnt
#undef polyL
#undef polyH
#undef scratch


#if USB_CFG_HAVE_MEASURE_FRAME_LENGTH
#ifdef __IAR_SYSTEMS_ASM__
/* Register assignments for usbMeasureFrameLength on IAR cc */
/* Calling conventions on IAR:
 * First parameter passed in r16/r17, second in r18/r19 and so on.
 * Callee must preserve r4-r15, r24-r29 (r28/r29 is frame pointer)
 * Result is passed in r16/r17
 * In case of the "tiny" memory model, pointers are only 8 bit with no
 * padding. We therefore pass argument 1 as "16 bit unsigned".
 */
#   define resL     r16
#   define resH     r17
#   define cnt16L   r30
#   define cnt16H   r31
#   define cntH     r18

#else  /* __IAR_SYSTEMS_ASM__ */ 
/* Register assignments for usbMeasureFrameLength on gcc */
/* Calling conventions on gcc:
 * First parameter passed in r24/r25, second in r22/23 and so on.
 * Callee must preserve r1-r17, r28/r29
 * Result is passed in r24/r25
 */
#   define resL     r24
#   define resH     r25
#   define cnt16L   r24
#   define cnt16H   r25
#   define cntH     r26
#endif
#   define cnt16    cnt16L

; extern unsigned usbMeasurePacketLength(void);
; returns time between two idle strobes in multiples of 7 CPU clocks
.global usbMeasureFrameLength
usbMeasureFrameLength:
    ldi     cntH, 6         ; wait ~ 10 ms for D- == 0
    clr     cnt16L
    clr     cnt16H
usbMFTime16:
    dec     cntH
    breq    usbMFTimeout
usbMFWaitStrobe:            ; first wait for D- == 0 (idle strobe)
    sbiw    cnt16, 1        ;[0] [6]
    breq    usbMFTime16     ;[2]
    sbic    USBIN, USBMINUS ;[3]
    rjmp    usbMFWaitStrobe ;[4]
usbMFWaitIdle:              ; then wait until idle again
    sbis    USBIN, USBMINUS ;1 wait for D- == 1
    rjmp    usbMFWaitIdle   ;2
    ldi     cnt16L, 1       ;1 represents cycles so far
    clr     cnt16H          ;1
usbMFWaitLoop:
    in      cntH, USBIN     ;[0] [7]
    adiw    cnt16, 1        ;[1]
    breq    usbMFTimeout    ;[3]
    andi    cntH, USBMASK   ;[4]
    brne    usbMFWaitLoop   ;[5]
usbMFTimeout:
#if resL != cnt16L
    mov     resL, cnt16L
    mov     resH, cnt16H
#endif
    ret

#undef resL
#undef resH
#undef cnt16
#undef cnt16L
#undef cnt16H
#undef cntH

#endif  /* USB_CFG_HAVE_MEASURE_FRAME_LENGTH */

;----------------------------------------------------------------------------
; Now include the clock rate specific code
;----------------------------------------------------------------------------

#ifndef USB_CFG_CLOCK_KHZ
#   define USB_CFG_CLOCK_KHZ 12000
#endif

#if USB_CFG_CHECK_CRC   /* separate dispatcher for CRC type modules */
#   if USB_CFG_CLOCK_KHZ == 18000
#       include "usbdrvasm18-crc.inc"
#   else
#       error "USB_CFG_CLOCK_KHZ is not one of the supported crc-rates!"
#   endif
#else   /* USB_CFG_CHECK_CRC */
#   if USB_CFG_CLOCK_KHZ == 12000
#       include "usbdrvasm12.inc"
#   elif USB_CFG_CLOCK_KHZ == 12800
#       include "usbdrvasm128.inc"
#   elif USB_CFG_CLOCK_KHZ == 15000
#       include "usbdrvasm15.inc"
#   elif USB_CFG_CLOCK_KHZ == 16000
//#       include "usbdrvasm16.S"

/* Name: usbdrvasm16.inc
 * Project: V-USB, virtual USB port for Atmel's(r) AVR(r) microcontrollers
 * Author: Christian Starkjohann
 * Creation Date: 2007-06-15
 * Tabsize: 4
 * Copyright: (c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * Revision: $Id: usbdrvasm16.inc 760 2009-08-09 18:59:43Z cs $
 */

/* Do not link this file! Link usbdrvasm.S instead, which includes the
 * appropriate implementation!
 */

/*
General Description:
This file is the 16 MHz version of the asssembler part of the USB driver. It
requires a 16 MHz crystal (not a ceramic resonator and not a calibrated RC
oscillator).

See usbdrv.h for a description of the entire driver.

Since almost all of this code is timing critical, don't change unless you
really know what you are doing! Many parts require not only a maximum number
of CPU cycles, but even an exact number of cycles!
*/

;max stack usage: [ret(2), YL, SREG, YH, bitcnt, shift, x1, x2, x3, x4, cnt] = 12 bytes
;nominal frequency: 16 MHz -> 10.6666666 cycles per bit, 85.333333333 cycles per byte
; Numbers in brackets are clocks counted from center of last sync bit
; when instruction starts

USB_INTR_VECTOR:
;order of registers pushed: YL, SREG YH, [sofError], bitcnt, shift, x1, x2, x3, x4, cnt
    push    YL                  ;[-25] push only what is necessary to sync with edge ASAP
    in      YL, SREG            ;[-23]
    push    YL                  ;[-22]
    push    YH                  ;[-20]
;----------------------------------------------------------------------------
; Synchronize with sync pattern:
;----------------------------------------------------------------------------
;sync byte (D-) pattern LSb to MSb: 01010100 [1 = idle = J, 0 = K]
;sync up with J to K edge during sync pattern -- use fastest possible loops
;The first part waits at most 1 bit long since we must be in sync pattern.
;YL is guarenteed to be < 0x80 because I flag is clear. When we jump to
;waitForJ, ensure that this prerequisite is met.
waitForJ:
    inc     YL
    sbis    USBIN, USBMINUS
    brne    waitForJ        ; just make sure we have ANY timeout
waitForK:
;The following code results in a sampling window of < 1/4 bit which meets the spec.
    sbis    USBIN, USBMINUS     ;[-15]
    rjmp    foundK              ;[-14]
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
    sbis    USBIN, USBMINUS
    rjmp    foundK
#if USB_COUNT_SOF
    lds     YL, usbSofCount
    inc     YL
    sts     usbSofCount, YL
#endif  /* USB_COUNT_SOF */
#ifdef USB_SOF_HOOK
    USB_SOF_HOOK
#endif
    rjmp    sofError
foundK:                         ;[-12]
;{3, 5} after falling D- edge, average delay: 4 cycles [we want 5 for center sampling]
;we have 1 bit time for setup purposes, then sample again. Numbers in brackets
;are cycles from center of first sync (double K) bit after the instruction
    push    bitcnt              ;[-12]
;   [---]                       ;[-11]
    lds     YL, usbInputBufOffset;[-10]
;   [---]                       ;[-9]
    clr     YH                  ;[-8]
    subi    YL, lo8(-(usbRxBuf));[-7] [rx loop init]
    sbci    YH, hi8(-(usbRxBuf));[-6] [rx loop init]
    push    shift               ;[-5]
;   [---]                       ;[-4]
    ldi     bitcnt, 0x55        ;[-3] [rx loop init]
    sbis    USBIN, USBMINUS     ;[-2] we want two bits K (sample 2 cycles too early)
    rjmp    haveTwoBitsK        ;[-1]
    pop     shift               ;[0] undo the push from before
    pop     bitcnt              ;[2] undo the push from before
    rjmp    waitForK            ;[4] this was not the end of sync, retry
; The entire loop from waitForK until rjmp waitForK above must not exceed two
; bit times (= 21 cycles).

;----------------------------------------------------------------------------
; push more registers and initialize values while we sample the first bits:
;----------------------------------------------------------------------------
haveTwoBitsK:
    push    x1              ;[1]
    push    x2              ;[3]
    push    x3              ;[5]
    ldi     shift, 0        ;[7]
    ldi     x3, 1<<4        ;[8] [rx loop init] first sample is inverse bit, compensate that
    push    x4              ;[9] == leap

    in      x1, USBIN       ;[11] <-- sample bit 0
    andi    x1, USBMASK     ;[12]
    bst     x1, USBMINUS    ;[13]
    bld     shift, 7        ;[14]
    push    cnt             ;[15]
    ldi     leap, 0         ;[17] [rx loop init]
    ldi     cnt, USB_BUFSIZE;[18] [rx loop init]
    rjmp    rxbit1          ;[19] arrives at [21]

;----------------------------------------------------------------------------
; Receiver loop (numbers in brackets are cycles within byte after instr)
;----------------------------------------------------------------------------

; duration of unstuffing code should be 10.66666667 cycles. We adjust "leap"
; accordingly to approximate this value in the long run.

unstuff6:
    andi    x2, USBMASK ;[03]
    ori     x3, 1<<6    ;[04] will not be shifted any more
    andi    shift, ~0x80;[05]
    mov     x1, x2      ;[06] sampled bit 7 is actually re-sampled bit 6
    subi    leap, -1    ;[07] total duration = 11 bits -> subtract 1/3
    rjmp    didUnstuff6 ;[08]

unstuff7:
    ori     x3, 1<<7    ;[09] will not be shifted any more
    in      x2, USBIN   ;[00] [10]  re-sample bit 7
    andi    x2, USBMASK ;[01]
    andi    shift, ~0x80;[02]
    subi    leap, 2     ;[03] total duration = 10 bits -> add 1/3
    rjmp    didUnstuff7 ;[04]

unstuffEven:
    ori     x3, 1<<6    ;[09] will be shifted right 6 times for bit 0
    in      x1, USBIN   ;[00] [10]
    andi    shift, ~0x80;[01]
    andi    x1, USBMASK ;[02]
    breq    se0         ;[03]
    subi    leap, -1    ;[04] total duration = 11 bits -> subtract 1/3
    nop2                ;[05]
    rjmp    didUnstuffE ;[06]

unstuffOdd:
    ori     x3, 1<<5    ;[09] will be shifted right 4 times for bit 1
    in      x2, USBIN   ;[00] [10]
    andi    shift, ~0x80;[01]
    andi    x2, USBMASK ;[02]
    breq    se0         ;[03]
    subi    leap, -1    ;[04] total duration = 11 bits -> subtract 1/3
    nop2                ;[05]
    rjmp    didUnstuffO ;[06]

rxByteLoop:
    andi    x1, USBMASK ;[03]
    eor     x2, x1      ;[04]
    subi    leap, 1     ;[05]
    brpl    skipLeap    ;[06]
    subi    leap, -3    ;1 one leap cycle every 3rd byte -> 85 + 1/3 cycles per byte
    nop                 ;1
skipLeap:
    subi    x2, 1       ;[08]
    ror     shift       ;[09]
didUnstuff6:
    cpi     shift, 0xfc ;[10]
    in      x2, USBIN   ;[00] [11] <-- sample bit 7
    brcc    unstuff6    ;[01]
    andi    x2, USBMASK ;[02]
    eor     x1, x2      ;[03]
    subi    x1, 1       ;[04]
    ror     shift       ;[05]
didUnstuff7:
    cpi     shift, 0xfc ;[06]
    brcc    unstuff7    ;[07]
    eor     x3, shift   ;[08] reconstruct: x3 is 1 at bit locations we changed, 0 at others
    st      y+, x3      ;[09] store data
rxBitLoop:
    in      x1, USBIN   ;[00] [11] <-- sample bit 0/2/4
    andi    x1, USBMASK ;[01]
    eor     x2, x1      ;[02]
    andi    x3, 0x3f    ;[03] topmost two bits reserved for 6 and 7
    subi    x2, 1       ;[04]
    ror     shift       ;[05]
    cpi     shift, 0xfc ;[06]
    brcc    unstuffEven ;[07]
didUnstuffE:
    lsr     x3          ;[08]
    lsr     x3          ;[09]
rxbit1:
    in      x2, USBIN   ;[00] [10] <-- sample bit 1/3/5
    andi    x2, USBMASK ;[01]
    breq    se0         ;[02]
    eor     x1, x2      ;[03]
    subi    x1, 1       ;[04]
    ror     shift       ;[05]
    cpi     shift, 0xfc ;[06]
    brcc    unstuffOdd  ;[07]
didUnstuffO:
    subi    bitcnt, 0xab;[08] == addi 0x55, 0x55 = 0x100/3
    brcs    rxBitLoop   ;[09]

    subi    cnt, 1      ;[10]
    in      x1, USBIN   ;[00] [11] <-- sample bit 6
    brcc    rxByteLoop  ;[01]
    rjmp    overflow

macro POP_STANDARD ; 14 cycles
    pop     cnt
    pop     x4
    pop     x3
    pop     x2
    pop     x1
    pop     shift
    pop     bitcnt
    endm
macro POP_RETI     ; 7 cycles
    pop     YH
    pop     YL
    out     SREG, YL
    pop     YL
    endm

/* Name: asmcommon.inc
 * Project: V-USB, virtual USB port for Atmel's(r) AVR(r) microcontrollers
 * Author: Christian Starkjohann
 * Creation Date: 2007-11-05
 * Tabsize: 4
 * Copyright: (c) 2007 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * Revision: $Id$
 */

/* Do not link this file! Link usbdrvasm.S instead, which includes the
 * appropriate implementation!
 */

/*
General Description:
This file contains assembler code which is shared among the USB driver
implementations for different CPU cocks. Since the code must be inserted
in the middle of the module, it's split out into this file and #included.

Jump destinations called from outside:
    sofError: Called when no start sequence was found.
    se0: Called when a package has been successfully received.
    overflow: Called when receive buffer overflows.
    doReturn: Called after sending data.

Outside jump destinations used by this module:
    waitForJ: Called to receive an already arriving packet.
    sendAckAndReti:
    sendNakAndReti:
    sendCntAndReti:
    usbSendAndReti:

The following macros must be defined before this file is included:
    .macro POP_STANDARD
    .endm
    .macro POP_RETI
    .endm
*/

#define token   x1

overflow:
    ldi     x2, 1<<USB_INTR_PENDING_BIT
    USB_STORE_PENDING(x2)       ; clear any pending interrupts
ignorePacket:
    clr     token
    rjmp    storeTokenAndReturn

;----------------------------------------------------------------------------
; Processing of received packet (numbers in brackets are cycles after center of SE0)
;----------------------------------------------------------------------------
;This is the only non-error exit point for the software receiver loop
;we don't check any CRCs here because there is no time left.
se0:
    subi    cnt, USB_BUFSIZE    ;[5]
    neg     cnt                 ;[6]
    sub     YL, cnt             ;[7]
    sbci    YH, 0               ;[8]
    ldi     x2, 1<<USB_INTR_PENDING_BIT ;[9]
    USB_STORE_PENDING(x2)       ;[10] clear pending intr and check flag later. SE0 should be over.
    ld      token, y            ;[11]
    cpi     token, USBPID_DATA0 ;[13]
    breq    handleData          ;[14]
    cpi     token, USBPID_DATA1 ;[15]
    breq    handleData          ;[16]
    lds     shift, usbDeviceAddr;[17]
    ldd     x2, y+1             ;[19] ADDR and 1 bit endpoint number
    lsl     x2                  ;[21] shift out 1 bit endpoint number
    cpse    x2, shift           ;[22]
    rjmp    ignorePacket        ;[23]
/* only compute endpoint number in x3 if required later */
#if USB_CFG_HAVE_INTRIN_ENDPOINT || USB_CFG_IMPLEMENT_FN_WRITEOUT
    ldd     x3, y+2             ;[24] endpoint number + crc
    rol     x3                  ;[26] shift in LSB of endpoint
#endif
    cpi     token, USBPID_IN    ;[27]
    breq    handleIn            ;[28]
    cpi     token, USBPID_SETUP ;[29]
    breq    handleSetupOrOut    ;[30]
    cpi     token, USBPID_OUT   ;[31]
    brne    ignorePacket        ;[32] must be ack, nak or whatever
;   rjmp    handleSetupOrOut    ; fallthrough

;Setup and Out are followed by a data packet two bit times (16 cycles) after
;the end of SE0. The sync code allows up to 40 cycles delay from the start of
;the sync pattern until the first bit is sampled. That's a total of 56 cycles.
handleSetupOrOut:               ;[32]
#if USB_CFG_IMPLEMENT_FN_WRITEOUT   /* if we have data for endpoint != 0, set usbCurrentTok to address */
    andi    x3, 0xf             ;[32]
    breq    storeTokenAndReturn ;[33]
    mov     token, x3           ;[34] indicate that this is endpoint x OUT
#endif
storeTokenAndReturn:
    sts     usbCurrentTok, token;[35]
doReturn:
    POP_STANDARD                ;[37] 12...16 cycles
    USB_LOAD_PENDING(YL)        ;[49]
    sbrc    YL, USB_INTR_PENDING_BIT;[50] check whether data is already arriving
    rjmp    waitForJ            ;[51] save the pops and pushes -- a new interrupt is already pending
sofError:
    POP_RETI                    ;macro call
    reti

handleData:
#if USB_CFG_CHECK_CRC
    CRC_CLEANUP_AND_CHECK       ; jumps to ignorePacket if CRC error
#endif
    lds     shift, usbCurrentTok;[18]
    tst     shift               ;[20]
    breq    doReturn            ;[21]
    lds     x2, usbRxLen        ;[22]
    tst     x2                  ;[24]
    brne    sendNakAndReti      ;[25]
; 2006-03-11: The following two lines fix a problem where the device was not
; recognized if usbPoll() was called less frequently than once every 4 ms.
    cpi     cnt, 4              ;[26] zero sized data packets are status phase only -- ignore and ack
    brmi    sendAckAndReti      ;[27] keep rx buffer clean -- we must not NAK next SETUP
#if USB_CFG_CHECK_DATA_TOGGLING
    sts     usbCurrentDataToken, token  ; store for checking by C code
#endif
    sts     usbRxLen, cnt       ;[28] store received data, swap buffers
    sts     usbRxToken, shift   ;[30]
    lds     x2, usbInputBufOffset;[32] swap buffers
    ldi     cnt, USB_BUFSIZE    ;[34]
    sub     cnt, x2             ;[35]
    sts     usbInputBufOffset, cnt;[36] buffers now swapped
    rjmp    sendAckAndReti      ;[38] 40 + 17 = 57 until SOP

handleIn:
;We don't send any data as long as the C code has not processed the current
;input data and potentially updated the output data. That's more efficient
;in terms of code size than clearing the tx buffers when a packet is received.
    lds     x1, usbRxLen        ;[30]
    cpi     x1, 1               ;[32] negative values are flow control, 0 means "buffer free"
    brge    sendNakAndReti      ;[33] unprocessed input packet?
    ldi     x1, USBPID_NAK      ;[34] prepare value for usbTxLen
#if USB_CFG_HAVE_INTRIN_ENDPOINT
    andi    x3, 0xf             ;[35] x3 contains endpoint
#if USB_CFG_SUPPRESS_INTR_CODE
    brne    sendNakAndReti      ;[36]
#else
    brne    handleIn1           ;[36]
#endif
#endif
    lds     cnt, usbTxLen       ;[37]
    sbrc    cnt, 4              ;[39] all handshake tokens have bit 4 set
    rjmp    sendCntAndReti      ;[40] 42 + 16 = 58 until SOP
    sts     usbTxLen, x1        ;[41] x1 == USBPID_NAK from above
    ldi     YL, lo8(usbTxBuf)   ;[43]
    ldi     YH, hi8(usbTxBuf)   ;[44]
    rjmp    usbSendAndReti      ;[45] 57 + 12 = 59 until SOP

; Comment about when to set usbTxLen to USBPID_NAK:
; We should set it back when we receive the ACK from the host. This would
; be simple to implement: One static variable which stores whether the last
; tx was for endpoint 0 or 1 and a compare in the receiver to distinguish the
; ACK. However, we set it back immediately when we send the package,
; assuming that no error occurs and the host sends an ACK. We save one byte
; RAM this way and avoid potential problems with endless retries. The rest of
; the driver assumes error-free transfers anyway.

#if !USB_CFG_SUPPRESS_INTR_CODE && USB_CFG_HAVE_INTRIN_ENDPOINT /* placed here due to relative jump range */
handleIn1:                      ;[38]
#if USB_CFG_HAVE_INTRIN_ENDPOINT3
; 2006-06-10 as suggested by O.Tamura: support second INTR IN / BULK IN endpoint
    cpi     x3, USB_CFG_EP3_NUMBER;[38]
    breq    handleIn3           ;[39]
#endif
    lds     cnt, usbTxLen1      ;[40]
    sbrc    cnt, 4              ;[42] all handshake tokens have bit 4 set
    rjmp    sendCntAndReti      ;[43] 47 + 16 = 63 until SOP
    sts     usbTxLen1, x1       ;[44] x1 == USBPID_NAK from above
    ldi     YL, lo8(usbTxBuf1)  ;[46]
    ldi     YH, hi8(usbTxBuf1)  ;[47]
    rjmp    usbSendAndReti      ;[48] 50 + 12 = 62 until SOP

#if USB_CFG_HAVE_INTRIN_ENDPOINT3
handleIn3:
    lds     cnt, usbTxLen3      ;[41]
    sbrc    cnt, 4              ;[43]
    rjmp    sendCntAndReti      ;[44] 49 + 16 = 65 until SOP
    sts     usbTxLen3, x1       ;[45] x1 == USBPID_NAK from above
    ldi     YL, lo8(usbTxBuf3)  ;[47]
    ldi     YH, hi8(usbTxBuf3)  ;[48]
    rjmp    usbSendAndReti      ;[49] 51 + 12 = 63 until SOP
#endif
#endif


; USB spec says:
; idle = J
; J = (D+ = 0), (D- = 1)
; K = (D+ = 1), (D- = 0)
; Spec allows 7.5 bit times from EOP to SOP for replies

bitstuffN:
    eor     x1, x4          ;[5]
    ldi     x2, 0           ;[6]
    nop2                    ;[7]
    nop                     ;[9]
    out     USBOUT, x1      ;[10] <-- out
    rjmp    didStuffN       ;[0]
    
bitstuff6:
    eor     x1, x4          ;[5]
    ldi     x2, 0           ;[6] Carry is zero due to brcc
    rol     shift           ;[7] compensate for ror shift at branch destination
    rjmp    didStuff6       ;[8]

bitstuff7:
    ldi     x2, 0           ;[2] Carry is zero due to brcc
    rjmp    didStuff7       ;[3]


sendNakAndReti:
    ldi     x3, USBPID_NAK  ;[-18]
    rjmp    sendX3AndReti   ;[-17]
sendAckAndReti:
    ldi     cnt, USBPID_ACK ;[-17]
sendCntAndReti:
    mov     x3, cnt         ;[-16]
sendX3AndReti:
    ldi     YL, 20          ;[-15] x3==r20 address is 20
    ldi     YH, 0           ;[-14]
    ldi     cnt, 2          ;[-13]
;   rjmp    usbSendAndReti      fallthrough

;usbSend:
;pointer to data in 'Y'
;number of bytes in 'cnt' -- including sync byte [range 2 ... 12]
;uses: x1...x4, btcnt, shift, cnt, Y
;Numbers in brackets are time since first bit of sync pattern is sent
;We don't match the transfer rate exactly (don't insert leap cycles every third
;byte) because the spec demands only 1.5% precision anyway.
usbSendAndReti:             ; 12 cycles until SOP
    in      x2, USBDDR      ;[-12]
    ori     x2, USBMASK     ;[-11]
    sbi     USBOUT, USBMINUS;[-10] prepare idle state; D+ and D- must have been 0 (no pullups)
    in      x1, USBOUT      ;[-8] port mirror for tx loop
    out     USBDDR, x2      ;[-7] <- acquire bus
; need not init x2 (bitstuff history) because sync starts with 0
    ldi     x4, USBMASK     ;[-6] exor mask
    ldi     shift, 0x80     ;[-5] sync byte is first byte sent
txByteLoop:
    ldi     bitcnt, 0x35    ;[-4] [6] binary 0011 0101
txBitLoop:
    sbrs    shift, 0        ;[-3] [7]
    eor     x1, x4          ;[-2] [8]
    out     USBOUT, x1      ;[-1] [9] <-- out N
    ror     shift           ;[0] [10]
    ror     x2              ;[1]
didStuffN:
    cpi     x2, 0xfc        ;[2]
    brcc    bitstuffN       ;[3]
    lsr     bitcnt          ;[4]
    brcc    txBitLoop       ;[5]
    brne    txBitLoop       ;[6]

    sbrs    shift, 0        ;[7]
    eor     x1, x4          ;[8]
didStuff6:
    out     USBOUT, x1      ;[-1] [9] <-- out 6
    ror     shift           ;[0] [10]
    ror     x2              ;[1]
    cpi     x2, 0xfc        ;[2]
    brcc    bitstuff6       ;[3]
    ror     shift           ;[4]
didStuff7:
    ror     x2              ;[5]
    sbrs    x2, 7           ;[6]
    eor     x1, x4          ;[7]
    nop                     ;[8]
    cpi     x2, 0xfc        ;[9]
    out     USBOUT, x1      ;[-1][10] <-- out 7
    brcc    bitstuff7       ;[0] [11]
    ld      shift, y+       ;[1]
    dec     cnt             ;[3]
    brne    txByteLoop      ;[4]
;make SE0:
    cbr     x1, USBMASK     ;[5] prepare SE0 [spec says EOP may be 21 to 25 cycles]
    lds     x2, usbNewDeviceAddr;[6]
    lsl     x2              ;[8] we compare with left shifted address
    subi    YL, 20 + 2      ;[9] Only assign address on data packets, not ACK/NAK in x3
    sbci    YH, 0           ;[10]
    out     USBOUT, x1      ;[11] <-- out SE0 -- from now 2 bits = 22 cycles until bus idle
;2006-03-06: moved transfer of new address to usbDeviceAddr from C-Code to asm:
;set address only after data packet was sent, not after handshake
    breq    skipAddrAssign  ;[0]
    sts     usbDeviceAddr, x2; if not skipped: SE0 is one cycle longer
skipAddrAssign:
;end of usbDeviceAddress transfer
    ldi     x2, 1<<USB_INTR_PENDING_BIT;[2] int0 occurred during TX -- clear pending flag
    USB_STORE_PENDING(x2)   ;[3]
    ori     x1, USBIDLE     ;[4]
    in      x2, USBDDR      ;[5]
    cbr     x2, USBMASK     ;[6] set both pins to input
    mov     x3, x1          ;[7]
    cbr     x3, USBMASK     ;[8] configure no pullup on both pins
    ldi     x4, 4           ;[9]
se0Delay:
    dec     x4              ;[10] [13] [16] [19]
    brne    se0Delay        ;[11] [14] [17] [20]
    out     USBOUT, x1      ;[21] <-- out J (idle) -- end of SE0 (EOP signal)
    out     USBDDR, x2      ;[22] <-- release bus now
    out     USBOUT, x3      ;[23] <-- ensure no pull-up resistors are active
    rjmp    doReturn

#   elif USB_CFG_CLOCK_KHZ == 16500
#       include "usbdrvasm165.inc"
#   elif USB_CFG_CLOCK_KHZ == 20000
#       include "usbdrvasm20.inc"
#   else
#       error "USB_CFG_CLOCK_KHZ is not one of the supported non-crc-rates!"
#   endif
#endif /* USB_CFG_CHECK_CRC */
