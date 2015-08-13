; Proof of concept for Banggood 4 digit 7 segment common anode display
; with dual 74hc595 latch - running on a Microchip PIC 12f1840 microcontroller
; Setting number values and preparing output done in main program
; running the display done using TMR0 interrupt @ 1MHz/256/4 ~ 975Hz
; or just over 1ms. Each interrupt step will either show next digit,
; or simply count down a delay depending on number of segments active
; more segments need more time to display, thus longer delay.
; The display data is collected in a structure of 12 bytes for the 4 digits
; Numeric value, Segments code, Delay. To blank a digit, enter segment
; code 0xff and delay 0x00.
; num0, segment0, delay0, num1, segment1, delay1, ... , segment3, delay3
; five bytes are also set aside for isr (interrupt service routine)
; to perform the actual display.
; To stop displaying, set all blank, then clear INTCON.T0IE to disable interrupt.

#include "p12f1840.inc"
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_ON & _CLKOUTEN_OFF & _IESO_ON & _FCMEN_ON
 __CONFIG _CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _BORV_LO & _LVP_OFF
; use internal oscillator (no external parts), disable watchdog timer.
; enable power-on timer, don't use mClr (it's now an IO pin)
; disable all protection of memory .. disable low voltage programming

; software configuration of the microcontroller
OSCCONBITS EQU (b'1101' << IRCF0) | (b'10' << SCS0) ; 4MHz clock, 1MHz tick
TRISABITS  EQU 0x00 ; all outputs
ANSELABITS EQU 0x00 ; analog select = none
OPTIONBITS EQU b'001' | (1 << NOT_WPUEN) ; no WPU, use TMR0 Prescaler at 1:4
APFCONBITS EQU 0x00 ; no reconfiguration of pin functions
ADCON0BITS EQU 0x00 ; no ad in use
ADCON1BITS EQU b'001' << ADCS0  ; AD conversion clock = Fosc/8
T1CONBITS  EQU 0x00 ; Timer 1 is off, no prescaler = Fosc/4

;     the pins      ;        ICSP             ;        what's in use
;       -----       ;        ----             ;            -----
; Vdd o|1   8|o Vss ;  Vdd o|1   8|o Vss      ;      +5v o|1   8|o gnd
; RA5 o|2   7|o RA0 ;      o|2   7|o icspdat  ;     sclk o|2   7|o
; RA4 o|3   6|o RA1 ;      o|3   6|o icspclk  ;     rclk o|3   6|o
; RA3 o|4   5|o RA2 ; mclr o|4   5|o          ;          o|4   5|o din
;       -----       ;        -----            ;            -----
; the pins of the only port, PORTA ..
rclk EQU RA4
sclk EQU RA5
din  EQU RA2

        cblock 0x20         ; bank0 RAM, 80 bytes
            ledStruct:12    ; value, segment-data and delay for each 4 leds
        endc

        cblock 0x70         ; 16 bytes of bank-free RAM
            counter         ; count 16 bits to send
            isrDigit        ; output digit(s) value (low byte)
            isrSegments     ; output segments for this digit
            isrDelay        ; number of segments lighted up
            isrTemp         ; temp variable for isr
            isrCurrent      ; current digit number
            temp
        endc

        org 0x00            ; reset vector
        goto init
        org 0x04            ; interrupt vector
        goto isr
init
        movlb 0             ; bank 0
        clrf INTCON         ; turn off all interrupts for now
        movlw T1CONBITS     ; settings for timer1
        movwf T1CON
        clrf PORTA          ; all outputs low
        movlb 1             ; bank 1
        movlw OPTIONBITS    ; option settings, practically timer0
        movwf OPTION_REG    ; could have been written simply "option"
        movlw OSCCONBITS    ; 4MHz clock, 1MHz instruction cycle
        movwf OSCCON
        clrf PIE1           ; clear all interrupt peripherals enable flags
        clrf PIE2
        movlw TRISABITS     ; port pins defined as inputs or outputs
        movwf TRISA         ; could be "tris 5" for "load W to TRISA"
        movlw ADCON0BITS    ; basic ADConverter configuration
        movwf ADCON0
        movlw ADCON1BITS    ; more ADConverter configuration
        movwf ADCON1
        movlb 2             ; bank 2
        movlw APFCONBITS
        movwf APFCON        ; configure alternative pins for peripheral functions
        movlb 3             ; bank 3
        movlw ANSELABITS
        movwf ANSELA        ; Analog select
        movlb 0             ; default to bank 0 
        clrf FSR0H          ; indirect 0 used for interrupts
        clrf FSR1H          ; indirects used for bank 0 traditional memory
main

        ; test display, show 23.59
        movlw 0x02          ; first digit - digit 3
        movwf ledStruct+9
        movlw 0x83          ; second digit, with decimal point - digit 2
        movwf ledStruct+6
        movlw 0x05          ; third digit - digit 1
        movwf ledStruct+3
        movlw 0x09          ; fourth digit - digit 0
        movwf ledStruct
        
        call calcLedData    ; prepare segments and delay-values
        clrf isrDigit       ; begin with digit 0 (rightmost)
        bsf INTCON, T0IE    ; enable timer0 interrupt
        bsf INTCON, GIE     ; interrupts master switch is on

        goto $              ; nothing more happens here ..

calcLedData
    ; given ledValues, calculate ledSegments and ledDelays
        movlw ledStruct         ; pointer to the data structure
        movwf FSR1L             ; we will use indirect register 1
        movlw 0x04              ; four digits to process
        movwf temp
calcLoop
        moviw FSR1++            ; w=numeric value, FSR offset = 1 (point to segments)
        call lookupSegments     ; convert to segments
        movwi FSR1--            ; store in segment, offset = 0 (point to num. value)
        moviw FSR1++            ; read numeric value again, offset = 1
        call lookupDelay        ; get appropriate delay value
        movwi ++FSR1            ; and store in delay, offset 2
        decf FSR1, f            ; step back to offset 1
        decf FSR1, f            ; .. to offset 0
        btfss INDF1, 7          ; decimal point ?
        goto calcNoDP
        incf FSR1, f            ; offset 1 = segment
        bcf INDF1, 7            ; yes, light the DP by setting bit 7 = 0
        incf FSR1, f            ; offset 2 = delay
        incf INDF1, f           ; add one to delay for the decimal point
        goto calcDP
calcNoDP
        incf FSR1, f            ; offset 1
        incf FSR1, f            ; offset 2
calcDP
        incf FSR1, f            ; next digit in the structure
        decfsz temp, f          ; done with all four ?
        goto calcLoop           ; no, do next digit
        return

sendit
        movlw 0x10              ; 16 bits to send
        movwf counter
        bcf PORTA, rclk         ; lower the RCLK for coming word input
sendLoop
        bcf PORTA, sclk         ; lower SCLK for coming bit
        bcf PORTA, din          ; assume low bit output
        lslf isrSegments, w     ; get msb of high byte
        rlf isrDigit, f         ; rotate into low byte
        rlf isrSegments, f      ; and beyond into high byte
        btfsc STATUS, C         ; C=1 ?
        bsf PORTA, din          ; yes, high data bit output
        nop                     ; allow data line to stabilize high
        bsf PORTA, sclk         ; clock the bit in with a rising edge
        decfsz counter, f       ; all 16 bits done ?
        goto sendLoop           ; no, continue
        bcf PORTA, sclk         ; last bit sent, lower SCLK
        bsf PORTA, rclk         ; and latch the word with RCLK
        return

lookupSegments
        ; assume input 0-15 (0x00-0x0f) - return segment code
        ; using standard 7 segment a-g mapping ( bit 7 for decimal point )
        ; these bits are active low, so 0's are displayed and 1's are not.
        andlw 0x0f              ; make sure a 0-15 input value only
        brw                     ; jump W steps ahead
        ; segs  .gfedcba
        retlw b'11000000' ; 0   ; and return the relevant segment mapping
        retlw b'11111001' ; 1
        retlw b'10100100' ; 2
        retlw b'10110000' ; 3
        retlw b'10011001' ; 4
        retlw b'10010010' ; 5
        retlw b'10000010' ; 6
        retlw b'11111000' ; 7
        retlw b'10000000' ; 8
        retlw b'10010000' ; 9
        retlw b'10001000' ; A
        retlw b'10000011' ; b
        retlw b'11000110' ; C
        retlw b'10100001' ; d
        retlw b'10000110' ; E
        retlw b'10001110' ; F
lookupDelay
        ; return number of lighted segments (0's here)
        ; for given value 0-15, so we can know how long to display a digit
        ; more segments need more time to appear as bright as digits with
        ; fewer active segments. A blank digit needs no delay.
        ; these values look ok, but one could play around with them
        ; perhaps adding or subtracting 1 from all values to synchronize
        ; relative brightness of digits with different segment patterns showing
        andlw 0x0f                      ; limit input value to 15 (0x0f)
        brw                             ; jump ahead W steps
        retlw 0x06 ; b'11000000' ; 0    ; and return delay value in W
        retlw 0x02 ; b'11111001' ; 1
        retlw 0x05 ; b'10100100' ; 2
        retlw 0x05 ; b'10110000' ; 3
        retlw 0x04 ; b'10011001' ; 4
        retlw 0x05 ; b'10010010' ; 5
        retlw 0x06 ; b'10000010' ; 6
        retlw 0x03 ; b'11111000' ; 7
        retlw 0x07 ; b'10000000' ; 8
        retlw 0x06 ; b'10010000' ; 9
        retlw 0x06 ; b'10001000' ; A
        retlw 0x05 ; b'10000011' ; b
        retlw 0x04 ; b'11000110' ; C
        retlw 0x05 ; b'10100001' ; d
        retlw 0x05 ; b'10000110' ; E
        retlw 0x04 ; b'10001110' ; F

isr
        ; our only isr is TMR0, to drive LED
        ; otherwise we might have to check INTCON.T0IF to see if we should run
        movlb 0              ; work in bank 0.
        ; Context is saved, and restored by "retfie" on this processor.
        bcf INTCON, T0IF     ; yes, we are handling TOIF condition
        movf isrDelay, f     ; is there a delay in progress ? (test-instruction)
        btfss STATUS, Z      ; skip if zero ..
        goto isrDelayHandler ; handle delay ( by counting it down ) - non-zero
        ; a previous delay is now finished. Show next digit:
        movfw isrCurrent     ; what digit are we going to display ? [0-3]
        addwf isrCurrent, w  ; multiply by 2
        addwf isrCurrent, w  ; multiply by 3, to get to right position in data
        addlw ledStruct+1    ; pointer to current digit segment data
        movwf FSR0           ; using indirect register #0
        moviw FSR0++         ; get segments for this digit
        movwf isrSegments
        movfw INDF0         ; get delay for this digit
        movwf isrDelay
        movlw 0x01          ; what display digit ? Make a mask with correct bit set
        movwf isrDigit      ; 0000 0001 for digit 0 ..010 for digit 1 and so on
        movfw isrCurrent    ; what digit are we displaying ? [0-3]
        movf 0x09, f        ; test Wreg for zero content
        btfsc STATUS, Z     ; zero means finished ..
        goto isrDigitReady  ; and we're done
        addlw 0xff          ; subtract one from W if greater than 0
        lslf isrDigit, f    ; and shift our mask up one bit
        goto $-5            ; check again if the right digit bit is now set
isrDigitReady
        call sendit         ; send this digit to the display module
        incf isrCurrent     ; prepare for next digit next time, after delay
        btfsc isrCurrent, 2 ; bit 2 set = value 4,
        clrf isrCurrent     ; so wrap around to 0 as we only work on [0-3]
        retfie              ; display isr-handler finished
isrDelayHandler
        decf isrDelay, f    ; delay handler is < 10 instructions
        retfie

        end

