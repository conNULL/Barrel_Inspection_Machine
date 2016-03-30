
#include <p18f4620.inc>	
	;Declare unbanked variables (at 0x70 and on)
	udata
lcd_tmp	res	1
lcd_d1	res	1
lcd_d2	res	1
lcd_td	res	1
dat	res	1
com	res	1
	;Declare constants for pin assignments (LCD on PORTD)
RS 	equ 2
E 	equ 3

	;Helper macros
WRT_LCD macro val
	movlw   val
	call    WrtLCD
	endm
	
;Delay: ~44us
LCD_DELAY macro
	movlw   0x23
	movwf   lcd_d1
	decfsz  lcd_d1,f
	goto    $-2
	endm
	

	code
	global InitLCD,WrtLCD,ClkLCD,ClrLCD, WR_DATA, WR_INS	;Only these functions are visible to other asm files.
    ;***********************************
InitLCD

	;bsf PORTD,E     ;E default high
	
	;Wait for LCD POR to finish (~15ms)
	call lcdLongDelay
	call lcdLongDelay
	call lcdLongDelay

	;Ensure 8-bit mode first (no way to immediately guarantee 4-bit mode)
	; -> Send b'0011' 3 times
    bcf     PORTD,RS       ;Instruction mode
	movlw   B'00110000'
	call    MovMSB
	call    ClkLCD         ;Finish last 4-bit send (if reset occurred in middle of a send)
	call    ClkLCD         ;Assuming 4-bit mode, set 8-bit mode
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	call    ClkLCD         ;(note: if it's in 8-bit mode already, it will stay in 8-bit mode)

    ;Now that we know for sure it's in 8-bit mode, set 4-bit mode.
	movlw B'00100000'
	call MovMSB
	call ClkLCD
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	;Give LCD init instructions
	WRT_LCD B'00101000' ; 4 bits, 2 lines,5X8 dot
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	WRT_LCD B'00001111' ; display on,cursor,blink
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	WRT_LCD B'00000110' ; Increment,no shift
	call    lcdLongDelay   ;->max instruction time ~= 5ms
	;Ready to display characters
	call    ClrLCD
    bsf     PORTD,RS    ;Character mode
	return
    ;************************************
WR_DATA
	bsf     PORTD, RS				
	movwf	dat
	movf	dat,w
	andlw	0xF0		
	addlw	4
	movwf	PORTD		
	bsf	PORTD, E				;
	call	lcdLongDelay	;__    __
	bcf	PORTD, E				;  |__|
	swapf	dat,w
	andlw	0xF0		
	addlw	4
	movwf	PORTD		
	bsf	PORTD, E				;
	call	lcdLongDelay	;__    __
	bcf	PORTD, E				;  |__|
	return
	;WrtLCD: Clock MSB and LSB of W to PORTD<7:4> in two cycles
WR_INS
	bcf	PORTD, RS				;clear RS
	movwf	com				;W --> com
	andlw	0xF0			;mask 4 bits MSB w = X0
	movwf	PORTD			;Send 4 bits MSB
	bsf	PORTD, E				;
	call	lcdLongDelay	;__    __
	bcf	PORTD, E				;  |__|
	swapf	com,w
	andlw	0xF0			;1111 0010
	movwf	PORTD			;send 4 bits LSB
	bsf	PORTD, E				;
	call	lcdLongDelay	;__    __
	bcf	PORTD, E				;  |__|
	call	lcdLongDelay
	return
WrtLCD
	movwf   lcd_tmp ; store original value
	call    MovMSB  ; move MSB to PORTD
	call    ClkLCD
	swapf   lcd_tmp,w ; Swap LSB of value into MSB of W
    call    MovMSB    ; move to PORTD
    call    ClkLCD
    ;call    tdelay    ;4swag
    
    return
    
;tdelay:    slight delay between writing characters
tdelay
	movlw   d'255'
	movwf   lcd_td
 tloop	    
	call    lcdLongDelay
	decfsz  lcd_td
	goto	tloop
    return
    
    ;ClrLCD: Clear the LCD display
ClrLCD
    bcf     PORTD,RS       ;Instruction mode
    WRT_LCD b'00000001'
    call    lcdLongDelay
    bsf	    PORTD, RS
    return

    ;ClkLCD: Pulse the E line low
ClkLCD
    ;LCD_DELAY
    bsf PORTD,E
    nop
	;LCD_DELAY   ; __    __
    bcf PORTD,E ;   |__|
	LCD_DELAY
    return

    ;****************************************

    ;MovMSB: Move MSB of W to PORTD, without disturbing LSB
MovMSB
    andlw 0xF0
    iorwf PORTD,f
    iorlw 0x0F
    andwf PORTD,f
    return

    ;Delay: ~5ms
lcdLongDelay
    movlw d'80'
    movwf lcd_d2
LLD_LOOP
    LCD_DELAY
    decfsz lcd_d2,f
    goto LLD_LOOP
    return
    

    end


