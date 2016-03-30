
#include <p18f4620.inc>
#include <lcd.inc>
list P=18F4620, F=INHX32, C=160, N=80, ST=OFF, MM=OFF, R=DEC

;;;;;;Configuration Bits;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		CONFIG OSC=HS, FCMEN=OFF, IESO=OFF
		CONFIG PWRT = OFF, BOREN = SBORDIS, BORV = 3
		CONFIG WDT = OFF, WDTPS = 32768
		CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF, CCP2MX = PORTC
		CONFIG STVREN = ON, LVP = OFF, XINST = OFF
		CONFIG DEBUG = OFF
		CONFIG CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
		CONFIG CPB = OFF, CPD = OFF
		CONFIG WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
		CONFIG WRTB = OFF, WRTC = OFF, WRTD = OFF
		CONFIG EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
		CONFIG EBTRB = OFF


;;;;;;Vectors;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


cblock 0x70
		numb
		dt1			;0x71		 addresses are used for the RTC module
        dt2			;0x72
        ADD			;0x73
        DAT			;0x74
        DOUT		;DOUT
        B1			;B1
	dig10		;dig10
	dig1		;dig1
	numb2
	tc
	dat
	dig2
	dig3
	lcd_td
	lcd_sd
	lcd_d1
	lcd_d2
	dig12
	dig22
	dig32
	left
	tot
	tot2
	temp
	temp2
	temp3
	time
	pwm
	sensitivity
	renc
	dt3
	XBUF
	count
	CONTROL
	flag
	B2
	ssec
	smin
	fsec
	fmin
	opmin
	opsec
	optime
	cap1
	capinv
	numBar
	Empty
	Half
	Full
	bt
	rightE
	leftE
	currentB
	b1L
	b1Li
	b2L
	b2Li
	b3L
	b3Li
	b4L
	b4Li
	b5L
	b5Li
	b6L
	b6Li
	b7L
	b7Li
	sdirection
	cdirection
	pwmL
	pwmR
	dirdiff
	USOF
	boolENC
	gyrox
	xhead
	bheight
	tallB
	shortB
	
	endc
		
RS 	equ 2
E 	equ 3
Ca	equ 1
	
variable _waitknt = 0
	
udata
	COUNTH	res 1	;const used in delay
	COUNTM	res	1	;const used in delay
	COUNTL	res	1	;const used in delay
org	0x0000
goto	Init
org	0x0008
call	enchandle
nop
retfie



carry	macro	num
	bcf	STATUS, C
	local	carend
	movlw	d'9'
	cpfsgt	num
	goto	carend
	subwf	num
	decf	num
	bsf	STATUS, C
carend	
	endm
write	macro   msg
	local	loop_
	movlw	upper msg
	movwf	TBLPTRU
	movlw	high msg
	movwf	TBLPTRH
	movlw	low msg
	movwf	TBLPTRL
	tblrd*
	movf	 TABLAT, W
	loop_	
	call	 WrtLCD
	tblrd+*
	movf	 TABLAT, W
	bnz	loop_
	endm
wdig1
	    movf    dig12, W
	    ;addlw   0x30
	    ;call    WR_DATA
	    ;return
	     addwf   WREG, W
	   
	   addwf   PCL, f
	   dt	   "0123456789"
	    
wdig2
	    movf    dig2, W
	    addwf   WREG, W
	    addwf   PCL, f
	    dt	   "0123456789"
wdig3
	    movf    dig3, W
	    addwf   WREG, W
	    addwf   PCL, f
	    dt	   "0123456789"
Ready
	    db	" Start-*", 0
RightInt
	    db	"Right", 0
LeftInt	   
	    db	"Left", 0
Running
	    db  "Running", 0
Done
	    db	"B: ", 0
Done2
	    db	" F: ", 0
Done3
	    db	"                           ", "H: ", 0
Done4
	    db	" E: ", 0
Done5
	    db	"L: ", 0
Done6	    
	    db	" R: ", 0
Barrel	
	    db	"No ", 0
Fulls
	    db  " Full: ", 0
Location
	    db "                      ","Location: ", 0
OperTime
	    db "Operation time: ", "                        ", 0
Halfs	
	    db	" Half; ", 0
Emptys	    
	    db	" Empt: ", 0
Nob 
	    db	"No more barrels", 0
seconds
	    db	"seconds", 0
	    
Hob
	    db	"Height:    ", 0
	    
ftall	
	    db	"tall", 0
fshort
	    db	"short", 0

stime	macro	tin tout
	movff	tin, temp
	movlw	b'00001111'
	andwf	temp, f
	movff	tin, temp2
	movlw	b'11110000'
	andwf	temp2, f
	
	endm


sepdig	macro	   num
	local	   loop3
	local	   loop2
	local	   loop1
	local	   fin
	movlw	   b'0'
	movwf	   dig12
	movwf	   dig2
	movwf	   dig3
	
	movf	   num, W
	movwf	   tot
loop3	movlw	   d'99'
	cpfsgt	   tot
	goto	   loop2
	incf	   dig12
	subwf	   tot, f
	decf	   tot, f
	goto	   loop3
loop2	movlw	   d'9'
	cpfsgt	   tot
	goto	   loop1
	incf	   dig2
	subwf	   tot, f
	decf	   tot, f
	goto	   loop2
loop1	movlw	   d'0'
	cpfsgt	   tot
	goto	   fin
	incf	   dig3
	subwf	   tot, f
	decf	   tot, f
	goto	   loop1
fin	
	endm
	
Save_BarrelH macro   Loc, Fl
	    movff   rightE, Loc
	    clrf	   Fl
	    btfsc	   PORTA, 0
	    bsf		   Fl, 2
	    btfsc	   PORTA, 1
	    bsf		   Fl, 1
	    btfss	   PORTA, 1
	    bsf		   Fl, 0
	    btfsc	   PORTA, 0
	    bcf		   Fl, 1
	    endm
	    
Save_BarrelL macro   Loc, Fl
	    movff   rightE, Loc
	    clrf	   Fl
	    btfsc	   PORTA, 2
	    bsf		   Fl, 2
	    btfsc	   PORTA, 3
	    bsf		   Fl, 1
	    btfss	   PORTA, 3
	    bsf		   Fl, 0
	    btfsc	   PORTA, 2
	    bcf		   Fl, 1
	    endm

	    
dFull	macro	   fulln
	local	   wfull
	local	   whalf
	local	   wempty
	local	   wend
	btfsc	   fulln, 2
	goto	    wfull
	btfsc	   fulln, 1
	goto	   whalf
	goto	   wempty
wfull	write	   Fulls
	goto	   wend
whalf	write	   Halfs
	goto	   wend
wempty	write	   Emptys	
wend
	endm
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;RTC;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
i2c_common_check_ack	macro	err_address		;If bad ACK bit received, goto err_address
	;banksel		SSPCON2
    btfsc       SSPCON2,ACKSTAT
    goto        err_address
    endm

i2c_common_start	macro
;input:		none
;output:	none
;desc:		initiate start conditionon the bus

    local	back1
    bsf         SSPCON2,SEN
back1   
    btfss       PIR1,SSPIF
    ;btfsc         SSPCON2,SEN
    goto        back1
    bcf		PIR1,SSPIF
    
    endm

i2c_common_stop	macro
;input: 	none
;output:	none
;desc:		initiate stop condition on the bus
	;banksel     SSPCON2
    local	back1
    bsf         SSPCON2,PEN
back1   
    btfss       PIR1,SSPIF
    ;btfsc        SSPCON2,PEN
    goto        back1
    bcf		PIR1,SSPIF
    endm

i2c_common_repeatedstart	macro
;input:		none
;output:	none
;desc:		initiate repeated start on the bus. Usually used for
;			changing direction of SDA without STOP event
	;banksel     SSPCON2
    local	back1
    bsf         SSPCON2,RSEN
back1   
    btfss       PIR1,SSPIF
    ;btfsc         SSPCON2,RSEN
    goto        back1
    bcf		PIR1,SSPIF
    endm

i2c_common_ack		macro
;input:		none
;output:	none
;desc:		send an acknowledge to slave device
   ; banksel     SSPCON2
    local	back1
    bcf         SSPCON2,ACKDT
    bsf         SSPCON2,ACKEN
back1   
    btfss       PIR1,SSPIF
    ;btfsc       SSPCON2,ACKEN
    goto        back1
    bcf		PIR1,SSPIF
    endm

i2c_common_nack	macro
;input:		none
;output:	none
;desc:		send an not acknowledge to slave device
   ;banksel     SSPCON2
    local	back1
    bsf         SSPCON2,ACKDT
    bsf         SSPCON2,ACKEN
back1   
    btfss       PIR1,SSPIF
    ;btfsc       SSPCON2,ACKEN
    goto        back1
    bcf		PIR1,SSPIF
    endm

i2c_common_write	macro
    local	back1
;input:		W
;output:	to slave device
;desc:		writes W to SSPBUF and send to slave device. Make sure
;			transmit is finished before continuing
  
    movwf       SSPBUF
back1   
    btfss       PIR1,SSPIF
    ;btfsc       SSPSTAT,R_W 
    goto        back1
    bcf		PIR1,SSPIF
    endm

i2c_common_read	macro
    local	back1
;input:		none
;output:	W
;desc:		reads data from slave and saves it in W.
   
    bsf         SSPCON2,RCEN    ;Begin receiving byte from
back1   
    btfss       PIR1,SSPIF
    ;btfsc       SSPCON2,RCEN
    goto        back1
    bcf		PIR1,SSPIF
    movf        SSPBUF,w
    endm
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;\
i2c_common_setup
;input:		none
;output:	none
;desc:		sets up I2C as master device with 100kHz baud rate

    clrf        SSPSTAT         ;I2C line levels, and clear all flags
    movlw       d'24'         	;100kHz baud rate: 10MHz osc / [4*(24+1)]

    movwf       SSPADD          ;RTC only supports 100kHz

    movlw       b'00101000'     ;Config SSP for Master Mode I2C

    movwf       SSPCON1
    bsf         SSPCON1 ,SSPEN    ;Enable SSP module
    
    i2c_common_stop        		;Ensure the bus is free
	return

;rtc Algorithms;;;;;;
	
write_gyro
	i2c_common_start
        movlw       0xD4        ;DS1307 address | WRITE bit
        i2c_common_write
        i2c_common_check_ack   gWR_ERR

        ;Write data to I2C bus (Register Address in RTC)

        movf        ADD,w       ;Set register pointer in RTC
        i2c_common_write
        i2c_common_check_ack   gWR_ERR

        ;Write data to I2C bus (Data to be placed in RTC register)

        movf       DAT,w       ;Write data to register in RTC
        i2c_common_write
        i2c_common_check_ack   gWR_ERR
        goto        gWR_END
gWR_ERR
        nop
gWR_END  
		i2c_common_stop	;Release the I2C bus
        return
write_mg
	i2c_common_start
        movlw       0x3C        ;DS1307 address | WRITE bit
        i2c_common_write
        i2c_common_check_ack   mWR_ERR

        ;Write data to I2C bus (Register Address in RTC)

        movf        ADD,w       ;Set register pointer in RTC
        i2c_common_write
        i2c_common_check_ack   mWR_ERR

        ;Write data to I2C bus (Data to be placed in RTC register)

        movf       DAT,w       ;Write data to register in RTC
        i2c_common_write
        i2c_common_check_ack   mWR_ERR
        goto        mWR_END
mWR_ERR
        nop
mWR_END  
		i2c_common_stop	;Release the I2C bus
        return
	
write_rtc
;input:		address of register in RTC
;output:	none
;Desc:		handles writing data to RTC
        ;Select the DS1307 on the bus, in WRITE mode
        i2c_common_start
        movlw       0xD0        ;DS1307 address | WRITE bit
        i2c_common_write
        i2c_common_check_ack   WR_ERR

        ;Write data to I2C bus (Register Address in RTC)

        movf        ADD,w       ;Set register pointer in RTC
        i2c_common_write
        i2c_common_check_ack   WR_ERR

        ;Write data to I2C bus (Data to be placed in RTC register)

        movf       DAT,w       ;Write data to register in RTC
        i2c_common_write
        i2c_common_check_ack   WR_ERR
        goto        WR_END
WR_ERR
        nop
WR_END  
		i2c_common_stop	;Release the I2C bus
        return
	
read_gyro
	i2c_common_start
        movlw       0xD4        
	i2c_common_write
        i2c_common_check_ack   RD_ERR
        movf        ADD,w       
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        i2c_common_repeatedstart
        movlw       0xD5       
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        i2c_common_read

        movwf       DOUT
        i2c_common_nack      ;Send acknowledgement of data reception
        
        goto        RD_END
	
read_mg
	i2c_common_start
        movlw       0x3C        
	i2c_common_write
        i2c_common_check_ack   RD_ERR
        movf        ADD,w       
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        i2c_common_repeatedstart
        movlw       0x3D       
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        i2c_common_read

        movwf       DOUT
        i2c_common_nack      ;Send acknowledgement of data reception
        
        goto        RD_END	
read_rtc
;input:		address of RTC
;output:	DOUT or DOUT
;Desc:		This reads from the selected address of the RTC
;			and saves it into DOUT or address DOUT
        ;Select the DS1307 on the bus, in WRITE mode
        i2c_common_start
        movlw       0xD0        ;DS1307 address | WRITE bit
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        ;Write data to I2C bus (Register Address in RTC)

        movf        ADD,w       ;Set register pointer in RTC
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        ;Re-Select the DS1307 on the bus, in READ mode
        i2c_common_repeatedstart
        movlw       0xD1        ;DS1307 address | READ bit
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        ;Read data from I2C bus (Contents of Register in RTC)
        i2c_common_read

        movwf       DOUT
        i2c_common_nack      ;Send acknowledgement of data reception
        
        goto        RD_END

RD_ERR 
        nop
        
        ;Release the I2C bus
RD_END  i2c_common_stop
        return

rtc_convert   
;input:		W
;output:	dig10 (dig10), dig1 (dig1)
;desc:		This subroutine converts the binary number
;			in W into a two digit ASCII number and place
;			each digit into the corresponding registers
;			dig10 or dig1

    movwf   B1             ; B1 = HHHH LLLL
    swapf   B1,w           ; W  = LLLL HHHH
    andlw   0x0f           ; Mask upper four bits 0000 HHHH
    addlw   0x30           ; convert to ASCII
    movwf   dig10		   ;saves into 10ths digit
   ;movwf    dig2
    movf    B1,w
    andlw   0x0f           ; w  = 0000 LLLL
    addlw   0x30           ; convert to ASCII		
    movwf	dig1	       ; saves into 1s digit
   ;movwf	dig3
 ;  movlw	d'0'
  ; movwf	dig12
   	return
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        ;Select the DS1307 on the bus, in WRITE mode
        i2c_common_start
		movlw       b'00010001'
        i2c_common_write
		i2c_common_check_ack   R_END

        i2c_common_read

        movwf       0x70
        i2c_common_nack      ;Send acknowledgement of data reception
R_END
		i2c_common_stop
        return
	 ;Set SDA and SCL to high-Z first as required for I2C
	 
rtc_resetAll	macro
;input:		none
;output:	none
;desc:		Resets all the time keeping registers on the RTC to zero

	clrf		DAT
	clrf		ADD
    call        write_rtc	
    incf        ADD   			;Set register address to 1
	call		write_rtc
    incf        ADD   			;Set register address to 2
	call		write_rtc
    incf        ADD   			;Set register address to 3
	call		write_rtc
    incf        ADD   			;Set register address to 4
	call		write_rtc
    incf        ADD  			;Set register address to 5
	call		write_rtc
    incf        ADD   			;Set register address to 6
	call		write_rtc
	endm

rtc_set		macro	addliteral,datliteral
;input:		addliteral: value of address
;			datliteral: value of data
;output:	none
;desc:		loads the data in datliteral into the 
;			address specified by addliteral in the RTC

	movlw	addliteral
	movwf	ADD

	movlw	datliteral
	movwf	DAT	
	call	write_rtc
	endm
gyro_read	macro   addliteral
	
	movlw	addliteral
	movwf	ADD
	call	read_gyro
	;sepdig	DOUT
	;call	WrtNum
	endm
	
mg_read		macro	addliteral
		
	movlw	addliteral
	movwf	ADD
	call	read_mg
	sepdig	DOUT
	call	WrtNum
	endm

	
rtc_read	macro	addliteral
;input:		addliteral
;output:	DOUT, dig10, dig1
;desc:		From the selected register in the RTC, read the data
;			and load it into DOUT. DOUT is also converted into 
;			ASCII characters and the tens digit is placed into
;			dig10 and the ones digit is placed in dig1
	movlw	addliteral

	movwf	ADD
	call	read_rtc

	movf	DOUT,w
	call	rtc_convert   
	endm
Gyroinit
	bsf	   TRISC,4		  
	bsf	   TRISC,3
	clrf      LATA
	clrf      LATB
	clrf      LATC 
	call	  i2c_common_setup
	;write	  Ready
	;goto	  mainwait
	return
MGinit
	bsf	   TRISC,4		  
	bsf	   TRISC,3
	clrf      LATA
	clrf      LATB
	clrf      LATC 
	call	  i2c_common_setup
	return
	
RTCinit
	
       
  ;Set SDA and SCL to high-Z first as required for I2C
		 bsf	   TRISC,4		  
		 bsf	   TRISC,3

         clrf      LATA
         clrf      LATB
         clrf      LATC 
        ; clrf      PORTD
		 ;Set up I2C for communication
		 call 	   i2c_common_setup
	;	 rtc_resetAll
		 
	 call	set_rtc_time
	; call	 show_RTC
	return	 
Init     clrf      INTCON         ; No interrupts
	 clrf	    temp
	 movlw	   d'0'
	 movwf	   renc
	 movwf	   numBar
	 movwf	   bt
	 movwf	   Full
	 movwf	   Half
	 movwf	   Empty
	 movwf	   rightE
	 movwf	   leftE
	 movwf	   currentB
	 movlw	   b'11111111'
	 movwf	   TRISA
         movlw     b'11110010'    ; Set required keypad inputs
         movwf     TRISB
	 ;call	   setints 
         clrf      TRISD
         clrf      LATA
         clrf      LATB
         clrf      LATC
         clrf      LATD
	 movlw	   b'00001111'
	 iorwf	   ADCON1
	 movlw	   b'11'
	 movwf	   cap1
	 movlw	   h'14'
	 movwf	   capinv
         call      InitLCD    ;Initialize the LCD 
	 
	; call	   MGinit
	;write	    Ready
	 call	   sdelay
	; stime	    ssec10, ssec1
	; rtc_read   0x01
	; stime	    smin10, smin1
	;goto	   US_module
	;call	   MGinit
	;goto	   mgtest
	 
	; write	    Ready
	; call	    keywait
	 ;call	    RTCinit
	; call	    rtc_resetAll
	; call	    keywait
	; call	    ClrLCD
	; call	    fgetoptime
	; goto	    US_module
	 ;goto	    PWM_module
	 ;goto	    LS_module
	 ;goto	    count_module
	 ;call	    straight 
	 ;goto	    encTest
	; call	    PWMinit
	 ;goto	    test50100
	;goto	    gyroinitt	  
	;goto	    gSteer
	;call	    arm_drive
	;write	    Ready
	 ;goto      mainloops
	; goto	    steer_module
	goto	    main
	;write	    Ready
	;write	    ftall
	mainwait    goto mainwait
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;ARM Module;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	
	arm_drive
	    ;bcf	    TRISC, 0
	   ; bsf	    LATC, 0
	;retract
	 ;   movlw   b'0110'
	 ;   movwf   TRISE
	  ;  bcf	    TRISA, 0
	  ;  call    armUS
	  ;  write   Ready
	;rest
	;bsf	    LATA, 0
	;    bcf	    LATE, 0
	    clrf    CCPR1L
	    clrf    CCPR2L
	    bsf	    LATC, 5
	armwait1
	    btfss   PORTC, 3
	    goto    armwait1
	    goto    Atestf
	 d1 bcf	    LATC, 5
	    call    setPWM
	 
	driveabit
	    call    sdelay
	    call    sdelay
	    call    sdelay
	    call    sdelay
	    call    sdelay
	   ; bcf	    LATC, 0
	    clrf    CCPR1L
	    clrf    CCPR2L
	    bsf	    LATC, 6
	    
	armwait2
	    btfss   PORTC, 7	
	    goto    armwait2
	    goto    Atestb
	d2  bcf	    LATC, 6
	   ; goto    rest
	   call	    setPWM
	return
	
	Atestf
	    btfss   PORTC, 3
	    goto    armwait1
	    btfss   PORTC, 3
	    goto    armwait1
	    btfss   PORTC, 3
	    goto    armwait1
	    goto    d1
	    
	Atestb
	    btfss   PORTC, 7
	    goto    armwait2
	    btfss   PORTC, 7
	    goto    armwait2
	    btfss   PORTC, 7
	    goto    armwait2
	    goto    d2
	    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Gyro Steer;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	    
gSteer	    call    gyroinitt
	    call    ClrLCD
	    movlw   d'160'
	    movwf   xhead
gsteerloop
	    call    ClrLCD
	    call    show_gyro
	    bcf	    gyrox, 0
	    rrncf   gyrox
	    sepdig  gyrox
	    call    WrtNum
	    clrf    WREG
	    cpfsgt  gyrox
	    goto    gsteerend
	    movlw   d'80'
	    cpfsgt  gyrox
	    goto    LeftT
RightT	    movf    gyrox, W
	    addwf   xhead, f
	   ; incf    xhead
	    goto    gsteerend
	    
LeftT	    movlw   d'128'
	    movwf   temp
	    movf    gyrox, W 
	    ;decf    xhead
	    subwf   temp, f
	    movf    temp, W
	    subwf   xhead
gsteerend
	    movlw   ' '
	    call    WR_DATA
	    movlw   'H'
	    call    WR_DATA
	    movlw   ':'
	    call    WR_DATA
	    movlw   ' '
	    call    WR_DATA
	    
	    sepdig  xhead
	    call    WrtNum
	    btfsc   PORTB, 4
	    goto    gSteer
	    call    sdelay
	    goto    gsteerloop
	    
	  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Steering Module;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
straight
	   bcf	    TRISC, 1
	   bcf	    TRISC, 2
	   bsf	    LATC, 1
	   bsf	    LATC, 2
	   return
straitwait  goto    straitwait
steer_module
	 call	    PWMinit
	 call	    mginit
	 mg_read    0x08
	 call	    show_MG
	 movff	    DOUT, sdirection
steerloop
	 movlw	    d'70'
	 movwf	    pwmR
	 movwf	    pwmL
	 call	    mginit
	 mg_read    0x08
	 movff	    DOUT, cdirection
	 call	    mginit
	 call	    show_MG
	 movf	    cdirection, W
	 cpfsgt	    sdirection
	 goto	    stleft
	 cpfslt	    sdirection
	 goto	    stright
	 goto	    steerend
	 
stleft	 
	 movff	    sdirection, temp
	 movf	    cdirection, W
	 subwf	    temp
	 movff	    temp, dirdiff
	 movf	    dirdiff, W
	 subwf	    pwmL
	 addwf	    pwmR
	 subwf	    pwmL
	 addwf	    pwmR
	 goto	    steerend
	 
stright
	 movf	    sdirection, W
	 subwf	    cdirection
	 movff	    cdirection, dirdiff
	 movf	    dirdiff, W
	 subwf	    pwmR
	 addwf	    pwmL
	 subwf	    pwmR
	 addwf	    pwmL
	 goto	    steerend
steerend
	 call	    setPWM
	 call	    sdelay
	 goto	    steerloop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Gyroscope Test;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	 
gyroinitt
	 write Ready
	call	Gyroinit
	movlw	0x20
	movwf	ADD
	movlw	b'00001010'
	movwf	DAT
	call	write_gyro
	
	movlw	0x24
	movwf	ADD
	movlw	b'00000000'
	movwf	DAT
	call	write_gyro
	return
	;movlw	0x23
	;movwf	ADD
	;movlw	b'00110000'
	;movwf	DAT
	;call	write_gyro
gyroloop
	call	sdelay
	;call	Gyroinit
	call	ClrLCD
	call	show_gyro
	call	tdelay
	call	tdelay
	call	tdelay
	call	tdelay
	call	tdelay
	call	tdelay
	call	tdelay
	call	tdelay
	goto	gyroloop
	    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Magnetormeter Test;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


	 
mginit	clrf		ADD
	 movlw		0x70
	 movwf		DAT
	 call		write_mg
	 
	 incf		ADD
	 movlw		0xA0
	 movwf		DAT
	 call		write_mg
	 
	 incf		ADD
	 clrf		DAT
	 call		write_mg
	 call		tdelay
	 return

mgtest	call	    mginit	 
mgloop
	call		show_MG
	call		tdelay
	call		tdelay
	call		tdelay
	call		tdelay
	call		tdelay
	call		tdelay
	call		tdelay
	call		tdelay
	goto		mgtest
	 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Optical Encoder Test;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;resetEnc
;	 clrf		rightE
;	 clrf		leftE	 
encTest	
	clrf		LATB
	 movlw		b'11111111'
	 movwf		TRISB
	 clrf		TRISC
	 call		setints
etloop	 call		tdelay
	 call		ClrLCD
	 write		Done5
	 sepdig		rightE
	 call		WrtNum2
	 write		Done6
	 sepdig		leftE
	 call		WrtNum2
	; btfsc		PORTB, 1
	; call		resetEnc
	 bcf		LATC, 0
	 btfsc		PORTB, 0
	 bsf		LATC, 0
	 goto		etloop
	 goto		etloop

main	
	movlw	    b'10001111'
	movwf	    TRISA
	movlw	    b'11111110'
	movwf	    TRISB
	movlw	    b'10011000'
	movwf	    TRISC
	movlw	    b'0111'
	movwf	    TRISE
	clrf	    LATA
	clrf	    LATB
	clrf	    LATC
	clrf	    LATE
	write		Ready
        call	keywait
	;call	RTCinit
	call	PWMinit
	call	ClrLCD
	 
mainloop
	 bcf	   LATA, 4
	 call	   US_module
	 goto	   mainloop
barrel_detected
	 ;bsf	   LATB, 3
	 ;btfsc	   PORTC, 7
	 ;bsf	   LATB, 4
	 bsf	   LATA, 4
	 btfss	   bt, 0
	 call	   addB
	 bsf	   bt, 0
	 movlw	   d'7'
	 cpfseq	   numBar
	 return
	 goto	    finish

keywait  
	 call	   sdelay
kloop	 btfss	   PORTB, 4
	 goto	   kloop
kloop2
	 btfsc	   PORTB, 4
	 goto	   kloop2
	 return
addB	
	 incf	   numBar
	 goto	   loopUS2
tallHandle
	 incf	   tallB
	 btfsc	   PORTA, 0
	 incf	   Full
	 btfsc	   PORTA, 1
	 incf	   Half
	 btfss	   PORTA, 1
	 incf	   Empty
	 btfsc	   PORTA, 0
	 decf	   Half
	 bsf	   bheight, 7
	 call	   find_CB
	 return
	
shortHandle
	 bcf	   bheight, 7
	 incf	   shortB
	 btfsc	   PORTA, 2
	 incf	   Full
	 btfsc	   PORTA, 3
	 incf	   Half
	 btfss	   PORTA, 3
	 incf	   Empty
	 btfsc	   PORTA, 2
	 decf	   Half
	 call	   find_CB
	 return
find_CB
	 btfss	   currentB, 0
	 goto	   sb1
	 btfss	   currentB, 1
	 goto	   sb2
	 btfss	   currentB, 2
	 goto	   sb3
	 btfss	   currentB, 3
	 goto	   sb4
	 btfss	   currentB, 4
	 goto	   sb5
	 btfss	   currentB, 5
	 goto	   sb6
	 btfss	   currentB, 6
	 goto	   sb7

	 
sb1	 bsf	   currentB, 0
	 btfsc	   bheight, 7
	 bsf	   bheight, 0
	 btfss	   bheight, 0
	 Save_BarrelL	b1L, b1Li
	 btfsc	   bheight, 0 
	 Save_BarrelH	b1L, b1Li
	 return
sb2	 bsf	   currentB, 1
	 btfsc	   bheight, 7
	 bsf	   bheight, 1
	 btfss	   bheight, 1
	 Save_BarrelL	b2L, b2Li
	 btfsc	   bheight, 1 
	 Save_BarrelH	b2L, b2Li
	 return
sb3	 bsf	   currentB, 2
	 btfsc	   bheight, 7
	 bsf	   bheight, 2
	 btfss	   bheight, 2
	 Save_BarrelL	b3L, b3Li
	 btfsc	   bheight, 2 
	 Save_BarrelH	b3L, b3Li
	 return
sb4	 bsf	   currentB, 3
	 btfsc	   bheight, 7
	 bsf	   bheight, 3
	 btfss	   bheight, 3
	 Save_BarrelL	b4L, b4Li
	 btfsc	   bheight, 3 
	 Save_BarrelH	b4L, b4Li
	 return
sb5	 bsf	   currentB, 4
	 btfsc	   bheight, 7
	 bsf	   bheight, 4
	 btfss	   bheight, 4
	 Save_BarrelL	b5L, b5Li
	 btfsc	   bheight, 4 
	 Save_BarrelH	b5L, b5Li
	 return
sb6	 bsf	   currentB, 5
	 btfsc	   bheight, 7
	 bsf	   bheight, 5
	 btfss	   bheight, 5
	 Save_BarrelL	b6L, b6Li
	 btfsc	   bheight, 5 
	 Save_BarrelH	b6L, b6Li
	 return
sb7	 bsf	   currentB, 6
	 btfsc	   bheight, 7
	 bsf	   bheight, 6
	 btfss	   bheight, 6
	 Save_BarrelL	b7L, b7Li
	 btfsc	   bheight, 6 
	 Save_BarrelH	b7L, b7Li
	 return

finish	; call	   fgetoptime
	; call	   keywait
	 bcf	   LATC, 4
	 clrf	    WREG
	 movwf	    CCPR1L
	 movwf	    CCPR2L
	 call	   ClrLCD
	 sepdig	   bheight
	 call	   WrtNum
	 call	   keywait
	 call	   ClrLCD
	 write	   Done
	 sepdig	   numBar
	 call	   WrtNum
	 write	   Done2
	 sepdig	   Full
	 call	   WrtNum
	 write	   Done3
	 sepdig	   Half
	 call	   WrtNum
	 write	   Done4
	 sepdig	   Empty
	 call	   WrtNum
	 call	   sdelay	 
	 call	   sdelay	 
	 call	   sdelay
dwait	 btfss	   PORTB, 1
	goto	   dwait
	
finish2	call	   ClrLCD
	write	   Done5
	sepdig	   rightE
	call	   WrtNum
	call	   WrtNum
	call	   sdelay	 
	 call	   sdelay	 
	 call	   sdelay
dwait2	 btfss	   PORTB, 1
	 goto	   dwait2

finish3	;movlw	    b'11111111'
	; movwf	    bheight
	 btfss	    currentB, 0
	 goto	    shend
	 call	   ClrLCD
	 write	    Barrel
	 movlw	    "1"
	 call	    WrtLCD
	 write	    Fulls
	 dFull	    b1Li
	 write	    Location
	 sepdig	    b1L	 
	 call	    WrtNum
	 call	    keywait
	 call	    ClrLCD
	 write	    Hob
	 btfsc	    bheight, 0
	 write	    ftall
	 write	    fshort
	 call	    keywait
	 
	 btfss	    currentB, 1
	 goto	    shend
	 call	   ClrLCD
	 write	    Barrel
	 movlw	    "2"	
	 call	    WrtLCD
	 write	    Fulls
	 dFull	    b2Li
	 write	    Location
	 sepdig	    b2L	 
	 call	    WrtNum
	 call	    keywait
	 call	    ClrLCD
	 write	    Hob
	 btfsc	    bheight, 1
	 write	    ftall
	 write	    fshort
	 call	    keywait
	 
	 btfss	    currentB, 2
	 goto	    shend
	 call	   ClrLCD
	 write	    Barrel
	 movlw	    "3"
	 call	    WrtLCD
	 write	    Fulls
	 dFull	    b3Li
	 write	    Location
	 sepdig	    b3L	 
	 call	    WrtNum
	 call	    keywait
	 call	    ClrLCD
	 write	    Hob
	 btfsc	    bheight, 2
	 write	    ftall
	 write	    fshort
	 call	    keywait
	 
	 btfss	    currentB, 3
	 goto	    shend
	 call	   ClrLCD
	 write	    Barrel
	 movlw	    "4"
	 call	    WrtLCD
	 write	    Fulls
	 dFull	    b4Li
	 write	    Location
	 sepdig	    b4L	 
	 call	    WrtNum
	 call	    keywait
	 call	    ClrLCD
	 write	    Hob
	 btfsc	    bheight, 3
	 write	    ftall
	 write	    fshort
	 call	    keywait
	 
	 btfss	    currentB, 4
	 goto	    shend
	 call	   ClrLCD
	 write	    Barrel
	 movlw	    "5"
	 call	    WrtLCD
	 write	    Fulls
	 dFull	    b5Li
	 write	    Location
	 sepdig	    b5L	 
	 call	    WrtNum
	 call	    keywait
	 call	    ClrLCD
	 write	    Hob
	 btfsc	    bheight, 4
	 write	    ftall
	 write	    fshort
	 call	    keywait
	 
	 btfss	    currentB, 5
	 goto	    shend
	 call	   ClrLCD
	 write	    Barrel
	 movlw	    "6"
	 call	    WrtLCD
	 write	    Fulls
	 dFull	    b6Li
	 write	    Location
	 sepdig	    b6L	 
	 call	    WrtNum
	 call	    keywait
	 call	    ClrLCD
	 write	    Hob
	 btfsc	    bheight, 5
	 write	    ftall
	 write	    fshort
	 call	    keywait
	 
	 btfss	    currentB, 6
	 goto	    shend
	 call	   ClrLCD
	 write	    Barrel
	 movlw	    "7"
	 call	    WrtLCD
	 write	    Fulls
	 dFull	    b7Li
	 write	    Location
	 sepdig	    b7L	 
	 call	    WrtNum
	 call	    keywait
	 call	    ClrLCD
	 write	    Hob
	 btfsc	    bheight, 6
	 write	    ftall
	 write	    fshort
	 call	    keywait
	 goto	    finish
shend	 call	    ClrLCD
	 write	    Nob
	 call	    keywait
	 goto	    finish
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Interrupts;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	 
setints	 clrf	   INTCON2
	 clrf	   INTCON3
	 bcf	   RCON, 7	  ;no priority interrupts*
	 bsf	   INTCON, 6
	 bsf	   INTCON, 7
	 clrf	   PIR1
	 clrf	   PIR2
	 bsf	   INTCON, 4
	 bsf	   INTCON3, 4
	 bsf	   INTCON2, 6
	 bsf	   INTCON2, 4
	 return
show_gyro
	 
		
		gyro_read    0x29
		movff	DOUT, gyrox
		;sepdig	gyrox
		;call	WrtNum
		;bcf	gyrox, 7
		;bcf	gyrox, 6
		;rlncf	gyrox
		;rlncf	gyrox
		
		;gyro_read    0x28
		;btfsc	    DOUT, 7
		;bsf	    gyrox, 1
		;btfsc	    DOUT, 6
		;bsf	    gyrox, 0
		
		;call	    ClrLCD
		;movlw	    "X"
		;call	    WR_DATA
		;movlw	    ":"
		;call	    WR_DATA
		;sepdig	    gyrox
		;call	    WrtNum
		
		return
show_MG
	
		call	ClrLCD
		movlw	"M"
		call	WR_DATA
		movlw	" "
		call	WR_DATA
		mg_read    0x07
		
		movlw	" "
		call	WR_DATA
		movlw	"L"
		call	WR_DATA
		movlw	" "
		call	WR_DATA
		
		mg_read    0x08
		
		
		
		
		
		return
show_RTC
		;clear LCD screen
		;movlw	b'00000001'
		;call	WR_INS
		call	ClrLCD
		;write	YES
		;Get year
		movlw	"2"				;First line shows 20**/**/**
		call	WrtLCD
		movlw	"0"
		call	WrtLCD
		;write	YES
		rtc_read	0x06		;Read Address 0x06 from DS1307---year
		
		
		movf	dig10, W
		call	WrtLCD
		movf	dig1, W
		call	WrtLCD

		movlw	"/"
		call	WrtLCD

		;Get month
		rtc_read	0x05		;Read Address 0x05 from DS1307---month
		movf	dig10, W
		call	WrtLCD
		movf	dig1, W
		call	WrtLCD

		movlw	"/"
		call	WrtLCD

		;Get day
		rtc_read	0x04		;Read Address 0x04 from DS1307---day
		movf	dig10, W
		call	WR_DATA
		movf	dig1, W
		call	WR_DATA

		movlw	B'11000000'		;Next line displays (hour):(min):(sec) **:**:**
		call	WR_INS

		;Get hour
		rtc_read	0x02		;Read Address 0x02 from DS1307---hour
		movf	dig10, W
		call	WR_DATA
		movf	dig1, W
		call	WR_DATA
		movlw			":"
		call	WR_DATA
fgetoptime
		;movlw	B'11000000'		;Next line displays (hour):(min):(sec) **:**:**
		;call	WR_INS
		;Get minute	
		call	ClrLCD
		write	OperTime
		movlw	b'11000000'
		call	WR_INS
		
		rtc_read	0x01		;Read Address 0x01 from DS1307---min
		movf	dig10, W
		call	WR_DATA
		movf	dig1, W
		call	WR_DATA		
		movlw	":"
		call	WR_DATA
		
		;Get seconds
		rtc_read	0x00		;Read Address 0x00 from DS1307---seconds
		movf	dig10, W
		call	WR_DATA
		movf	dig1, W
		call	WR_DATA
		
		;call	OneS			;Delay for exactly one seconds and read DS1307 again
		
		return

;***************************************
; Setup RTC with time defined by user
;***************************************
set_rtc_time

		rtc_resetAll	;reset rtc

		rtc_set	0x00,	B'10000000'

		;set time 
		rtc_set	0x06,	B'00010110'		; Year
		rtc_set	0x05,	B'00000011'		; Month
		rtc_set	0x04,	B'00000110'		; Date
		rtc_set	0x03,	B'00000010'		; Day
		rtc_set	0x02,	B'00000001'		; Hours
		rtc_set	0x01,	B'00000000'		; Minutes
		rtc_set	0x00,	B'00000000'		; Seconds
	
		return


;***************************************
; Delay 1s
;***************************************
OneS
      local	OneS_0
      movlw 0x10
      movwf COUNTH
      movlw 0x7A
      movwf COUNTM
      movlw 0x06
      movwf COUNTL

OneS_0
      decfsz COUNTH, f
      goto   l8
      decfsz COUNTM, f
 l8     goto   l9
      decfsz COUNTL, f
l9      goto   OneS_0

      goto l10
l10     nop
      nop
		return


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Counter;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
count_module
	 movlw	    b'11111111'
	 movwf	    TRISC	
	 movlw	    b'00000000'
	 movwf	    LATC
cwait	 
	 call	    sdelay
	 goto	    cwait
	 bsf	    LATC, 0
	 call	    sdelay
	 bcf	    LATC, 0
	 call	    sdelay
	 goto	    cwait
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;Light Sensor;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
LS_module
	 movlw	    b'11110000'
	 movwf	    TRISC
	 movlw	    b'00000000'
	 movwf	    LATC
LSloop	 movf	    PORTC, W
	 rlncf	    WREG, f
	 movwf	    LATC
	 goto	    LSloop
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;PWM;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PWM_module
PWMinit
	; bcf	    TRISC, 1
	; bcf	    TRISC, 2
	; bcf	    TRISC, 0
	 bcf	    LATC, 0
	 movlw	    d'99'
	 movwf	    PR2
	 ;movff	    pwm, WREG
	 movlw	    d'100'
	 movwf	    CCPR2L
	 movwf	    pwmR
	 movlw	    d'100'
	 movwf	    CCPR1L
	 movwf	    pwmL
	 
	 movlw	    b'00111100'
	 movwf	    CCP1CON
	 movwf	    CCP2CON
	 movlw	    b'00000100'
	 movwf	    T2CON
	 movlw	    d'100'
	 return
pwmwait	goto	    pwmwait
test50100
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 movlw	    d'100'
	 movff	    WREG, CCPR2L
	 movff	    WREG, CCPR1L
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 movlw	    d'30'
	 movff	    WREG, CCPR2L
	 movff	    WREG, CCPR1L
	 goto	    test50100
setPWM	
	 movff	    pwmL, CCPR1L
	 movff	    pwmR, CCPR2L
	 return
DirectionC
	 bcf	    LATC, 0
	 bcf	    LATC, 1
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 bsf	    LATC, 1
	 call	    test50100
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 bcf	    LATC, 1
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 bsf	    LATC, 0
	 call	    test50100
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 call	    sdelay
	 goto	    DirectionC
	; goto	    swait
	 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;ULTRASONIC;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

armUS
	bsf	    TRISC, 7
	bcf	    TRISC, 1
	bcf	    LATC, 1
	write	    Ready
aloopUS
	call	    tdelay
	clrf	    time
	bsf	    LATC, 1
	bcf	    USOF, 0
	movlw	    d'3'
apause
	decfsz	    WREG, f
	goto	    apause
	bcf	    LATC, 1
aw1	 btfsc	    PORTC, 7
	 goto	    aw2
	 goto	    aw1
aw2	 btfss	    PORTC, 7
	 goto	    aendUS
	 call	    check_sense
	 incf	    time
	 movlw	    d'255'
	 cpfseq	    time
	 goto	    aw2
	 bsf	    USOF, 0
	 goto	    aendUS
aendUS
	 btfsc	    USOF, 0
	 goto	    aloopUS
	 sepdig	    time
	 call	    ClrLCD
	 call	    WrtNum
	 movlw	    d'20'
	 cpfsgt	    time
	 return
	 call	    sdelay
	 goto	    aloopUS
US_module

us0c
	 movlw	    " "
	 call	    WR_DATA
	 movlw	    "0"
	 call	    WR_DATA
	 movlw	    "0"
	 call	    WR_DATA
	 movlw	    "0"
	 call	    WR_DATA
loopUS	;call	    sdelay
	 bcf	    PORTE, 1
	 bcf	    USOF, 0
	 call	   tdelay
	 movlw	    d'0'
	 movwf	    time
	 bsf	    LATA, 5
	 movlw	    d'3'
pause	 decfsz	    WREG, f
	 goto	    pause
	 bcf	    LATA, 5
w1	 btfsc	    PORTE, 1
	 goto	    w2
	 goto	    w1
w2	 btfss	    PORTE, 1
	 goto	    endUS
	 bcf	    LATC, 5	 
	 call	    check_sense
	 incf	    time
	 movlw	    d'255'
	 cpfseq	    time
	 goto	    w2
	 bsf	    USOF, 0
	 goto	    loopUS1
endUS	 btfsc	    USOF, 0	
	 goto	    loopUS
	 movlw	    d'30'
	 call	    sdelay
	 ;call	    ClrLCD
	 ;sepdig	    time
	 ;call	    WrtNum
	 cpfsgt	    time
	 
	 goto	    arm_drive
	 goto	    loopUS1
	 ;goto	    loop_US
	; return
	 ;bsf	    LATC, 5
	; call       barrel_detected
	; movlw	    d'30'
	 ;cpfslt	    time
	 ;bcf	    bt, 0
	; call	    sdelay
us1c	 call	    ClrLCD
	 movlw	    "0"
	 call	    WR_DATA
	 movlw	    "0"
	 call	    WR_DATA
	 movlw	    "0"
	 call	    WR_DATA
	
loopUS1	 bcf	    PORTE, 0
	 bcf	    USOF, 1
	 call	   tdelay
	 movlw	    d'0'
	 movwf	    time
	 bsf	    LATA, 5
	 movlw	    d'3'
pause1	 decfsz	    WREG, f
	 goto	    pause1
	 bcf	    LATA, 5
w11	 btfsc	    PORTE, 0
	 goto	    w21
	 goto	    w11
w21	 btfss	    PORTE, 0
	 goto	    endUS1
	 bcf	    LATC, 4	 
	 call	    check_sense
	 incf	    time
	 movlw	    d'255'
	 cpfseq	    time
	 goto	    w21
	 bsf	    USOF, 1
	 goto	    endUS1
endUS1	 btfsc	    USOF, 1
	 goto	    uss
	; sepdig	    time
	; call	    ClrLCD
	; call	    WrtNum
	 movlw	    d'20'
	 cpfsgt	    time
	 ;bsf	    LATC, 4
	 call       barrel_detected
uss	 movlw	    d'20'
	 cpfslt	    time
	 bcf	    bt, 0
	 return
	; call	    tdelay
us2c	
	; movlw	    " "
	; call	    WR_DATA
	 ;movlw	    "0"
	 ;call	    WR_DATA
	 ;movlw	    "0"
	 ;call	    WR_DATA
	 ;movlw	    "0"
	 ;call	    WR_DATA
loopUS2	 bcf	    PORTE, 2
	 bcf	    USOF, 2
	 call	   tdelay
	 movlw	    d'0'
	 movwf	    time
	 bsf	    LATA, 5
	 movlw	    d'3'
pause2	 decfsz	    WREG, f
	 goto	    pause2
	 bcf	    LATA, 5
w12	 btfsc	    PORTE, 2
	 goto	    w22
	 goto	    w12
w22	 btfss	    PORTE, 2
	 goto	    endUS2
	 bcf	    LATC, 7	 
	 call	    check_sense
	 incf	    time
	 movlw	    d'255'
	 cpfseq	    time
	 goto	    w22
	 bsf	    USOF, 2
	 goto	    endUS2
endUS2	 btfsc	    USOF, 2
	 goto	    shortHandle
	; sepdig	    time
	; call	    ClrLCD
	; call	    WrtNum
	 call	    sdelay
	 bcf	    LATC, 7
	 movlw	    d'20' 
	 cpfsgt	    time
	 ;bsf	    LATC, 7
	 goto	    tallHandle
	 goto	    shortHandle
	; call       barrel_detected
	 movlw	    d'20'
	 cpfslt	    time
	 
	 ;bcf	    bt, 0
	 goto	    loopUS
	
	
wait	  goto	   wait

check_sense	
	movlw	d'18'
	
	movwf	sensitivity
senseloop
	decfsz	sensitivity, f
	goto	senseloop
	return
	
enchandle
	btfsc	INTCON, 1
	call	rhandle
	btfsc	INTCON3, 1
	call	lhandle
	return
rhandle
	bcf	INTCON, 1
	incf	rightE
	incf	rightE
	incf	rightE
	btfss	boolENC, 0
	goto	encextra
	bcf	boolENC, 0
encend	
	return
encextra
	incf	rightE
	bsf	boolENC, 0
	goto	encend
	
lhandle	
	bcf	INTCON3, 1
	incf	leftE
	incf	leftE
	incf	leftE
	return
sdelay	movlw	d'20'
	movwf	lcd_sd
sloop
	call	tdelay
	decfsz	lcd_sd
	goto	sloop
	return
	 
tdelay
	movlw   d'255'
	movwf   lcd_td
 tloop	    
	call    lcdLongDelay
	decfsz  lcd_td
	goto	tloop
    return
lcdLongDelay
    movlw d'80'
    movwf lcd_d2
LLD_LOOP
    LCD_DELAY
    decfsz lcd_d2,f
    goto LLD_LOOP
    return
WrtNum2	    bcf	    STATUS,C
	    movf    dig3, W
	    addwf   dig3
	   ; carry   dig3	    
	   ; bcf	    STATUS,C
	    movf    dig2, W
	    addwfc  dig2
	   ; carry   dig2
	    movf    dig12, W
	    addwfc  dig12    
	    call    wdig1
	    call    WrtLCD
	    call    wdig2
	    call    WrtLCD
	    call    wdig3
	    call    WrtLCD
	    return
	    
WrtNum	     
	    call  wdig1
	    call    WrtLCD
	    call    wdig2
	    call    WrtLCD
	    call    wdig3
	    call    WrtLCD
	    return
	
END	 