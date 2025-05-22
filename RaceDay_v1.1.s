processor 18F45K22
    
    // <editor-fold defaultstate="collapsed" desc="Bit Configuration"> 
    CONFIG  FOSC = INTIO67
    CONFIG  WDTEN = OFF
    CONFIG  LVP	= ON
    // </editor-fold>
    
    #include	<xc.inc>
    #include	"pic18f45k22.inc"
    
   ;------- Variables --------
    // <editor-fold defaultstate="collapsed" desc="Variables">
    DELAY1  equ	0x00
    DELAY2  equ	0x01
    Count   equ	0x02
    SampleNum	equ 0x03 ; used to check how many times we have sampled
    SensorNum	equ 0x29 ; used to cycle through the individual registers	
	
   ;Summing register for averaging
    SUML    equ	0x04
    SUMH    equ	0x05
	
    ;Active color reg
    RED_reg	equ	0x06
    GREEN_reg	equ	0x07
    BLUE_reg	equ	0x30
    WHITE_reg	equ	0x31
    BLACK_reg	equ	0x32
    CALC_reg	equ	0x33
    CROSS_reg	equ	0x34
    POS_reg	equ	0x3A
    INDIRECT_reg    equ	0x3B
    COUNTER     equ 0x3C	
   ;Extra Variables
    next     equ    0x35
    follow   equ    0x36
    detected equ    0x37
    state    equ    0x38
    OFFSET_reg equ  0x39
    prev_mov	equ 0x40
    in_search	equ 0x41
	
    // </editor-fold>
    
   ; ----------- Reset Vector ------------
    PSECT code, abs
	org 0x00
	
    GOTO    Setup
    
   ;------------- Interrupt Vector --------------
    org	0x08
    BTFSC   INT0IF
    GOTO    ISR_change ;goto interrupt service routine
    BTFSC   INT1IF
    GOTO    ISR_next
    RETFIE
   
;Use PORTA for sensor input
;Use PORTC for LED strobing
;Use PORTD for SSD
;Use PORTE 
    
    // <editor-fold defaultstate="collapsed" desc="Setup"> 
Setup:
    MOVLB   0xF
    
    CLRF    ADRESH,0
    CLRF    detected,0
    
    LFSR    0, 0x200
    
    BCF    next,0
    BCF    next,2
    CLRF    state,0
    CLRF    detected,0
    CLRF    SUMH,0
    CLRF    SUML,0
    
    MOVLW   00011001B ; 6 TAD, Fosc/8 = 2 us
    MOVWF   ADCON2,0
    
    MOVLW   00000000B 
    MOVWF   ADCON1,0
    
    CLRF    PORTA,0
    CLRF    LATA,0
    CLRF    ANSELA,1
    BSF	    ANSELA,0,1 ; Set RA0 as analog
    BSF	    ANSELA,1,1 ; Set RA1 as analog
    BSF	    ANSELA,2,1 ; Set RA2 as analog
    BSF	    ANSELA,3,1 ; Set RA3 as analog
    BSF	    ANSELA,5,1 ; Set RA5 as analog
   
    
    CLRF    TRISA,0
    BSF	    TRISA,0,0 ; Set RA0 as input
    BSF	    TRISA,1,0 ; Set RA1 as input
    BSF	    TRISA,2,0 ; Set RA2 as input
    BSF	    TRISA,3,0 ; Set RA3 as input
    BSF	    TRISA,5,0 ; Set RA5 as input
    
    ; Setup PORTB   
    CLRF    PORTB,0
    CLRF    LATB,0
    
    CLRF    ANSELB,1
    CLRF    TRISB,0
    
    BSF	    TRISB,0x00,0	; make IO Pin B.0 an input
    BSF	    TRISB,0x01,0	; make IO pin B1 an input
    
    ; Setup PORTC
    CLRF    PORTC,0
    CLRF    LATC,0
    CLRF    ANSELC,1
    CLRF    TRISC,0
    
    ; Setup PORTD
    CLRF    PORTD,0
    CLRF    LATD,0
    CLRF    ANSELD,1
    CLRF    TRISD,0
    
    ; Setup  PORTE
    CLRF    PORTE,0
    CLRF    LATE,0
    CLRF    ANSELE,1
    CLRF    TRISE,0
    
    BSF	    ADON
    
    ; Set oscillator speed at 4 MHz, intruction cycle is now 1MHz = 1us per instruction
    bsf IRCF0
    bcf	IRCF1
    bsf	IRCF2
    
    ; Set up external interrupt
    CLRF    INTCON,0    ; clear all interrupt bits
			    ; 0 = access bank
    BSF	    INT0IE	    ; enable RB0 interrupt
    BSF	    INT1IE	    ; enable RB1 interrupt
    BSF	    GIE	    ; enable interrupts
    
        ; set-up timer and PWM
    ; Set RC2 and RC1 as outputs for PWM
    BCF     TRISC,2   ; Set RC2 as output (CCP1)
    BCF     TRISC,1   ; Set RC1 as output (CCP2)
    
    ; Oscillator - 4 MHz, instruction cycle is now 1MHz = 1us per instruction
    bsf     IRCF0
    bcf     IRCF1
    bsf     IRCF2
    
    ; Set up timer and PWM for 1 kHz
    MOVLW   249
    MOVWF   PR2 
    MOVLW   00000011B   
    MOVWF   T2CON
    CLRF    TMR2
    
    BSF     TMR2ON  ; Start Timer2 before setting PWM mode

    ; Configure PWM mode
    BCF     CCP1M0
    BCF     CCP1M1
    BSF     CCP1M2
    BSF     CCP1M3
    
    BCF     CCP2M0
    BCF     CCP2M1
    BSF     CCP2M2
    BSF     CCP2M3
    
    BCF     CCP3M0
    BCF     CCP3M1
    BSF     CCP3M2
    BSF     CCP3M3
    
    BCF     CCP4M0
    BCF     CCP4M1
    BSF     CCP4M2
    BSF     CCP4M3
    
    MOVLW   0x00
    MOVWF   CCPR1L
    MOVWF   CCPR2L  
    MOVWF   CCPR3L
    MOVWF   CCPR4L
    
    CLRF    follow
    
    BCF	    TRISB,5
    BCF	    ANSELB,5
    BCF	    ANSELD,1
    BCF	    TRISD,1
    
    MOVLB   0x0
    
    // </editor-fold>
; ---------- Calibration Start -------------
State0:
    ; White --- Red ____
    ; Begin Calibration, start with White, start with Red, Green, Blue
    ;; SET STATE BIT
    BSF	    next,2
    BSF	    GIE
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    CLRF    PORTD
    BSF	    PORTD,7
    BSF	    PORTD,6
    BSF	    PORTD,5
    BCF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BCF	    PORTD,0
    
    BSF	    PORTC,5 ;Set LED to red
    BCF	    PORTE,2
    BCF	    PORTE,1
    
    LFSR    0, 0x200
    MOVLW   0x00
    MOVWF   OFFSET_reg
    
    ; Clean DATA MEMORY
    MOVLW   75
    MOVWF   Count
    
    CLEAN_MEM:
    MOVLW   0x00
    MOVWF   POSTINC0
    DECFSZ  Count

    GOTO    CLEAN_MEM
    
    LFSR    0,0x200
    
    ; Press when ready to start calibrating
    BTFSS   next,0  ; When button is pressed change colours and move on
    BRA	    $-2
    
    MOVLW   95 ; Left motor
    MOVWF   CCPR1L
    MOVLW   0
    MOVWF   CCPR3L
    MOVWF   CCPR4L
    MOVLW   67 ; Right motor
    MOVWF   CCPR2L
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    ; 7 = red
    ; 6 = green
    ; 5 - blue
// <editor-fold defaultstate="collapsed" desc="WhiteSamples">
SampleLoopW_R: ; 
    ; Store a single averaged value for each register for this particular color pair 
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF    ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopW_R
    
    CALL    Averaging
    
    MOVLW   10 ; Subtract for threshold then store in relevant register
    SUBWF   SUML, 1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopW_R
    
    BCF	    PORTC,5; 
    BSF	    PORTE,2; Set LED to green
    BCF	    PORTE,1
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    CLRF    ADCON0
    
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
SampleLoopW_G: ; Sample values for Blue LED on white
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF   ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopW_G
    
    CALL    Averaging
    
    MOVLW   10 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopW_G
    
    BCF	    PORTC,5 
    BCF	    PORTE,2
    BSF	    PORTE,1; Set LED to blue
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2

SampleLoopW_B: ; Sample values for Blue LED on white
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF   ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopW_B
    
    CALL    Averaging
    
    MOVLW   10 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopW_B
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    BSF	    PORTC,5; Set LED to Red
    BCF	    PORTE,2
    BCF	    PORTE,1
    
    MOVLW   0x03
    MOVWF   Count
    Call    FlashLED
    
    BSF	    PORTD,7
    BSF	    PORTD,6
    BSF	    PORTD,5
    BCF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BSF	    PORTD,0
    
    BCF	    next,0
    BTFSS   next,0  ; When button is pressed change colours and move on
    BRA	    $-2
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="RedSample">
SampleLoopR_R:
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF    ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopR_R
    
    CALL    Averaging
    
    MOVLW   20 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    
        ; TURN OFF THE LEDS AND WAIT FOR RECONFIG
    BCF     PORTC,5
    BCF     PORTE,2
    BCF     PORTE,1

    BCF	    next,0
    BTFSS   next,0  ; When button is pressed change channel and move on
    BRA	    $-2
    
    MOVLW   0x03
    MOVWF   Count
    Call    FlashLED

    BSF     PORTC,5
    
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopR_R
    
    CLRF    SUML
    CLRF    SUMH
    
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    BCF	    PORTC,5 
    BSF	    PORTE,2; set LED to Green
    BCF	    PORTE,1
    
    BCF	    next,0
    BTFSS   next,0  ; When button is pressed change channel and move on
    BRA	    $-2
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
SampleLoopR_G:
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF    ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register

    DECFSZ  SampleNum
    GOTO    SampleLoopR_G
    
    CALL    Averaging
    
    MOVLW   5 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopR_G
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05    
    MOVWF   SensorNum
    
    BCF	    PORTC,5 
    BCF	    PORTE,2
    BSF	    PORTE,1; Set LED to blue
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
SampleLoopR_B:
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF    ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopR_B
    
    CALL    Averaging
    
    MOVLW   5 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopR_B
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    BSF	    PORTC,5; Set LED to Red 
    BCF	    PORTE,2
    BCF	    PORTE,1
    
    MOVLW   0x03
    MOVWF   Count
    Call    FlashLED
    
    BCF	    PORTD,7
    BSF	    PORTD,6
    BSF	    PORTD,5
    BSF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BCF	    PORTD,0
    
    BCF	    next,0
    BTFSS   next,0  ; When button is pressed change colours and move on
    BRA	    $-2
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="BlackSamples">
SampleLoopK_R: ; Sample black with red LED
    BSF	    GIE
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF    ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopK_R
    
    CALL    Averaging
    
    MOVLW   5 ; Add for threshold then store in relevant register
    ADDWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopK_R
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05    
    MOVWF   SensorNum
    
    BCF	    PORTC,5
    BSF	    PORTE,2; Set LED to Green
    BCF	    PORTE,1
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
SampleLoopK_G: ; Sample black with green LED
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF    ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopK_G
    
    CALL    Averaging
    
    MOVLW   5 ; Add for threshold then store in relevant register
    ADDWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopK_G
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    BCF	    PORTC,5
    BCF	    PORTE,2
    BSF	    PORTE,1; Set LED to Blue  
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2

SampleLoopK_B: ; Sample black with blue LED
    BSF	    GIE
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF   ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopK_B
    
    CALL    Averaging
    
    MOVLW   5 ; Add for threshold then store in relevant register
    ADDWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopK_B
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    BSF	    PORTC,5; Set LED to Red 
    BCF	    PORTE,2
    BCF	    PORTE,1
    
    MOVLW   0x03
    MOVWF   Count
    Call    FlashLED
    
    BSF	    PORTD,7
    BSF	    PORTD,6
    BSF	    PORTD,5
    BSF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BSF	    PORTD,0
    
    BCF	    next,0
    BTFSS   next,0  ; When button is pressed change colours and move on
    BRA	    $-2
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    // </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="BlueSamples">
SampleLoopB_R: ; Sample black with blue LED
    BSF	    GIE
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF    ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopB_R
    
    CALL    Averaging
    
    MOVLW   5 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopB_R
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    BCF	    PORTC,5
    BSF	    PORTE,2; Set LED to Green
    BCF	    PORTE,1    
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
SampleLoopB_G: ; Sample black with blue LED
    BSF	    GIE
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF    ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopB_G
    
    CALL    Averaging
    
    MOVLW   5 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopB_G
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    BCF	    PORTC,5; Set LED to Red 
    BCF	    PORTE,2
    BSF	    PORTE,1
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
SampleLoopB_B: ; Sample black with blue LED
    BSF	    GIE
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF   ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopB_B
    
    CALL    Averaging
    
    MOVLW   5 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopB_B
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    BSF	    PORTC,5; Set LED to Red 
    BCF	    PORTE,2
    BCF	    PORTE,1
    
    MOVLW   0x03
    MOVWF   Count
    Call    FlashLED
    
    BSF	    PORTD,7
    BCF	    PORTD,6
    BSF	    PORTD,5
    BSF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BSF	    PORTD,0
    
    BCF	    next,0
    BTFSS   next,0  ; When button is pressed change colours and move on
    BRA	    $-2
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    // </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="GreenSamples">
SampleLoopG_R: ; Sample black with blue LED
    BSF	    GIE
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF   ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopG_R
    
    CALL    Averaging
    
    MOVLW   5 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
        
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopG_R
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    BCF	    PORTC,5
    BSF	    PORTE,2; Set LED to Green
    BCF	    PORTE,1
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
SampleLoopG_G: ; Sample black with blue LED
    BSF	    GIE
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF   ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopG_G
    
    CALL    Averaging
    
    MOVLW   5	 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopG_G
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    BCF	    PORTC,5
    BCF	    PORTE,2
    BSF	    PORTE,1; Set LED to Blue
    
    MOVLW   16
    MOVWF   SampleNum
    MOVLW   0x05
    MOVWF   SensorNum
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
SampleLoopG_B: ; Sample black with blue LED
    BSF	    GIE
    
    CALL    Delay_loop
    BSF	    GO
    BTFSC   GO
    BRA	    $-2
    
    MOVF   ADRESH, W
    ADDWF   SUML,1 ; Store resukt from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register
    
    DECFSZ  SampleNum
    GOTO    SampleLoopG_B
    
    CALL    Averaging
    
    MOVLW   5 ; Subtract for threshold then store in relevant register
    SUBWF   SUML,1
    MOVF    OFFSET_reg, 0
    MOVFF   SUML, PLUSW0
    
    CLRF    SUML
    CLRF    SUMH
    CLRF    ADRESH
    
    MOVLW   16
    MOVWF   SampleNum
    
    INCF    OFFSET_reg
    CALL    ChangeADChannel
    DECFSZ  SensorNum
    GOTO    SampleLoopG_B
    
    CLRF    SUML
    CLRF    SUMH
    MOVLW   00000001B
    MOVWF   ADCON0
    
    BSF	    PORTC,5; Set LED to Red 
    BCF	    PORTE,2
    BCF	    PORTE,1
    
    MOVLW   0x03
    MOVWF   Count
    Call    FlashLED
    
    // </editor-fold>
    
    MOVLW   0 ; Left motor
    MOVWF   CCPR1L
    MOVLW   0
    MOVWF   CCPR3L
    MOVWF   CCPR4L
    MOVLW   0 ; Right motor
    MOVWF   CCPR2L
    BCF	    next,0

; -------------- WRITING TO FLASH MEMORY-------------;
EraseBlock:

    ; Load TBLPTR
    CLRF    TBLPTRU,0
    MOVLW   0x40
    MOVWF   TBLPTRH,0
    CLRF    TBLPTRL,0

    ; Set up for write to flash memory
    BSF	    EEPGD   ; point to Flash program memory
    BCF	    CFGS    ; access Flash program memory
    BSF	    WREN    ; enable write to memory
    BSF	    FREE    ; enable Erase operation
    BCF	    GIE	    ; disable interrupts

    ;Required write sequence to EECON2 which is not a physically implemented
    ;register
    MOVLW   55h
    MOVWF   EECON2,0	    ; write 55h
    MOVLW   0AAh
    MOVWF   EECON2,0	    ; write 0AAh

    ; Erase sequence
    BSF	    EECON1, 1,0	; Bit WR, start erase (CPU stall)
    BSF	    GIE		; Re-enable interrupts
    TBLRD*-		; dummy read decrement

WriteToHoldingRegisters:	
    ; We are writing all the data from the calibration in 0x200...for 75 bytes to the holding register
    MOVLW   64
    MOVWF   Count
    LFSR    0,0x200

    COPY_TO_HOLDING:
        MOVFF   POSTINC0, TABLAT
        TBLWT+*
        DECFSZ  Count
        BRA     COPY_TO_HOLDING
	
ProgramToMem:
    BSF EEPGD   ; point to Flash program memory (EECON1)
    BCF CFGS    ; access Flash program memory (EECON1)
    BSF WREN    ; enable write to memory (EECON1)
    BCF GIE	; disable interrupts (INTCON)

    MOVLW	55h
    MOVWF	EECON2,0 ; write 55h
    MOVLW	0AAh
    MOVWF	EECON2,0 ; write 0AAh

    BSF	EECON1, 1,0 ; bit WR, start program (CPU stall)

    BSF GIE	    ; re-enable interrupts (INTCON)
    BCF EECON1, 2,0 ; bit WREN, disable write to memory

EraseBlock_Second:

    ; Load TBLPTR
    CLRF    TBLPTRU,0
    MOVLW   0x40
    MOVWF   TBLPTRH,0
    MOVWF   TBLPTRL,0

    ; Set up for write to flash memory
    BSF	    EEPGD   ; point to Flash program memory
    BCF	    CFGS    ; access Flash program memory
    BSF	    WREN    ; enable write to memory
    BSF	    FREE    ; enable Erase operation
    BCF	    GIE	    ; disable interrupts

    ;Required write sequence to EECON2 which is not a physically implemented
    ;register
    MOVLW   55h
    MOVWF   EECON2,0	    ; write 55h
    MOVLW   0AAh
    MOVWF   EECON2,0	    ; write 0AAh

    ; Erase sequence
    BSF	    EECON1, 1,0	; Bit WR, start erase (CPU stall)
    BSF	    GIE		; Re-enable interrupts
    TBLRD*-		; dummy read decrement

WriteToHoldingRegisters_2:	
    ; We are writing all the data from the calibration in 0x200...for 75 bytes to the holding register
    MOVLW   11
    MOVWF   Count

    COPY_TO_HOLDING_2:
        MOVFF   POSTINC0, TABLAT
        TBLWT+*
        DECFSZ  Count
        BRA     COPY_TO_HOLDING_2
	
    MOVLW	53 ; number of bytes left in block (64-5)
    MOVWF	COUNTER,1
    CLRF	TABLAT,0

LoadRestOfHoldingRegs_2:
    TBLWT+*
    DECFSZ	COUNTER,1,0
    BRA		LoadRestOfHoldingRegs_2

ProgramToMem_2:
    BSF EEPGD   ; point to Flash program memory (EECON1)
    BCF CFGS    ; access Flash program memory (EECON1)
    BSF WREN    ; enable write to memory (EECON1)
    BCF GIE	; disable interrupts (INTCON)

    MOVLW	55h
    MOVWF	EECON2,0 ; write 55h
    MOVLW	0AAh
    MOVWF	EECON2,0 ; write 0AAh

    BSF	EECON1, 1,0 ; bit WR, start program (CPU stall)

    BSF GIE	    ; re-enable interrupts (INTCON)
    BCF EECON1, 2,0 ; bit WREN, disable write to memory
    
    CALL    Delay_loop
    
    GOTO    CALIBRATION_RETRIEVAL

;;--------------- Colour Detection --------------------
;// <editor-fold defaultstate="collapsed" desc="ColourDetectRed">
;ColourDetection:
;    MOVLW   16
;    MOVWF   SampleNum
;    MOVLW   0x0
;    MOVWF   POS_reg
;    
;    BCF	    detected,0
;    
;    CLRF    SUML
;    CLRF    SUMH
;    CLRF    CALC_reg
;    
;    BTFSC   next,0  
;    GOTO    LLI_SETUP
;    
;    
;CD_Sample:
;    CALL    TEST_WHITE
;    BTFSC   CALC_reg, 0
;    GOTO    OnWhite
;        
;    CALL    TEST_RED
;    BTFSC   CALC_reg, 1
;    GOTO    OnRed
;    
;    CALL    TEST_GREEN
;    BTFSC   CALC_reg, 2
;    GOTO    OnGreen
;    
;    CALL    TEST_BLUE
;    BTFSC   CALC_reg, 3
;    GOTO    OnBlue
;    
;    CALL    TEST_BLACK
;    BTFSC   CALC_reg, 4
;    GOTO    OnBlack
;    
;    GOTO    ColourDetection
;    
;    // </editor-fold>
;    
// <editor-fold defaultstate="collapsed" desc="BranchesForColourDetectSSD">
OnWhite:
    BSF	    PORTD,7
    BSF	    PORTD,6
    BSF	    PORTD,5
    BCF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BCF	    PORTD,0
    
    RETURN
OnRed:
    BSF	    PORTD,7
    BSF	    PORTD,6
    BSF	    PORTD,5
    BCF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BSF	    PORTD,0
   
    RETURN
OnGreen:
    BSF	    PORTD,7
    BCF	    PORTD,6
    BSF	    PORTD,5
    BSF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BSF	    PORTD,0
    
    RETURN
OnBlue:
    BSF	    PORTD,7
    BSF	    PORTD,6
    BSF	    PORTD,5
    BSF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BSF	    PORTD,0
    
    RETURN
OnBlack:
    BCF	    PORTD,7
    BSF	    PORTD,6
    BSF	    PORTD,5
    BSF	    PORTD,4
    BSF	    PORTD,3
    BSF	    PORTD,2
    BCF	    PORTD,0
    
    RETURN
    
// </editor-fold>
    
CALIBRATION_RETRIEVAL:
    MOVLW   75
    MOVWF   Count
    LFSR    0,0x200
    
    CLEAN_MEM_2:
    MOVLW   0x00
    MOVWF   POSTINC0
    DECFSZ  Count
    GOTO    CLEAN_MEM_2
    
    LFSR    0,0x200
    
    ; Move the flash memory to data memory

MOVLW	75 ; 3 numbers in first line + 3 numbers in second line
MOVWF	Count,0

; The table address consists of 3 bytes = 24 bits to address the 21 bit program
; memory address space. We define the table to start at 100h in program
; memory, so we need to set the table address to that location.
				; Table starts at 0x100
CLRF 	TBLPTRU,A 	; Upper byte of table address 00
MOVLW 	0x40
MOVWF 	TBLPTRH,A 	; High byte of table address 01
MOVLW 	0x00
MOVWF 	TBLPTRL,A	; Low byte of table address 00

LFSR    0,0x200

Loop_TBL_RETRIEVE:
    TBLRD*+	    
    MOVFF	TABLAT, POSTINC0	; Move content of TABLAT to W registe
    DECFSZ	Count,1,0
    GOTO 	Loop_TBL_RETRIEVE
    
GOTO	LLI_SETUP
; ---------- LLI ------------ 
// <editor-fold defaultstate="collapsed" desc="LLI">
LLI_SETUP:
    
MOVLW   0x03
MOVWF   Count
CALL	FlashLED   
BCF	next, 0
MOVLW	0x01
MOVWF	follow
    

BSF	GIE
CLRF	prev_mov
GOTO	CAP_TOUCH
    
LLI:
; Clear the color registers
CLRF    GREEN_reg
CLRF    RED_reg
CLRF    WHITE_reg
CLRF    BLACK_reg
CLRF    BLUE_reg
CLRF    CALC_reg
CLRF	CROSS_reg
CLRF	in_search

; The follow register will store the color that we are interested in following
FOLLOW_BLUE:
    ; We are going to cycle between 1-4 colours;
    ; 1 - BLUE
    ; 2 - RED
    ; 3 - GREED
    
    ; SET THE SSD, SET THE TARGET AND CROSS COLOUR REGS ASWELL
    
    ; FOLLOW 1 = BLUE
    MOVLW   0x01
    CPFSEQ  follow
    GOTO    FOLLOW_RED
    CLRF    POS_reg
    CLRF    INDIRECT_reg
    
MM_BLUE:
    ; This is the middle sensor for the red color
    CLRF    CALC_reg
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   0x00
    MOVWF   POS_reg
    
    ; We want to strobe the led and compare to determine if it is white
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 2
    BTFSC   CALC_reg, 0
    GOTO    LM_BLUE
    
    MOVLW   0x00
    MOVWF   POS_reg
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF     CROSS_reg, 2
    BTFSC   CALC_reg, 1
    GOTO    LM_BLUE
    
    MOVLW   0x00
    MOVWF   POS_reg
    
    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 2
    BTFSC   CALC_reg, 2
    GOTO    LM_BLUE
    
    MOVLW   0x00
    MOVWF   POS_reg
    
    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    GOTO    STRAIGHT
    
    ; Test for the other colours
    BSF     CROSS_reg, 2
    BSF	    BLACK_reg, 2    

LM_BLUE:
        ; We want to strobe the led and compare to determine if it is white
    CLRF    CALC_reg
    MOVLW   00000101B
    MOVWF   ADCON0
    
    MOVLW   0x01
    MOVWF   POS_reg
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 3
    BTFSC   CALC_reg, 0
    GOTO    RM_BLUE
    
    ; Test the sensor for a color and store the result in the register
    MOVLW   0x01
    MOVWF   POS_reg
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF	    CROSS_reg, 3
    BTFSC   CALC_reg, 1
    GOTO    RM_BLUE
    
    ; Test for the other colours
    MOVLW   0x01
    MOVWF   POS_reg
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 3
    BTFSC   CALC_reg, 2
    GOTO    RM_BLUE
    
    ; Test for the other colours
    MOVLW   0x01
    MOVWF   POS_reg
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    GOTO    MID_LEFT
    
        ; Test for the other colours
    BSF     CROSS_reg, 3
    BSF	    BLACK_reg, 3  

RM_BLUE:
            ; We want to strobe the led and compare to determine if it is white
    CLRF    CALC_reg
    MOVLW   00001001B
    MOVWF   ADCON0
    
    MOVLW   0x02
    MOVWF   POS_reg
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 1
    BTFSC   CALC_reg, 0
    GOTO    LL_BLUE
    
    ; Test the sensor for a color and store the result in the register
    MOVLW   0x02
    MOVWF   POS_reg
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF	    CROSS_reg, 1
    BTFSC   CALC_reg, 1
    GOTO    LL_BLUE
    
    ; Test for the other colours
    MOVLW   0x02
    MOVWF   POS_reg
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 1
    BTFSC   CALC_reg, 2
    GOTO    LL_BLUE
    
    ; Test for the other colours
    MOVLW   0x02
    MOVWF   POS_reg
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    GOTO    MID_RIGHT
    
    ; Test for the other colours
    BSF     CROSS_reg, 1
    BSF	    BLACK_reg, 1 
    
LL_BLUE:
    ;We want to strobe the led and compare to determine if it is white
    CLRF    CALC_reg
    MOVLW   00001101B
    MOVWF   ADCON0
    
    MOVLW   0x03
    MOVWF   POS_reg
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 4
    BTFSC   CALC_reg, 0
    GOTO    RR_BLUE
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF     CROSS_reg, 4
    BTFSC   CALC_reg, 1
    GOTO    RR_BLUE
    
    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 4
    BTFSC   CALC_reg, 2
    GOTO    RR_BLUE
    
    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    GOTO    OUT_LEFT
    
    ; Test for the other colours
    BSF     CROSS_reg, 4
    BSF	    BLACK_reg, 4

RR_BLUE:
        ;We want to strobe the led and compare to determine if it is white
    CLRF    CALC_reg
    MOVLW   00010001B
    MOVWF   ADCON0
    
    MOVLW   0x04
    MOVWF   POS_reg
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 0
    
    BTFSC   CALC_reg, 0
    GOTO    RESOLUTION_BLUE
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF	    CROSS_reg, 0
    BTFSC   CALC_reg, 1
    GOTO    RESOLUTION_BLUE
    
    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 0
    BTFSC   CALC_reg, 2
    GOTO    RESOLUTION_BLUE
    
    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    GOTO    OUT_RIGHT
    
        ; Test for the other colours
    BSF     CROSS_reg, 0
    BSF	    BLACK_reg, 0
    
RESOLUTION_BLUE:
    ; At this point we know that we have not seen the target color - RED
    
    ; Check if all black to stop
    MOVLW   31
    SUBWF   BLACK_reg, 0
    BTFSC   STATUS,0
    GOTO    STOP_F
 
    ; Check if for all white
    MOVLW   31
    SUBWF   WHITE_reg, 0
    BTFSC   STATUS,0
    GOTO    SEARCH
    
    ; Check each sensor sequentially for a color to follow because we know a sensor is seeing a non white color that isn't blue
    BTFSC   CROSS_reg, 2
    GOTO    STRAIGHT
    BTFSC   CROSS_reg, 1
    GOTO    MID_RIGHT
    BTFSC   CROSS_reg, 3
    GOTO    MID_LEFT
    BTFSC   CROSS_reg, 4
    GOTO    OUT_LEFT
    BTFSC   CROSS_reg, 0
    GOTO    OUT_RIGHT
    
    GOTO    SEARCH ; FALLBACK IF WE CAN'T REACH THE OTHER STATES
     
; The follow register will store the color that we are interested in following
FOLLOW_RED:
    ; We are going to cycle between 1-4 colours;
    ; 1 - BLUE
    ; 2 - RED
    ; 3 - GREEN
    
    ; SET THE SSD, SET THE TARGET AND CROSS COLOUR REGS ASWELL
    
    ; FOLLOW 3 = RED
    MOVLW   0x02
    CPFSEQ  follow
    GOTO    FOLLOW_GREEN
    
    CLRF    POS_reg
    CLRF    INDIRECT_reg
    
MM_RED:
    ; This is the middle sensor for the red color
    CLRF    CALC_reg
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   0x00
    MOVWF   POS_reg
    
    ; We want to strobe the led and compare to determine if it is white
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 2
    BTFSC   CALC_reg, 0
    GOTO    LM_RED
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    GOTO    STRAIGHT
    
    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 2
    BTFSC   CALC_reg, 2
    GOTO    LM_RED

    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 2
    BTFSC   CALC_reg, 3
    GOTO    LM_RED
    
    ; Test for the other colours
    BSF     CROSS_reg, 2
    BSF	    BLACK_reg, 2    

LM_RED:
        ; We want to strobe the led and compare to determine if it is white
    CLRF    CALC_reg
    MOVLW   00000101B
    MOVWF   ADCON0
    
    MOVLW   0x01
    MOVWF   POS_reg
    
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 3
    BTFSC   CALC_reg, 0
    GOTO    RM_RED
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    GOTO    MID_LEFT 
    
    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 3
    BTFSC   CALC_reg, 2
    GOTO    RM_RED
    
    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 3
    BTFSC   CALC_reg, 3
    GOTO    RM_RED

    ; Test for the other colours
    BSF     CROSS_reg, 3
    BSF	    BLACK_reg, 3  

RM_RED:
            ; We want to strobe the led and compare to determine if it is white
    CLRF    CALC_reg
    MOVLW   00001001B
    MOVWF   ADCON0
    
    MOVLW   0x02
    MOVWF   POS_reg
    
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 1
    BTFSC   CALC_reg, 0
    GOTO    LL_RED
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    GOTO    MID_RIGHT
    
    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 1
    BTFSC   CALC_reg, 2
    GOTO    LL_RED

    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 1
    BTFSC   CALC_reg, 3
    GOTO    LL_RED
    
        ; Test for the other colours
    BSF	    CROSS_reg, 1
    BSF	    BLACK_reg, 1 
    
LL_RED:
    ;We want to strobe the led and compare to determine if it is white
    CLRF    CALC_reg
    MOVLW   00001101B
    MOVWF   ADCON0
    
    MOVLW   0x03
    MOVWF   POS_reg
    
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 4
    BTFSC   CALC_reg, 0
    GOTO    RR_RED
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    GOTO    OUT_LEFT 
    
    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 4
    BTFSC   CALC_reg, 2
    GOTO    RR_RED

    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 4
    BTFSC   CALC_reg, 3
    GOTO    RR_RED

    ; Test for the other colours
    BSF	    CROSS_reg, 4
    BSF	    BLACK_reg, 4

RR_RED:
    ;We want to strobe the led and compare to determine if it is white
    CLRF    CALC_reg
    MOVLW   00010001B
    MOVWF   ADCON0
    
    MOVLW   0x04
    MOVWF   POS_reg
    
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 0
    BTFSC   CALC_reg, 0
    GOTO    RESOLUTION_RED 
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    GOTO    OUT_RIGHT
    
    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    BSF	    CROSS_reg, 0
    BTFSC   CALC_reg, 2
    GOTO    RESOLUTION_RED

    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 0
    BTFSC   CALC_reg, 3
    GOTO    RESOLUTION_RED
    
        ; Test for the other colours
    BSF	    CROSS_reg, 0
    BSF	    BLACK_reg, 0
    
RESOLUTION_RED:
    ; At this point we know that we have not seen the target color - RED
    
    ; Check if all black to stop
    MOVLW   31
    SUBWF   BLACK_reg, 0
    BTFSC   STATUS,0
    GOTO    STOP_F
 
    ; Check if for all white
    MOVLW   31
    SUBWF   WHITE_reg, 0
    BTFSC   STATUS,0
    GOTO    SEARCH
    
    ; Check each sensor sequentially for a color to follow because we know a sensor is seeing a non white color that isn't red
    BTFSC   CROSS_reg, 2
    GOTO    STRAIGHT
    BTFSC   CROSS_reg, 1
    GOTO    MID_RIGHT
    BTFSC   CROSS_reg, 3
    GOTO    MID_LEFT
    BTFSC   CROSS_reg, 4
    GOTO    OUT_LEFT
    BTFSC   CROSS_reg, 0
    GOTO    OUT_RIGHT
    
    GOTO    SEARCH ; FALLBACK IF WE CAN'T REACH THE OTHER STATES
    
; The follow register will store the color that we are interested in following
FOLLOW_GREEN:
    ; We are going to cycle between 1-4 colours;
    ; 1 - BLUE
    ; 2 - RED
    ; 3 - GREEN
    
    ; SET THE SSD, SET THE TARGET AND CROSS COLOUR REGS ASWELL
    
    ; FOLLOW 1 = BLUE
    MOVLW   0x03
    CPFSEQ  follow
    GOTO    CAP_TOUCH
    
    CLRF    POS_reg
    CLRF    INDIRECT_reg   
    
MM_GREEN:
    ; This is the middle sensor for the red color
    CLRF    CALC_reg
    MOVLW   00000001B
    MOVWF   ADCON0
    
    MOVLW   0x00
    MOVWF   POS_reg
    
    ; We want to strobe the led and compare to determine if it is white
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 2
    BTFSC   CALC_reg, 0
    GOTO    LM_GREEN
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF	    CROSS_reg, 2
    BTFSC   CALC_reg, 1
    GOTO    LM_GREEN

    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    GOTO    STRAIGHT

    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 2
    BTFSC   CALC_reg, 3
    GOTO    LM_GREEN

    ; Test for the other colours
    BSF	    CROSS_reg, 2
    BSF	    BLACK_reg, 2    

LM_GREEN:
        ; We want to strobe the led and compare to determine if it is white
    ; This is the middle sensor for the red color
    CLRF    CALC_reg
    MOVLW   00000101B
    MOVWF   ADCON0
    
    MOVLW   0x01
    MOVWF   POS_reg
    
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 3
    BTFSC   CALC_reg, 0
    GOTO    RM_GREEN
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF	    CROSS_reg, 3
    BTFSC   CALC_reg, 1
    GOTO    RM_GREEN

    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    GOTO    MID_LEFT
    
    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 3
    BTFSC   CALC_reg, 3
    GOTO    RM_GREEN

        ; Test for the other colours
    BSF	    CROSS_reg, 3
    BSF	    BLACK_reg, 3  

RM_GREEN:
            ; We want to strobe the led and compare to determine if it is white
        ; This is the middle sensor for the red color
    CLRF    CALC_reg
    MOVLW   00001001B
    MOVWF   ADCON0
    
    MOVLW   0x02
    MOVWF   POS_reg
    
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 1
    BTFSC   CALC_reg, 0
    GOTO    LL_GREEN
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF	    CROSS_reg, 1
    BTFSC   CALC_reg, 1
    GOTO    LL_GREEN
    
    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    GOTO    MID_RIGHT
    
    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 1
    BTFSC   CALC_reg, 3
    GOTO    LL_GREEN
    
        ; Test for the other colours
    BSF	    CROSS_reg, 1
    BSF	    BLACK_reg, 1 
    
LL_GREEN:
    ;We want to strobe the led and compare to determine if it is white
        ; This is the middle sensor for the red color
    CLRF    CALC_reg
    MOVLW   00001101B
    MOVWF   ADCON0
    
    MOVLW   0x03
    MOVWF   POS_reg
    
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 4
    BTFSC   CALC_reg, 0
    GOTO    RR_GREEN
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF	    CROSS_reg, 4
    BTFSC   CALC_reg, 1
    GOTO    RR_GREEN

    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    GOTO    OUT_LEFT
    
    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 4
    BTFSC   CALC_reg, 3
    GOTO    RR_GREEN

    ; Test for the other colours
    BSF	    CROSS_reg, 4
    BSF	    BLACK_reg, 4

RR_GREEN:
    ;We want to strobe the led and compare to determine if it is white
        ; This is the middle sensor for the red color
    CLRF    CALC_reg
    MOVLW   00010001B
    MOVWF   ADCON0
    
    MOVLW   0x04
    MOVWF   POS_reg
    
    CALL    TEST_WHITE
    
    ; Check the result in the calc register - IF IT IS WHITE THEN WE MUST SET THE WHITE REGISTER AND MOVE ON TO THE NEXT SENSOR
    BTFSC   CALC_reg, 0
    BSF	    WHITE_reg, 0
    BTFSC   CALC_reg, 0
    GOTO    RESOLUTION_GREEN 
    
    ; Test the sensor for a color and store the result in the register
    CALL    TEST_RED
    BTFSC   CALC_reg, 1
    BSF	    CROSS_reg, 0
    BTFSC   CALC_reg, 1
    GOTO    RESOLUTION_GREEN

    ; Test for the other colours
    CALL    TEST_GREEN
    BTFSC   CALC_reg, 2
    GOTO    OUT_RIGHT
    
    ; Test for the other colours
    CALL    TEST_BLUE
    BTFSC   CALC_reg, 3
    BSF	    CROSS_reg, 0
    BTFSC   CALC_reg, 3
    GOTO    RESOLUTION_GREEN
        ; Test for the other colours
    BSF	    CROSS_reg, 0
    BSF	    BLACK_reg, 0
    
RESOLUTION_GREEN:
    ; At this point we know that we have not seen the target color - RED
    
    ; Check if all black to stop
    MOVLW   31
    SUBWF   BLACK_reg, 0
    BTFSC   STATUS,0
    GOTO    STOP_F
 
    ; Check if for all white
    MOVLW   31
    SUBWF   WHITE_reg, 0
    BTFSC   STATUS,0
    GOTO    SEARCH
    
    ; Check each sensor sequentially for a color to follow because we know a sensor is seeing a non white color that isn't red
    BTFSC   CROSS_reg, 2
    GOTO    STRAIGHT
    BTFSC   CROSS_reg, 1
    GOTO    MID_RIGHT
    BTFSC   CROSS_reg, 3
    GOTO    MID_LEFT
    BTFSC   CROSS_reg, 4
    GOTO    OUT_LEFT
    BTFSC   CROSS_reg, 0
    GOTO    OUT_RIGHT
    
    GOTO    SEARCH ; FALLBACK IF WE CAN'T REACH THE OTHER STATES
    
SEARCH:
    
    CLRF    PORTD
    BSF	    PORTD,0
    
    BSF	    in_search,0
    
    BTFSC   prev_mov,0 ; STRAIGHT
    GOTO    STRAIGHT
    BTFSC   prev_mov,1 ; MIDLEFT
    GOTO    MID_LEFT
    BTFSC   prev_mov,2 ; OUTLEFT
    GOTO    OUT_RIGHT
    BTFSC   prev_mov,3 ; MIDRIGHT
    GOTO    MID_RIGHT
    BTFSC   prev_mov,4 ; OUTRIGHT
    GOTO    OUT_LEFT
    
    GOTO    LLI

STOP_F:
    CLRF    PORTD
    BSF	    PORTD,7
    
    MOVLB   0xF
    
    MOVLW   0
    MOVWF   CCPR1L
    MOVWF   CCPR2L
    MOVWF   CCPR3L
    MOVWF   CCPR4L
    
    MOVLB   0x0
    
    GOTO    CAP_TOUCH

OUT_LEFT:
    BTFSC   prev_mov, 2
    GOTO    HANDLE_OL
    
    CLRF    prev_mov
    BSF	    prev_mov,2
    CLRF    PORTD
    BSF	    PORTD,6
    BSF	    PORTD,5
    
    MOVLB   0xF
    
    MOVLW   60
    MOVWF   CCPR2L
    MOVLW   0
    MOVWF   CCPR3L
    MOVWF   CCPR1L
    MOVLW   80
    MOVWF   CCPR4L
    
    MOVLB   0x0
    
    HANDLE_OL:
    
    BTFSS   in_search,0
    GOTO    LLI
    
    MOVLW   5
    MOVWF   Count
    
    CALL    Delay_loop
    DECFSZ  Count
    BRA	    $-4
    
    GOTO    LLI

STRAIGHT:
    
    BTFSC   prev_mov, 0
    GOTO    HANDLE_STR
    
    CLRF    prev_mov
    BSF	    prev_mov,0
    CLRF    PORTD
    BSF	    PORTD,4
    
    MOVLB   0xF
    
    MOVLW   95 ; Left motor
    MOVWF   CCPR1L
    MOVLW   0
    MOVWF   CCPR3L
    MOVWF   CCPR4L
    MOVLW   67 ; Right motor
    MOVWF   CCPR2L
    
    MOVLB   0x0
    
    HANDLE_STR:
    
    BTFSS   in_search,0
    GOTO    LLI
    
    MOVLW   255
    MOVWF   Count
    
    CALL    Delay_loop
    DECFSZ  Count
    BRA	    $-4
    
    GOTO    LLI

    ; 1 = Left forward
    ; 2 = Right forward
    ; 3 = right reverse
    ; 4 = left reverse
MID_LEFT:
    BTFSC   prev_mov, 1
    GOTO    HANDLE_ML
    
    CLRF    prev_mov
    BSF	    prev_mov,1
    CLRF    PORTD
    BSF	    PORTD,5
    
    MOVLB   0xF
    
    MOVLW   60 
    MOVWF   CCPR2L
    MOVLW   0
    MOVWF   CCPR3L
    MOVWF   CCPR1L
    MOVLW   0
    MOVWF   CCPR4L
    
    MOVLB   0x0
    
    HANDLE_ML:
    BTFSS   in_search,0
    GOTO    LLI
    
    MOVLW   255
    MOVWF   Count
    
    CALL    Delay_loop
    DECFSZ  Count
    BRA	    $-4
    
    GOTO    LLI

MID_RIGHT:
    BTFSC   prev_mov,3
    GOTO    HANDLE_MR
    
    CLRF    prev_mov
    BSF	    prev_mov,3
    CLRF    PORTD
    BSF	    PORTD,3
    
    MOVLB   0xF
    
    MOVLW   80
    MOVWF   CCPR1L
    MOVLW   0
    MOVWF   CCPR2L
    MOVWF   CCPR4L
    MOVLW   0
    MOVWF   CCPR3L
    
    MOVLB   0x0
    
    HANDLE_MR:
    BTFSS   in_search,0
    GOTO    LLI
    
    MOVLW   255
    MOVWF   Count
    
    CALL    Delay_loop
    DECFSZ  Count
    BRA	    $-4
    
    GOTO    LLI

OUT_RIGHT:
    BTFSC   prev_mov,4
    GOTO    HANDLE_OR
    
    CLRF    prev_mov
    BSF	    prev_mov,4
    CLRF    PORTD
    BSF	    PORTD,3
    BSF	    PORTD,2
    
    MOVLB   0xF
    
    MOVLW   80
    MOVWF   CCPR1L
    MOVLW   0
    MOVWF   CCPR4L
    MOVWF   CCPR2L
    MOVLW   60
    MOVWF   CCPR3L
    
    MOVLB   0x0
    
    HANDLE_OR:
    
    BTFSS   in_search,0
    GOTO    LLI
    
    MOVLW   255
    MOVWF   Count
    
    CALL    Delay_loop
    DECFSZ  Count
    BRA	    $-4
    
    GOTO    LLI
    
    
; ---------- CAPTOUCH -------
CAP_TOUCH:
    ;; CLEAR STATE BIT AND SHOW COLOUR WE WILL RACE ON
    
    BCF	    next,2
    BCF	    next,0
    BSF	    GIE
    
Shine_blue:
	MOVLW   1
	CPFSEQ  follow
	GOTO    Shine_red
    
	BSF	    PORTD, 7
	BSF	    PORTD, 6
	BSF	    PORTD, 5
	BSF	    PORTD, 4
	BSF	    PORTD, 3
	BSF	    PORTD, 2
	BSF	    PORTD, 0
	
	GOTO	CAP_start
	
    Shine_red:
	
	MOVLW   2
	CPFSEQ  follow
	GOTO    Shine_green

	BSF	    PORTD, 7
	BSF	    PORTD, 6
	BSF	    PORTD, 5
	BCF	    PORTD, 4
	BSF	    PORTD, 3
	BSF	    PORTD, 2
	BSF	    PORTD, 0
	
	GOTO	CAP_start
	
    Shine_green:
	
	MOVLW   3
	CPFSEQ  follow

	BSF	    PORTD, 7
	BCF	    PORTD, 6
	BSF	    PORTD, 5
	BSF	    PORTD, 4
	BSF	    PORTD, 3
	BSF	    PORTD, 2
	BSF	    PORTD, 0
	
    CAP_start:
    
    BCF	    ANSELB,3
    BCF	    TRISB,3

    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP

    MOVLW   00100001B
    MOVWF   ADCON0

  ; Set ADC channel to AN9 on RB3
    BCF	PORTB,3

    BSF	ANSELB,3
    BSF	TRISB,3

    MOVLW	    00100101B
    MOVWF	    ADCON0

    BSF	    GO ; Poll till the ADC conversion is done
    BTFSC	    GO
    BRA	    $-2

    MOVLW	    0x80
    CPFSGT	    ADRESH
    BSF		    next, 1 
    CPFSGT	    ADRESH
    GOTO	    LLI

    GOTO	    CAP_TOUCH
    
// </editor-fold>
    
; ---------- Subroutines -------------
// <editor-fold defaultstate="collapsed" desc="Subroutines">
    // <editor-fold defaultstate="collapsed" desc="Subroutines">
TEST_RED:
BCF	    PORTC,5 ; Set LED to red
BCF	    PORTE,2
BCF	    PORTE,1
    
BSF	    PORTC,5 ; Set LED to red

; Perform ADC and see if it pulls high
MOVLW   4
MOVWF   SampleNum
CLRF    SUML
CLRF    SUMH
    
Call    Delay_loop
BSF	GO ; Poll till the ADC conversion is done
BTFSC   GO
BRA	$-2
    
TEST_RED_SAMPLE:
    Call    Delay_loop
    BSF	GO ; Poll till the ADC conversion is done
    BTFSC   GO
    BRA	$-2

    MOVF   ADRESH,W
    ADDWF   SUML,1 ; Store result from ADC in SUML
    BTFSC   STATUS,0
    INCF    SUMH,1 ; If has a carry over, increment the SUMH register

    DECFSZ  SampleNum
    GOTO    TEST_RED_SAMPLE

    CALL    Averaging_short

    ; Shift the LFSR to the correct sensor
    LFSR	0, 0x20F
    MOVFF	POS_reg, INDIRECT_reg
    
    Shift_Ind_R:
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	MOVF	POSTINC0, 0, 0
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	DECFSZ	INDIRECT_reg
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	GOTO	Shift_Ind_R

    MOVF    SUML,W
    
    CPFSGT  INDF0
    BSF	    CALC_reg, 1

    RETURN
    
TEST_GREEN:
    BCF	    PORTC,5
    BSF	    PORTE,2 ; SET LED to GREEN
    BCF	    PORTE,1
    
    ; Perform ADC and see if it pulls high
    MOVLW   4
    MOVWF   SampleNum
    CLRF    SUML
    CLRF    SUMH
    
    Call    Delay_loop
    BSF	GO ; Poll till the ADC conversion is done
    BTFSC   GO
    BRA	$-2
    
    TEST_GREEN_SAMPLE:
	Call    Delay_loop
	BSF	GO ; Poll till the ADC conversion is done
	BTFSC   GO
	BRA	$-2

	MOVF   ADRESH,W
	ADDWF   SUML,1 ; Store result from ADC in SUML
	BTFSC   STATUS,0
	INCF    SUMH,1 ; If has a carry over, increment the SUMH register

	DECFSZ  SampleNum
	GOTO    TEST_GREEN_SAMPLE
    
    CALL    Averaging_short
    
    ; Shift the LFSR to the correct sensor
    LFSR	0, 0x241
    MOVFF	POS_reg, INDIRECT_reg
    
    Shift_Ind_G:
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	MOVF	POSTINC0, 0, 0
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	DECFSZ	INDIRECT_reg
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	GOTO	Shift_Ind_G

    MOVF    SUML,W
    
    CPFSGT  INDF0
    BSF	    CALC_reg, 2
    
    RETURN
    
TEST_BLUE:
    BCF	    PORTC,5
    BCF	    PORTE,2 
    BSF	    PORTE,1 ; SET LED to BLUE
     
    ; Perform ADC and see if it pulls high
    MOVLW   4
    MOVWF   SampleNum
    CLRF    SUML
    CLRF    SUMH
    
    Call    Delay_loop
    BSF	GO ; Poll till the ADC conversion is done
    BTFSC   GO
    BRA	$-2
    
    TEST_BLUE_SAMPLE:
	Call    Delay_loop
	BSF	GO ; Poll till the ADC conversion is done
	BTFSC   GO
	BRA	$-2

	MOVF   ADRESH,W
	ADDWF   SUML,1 ; Store result from ADC in SUML
	BTFSC   STATUS,0
	INCF    SUMH,1 ; If has a carry over, increment the SUMH register

	DECFSZ  SampleNum
	GOTO    TEST_BLUE_SAMPLE
    
    CALL    Averaging_short
    ; Shift the LFSR to the correct sensor
    LFSR	0, 0x237
    MOVFF	POS_reg, INDIRECT_reg
    
    Shift_Ind_B:
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	MOVF	POSTINC0, 0, 0
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	DECFSZ	INDIRECT_reg
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	GOTO	Shift_Ind_B

    MOVF    SUML,W
    
    CPFSGT  INDF0
    BSF	    CALC_reg, 3
    
    RETURN
    
TEST_BLACK: 
    BCF	    PORTC,5 
    BCF	    PORTE,2 
    BSF	    PORTE,1 ; Set the led to blue
     
    ; Perform ADC and see if it pulls high
    Call    Delay_loop
    MOVLW   4
    MOVWF   SampleNum
    CLRF    SUML
    CLRF    SUMH
    
    Call    Delay_loop
    BSF	GO ; Poll till the ADC conversion is done
    BTFSC   GO
    BRA	$-2
    
    TEST_BLACK_SAMPLE:
	BSF	GO ; Poll till the ADC conversion is done
	BTFSC   GO
	BRA	$-2

	MOVF   ADRESH,W
	ADDWF   SUML,1 ; Store result from ADC in SUML
	BTFSC   STATUS,0
	INCF    SUMH,1 ; If has a carry over, increment the SUMH register

	DECFSZ  SampleNum
	GOTO    TEST_BLACK_SAMPLE
    
    CALL    Averaging_short
    ; Shift the LFSR to the correct sensor
    LFSR	0, 0x228
    MOVFF	POS_reg, INDIRECT_reg

    Shift_Ind_K:
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	MOVF	POSTINC0, 0, 0
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	DECFSZ	INDIRECT_reg
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	GOTO	Shift_Ind_K
    MOVF    SUML,W
    
    CPFSGT  INDF0
    BSF	    CALC_reg, 4
    
    RETURN
    
TEST_WHITE_RED:
    BSF	    PORTC,5
    BCF	    PORTE,2 
    BCF	    PORTE,1
    
    ; Perform ADC and see if it pulls high
    MOVLW   4
    MOVWF   SampleNum
    CLRF    SUML
    CLRF    SUMH
    
    Call    Delay_loop
    BSF	GO ; Poll till the ADC conversion is done
    BTFSC   GO
    BRA	$-2
    
    LOOP_RED_LLI:
        Call    Delay_loop
	BSF	GO ; Poll till the ADC conversion is done
	BTFSC   GO
	BRA	$-2

	MOVF   ADRESH,W
	ADDWF   SUML,1 ; Store result from ADC in SUML
	BTFSC   STATUS,0
	INCF    SUMH,1 ; If has a carry over, increment the SUMH register

	DECFSZ  SampleNum
	GOTO    LOOP_RED_LLI
    
    CALL    Averaging_short
    ; Shift the LFSR to the correct sensor
    LFSR	0, 0x200
    MOVFF	POS_reg, INDIRECT_reg
    
    Shift_Ind_RW:
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	MOVF	POSTINC0, 0, 0
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	DECFSZ	INDIRECT_reg
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	GOTO	Shift_Ind_RW

    MOVF    SUML,W
    
    CPFSLT  INDF0
    RETURN
    
    BSF	    CALC_reg, 0
    RETURN
    
TEST_WHITE:
    BCF	    PORTC,5
    BCF	    PORTE,2 ; Set LED to Green
    BCF	    PORTE,1
    
    BSF	    PORTE,2 ; Set LED to Green
    
    ; Perform ADC and see if it pulls high
    MOVLW   4
    MOVWF   SampleNum
    CLRF    SUML
    CLRF    SUMH
    
    Call    Delay_loop
    BSF	GO ; Poll till the ADC conversion is done
    BTFSC   GO
    BRA	$-2
    
    LOOP_GREEN_LLI:
        Call    Delay_loop
	BSF	GO ; Poll till the ADC conversion is done
	BTFSC   GO
	BRA	$-2

	MOVF   ADRESH,W
	ADDWF   SUML,1 ; Store result from ADC in SUML
	BTFSC   STATUS,0
	INCF    SUMH,1 ; If has a carry over, increment the SUMH register

	DECFSZ  SampleNum
	GOTO    LOOP_GREEN_LLI
    
    CALL    Averaging_short
    ; Shift the LFSR to the correct sensor
    LFSR	0, 0x205
    MOVFF	POS_reg, INDIRECT_reg
    
    Shift_Ind_GW:
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	MOVF	POSTINC0, 0, 0
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	DECFSZ	INDIRECT_reg
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	GOTO	Shift_Ind_GW

    MOVF    SUML,W
    
    CPFSLT  INDF0
    RETURN
    BCF	    PORTC,5
    BCF	    PORTE,2 ; Set LED to Green
    BCF	    PORTE,1
    
    BSF	    PORTE,1 ; Set LED to blue
    
    ; Perform ADC and see if it pulls high
    MOVLW   4
    MOVWF   SampleNum
    CLRF    SUML
    CLRF    SUMH
    
    Call    Delay_loop
    BSF	GO ; Poll till the ADC conversion is done
    BTFSC   GO
    BRA	$-2
    
    LOOP_BLUE_LLI:
        Call    Delay_loop
	BSF	GO ; Poll till the ADC conversion is done
	BTFSC   GO
	BRA	$-2

	MOVF   ADRESH,W
	ADDWF   SUML,1 ; Store result from ADC in SUML
	BTFSC   STATUS,0
	INCF    SUMH,1 ; If has a carry over, increment the SUMH register

	DECFSZ  SampleNum
	GOTO    LOOP_BLUE_LLI
    
    CALL    Averaging_short
    ; Shift the LFSR to the correct sensor
    LFSR	0, 0x20A
    MOVFF	POS_reg, INDIRECT_reg
    
    Shift_Ind_GB:
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	MOVF	POSTINC0, 0, 0
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	DECFSZ	INDIRECT_reg
	MOVLW	0x00
	CPFSEQ	INDIRECT_reg
	GOTO	Shift_Ind_GB

    MOVF    SUML,W
    
    CPFSLT  INDF0
    RETURN
    
    ; At this point, we know that we are on white
    BSF	    CALC_reg, 0
    RETURN
    
Averaging:
    MOVLW   0x4
    MOVWF   SampleNum
AverageStart:
    RRCF   SUMH,1
    RRCF   SUML,1
    BCF	   STATUS,0
    
    DECFSZ  SampleNum
    GOTO    AverageStart
    
    RETURN
    
Averaging_short:
    MOVLW   0x2
    MOVWF   SampleNum
AverageStart_s:
    RRCF   SUMH,1
    RRCF   SUML,1
    BCF	   STATUS,0
    
    DECFSZ  SampleNum
    GOTO    AverageStart_s
    
    RETURN
    
ChangeADChannel: ; used to change the ADC channel to sample each photodiode
    MOVLW   00000100B
    ADDWF   ADCON0   ; Add 1 to the ADCOn register to shift it channel
    MOVLW   00010101B ; if at channel 4, skip and move on to change back to chanel 0
    CPFSEQ  ADCON0
    RETURN
    
    MOVLW   00000001B
    MOVWF   ADCON0
    
    RETURN
    
FlashLED:
    BSF	    PORTE,2		; turn on LED
    CALL    FlashDelay
    BCF     PORTE,2			; turn off LED 
    CALL    FlashDelay
    DECFSZ  Count,1
    GOTO    FlashLED
    CLRF    Count
    
    RETURN
    
Delay_loop: ; Delay for flashing LED in Calibration Sequence	
    ; Delay is inner loop = 3 instructions * 256 loops = 768
    ; outer loop = (5 instructions + 768) * 214 = 166422 us
    MOVLW	0x01
    MOVWF	DELAY2		
Go1:					
    MOVLW	0x60
    MOVWF	DELAY1
Go2:	
    DECFSZ	DELAY1,f	
    GOTO	Go2		
    DECFSZ	DELAY2,f	
    GOTO	Go1		

    RETURN
    
FlashDelay: ; Delay for flashing LED in Calibration Sequence	
    ; Delay is inner loop = 3 instructions * 256 loops = 768
    ; outer loop = (5 instructions + 768) * 214 = 166422 us
    MOVLW	0xD6
    MOVWF	DELAY2		
Go1F:					
    MOVLW	0xFF
    MOVWF	DELAY1
Go2F:	
    DECFSZ	DELAY1,f	
    GOTO	Go2F		
    DECFSZ	DELAY2,f	
    GOTO	Go1F		

    RETURN
    
; ---------- Interrupts ------------
// <editor-fold defaultstate="collapsed" desc="Interrupts">
ISR_next: 
    BCF	    INT1IF
    BCF	    INT0IF  ; clear RB0 interrupt flag
    
    BTFSC   next,2 ; check if in captouch
    GOTO    CALIBRATION_RETRIEVAL ; if not, move on to captouch
    GOTO    State0
    
    RETFIE
    
ISR_change:
    BCF	    INT0IF
    BCF     INT1IF
    
    BTFSC   next,2 
    BRA	    CALIBRATIONROUT
    BRA	    RACEROUT
    
CALIBRATIONROUT:
    MOVF    PORTB ; Read modify error
    BSF	    next, 0
    RETFIE
    
RACEROUT:    
    MOVF    PORTB
    MOVLW   3
    CPFSLT  follow ; If not following green skip, else make 0
    SUBWF   follow
    MOVLW   1
    ADDWF   follow
    
    ;CHANGE SSD to represent the following
    ; 1 - BLUE
    ; 2 - RED
    ; 3 - GREEN
    
    MOVLW 0x01
    CPFSGT follow
    BRA	   makeB
    
    MOVLW   0x02
    CPFSGT  follow
    BRA	    makeR
    
    CALL    OnGreen
    RETFIE
    
    makeB:
    CALL  OnBlue
    RETFIE
    
    makeR:
    CALL  OnRed
    RETFIE
    // </editor-fold>
    
   end
