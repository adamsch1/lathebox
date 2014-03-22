;**** **** **** **** ****
;
;Die Benutzung der Software ist mit folgenden Bedingungen verbunden:
;
;1. Da ich alles kostenlos zur Verfügung stelle, gebe ich keinerlei Garantie
;   und übernehme auch keinerlei Haftung für die Folgen der Benutzung.
;
;2. Die Software ist ausschließlich zur privaten Nutzung bestimmt. Ich
;   habe nicht geprüft, ob bei gewerblicher Nutzung irgendwelche Patentrechte
;   verletzt werden oder sonstige rechtliche Einschränkungen vorliegen.
;
;3. Jeder darf Änderungen vornehmen, z.B. um die Funktion seinen Bedürfnissen
;   anzupassen oder zu erweitern. Ich würde mich freuen, wenn ich weiterhin als
;   Co-Autor in den Unterlagen erscheine und mir ein Link zur entprechenden Seite
;   (falls vorhanden) mitgeteilt wird.
;
;4. Auch nach den Änderungen sollen die Software weiterhin frei sein, d.h. kostenlos bleiben.
;
;!! Wer mit den Nutzungbedingungen nicht einverstanden ist, darf die Software nicht nutzen !!
;
; Juli 2004
; autor: Bernhard Konze
; email: bernhard.konze(at)versanet.de
;
;**** **** **** **** ****
; Device
;
;**** **** **** **** ****
.include "2313def.inc"
;

.equ MOT_BRAKE    = 0

.equ UART_CONTROL = 0
.equ RC_PULS 	  = 1

; if full rc-pulse-range is lower than 800ms - make 533ms range and set to "1"
.equ RANGE533	= 0

.include "teba.inc"
;.include "sergey.inc"
;.include "mmormota.inc"

.equ	CHANGE_TIMEOUT	= 0x32
.equ	CHANGE_TOT_LOW	= 0x28

.equ	POWER_RANGE	= 100			; full range of tcnt0 setting
.equ	MIN_DUTY	= 5			; no power
.equ	NO_POWER	= 256-MIN_DUTY		; (POWER_OFF)
.equ	MAX_POWER	= 256-POWER_RANGE	; (FULL_POWER)

.equ	PWR_MAX_RPM1	= POWER_RANGE/4
.equ	PWR_MAX_RPM2	= POWER_RANGE/2

.equ	PWR_STARTUP	= MIN_DUTY
.equ	PWR_MAX_STARTUP	= MIN_DUTY+5

.equ	compScanSTART	= 18000

.equ	timeoutSTART	= 48000
.equ	timeoutMIN	= 36000

.equ	T1STOP	= 0x00
.equ	T1CK8	= 0x02

.equ	EXT0_DIS	= 0x00	; disable ext0int
.if RC_PULS == 1
.equ	EXT0_EN		= 0x40	; enable ext0int
.else
.equ	EXT0_EN		= 0x00	; disable ext0int
.endif

.if RANGE533 == 1
	.equ	MIN_RC_PULS	= 1250	; µs (or lower) = NO_POWER
.else
	.equ	MIN_RC_PULS	= 1100	; µs (or lower) = NO_POWER
.endif	; RANGE533 == 1


.equ	PWR_RANGE1	= 0x40	; ( ~2400 RPM )
.equ	PWR_RANGE2	= 0x20	; ( ~4800 RPM )

.equ	ENOUGH_GOODIES	= 60

;**** **** **** **** ****
; Register Definitions
.def	i_sreg		 = r1	; status register save in interrupts
.def	tcnt0_power_on	 = r2	; timer0 counts nFETs are switched on
.def	tcnt0_change_tot = r3	; when zero, tcnt0_power_on is changed by one (inc or dec)
.def	TCNT1X	 	 = r4	; upper 8bit timer1 (software-) register
.def	uart_cnt	 = r5
.def	tcnt0_pwron_next = r6

.def	start_rcpuls_l	 = r7
.def	start_rcpuls_h	 = r8
.def	new_rcpuls_l	 = r9
.def	new_rcpuls_h	 = r10
.def	rcpuls_timeout	 = r11
.equ	RCP_TOT		 = 100

.def	current_err	 = r12	; counts consecutive current errors
.equ	CURRENT_ERR_MAX  = 3	; performs a reset after MAX errors

.def	sys_control	 = r13
.def	t1_timeout	 = r14
.def	run_control	 = r15


.def	temp1	= r16			; main temporary
.def	temp2	= r17			; main temporary
.def	temp3	= r18			; main temporary
.def	temp4	= r19			; main temporary

.def	i_temp1	= r20			; interrupt temporary
.def	i_temp2	= r21			; interrupt temporary
.def	i_temp3	= r22			; interrupt temporary

.def	state0	= r23	; state flags
	.equ	OCT1_PENDING	= 0	; if set, output compare interrunpt is pending
	.equ	UB_LOW 		= 1	; set if accu voltage low
	.equ	I_pFET_HIGH	= 2	; set if over-current detect
	.equ	GET_STATE	= 3	; set if state is to be send
	.equ	C_FET		= 4	; if set, C-FET state is to be changed
	.equ	A_FET		= 5	; if set, A-FET state is to be changed
	     ; if neither 1 nor 2 is set, B-FET state is to be changed
	.equ	I_OFF_CYCLE	= 6	; if set, current off cycle is active
	.equ	T1OVFL_FLAG	= 7	; each timer1 overflow sets this flag - used for voltage + current watch

.def	state1	= r24	; state flags
	.equ	POWER_OFF	= 0	; switch fets on disabled
	.equ	FULL_POWER	= 1	; 100% on - don't switch off, but do OFF_CYCLE working
	.equ	CALC_NEXT_OCT1	= 2	; calculate OCT1 offset, when wait_OCT1_before_switch is called
	.equ	RC_PULS_UPDATED	= 3	; new rc-puls value available
	.equ	EVAL_RC_PULS	= 4	; if set, new rc puls is evaluated, while waiting for OCT1
	.equ	EVAL_SYS_STATE	= 5	; if set, overcurrent and undervoltage are checked
	.equ	EVAL_I_pFET	= 6	; if set, next PWM on should look for current
	.equ	EVAL_UART	= 7	; if set, next PWM on should look for uart

.def	state2	= r25
	.equ	RPM_RANGE1	= 0	; if set RPM is lower than 1831 RPM
	.equ	RPM_RANGE2	= 1	; if set RPM is between 1831 RPM and 3662 RPM
	.equ	SCAN_TIMEOUT	= 2	; if set a startup timeout occurred
	.equ	POFF_CYCLE	= 3	; if set one commutation cycle is performed without power
	.equ	EVAL_UB		= 4	; if set voltage is measured
	.equ	STARTUP		= 5	; if set startup-phase is active


; here the XYZ registers are placed ( r26-r31)

; ZH = new_duty		; PWM destination


;**** **** **** **** ****
; RAM Definitions
.dseg					;EEPROM segment
.org 0x0060

tcnt1_sav_l:	.byte	1	; actual timer1 value
tcnt1_sav_h:	.byte	1
last_tcnt1_l:	.byte	1	; last timer1 value
last_tcnt1_h:	.byte	1
timing_l:	.byte	1	; holds time of 4 commutations 
timing_h:	.byte	1
timing_x:	.byte	1

wt_comp_scan_l:	.byte	1	; time from switch to comparator scan
wt_comp_scan_h:	.byte	1       
com_timing_l:	.byte	1	; time from zero-crossing to switch of the appropriate FET
com_timing_h:	.byte	1
wt_OCT1_tot_l:	.byte	1	; OCT1 waiting time
wt_OCT1_tot_h:	.byte	1
zero_wt_l:	.byte	1
zero_wt_h:	.byte	1
last_com_l:	.byte	1
last_com_h:	.byte	1

duty_offset:	.byte	1
goodies:	.byte	1
comp_state:	.byte	1

uart_data:	.byte	100		; only for debug requirements


;**** **** **** **** ****
; 2313 interrupts
;.equ	INT0addr=$001	;External Interrupt0 Vector Address
;.equ	INT1addr=$002	;External Interrupt1 Vector Address
;.equ	ICP1addr=$003	;Input Capture1 Interrupt Vector Address
;.equ	OC1addr =$004	;Output Compare1 Interrupt Vector Address
;.equ	OVF1addr=$005	;Overflow1 Interrupt Vector Address
;.equ	OVF0addr=$006	;Overflow0 Interrupt Vector Address
;.equ	URXCaddr=$007	;UART Receive Complete Interrupt Vector Address
;.equ	UDREaddr=$008	;UART Data Register Empty Interrupt Vector Address
;.equ	UTXCaddr=$009	;UART Transmit Complete Interrupt Vector Address
;.equ	ACIaddr =$00a	;Analog Comparator Interrupt Vector Address

;-----bko-----------------------------------------------------------------

;**** **** **** **** ****
.cseg
.org 0
;**** **** **** **** ****

;-----bko-----------------------------------------------------------------
; reset and interrupt jump table
		rjmp	reset
.if RC_PULS == 1
		rjmp	ext_int0
.else
		nop	; int0
.endif
		nop	; int1
		nop	; icp1
		rjmp	t1oc_int
		rjmp	t1ovfl_int
		rjmp	t0ovfl_int
		nop	; urxc
		nop	; udre
.if UART_CONTROL == 1
		rjmp	utxc
.else
		nop	; utxc
.endif	; UART_CONTROL == 1

; not used	nop	; aci


version:	.db	"bko410-r05"


;-----bko-----------------------------------------------------------------
; init after reset

reset:		ldi	temp1, RAMEND
		out	SPL, temp1		; stack = RAMEND

	; portB - all FETs off
		ldi	temp1, INIT_PB		; PORTB initially holds 0x00
		out	PORTB, temp1
		ldi	temp1, DIR_PB		; pb1+pb0 are inputs
		out	DDRB, temp1		; portB is output

	; portD reads comparator inputs + rc-puls
		ldi	temp1, INIT_PD		; enable pullup's for comparator outputs
		out	PORTD, temp1
		ldi	temp1, DIR_PD
		out	DDRD, temp1

	; timer0: PWM + beep control = 0x02 	; start timer0 with CK/8 (1µs/count)
		ldi	temp1, 0x02
		out	TCCR0, temp1

	; timer1: commutation control = 0x02	; start timer1 with CK/8 (1µs/count)
		ldi	temp1, T1CK8
		out	TCCR1B, temp1

	; reset state flags
		clr	state0
		clr	state1
		clr	state2

	; clear RAM
		clr	XH
		ldi	XL, 0x60
		clr	temp1
clear_ram:	st	X+, temp1
		cpi	XL, uart_data+1
		brlo	clear_ram

	; power off
		rcall	switch_power_off

	; reset rc puls timeout
		ldi	temp1, RCP_TOT
		mov	rcpuls_timeout, temp1
		
.if UART_CONTROL == 1

		ldi	ZH,high(version*2)
		ldi	ZL,low(version*2)

	;**** UART Initialization ****
		ldi	temp1, 12
		out	UBRR, temp1	; set to 38400baud (8MHz)
		ldi	temp1, 0x18
		out	UCR, temp1	; enable rx+tx - no interrupts !
		in 	temp1, UDR	; clear possibly pending rxc
		ldi	temp1, 0
		mov	uart_cnt, temp1

		lpm
		adiw	ZL,1		;increment Z-pointer
		mov	temp1, r0	; b
		rcall	send_byte
.endif	; UART_CONTROL == 1

.if RC_PULS == 1
		rcall	wait260ms	; wait a while
		rcall	wait260ms
.endif	; RC_PULS == 1

.if UART_CONTROL == 1
		lpm
		adiw	ZL,1		;increment Z-pointer
		mov	temp1, r0	; k
		rcall	send_byte
.endif	; UART_CONTROL == 1

		rcall	beep_f1
		rcall	wait30ms
		rcall	beep_f2
		rcall	wait30ms
		rcall	beep_f3
		rcall	wait30ms

.if UART_CONTROL == 1
		lpm
		adiw	ZL,1		;increment Z-pointer
		mov	temp1, r0	; o
		rcall	send_byte
		lpm
		adiw	ZL,1		;increment Z-pointer
		mov	temp1, r0	; (1st number)
		rcall	send_byte
.endif	; UART_CONTROL == 1

control_start:	; init variables
		ldi	temp1, CHANGE_TIMEOUT
		mov	tcnt0_change_tot, temp1
		ldi	temp1, NO_POWER
		mov	tcnt0_power_on, temp1

		ldi	temp1, 0		; reset error counters
		mov	current_err,temp1
		mov	sys_control, temp1

	; init registers and interrupts
		ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0)
		out	TIFR, temp1		; clear TOIE1,OCIE1A & TOIE0
		out	TIMSK, temp1		; enable TOIE1,OCIE1A & TOIE0 interrupts

.if UART_CONTROL == 1
		lpm
		adiw	ZL,1		;increment Z-pointer
		mov	temp1, r0	; (2nd number)
		rcall	send_byte
.endif	; UART_CONTROL == 1

		sei				; enable all interrupts

.if RC_PULS == 1
; init rc-puls
		ldi	temp1, (1<<ISC01)+(1<<ISC00)
		out	MCUCR, temp1		; set next int0 to rising edge
		ldi	temp1, EXT0_EN		; enable ext0int
		out	GIMSK, temp1
i_rc_puls1:	ldi	temp3, 10		; wait for this count of receiving power off
i_rc_puls2:	sbrs	state1, RC_PULS_UPDATED
		rjmp	i_rc_puls2
		mov	temp1, new_rcpuls_l
		mov	temp2, new_rcpuls_h
		cbr	state1, (1<<RC_PULS_UPDATED) ; rc impuls value is read out
		subi	temp1, low  (MIN_RC_PULS)	; power off received ?
		sbci	temp2, high (MIN_RC_PULS)
		brcc	i_rc_puls1		; no - reset counter
		dec	temp3			; yes - decrement counter
		brne	i_rc_puls2		; repeat until zero
		cli				; disable all interrupts
		rcall	beep_f4			; signal: rcpuls ready
		rcall	beep_f4
		rcall	beep_f4
		sei				; enable all interrupts
.endif	; RC_PULS == 1

.if UART_CONTROL == 1
		lpm
		adiw	ZL,1		;increment Z-pointer
		mov	temp1, r0	; (3rd number)
		rcall	send_byte

		cli				; disable all interrupts
		rcall	wait260ms
		rcall	beep_f4			; signal: rcpuls ready
		rcall	beep_f4
		rcall	beep_f4
		sei				; enable all interrupts

.endif	; UART_CONTROL == 1

		ldi	YL, low  (timeoutSTART)
		ldi	YH, high (timeoutSTART)
		sts	wt_OCT1_tot_l, YL
		sts	wt_OCT1_tot_h, YH

		ldi	temp1, 30
		sts	duty_offset, temp1

		rjmp	init_startup
		
;-----bko-----------------------------------------------------------------
; external interrupt0 = rc pulse input
.if RC_PULS == 1
ext_int0:	in	i_sreg, SREG
		clr	i_temp1			; disable extint edge may be changed
		out	GIMSK, i_temp1

; evaluate edge of this interrupt
		in	i_temp1, MCUCR
		sbrs	i_temp1, ISC00
		rjmp	falling_edge		; bit is clear = falling edge

; should be rising edge - test rc impuls level state for possible jitter
		sbis	PIND, rcp_in
		rjmp	extint1_exit		; jump, if low state

; rc impuls is at high state
		ldi	i_temp1, (1<<ISC01)
		out	MCUCR, i_temp1		; set next int0 to falling edge

; get timer1 values
		in	i_temp1, TCNT1L
		in	i_temp2, TCNT1H
		mov	start_rcpuls_l, i_temp1
		mov	start_rcpuls_h, i_temp2

		rjmp	extint1_exit

; rc impuls is at low state
falling_edge:	sbic	PIND, rcp_in		; test level of rc impuls
		rjmp	extint1_exit		; seems to be a spike

		ldi	i_temp1, (1<<ISC01)+(1<<ISC00)
		out	MCUCR, i_temp1		; set next int0 to rising edge
		sbrc	state1, RC_PULS_UPDATED
		rjmp	extint1_exit

; get timer1 values
		in	i_temp1, TCNT1L
		in	i_temp2, TCNT1H
		sub	i_temp1, start_rcpuls_l
		sbc	i_temp2, start_rcpuls_h

	; save impuls length
		mov	new_rcpuls_l, i_temp1
		mov	new_rcpuls_h, i_temp2
		cpi	i_temp1, low (2200)
		ldi	i_temp3, high(2200)	; test range high
		cpc	i_temp2, i_temp3
		brsh	extint1_exit		; through away
		cpi	i_temp1, low (800)
		ldi	i_temp3, high(800)	; test range low
		cpc	i_temp2, i_temp3
		brlo	extint1_exit		; through away
		sbr	state1, (1<<RC_PULS_UPDATED) ; set to rc impuls value is ok !
		ldi	i_temp1, RCP_TOT
		mov	rcpuls_timeout, i_temp1

; enable int1 again -  also entry for spike detect
extint1_exit:	ldi	i_temp2, EXT0_EN
		out	GIMSK, i_temp2
		out	SREG, i_sreg
		reti
.endif  ; RC_PULS == 1
;-----bko-----------------------------------------------------------------
; output compare timer1 interrupt
t1oc_int:	in	i_sreg, SREG
		cbr	state0, (1<<OCT1_PENDING) ; signal OCT1 passed
		out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; overflow timer1 / happens all 65536µs
t1ovfl_int:	in	i_sreg, SREG
		inc	TCNT1X			; make a 24-bit counter
		sbr	state0, (1<<T1OVFL_FLAG)

		tst	t1_timeout
		breq	t1ovfl_10
		dec	t1_timeout
t1ovfl_10:
.if RC_PULS == 1
		tst	rcpuls_timeout
		breq	t1ovfl_99
		dec	rcpuls_timeout
.endif	; RC_PULS == 1

t1ovfl_99:	out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
; timer0 overflow interrupt
t0ovfl_int:	in	i_sreg, SREG
		sbrc	state0, I_OFF_CYCLE
		rjmp	t0_on_cycle

t0_off_cycle:
	; save comparator state just before switching off
		in	i_temp1, PIND
		sts	comp_state, i_temp1
	; changes in PWM ?
		mov	i_temp1, tcnt0_power_on
		mov	i_temp2, tcnt0_pwron_next
		cp	i_temp2, i_temp1
		brsh	lower_pwm		; next power-on-time is lower or same
higher_pwm:	dec	tcnt0_change_tot	; change-timeout passed ?
		brne	nFET_off		; .. no
		ldi	i_temp2, CHANGE_TIMEOUT	; .. yes - change-timeout for more power
		mov	tcnt0_change_tot, i_temp2 ; reset change-timeout and decrement
		dec	i_temp1			; <dec> increases power-on-time
		rjmp	set_next_pwm

lower_pwm:	breq	nFET_off		; pwm is unchanged
		dec	tcnt0_change_tot	; change-timeout passed ?
		brne	nFET_off		; .. no
		ldi	i_temp2, CHANGE_TOT_LOW ; .. yes - change-timeout for lower power
		mov	tcnt0_change_tot, i_temp2 ; reset change-timeout and increment
		inc	i_temp1			; <inc> decreases power-on-time

set_next_pwm:	mov	tcnt0_power_on, i_temp1

nFET_off:	sbr	state0, (1<<I_OFF_CYCLE) ; PWM state = off cycle

	; switch appropriate nFET off
		sbrs	state0, C_FET
		rjmp	test_AnFET

; C_FET is active
		sbrs	state2, EVAL_UB		; evaluation request ?
		rjmp	switch_CnFET
	; Ap + Cn active - look for undervoltage
		sbis	PIND, compUB
		sbr	state0, (1<<UB_LOW)
		cbr	state2, (1<<EVAL_UB)

switch_CnFET:	sbrs	state1, FULL_POWER
		cbi	PORTB, CnFET		; Cn off
		rjmp	reload_t0_off_cycle

test_AnFET:	sbrs	state0, A_FET
		rjmp	switch_BnFET

; A_FET is active
switch_AnFET:	sbrs	state1, FULL_POWER
		cbi	PORTB, AnFET		; An off
		rjmp	reload_t0_off_cycle

; B_FET is active
switch_BnFET:	sbrs	state1, FULL_POWER
		cbi	PORTB, BnFET		; Bn off

	; reload timer0 with the appropriate value
reload_t0_off_cycle:
		mov	i_temp1, tcnt0_power_on
		subi	i_temp1, -POWER_RANGE	; adi i_temp1, POWER_RANGE
		com	i_temp1			; timer0 increments
		out	TCNT0, i_temp1

		rjmp	t0_int_exit

; reload timer90 + switch appropriate nFET on
t0_on_cycle:	mov	i_temp1, tcnt0_power_on
		out	TCNT0, i_temp1		; reload t0
		cbr	state0, (1<<I_OFF_CYCLE) ; PWM state = on cycle (no off cycle)

; switch appropriate nFET on
nFET_on:	sbrs	state0, C_FET		; is Cn choppered ?
		rjmp	test_AnFET_on			; .. no - test An
		sbrs	state1, POWER_OFF
		sbi	PORTB, CnFET		; Cn on
		rjmp	eval_power_state
test_AnFET_on:	sbrs	state0, A_FET		; is An choppered ?
		rjmp	sw_BnFET_on			; .. no - Bn has to be choppered
		sbrs	state1, POWER_OFF
		sbi	PORTB, AnFET		; An on
		rjmp	eval_power_state
sw_BnFET_on:	sbrs	state1, POWER_OFF
		sbi	PORTB, BnFET		; Bn on

	; evaluate power state
eval_power_state:
		cpi	i_temp1, MAX_POWER+1
		brsh	not_full_power
	; FULL POWER
		sbr	state1, (1<<FULL_POWER)	; tcnt0_power_on = MAX_POWER means FULL_POWER
		cbr	state1, (1<<POWER_OFF)
		rjmp	eval_ApFET_voltage
not_full_power:	cpi	i_temp1, NO_POWER
		brlo	neither_full_nor_off
	; POWER OFF
		cbr	state1, (1<<FULL_POWER)	; tcnt0_power_on = NO_POWER means power off
		sbr	state1, (1<<POWER_OFF)
		rjmp	eval_ApFET_voltage
neither_full_nor_off:
		cbr	state1, (1<<FULL_POWER)	; tcnt0_power_on = MAX_POWER means FULL_POWER
		cbr	state1, (1<<POWER_OFF)

eval_ApFET_voltage:
		sbrs	state1, EVAL_I_pFET	; evaluation request ?
		rjmp	t0_int_exit
		sbis	PORTB, BnFET		; also Bn on ?
		rjmp	t0_int_exit
	; Cp + Bn active - look for overcurrent
		sbis	ACSR, ACO		; if ACO=1 current is ok
		sbr	state0, (1<<I_pFET_HIGH)
		cbr	state1, (1<<EVAL_I_pFET)

t0_int_exit:	sbrc	state2, POFF_CYCLE
		sbr	state1, (1<<POWER_OFF)
		out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------
.if UART_CONTROL == 1
utxc:		in	i_sreg, SREG
		ld	i_temp1,X+
		out	UDR, i_temp1
		dec	uart_cnt
		brne	utxc_90
		cbi	UCR, TXCIE		; disable irq
utxc_90:	out	SREG, i_sreg
		reti
.endif	; UART_CONTROL == 1
;-----bko-----------------------------------------------------------------
; beeper: timer0 is set to 8µs/count - 2072µs period (about 500hz beep)
; beeper: timer0 is set to 1µs/count
beep_f1:	ldi	temp4, 200
		ldi	temp2, 80
		rjmp	beep

beep_f2:	ldi	temp4, 180
		ldi	temp2, 100
		rjmp	beep

beep_f3:	ldi	temp4, 160
		ldi	temp2, 120
		rjmp	beep

beep_f4:	ldi	temp4, 100
		ldi	temp2, 200
		rjmp	beep

beep:		clr	temp1
		out	TCNT0, temp1
		sbi	PORTB, BpFET		; BpFET on
		sbi	PORTB, AnFET		; CnFET on
beep_BpCn10:	in	temp1, TCNT0
		cpi	temp1, 32		; 32µs on
		brne	beep_BpCn10
		cbi	PORTB, BpFET		; BpFET off
		cbi	PORTB, AnFET		; CnFET off
		ldi	temp3, 8		; 2040µs off
beep_BpCn12:	clr	temp1
		out	TCNT0, temp1
beep_BpCn13:	in	temp1, TCNT0
		cp	temp1, temp4
		brne	beep_BpCn13
		dec	temp3
		brne	beep_BpCn12
		dec	temp2
		brne	beep
		ret

wait30ms:	ldi	temp2, 15
beep_BpCn20:	ldi	temp3, 8
beep_BpCn21:	clr	temp1
		out	TCNT0, temp1
beep_BpCn22:	in	temp1, TCNT0
		cpi	temp1, 255
		brne	beep_BpCn22
		dec	temp3
		brne	beep_BpCn21
		dec	temp2
		brne	beep_BpCn20
		ret

	; 128 periods = 261ms silence
wait260ms:	ldi	temp2, 128
beep2_BpCn20:	ldi	temp3, 8
beep2_BpCn21:	clr	temp1
		out	TCNT0, temp1
beep2_BpCn22:	in	temp1, TCNT0
		cpi	temp1, 255
		brne	beep2_BpCn22
		dec	temp3
		brne	beep2_BpCn21
		dec	temp2
		brne	beep2_BpCn20
		ret
;-----bko-----------------------------------------------------------------
tcnt1_to_temp:	ldi	temp4, EXT0_DIS		; disable ext0int
		out	GIMSK, temp4
		ldi	temp4, T1STOP		; stop timer1
		out	TCCR1B, temp4
		ldi	temp4, T1CK8		; preload temp with restart timer1
		in	temp1, TCNT1L		;  - the preload cycle is needed to complete stop operation
		in	temp2, TCNT1H
		out	TCCR1B, temp4
		ret				; !!! ext0int stays disabled - must be enabled again by caller
	; there seems to be only one TEMP register in the AVR
	; if the ext0int interrupt falls between readad LOW value while HIGH value is captured in TEMP and
	; read HIGH value, TEMP register is changed in ext0int routine
;-----bko-----------------------------------------------------------------
.if RC_PULS == 1
evaluate_rc_puls:
		cbr	state1, (1<<EVAL_RC_PULS)
		sbrs	state1, RC_PULS_UPDATED
		rjmp	eval_rc_p90
		mov	temp1, new_rcpuls_l
		mov	temp2, new_rcpuls_h
		cbr	state1, (1<<RC_PULS_UPDATED) ; rc impuls value is read out
		subi	temp1, low  (MIN_RC_PULS)
		sbci	temp2, high (MIN_RC_PULS)
		brcc	eval_rc_p00
		clr	temp1
		clr	temp2
eval_rc_p00:	lsr	temp2			; (0-800) -> (0-100)
		ror	temp1
		lsr	temp2
		ror	temp1

.if RANGE533 == 1	; full rc-pulse-range is lower than 800ms - make 533ms range
	; actual (0-533) -> (0-133)
		mov	temp3, temp1	; (0-133) -> (0-100)
		lsr	temp3
		lsr	temp3
		sub	temp1, temp3
.else
	; actual (0-800) -> (0-200)
		lsr	temp2		; (0-200) -> (0-100)
		ror	temp1

.endif	; RANGE533 == 1

		mov	temp3, temp1		
		subi	temp1, low  (POWER_RANGE)
		sbci	temp2, high (POWER_RANGE)
		brcs	eval_rc_p10
		ldi	temp3, low  (POWER_RANGE)
eval_rc_p10:	mov	ZH, temp3
eval_rc_p90:	ret
.endif	; RC_PULS == 1
;-----bko-----------------------------------------------------------------
evaluate_uart:	cbr	state1, (1<<EVAL_UART)
.if UART_CONTROL == 1
		sbic	USR, RXC
		rcall	read_uart
.endif
		ret
;-----bko-----------------------------------------------------------------
evaluate_sys_state:
		cbr	state1, (1<<EVAL_SYS_STATE)
		sbrs	state0, T1OVFL_FLAG
		rjmp	eval_sys_s99

	; do it not more often as every 65µs
		cbr	state0, (1<<T1OVFL_FLAG)

	; control current
eval_sys_i:	sbrs	state0, I_pFET_HIGH
		rjmp	eval_sys_i_ok
		cbr	state0, (1<<I_pFET_HIGH)
		mov	i_temp1, current_err
		cpi	i_temp1, CURRENT_ERR_MAX
		brcc	panic_exit
		inc	current_err
		rjmp	eval_sys_ub

eval_sys_i_ok:	tst	current_err
		breq	eval_sys_ub
		dec	current_err

	; control voltage
eval_sys_ub:	sbrs	state0, UB_LOW
		rjmp	eval_sys_ub_ok
		cbr	state0, (1<<UB_LOW)
		mov	i_temp1, sys_control
		cpi	i_temp1, POWER_RANGE
		brcc	eval_sys_s99
		inc	sys_control
		rjmp	eval_sys_s99

eval_sys_ub_ok:	tst	sys_control
		breq	eval_sys_s99
		dec	sys_control
		
eval_sys_s99:	rcall	set_new_duty
		ret

panic_exit:	; !!!!!! OVERCURRENT !!!!!!!!
		cli
		rjmp	reset
;-----bko-----------------------------------------------------------------
update_timing:	rcall	tcnt1_to_temp
		sts	tcnt1_sav_l, temp1
		sts	tcnt1_sav_h, temp2
		add	temp1, YL
		adc	temp2, YH
		ldi	temp4, (1<<TOIE1)+(1<<TOIE0)
		out	TIMSK, temp4
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sbr	state0, (1<<OCT1_PENDING)
		ldi	temp4, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0) ; enable interrupt again
		out	TIMSK, temp4
		ldi	temp4, EXT0_EN		; ext0int enable
		out	GIMSK, temp4		; enable ext0int

	; calculate next waiting times - timing(-l-h-x) holds the time of 4 commutations
		lds	temp1, timing_l
		lds	temp2, timing_h
		lds	ZL, timing_x
		lsr	ZL			; build a quarter
		ror	temp2
		ror	temp1

		sts	zero_wt_l, temp1	; save for zero crossing timeout
		sts	zero_wt_h, temp2

		lsr	ZL
		ror	temp2
		ror	temp1
		lds	temp3, timing_l		; .. and subtract from timing
		lds	temp4, timing_h
		lds	ZL, timing_x
		sub	temp3, temp1
		sbc	temp4, temp2
		sbci	ZL, 0

		lds	temp1, tcnt1_sav_l	; calculate this commutation time
		lds	temp2, tcnt1_sav_h
		lds	YL, last_tcnt1_l
		lds	YH, last_tcnt1_h
		sts	last_tcnt1_l, temp1
		sts	last_tcnt1_h, temp2
		sub	temp1, YL
		sbc	temp2, YH
		sts	last_com_l, temp1
		sts	last_com_h, temp2

		add	temp3, temp1		; .. and add to timing
		adc	temp4, temp2
		ldi	temp2, 0
		adc	ZL, temp2

	; limit RPM to 120.000
		tst	ZL
		brne	update_t90
		tst	temp4
		breq	update_t10
		cpi	temp4, 1
		brne	update_t90
		cpi	temp3, 0x4c		; 0x14c = 120.000 RPM
		brcc	update_t90
	; set RPM to 120.000
update_t10:	ldi	temp4, 0x01
		ldi	temp3, 0x4c
		tst	run_control 
		brne	update_t90		; just active
		ldi	temp1, 0xff		; not active - reactivate
		mov	run_control, temp1

update_t90:	sts	timing_l, temp3
		sts	timing_h, temp4
		andi	ZL, 0x03		; limit range
		sts	timing_x, ZL

		ret
;-----bko-----------------------------------------------------------------
calc_next_timing:
		lds	YL, wt_comp_scan_l	; holds wait-before-scan value
		lds	YH, wt_comp_scan_h
		rcall	update_timing

		lsr	ZL			; a 16th is the next wait before scan
		ror	temp4
		ror	temp3
		lsr	ZL
		ror	temp4
		ror	temp3
		lsr	ZL
		ror	temp4
		ror	temp3
		lsr	ZL
		ror	temp4
		ror	temp3
		sts	wt_comp_scan_l, temp3
		sts	wt_comp_scan_h, temp4

	; use the same value for commutation timing (15°)
		sts	com_timing_l, temp3
		sts	com_timing_h, temp4
		ret

wait_OCT1_tot:	sbrc	state0, OCT1_PENDING
		rjmp	wait_OCT1_tot

set_OCT1_tot:
;@@		ldi	YH, high (timeoutMIN)
;@@		ldi	YL, low  (timeoutMIN)
		lds	YH, zero_wt_h
		lds	YL, zero_wt_l
		rcall	tcnt1_to_temp
		add	temp1, YL
		adc	temp2, YH
		ldi	temp4, (1<<TOIE1)+(1<<TOIE0)
		out	TIMSK, temp4
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sbr	state0, (1<<OCT1_PENDING)
		ldi	temp4, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0)
		out	TIMSK, temp4
		ldi	temp4, EXT0_EN		; ext0int enable
		out	GIMSK, temp4		; enable ext0int

		ret
;-----bko-----------------------------------------------------------------
wait_OCT1_before_switch:
		rcall	tcnt1_to_temp
		lds	YL, com_timing_l
		lds	YH, com_timing_h
		add	temp1, YL
		adc	temp2, YH
		ldi	temp3, (1<<TOIE1)+(1<<TOIE0)
		out	TIMSK, temp3
		out	OCR1AH, temp2
		out	OCR1AL, temp1
		sbr	state0, (1<<OCT1_PENDING)
		ldi	temp3, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0)
		out	TIMSK, temp3
		ldi	temp4, EXT0_EN		; ext0int enable
		out	GIMSK, temp4		; enable ext0int

	; don't waste time while waiting - do some controls, if indicated
.if RC_PULS == 1
		sbrc	state1, EVAL_RC_PULS
		rcall	evaluate_rc_puls
.endif
		sbrc	state1, EVAL_SYS_STATE
		rcall	evaluate_sys_state

.if UART_CONTROL == 1
		sbrc	state1, EVAL_UART
		rcall	evaluate_uart
.endif
OCT1_wait:	sbrc	state0, OCT1_PENDING
		rjmp	OCT1_wait
		ret
;-----bko-----------------------------------------------------------------
start_timeout:	lds	YL, wt_OCT1_tot_l
		lds	YH, wt_OCT1_tot_h
		rcall	update_timing

		in	temp1, TCNT1L
		andi	temp1, 0x0f
		sub	YH, temp1
		cpi	YH, high (timeoutMIN)
		brcc	set_tot2
		ldi	YH, high (timeoutSTART)		
set_tot2:
		sts	wt_OCT1_tot_h, YH

		rcall	sync_with_poweron	; wait at least 100+ microseconds
		rcall	sync_with_poweron	; for demagnetisation - one sync may be added

		ret
;-----bko-----------------------------------------------------------------
set_new_duty:	mov	temp1, ZH
		sub	temp1, sys_control
		brcc	set_new_duty10
		ldi	temp1, MIN_DUTY-1
set_new_duty10:	lds	temp2, timing_x
		tst	temp2
		brne	set_new_duty12
		lds	temp2, timing_h	; get actual RPM reference high
		cpi	temp2, PWR_RANGE1	; lower range1 ?
		brcs	set_new_duty20		; on carry - test next range
set_new_duty12:	sbr	state2, (1<<RPM_RANGE1)
		sbr	state2, (1<<RPM_RANGE2)
		ldi	temp2, PWR_MAX_RPM1	; higher than range1 power max ?
		cp	temp1, temp2
		brcs	set_new_duty31		; on carry - not higher, no restriction
		mov	temp1, temp2		; low (range1) RPM - set PWR_MAX_RPM1
		rjmp	set_new_duty31
set_new_duty20:	sbrs	state2, STARTUP
		rjmp	set_new_duty25
		ldi	temp3, PWR_MAX_STARTUP	; limit power in startup phase
		cp	temp1, temp3
		brcs	set_new_duty25		; on carry - not higher, test range 2
		mov	temp1, temp3		; set PWR_MAX_STARTUP limit
		rjmp	set_new_duty31
set_new_duty25:	cpi	temp2, PWR_RANGE2	; lower range2 ?
		brcs	set_new_duty30		; on carry - not lower, no restriction
		cbr	state2, (1<<RPM_RANGE1)
		sbr	state2, (1<<RPM_RANGE2)
		ldi	temp2, PWR_MAX_RPM2	; higher than range2 power max ?
		cp	temp1, temp2
		brcs	set_new_duty31		; on carry - not higher, no restriction
		mov	temp1, temp2		; low (range2) RPM - set PWR_MAX_RPM2
		rjmp	set_new_duty31
set_new_duty30:	cbr	state2, (1<<RPM_RANGE1)+(1<<RPM_RANGE2)
set_new_duty31:	com	temp1			; down-count to up-count (T0)
		mov	tcnt0_pwron_next, temp1	; save in next
	; tcnt0_power_on is updated to tcnt0_pwron_next in acceptable steps
		ret
;-----bko-----------------------------------------------------------------
.if UART_CONTROL == 1
read_uart:	tst	uart_cnt
		breq	read_uart_00
		rjmp	read_uart_99

read_uart_00:	clr	XH		; prepare sendbuffer
		ldi	XL, uart_data
		in	temp1,UDR	; read data - clear rxc-flag

		cpi	temp1,'+'
		brne	read_uart_10
	; do (+) job
		st	X+,temp1
		cpi	ZH, POWER_RANGE
		brsh	read_uart_01
		inc	ZH
		rcall	set_new_duty
read_uart_01:	mov	temp1, ZH
		rcall	hex2buf
		rjmp	read_uart_90
read_uart_10:	cpi	temp1,'-'
		brne	read_uart_20
	; do (-) job
		st	X+,temp1
		cpi	ZH, MIN_DUTY-1
		breq	read_uart_11
		dec	ZH
		rcall	set_new_duty
read_uart_11:	mov	temp1, ZH
		rcall	hex2buf
		rjmp	read_uart_90
read_uart_20:
		cpi	temp1,'#'
		brne	read_uart_30
	; do (#) job
		rcall	send_state
		rjmp	read_uart_90
read_uart_30:	cpi	temp1,'1'
		brne	read_uart_40
	; do (1) job
		st	X+,temp1
		lds	temp2, duty_offset
		ldi	ZH, MIN_DUTY
		add	ZH, temp2
		mov	temp1, ZH
		rcall	hex2buf
		rcall	set_new_duty
		rjmp	read_uart_90
read_uart_40:	cpi	temp1,'2'
		brne	read_uart_89
	; do (2) job
		st	X+,temp1
		ldi	ZH, MIN_DUTY-1
		rcall	set_new_duty
		rjmp	read_uart_90
read_uart_89:	ldi	temp1,'?'
		st	X+,temp1
read_uart_90:	subi	XL,uart_data		; build data count
		mov	uart_cnt,XL
		ldi	XL,uart_data		; point to first byte
		sbi	USR, TXC		; clear flag
		sbi	UCR, TXCIE		; enable irq
		ldi	temp1,'~'		; start transmission
		out	UDR,temp1
read_uart_99:	ret
;-----bko-----------------------------------------------------------------
send_state:	ldi	temp2, 0x0d
		st	X+, temp2
		ldi	temp2, 0x0a
		st	X+, temp2
		st	X+, temp1
		mov	temp1, state0
		rcall	hex2buf
		mov	temp1, state1
		rcall	hex2buf
		mov	temp1, state2
		rcall	hex2buf
		mov	temp1, tcnt0_power_on
		rcall	hex2buf
		ldi	temp1,':'
		st	X+, temp1
		lds	temp1, timing_x
		rcall	hex2buf
		lds	temp1, timing_h
		rcall	hex2buf
		lds	temp1, timing_l
		rcall	hex2buf
		ret
;-----bko-----------------------------------------------------------------
send_byte:	sbi	USR,TXC		; clear flag
		out	UDR,temp1
send_b20:	sbis	USR,TXC
		rjmp	send_b20
		ret
;-----bko-----------------------------------------------------------------
hex2buf:	mov	temp2,temp1
		swap	temp1
		rcall	nibble2buf
		mov	temp1,temp2
nibble2buf:	andi	temp1,0x0f
		ori	temp1,0x30
		cpi	temp1,0x3a
		brlo	nibble2buf_10
		subi	temp1,-7
nibble2buf_10:	st	X+,temp1
		ret
.endif		; UART_CONTROL == 1
;-----bko-----------------------------------------------------------------
switch_power_off:
		ldi	ZH, MIN_DUTY-1		; ZH is new_duty
		ldi	temp1, NO_POWER		; lowest tcnt0_power_on value
		mov	tcnt0_power_on, temp1
		mov	tcnt0_pwron_next, temp1
		ldi	temp1, INIT_PB		; all off
		out	PORTB, temp1
		ldi	temp1, CHANGE_TIMEOUT	; reset change-timeout
		mov	tcnt0_change_tot, temp1
		sbr	state1, (1<<POWER_OFF)	; disable power on
		cbr	state2, (1<<POFF_CYCLE)
		sbr	state2, (1<<STARTUP)
		ret				; motor is off
;-----bko-----------------------------------------------------------------
wait_if_spike:	ldi	temp1, 4
wait_if_spike2:	dec	temp1
		brne	wait_if_spike2
		ret
;-----bko-----------------------------------------------------------------
sync_with_poweron:
		sbrc	state0, I_OFF_CYCLE	; first wait for power on
		rjmp	sync_with_poweron
wait_for_poweroff:
		sbrs	state0, I_OFF_CYCLE	; now wait for power off
		rjmp	wait_for_poweroff
		lds	temp1, comp_state	; preload temp1
		ret

show_compa:
.if RC_PULS == 0
		sbrs	temp1, compA
		cbi	PORTD, rcp_in
		sbrc	temp1, compA
		sbi	PORTD, rcp_in
.endif
		ret
show_compb:
.if RC_PULS == 0
		sbrs	temp1, compB
		cbi	PORTD, rcp_in
		sbrc	temp1, compB
		sbi	PORTD, rcp_in
.endif
		ret
show_compc:
.if RC_PULS == 0
		sbrs	temp1, compC
		cbi	PORTD, rcp_in
		sbrc	temp1, compC
		sbi	PORTD, rcp_in
.endif
		ret
;-----bko-----------------------------------------------------------------
motor_brake:
.if MOT_BRAKE == 1
		ldi	temp2, 40		; 40 * 0.065ms = 2.6 sec
		ldi	temp1, BRAKE_PB		; all N-FETs on
		out	PORTB, temp1
mot_brk10:	sbrs	state0, T1OVFL_FLAG
		rjmp	mot_brk10
		cbr	state0, (1<<T1OVFL_FLAG)
.if RC_PULS == 1
		push	temp2
		rcall	evaluate_rc_puls
		pop	temp2
		cpi	ZH, MIN_DUTY+3		; avoid jitter detect
		brcs	mot_brk20
		rjmp	mot_brk90
mot_brk20:
.endif
		dec	temp2
		brne	mot_brk10
.if RC_PULS == 1
mot_brk90:
.endif
		ldi	temp1, INIT_PB		; all off
		out	PORTB, temp1
.endif	; MOT_BRAKE == 1
		ret
;-----bko-----------------------------------------------------------------
; **** startup loop ****

init_startup:	rcall	switch_power_off
		rcall	motor_brake
wait_for_power_on:
.if RC_PULS == 1
		rcall	evaluate_rc_puls
.endif
.if UART_CONTROL == 1
		sbic	USR, RXC
		rcall	read_uart
.endif
		cpi	ZH, MIN_DUTY
		brcs	wait_for_power_on

		ldi	temp1, PWR_STARTUP	; begin startup with low power
		com	temp1
		mov	tcnt0_pwron_next, temp1

; switch to start6
; state 6 = B(p-on) + A(n-choppered) - comparator C evaluated
		ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		cbr	state0, (1<<A_FET)	; next nFET = CnFET
		sbr	state0, (1<<C_FET)
		sbi	PORTB, BpFET		; Bp on
		sbrc	state1, FULL_POWER
		rjmp	s6_sw
		sbrc	state0, I_OFF_CYCLE	; was power off ?
		rjmp	s6_do			; .. yes - futhermore work is done in timer0 interrupt
s6_sw:		cbi	PORTB, AnFET		; An off
		sbrs	state1, POWER_OFF
		sbi	PORTB, CnFET		; Cn on
s6_do:		ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0) ; let timer0 do his work again
		out	TIMSK, temp1

		cbr	state2, (1<<SCAN_TIMEOUT)
		ldi	temp1, 0
		sts	goodies, temp1

		ldi	temp1, 40	; x 65msec
		mov	t1_timeout, temp1

		rcall	start_timeout
;-----bko-----------------------------------------------------------------
; **** start control loop ****

; state 1 = B(p-on) + C(n-choppered) - comparator A evaluated
; out_cA changes from low to high

start1:		sbrs	temp1, compA		; high ?
		rjmp	start1_2		; .. no - loop, while high

start1_0:	sbrc	state0, OCT1_PENDING
		rjmp	start1_1
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start1_9
start1_1:	rcall	sync_with_poweron
		rcall	show_compa

		sbrc	temp1, compA		; high ?
		rjmp	start1_0		; .. no - loop, while high

; do the special 120° switch
		ldi	temp1, 0
		sts	goodies, temp1
		rcall	com1com2
		rcall	com2com3
		rcall	com3com4
.if RC_PULS == 1
		rcall	evaluate_rc_puls
.endif
		rcall	evaluate_uart
		rcall	start_timeout
		rjmp	start4
	
start1_2:	sbrc	state0, OCT1_PENDING
		rjmp	start1_3
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start1_9
start1_3:	rcall	sync_with_poweron
		rcall	show_compa
		sbrs	temp1, compA		; high ?
		rjmp	start1_2		; .. no - loop, while low

start1_9:
		rcall	com1com2
		rcall	start_timeout

; state 2 = A(p-on) + C(n-choppered) - comparator B evaluated
; out_cB changes from high to low

start2:		sbrc	temp1, compB
		rjmp	start2_2

start2_0:	sbrc	state0, OCT1_PENDING
		rjmp	start2_1
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start2_9
start2_1:	rcall	sync_with_poweron
		rcall	show_compb
		sbrs	temp1, compB
		rjmp	start2_0
		rjmp	start2_9

start2_2:	sbrc	state0, OCT1_PENDING
		rjmp	start2_3
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start2_9
start2_3:	rcall	sync_with_poweron
		rcall	show_compb
		sbrc	temp1, compB
		rjmp	start2_2

start2_9:
		rcall	com2com3
.if RC_PULS == 1
		rcall	evaluate_rc_puls
.endif
		rcall	start_timeout

; state 3 = A(p-on) + B(n-choppered) - comparator C evaluated
; out_cC changes from low to high

start3:		sbrs	temp1, compC
		rjmp	start3_2

start3_0:	sbrc	state0, OCT1_PENDING
		rjmp	start3_1
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start3_9
start3_1:	rcall	sync_with_poweron
		rcall	show_compc
		sbrc	temp1, compC
		rjmp	start3_0
		rjmp	start3_9

start3_2:	sbrc	state0, OCT1_PENDING
		rjmp	start3_3
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start3_9
start3_3:	rcall	sync_with_poweron
		rcall	show_compc
		sbrs	temp1, compC
		rjmp	start3_2

start3_9:
		rcall	com3com4
		rcall	evaluate_uart
		rcall	start_timeout

; state 4 = C(p-on) + B(n-choppered) - comparator A evaluated
; out_cA changes from high to low

start4:		sbrc	temp1, compA
		rjmp	start4_2

start4_0:	sbrc	state0, OCT1_PENDING
		rjmp	start4_1
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start4_9
start4_1:	rcall	sync_with_poweron
		rcall	show_compa
		sbrs	temp1, compA
		rjmp	start4_0
		rjmp	start4_9

start4_2:	sbrc	state0, OCT1_PENDING
		rjmp	start4_3
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start4_9
start4_3:	rcall	sync_with_poweron
		rcall	show_compa
		sbrc	temp1, compA
		rjmp	start4_2

start4_9:
		rcall	com4com5
		rcall	start_timeout


; state 5 = C(p-on) + A(n-choppered) - comparator B evaluated
; out_cB changes from low to high


start5:		sbrs	temp1, compB
		rjmp	start5_2

start5_0:	sbrc	state0, OCT1_PENDING
		rjmp	start5_1
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start5_9
start5_1:	rcall	sync_with_poweron
		rcall	show_compb
		sbrc	temp1, compB
		rjmp	start5_0
		rjmp	start5_9

start5_2:	sbrc	state0, OCT1_PENDING
		rjmp	start5_3
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start5_9
start5_3:	rcall	sync_with_poweron
		rcall	show_compb
		sbrs	temp1, compB
		rjmp	start5_2

start5_9:
		rcall	com5com6
		rcall	evaluate_sys_state
		rcall	start_timeout

; state 6 = B(p-on) + A(n-choppered) - comparator C evaluated
; out_cC changes from high to low

start6:		sbrc	temp1, compC
		rjmp	start6_2

start6_0:	sbrc	state0, OCT1_PENDING
		rjmp	start6_1
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start6_9
start6_1:	rcall	sync_with_poweron
		rcall	show_compc
		sbrs	temp1, compC
		rjmp	start6_0
		rjmp	start6_9

start6_2:	sbrc	state0, OCT1_PENDING
		rjmp	start6_3
		sbr	state2, (1<<SCAN_TIMEOUT)
		rjmp	start6_9
start6_3:	rcall	sync_with_poweron
		rcall	show_compc
		sbrc	temp1, compC
		rjmp	start6_2

start6_9:
		rcall	com6com1

		mov	temp1, tcnt0_power_on
		cpi	temp1, NO_POWER
		brne	s6_power_ok
		rjmp	init_startup

s6_power_ok:	tst	rcpuls_timeout
		brne	s6_rcp_ok
		rjmp	restart_control

s6_rcp_ok:	tst	t1_timeout
		brne	s6_test_rpm
		rjmp	wait_for_power_on
;@@		rjmp	restart_control
		
s6_test_rpm:	lds	temp1, last_com_h
		cpi	temp1, 5		; ~2400 RPM ?
		brcs	s6_run1
		
s6_goodies:	lds	temp1, goodies
		sbrc	state2, SCAN_TIMEOUT
		clr	temp1
		inc	temp1
		sts	goodies,  temp1
		cbr	state2, (1<<SCAN_TIMEOUT)
		cpi	temp1, ENOUGH_GOODIES
		brcs	s6_start1	

s6_run1:	ldi	temp1, 0xff
		mov	run_control, temp1

		rcall	calc_next_timing
		rcall	set_OCT1_tot

		cbr	state2, (1<<STARTUP)
		rjmp	run1			; running state begins

s6_start1:	rcall	start_timeout		; need to be here for a correct temp1=comp_state
		rjmp	start1			; go back to state 1

;-----bko-----------------------------------------------------------------
; **** running control loop ****

; run 1 = B(p-on) + C(n-choppered) - comparator A evaluated
; out_cA changes from low to high

run1:		sbrc	state0, OCT1_PENDING
		rjmp	run1_1
		rjmp	run_to_start
run1_1:		sbis	PIND, compA		; high ?
		rjmp	run1			; .. no - loop, while low
		rcall	wait_if_spike		; .. yes - look for a spike
		sbis	PIND, compA		; test again
		rjmp	run1			; .. is low again, was a spike

		rcall	wait_OCT1_before_switch
		rcall	com1com2
		rcall	calc_next_timing
		rcall	wait_OCT1_tot
		sbr	state2, (1<<EVAL_UB)	; voltage evaluation request

; run 2 = A(p-on) + C(n-choppered) - comparator B evaluated
; out_cB changes from high to low

run2:		sbrc	state0, OCT1_PENDING
		rjmp	run2_1
		rjmp	run_to_start
run2_1:		sbic	PIND, compB		; low ?
		rjmp	run2			; .. no
		rcall	wait_if_spike
		sbic	PIND, compB		; low ?
		rjmp	run2			; high again

		sbr	state1, (1<<EVAL_RC_PULS)
		rcall	wait_OCT1_before_switch
		cbr	state2, (1<<EVAL_UB)	; voltage evaluation request
		rcall	com2com3
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 3 = A(p-on) + B(n-choppered) - comparator C evaluated
; out_cC changes from low to high

run3:		sbrc	state0, OCT1_PENDING
		rjmp	run3_1
		rjmp	run_to_start
run3_1:		sbis	PIND, compC
		rjmp	run3
		rcall	wait_if_spike
		sbis	PIND, compC
		rjmp	run3

		sbr	state1, (1<<EVAL_UART)
		rcall	wait_OCT1_before_switch
		rcall	com3com4
		rcall	calc_next_timing
		rcall	wait_OCT1_tot
		sbr	state1, (1<<EVAL_I_pFET) ; current evaluation request


; run 4 = C(p-on) + B(n-choppered) - comparator A evaluated
; out_cA changes from high to low
run4:		sbrc	state0, OCT1_PENDING
		rjmp	run4_1
		rjmp	run_to_start
run4_1:		sbic	PIND, compA
		rjmp	run4
		rcall	wait_if_spike
		sbic	PIND, compA
		rjmp	run4

		rcall	wait_OCT1_before_switch
		cbr	state1, (1<<EVAL_I_pFET) ; current evaluation request
		rcall	com4com5
		rcall	calc_next_timing
		rcall	wait_OCT1_tot


; run 5 = C(p-on) + A(n-choppered) - comparator B evaluated
; out_cB changes from low to high

run5:		sbrc	state0, OCT1_PENDING
		rjmp	run5_1
		rjmp	run_to_start
run5_1:		sbis	PIND, compB
		rjmp	run5
		rcall	wait_if_spike
		sbis	PIND, compB
		rjmp	run5

		sbr	state1, (1<<EVAL_SYS_STATE)
		rcall	wait_OCT1_before_switch
		rcall	com5com6
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

; run 6 = B(p-on) + A(n-choppered) - comparator C evaluated
; out_cC changes from high to low

run6:		sbrc	state0, OCT1_PENDING
		rjmp	run6_1
		rjmp	run_to_start
run6_1:		sbic	PIND, compC
		rjmp	run6
		rcall	wait_if_spike
		sbic	PIND, compC
		rjmp	run6

		rcall	wait_OCT1_before_switch
		rcall	com6com1
		rcall	calc_next_timing
		rcall	wait_OCT1_tot

		tst	rcpuls_timeout
		breq	restart_control

		lds	temp1, timing_x
		tst	temp1
		breq	run6_2			; higher than 610 RPM if zero
run_to_start:	sbr	state2, (1<<STARTUP)
		cbr	state2, (1<<POFF_CYCLE)
		rjmp	wait_for_power_on

run6_2:		cbr	state2, (1<<POFF_CYCLE)
		tst	run_control		; only once !
		breq	run6_9
		dec	run_control
		breq	run6_3			; poweroff if 0
		mov	temp1, run_control
		cpi	temp1, 1		; poweroff if 1
		breq	run6_3
		cpi	temp1, 2		; poweroff if 2
		brne	run6_9
run6_3:		sbr	state2, (1<<POFF_CYCLE)

run6_9:		rjmp	run1			; go back to run 1

restart_control:
		cli				; disable all interrupts
		rcall	switch_power_off
		rjmp	reset


;-----bko-----------------------------------------------------------------
; *** commutation utilities ***

com1com2:	cbi	PORTB, BpFET		; Bp off
		sbrs	state1, POWER_OFF
		sbi	PORTB, ApFET		; Ap on
		ret

com2com3:	ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		cbr	state0, (1<<A_FET)	; next nFET = BnFET
		cbr	state0, (1<<C_FET)
		sbrc	state1, FULL_POWER
		rjmp	c2_switch
		sbrc	state0, I_OFF_CYCLE	; was power off ?
		rjmp	c2_done			; .. yes - futhermore work is done in timer0 interrupt
c2_switch:	cbi	PORTB, CnFET		; Cn off
		sbrs	state1, POWER_OFF
		sbi	PORTB, BnFET		; Bn on
c2_done:	ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0) ; let timer0 do his work again
		out	TIMSK, temp1
		ret

com3com4:	cbi	PORTB, ApFET		; Ap off
		sbrs	state1, POWER_OFF
		sbi	PORTB, CpFET		; Cp on
		ret

com4com5:	ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		sbr	state0, (1<<A_FET)	; next nFET = AnFET
		cbr	state0, (1<<C_FET)
		sbrc	state1, FULL_POWER
		rjmp	c4_switch
		sbrc	state0, I_OFF_CYCLE	; was power off ?
		rjmp	c4_done			; .. yes - futhermore work is done in timer0 interrupt
c4_switch:	cbi	PORTB, BnFET		; Bn off
		sbrs	state1, POWER_OFF
		sbi	PORTB, AnFET		; An on
c4_done:	ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0) ; let timer0 do his work again
		out	TIMSK, temp1
		ret

com5com6:	cbi	PORTB, CpFET		; Cp off
		sbrs	state1, POWER_OFF
		sbi	PORTB, BpFET		; Bp on
		ret

com6com1:	ldi	temp1, (1<<OCIE1A)+(1<<TOIE1) ; stop timer0 interrupt
		out	TIMSK, temp1		;  .. only ONE should change these values at the time
		nop
		cbr	state0, (1<<A_FET)	; next nFET = CnFET
		sbr	state0, (1<<C_FET)
		sbrc	state1, FULL_POWER
		rjmp	c6_switch
		sbrc	state0, I_OFF_CYCLE	; was power off ?
		rjmp	c6_done			; .. yes - futhermore work is done in timer0 interrupt
c6_switch:	cbi	PORTB, AnFET		; An off
		sbrs	state1, POWER_OFF
		sbi	PORTB, CnFET		; Cn on
c6_done:	ldi	temp1, (1<<TOIE1)+(1<<OCIE1A)+(1<<TOIE0) ; let timer0 do his work again
		out	TIMSK, temp1
		ret
;-----bko-----------------------------------------------------------------
.exit
