;**** **** **** **** ****
; Motor connections
; power PCB top view, FET on bottom
; +---------------------------+
; |        control pins       |
; |   X  X  X  X  X  X  X  X  |
; |                           |
; |                           |
; |                           |
; |                           |
; |                           |
; | left=C  middle=B  right=A |
; |                           |
; +---------------------------+

;**** **** **** **** ****
;  comparator schematic
;**** **** **** **** ****
;
;**** **** **** **** ****
; overcurrent detect
;
; Uaccu ----|>|-----10k---+---2k2---GND
;         shottky         |
;          diode          |
;                         |
;                         |
;                         |
;                         |      |\                      
;                         |      |  \
;                         +------|-   \
;                             PB1|      \                 
;                                |        >---- 2313 internal ACO                 
;                                |      /                   
;                         +------|+   /                  
;                         |   PB0|  /                    
;                         |      |/                    
;                         |
; BEMF detect             |
;                         |
;                 GND     1
;                  |      0
;                  2      k
;                  k      | 
;                  2      |
;                  |      |
;                  +------+ signal of connector A is used
;                  |
;                  |
;                  |   |\
;                  |   |  \
;  C -----10k------+---|+   \
;                      |      \
;                      | LM339  >------ compA
;  B -----20k--+       |      /
;              +---+---|-   /
;  A -----20k--+   |   |  /
;                  |   |/
;                  |
;                  |
;                  |
;                  2
;                  k
;                  2
;                  |
;                 GND
;
;**** **** **** **** ****
; A (compA) and B (compB) are connected in the same way - w/o overcurrent detect
;
;
;**** **** **** **** ****
; low accu voltage detect - red+green LEDs = low power versions
; detect at 5.6V measured
;
;                 GND                 +5V
;                  |                   |
;                  |                   |
;                  1                   2
;                  0                   k
;                  k                   2
;                  |                   |
;                  |                   L
;                  |   |\              E red 
;                  |   |  \            D
;  Uaccu ---20k----+---|+   \          |
;                      |      \        |
;                      | LM339  >------+-------  compUB
;                      |      /
;  5V ------2k2----+---|-   /
;                  |   |  /
;                  |   |/
;                  |
;                  |
;                  |
;                  L
;                  E green (about 1.85V)
;                  D
;                  |
;                  |
;                 GND
;

;**** **** **** **** ****
; PORT D definitions
;
; PD6 C comparator input
; PD5 A comparator input
; PD4 B comparator input
;
; PD3 accu voltage control - also connected to red LED
; PD2 rc impulse input
;
; PD1 TxD config+debug serial i/o 38400baud
; PD0 RxD
;
;**** **** **** **** ****
; Port D pins

.equ	compC	= 6	;i comparator C signal
.equ	compA	= 5	;i comparator A signal
.equ	compB	= 4	;i comparator B signal
.equ	compUB	= 3	;i/o comparator accu voltage input / LED red output
.equ	redLED	= 3	;o LED red output
.equ	rcp_in	= 2	;i r/c pulse input
.equ	txd	= 1	;o uart tx
.equ	rxd	= 0	;i uart rx
.equ	INIT_PD	= 0x78	; enable pullups
.equ	DIR_PD	= 0x00


;**** **** **** **** ****
; PORT B definitions
; the FETs are switched on with port=high and off with port=GND
; PB7 A n-channel control (SCK)
; PB6 C n-channel control (MISO)
; PB5 B n-channel control (MOSI)
; PB4 B p-channel control
; PB3 A p-channel control
; PB2 C p-channel control
;
; PB1 (-) (UB minus shottky-voltage) with voltage divider (10k/2k2)
; PB0 (+) connected via 10k with (+) of C phase comparator
;
;**** **** **** **** ****
; Port B pins
;
.equ	AnFET	= 7	;o nFET + (mosi stk200 interface)
.equ	CnFET	= 6	;o nFET + (miso stk200 interface)
.equ	BnFET	= 5	;o nFET + (sck stk200 interface)
.equ	BpFET	= 4	;o pFET
.equ	ApFET	= 3	;o pFET
.equ	CpFET	= 2	;o pFET
.equ	motor_a	= 1	;i- motor A signal
.equ	ub_ref	= 0	;i+ accu voltage minus shottky voltage
.equ	INIT_PB	= 0x00
.equ	DIR_PB	= 0xfc
.equ	BRAKE_PB = (1<<AnFET)+(1<<BnFET)+(1<<CnFET)