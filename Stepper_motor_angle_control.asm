;Note that you have to know pinout diagram for atmega32a chip on google
;Define LCD  pins 
;A has the pins from 0-7 
;PORTA OUTPUT Registers for A 
;DDRA DATA Direction Register for A
;PINA Input Register A 
;LCD_DPRT is the pins on LCD D0-D7
.EQU LCD_DPRT = PORTA
.EQU LCD_DDDR = DDRA
.EQU LCD_DPIN = PINA

;Define control pins RS , RW , EN
;They may be input/output depenedin on mode operation
;Video for LCD
;https://www.youtube.com/watch?v=P6MO-n3MUAY
.EQU LCD_CPRT = PORTB
.EQU LCD_CDDR = DDRB
.EQU LCD_CPIN = PINB

;Pins definition of RS , RW , EN of the AVR
.EQU LCD_RS = 0
.EQU LCD_RW = 1
.EQU LCD_EN = 2

 ;Same for Keypad pins definition But for D
.EQU KEY_PORT = PORTD
.EQU KEY_PIN  = PIND
.EQU KEY_DDR  = DDRD
 
;same for Motor Pins C 
.EQU MTR_PRT = PORTC
.EQU MTR_DDR = DDRC
.EQU MTR_PIN = PINC

;we use only 0 , 1 , 6 , 7
;pins 2 - 5 reserved for debugger they cannot used only if activated
;https://www.avrfreaks.net/forum/problem-using-port-c-atmega32
.EQU MTR_IN1 = 0
.EQU MTR_IN2 = 1
.EQU MTR_IN3 = 6
.EQU MTR_IN4 = 7

; brginning at memory location 0x0000
; starting declaring variables
.org 0x00
 ANGLE_LOW  : .db 1    ; definition of ones of the angle  .db means reserve byte in memory it fills the lower byte by one 00
 ANGLE_MID  : .db 1    ; definition of tens of the angle initial value 1
 ANGLE_HIGH : .db 1    ; definition of hunderds of the angle initial value 1
 SPEED : .db 1         ; definition of speed full / half initial value 1
 STEP_NUMBER : .db  0  ; step number ranges from 0-3 depends on output to motor 1000 , 0100 , 0010 , 0001
 ERR : .db "ERROR"     ; definition of string consists of 5 characters (no need to end with \0 as C because we deal with assembly)
 https://www.studytonight.com/c/string-and-character-array.php 
Type_angle : .db "TYPE ANGLE A:FU-LL B:HALF: " ; definition of string of 27 characters 

;keypad matrix definition as arrays
KCODE0: .db '1','4','7','*' ;first column 
KCODE1: .db '2','5','8','0' ;second column 
KCODE2: .db '3','6','9','#' ;third column
KCODE3: .db 'A','B','C','D' ;fourth column

;Make the stack pointer refer to the last location of memory
LDI R21,HIGH(RAMEND)  ; last location high 
OUT SPH , R21         ; store at high byte stack pointer register
LDI R21,LOW(RAMEND)   ; last location low
OUT SPL , R21         ; store at high byte stack pointer register

;intialize LCD using LCD commands
START:                 ; Label define The actual start of the program
LDI R21 , 0xFF         ; load the general purpose register R20 with 0xFF all pins outputs
OUT LCD_DDDR , R21     ; OUT the value to the I/O Register DDRA
OUT LCD_CDDR , R21     ; OUT the same value to DDRB make all pins out note we need only pins 0,1,2
CBI LCD_DPRT,LCD_EN    ; clear bit in i/o register
CALL DELAY_2ms         ; call the delay 2 ms
LDI R16,0x38           ; Load register R16 , 0x38
CALL CMNDWRT           ; Call CMNDWRT which uses R16
CALL DELAY_2ms         ;.........
LDI R16 , 0x0E         ;.........
CALL CMNDWRT
LDI R16,0x01
CALL CMNDWRT
CALL DELAY_2ms
LDI R16,0x06
CALL CMNDWRT


;Type Data on LCD  
;Split the data into two data lines
/* note that R31 - R30 related to Z (16 - bit Register)*/
LDI R31, high(Type_angle << 1)  ; load R31 with high 1 byte of address
LDI R30, low(Type_angle  << 1)  ; load R31 with low 1 byte of address                       
LDI R17 , 16                    ; load counter with 16 first Row of Lcd
LOOP :                          ; begin loop to print letter by letter
LPM  R16, Z+                    ; move letter to R16  then increments to the next letter
CALL DATAWRT                    ; call Lcd data print to print this letter using R16
DEC R17                         ; loop counter decrment
BRNE LOOP                       ; Branch if Zero flag = 0 which means last decrement value not make the result 0

LDI R16, 0xC0                   ; Move to the next line
CALL CMNDWRT                    ; call cmnd write

LDI R17 , 11                    ; load counter with 11 Second Row of Lcd
LOOP2 :                         ; begin loop
LPM  R16, Z+                    ; .....
CALL DATAWRT                    ; .....
DEC R17                         ;.....
BRNE LOOP2                      ;.....

/*begin of Keypad interfacing */
;https://www.youtube.com/watch?v=bNOVg9vwFRM

LDI R20,0xF0                    ;load register R20 with 0xF0
OUT KEY_DDR , R20               ;output R20 into KEY_DDR DDRD making the highest 4 pins output and lowest 4 pins input
LDI R17 , 4                     ;counter for numbers of characters inputted 
                                ;123A angle 123 degrees and A full speed 

 GROUND_ALL_ROWS:               ; Label for start Reading from Keypad
 LDI R20, 0x0F                  ; load R20 with 0x0F
 OUT KEY_PORT , R20             ; output R20 to KEY_PORT
 /*This process of making the input pins high without making them output called pull up the pins which , refer 
 to this link https://electronics.stackexchange.com/questions/185953/what-is-the-purpose-of-a-pull-up-resistor-in-a-microcontroller */

 ; wait untill all pins is high which means no pin is pressed

 WAIT_FOR_RELEASE:
 NOP                 ;no operation ( clock cycle with no instruction)
 IN R21,KEY_PIN      ; read the input
 ANDI R21,0x0F       ; and input to the pin with 0x0F R21 = R21 & 0x0F
 CPI R21,0x0F        ; compare R21 with 0x0F Subtract
 BRNE WAIT_FOR_RELEASE ; Branch if not equal

 /* Assume if PIND = 0b00001001
 NOP -> no operation 
 IN R21,KEY_PIN -> R21 = 0b01001001  note the low 4 pins are the input pins
 ANDI R21,0x0F  -> R21 = 0b00001001
 CPI R21 , 0x0F -> Z (zero flag) != 0
 BRNE WAIT_FOR_RELEASE -> go to label WAIT_FOR_RELEASE*/
 ; it only will proceed in th eprogram if KEY_PIN = 0bxxxx1111

 WAIT_FOR_KEY:
 NOP
 IN R21,KEY_PIN
 ANDI R21,0x0F
 CPI R21,0x0F          ; R21 - 0x0F if equal zero zero flag will be zero
 BREQ WAIT_FOR_KEY     ; Branch if Z(zero flag) = 0
 CALL WAIT20MS         ; wait for 20ms then read the key again
 IN R21,KEY_PIN       
 ANDI R21,0x0F
 CPI R21,0x0F
 BREQ WAIT_FOR_KEY

 ;Start comparing the results 
 LDI R21,0b01111111   ;load R21 with 0b01111111 binary representation
 OUT KEY_PORT,R21     ;output to the KEY_PORT 
 NOP                  
 IN R21,KEY_PIN       ; Read the value
 ANDI R21,0x0F        ; 
 CPI R21,0x0F         ;R21 - 0x0F
 BRNE COL1            ;Branch if Z != 0
 LDI R21,0b10111111
 OUT KEY_PORT,R21
 NOP
 IN R21,KEY_PIN
 ANDI R21,0x0F
 CPI R21,0x0F
 BRNE COL2
 LDI R21,0b11011111
 OUT KEY_PORT,R21
 NOP
 IN R21,KEY_PIN
 ANDI R21,0x0F
 CPI R21,0x0F
 BRNE COL3
 LDI R21,0b11101111
 OUT KEY_PORT,R21
 NOP
 IN R21,KEY_PIN
 ANDI R21,0x0F
 CPI R21,0x0F
 BRNE COL4

 COL1:              ; means you ae a column one 
 LDI R30,LOW(KCODE0 << 1) ; loading te first column in the R30 - R31 
 LDI R31,HIGH(KCODE0 << 1) ;....................
 RJMP FIND                    ; jmp to find the specific key pressed
 COL2:
 LDI R30,LOW(KCODE1 << 1)
 LDI R31,HIGH(KCODE1 << 1)
 RJMP FIND
 COL3:
 LDI R30,LOW(KCODE2 << 1)
 LDI R31,HIGH(KCODE2 << 1)
 RJMP FIND
 COL4:
 LDI R30,LOW(KCODE3 << 1)
 LDI R31,HIGH(KCODE3 << 1)
 RJMP FIND
 FIND:  
 ;https://books.google.com.eg/books?id=rj9dDwAAQBAJ&pg=PA204&lpg=PA204&dq=carry+after+logical+shift+microprocessor&source=bl&ots=nLOy5Pv4pd&sig=ACfU3U2jB3HtnSL5TO5JXPtfwShyC2J10g&hl=ar&sa=X&ved=2ahUKEwiPrpuWvJfmAhV1o3EKHQ0GD3QQ6AEwC3oECAoQAQ
  LSR R21        ;logical shift right R21 is input KEY_PIN when shifted right
                 ; carry is the bit b0
  BRCC MATCH     ; branch if Carry cleared   
  LPM R20,Z+     ; load R20 with the next charater in column
  RJMP FIND      ; jmp to the label FIND
 
 MATCH:          
 LPM R16,Z       ; load R16 with charcter
 CALL DATAWRT    ; print on lcd
 SUBI R16 , 0x30  ; get the absolute number R16 - 0x30
                  ; on in ASCII 0x31
				  ;to get one as integer subtract 0x31 - 0x30 = 0x01
 CPI R16 , 0x0A   ;compare if equal or greater than 10
 BRGE out0        ; branch it is not a number
 CPI R16, 0x00    ; compare if less than 0
 BRLO out0        ; Branch if not a number
 CPI R17 , 1     ; compare if we at the last digit
 BREQ out0       ; branch if Z = 1
 CPI R17 , 3      ; compare the counter what digit
 BREQ L2          ; branch L2 if dighit 1(2)3A
 CPI R17 , 2      ; compare 
 BREQ L1          ; branch L1 if dighit 12(3)A
 /* Proceed */


 STS ANGLE_HIGH  , R16  ; stores the hunderds digit
 RJMP OUT1              ; jump to label OUT1
 
 L2:
 STS ANGLE_MID , R16    ; stores the tens digit
 RJMP OUT1              ; jump to label OUT1
 
 L1: 
 STS ANGLE_LOW  , R16   ; stores the ones digit
 RJMP OUT1              ; jump to label OUT1

 /* ERROR at the input data detection and speed storing */
 out0 :                 ; not that label branch if input char is less than zero or greater than 
 CPI R17 , 1            ; compare if we at the last digit 
 BREQ ok                 ; branch if ok
 CALL ERROR             ; error if not ok
 RJMP START             ; go to the beginning

 ok:
 CPI R16 , 17          ; compare if R16 = 17 ( note that R16 = fourth digit - 0x30  'A' 0x30 = 17)
 BREQ FULL_SPEED       ; Branch to full speed if 'A'
 CPI R16 , 18
 BREQ HALF_SPEED
 CALL ERROR            ; call Error if not A or B
 RJMP START            ; Jump to the beginning
 FULL_SPEED :          
 LDI R20, 1
 STS   SPEED , R20     ; store speed = 1 to SPEED
 RJMP OUT1 
 HALF_SPEED :
 LDI R20 , 2
 STS SPEED , R20       ; Store Speed = 2 to speed
 OUT1:
 DEC R17               ; decrement counter 
 BREQ HERE             ; if counter = 0 branch to here (start move the motor)
 RJMP GROUND_ALL_ROWS  ; Read the second digit


HERE:                 
 LDI R20 , 0xC3        ; make pin 11000011 output of motor
 OUT MTR_DDR , R20     

 /*  Another Error checking 
 check whether the inputted angle greater than 360
 */
 lds R26 ,  ANGLE_HIGH  ;load the highest digit
 lds R27 ,  ANGLE_MID   ;load the mid digit
 lds R28 ,  ANGLE_LOW   ;load the first digit
 /* suppose angle 
   399
 - 361
 -----
   038
eirst subtract 9 -1 = 8
second subtract 9 - 6 = 3
third subtract 3 - 3 =0
 S (sign flag ) = 0
 which means the number is greater
 which means ERROR
 */
 subi R28 , 1           
 sbci R27 , 6
 sbci R26 , 3
 BRGE THERE_T_JMP  ; note that we cannot jump directly to There because it is out of reach to branch so we use RJMP
 jmp DIGIT1
  THERE_T_JMP:
  CALL ERROR
   RJMP THERE

  /* start moving the motor hundreds then tens then ones*/
  /* 199
     ---
	 100 + 90 +9
	 -----------
	 100*5.7 +90*5.7 +9*5.7
	 -----------------------
	 570   then 513 then ~ 51
	 */
; Motor Tutorial
;https://www.youtube.com/watch?v=avrdDZD7qEQ
;code In C https://github.com/NikodemBartnik/ArduinoTutorials/blob/master/28BYJ-48/28BYJ-48.ino

;first hundreds not if hunderds equal to zero it will step this part because it won't find a match
 DIGIT1:   
 LDS R25 ,  ANGLE_HIGH   ; load R25 with the highest digit hunderds
 CPI R25 , 1             ; compare if = 1 
 BREQ DIGIT3100          ; branch to DIGIT3100
 CPI R25 , 2
 BREQ DIGIT3200
 CPI R25 , 3
 BREQ DIGIT3300
 JMP DIGIT2
 DIGIT3100:
 LDI R31 , HIGH(570)   ; load R31-R30 with the number 570
 LDI R30 , LOW(570)
 jmp OUT5
 DIGIT3200:
 LDI R31 , HIGH(1140)
 LDI R30 , LOW(1140)
 JMP OUT5
 DIGIT3300:
 LDI R31 , HIGH(1710)
 LDI R30 , LOW(1710)
 OUT5:
 TWO2:  
 CALL ONE_STEP       ; step one step from 2048 step
 lds R25 ,  SPEED    ; load R25 with speed  1 or 0
 cpi R25 , 1         ; compare with 1 
 BREQ Delay2_1       ; branch if equal 1
 CALL DELAY_4ms      ; if other make the delay larger 4ms
 jmp Delayout2_1     ; go to Delayout2_1
 Delay2_1:
 cALL DELAY_2ms      ; delay less 2ms
 Delayout2_1:
 SBIW Z , 1
 BRNE TWO2          ; go to the tens
 
 DIGIT2:
 LDS R25 , ANGLE_MID
 CPI R25 , 1
 BREQ DIGIT210
 CPI R25 , 2
 BREQ DIGIT220
 CPI R25 , 3
 BREQ DIGIT230
 CPI R25 , 4
 BREQ DIGIT240
 CPI R25 , 5
 BREQ DIGIT250
 CPI R25 , 6
 BREQ DIGIT260
 CPI R25 , 7
 BREQ DIGIT270
 CPI R25 , 8
 BREQ DIGIT280
 CPI R25 , 9
 BREQ DIGIT290
 JMP DIGIT3   ; end 
 DIGIT210:
 LDI R31 , HIGH(57)
 LDI R30 , LOW(57)
 jmp OUT6
 DIGIT220:
 LDI R31 , HIGH(114)
 LDI R30 , LOW(114)
 JMP OUT6
 DIGIT230:
 LDI R31 , HIGH(171)
 LDI R30 , LOW(171)
 JMP OUT6
 DIGIT240:
 LDI R31 , HIGH(228)
 LDI R30 , LOW(228)
 JMP OUT6
 DIGIT250:
 LDI R31 , HIGH(285)
 LDI R30 , LOW(285)
 JMP OUT6
 DIGIT260:
 LDI R31 , HIGH(342)
 LDI R30 , LOW(342)
 JMP OUT6
  DIGIT270:
 LDI R31 , HIGH(399)
 LDI R30 , LOW(399)
  JMP OUT6
 DIGIT280:
 LDI R31 , HIGH(456)
 LDI R30 , LOW(456)
 JMP OUT6
 DIGIT290:
 LDI R31 , HIGH(513)
 LDI R30 , LOW(513)
OUT6:
 TWO1:  
 CALL ONE_STEP
 lds R25 ,  SPEED 
 cpi R25 , 1
 BREQ Delay2_2
 CALL DELAY_4ms
 jmp Delayout2_2
 Delay2_2:
 CALL DELAY_2ms
 Delayout2_2:
 SBIW Z , 1
 BRNE TWO1

  DIGIT3:
 LDS R25 , ANGLE_LOW 
  CPI R25 , 1
 BREQ DIGIT31
 CPI R25 , 2
 BREQ DIGIT32
 CPI R25 , 3
 BREQ DIGIT33
 CPI R25 , 4
 BREQ DIGIT34
 CPI R25 , 5
 BREQ DIGIT35
 CPI R25 , 6
 BREQ DIGIT36
 CPI R25 , 7
 BREQ DIGIT37
 CPI R25 , 8
 BREQ DIGIT38
 CPI R25 , 9
 BREQ DIGIT39
 JMP THERE   ; end 
 DIGIT31:
 LDI R17 , 6
 JMP OUT7
 DIGIT32:
 LDI R17 , 11
 JMP OUT7
 DIGIT33:
 LDI R17 , 17
 JMP OUT7
 DIGIT34:
 LDI R17 , 23
 JMP OUT7
 DIGIT35:
 LDI R17 , 29
 JMP OUT7
 DIGIT36:
 LDI R17 , 34
 JMP OUT7
 DIGIT37:
 LDI R17 , 40
 JMP OUT7
 DIGIT38:
 LDI R17 , 46
 JMP OUT7
 DIGIT39:
 LDI R17 , 51
 JMP OUT7
 OUT7:
 TWO3:  
 CALL ONE_STEP
 lds R25 ,  SPEED 
 cpi R25 , 1
 BREQ Delay2_3
 CALL DELAY_4ms
 jmp Delayout2_3
 Delay2_3:
 cALL DELAY_2ms
 Delayout2_3:
 SUBI R17 , 1
 BRNE TWO3


 THERE : JMP START  ; jmp to the start of program

 ERROR :
LDI R31, high(ERR  << 1)  
LDI R30, low(ERR << 1) 
push R17
LDI R17 , 5
LDI R16, 0xCB   ; got to position 11 on Row 2 of lcd
CALL CMNDWRT
CALL DELAY_100us
LOOP0 :
LPM  R16, Z+  ; type letter then increments to the next letter
CALL DATAWRT
DEC R17
BRNE LOOP0
POP R17
PUSH R17
LDI R17,200
LDR3 : CALL DELAY_2ms
DEC R17
BRNE LDR3
POP R17
RET
CMNDWRT:
OUT LCD_DPRT , R16
CBI LCD_CPRT , LCD_RS
CBI LCD_CPRT , LCD_RW
SBI LCD_CPRT , LCD_EN
CALL SDELAY
CBI LCD_CPRT ,LCD_EN
CALL DELAY_100us
RET

DATAWRT:
OUT LCD_DPRT , R16
SBI LCD_CPRT , LCD_RS
CBI LCD_CPRT , LCD_RW
SBI LCD_CPRT , LCD_EN
CALL SDELAY
CBI LCD_CPRT ,LCD_EN
CALL DELAY_100us
RET

;DELAY PROCEDURE 
;Note that each clock cycle is equal to 1/1000000(frequecny) = 1us
;push 2 clock cycles 2us
;LDI 1 clock cycle 1us
;CAll 4 clock cycles 4us
;DEC 1 clock cycle 1us
;BRNE 2 clock cycles if branch 1 clock cycles if not branch 2us , 1us
;POP 1 clock cycle 1us
;RET 4 clock 4us
DELAY_100us:
PUSH R17
LDI R17,7
DR0: CALL SDELAY 
DEC R17
BRNE DR0
POP R17
RET
;nop 1 clock 1us
SDELAY :
NOP 
RET
; to clalculate the period of the delay
; 1*CALL + 1* PUSH + 1*LDI + X * CALL + X * NOP + X*RET +X *DEC + (X-1) * BRNE(Branch) + 1 * BRNE(not branch) + 1*POP + 1 * RET = 100us
; 1* 4   + 1* 2    + 1* 1  + X *  4   + X * 1   + X *4  + X * 1 + (X -1 ) * 2  + 1*1   + 1 * 1 + 1 * 2 + 1 * 4 = 100 clocks
; X (number of loops) ~ 7 



DELAY_4ms:
PUSH R17
LDI R17, 40
LDR6 : CALL DELAY_100us
DEC R17
BRNE LDR6
POP R17
RET

DELAY_2ms:
PUSH R17
LDI R17, 20
LDR4 : CALL DELAY_100us
DEC R17
BRNE LDR4
POP R17
RET

WAIT20MS:
PUSH R17
LDI R17,10
LDR2 : CALL DELAY_2ms
DEC R17
BRNE LDR2
POP R17
RET

; motor stepping by one step
ONE_STEP:
LDS R20,( STEP_NUMBER << 1)   ; load the step number
SUBI R20,1                    ; compare by subtracting R20 = R20 -1
BREQ FUN1                     ; branch if Z = 1
LDS R20 ,( STEP_NUMBER << 1)  
SUBI R20,2
BREQ FUN2
LDS R20, ( STEP_NUMBER << 1)
SUBI R20,3
BREQ FUN3
SBI MTR_PRT,MTR_IN1
CBI MTR_PRT,MTR_IN2
CBI MTR_PRT,MTR_IN3
CBI MTR_PRT,MTR_IN4
JMP END
FUN1:
CBI MTR_PRT,MTR_IN1
SBI MTR_PRT,MTR_IN2
CBI MTR_PRT,MTR_IN3
CBI MTR_PRT,MTR_IN4
JMP END
FUN2:
CBI MTR_PRT,MTR_IN1
CBI MTR_PRT,MTR_IN2
SBI MTR_PRT,MTR_IN3
CBI MTR_PRT,MTR_IN4
JMP END
FUN3:
CBI MTR_PRT,MTR_IN1
CBI MTR_PRT,MTR_IN2
CBI MTR_PRT,MTR_IN3
SBI MTR_PRT,MTR_IN4
END:
;https://www.avrfreaks.net/forum/newbie-variables-assembler
LDS R20,( STEP_NUMBER << 1)              
INC R20
STS  ( STEP_NUMBER << 1), R20

CPI R20 , 4
BRNE RET1
LDI R20,0
STS  ( STEP_NUMBER << 1), R20
RET1:
RET


