//Питание 3V CR2032
//Tiny2313A 128kHz
//DS18B20 принудительно переводится в режим 9 бит
//FUSE LOW 0xC6, HIGH 0xDF

/* 				Port Pin Configurations

DDR 	PORT 		PUD 	I/O 		Pull-up 	Comment

0 		0 			X 		Input 		No 			Tri-state (Hi-Z)
0 		1 			0 		Input 		Yes 		Pxn will source current if ext. pulled low.
0 		1 			1 		Input 		No 			Tri-state (Hi-Z)

1 		0 			X 		Output 		No 			Output Low (Sink)
1 		1 			X 		Output 		No 			Output High (Source)

*/

.include  "C:\Program Files (x86)\Atmel\AVR Tools\AvrAssembler2\Appnotes\tn2313Adef.inc"

;-----MACRO-----
	.macro ds18b20_on
		sbi PORTB, PB4
	.endm

	.macro ds18b20_off
		cbi PORTB, PB4
	.endm

	.macro dq_input
		cbi DDRB, PB3
	.endm

	.macro dq_output
		sbi DDRB, PB3
	.endm

	.macro dq_hi
		sbi PORTB, PB3
	.endm

	.macro dq_lo
		cbi PORTB, PB3		
	.endm
;---------------


;---CONSTANTS---
	.equ CRC_GENERATING_POLYNOM = 0x8C

	.equ SKIP_ROM = 0xCC
	.equ CONVERT_T = 0x44

	.equ READ_SCRATCHPAD = 0xBE
	.equ WRITE_SCRATCHPAD = 0x4E
	.equ COPY_SCRATCHPAD = 0x48

	.equ CONFIG_REG_9BIT = 0x1F
;---------------


;---VARIABLES---
	.def count_bytes = R0
	.def count_bits = R1
	.def crc_polynom = R2

	.def fract = R15
	.def temp = R16	

	// R17-R19 - delay routines

	.def ds_byte = R20
	.def i = R21

	.def _r1 = R22 //левый (старший) семисегментник
	.def _r2 = R23 
	.def _r3 = R24 //правый (младший) семисегментник

	.def blink = R25	

	// R26-R27 - XH:XL

	.def CRC = R28 		//crc
	.def CRC_next = R29 //crc next byte
	.def Tlsb = R30		//temp 
	.def Tmsb = R31		//temp 


;---------------


	.cseg
	.org 0


;---INERRUPTS---
 rjmp START ; Reset Handler
	reti ; External Interrupt0 Handler
	reti ; External Interrupt1 Handler
	reti ; Timer1 Capture Handler
	reti ; Timer1 CompareA Handler
	reti ; Timer1 Overflow Handler
	reti ; Timer0 Overflow Handler
	reti ; USART0 RX Complete Handler
	reti ; USART0,UDR Empty Handler
	reti ; USART0 TX Complete Handler
	reti ; Analog Comparator Handler
	reti ; PCINT0 Handler
	reti ; Timer1 Compare B Handler
	reti ; Timer0 Compare A Handler
	reti ; Timer0 Compare B Handler
	reti ; USI Start Handler
	reti ; USI Overflow Handler
	reti ; EEPROM Ready Handler
	reti ; Watchdog Overflow Handler
	reti ; PCINT1 Handler
	reti ; PCINT2 Handler
;---------------


;###########################################################
START:
	ldi temp, low(RAMEND) ; инициализация
	out SPL, temp         ; стека


	ldi temp, (1<<PRTIM1)|(1<<PRTIM0)|(1<<PRUSI)|(1<<PRUSART)	//меньше жрем
	out PRR, temp

	ldi temp, (1<<ACD)											//еще меньше жрем
	out ACSR, temp
	

	ldi temp, (0<<DDB7)|(0<<DDB6)|(0<<DDB5)|(1<<DDB4)|(0<<DDB3)|(0<<DDB2)|(0<<DDB1)|(1<<DDB0)	;i/o direction
	out DDRB, temp


	ldi temp, (1<<PB7)|(1<<PB6)|(1<<PB5)|(1<<PB4)|(0<<PB3)|(1<<PB2)|(1<<PB1)|(0<<PB0)			;level/pull-up
	out PORTB, temp


	ldi temp, (1<<DDD6)|(1<<DDD5)|(1<<DDD4)|(1<<DDD3)|(1<<DDD2)|(1<<DDD1)|(1<<DDD0)		;i/o direction
	out DDRD, temp


	ldi temp,  (0<<PD6)|(0<<PD5)|(0<<PD4)|(0<<PD3)|(0<<PD2)|(0<<PD1)|(0<<PD0)			;level/pull-up
	out PORTD, temp


	ldi temp, (0<<DDA2)|(1<<DDA1)|(1<<DDA0)			;i/o direction
	out DDRA, temp


	ldi temp,  (1<<PA2)|(0<<PA1)|(0<<PA0)			;level/pull-up
	out PORTA, temp


/*
  АЛГОРИТМ

  Инициализация
  Микроконтроллер записывает команду skip rom
  Микроконтроллер записывает команду convert t
  Микроконтроллер формирует тайм слоты чтения и ожидает единицу

  Инициализация
  Микроконтроллер записывает команду skip rom
  Микроконтроллер записывает команду read scratchpad
  Микроконтроллер считывает 9 байт ОЗУ датчика
*/


; ============================= Замер температуры



	rcall _ds_reset	
	rcall _ds_reset				//reset


	brtc _ds_not_connected		//проверка подключения датчика
	rjmp _continue
_ds_not_connected:
	ds18b20_off
	dq_output
	dq_lo
    rcall _error
	rjmp _loop


_continue:
	ldi ds_byte, SKIP_ROM		//skip rom
	rcall _ds_write_byte


	ldi ds_byte, CONVERT_T		//convert T
	rcall _ds_write_byte


_ds_wait_convertation:
	rcall _ds_read_bit
	brsh _ds_wait_convertation


	rcall _ds_reset				//reset


	ldi ds_byte, SKIP_ROM		//skip rom
	rcall _ds_write_byte


	ldi ds_byte, READ_SCRATCHPAD		//read scratchpad
	rcall _ds_write_byte

	rcall _copy_scratchpad_to_ram

	lds Tlsb, SRAM_START				//save temp
	lds Tmsb, SRAM_START + 1

; ============================= Подсчет контрольной суммы

	rcall _calc_crc

	cpi CRC, 0 
	brne _crc_error
	rjmp _crc_ok

_crc_error:
	ds18b20_off
	dq_output
	dq_lo
	rcall _error
	rjmp _loop

_crc_ok:
	lds temp, SRAM_START + 4		//check is 9bit?
	cpi temp, CONFIG_REG_9BIT
	breq _skip_forced_9bit
		


; ============================= Перевод в 9 бит
	rcall _ds_reset			//reset

	ldi ds_byte, SKIP_ROM		//skip rom
	rcall _ds_write_byte	

	ldi ds_byte, WRITE_SCRATCHPAD		//write scratchpad
	rcall _ds_write_byte


	ldi ds_byte, 0x00
	rcall _ds_write_byte
	rcall _ds_write_byte

	ldi ds_byte, CONFIG_REG_9BIT
	rcall _ds_write_byte		


	rcall _ds_reset			//reset

	ldi ds_byte, SKIP_ROM		//skip rom
	rcall _ds_write_byte

	ldi ds_byte, READ_SCRATCHPAD		//read scratchpad
	rcall _ds_write_byte


	rcall _copy_scratchpad_to_ram


////////////////////////
	rcall _calc_crc

	cpi CRC, 0 
	brne _crc_error2
	rjmp _crc_ok2

_crc_error2:
	ds18b20_off
	dq_output
	dq_lo
	rcall _error
	rjmp _loop

_crc_ok2:

	rcall _ds_reset			//reset

	ldi ds_byte, SKIP_ROM		//skip rom
	rcall _ds_write_byte

	ldi ds_byte, COPY_SCRATCHPAD		//copy scratchpad
	rcall _ds_write_byte	

	rcall _error					//после смены на 9 бит весело мигнем 9 раз
	rcall _error
	rcall _error


_skip_forced_9bit:	

; ============================= Выключаем датчик, он нам уже не нужен
	ds18b20_off
	dq_output
	dq_lo


; ============================= Перевод температуры в читаемый вид и округление с точностью +-0,5



	mov fract, Tlsb 	//сохраняем дробную часть
	ldi temp, 0x0F
	and fract, temp


	andi Tlsb, 0xF0 	//выделяем значение температуры по модулю
	andi Tmsb, 0x0F
	or Tlsb, Tmsb
	swap Tlsb

	mov temp, Tlsb 		//проверка температуры на отрицательность
	andi temp, 0x80
	brne _negative_temperature
	ldi Tmsb, 0x00 		//знак температуры +

	mov temp, fract
	cpi temp, 0x08
	brlo _bcd 			//если дробная часть меньше чем 0,5 то отбрасываем дробную часть
	inc Tlsb 			//если дробная часть больше или равно чем 0,5 то округляем вверх до целого
	rjmp _bcd

_negative_temperature:
	ldi Tmsb, 0xFF 		//знак температуры -

	mov temp, fract
	cpi temp, 0x08 		//сравниваем дробную часть с 0,5
	brlo _05 			//если дробная часть меньше чем 0,5 то переводим в обратный код и инкрементируем
	com Tlsb 			//если дробная часть больше или равно чем 0,5 то просто переводим в обратный код

	//проверка на отрицательный ноль
	cpi Tlsb, 0
	brne _bcd
	ldi Tmsb, 0x00 		//знак температуры +, это положительный ноль

	rjmp _bcd

_05:
	com Tlsb
	inc Tlsb
				
; =============================	Разбиение на десятичные разряды

_bcd:
	push Tlsb
	ldi _r1, 0
	ldi _r2, 0
	ldi _r3, 0

_100:
	cpi Tlsb, 100
	brlo _10
	subi Tlsb, 100
	inc _r1
	rjmp _100

_10:
	cpi Tlsb, 10
	brlo _1
	subi Tlsb, 10
	inc _r2
	rjmp _10

_1:
	cpi Tlsb, 1
	brlo _end
	subi Tlsb, 1
	inc _r3
	rjmp _1

_end:
	pop Tlsb


; =============================	Добавление знаков "минус"/"градус" и исключение незначащих нулей

	cpi Tlsb, 10
	brsh _to_100
	cpi Tmsb, 0
	brne _neg_10
//положительная температура до 10 град
	mov blink, _r3
	rcall do_blink
	rcall _pause_off
	rjmp _loop
_neg_10:
//отрицательная температура до -10 град
	ldi blink, 0
	rcall do_blink
	rcall _pause_off
	mov blink, _r3
	rcall do_blink
	rjmp _loop

_to_100:
	cpi Tlsb, 100
	brsh _to_128	
	cpi Tmsb, 0
	brne _neg_100
//положительная температура до 100 град
	mov blink, _r2
	rcall do_blink
	rcall _pause_off
	mov blink, _r3
	rcall do_blink
	rjmp _loop
_neg_100:
//отрицательная температура до -55 град
	ldi blink, 0
	rcall do_blink
	rcall _pause_off
	mov blink, _r2
	rcall do_blink
	rcall _pause_off
	mov blink, _r3
	rcall do_blink
	rjmp _loop
//положительная температура до 128 град
_to_128:
	mov blink, _r1
	rcall do_blink
	rcall _pause_off
	mov blink, _r2
	rcall do_blink
	rcall _pause_off
	mov blink, _r3
	rcall do_blink



_loop:

; =============================	Сон

	ldi temp, (1<<SE)|(0<<SM1)|(1<<SM0)
	out MCUCR, temp
	sleep


_endloop:
	rjmp _endloop



;------------------------------------------------------------------------------
; Мигание
do_blink:
	
	cpi blink, 0
	brne _to_1
	sbi PORTA, PA0
	rcall _pause_on
	cbi PORTA, PA0
	rjmp _end_blink

_to_1:
	cpi blink, 1
	brne _to_2
	sbi PORTD, PD6
	rcall _pause_on
	cbi PORTD, PD6
	rjmp _end_blink

_to_2:	
	cpi blink, 2
	brne _to_3
	sbi PORTD, PD4
	rcall _pause_on
	cbi PORTD, PD4
	rjmp _end_blink

_to_3:
	cpi blink, 3
	brne _to_4
	sbi PORTA, PA1
	rcall _pause_on
	cbi PORTA, PA1
	rjmp _end_blink

_to_4:
	cpi blink, 4
	brne _to_5
	sbi PORTB, PB0
	rcall _pause_on
	cbi PORTB, PB0
	rjmp _end_blink

_to_5:
	cpi blink, 5
	brne _to_6
	sbi PORTD, PD3
	rcall _pause_on
	cbi PORTD, PD3
	rjmp _end_blink

_to_6:
	cpi blink, 6
	brne _to_7
	sbi PORTD, PD1
	rcall _pause_on
	cbi PORTD, PD1
	rjmp _end_blink

_to_7:
	cpi blink, 7
	brne _to_8
	sbi PORTD, PD5
	rcall _pause_on
	cbi PORTD, PD5
	rjmp _end_blink

_to_8:
	cpi blink, 8
	brne _to_9
	sbi PORTD, PD2
	rcall _pause_on
	cbi PORTD, PD2
	rjmp _end_blink

_to_9:
	cpi blink, 9
	brne _end_blink
	sbi PORTD, PD0
	rcall _pause_on
	cbi PORTD, PD0
	rjmp _end_blink

_end_blink:

		
	ret



;------------------------------------------------------------------------------
; Сообщение об ошибке
_error:

	sbi PORTA, PA0
	rcall _pause_100ms
	cbi PORTA, PA0
	rcall _pause_100ms

	sbi PORTA, PA0
	rcall _pause_100ms
	cbi PORTA, PA0
	rcall _pause_100ms

	sbi PORTA, PA0
	rcall _pause_100ms
	cbi PORTA, PA0
	rcall _pause_100ms

	ret


;------------------------------------------------------------------------------
; Копирование скрэтчпада в ОЗУ микроконтроллера
_copy_scratchpad_to_ram:
	ldi XL, Low(SRAM_START)
	ldi XH, High(SRAM_START)

	ldi temp, 9
_read_scratchpad:
	
	rcall _ds_read_byte
	
	st X+, ds_byte

	dec temp
	brne _read_scratchpad
	
	ret


;------------------------------------------------------------------------------
; ВЫЧИСЛЕНИЕ КОНТРОЛЬНОЙ СУММЫ
_calc_crc:
	ldi temp, CRC_GENERATING_POLYNOM
	mov crc_polynom, temp

	ldi XL, Low(SRAM_START)
	ldi XH, High(SRAM_START) 
	ld CRC, X+

	ldi temp, 8
	mov count_bytes, temp
_loop_bytes:
	ld CRC_next, X+

	ldi temp, 8
	mov count_bits, temp
_loop_bits:
	lsr CRC_next
	ror CRC
	brcc _zero
	eor CRC, crc_polynom
_zero:
	dec count_bits
	brne _loop_bits

	dec count_bytes
	brne _loop_bytes
	
	ret



;------------------------------------------------------------------------------
; СБРОС
_ds_reset:

	set

	dq_lo
	dq_output

	rcall _delay_480us

	dq_input

	nop
	nop
	nop
	nop
	nop
	nop
	nop

	sbic PINB, PB3
	clt

	//если флаг Т = 1 - датчик подключен
	//если флаг Т = 0 - датчик не подключен

	rcall _delay_480us
		
	ret



;------------------------------------------------------------------------------
; ОТПРАВКА 1 БИТА
_ds_write_bit:

	brcc _write_0

_write_1:
	dq_lo
	dq_output

	dq_input
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	ret

_write_0:
	dq_lo
	dq_output
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	dq_input

	ret



;------------------------------------------------------------------------------
; ОТПРАВКА 1 БАЙТА
_ds_write_byte:

	ldi i, 8

_write_loop:
	ror ds_byte
	rcall _ds_write_bit
	dec i
	brne _write_loop

	ret	



;------------------------------------------------------------------------------
; ЧТЕНИЕ 1 БИТА
_ds_read_bit:
	
	clc	

	dq_lo
	dq_output

	dq_input

nop


	sbic PINB, PB3
	sec

	nop
	nop
	nop
	nop
	nop

	ret



;------------------------------------------------------------------------------
; ЧТЕНИЕ 1 БАЙТА
_ds_read_byte:
	
	ldi i, 8

_read_loop:
	rcall _ds_read_bit
	ror ds_byte
	dec i
	brne _read_loop

	ret


;------------------------------------------------------------------------------
; 
_pause_on:
; ============================= 
;    delay loop generator 
;     51200 cycles:
; ----------------------------- 
; delaying 51198 cycles:
          ldi  R17, $6A
WGLOOP0:  ldi  R18, $A0
WGLOOP1:  dec  R18
          brne WGLOOP1
          dec  R17
          brne WGLOOP0
; ----------------------------- 
; delaying 2 cycles:
          nop
          nop
; ============================= 
ret



_pause_off:
; ============================= 
;    delay loop generator 
;     51200 cycles:
; ----------------------------- 
; delaying 51198 cycles:
          ldi  R17, $6A
WGLOOP2:  ldi  R18, $A0
WGLOOP3:  dec  R18
          brne WGLOOP3
          dec  R17
          brne WGLOOP2
; ----------------------------- 
; delaying 2 cycles:
          nop
          nop
; ============================= 
ret




_pause_100ms:
; ============================= 
;    delay loop generator 
;     12800 cycles:
; ----------------------------- 
; delaying 12798 cycles:
          ldi  R17, $12
WGLOOP4:  ldi  R18, $EC
WGLOOP5:  dec  R18
          brne WGLOOP5
          dec  R17
          brne WGLOOP4
; ----------------------------- 
; delaying 2 cycles:
          nop
          nop
; ============================= 
ret
		

_delay_480us:
; ============================= 
;    delay loop generator 
;     61 cycles:
; ----------------------------- 
; delaying 60 cycles:
          ldi  R17, $14
WGLOOP6:  dec  R17
          brne WGLOOP6
; ----------------------------- 
; delaying 1 cycle:
          nop
; ============================= 
	ret



