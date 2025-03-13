
Stack_Size EQU 0x4E1C   ; 19996
top_of_stack EQU 0x20000000 + Stack_Size
 ;---C R Y S T A L   Q U A R T Z  =8000000 Hz-----
 
  
  EXPORT  __Vectors                ; Export the vector table
  PRESERVE8
  THUMB
  AREA RESET, DATA, READONLY
	  ;INTERRUPT VECTOR TABLE 
__Vectors       DCD     top_of_stack  ; Top of Stack $00000000
                DCD     Start         ; Reset Handler  $00000004
                DCD     Def_Vec       ; NMI Handler  $08
                DCD     TRASH_ISR      ; Hard Fault Handler  $0C
                DCD     TRASH_ISR_MPU      ; MPU Fault Handler    $10
                DCD     TRASH_ISR_BUS      ; Bus Fault Handler   $14
                DCD     TRASH_ISR_USAGE    ; Usage Fault Handler   $18
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     0                          ; Reserved
                DCD     Def_Vec      ; SVCall Handler  $2C
                DCD     Def_Vec      ; Debug Monitor Handler  $30
                DCD     0            ; Reserved
                DCD     Def_Vec     ; PendSV Handler     $38
                DCD     Def_Vec     ; SysTick Handler   $3C

                ; External Interrupts
                DCD     Def_Vec    ; Window Watchdog $40
                DCD     Def_Vec    ; PVD through EXTI Line detect $44
                DCD     Def_Vec     ; Tamper  $48
                DCD     Def_Vec     ; RTC  $4C
                DCD     Def_Vec    ; Flash  $50
                DCD     Def_Vec    ; RCC  $54
                DCD     Def_Vec    ; EXTI Line 0  $58
                DCD     Def_Vec    ; EXTI Line 1  $5c
                DCD     Def_Vec    ; EXTI Line 2  $60
                DCD     Def_Vec    ; EXTI Line 3  $64
                DCD     Def_Vec    ; EXTI Line 4  $68
                DCD     Def_Vec   ; DMA1 Channel 1  $6C
                DCD     spi1DmaRx   ; DMA1 Channel 2  $70
                DCD     spi1DmaTx  ; DMA1 Channel 3  $74
                DCD     Def_Vec   ; DMA1 Channel 4  $78
                DCD     Def_Vec   ; DMA1 Channel 5  $7C
                DCD     DMA1_CH6_ISR   ; DMA1 Channel 6   $80
                DCD     DMA1_CH7_ISR   ; DMA1 Channel 7   $84
                DCD     Def_Vec   ; ADC1_2         $88   
                DCD     Def_Vec    ; USB High Priority or CAN1 TX  $8C
                DCD     Def_Vec    ; USB Low  Priority or CAN1 RX0  $90
                DCD     Def_Vec   ; CAN1 RX1                      $94
                DCD     Def_Vec    ; CAN1 SCE                      $98
                DCD     Def_Vec    ; EXTI Line 9..5               $9C
                DCD     Def_Vec   ; TIM1 Break                    $A0
                DCD     Def_Vec   ; TIM1 Update                  $A4
                DCD     Def_Vec   ; TIM1 Trigger and Commutation      $A8
                DCD     Def_Vec    ; TIM1 Capture Compare         $AC
                DCD     Def_Vec   ; TIM2                      $B0
                DCD     tim3UpdateISR    ; TIM3                      $B4
                DCD     Def_Vec    ; TIM4                       $B8
                DCD     Def_Vec    ; I2C1 Event                   $BC
                DCD     Def_Vec    ; I2C1 Error                   $C0
                DCD     Def_Vec    ; I2C2 Event                   $C4
                DCD     Def_Vec    ; I2C2 Error                   $C8
                DCD     spi1Interrupt    ; SPI1                      $CC
                DCD     Def_Vec    ; SPI2                      $D0
                DCD     Def_Vec    ; USART1                      $D4
                DCD     usart2ISR    ; USART2                      $D8
                DCD     Def_Vec    ; USART3                      $DC
                DCD     gpioInterruptISR  ; EXTI Line 15..10               $E0
                DCD     Def_Vec   ; RTC Alarm through EXTI Line   $E4
                DCD     Def_Vec    ; USB Wakeup from suspend        $E8
					
   AREA myConst , DATA, READONLY 
prompt   DCB "Donald Duck is our favorite candidate in President election!!!!!!!!!!!"	
   
   AREA glVariables, DATA, READWRITE
uartRxBuffer      SPACE 32	
intermBuffer      SPACE 32	
dma1_7_disable_ch SPACE 4
dma1_7_data_to_transmit SPACE 4
semaphore         SPACE 4
byteCounter       SPACE 4
bytePointer       SPACE 4
int32TestVar1     SPACE 4
audioTimerDivider SPACE 4
soundCardMode     SPACE 4
playerTimeoutCounter  SPACE 4
debugBuffer       SPACE 128 ;@0x20000058
uartTxBuffer      SPACE 16384 ;0x4000 , 0x2000 -half  


   AREA MainCode, CODE, READONLY
	  ; |.text|
	  INCLUDE library.s  
   ENTRY
   
Start         PROC

      ;---debug buffer for test data corruption
	  LDR R0, =uartTxBuffer
	  LDR R1, =0x00
	  LDR R2, =0x00
lDbg01      
		STRB R2, [R0]
		ADD R1, #0x01
		ADD R0, #0x1
		TEQ R1, #0x1000
		BNE lDbg01	  
	  ;--debug buffer init end
	  LDR R1, =0x00000000
	  LDR R0, =semaphore
	  STR R1, [R0]
	  LDR R0, =byteCounter
	  LDR R1, =0x9
	  STR R1, [R0] ;init byte counter
	  LDR R1, =0x00000000
      LDR r0, =RCC_APB2ENR
	  STR R1, [R0]  ;clear APB2ENR
	  
	  LDR R1, =intermBuffer
      LDR R0, =bytePointer
      STR R1, [R0]	;initialize pointer  
	  BL gpio_init     ; Call the gpio_init procedure from the other file
	  LDR R0, =GPIOA_CRH
	  LDR R1, =0x0000000B
	  STR R1,[R0]
	  ;PA1 - alternative function TIM2_CH2
	  LDR R0, =GPIOA_CRL
	  LDR R1, =0x00004BB0 ;B alternative function push-pull, 50MHz
	  STR R1, [R0]
	  ;PORT B h
	  LDR R0, =GPIOB_CRH
	  LDR R1, =0x43344444  ;B13,B14 - output
	  STR R1,[R0]
	  ;PORTB low
	  LDR R0, =GPIOB_CRL
	  LDR R1,=0x44444444
	  STR R1, [R0]
	  ;---
	  LDR R1, =0x04300000  ;MODE C13=11 (bits20,21),   
	  LDR R0, =GPIOC_CRH 
	  STR R1, [R0]
	  ;Map GPIOC14 to EXTI Line 14
	  LDR R0, =AFIO_EXTICR4  
	  LDR R1, =0x00000200 ;mapping GPIOC14 to EXTI14  
	  STR R1, [R0]
	  ;Configure EXTI for Line 14
	  LDR R0, =EXTI_IMR       ; EXTI base address
      LDR R1, =(1 << 14) ;enable bit 14
	  STR R1, [R0];
	  
	  LDR R0, =EXTI_RTSR
	  LDR R1, =(1 << 14)  ; bit 14 on rising edge
	  STR R1, [R0]
	  ;priority level
	   LDR R0,=0xE000E428       ; Address of NVIC_IPR10
	   LDR R1, [R0]
	   LDR R2, =0x04
	   ORR R1, R2
	   STR R1, [R0]
	  ;Enable EXTI14 Interrupt in NVIC
	  
	   ;LDR R0, =NVIC_ISER1        ; NVIC_ISER1 address (for IRQ numbers 32-63)
       ;LDR R1, =(1 << (40 - 32)) ; Enable IRQ40 (EXTI14 is IRQ40)
       ;STR R1, [R0]              ; Write to NVIC_ISER1
	  ;----------------------C L O C K --------------------
	  ;---------sys_clock = (8 000 000Hz / 2) * 9 = 36 000 000 
	  LDR R5, =0x07010001
	  LDR R6, =0x00000000
	  PUSH {R5,R6}
  ;first@32[PLL_mult|HSE_psc|AHB_psc|MCO_EN] ,
  ;second@32[APB1_psc|APB2_psc|ADC_psc|USB_psc]
	  BL clkConfig
	  ;--turn led on
	  LDR R0, =0x00002000 ; bit13  set b13 ($2000), reset b29 ($20000000)
	  LDR R1, =GPIOC_BSRR
	  STR R0, [R1]
	  LDR R0, =0x20000000 ; bit13  set b13 ($2000), reset b29 ($20000000)
	  STR R0, [R1]
	  LDR R0, =0x01002000 ; bit13 
	  LDR R1, =GPIOC_ODR
	  STR R0,[R1]
 
   ;--SET-UP-WORKING MODE- waiting command from USART2
;I N F O:  BITSTREAM SOUND CARD COMMAND BYTE FORMAT:
; b7,b6-MODE     |b5-b0 S_RATE (bitstream rate / 64) 
;-----------*-0b00000000---------------------------------
;MODE - 1-play, 2-record  | S_RATE - 1,2,3,4
;--------------------------------------------------------
;S_RATE may be: 1->6KSpS (means 384KbPS  bitstream),2->8KSpS,
;               3->11,025KSpS,4->16000KSPS
;-END OF INFO--------------------------------------------

;=======FUNCTION uart_init_rx with Interrupt====================================
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)]  par2@[-|interrupt_RX_enable(8)|parity_type(8)|prity_en(8)]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, 0b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
;parity_en:   0-> disable parity, 1-> enable parity
;parity_type: 1->odd, 0->even
;interrupt_enable: FF->enable
   LDR R0, =0x00000018 ; 1500000 Bauds 
   LDR R1, =0x00FF0000
   PUSH {R0, R1}
   BL uart_init_rx
   ;waiting until a command has been received
inf_loop_001
   LDR R0, =semaphore
   LDR R1, [R0]
   CMP R1, #1
   BEQ _inf_loop_exit002   
   CMP R1, #2
   BEQ _inf_loop_exit002
   B inf_loop_001
_inf_loop_exit002
   ;when a command has been recognized:
   ;disable usart2
   BL usart2DeInit
   LDR R0, =soundCardMode
   LDR R1, [R0]
   CMP R1, #1
   BNE label_main001
     BL _playerRun
label_main001
   CMP R1, #2
   BNE label_main002
     BL _recorderRun
label_main002
   ;when a command is undefined:
lab_undefined_command_001   
   LDR R0, =audioTimerDivider
   LDR R1, =46 ;   (36000000 / (46+1) ) / 2=384000 
   STR R1, [R0] ;save in a variable
   LTORG
	ENDP
;---P R O C E D U R E S---

_playerRun      PROC
  ;-->>>>>-DAC-init--BEGIN----
   ;====Usart2DuplexDmaRxOnly================TST=BEGIN========	
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)]  
;--par2@[-|-|-|Circular]
;--par3@[pointerToDmaBuffer]   
;--par4@[tx_interrupt_en.DMA_rx_interrupt_en|priority_channel(8)|buffer_size(16)]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
;DMA_rx_int_en: 0x1 -> enable DMA interrupts, 0x0->interrupts disable
;tx_int_en: 0x1->TC transmitter interrupt enable, 0x0->disable
;***************************************************************
;NOTES: 1) After a full DMA transaction content of the DMA1_CNDTR7 reister reaches zero
;It MUST be restored (disable channel->restore CNDTR->enable_channel(start)) before start the next DMA transaction
;     2)To start DMA TX transaction - set bit 0 in DAM_CCR7 channel 
   LDR R0,   =0x00000018; 1500000 Bauds
   LDR R1, =0x00000000 ;usual mode
   LDR R2, =uartTxBuffer
   LDR R3, =0x00022000 ;  8192 bytes (half)
   PUSH {LR}
   PUSH {R0, R1, R2, R3};
   BL Usart2DuplexDmaRxOnly
   POP {LR}
   LDR R0, =DMA1_CCR6
   LDR R1, [R0]
   ORR R1, #0x1
   STR R1, [R0]

;----spiSlaveTransmitterWithDma   
;NECCESSERY NOTE for PROGRAMMERS: to S T A R T  DMA transmission- 
	;1)When run second time or later - Set DMA1_CNDTR3 amount of words in transaction
	     ;(it cleared at the end of each transaction)
		 ;When it runs for the first time - in shoul be setted later  
	;2)Turn ON DMA1_CCR3 [bit 0]. Transaction is starting NOW.
;***************************************************************************	
;INFO: point "."  a separator of nibbles (4bit fields), '|' byte separator
;parameter1 @ [frame_length.---|LSBFIRST|SSM|CPOL.CPHA]
;----------------------------
;bits:
;(0-3) CPHA , (16-23) LSBFIRST
;(4-7) CPOL , (24-27) Divider - not set in slave mode 
;(8-15) SSM , (28-31) frame length
;---------------------------
;parameter2 @ [addressOfBuffer]
;---------------------------
;parameter3 @ [-|circularMode8|wordsInBuffer16]
;wordsInBuffer - amount of 8bit or 16bit transactions to transmitt
;circularMode - enable circular mode
;------------------------------
    LDR R0, =0x00010010
	LDR R1, =uartTxBuffer
	LDR R2, =0x00014000  ;16384-full buffer
	PUSH {LR}
	PUSH {R0, R1, R2}
	BL spi1SlaveTransmitterWithDma 
	POP {LR}
	LDR R0, =DMA1_CCR3
    LDR R1, [R0]
	LDR R2, =0x1
	ORR R1, R2
	STR R1, [R0]
;====FUNCTION initialize TIM2 CH2 Output Compare
;    A@[b31 presc| period b0],
;Width@[b31 null |  width b0]
;example bassing params PUSH {A,Width}
     LDR R0, =audioTimerDivider
	 LDR R1, [R0]
	;LDR R1,  =0x00000027 ;=>384kHz
	LDR R2, =0x02;
    PUSH {LR}	
	PUSH {R1,R2}
	BL tim2OcCh2Setup	
	POP {LR}

l_0005_x
  B l_0005_x
  ;--<<<<<<-DEBUG--END
  ;--DAC initialization end-----
  LTORG
  ENDP
	  
_recorderRun   PROC 
  ;---DeltaSigma-ADC--mode----begin 
 ;---SPI--------------
;--To start : 2)Clear SSI (bit8) SPI1_CR1
;             1)Enable DMA1 channel 2 (Bit0) DMA1_CCR2
;             3)When mode NOT circular - update DMA1_CNDTR2 register after end of DMA transaction 
;***************************************************************************	
;INFO: point "."  a separator of nibbles (4bit fields), '|' byte separator
;parameter1 @ [frame_length.-|LSBFIRST|SSM|CPOL.CPHA]
;----------------------------
;bits:
;(0-3) CPHA , (16-23) LSBFIRST
;(4-7) CPOL , (24-27) - 
;(8-15) SSM , (28-31) frame length (8[0] or 16 [1] bits)
;---------------------------
;parameter2 @ [addressOfBuffer]
;---------------------------
;parameter3 @ [-|circularMode|wordsInBuffer]
;wordsInBuffer - amount of 8bit or 16bit transactions to transmitt
;circularMode - enable circular mode
;------------------------------
    LDR R0, =0x00010110
	LDR R1, =uartTxBuffer
	LDR R2, =0x00014000 ;16384words(4096bytes) per transaction
	PUSH {LR}
	PUSH {R0, R1, R2}
	BL   spi1SlaveOnlyReceiverDMAIt
    POP {LR}	
    ;tusn on SPI DMA
	;1) Clear SSI (bit8) SPI1_CR1
	LDR R0, =SPI1_CR1
	LDR R1, [R0]
	LDR R2, =(1<<8)
	MVN R2, R2
	AND R1, R2
	STR R1, [R0]
	;2)Enable DMA1 CH2 (b0) in the DMA1_CCR2 reg
	LDR R0, =DMA1_CCR2
	LDR R1, [R0]
	LDR R2, =0x1
	ORR R1, R2
	STR R1, [R0]
	;3)Enable DMA1 channel 2 (Bit0) DMA1_CCR2
	LDR R0, =DMA1_CCR2
	LDR R1, [R0]
	LDR R2, =0x1
	ORR R1, R2
	STR R1, [R0]
    NOP

	  ;----SPI END

;=======USART2_TRANSMITTER_DRIVEN_DMA==============
;--par1@[WordLength(8)|stopBits(8)|bauds_divider(16)]  
;--par2@[-|-|-|interrupts_enable]
;--par3@[pointerToDmaBuffer]   
;--par4@[null(4)|DMA_TC_interrupt(4)|priority_channel(8)|buffer_size(16)]
;WordLength: WHEN 1 -> 1 Start bit, 9 Data bits, n Stop bit
;            WHEN 0-> 0: 1 Start bit, 8 Data bits, n Stop bit
;StopBits: 0b00-> 1 bit, b01-> 0.5 Stop bit, b10-> 2 Stop bits, b11-> 1.5 Stop bit,
;interrupts_enable: 0xFFFF -> enable TXE TC interrupts, 0->interrupts disable
;***************************************************************
;NOTES: 1) After a full DMA transaction content of the DMA1_CNDTR7 reister reaches zero
;It MUST be restored before start the next DMA transaction
;     2)To start DMA TX transaction - set bit 0 in DAM_CCR7 channel    
   LDR R0,   =0x00000018; 1500000 Bauds
   LDR R1, =0x00000000;
   LDR R2, =uartTxBuffer ;buffer address
   LDR R3, =0x000002000 ;8192 bytes TX ROM buffer
   PUSH {LR}
   PUSH {R0,R1,R2,R3}
   BL uart_init_tx_dma
   POP {LR}
	;====FUNCTION initialize TIM2 CH2 Output Compare
	;    A@[b31 presc| period b0],
	;Width@[b31 null |  width b0]
	;example bassing params PUSH {A,Width}
	     LDR R0, =audioTimerDivider
	     LDR R1, [R0]
		;;LDR R1, =29 ; (30 720 000 / (29+1))   / 2 = 512000 8KspS
		LDR R2, =0x02; //align edges
		PUSH {LR}
		PUSH {R1,R2}
		BL tim2OcCh2Setup
		POP {LR}
mylabel
	 
	B mylabel
	LTORG
 ENDP



 ;----ISR-handlers---------
;  ___ ____  ____   
; |_ _/ ___||  _ \  
;  | |\___ \| |_) | 
;  | | ___) |  _ <  
; |___|____/|_| \_\ 
                   
 ;===exti port c=======
 
spi1DmaRx  PROC
    ;;--test for SPI errors
	LDR R0, =SPI1_SR
	LDR R1, [R0]
	TST R1, #(1<<3)|(1<<5)
	BEQ l_spi1DmaRx_01  ;jump when it isn`t any SPI error
    LDR R0, =GPIOB_BSRR
    LDR R1, =(1<<14)  ;set
    STR R1, [R0]
	B l_spi1DmaRx_02 
l_spi1DmaRx_01
    LDR R1, =(1<<30)  ;clear
	STR R1, [R0]
l_spi1DmaRx_02
	;--Is it half transfer interrupts?
	LDR R0, =DMA1_ISR
	LDR R1, [R0]
	LDR R2, =(1<<6) ;HTIF2 flag
	ANDS R2, R1 ;are there flag?
	BEQ l_isr001_next_check
	;half-transfer interrupts
	   ;a)disable DMA channel (UART Tx)
	  LDR  R0, =DMA1_CCR7
	  LDR R1, [R0]
	  LDR R2, =0x1
	  MVN R2,  R2
	  AND R1,  R2
	  STR R1, [R0]
	  ;b)assign UART Tx DMA CH7 buffer address
	  LDR R0, =uartTxBuffer  ;first half (bytes 0-8191)
	  LDR R1, =DMA1_CMAR7
	  STR R0, [R1] ; store memory start point for  DMA Ch7
	  ;c) amount of words in DMA1 Ch7 (UART Tx) transaction
	  LDR R0, =DMA1_CNDTR7
	  LDR R1, =0x2000 ; 8192 words in one trnsaction (1 word - 1 byte)
	  STR R1, [R0]
	  ;d)enable DMA channel
	  LDR  R0, =DMA1_CCR7
	  LDR R1, [R0]
	  LDR R2, =0x1
	  ORR R1,  R2
	  STR R1, [R0]  ;  ->> transaction is starting here	  
	   ;--turn LED ON
	  LDR R0, =GPIOC_BSRR
	  LDR R1, =(1 << 13)
	  STR R1,[R0]
l_isr001_next_check
    LDR R2, =(1<<5) ;TCIF2 flag
	ANDS R2, R1 ;are there flag
	BEQ l_isr001_fin
	;full transfer interrupts
	  
	 ;a)disable DMA channel
	  LDR  R0, =DMA1_CCR7
	  LDR R1, [R0]
	  LDR R2, =0x1
	  MVN R2,  R2
	  AND R1,  R2
	  STR R1, [R0]
	  ;b)assign UART Tx DMA CH7 buffer address
	  LDR R0, =uartTxBuffer
	  ADD R0, #0x2000  ;second half (bytes 8192-16384)
	  LDR R1, =DMA1_CMAR7
	  STR R0, [R1] ; store memory start point for  DMA Ch7
	  ;c) amount of words in DMA1 Ch7 (UART Tx) transaction
	  LDR R0, =DMA1_CNDTR7
	  LDR R1, =0x2000 ; 8192 words in one trnsaction (1 word - 1 byte)
	  STR R1, [R0]
	  ;d)enable DMA channel
	  LDR  R0, =DMA1_CCR7
	  LDR R1, [R0]
	  LDR R2, =0x1
	  ORR R1,  R2
	  STR R1, [R0]  ;  ->> transaction is starting here	 
	  ;--LED OFF
	  LDR R0, =GPIOC_BSRR
      LDR R1, =(1 << 29)
	  STR R1,[R0]

l_isr001_fin
	;clear all flags in DMA_CH2 
	LDR R0, =DMA1_IFCR
	LDR R1, =(1<<4) ; CGIF2
	STR R1, [R0]	
	BX LR
	ENDP
 
spi1Interrupt PROC
	LDR R0, =int32TestVar1
	LDR R2, =0x000000FF ; mask
	LDR R1, [R0]
	ADD R1, R1, #1
	AND R1, R2
	STR R1, [R0]

	BX LR
	ENDP
;--DMA1 Ch3 (SPI1 Tx) Interrupt service routine
spi1DmaTx PROC
	;is there half or full transfer?
	LDR R0, =DMA1_ISR
	LDR R1, [R0]
	ANDS R1, #(1<<10)  ;HTIF3
	BEQ spi1DmaTx_full_transfer
		;half transfer
		;so, copy first half - from uart2 DMA ch6
		;disable DAM1_CH6
		LDR R0, =DMA1_CCR6
		LDR R1, [R0] ;read content
		ORR R1, 0x1 ;set 1  to correct working when EN hasn`t been set
		EOR R1, #0x1 ;clear flag
		STR R1, [R0]  ;store to CCR6
		;set address of entry point for DMA Ch6
		LDR R1, =uartTxBuffer
		LDR R0, =DMA1_CMAR6
		STR R1, [R0] ;memory addres for UART Tx DMA transaction
		;transaction length
		LDR R0, =DMA1_CNDTR6
		LDR R1, =0x2000 ;8192bytes
		STR r1, [R0]
		;enable DMA channnel
		LDR R0, =DMA1_CCR6
		LDR R1, [R0] ;read content
		ORR R1, 0x1 ;set 1 
		STR R1, [R0]  ;store to CCR6
		;send byte to host through UART2
		LDR R0, =USART2_DR
		LDR R1, =0x10
		STR R1, [R0]
		;LED
		 LDR R0, =GPIOC_BSRR
		 LDR R1, =(1 << 13)
		 STR R1,[R0]
		B spi1DmaTx_fin_part
spi1DmaTx_full_transfer
     ;when full transfer -copying the second part
	 ;has a transaction been done?
	 LDR R0, =DMA1_ISR
	 LDR R1, [R0]
	 ANDS R1, #(1<<20) ;GIF6
	 BEQ l_usart2_no_dma_rx
	 ;when DMA6 UART1 Tx transaction happened:
     LDR R0, =DMA1_IFCR
     LDR R1, =(1<<20)
     STR R1, [R0] ;clear GIF6
     LDR R0, =playerTimeoutCounter
	 LDR R1, =0
	 STR R1, [R0]  ;clear timeout variable
l_usart2_no_dma_rx
     LDR R0, =playerTimeoutCounter
	 LDR R1, [R0]
	 ADD R1, #1
	 STR R1, [R0]  ;increment timeout counter
	 ;is the counter > MAX?
	 CMP R1, #0x04  ;     !!!DELAY BEFORE RESET
	 BNE l_no_timeout001
	 	 ;*********************RESET CPU***SEQUENCE**
	  LDR R0, =GPIOB_BSRR
	  LDR R1, =(1<<13) ;turn on led
	  STR R1, [R0]
	  NOP
	  NOP
	  NOP  ;for led flash
	  LDR R0, =0xE000ED0C         ; Address of SCB->AIRCR
	  LDR R1, =0x05FA0004         ; VECTKEY (0x5FA << 16) | SYSRESETREQ (1 << 2)
	  STR R1, [R0]                ; Write to AIRCR to trigger reset

		 ;**********************END RESET SEQ
l_no_timeout001
	 	 ;disable DAM1_CH6
	LDR R0, =DMA1_CCR6
	LDR R1, [R0] ;read content
	ORR R1, 0x1 ;set 1  to correct working when EN hasn`t been set
	EOR R1, #0x1 ;clear flag
	STR R1, [R0]  ;store to CCR6
	;set address of entry point for DMA Ch6
	LDR R1, =uartTxBuffer
	;working with second half
	LDR R2, =0x2000
	ADD R1,R1, R2
	LDR R0, =DMA1_CMAR6
	STR R1, [R0] ;memory addres for UART Tx DMA transaction
	;transaction length
	LDR R0, =DMA1_CNDTR6
	LDR R1, =0x2000 ;8192bytes
	STR r1, [R0]
	;enable DMA channnel
	LDR R0, =DMA1_CCR6
	LDR R1, [R0] ;read content
	ORR R1, #0x1 ;set 1 
	STR R1, [R0]  ;store to CCR6
	;send byte to host through UART2
	LDR R0, =USART2_DR
	LDR R1, =0x20
	STR R1, [R0]
	;LED  
	  LDR R0, =GPIOC_BSRR
     LDR R1, =(1 << 29)
	 STR R1,[R0]
spi1DmaTx_fin_part
	;clear all flags in DMA_CH3 
	LDR R0, =DMA1_IFCR
	LDR R1, =(1<<8) ; CGIF3
	STR R1, [R0]
 
	BX LR
    ENDP    
; external interrupts 
gpioInterruptISR  PROC
    LDR R0, =EXTI_PR           ; Load EXTI Pending Register address
    LDR R1, =(1 << 14)         ; Prepare mask for EXTI Line 14
    STR R1, [R0]               ; Write to EXTI_PR to clear the pending flag
	
	  BX LR
	  ENDP
 
 ;============default ISR handler.Infinite loop
Def_Vec PROC
    B   .  ; Infinite loop
    ENDP
;====ext vector interrupt

;====TIM2=====ISR========		
tim3UpdateISR   PROC
	;clear flag UIE
	LDR R0, =TIM3_SR
    MOV R1,#0
	STR R1, [R0] ; update the SR
	BX LR ;reti
	ENDP
;=====USART2==Interrupt Service Routine=
usart2ISR  PROC
	LDR R0, =USART2_SR
	LDR R1, [R0]
	MVN R2, #(1<<6)
	AND R1, R2
	STR R1, [R0] 
	;read a command
	LDR R0, =USART2_DR
	LDR R3, [R0]
	
	;set timer 2 divider (clock for SPI in slave mode)
	;F timer = (ARR+1)/2
    LDR R1, =0x1	
	;operator if-else
	MOV R2, R3
	AND R2, #0x3F
	LDR R0, =audioTimerDivider ;addres to store variable value
	CMP R2, #1 ;6KHz sample rate
	BNE entryComm_j1
	;set divider for 6KHz sample rate (384KHz clock delta-sigma)
	LDR R1, =46 ;   (36000000 / (46+1) ) / 2=384000 
	STR R1, [R0] ;save in a variable
entryComm_j1
	CMP R2, #2 ;8KHz sample rate
	BNE entryComm_j2
	;set divider for 8KHz sample rate (512KHz clock delta-sigma)
	 LDR R1, =34 ; (36000000 / (34+1))   / 2 = 512000
	 LDR R0, =audioTimerDivider
	 STR R1, [R0]
entryComm_j2
	CMP R2, #3 ;11025 Hz sample rate
	BNE entryComm_j3
	;set divider for 11025Hz sample rate
	LDR R1, =25  ;(36000000 / 25+1) / 2 = 705600
	STR R1, [R0]
entryComm_j3
    CMP R2, #4
	BNE entryComm_j4
   ;set divider for 16000Hz sample rate
    LDR R1, =17   ;(36000000 / 17+1) / 2 = 1 024 000  (roughly 2%err)
	STR R1, [R0]
entryComm_j4
    CMP R2, #5
	BNE entryComm_j5
   ;set divider for 22050Hz sample rate
    LDR R1, =12   ;(36000000 / 12+1) / 2 = 1 411 200 (roughly 2%err)
	STR R1, [R0]
entryComm_j5
    ;What is the command here?
	MOV R2, R3
	AND R2, #0xc0 ; bits 6.7
	LSR R2, #6
	LDR R0, =soundCardMode
	STR R2, [R0]
	LDR R0, =semaphore
	LDR R1, =1
	STR R1, [R0] ;save semaphore to exit from infinite loop
	
	BX LR
	  ENDP
;==========DMA1 CH7 (UART2 TX)
DMA1_CH7_ISR PROC
	;clear flag 
	LDR R0,=DMA1_IFCR
	LDR R1,=(1<<24)
	STR R1,[R0]
   BX LR
     ENDP
;====DMA1 Ch6 (USART2 RX)
DMA1_CH6_ISR PROC
	;--Is it half transfer interrupts?
	LDR R0, =DMA1_ISR
	LDR R1, [R0]
	LDR R2, =(1<<22) ;HTIF6 flag
	ANDS R2, R1 ;are there flag?
	BEQ l_isr001_to_full_tr
	  ;--echo
	  LDR R0, =USART2_DR
	  LDR R1, =debugBuffer
	  LDR R2, [R1]
	  STR R2, [R0]
	  ;--LED ON
	  LDR R0, =GPIOC_BSRR
	  LDR R1, =(1 << 13)
	  STR R1,[R0]
	  B l_end_001
l_isr001_to_full_tr
	 LDR R0, =GPIOC_BSRR
     LDR R1, =(1 << 29)
	 STR R1,[R0]

l_end_001
	;clear interupt flag 
  ;clear
	LDR R0,=DMA1_IFCR
	LDR R1,=(1<<20)
	STR R1,[R0]
    BX LR
      ENDP

;======hard fault ISR
TRASH_ISR PROC
	LDR R0, =GPIOB_BSRR
	LDR R1, =(1<<13) ;turn on led
	STR R1, [R0]
	B .
	ENDP
		
TRASH_ISR_BUS PROC
	LDR R0, =GPIOB_BSRR
	LDR R1, =(1<<13) ;turn on led
	STR R1, [R0]
	B .
	ENDP

TRASH_ISR_MPU PROC
	LDR R0, =GPIOB_BSRR
	LDR R1, =(1<<13) ;turn on led
	STR R1, [R0]
	B .
	ENDP
		
TRASH_ISR_USAGE PROC
	LDR R0, =GPIOB_BSRR
	LDR R1, =(1<<15) ;turn on led
	STR R1, [R0]
	B .
	ENDP
		


   ALIGN
   END