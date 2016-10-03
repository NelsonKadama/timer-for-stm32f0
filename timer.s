@ NLSANG001
@ Intro to micros practical exam 0 ; 2015-09-15

    .syntax unified
    .global _start

vectors:
    .word 0x20002000                @ 0x00: defines the reset value of the stack pointer
    .word _start + 1                @ 0x04: defines the reset vector, thereby specifying the first instruction.
    .word Default_Handler + 1       @ 0x08: NMI vector
    .word HardFault_Handler + 1     @ 0x0C: HardFault vector
    .word Default_Handler + 1       @ 0x10: reserved
    .word Default_Handler + 1       @ 0x14: reserved
    .word Default_Handler + 1       @ 0x18: reserved
    .word Default_Handler + 1       @ 0x1C: reserved
    .word Default_Handler + 1       @ 0x20: reserved
    .word Default_Handler + 1       @ 0x24: reserved
    .word Default_Handler + 1       @ 0x28: reserved
    .word Default_Handler + 1       @ 0x2C: SVCall
    .word Default_Handler + 1       @ 0x30: reserved
    .word Default_Handler + 1       @ 0x34: reserved
    .word Default_Handler + 1       @ 0x38: PendSV
    .word Default_Handler + 1       @ 0x3C: SysTick
    .word Default_Handler + 1       @ 0x40: WWDG
    .word Default_Handler + 1       @ 0x44: PVD_VDDIO2 
    .word Default_Handler + 1       @ 0x48: RTC
    .word Default_Handler + 1       @ 0x4C: FLASH
    .word Default_Handler + 1       @ 0x50: RCC_CRS
    .word Default_Handler + 1       @ 0x54: EXTI0_1
    .word Default_Handler + 1       @ 0x58: EXTI2_3
    .word Default_Handler + 1       @ 0x5C: EXTI4_15
    .word Default_Handler + 1       @ 0x60: TSC vector
    .word Default_Handler + 1       @ 0x64: DMA_CH1
    .word Default_Handler + 1       @ 0x68: DMA_CH2_3
    .word Default_Handler + 1       @ 0x6C: DMA_CH[4:7]
    .word Default_Handler + 1       @ 0x70: ADC_COMP v
    .word Default_Handler + 1       @ 0x74: TIM1_BRK_UP_TRG_COM
    .word Default_Handler + 1       @ 0x78: TIM1_CC
    .word Default_Handler + 1       @ 0x7C: TIM2
    .word Default_Handler + 1       @ 0x80: TIM3
    .word TIM6_Handler + 1          @ 0x84: TIM6_DAC
    .word Default_Handler + 1       @ 0x88: TIM7 (not implemented)
    .word Default_Handler + 1       @ 0x8C: TIM14
    .word Default_Handler + 1       @ 0x90: TIM15
    .word Default_Handler + 1       @ 0x94: TIM16
    .word Default_Handler + 1       @ 0x98: TIM17
    .word Default_Handler + 1       @ 0x9C: I2C1
    .word Default_Handler + 1       @ 0xA0: I2C2
    .word Default_Handler + 1       @ 0xA4: SPI1
    .word Default_Handler + 1       @ 0xA8: SPI2
    .word Default_Handler + 1       @ 0xAC: USART1
    .word Default_Handler + 1       @ 0xB0: USART2
    .word Default_Handler + 1       @ 0xB4: USART3_4
    .word Default_Handler + 1       @ 0xB8: CEC_CAN
    .word Default_Handler + 1       @ 0xBC: USB

HardFault_Handler:
    NOP
    B HardFault_Handler

Default_Handler:
    NOP
    B Default_Handler

TIM6_Handler:
    PUSH {LR}  @ saving the LR to the stack allows us to call subroutines from the ISR
    
	@ Ack the IRQ. 
    LDR R0, TIM6_BASE
	MOVS R2, #0
	STR R2, [R0, #0x10]
    @ END Ack the IRQ.
	
	@IF SWITCHE'S PRESSED, DONT CHANGE VALUE, EXIT TIMER
    LDR R0, GPIOA_BASE_ADDRESS		@SW0
	LDR R3, [R0, #0x10]
	MOVS R1, #1
	ANDS R3, R3, R1
	CMP R3, #0
	BEQ wrapped
	
	LDR R0, GPIOA_BASE_ADDRESS		@SW1
	LDR R3, [R0, #0x10]
	MOVS R1, #2
	ANDS R3, R3, R1
	CMP R3, #0
	BEQ wrapped
    
	LDR R0, GPIOA_BASE_ADDRESS		@SW2
	LDR R3, [R0, #0x10]
	MOVS R1, #4
	ANDS R3, R3, R1
	CMP R3, #0
	BEQ wrapped
    
    @ Fetch current value on LEDs
    @ Increment it.
    @ If it reaches 0, wrap back to pot value.
    
	@R0 has address of RAM
	@R1 gets counter value
	@R2 gets flag value
	@R4 gets LED value	
	
	LDR R0, RAM_START
	
	LDRB R4, [R0]
	LDRB R1, [R0, #0x03]
	LDRB R2, [R0, #0x02]
	
	LDR R7, GPIOB_BASE_ADDRESS
	
	CMP R2, #1			@check flag high
	BEQ set_flag_low
	
	CMP R2, #0			@check flag low
	BEQ set_flag_high
	
	after_flag:
	MOVS R3, R4			@SO R4 VALUE NOT AFFECTED WHEN MINUTE IS UP
	SUBS R3,R3,R2
    STR R3, [R7, #0x14]	@@@@
	
	SUBS R1, R1, #1
	STRB R1, [R0, #0x03]
	CMP R1, #0
	BEQ minute_up
    
	wrapped:
    POP {PC}  @ take that return code from stack into PC, thereby telling the CPU we want to exit from the ISR

    minute_up:
	MOVS R1, #60
	STRB R1, [R0, #0x03]
	SUBS R4, R4, #1
	STRB R4, [R0]
    CMP R4, #0
    BEQ back_to_max
	B wrapped
    
    back_to_max:
	LDR R0, RAM_START
	LDRB R4, [R0, #0x01]
	STRB R4, [R0]
	B wrapped
    
	set_flag_low:
	MOVS R2, #0
	STRB R2, [R0, #0x02]
	B after_flag
	
	set_flag_high:
	MOVS R2, #1
	STRB R2, [R0, #0x02]
	B after_flag
	
	
_start:
    @ Initliase LEDs
    
    @ enable GPIO A/B
	LDR R0, RCC_BASE_ADDRESS
	LDR R1, [R0, #0x14]
	LDR R2, PORT_A_B_ENABLE
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]
    
    @enable LEDs
	LDR R7, GPIOB_BASE_ADDRESS
	LDR R1, [R7, #0x00]
	LDR R2, LED_ENABLE
	ORRS R1, R1, R2
	STR R1, [R7, #0x00]
    
    @ set push-buttons pins to inputs with pull-up resistors
	LDR R6, GPIOA_BASE_ADDRESS
	LDR R1, [R6, #0x0C]
	LDR R2, SWITCH_ENABLE
	ORRS R1, R1, R2
	STR R1, [R6, #0x0C]
	
	@START POT INITIALISATION -----------------------------------------------------------------------------------------------
    @ SETTING PIN A5 TO ANALOG MODE
	LDR R6, GPIOA_BASE_ADDRESS
	LDR R1, [R6, #0x00]
	LDR R2, A5_ENABLE
	ORRS R1, R1, R2
	STR R1, [R6, #0x00]
	
	@CLOCKING THE ADC
	LDR R0, RCC_BASE_ADDRESS
	LDR R1, [R0, #0x18]
	LDR R2, ADC_CLOCK
	ORRS R1, R1, R2
	STR R1, [R0, #0x18]
	
	@SETTING ALIGN AND RESOLUTION
	LDR R0, ADC_BASE_ADDRESS
	LDR R1, [R0, #0x0C]
	LDR R2, ADC_ALIGN_RES
	ORRS R1, R1, R2
	STR R1, [R0, #0x0C]
@vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv	
	@ENABLING THE ADC
	LDR R0, ADC_BASE_ADDRESS
	MOVS R1, #0
	STR R1, [R0, #0x08]
	LDR R1, ADC_ENABLE
	STR R1, [R0, #0x08]
	B adc_ready
	
adc_ready:
	LDR R0, ADC_BASE_ADDRESS
	LDR R1, [R0, 0x00]
	MOVS R5, #1
	ANDS R1, R1, R5
	CMP R1, #1
	BNE adc_ready
@^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	@CHANNEL SELECTION
	LDR R0, ADC_BASE_ADDRESS
	LDR R1, [R0, #0x28]
	LDR R2, ADC_CHANNEL
	ORRS R1, R1, R2
	STR R1, [R0, #0x28]
	
	@END POT INITIALISATION -----------------------------------------------------------------------------------------------
	
    @ enable TIM6
	LDR R0, RCC_BASE_ADDRESS
	LDR R1, [R0, #0x1C]
	LDR R2, TIM6_ENABLE
	ORRS R1, R1, R2
	STR R1, [R0, #0x1C]
    
    @PSC
	LDR R0, TIM6_BASE
	LDR R1, PSC_VALUE
	STR R1, [R0, #0x28]
	
	@ARR
	LDR R0, TIM6_BASE
	LDR R1, ARR_VALUE_1
	STR R1, [R0, #0x2C]
	
	@UIE
	LDR R0, TIM6_BASE
	LDR R1, [R0, #0x0C]
	MOVS R2, #1
	ORRS R1, R1, R2
	STR R1, [R0, #0x0C]
	
	@CEN
	LDR R0, TIM6_BASE
	LDR R1, [R0, #0x00]
	MOVS R2, #1
	ORRS R1, R1, R2
	STR R1, [R0, #0x00]

    @ Enable TIM6 IRQ in NVIC
	LDR R0, NVIC_BASE
	LDR R1, [R0, #0x00]
	LDR R2, TIM6_INTERRUPT
	ORRS R1, R1, R2
	STR R1, [R0, #0x00]

    @ SETTING INITIAL RAM VALUES -------------------------------------------------------------------------------------------------
	
    LDR R0, RAM_START
	LDR R4, ALL_ON			@DEFAULT LED START VALUE
    
    to_RAM:
	STRB R4, [R0]			@ALL ON 
	MOVS R1, #60
	STRB R1, [R0, #0x03]	@60 sec counter
	MOVS R2, #0
	STRB R2, [R0, #0x02]	@Flag value
	
    
copy_to_RAM_complete:

    display_maximum:
    STR R4, [R7, #0x14]
    
    B display_maximum_done

display_maximum_done:

    @ Initialise TIM6, NVIC, push buttons, ADC

main_loop:
    @ If SW0 held:
    @ -- sample POT0 and set timer. 
	CHECK_IF_SW0_held:
	LDR R0, GPIOA_BASE_ADDRESS
	LDR R3, [R0, #0x10]
	MOVS R1, #1
	ANDS R3, R3, R1
	CMP R3, #0
	BEQ IF_SW0_pressed

    @ If SW1: 
    @ - Display value on POT (default FF)
    CHECK_IF_SW1_held:
	LDR R0, GPIOA_BASE_ADDRESS
	LDR R3, [R0, #0x10]
	MOVS R1, #2
	ANDS R3, R3, R1
	CMP R3, #0
	BEQ IF_SW1_pressed
    
    @ If SW2: 
    @ - Display timer position/value. Used to clear confusion of blinking LED
    CHECK_IF_SW2_held:
	LDR R0, GPIOA_BASE_ADDRESS
	LDR R3, [R0, #0x10]
	MOVS R1, #4
	ANDS R3, R3, R1
	CMP R3, #0
	BEQ IF_SW2_pressed
	   
    B main_loop
    
    IF_SW1_pressed:
	LDR R0, RAM_START
	LDRB R4, [R0, #0x01]
	STR R4, [R7, #0x14]
	B main_loop
		
	IF_SW2_pressed:
	LDR R0, RAM_START
	LDRB R4, [R0]
	STR R4, [R7, #0x14]
	B main_loop
		
	IF_SW0_pressed:
	@vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
	@SETTING ADSTART
	LDR R3, ADC_BASE_ADDRESS
	LDR R2, [R3, #0x08]
	LDR R2, ADC_ADSTART
	ORRS R2, R2, R2
	STR R2, [R3, #0x08]
	B eoc
	
	eoc:											
	LDR R3, ADC_BASE_ADDRESS
	LDR R2, [R3, #0x00]
	MOVS R5, #4
	ANDS R2, R2, R5
	CMP R2, #4
	BNE eoc
	
	@READING FROM DATA REGISTER AND DISPLAYING
	LDR R3, ADC_BASE_ADDRESS
	LDR R4, [R3, #0x40]
	STR R4, [R7, #0x14]
	LDR R0, RAM_START
	STRB R4, [R0]
	STRB R4, [R0, #0x01]
@^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	B main_loop

    .align
    
    RCC_BASE_ADDRESS: .word 0x40021000
    PORT_A_B_ENABLE: .word 0b1100000000000000000
    GPIOB_BASE_ADDRESS: .word 0x48000400
    GPIOA_BASE_ADDRESS: .word 0x48000000
    LED_ENABLE: .word 0b0101010101010101
    
    SWITCH_ENABLE: .word 0b01010101
    
    TIM6_BASE: .word 0x40001000
    TIM6_ENABLE: .word 0b10000
    PSC_VALUE: .word 0b111111111111	@4095
    ARR_VALUE_1: .word 0b11110011111 @1951    
    ARR_VALUE_2: .word 0b111101000000 @3904
    TIM6_INTERRUPT: .WORD 0b100000000000000000
    NVIC_BASE: .word 0xE000E100
    A5_ENABLE: .word 0b110000000000
	ADC_CLOCK: .word 0b1000000000
	ADC_ENABLE: .word 0b1
	ADC_BASE_ADDRESS: .word 0x40012400
	ADC_ALIGN_RES: .word 0b011000	6bit resolution so its not too sensitive
	ADC_CHANNEL: .word 0b100000
	ADC_ADSTART: .word 0b100
    
	@EACH ADRESS TAKES 8BITS
    RAM_START: .word 0x20000000
	ALL_ON: .word 0xFF
	POT_VALUE_OFFSET: .word 0x01
	FLAG_OFFSET: .word 0x02
	COUNTER_VALUE_OFFSET: .word 0x03
