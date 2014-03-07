	.ref	_main
	.sect	".begin"

	LB	_main

; These pre-ISRs make the comm ISRs pre-emptible by turning on global interrupts.
; You want to do this before jumping to the C ISR because the compiler prepends
; a bunch of code to ISRs, and if you execute that with global interrupts turned
; off, you will see increased interrupt latency jitter in the PWM ISR. 

    .ref     _uartTxCharIsr
    .sect    ".h0_code"
    .def     _uartTxCharPreIsr
_uartTxCharPreIsr:
    EINT                    ; enable interrupts to make this ISR preemptable
    LB        _uartTxCharIsr      ; jump to the ISR


    .ref     _modbusPacketTimerIsr
    .sect    ".h0_code"
    .def     _modbusPacketTimerPreIsr
_modbusPacketTimerPreIsr:
    EINT                    ; enable interrupts to make this ISR preemptable
    LB        _modbusPacketTimerIsr      ; jump to the ISR

    .ref     _uartRxCharIsr
    .sect    ".h0_code"
    .def     _uartRxCharPreIsr
_uartRxCharPreIsr:
    EINT                    ; enable interrupts to make this ISR preemptable
    LB        _uartRxCharIsr      ; jump to the ISR
 
    .end                                                   

