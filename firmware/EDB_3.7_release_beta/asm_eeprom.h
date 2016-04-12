#include <avr/io.h>
.secion .text
.global EEWrite
.def EEdwr=r16
.def EEawr=r17
EEWrite:
sbic EECR,EEWE
rjmp EEWrite
out EEAR,EEawr
out EEDR,EEdwr
sbi EECR,EEWE
ret
.def EEdrd=r0
.def EEard=r16
.global EERead
EERead:
sbic EECR,EEWE
rjmp EERead
out EEDR,EEard
sbi EECR,EERE
sbi EECR,EERE
in EEdrd,EEDR
ret





