RESULT DCD     0,0,0,0,0,0,0,0,0,0
INPUT  DCD     7

START  LDR     R0,=INPUT
       LDR     R2,[R0]
       LDR     R5,[R0]
       LDR     R1,=RESULT
       MOV     R3,#0 ;COUNTER
       MOV     R4,#10 ;LIMIT

LOOP   CMP     R4,R3
       BEQ     EXIT
       ADD     R3,R3,#1
       STR     R2,[R1],#4
       ADD     R2,R2,R5
       B       LOOP

EXIT   LDR     R1,=RESULT
       END