RESULT DCD     2

START  LDR     R0,=RESULT
       LDR     R2,[R0]
       MOV     R3,#0 ;COUNTER
       MOV     R4,#8 ;LIMIT

LOOP   CMP     R4,R3
       BEQ     EXIT
       ADD     R3,R3,#1
       STR     R2,[R0],#4
       LSL     R2,R2,#1
       B       LOOP

EXIT   LDR     R0,=RESULT
       END
