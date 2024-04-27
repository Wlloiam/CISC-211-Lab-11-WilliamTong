/*** asmFmax.s   ***/
#include <xc.h>
.syntax unified

@ Declare the following to be in data memory
.data  

@ Define the globals so that the C code can access them

.global f1,f2,fMax,signBitMax,biasedExpMax,expMax,mantMax
.type f1,%gnu_unique_object
.type f2,%gnu_unique_object
.type fMax,%gnu_unique_object
.type signBitMax,%gnu_unique_object
.type biasedExpMax,%gnu_unique_object
.type expMax,%gnu_unique_object
.type mantMax,%gnu_unique_object

.global sb1,sb2,biasedExp1,biasedExp2,exp1,exp2,mant1,mant2
.type sb1,%gnu_unique_object
.type sb2,%gnu_unique_object
.type biasedExp1,%gnu_unique_object
.type biasedExp2,%gnu_unique_object
.type exp1,%gnu_unique_object
.type exp2,%gnu_unique_object
.type mant1,%gnu_unique_object
.type mant2,%gnu_unique_object
 
.align
@ use these locations to store f1 values
f1: .word 0
sb1: .word 0
biasedExp1: .word 0  /* the unmodified 8b exp value extracted from the float */
exp1: .word 0
mant1: .word 0
 
@ use these locations to store f2 values
f2: .word 0
sb2: .word 0
exp2: .word 0
biasedExp2: .word 0  /* the unmodified 8b exp value extracted from the float */
mant2: .word 0
 
@ use these locations to store fMax values
fMax: .word 0
signBitMax: .word 0
biasedExpMax: .word 0
expMax: .word 0
mantMax: .word 0

.global nanValue 
.type nanValue,%gnu_unique_object
nanValue: .word 0x7FFFFFFF            

@ Tell the assembler that what follows is in instruction memory    
.text
.align

/********************************************************************
 function name: initVariables
    input:  none
    output: initializes all f1*, f2*, and *Max varibales to 0
********************************************************************/
.global initVariables
 .type initVariables,%function
initVariables:
    /* YOUR initVariables CODE BELOW THIS LINE! Don't forget to push and pop! */
    push {r4-r11,LR}
    
    mov r10,0
    mov r11,1
    ldr r4,=f1
    str r0,[r4]
    ldr r4,=sb1
    str r10,[r4]
    ldr r4,=biasedExp1
    str r10,[r4]
    ldr r4,=exp1
    str r10,[r4]
    ldr r4,=mant1
    str r10,[r4]
    
    ldr r4,=f2
    str r1,[r4]
    ldr r4,=sb2
    str r10,[r4]
    ldr r4,=biasedExp2
    str r10,[r4]
    ldr r4,=exp2
    str r10,[r4]
    ldr r4,=mant2
    str r10,[r4]
    
    ldr r4,=fMax
    str r10,[r4]
    ldr r4,=signBitMax
    str r10,[r4]
    ldr r4,=biasedExpMax
    str r10,[r4]
    ldr r4,=expMax
    str r10,[r4]
    ldr r4,=mantMax
    str r10,[r4]
    
    pop {r4-r11,LR}
    mov pc, lr
    /* YOUR initVariables CODE ABOVE THIS LINE! Don't forget to push and pop! */

    
/********************************************************************
 function name: getSignBit
    input:  r0: address of mem containing 32b float to be unpacked
            r1: address of mem to store sign bit (bit 31).
                Store a 1 if the sign bit is negative,
                Store a 0 if the sign bit is positive
                use sb1, sb2, or signBitMax for storage, as needed
    output: [r1]: mem location given by r1 contains the sign bit
********************************************************************/
.global getSignBit
.type getSignBit,%function
getSignBit:
    /* YOUR getSignBit CODE BELOW THIS LINE! Don't forget to push and pop! */
    push {r4-r11,LR}
    mov r11,1
    mov r10,0
    mov r4,0x80000000
    ldr r5,[r0]
    ands r5,r5,r4
    movmi r5,r11
    movpl r5,r10
    str r5,[r1]
    
    pop {r4-r11,LR}
    mov pc, lr
    /* YOUR getSignBit CODE ABOVE THIS LINE! Don't forget to push and pop! */
    

    
/********************************************************************
 function name: getExponent
    input:  r0: address of mem containing 32b float to be unpacked
            r1: address of mem to store BIASED
                bits 23-30 (exponent) 
                BIASED means the unpacked value (range 0-255)
                use exp1, exp2, or expMax for storage, as needed
            r2: address of mem to store unpacked and UNBIASED 
                bits 23-30 (exponent) 
                UNBIASED means the unpacked value - 127
                use exp1, exp2, or expMax for storage, as needed
    output: [r1]: mem location given by r1 contains the unpacked
                  original (biased) exponent bits, in the lower 8b of the mem 
                  location
            [r2]: mem location given by r2 contains the unpacked
                  and UNBIASED exponent bits, in the lower 8b of the mem 
                  location
********************************************************************/
.global getExponent
.type getExponent,%function
getExponent:
    /* YOUR getExponent CODE BELOW THIS LINE! Don't forget to push and pop! */
    push {r4-r11,LR}
    
    ldr r4,=0x7F800000
    ldr r5,[r0]
    AND r5,r5,r4
    lsr r5,r5,23
    
    str r5,[r1]
    sub r5,r5,127
    str r5,[r2]
    
    
    pop {r4-r11,LR}
    mov pc, lr
    /* YOUR getExponent CODE ABOVE THIS LINE! Don't forget to push and pop! */
   

    
/********************************************************************
 function name: getMantissa
    input:  r0: address of mem containing 32b float to be unpacked
            r1: address of mem to store unpacked bits 0-22 (mantissa) 
                of 32b float. 
                Use mant1, mant2, or mantMax for storage, as needed
    output: [r1]: mem location given by r1 contains the unpacked
                  mantissa bits
********************************************************************/
.global getMantissa
.type getMantissa,%function
getMantissa:
    /* YOUR getMantissa CODE BELOW THIS LINE! Don't forget to push and pop! */
    push {r4-r11,LR}
    
    ldr r6,=0x7FFFFF
    ldr r7,=0x7F800000 
    ldr r8,=0x800000
    
    ldr r4,[r0]
    mov r5,r4
    and r4,r4,r6
    
    and r5,r5,r7
    cmp r5,0
    bne setbit23_1
    beq setbit23_0
    
    setbit23_1:
    cmp r5,r7
    cmpeq r4,0
    beq this_is_inf_case
    orr r4,r4,r8
    str r4,[r1]
    b done_for_sign
    
    setbit23_0:
    str r4,[r1]
    b done_for_sign
    
    this_is_inf_case:
    mov r4,0
    str r4,[r1]
    
    done_for_sign:
    pop {r4-r11,LR}
    mov pc,lr
    /* YOUR getMantissa CODE ABOVE THIS LINE! Don't forget to push and pop! */
   


    
/********************************************************************
function name: asmFmax
function description:
     max = asmFmax ( f1 , f2 )
     
where:
     f1, f2 are 32b floating point values passed in by the C caller
     max is the ADDRESS of fMax, where the greater of (f1,f2) must be stored
     
     if f1 equals f2, return either one
     notes:
        "greater than" means the most positive numeber.
        For example, -1 is greater than -200
     
     The function must also unpack the greater number and update the 
     following global variables prior to returning to the caller:
     
     signBitMax: 0 if the larger number is positive, otherwise 1
     expMax:     The UNBIASED exponent of the larger number
                 i.e. the BIASED exponent - 127
     mantMax:    the lower 23b unpacked from the larger number
     
     SEE LECTURE SLIDES FOR EXACT REQUIREMENTS on when and how to adjust values!


********************************************************************/    
.global asmFmax
.type asmFmax,%function
asmFmax:   

    /* Note to Profs: Solution used to test c code is located in Canvas:
     *    Files -> Lab Files and Coding Examples -> Lab 11 Float Solution
     */

    /* YOUR asmFmax CODE BELOW THIS LINE! VVVVVVVVVVVVVVVVVVVVV  */
    push {r4-r11,LR}
    mov r10,0
    BL initVariables 
    /*
    ldr r11,=f1
    ldr r9,=0x7fc00000
    str r9,[r11]
    
    ldr r11,=f2
    ldr r9,=0x3f800000
    str r9,[r11]
    */
    
    ldr r0,=f1
    ldr r1,=sb1
    BL getSignBit
    ldr r5,[r1]
    
    ldr r0,=f2
    ldr r1,=sb2
    BL getSignBit
    ldr r6,[r1]
    
    check_Nan_Or_Inf:
    ldr r0,=f1
    ldr r1,=biasedExp1
    ldr r2,=exp1
    BL getExponent
    ldr r7,[r1]
    cmp r7,0xFF
    beq f1_Nan_or_Inf
    bne check_f2_Nan_or_Inf
    
    check_f2_Nan_or_Inf:
    ldr r0,=f2
    ldr r1,=biasedExp2
    ldr r2,=exp2
    BL getExponent
    ldr r7,[r1]
    cmp r7,0xFF
    beq f2_Nan_or_Inf
    bne Not_Nan_or_Inf_Case
    
    f1_Nan_or_Inf:
    ldr r0,=f1
    ldr r1,=mant1
    BL getMantissa
    ldr r8,[r1]
    cmp r8,0x00400000
    beq Nan_Case
    bne f1_Inf_Case
    
    f2_Nan_or_Inf:
    ldr r0,=f2
    ldr r1,=mant2
    BL getMantissa
    ldr r8,[r1]
    cmp r8,0x00400000
    beq Nan_Case
    bne f2_Inf_Case
    
    Nan_Case:
    ldr r7,=fMax
    mov r8,0x7FFFFFFF
    str r8,[r7]
    b Max_Var_Case
    
    f1_Inf_Case:
    ldr r0,=f1
    ldr r1,=sb1
    BL getSignBit
    ldr r5,[r1]
    cmp r5,1
    beq f2_Max
    bne f1_Max
    
    f2_Inf_Case:
    ldr r0,=f2
    ldr r1,=sb2
    BL getSignBit
    ldr r5,[r1]
    cmp r5,1
    beq f1_Max
    bne f2_Max
    
    Not_Nan_or_Inf_Case:
    cmp r5,0
    bne f1_neg
    beq f1_pos
    
    f1_neg:
    cmp r6,0
    bne neg_equal_case
    beq f2_Max
    
    f1_pos:
    cmp r6,0
    beq pos_equal_case
    bne f1_Max
    
    
    pos_equal_case:
    ldr r0,=f1
    ldr r1,=biasedExp1
    ldr r2,=exp1
    BL getExponent
    ldr r8,[r1]
    
    ldr r0,=f2
    ldr r1,=biasedExp2
    ldr r2,=exp2
    BL getExponent
    ldr r9,[r1]
    cmp r8,r9
    beq pos_exp_same_case
    bhi f1_Max
    bls f2_Max
    
    pos_exp_same_case:
    ldr r0,=f1
    ldr r1,=mant1
    BL getMantissa
    ldr r8,[r1]
    ldr r0,=f2
    ldr r1,=mant2
    BL getMantissa
    ldr r9,[r1]
    
    cmp r8,r9
    bhi f1_Max
    bls f2_Max
    
    neg_equal_case:
    ldr r0,=f1
    ldr r1,=biasedExp1
    ldr r2,=exp1
    BL getExponent
    ldr r8,[r1]
    
    ldr r0,=f2
    ldr r1,=biasedExp2
    ldr r2,=exp2
    BL getExponent
    ldr r9,[r1]
    cmp r8,r9
    beq neg_exp_same_case
    bhi f2_Max
    bls f1_Max
    
    neg_exp_same_case:
    ldr r0,=f1
    ldr r1,=mant1
    BL getMantissa
    ldr r8,[r1]
    ldr r0,=f2
    ldr r1,=mant2
    BL getMantissa
    ldr r9,[r1]
    cmp r8,r9
    bhi f2_Max
    bls f1_Max
    
   
    f1_Max:
    ldr r7,=fMax
    ldr r8,=f1
    ldr r8,[r8]
    str r8,[r7]
    b Max_Var_Case
    
    f2_Max:
    ldr r7,=fMax
    ldr r8,=f2
    ldr r8,[r8]
    str r8,[r7]
    b Max_Var_Case
    
    Max_Var_Case:
    ldr r0,=fMax
    ldr r1,=signBitMax
    BL getSignBit
    
    ldr r0,=fMax
    ldr r1,=biasedExpMax
    ldr r2,=expMax
    BL getExponent
    
    ldr r8,[r1]
    cmp r8,0
    beq BEM_0_case
    b   BEM_not_0_case
    BEM_0_case:
    ldr r7,=expMax
    mov r8,-126
    str r8,[r7]
    b BEM_case_done
    
    BEM_not_0_case:
    ldr r8,[r2]
    ldr r7,=expMax
    str r8,[r7]
    
    BEM_case_done:
    ldr r0,=fMax
    ldr r1,=mantMax
    BL getMantissa
    
    done:
    ldr r7,=fMax
    mov r0,r7
    pop {r4-r11,LR}
    mov pc,lr
    /* YOUR asmFmax CODE ABOVE THIS LINE! ^^^^^^^^^^^^^^^^^^^^^  */

   

/**********************************************************************/   
.end  /* The assembler will not process anything after this directive!!! */
           




