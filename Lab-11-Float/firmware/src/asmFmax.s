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
    /* save the caller's registers, as required by the ARM calling convention */
    push {r4-r11,LR}
    /**This is for f1 and its variables**/
    mov r10,0	    /**storing 0 in r10 for initializing varliables to 0**/    
    ldr r4,=f1	    /**storing memory location of f1 in r4**/
    str r0,[r4]	    /**storing f1, values which is located at r0, in memory location of f1**/
    ldr r4,=sb1	    /**storing memory location of sb1 in r4**/
    str r10,[r4]    /**storing 0 in memory locaiton of sb1**/
    ldr r4,=biasedExp1	/**storing memory location of biasedExp1 in r4**/
    str r10,[r4]	/**storing 0 in memory location of biasedExp1**/
    ldr r4,=exp1	/**storing memory location of exp1 in r4**/
    str r10,[r4]	/**storing 0 in memory location of exp1**/
    ldr r4,=mant1	/**storing memory location of mant1 in r4**/
    str r10,[r4]	/**storing 0 in memory location of mant1**/
    /**This is for f2 and its variables**/
    ldr r4,=f2		/**storing memory location fo f2 in r4**/
    str r1,[r4]		/**storing f2 value, which is located in r1, in memory location of f2**/
    ldr r4,=sb2		/**storing memory location of sb2 in r4**/
    str r10,[r4]	/**storing 0 in memory location of sb2**/
    ldr r4,=biasedExp2	/**storing memory location of biasedExp2 in r4**/
    str r10,[r4]	/**storing 0 in memory location of biasedExp2**/
    ldr r4,=exp2	/**storing memory location of exp2 in r4**/
    str r10,[r4]	/**storing 0 in memory location of exp2**/
    ldr r4,=mant2	/**storing memory location of mant2 in r4**/
    str r10,[r4]	/**storing 0 in memory location of mant2**/
    /**This is for fMax and its varibales**/
    ldr r4,=fMax	/**stoiring memory location of fMax in r4**/
    str r10,[r4]	/**storing 0 in memory location of fMax**/
    ldr r4,=signBitMax	/**storing memory location of signBitMax in r4**/
    str r10,[r4]	/**storing 0 in memory location of signBitMax**/
    ldr r4,=biasedExpMax/**storing memory location of biasedExpMax in r4**/
    str r10,[r4]	/**storing 0 in memory location of biasedExpMax**/
    ldr r4,=expMax	/**storing memory location of expMax in r4**/
    str r10,[r4]	/**storing 0 in memory location of expMax**/
    ldr r4,=mantMax	/**storing memory location of mantMax in r4**/
    str r10,[r4]	/**storing 0 in memory location of mantMax**/
    /**restore the caller's registers, as required by the ARM calling convention**/
    pop {r4-r11,LR}
    mov pc, lr		/**initVariables return to caller**/
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
    /* save the caller's registers, as required by the ARM calling convention */
    push {r4-r11,LR}
    
    mov r11,1		/**storing 1 in r11 for the negative sign bit**/
    mov r10,0		/**storing 0 in r10 for the postive sign bit**/
    mov r4,0x80000000	/**storing 0x80000000 in r4, this is to get the most significant bit, which is the sign bit**/
    ldr r5,[r0]		/**storing the 32b float packed values which is in input r0 in r5**/
    ands r5,r5,r4	/**using and operation, r5 and r4, to get the sign bit and stores it in r5, then also updates the flag**/
    movmi r5,r11	/**if the negative flag is set store 1 in r5**/
    movpl r5,r10	/**if the negative flag is not set, store 0 in r5**/
    str r5,[r1]		/**store the value in r5 to output r1, mem location given by r1**/
    /**restore the caller's registers, as required by the ARM calling convention**/
    pop {r4-r11,LR}
    mov pc, lr		/**getSignBit return to caller**/
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
    /* save the caller's registers, as required by the ARM calling convention */
    push {r4-r11,LR}
    
    ldr r4,=0x7F800000	/**storing the 0x7F800000 in r4, which will be used to get the exponent value**/
    ldr r5,[r0]		/**storing the value which is located in mem location addressed by input r0 in r5**/
    AND r5,r5,r4	/**using and operation, r5 and r4, and store the result in r5**/
    lsr r5,r5,23	/**shifting r5 to the right 23 bits, and store the result in r5**/
	
    str r5,[r1]		/**store the result value in r5, in the memory location addressed by input r1. This is the biased exponent**/
    cmp r5,0		/**compare the biased exponent value with 0**/
    beq biased_0	/**if biased exponent is equal to 0, then direct to the biased_0 branch**/
    bne biased_not_0	/**if not, direct to the biased_not_0 branch**/
    /**This is for the case when biased exponent equals to 0. When the biased exponent is 0, the unbiased exponent value
     will be -126 instead of -127**/
    biased_0:
    mov r5,-126		/**storing -126 in r5**/
    str r5,[r2]		/**storing the value in r5, -126,  in the memory location addressed by input r2**/
    b done_getExponent	/**direct to the done_getExponent branch**/
    /**This is for the case when biased exponent not equal to 0. unbiased = biased - 127**/
    biased_not_0:
    sub r5,r5,127	/**To get the unbiased exponent, subtract 127 from r5, and store it in r5**/
    str r5,[r2]		/**store the result value in r5, in the memory location addressed by input r2. This is the unbiased exponent**/
    
    done_getExponent:	/**This is the end of the getExponent function**/
    /**restore the caller's registers, as required by the ARM calling convention**/
    pop {r4-r11,LR}
    mov pc, lr		/**getExponent return to caller**/
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
    /* save the caller's registers, as required by the ARM calling convention */
    push {r4-r11,LR}
    
    ldr r6,=0x7FFFFF	    /**storing 0x7FFFFF in r6, which will be used to get the mantissa value**/
    ldr r7,=0x7F800000	    /**storing 0x7F800000 in r7, which will be used to check whether all the exponent bit are set or not**/
    ldr r8,=0x800000	    /**storing 0x800000 in r8, which will be used to add 24th implied bit**/
    
    ldr r4,[r0]		    /**storing the value located in the mem location addressed by r0 in r4**/
    mov r5,r4		    /**storing values in r4 to the r5**/
    and r4,r4,r6	    /**using and operation, r4 and r6(0x7FFFFF), to get the mantissa value**/
    
    and r5,r5,r7	    /**using and operation, r5 and r7(0x7F800000), to check get the exponent bit**/
    cmp r5,0		    /**compare r5 with 0**/
    bne setbit23_1	    /**if any of the exponent bits are set, and not equal to 0, direct to the setbit23_1 branch**/
    beq setbit23_0	    /**if all of the expenent bits are not m and euqal to 0, direct to the setbit23_0 branch**/
    
    /**This is for adding 24 implied bit, and assign it to 1**/
    setbit23_1:
    cmp r5,r7		    /**compare the exponent bits which  is in r5 with r7(0x7F800000)**/
    cmpeq r4,0		    /**if the exponent bits is equal to 0x7F800000, compare r4(mantissa value) with 0**/
    beq this_is_inf_case    /**if r4 is equal to 0, that means the number is infinity, so direct to the this_is_inf_case branch**/
    orr r4,r4,r8	    /**if not, using orr operation, r4 or r8, to add the 24 implied bit which is 1 in matissa value, the store the result in r4**/
    str r4,[r1]		    /**store the result value in mem location addressed by input r1**/
    b done_for_sign	    /**directing to the done_for_sign branch**/
    /**This is for adding 24 implied bit, and assign it to 0**/
    setbit23_0:
    str r4,[r1]		    /**Since we don't need to change, store the value r4 in the mem location addressed by input r1**/
    b done_for_sign	    /**directing to the done_for_sign branch**/
    /**This is for the case when the number is infinity**/
    this_is_inf_case:
    mov r4,0		    /**storing 0 in r4**/
    str r4,[r1]		    /**storing 0 in mem location addressed by input r1**/
    
    done_for_sign:	    /**end of the getMantissa method**/
    /**restore the caller's registers, as required by the ARM calling convention**/
    pop {r4-r11,LR}
    mov pc,lr		    /**getMantissa return to caller**/
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
    /* save the caller's registers, as required by the ARM calling convention */
    push {r4-r11,LR}
    mov r10,0		/**storing 0 in r10 to mamipulate the variables later**/
    
    BL initVariables	/**initiliazing all the variables by calling initVariables function**/
    
    ldr r0,=f1		/**store the address of f1 in r0**/
    ldr r1,=sb1		/**store the address of sb1 in r1**/
    BL getSignBit	/**call the getSignBit function**/
    ldr r5,[r1]		/**store the output value, which is the sb1 value, from getSignBit in r5**/
    
    ldr r0,=f2		/**store the address of f2 in r0**/
    ldr r1,=sb2		/**store the address of sb2 in r1**/
    BL getSignBit	/**call the getSignBit function**/
    ldr r6,[r1]		/**store the output value, which is the sb2 value, from getSignBit in r6**/
    
    /**This is for checking f1 number wheter it is NaN or Infinity, or neither of them**/
    check_Nan_Or_Inf:
    ldr r0,=f1		/**store the address of f1 in r0**/
    ldr r1,=biasedExp1	/**store the address of biasedExp1 in r1**/
    ldr r2,=exp1	/**store the address of exp1 in r2**/
    BL getExponent	/**call the getExponent function**/
    ldr r7,[r1]		/**store the output value, which is biasedExp1,from getExponent function in r7**/
    cmp r7,0xFF		/**compare the value in r7 with 0xFF**/
    beq f1_Nan_or_Inf	/**if r7 and 0xFF are equal, direct to the f1_Nan_or_Inf branch**/
    bne check_f2_Nan_or_Inf /**if not equal, direct to the check_f2_Nan_or_Inf branch**/
    /**This is for checking f2 number wheter it is NaN or Infinity, or neither of them**/
    check_f2_Nan_or_Inf:
    ldr r0,=f2		/**store the address of f2 in r0**/
    ldr r1,=biasedExp2	/**store the address of biasedExp2 in r1**/
    ldr r2,=exp2	/**store the address of exp2 in r2**/
    BL getExponent	/**call the getExponent function**/
    ldr r7,[r1]		/**store the output value, which is biasedExp2, from getExpoent function in r7**/
    cmp r7,0xFF		/**compare the value in r7 with 0xFF**/
    beq f2_Nan_or_Inf	/**if r7 and 0xFF are equal, direct to the f2_Nan_or_Inf branch**/
    bne Not_Nan_or_Inf_Case /**if not equal, direct to the Not_Nan_or_Inf_Case bracnh**/
    /**This is checking f1 is NaN or Inf**/
    f1_Nan_or_Inf:
    ldr r0,=f1		/**storing the address of f1 in r0**/
    ldr r1,=mant1	/**storing the adress of mant1 in r1**/
    BL getMantissa	/**call the getMantissa funciton**/
    ldr r8,[r1]		/**store the output value, which is the mantissa value of f1, from getMantissa function in r8**/
    cmp r8,0x00400000	/**compare the value in r8 with 0x00400000**/
    beq Nan_Case	/*if r8 and 0x00400000 are equal, then the f1 is NaN, so direct to the Nan_Case branch**/
    bne f1_Inf_Case	/**if not equal, f1 is infity, direct to the f1_Inf_Case**/
    /**This is checking f2 is NaN or Inf**/
    f2_Nan_or_Inf:	
    ldr r0,=f2		/**store the address of f2 in r0**/
    ldr r1,=mant2	/**store the address of mant2 in r1**/
    BL getMantissa	/**call the getMantissa function**/
    ldr r8,[r1]		/**store the output value, which is the mantissa value of f2, from getMantissa in r8**/
    cmp r8,0x00400000	/**compare the value in r8 with 0x00400000**/
    beq Nan_Case	/**if r8 and 0x00400000 are equal, f2 is NaN, so direct to the Nan_Case branch**/
    bne f2_Inf_Case	/**if not equal, f2 is infinity, direct to the f2_Inf_Case**/
    
    /**This is for the case when either f1 or f2 is Nan**/
    Nan_Case:
    ldr r7,=fMax	/**store the address of fMax in r7**/
    mov r8,0x7FFFFFFF	/**store 0x7FFFFFFF in r8**/ 
    str r8,[r7]		/**store 0x7FFFFFFF in mem location of fMax**/
    b Max_Var_Case	/**direct to the Max_Var_Case branch**/
    /**This is for the case when f1 is infinity**/
    f1_Inf_Case:
    ldr r0,=f1		/**store the address of f1 in r0**/
    ldr r1,=sb1		/**store the address of sb1 in r1**/
    BL getSignBit	/**call the getSignBit function**/
    ldr r5,[r1]		/**store the output value, whcih is the Sign bit of f1, from the getSignBit function in r5**/
    cmp r5,1		/**compare the sign bit of f1 with 1**/
    beq f2_Max		/**If the sign bit of f1 equal to 1, f1 is negative inf, so f2 will the Max value**/
    bne f1_Max		/**If not equal, f1 is postivie inf, so f1 will be the Max value**/
    /**This is for the case when f2 is infinity**/
    f2_Inf_Case:
    ldr r0,=f2		/**store the address of f2 in r0**/
    ldr r1,=sb2		/**store the address of sb2 in r1**/
    BL getSignBit	/**call the getSignBit function**/
    ldr r5,[r1]		/**store the output value, whcih is the Sign bit of f2, from the getSignBit function in r5**/
    cmp r5,1		/**compare the sign bit of f1 with 1**/
    beq f1_Max		/**If the sign bit of f2 equal to 1, f2 is negative inf, so f1 will the Max value**/
    bne f2_Max		/**If not equal, f2 is postivie inf, so f2 will be the Max value**/
    /**This is for the case when both of f1 and f2 are not NaN or Inf**/
    Not_Nan_or_Inf_Case:
    cmp r5,0		/**compare the values in r5, which is the sign bit for f1, with 0**/
    bne f1_neg		/**if r5 not equal to 0, then f1 is negative, and direct to the f1_neg branch**/
    beq f1_pos		/**if not, f1 is positive, and direct to the f1_pos branch**/
    /**This if for the case when f1 is not NaN and Inf, and f1 is negative**/
    f1_neg:
    cmp r6,0		/**compare the values in r6, which is the sign bit for f2, with 0**/
    bne neg_equal_case	/**if r6 not equal to 0, then f2 is negative, so direct to the neg_equal_case branch**/
    beq f2_Max		/**if not, f2 is positive, and direct to the f2_Max branch**/
    /**This if for the case when f1 is not NaN and Inf, and f1 is positive**/
    f1_pos:
    cmp r6,0		/**compare the values in r6, which is the sign bit for f2, with 0**/
    beq pos_equal_case	/**if r6 equal to 0, then f2 is positive, and direct to the pos_equal_case branch**/
    bne f1_Max		/**if not, f2 is negative, and direct to the f1_Max branch**/
    
    /**This is for the case when the both f1 and f2 are positive, and not Nan or Inf. check the exponent of f1 and f2,
     then compare, pick the greater one for fMax, if expoent are equals, direct to the pos_exp_same_case. **/
    pos_equal_case:
    ldr r0,=f1		/**store the address of f1 in r0**/
    ldr r1,=biasedExp1	/**store the address of biasedExp1 in r1**/
    ldr r2,=exp1	/**store the address of exp1 in r2**/
    BL getExponent	/**call the getExponent fucntion**/
    ldr r8,[r1]		/**store the output value, which is biasedExp1, in r8**/
    
    ldr r0,=f2		/**store the address of f2 in r0**/
    ldr r1,=biasedExp2	/**store the address of biasedExp2 in r1**/
    ldr r2,=exp2	/**store the address of exp2 in r2**/
    BL getExponent	/**call the getExponent fucntion**/
    ldr r9,[r1]		/**store the output value, which is biasedExp2, in r9**/
    cmp r8,r9		/**compare biasedExp1 with biasedExp2**/
    beq pos_exp_same_case   /**if biasedExp1 and biasedExp1 are equal, direct to the pos_exp_same_case branch**/
    bhi f1_Max		/**if biasedExp1 is higher, then direct to the f1_Max branch**/
    bls f2_Max		/**if biasedExp1 is lower, then direct to the f2_Max branch**/
    
    /**This is for the case when the exponents value of f1 and f2 are equal. Check the mantissa of f1 and f2, 
     compare them, and pick the greater one for fMax**/
    pos_exp_same_case:
    ldr r0,=f1		/**store the address of f1 in r0**/
    ldr r1,=mant1	/**store the address of mant1 in r1**/
    BL getMantissa	/**call the getMantissa function**/
    ldr r8,[r1]		/**store the output value, which is mantissa value of f1, in r8**/
    ldr r0,=f2		/**store the address of f2 in r0**/
    ldr r1,=mant2	/**store the address of mant2 in r1**/
    BL getMantissa	/**call the getMantissa function**/
    ldr r9,[r1]		/**store the output value, which is mantissa value of f2, in r9**/
    
    cmp r8,r9		/**compare the mantissa value of f1 and f2**/
    bhi f1_Max		/**if the mantissa value of f1 is higher, then direct to the f1_Max branch**/
    bls f2_Max		/**if the mantissa value of f2 is lower or equal, then direct to the f2_Max branch**/
    
    /**This is for the case when the both f1 and f2 are negative, and not Nan or Inf. check the exponent of f1 and f2,
     then compare, pick the lower one for fMax(Since this is negative case), if expoent are equals, 
     direct to the neg_exp_same_case. **/
    neg_equal_case:
    ldr r0,=f1		/**store the address of f1 in r0**/
    ldr r1,=biasedExp1	/**store the address of biasedExp1 in r1**/
    ldr r2,=exp1	/**store the address of exp1 in r2**/
    BL getExponent	/**call the getExponent fucntion**/
    ldr r8,[r1]		/**store the output value, which is biasedExp1, in r8**/
    
    ldr r0,=f2		/**store the address of f2 in r0**/
    ldr r1,=biasedExp2	/**store the address of biasedExp2 in r1**/
    ldr r2,=exp2	/**store the address of exp2 in r2**/
    BL getExponent	/**call the getExponent fucntion**/
    ldr r9,[r1]		/**store the output value, which is biasedExp2, in r9**/
    cmp r8,r9		/**compare biasedExp1 with biasedExp2**/
    beq neg_exp_same_case   /**if biasedExp1 and biasedExp1 are equal, direct to the neg_exp_same_case branch**/
    bhi f2_Max		/**if biasedExp1 is higher, direct to the f2_Max branch**/
    bls f1_Max		/**if biasedExp1 is lower, direct to the f1_Max branch**/
    
    /**This is for the case when the exponents value of f1 and f2 are equal. Check the mantissa of f1 and f2, 
     compare them, and pick the lower one for fMax**/
    neg_exp_same_case:
    ldr r0,=f1		/**store the address of f1 in r0**/
    ldr r1,=mant1	/**store the address of mant1 in r1**/
    BL getMantissa	/**call the getMantissa function**/
    ldr r8,[r1]		/**store the output value, which is mantissa value of f1, in r8**/
    ldr r0,=f2		/**store the address of f2 in r0**/
    ldr r1,=mant2	/**store the address of mant12 in r1**/
    BL getMantissa	/**call the getMantissa function**/
    ldr r9,[r1]		/**store the output value, which is mantissa value of f2, in r9**/
    cmp r8,r9		/**compare the mantissa value of f1 and f2**/
    bhi f2_Max		/**if the mantissa value of f1 is higher, then direct to the f2_Max branch**/
    bls f1_Max		/**if the mantissa value of f2 is lower or equal, then direct to the f1_Max branch**/
    
   /**This is for the case when f1 is the maximum number**/
    f1_Max:
    ldr r7,=fMax	/**store the address of fMax in r7**/
    ldr r8,=f1		/**store the address of f1 in r8**/
    ldr r8,[r8]		/**store the value of f1 in r8**/
    str r8,[r7]		/**store the f1 in mem location of fMax**/
    b Max_Var_Case	/**direct to the Max_Var_Case**/
    /**This is for the case when f2 is the maximum number**/
    f2_Max:
    ldr r7,=fMax	/**store the address of fMax in r7**/
    ldr r8,=f2		/**store the address of f2 in r8**/
    ldr r8,[r8]		/**store the value of f2 in r8**/
    str r8,[r7]		/**store the f2 in mem location of fMax**/
    b Max_Var_Case	/**direct to the Max_Var_Case**/
    /**This is for finding signBitMax, biasedExpMax, expMax, and mantMax based on the result of fMax we get**/
    Max_Var_Case:
    ldr r0,=fMax	/**store the address of fMax in r0**/
    ldr r1,=signBitMax	/**store the address of signBitMax in r1**/
    BL getSignBit	/**call the getSignBit function to get the signBitMax value**/
    
    ldr r0,=fMax	/**store the address of fMax in r0**/
    ldr r1,=biasedExpMax    /**store the address of biasedExpMax in r1**/
    ldr r2,=expMax	/**store the address of expMax in r2**/
    BL getExponent	/**call the getExponent function to get biasedExpMax, and expMax values**/
    
    ldr r0,=fMax	/**store the address of fMax in r0**/
    ldr r1,=mantMax	/**store the address of mantMax in r1**/
    BL getMantissa	/**call the getMantissa function to get the mantMax value**/
    
    done:		/**This is the end of the asmFmax function**/
    /**restore the caller's registers, as required by the ARM calling convention**/
    pop {r4-r11,LR}
    mov pc,lr		/**asmFmax return to caller**/
    
    /* YOUR asmFmax CODE ABOVE THIS LINE! ^^^^^^^^^^^^^^^^^^^^^  */

   

/**********************************************************************/   
.end  /* The assembler will not process anything after this directive!!! */
           




