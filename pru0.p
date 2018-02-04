.origin 0  
.entrypoint START  

#define EQEP0     0x48300180
#define EQILAT	  0x48300190
#define EQCTL	  0x483001AA
#define MIDPOINT #0x80000000

#define ENA	0
#define DIR	14
#define PUL	15

.struct ParamsStruct
    // set by cpu
    .u32 quit				// 0  r08
    
    .u8  stop				// 8  r09
    .u8  idle				// 9  r09
    .u8  rev				// 10 r09
    .u8  reserved0			// 11 r09
    
    .u8  shuttle_fw			// 12 r10
    .u8  shuttle_bw			// 13 r10
    .u8  reserved1			// 14 r10
    .u8  reserved2			// 15 r10
    
    .u32 MotorFactorHigh		// 16 r11
    .u32 MotorFactorLow			// 20 r12
    .u32 OversampleFactor		// 24 r13

    // set by ppu
    .u32 SpindlePosition		// 28 r14
    .u32 MotorPosition			// 32 r15
    .u32 MotorOffsetHigh		// 36 r16
    .u32 MotorOffsetLow			// 40 r17
    .u32 IndexLatchError		// 44 r18

    // temp vars, private
    .u32 PrevSpindlePosition		// 48 r19
    .u32 diffCount			// 52 r20
    .u32 checkOffset			// 56 r21
    .u32 curDir				// 60 r22
    .u32 tempVar0			// 64 r23
    .u32 tempVar1			// 68 r24
    .u32 IndexLatch			// 72 r25
    .u32 PrevIndexLatch			// 76 r26
.ends
.assign ParamsStruct, r8, r26, Params

.macro STEP_ONE
.mparam newDir

    qbeq _skip_wait_dir, Params.curDir, newDir
    mov Params.curDir, newDir
    qbeq _set_dir_forw, newDir, #0x00 
    clr r30, r30, DIR
    qba _set_dir_back
_set_dir_forw:
    set r30, r30, DIR
_set_dir_back:            

    // wait
    mov	Params.tempVar0, #0x1000
_wait_dir:
    sub Params.tempVar0, Params.tempVar0, #0x01
    qbne _wait_dir, Params.tempVar0, #0x00

_skip_wait_dir:

    clr r30, r30, ENA
    set r30, r30, PUL
    
    // wait
    mov	Params.tempVar0, #0x180
_wait_pul_high:
    sub Params.tempVar0, Params.tempVar0, #0x01
    qbne _wait_pul_high, Params.tempVar0, #0x00
    
    clr r30, r30, PUL

    // wait
    mov	Params.tempVar0, #0x300
_wait_pul_low:
    sub Params.tempVar0, Params.tempVar0, #0x01
    qbne _wait_pul_low, Params.tempVar0, #0x00
.endm
   
START:  
    // enable ocp master port
    lbco r0, c4, 4, 4
    clr r0, r0, 4
    sbco r0, c4, 4, 4

    // enable motor
    clr r30, r30, ENA

    mov r0, EQCTL
    lbbo r1.w0, r0, 0, 2
    set r1, 12
    clr r1, 13
    sbbo r1.w0, r0, 0, 2

    mov Params.MotorOffsetLow, 0
    mov Params.MotorOffsetHigh, MIDPOINT

    mov r0, EQEP0
    lbbo Params.PrevSpindlePosition, r0, 0, 4
    mov r0, MIDPOINT
    add Params.PrevSpindlePosition, Params.PrevSpindlePosition, r0

    mov Params.MotorPosition, MIDPOINT
    
MAIN_LOOP:
    // read pdu ram
    lbco r8, c24, 0, 6*4
    qbne EXIT, Params.quit, #0x00

    // aquire eqep position
    mov Params.tempVar0, EQEP0
    lbbo Params.SpindlePosition, Params.tempVar0, 0, 4
    mov Params.tempVar0, MIDPOINT
    add Params.SpindlePosition, Params.SpindlePosition, Params.tempVar0
    
    // If shuttle forward or backward is enable go into sub routine
    qbne SHUTTLE_FW, Params.shuttle_fw, #0x00
    qbne SHUTTLE_BW, Params.shuttle_bw, #0x00
 
    // Check for idle and disable driver if on   
    qbne NO_IDLE, Params.idle, #0x01
    set r30, r30, ENA
    qba CONTINUE
NO_IDLE:
    clr r30, r30, ENA

    // If we are stopped do nothing
    qbeq CONTINUE, Params.stop, #0x01

    // Calc absolute diff count of spindle
    qblt DIFF_POS, Params.PrevSpindlePosition, Params.SpindlePosition
    sub Params.diffCount, Params.SpindlePosition, Params.PrevSpindlePosition
    qba DIFF_NEG
DIFF_POS:
    sub Params.diffCount, Params.PrevSpindlePosition, Params.SpindlePosition
DIFF_NEG:
    qbeq CONTINUE, Params.diffCount, #0x00

    // Forward or reverse mode
    qbeq REV_MODE, Params.rev, #0x00
FWD_MODE:
    qblt FORWARD, Params.PrevSpindlePosition, Params.SpindlePosition
    qbgt BACKWARD, Params.PrevSpindlePosition, Params.SpindlePosition
    qba CONTINUE
REV_MODE:
    qblt BACKWARD, Params.PrevSpindlePosition, Params.SpindlePosition
    qbgt FORWARD, Params.PrevSpindlePosition, Params.SpindlePosition
    qba CONTINUE

    // Perform backwards steps
BACKWARD:    
    sub Params.MotorOffsetLow, Params.MotorOffsetLow, Params.MotorFactorLow
    suc Params.MotorOffsetHigh, Params.MotorOffsetHigh, Params.MotorFactorHigh
    
    mov Params.checkOffset, MIDPOINT
    sub Params.checkOffset, Params.checkOffset, Params.OversampleFactor
    qble BACKWARD_CONTINUE, Params.MotorOffsetHigh, Params.checkOffset

BACKWARD_STEPS:
    sub Params.MotorPosition, Params.MotorPosition, #0x01

    mov Params.tempVar1, 0x00
    STEP_ONE Params.tempVar1    

    add Params.MotorOffsetHigh, Params.MotorOffsetHigh, Params.OversampleFactor
    qbgt BACKWARD_STEPS, Params.MotorOffsetHigh, Params.checkOffset

BACKWARD_CONTINUE:
    qbeq CONTINUE, Params.diffCount, #0x00
    sub Params.diffCount, Params.diffCount, #0x01
    qbne BACKWARD, Params.diffCount, #0x00
    qba CONTINUE

    // Perform forwards steps
FORWARD:
    add Params.MotorOffsetLow, Params.MotorOffsetLow, Params.MotorFactorLow
    adc Params.MotorOffsetHigh, Params.MotorOffsetHigh, Params.MotorFactorHigh
    
    mov Params.checkOffset, MIDPOINT
    add Params.checkOffset, Params.checkOffset, Params.OversampleFactor
    qbge FORWARD_CONTINUE, Params.MotorOffsetHigh, Params.checkOffset

FORWARD_STEPS:
    add Params.MotorPosition, Params.MotorPosition, #0x01

    mov Params.tempVar1, 0x01
    STEP_ONE Params.tempVar1
    
    sub Params.MotorOffsetHigh, Params.MotorOffsetHigh, Params.OversampleFactor
    qblt FORWARD_STEPS, Params.MotorOffsetHigh, Params.checkOffset    

FORWARD_CONTINUE:
    qbeq CONTINUE, Params.diffCount, #0x00
    sub Params.diffCount, Params.diffCount, #0x01
    qbne FORWARD, Params.diffCount, #0x00
    qba CONTINUE

SHUTTLE_FW:
    // read pdu ram
    lbco r8, c24, 0, 6*4
    qbne EXIT, Params.quit, #0x00
    
    // Do stuff
    
    qba CONTINUE

SHUTTLE_BW:
    // read pdu ram
    lbco r8, c24, 0, 6*4
    qbne EXIT, Params.quit, #0x00
    
    // Do stuff

    qba CONTINUE
    
CONTINUE:
    // Update for next aquisition
    mov Params.PrevSpindlePosition, Params.SpindlePosition

    mov Params.tempVar0, EQILAT
    lbbo Params.IndexLatch, Params.tempVar0, 0, 4
    qbgt INDEX_LATCH_LARGER, Params.IndexLatch, Params.PrevIndexLatch
    qblt INDEX_LATCH_SMALLER, Params.IndexLatch, Params.PrevIndexLatch
    qba INDEX_LATCH_CONTINUE

INDEX_LATCH_SMALLER:
    sub Params.IndexLatchError, Params.IndexLatch, Params.PrevIndexLatch
    mov Params.PrevIndexLatch, Params.IndexLatch
    qba INDEX_LATCH_CONTINUE

INDEX_LATCH_LARGER:
    sub Params.IndexLatchError, Params.PrevIndexLatch, Params.IndexLatch
    mov Params.PrevIndexLatch, Params.IndexLatch
    qba INDEX_LATCH_CONTINUE

INDEX_LATCH_CONTINUE:    

    // write back to pdu ram
    sbco r14, c24, 6*4, 5*4
    qba MAIN_LOOP
   
EXIT:  
    // disable motor
    set r30, r30, ENA
    // Notify CPU
    mov r31.b0, 19+16  
    halt  

