
#define PRU0

#define PRU_SHARED_MEM_ADDR 0x00012000
#define EQEP0_ADDR          0x48300180

#define MIDPOINT    0x80000000
#define MIDPOINT64  0x8000000000000000UUL


volatile register uint32_t __R30;
volatile register uint32_t __R31;

struct pru_params {
    uint32_t quit;

    uint8_t stop;
    uint8_t idle;
    uint8_t reverse;
    uint8_t reserved1;
    
    uint64_t motor_factor;
    uint64_t oversample_factor;
    uint64_t motor_position;
    uint64_t motor_offset;

    uint32_t spindle_position;
    uint32_t index_latch_error;
};

struct epep_regs {
	uint32_t	poscnt;		    /*	0x00	position counter */
	uint32_t	posinit;		/*	0x04	position counter initialization */
	uint32_t	posmax;		    /*	0x08	maximum position count */
	uint32_t	poscmp;		    /*	0x0C	position compare */
	uint32_t	posilat;		/*	0x10	index position latch */
	uint32_t	posslat;		/*	0x14	strobe position latch */
	uint32_t	poslat;		    /*	0x18	position counter latch */
	uint32_t	utmr;			/*	0x1C	unit timer */
	uint32_t	uprd;			/*	0x20	unit period */
	uint16_t	wdtmr;		    /*	0x24	watchdog timer */
	uint16_t	wdprd;		    /*	0x26	watchdog period */

    #define QSRC1		(1 << 15)
    #define QSRC0		(1 << 14)
    #define SOEN		(1 << 13)
    #define SPSEL		(1 << 12)
    #define XCR		    (1 << 11)
    #define SWAP		(1 << 10)
    #define IGATE		(1 << 9)
    #define QAP		    (1 << 8)
    #define QBP		    (1 << 7)
    #define QIP		    (1 << 6)
    #define QSP		    (1 << 5)
	uint16_t	decctl;		    /*	0x28	decoder control */
    
    #define FREESOFT1	(1 << 15)
    #define FREESOFT0	(1 << 14)
    #define PCRM1		(1 << 13)
    #define PCRM0		(1 << 12)
    #define SEI1		(1 << 11)
    #define SEI0		(1 << 10)
    #define IEI1		(1 << 9)
    #define IEI0		(1 << 8)
    #define SWI		    (1 << 7)
    #define SEL		    (1 << 6)
    #define IEL1		(1 << 5)
    #define IEL0		(1 << 4)
    #define PHEN		(1 << 3)
    #define QCLM		(1 << 2)
    #define UTE		    (1 << 1)
    #define WDE		    (1 << 0)
	uint16_t	epctl;		    /*	0x2A	control register */

    #define CEN		    (1 << 15)
    #define CCPS2		(1 << 6)
    #define CCPS0		(1 << 5)
    #define CCPS1		(1 << 4)
    #define UPPS3		(1 << 3)
    #define UPPS2		(1 << 2)
    #define UPPS1		(1 << 1)
    #define UPPS0		(1 << 0)
	uint16_t	capctl;		    /*	0x2C	capture control */
    
    #define PCSHDW		(1 << 15)
    #define PCLOAD		(1 << 14)
    #define PCPOL		(1 << 13)
    #define PCE		    (1 << 12)
    #define PCSPW11		(1 << 11)
    #define PCSPW10		(1 << 10)
    #define PCSPW9		(1 << 9)
    #define PCSPW8		(1 << 8)
    #define PCSPW7		(1 << 7)
    #define PCSPW6		(1 << 6)
    #define PCSPW5		(1 << 5)
    #define PCSPW4		(1 << 4)
    #define PCSPW3		(1 << 3)
    #define PCSPW2		(1 << 2)
    #define PCSPW1		(1 << 1)
    #define PCSPW0		(1 << 0)
	uint16_t	posctl;		    /*	0x2E	position compare control */

	uint16_t	eint;			/*	0x30	interrupt enable */
	uint16_t	flg;			/*	0x32	interrupt flag */
	uint16_t	clr;			/*	0x34	interrupt clear */
	uint16_t	frc;			/*	0x36	interrupt force */
	uint16_t	epsts;		    /*	0x38	status */
	uint16_t	ctmr;			/*	0x3A	capture timer */
	uint16_t	cprd;			/*	0x3C	capture period */
	uint16_t	ctmrlat;		/*	0x3E	capture timer latch */
	uint16_t	prdlat;		    /*	0x40	capture period latch */
	uint8_t	    fill1[0x5c-0x40];
	uint32_t	revid;		    /*	0x5C	revision id */
};

#define ENABLE_PIN	    2
#define DIRECTION_PIN	14
#define PULSE_PIN 	    15

static void wait(uint32_t count) {
    for (volatile uint32_t i = 0;  i < count; c++) {}
}

void step_one(volatile pru_params *params, int direction) {
    if (params.current_direction != direction) {
        if (direction) {
            __R30 |=  (1UL<<DIRECTION_PIN);
        } else {
            __R30 &= ~(1UL<<DIRECTION_PIN);
        }
        wait(0x1000);
    }
    
    __R30 |=  (1UL<<PULSE_PIN);

    wait(0x180);
    
    __R30 &= ~(1UL<<PULSE_PIN);
    
    wait(0x300);
}

void main(){

	volatile pru_params *params = (volatile pru_params *) PRU_SHARED_MEM_ADDR;
    volatile epep_regs *eqep0 = (volatile epep_regs *) EQEP0;

	// enable OCP
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    // enable motor
    __R30 &= ~(1UL<<ENABLE_PIN);
    
    // configure qep0
    eqep0->epctl |=  (1UL<<PCRM0);
    eqep0->epctl &= ~(1UL<<PCRM1);
    
    params.motor_offset = MIDPOINT;
    
    uint32_t prev_spindle_position = eqep0->poscnt + MIDPOINT;
    params.motor_position = MIDPOINT;
    
	while(!params.quit) {

        // always update spindle positions
        prev_spindle_position = params->spindle_position;
        
        // read and store once
        params->spindle_position = eqep0->poscnt + MIDPOINT;
        
        if (params->idle) {

            // disable motor in idle mode
            __R30 &= ~(1UL<<ENABLE_PIN);

            // nothing more to do
            continue;
        }

        // enable motor for normal processing
        __R30 |= (1UL<<ENABLE_PIN);
        
        if (params->stop) {
            // do nothing
            continue;
        }
        
        int32_t diff = params->spindle_position - prev_spindle_position;
        
        if (diff == 0) {
            // do nothing
            continue;
        }
        
        // move backwards?
        if (params.reverse) {
            diff = -diff;
        }
        
        if (diff < 0) {
            params->motor_offset -= params->motor_factor;
            
            for (uint64_t check_offset = MIDPOINT64 - params->oversample_factor; params->motor_offset < check_offset ;) {
                params->motor_position --;
                step_one(0);
                params->motor_offset += param->oversample_factor;
            }
        } else {
            params->motor_offset += params->motor_factor;

            for (uint64_t check_offset = MIDPOINT64 + params->oversample_factor; params->motor_offset > check_offset ;) {
                params->motor_position ++;
                step_one(0);
                params->motor_offset -= param->oversample_factor;
            }
        }
    }

    // disable motor in idle mode
    __R30 &= ~(1UL<<ENABLE_PIN);

    // notify cpu
    __R31 &= !(0xFF);
    __R31 |=  (19+16);
    
	__halt();

	return 0;
}
