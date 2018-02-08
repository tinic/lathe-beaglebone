#include <stdio.h>  
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <memory.h>
#include <math.h>
#include <time.h>
#include <prussdrv.h>  
#include <pruss_intc_mapping.h>  
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <linux/fb.h>

#include "easyBlack/src/memGPIO.hpp"

easyBlack::memGPIO gpio;
easyBlack::memGPIO::gpioPin gpioCD;

#if 0
static int spiFD = 0;

static bool spi_open() {
	spiFD = open("/dev/spidev1.0", O_RDWR);
	if (spiFD < 0) {
		return false;
	}
	int nLSB = 0;
	if (ioctl(spiFD, SPI_IOC_WR_LSB_FIRST, &nLSB) < 0) {

		return false;
	}
	int nSpeed = 2000000;
	if (ioctl(spiFD, SPI_IOC_WR_MAX_SPEED_HZ, &nSpeed) < 0) {

		return false;
	}
	if (ioctl(spiFD, SPI_IOC_RD_MAX_SPEED_HZ, &nSpeed) < 0) {

		return false;
	}
	int nSPIMode = SPI_MODE_0;
  	if (ioctl(spiFD, SPI_IOC_WR_MODE, &nSPIMode) < 0) {

		return false;
	}
	int nBits = 8;
        if(ioctl(spiFD, SPI_IOC_WR_BITS_PER_WORD, &nBits) < 0) {

                return false;
        }
	return true;
}

static bool spi_write(const uint8_t *in, const uint8_t *out, int n) {
  	struct spi_ioc_transfer xfer = {0};
	xfer.tx_buf = (uintptr_t)in;
	xfer.rx_buf = (uintptr_t)out;
	xfer.len = n;
	xfer.delay_usecs = 0;
	if (ioctl(spiFD, SPI_IOC_MESSAGE(1), &xfer) < 0) {
		return false;
	}
	return true;
}
#endif

#define ENCODER_STEPS_PER_REV             2880
#define MOTOR_STEPS_PER_REV               3200
#define LEAD_SCREW_ROTATIONS_PER_INCH     12
#define MOTOR_OVERSAMPLE_FACTOR           double(1L<<26)
#define MOTOR_PITCH_MM			  (25.4*(1/double(LEAD_SCREW_ROTATIONS_PER_INCH)))

const int FONT_WIDTH = 9;
const int FONT_HEIGHT = 9;

const int TEXT_WIDTH = 50;
const int TEXT_HEIGHT = 30;

// Screen dimension constants
const int SCREEN_WIDTH = 480;
const int SCREEN_HEIGHT = 272;

uint8_t font[256*256] = { 0 };
uint8_t text_screen[TEXT_WIDTH * TEXT_HEIGHT * 2] = { 0xFF };
uint16_t pixel_screen[2][SCREEN_WIDTH * SCREEN_HEIGHT * 2] = { 0 };

uint16_t ega_palette[] = {
    0x0000,
                  0x0015,
           0x0540,
           0x0540|0x0015,
    0xa800,
    0xa800       |0x0015,
    0xa800|0x02a0,
    0xa800|0x0540|0x0015,
    0x5000|0x02a0|0x000a,
    0x5000|0x02a0|0x001f,
    0x5000|0x07e0|0x000a,
    0x5000|0x07e0|0x001f,
    0xf800|0x02a0|0x000a,
    0xf800|0x02a0|0x001f,
    0xf800|0x07e0|0x000a,
    0xf800|0x07e0|0x001f,
/*
	0x000000,
	0x0000AA,
	0x00AA00,
	0x00AAAA,
	0xAA0000,
	0xAA00AA,
	0xAA5500,
	0xAAAAAA,
	0x555555,
	0x5555FF,
	0x55FF55,
	0x55FFFF,
	0xFF5555,
	0xFF55FF,
	0xFFFF55,
	0xFFFFFF
*/
};

enum {
    mode_metric_normal,
    mode_metric_extended,
    mode_sae_normal,
    mode_sae_extended,
    mode_count
};

float options_val[][12] = {
    { 0.025, 0.05, 0.10, 0.20, 0.50, 0.70, 0.80, 1.00, 1.25, 1.50, 1.75, 2.00 } ,
    { 0.01, 0.04, 0.15, 0.35, 0.40, 0.45, 2.50, 3.00, 3.50, 4.00, 4.50, 5.00 } ,
    { 0.0254, 0.1270, 0.2540, .79375, 1.05833333, 1.270, 1.4111111, 1.5875, 1.814, 2.117, 2.309, 2.540 } ,
    { .3175, .3527777777, .396875, .45357142857, .529166666, .57727272727, 0.635, .7055555555555, 1.155,  1.954, 2.822, 3.175 }
}; 

const char *options_str[][12] = {
    { " .025 ", " 0.05 ", " 0.10 ", " 0.20 ", " 0.50 ", " 0.70 ", " 0.80 ", " 1.00 ", " 1.25 ", " 1.50 ", " 1.75 ", " 2.00 " } ,
    { " 0.01 ", " 0.04 ", " 0.15 ", " 0.35 ", " 0.40 ", " 0.45 ", " 2.50 ", " 3.00 ", " 3.50 ", " 4.00 ", " 4.50 ", " 5.00 " } ,
    { "\xff"".001", "\xff"".005 ", "\xff"".010 ", "  32  ", "  24  ", "  20  ", "  18  ", "  16  ", "  14  ", "  12  ", "  11  ", "  10  " } ,
    { "  80 ", "  72  ", "  64  ", "  56  ", "  48  ", "  44  ", "  40  ", "  36  ", "  22  ", "  13  ", "   9  ", "   8  " }
};

static uint8_t mode = mode_metric_normal;
static uint8_t option[mode_count] = {
    2,
    0,
    1,
    1
};

struct __attribute__((packed, aligned(4))) Params {
    uint32_t quit;

    uint8_t  stop;
    uint8_t  idle;    
    uint8_t  rev;
    uint8_t  reserved0;
    
    uint8_t  shuttle_fw;
    uint8_t  shuttle_bw;   
    uint8_t  reserved1; 
    uint8_t  reserved2; 

    uint32_t motorFactorHigh;
    uint32_t motorFactorLow;
    uint32_t oversampleFactor;

    uint32_t spindlePosition;
    uint32_t motorPosition;
    uint32_t motorOffsetHigh;
    uint32_t motorOffsetLow;
    uint32_t indexLatch;

    uint32_t prevSpindlePosition;
    uint32_t diffCount;
    uint32_t checkOffset;
    uint32_t curDir;
};

static Params *params = 0;

bool inTouch = false;
int32_t xTouch = 0;
int32_t yTouch = 0;
int32_t zTouch = 0;

int64_t MotorSpeedFactorOversampled = 0;
int64_t MotorSpeedOversampleFactor = 0;

time_t startTime = { 0 };
static double JiffyClock = 0;
static int32_t MotorOffset = 0;
static int32_t RotaryEncoderCounter = 0;
static int32_t StepperOffsetCounter = 0;

float stop	= 0.0f;
float idle	= 0.0f;
float dir   	= 1.0;

double rpm   	= 0.0;
double pitch 	= 0.0; // always metric
double posz  	= 0.0; // always metric
double posc  	= 0.0; // always metric
double post	= 0.0;

#define Y_OFF 2

void load_font() {
    FILE *file = fopen("font_480_272.raw","rb");
    if (file) {
        fread(font, FONT_WIDTH * FONT_HEIGHT * 256, 1, file);
        fclose(file);
    }
}

void load_metric() {
    FILE *file = fopen("screen.bin","rb");
    if (file) {
        fread(text_screen, TEXT_WIDTH * TEXT_HEIGHT * 2, 1, file);
        fclose(file);
    }
}

void map_screen(uint16_t *dst) {
    for (uint32_t y = 0; y < TEXT_HEIGHT; y++) {
        for (uint32_t x = 0; x < TEXT_WIDTH; x++) {
            uint8_t chr = text_screen[(y * TEXT_WIDTH + x)*2+0];
            uint8_t col = text_screen[(y * TEXT_WIDTH + x)*2+1];
            uint8_t *s = (uint8_t *)&font[(chr/16) * FONT_WIDTH * FONT_HEIGHT * 16 + (chr%16) * FONT_WIDTH];
            for (uint32_t yy = 0; yy < FONT_HEIGHT; yy++) {
                for (uint32_t xx = 0; xx < FONT_WIDTH; xx++) {
                    uint16_t *d = &dst[(y * FONT_WIDTH + yy) * SCREEN_WIDTH + (x * FONT_WIDTH + xx)];
                    if(*s++) {
                        *d = ega_palette[col%16];//(ega_palette[col%16]<<8) | (ega_palette[col%16]>>8);
                    } else {
                        *d = ega_palette[col/16];//(ega_palette[col/16]<<8) | (ega_palette[col/16]>>8);
                    }
                }
                s += FONT_WIDTH * 16 - FONT_WIDTH;
            }
        }
    }
}

void place_text(const char *str, uint32_t x, uint32_t y) {
    for (size_t c = 0; c < strlen(str); c++) {
        size_t index = (y*TEXT_WIDTH+x+c)*2;
        if (index < TEXT_WIDTH*TEXT_HEIGHT*2) {
            text_screen[index] = str[c];
        }
    }
}

void fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t col) {
    for (uint32_t yy = y; yy < y + h; yy++ ) {
        for (uint32_t xx = x; xx < x + w; xx++ ) {
            size_t index = (yy*50+xx)*2+1;
            if (index < TEXT_WIDTH*TEXT_HEIGHT*2) {
                text_screen[index] = col;
            }
        }
    }
}

void option_rect(uint32_t o, uint32_t &x, uint32_t &y, uint32_t &w, uint32_t &h) {
    x = o%4 * 9 + 1;
    y = Y_OFF + 8 + o/4 * 4;
    w = 8;
    h = 3;
}

void update_options() {
    for (int32_t c = 0; c < 12; c++) {
        place_text(options_str[mode][c], 2 + c%4 * 9, Y_OFF + 9 + c/4 * 4);
        uint32_t x, y, w, h;
        option_rect(c, x, y, w, h);
        fill_rect(x, y, w, h, (option[mode] == c) ? 0xcf : 0x1f);
    }
}

void update_stats() {

    bool metric = true;

    pitch = options_val[mode][option[mode]];
    
    float posz_converted = posz;
    float pitch_converted = pitch;

    switch(mode) {
        case mode_metric_normal:
        case mode_metric_extended:
            metric = true;
            fill_rect(20,1 + Y_OFF,8,6, 0xcf);
            fill_rect(29,1 + Y_OFF,8,6, 0x1f);
            break;
        case mode_sae_normal:
        case mode_sae_extended:
            // convert to SAE
            pitch_converted = (pitch_converted > 0.0001) ? (25.4 / pitch_converted) : 0.0f;
            posz_converted = posz_converted / 25.4;
            fill_rect(20,1 + Y_OFF,8,6, 0x1f);
            fill_rect(29,1 + Y_OFF,8,6, 0xcf);
            metric = false;
            break;
    }

    switch(mode) {
        case mode_metric_normal:
        case mode_sae_normal:
			place_text("1",34,19 + Y_OFF);
			break;
        case mode_metric_extended:
        case mode_sae_extended:
			place_text("2",34,19 + Y_OFF);
			break;
	}

	if (dir > 0) {
    	fill_rect(40, 1 + Y_OFF, 7, 6, 0x1f);	
	} else {
    	fill_rect(40, 1 + Y_OFF, 7, 6, 0xcf);	
	}

	if (idle > 0) {
    	fill_rect(40, 22 + Y_OFF, 6, 5, 0xcf);	
	} else {
    	fill_rect(40, 22 + Y_OFF, 6, 5, 0x1f);	
	}

	if (stop > 0 || idle > 0) {
    	fill_rect(39, 10 + Y_OFF, 9, 7, 0xcf);	
	} else {
    	fill_rect(39, 10 + Y_OFF, 9, 7, 0x1f);	
	}
    
    update_options();
    
    char str[256] = { 0 };
    sprintf(str, "%10.2f%c", fabs(rpm), ( (rpm < 0 ) ? 0x1b : 0x1a));
    
    place_text(str, 6, 1 + Y_OFF);
    if (metric) {
            sprintf(str, "%10.4fmm ", pitch_converted);
            place_text(str, 6, 2 + Y_OFF);
            sprintf(str, "%10.4fmm ", posz_converted);
            place_text(str, 6, 3 + Y_OFF);
    } else {
            sprintf(str, "%10.4ftpi", pitch_converted);
            place_text(str, 6, 2 + Y_OFF);
            sprintf(str, "%10.4fin ", posz_converted);
            place_text(str, 6, 3 + Y_OFF);
    }
    sprintf(str, "%10.4f%c", fmod((double)posc+360.*100000.,360.), 0xf8);
    place_text(str, 6, 4 + Y_OFF);
    sprintf(str, "%10.0f   ", floor(posc/360));
    place_text(str, 6, 5 + Y_OFF);
	if (JiffyClock >= 0) {
	    sprintf(str, "%02d:%02d:%02d", ((int32_t(JiffyClock))/60)/60 ,((int32_t(JiffyClock))/60)%60, (int32_t(JiffyClock))%60);
	}
    place_text(str, 8, 6 + Y_OFF);

    fill_rect(0,0,40,1,0x10);
    sprintf(str,"                                                  ");
    place_text(str, 0, 0);
    sprintf(str,"S:%08x L:%08x I:%08x", 
        params->spindlePosition, params->motorPosition, params->indexLatch
        );
    place_text(str, 0, 0);
}

void tap_screen(uint32_t x, uint32_t y) {
    if ( x > 0 && x < 19 &&
         y > (0  + Y_OFF) && y < (7 + Y_OFF) ) {
        posz = 0;
        posc = 0;
    }
    if ( x > 19 && x < 30 &&
         y > (0 + Y_OFF) && y < (7 + Y_OFF) ) {
        if (mode == mode_metric_normal) {
            mode = mode_metric_extended;
        } else {
            mode = mode_metric_normal;
        }
    }
    if ( x > 28 && x < 39 &&
         y > (0 + Y_OFF) && y < (7 + Y_OFF) ) {
        if (mode == mode_sae_normal) {
            mode = mode_sae_extended;
        } else {
            mode = mode_sae_normal;
        }
    }
    if ( x > 39 && x < 48 &&
         y > (0 + Y_OFF) && y < (7 + Y_OFF) ) {
        if (dir > 0) {
			stop = 1.0f;
            dir = -1.0f;
        } else {
			stop = 1.0f;
			dir = +1.0f;
        }
    }
    if ( x > 39 && x < 48 &&
         y > (20 + Y_OFF) && y < (29 + Y_OFF) ) {
        if (idle > 0) {
            idle = 0.0f;
        } else {
            idle = 1.0f;
            stop = 1.0f;
        }
    }

    if ( x > 38 && x < 49 &&
         y > (9 + Y_OFF) && y < (19 + Y_OFF) ) {
        if (stop > 0) {
            stop = 0.0f;
            idle = 0.0f;
        } else {
            stop = 1.0f;
        }
    }

    for (int32_t c = 0; c < 12; c++) {
        uint32_t xx, yy, w, h;
        option_rect(c, xx, yy, w, h);
        if (x >= xx && x <= xx+w &&
            y >= yy && y <= yy+h) {
            option[mode] = c;
            break;
        }
    }
    pitch = options_val[mode][option[mode]];
}

void frame(void (*fill_span)(uint32_t xl, uint32_t xr, uint8_t *data)) {

    static int32_t frame = 0;

    static uint32_t flip = 0;

    map_screen((uint16_t *)&pixel_screen[flip][0]);
    uint16_t *_new = &pixel_screen[flip  ][0];
    uint16_t *_old = &pixel_screen[flip^1][0];

	if ((frame % 1024) == 0) {
            fill_span(0, SCREEN_WIDTH * SCREEN_HEIGHT, (uint8_t *)_new);
            return;
	}

    uint32_t xl = 0;
    uint32_t xr = 0;
    uint32_t lim = SCREEN_WIDTH * SCREEN_HEIGHT;
    for (uint32_t x = 0; x < lim; ) {
        xl = x;
        for ( ; x < lim; x++ ) {
            uint32_t a = _old[x];
            uint32_t b = _new[x];
            if (a != b) {
                xl = x;
                break;
            }
        }
        if(x == lim) {
            break;
        }
        xr = xl + 1;
        for ( ; ( xr - xl ) < 2048 && xr < lim; xr++ ) {
            bool match = false;
            for (uint32_t xx = 0; xx < 16 ; xx++) {
                if (xr+xx >= lim) {
                    break;
                }
                if(_old[xr+xx] == _new[xr+xx]) {
                    match = true;
                    break;
                }
            }
            if (match) {
                break;
            }
        }
        if (xr >= lim) {
            xr = lim;
        }
        fill_span(xl, xr, (uint8_t *)&_new[xl]);
        x = xr;
    }

    flip ^= 1;
}



static void SetMillimetersPerRotation(Params *params, double mm) {
    double c = (mm / 25.4) * LEAD_SCREW_ROTATIONS_PER_INCH;
    double d = c * MOTOR_OVERSAMPLE_FACTOR;
    double e = d * MOTOR_STEPS_PER_REV;
    double f = e / ENCODER_STEPS_PER_REV;    
    params->motorFactorHigh = (uint32_t)floor(f);
    params->motorFactorLow = (uint32_t)((f - floor(f)) * double(int64_t(1)<<32));
    params->oversampleFactor = MOTOR_OVERSAMPLE_FACTOR;
    params->stop = stop > 0 ? 1 : 0;
    params->idle = idle > 0 ? 1 : 0;
    params->rev = dir < 0 ? 1 : 0;
}

inline uint32_t pixel_color(uint8_t r, uint8_t g, uint8_t b, struct fb_var_screeninfo *vinfo) {
    return (r<<vinfo->red.offset) | (g<<vinfo->green.offset) | (b<<vinfo->blue.offset);
}

static int fb_fd = 0;
static uint8_t *fbp = 0;
static struct fb_var_screeninfo vinfo;
static struct fb_fix_screeninfo finfo;

static void init_fb() {
    
    fb_fd = open("/dev/fb0", O_RDWR);
    if (fb_fd < 0) {
        printf("Could not open fb0\n");
        exit(1);
    }

    ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo);
    ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo);

    size_t screensize = vinfo.yres_virtual * finfo.line_length;
    fbp = (uint8_t *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, (off_t)0);
	if (!fbp) {
        printf("Could not map fb0\n");
        exit(1);
	}
}

void fill_span(uint32_t xl, uint32_t xr, uint8_t *data) {
	int32_t s_y = xl / SCREEN_WIDTH;
	int32_t e_y = xr / SCREEN_WIDTH;
	size_t off = ((xl % SCREEN_WIDTH)+vinfo.xoffset) * (vinfo.bits_per_pixel/8) + (s_y+vinfo.yoffset) * finfo.line_length;	
	if (s_y == e_y) {
		memcpy(fbp+off,data,(xr - xl) * 2);
	} else {
		memcpy(fbp+off,data,(SCREEN_WIDTH - (xl % SCREEN_WIDTH)) * 2);
		data += (SCREEN_WIDTH - (xl % SCREEN_WIDTH)) * 2;
		for (uint32_t y = s_y + 1; y < e_y; y++) {
    		off = (vinfo.xoffset) * (vinfo.bits_per_pixel/8) + (y+vinfo.yoffset) * finfo.line_length;	
			memcpy(fbp+off,data,SCREEN_WIDTH * 2);
			data += SCREEN_WIDTH * 2;
		}
		off = (vinfo.xoffset) * (vinfo.bits_per_pixel/8) + (e_y +vinfo.yoffset) * finfo.line_length;	
		memcpy(fbp+off,data,(xr % SCREEN_WIDTH) * 2);
	}
}

static const uint8_t CTRL_LO_DFR = 0b0011;
static const uint8_t CTRL_LO_SER = 0b0100;
static const uint8_t CTRL_HI_X = 0b1001  << 4;
static const uint8_t CTRL_HI_Y = 0b1101  << 4;

static const uint16_t ADC_MAX = 0x0fff;  // 12 bits

#if 1

#include <linux/input.h>

#define EVENT_DEVICE    "/dev/input/event0"
#define EVENT_TYPE      EV_ABS
#define EVENT_CODE_X    ABS_X
#define EVENT_CODE_Y    ABS_Y

static int fd_event = 0;

static void TouchInit() {
    fd_event = open(EVENT_DEVICE, O_RDONLY);
    char name[256] = "Unknown";
        
    if (fd_event == -1) {
        fprintf(stderr, "%s is not a vaild device\n", EVENT_DEVICE);
        exit(-1);        
    }
    
    ioctl(fd_event, EVIOCGNAME(sizeof(name)), name);
    printf("Reading from:\n");
    printf("device file = %s\n", EVENT_DEVICE);
    printf("device name = %s\n", name);
                            
}

static void TouchCalc(int32_t &x, int32_t &y, int32_t &t) {
    static struct input_event ev[64] = {0};
    
    static int xx = 0;
    static int yy = 0;
    
    fd_set readfds;
    struct timeval tv = {0};
    
    FD_ZERO(&readfds);
    FD_SET(fd_event, &readfds);
    int ret = select(fd_event + 1, &readfds, 0, 0, &tv);
    if (ret <= 0) {
        x = xx;
        y = yy;
        return;
    } 
    
    int rb = read(fd_event,ev,sizeof(struct input_event)*64);
    for (int32_t c = 0; c < (rb/sizeof(struct input_event)); c++) {
        if (ev[c].type == EVENT_TYPE) {
            switch(ev[c].code) {
                case	EVENT_CODE_X:
                        if (t) xx = ev[c].value;
                        break;
                case	EVENT_CODE_Y:
                        if (t) yy = ev[c].value;
                        break;
                case 	0x39:
                        if (ev[c].value < 0) {
                            t = 0;
                        } else {
                            t = 1;
                        }
                        break;
            }
        }
    }

    x = xx;
    y = yy;
    
//    printf("%d %d %d\n", x, y, t);
}
#else
static void TouchInit() {
	uint8_t init[] = {CTRL_HI_Y | CTRL_LO_SER, 0, 0 };
	spi_write(init, init, sizeof(init));
}

static void TouchCalc(int32_t &x, int32_t &y, int32_t &t) {
	uint8_t pinInt = gpio.digitalRead(gpioCD);

	uint8_t hi_x[3] = { CTRL_HI_X | CTRL_LO_DFR, 0, 0 };
	spi_write(hi_x, hi_x, sizeof(hi_x));
	uint8_t hi_y[3] = { CTRL_HI_Y | CTRL_LO_DFR, 0, 0 };
	spi_write(hi_y, hi_y, sizeof(hi_y));
	uint8_t init[3] = { CTRL_HI_Y | CTRL_LO_SER, 0, 0 };
	spi_write(init, init, sizeof(init));

	float yy = ((hi_x[1] << 4) | (hi_x[2] >> 4));
	float xx = ((hi_y[1] << 4) | (hi_y[2] >> 4));
	float ii = ((init[1] << 4) | (init[2] >> 4));

	xx = (xx - 280) / (1800 - 280) * SCREEN_WIDTH;
	yy = (yy - 220) / (1820 - 220) * SCREEN_HEIGHT;

	x = int32_t(xx);
	y = int32_t(yy);

	t = !pinInt;

//	printf("%04d %04d %02d\n", x, y, i, pinInt);
}
#endif
  
static void intHandler(int) {
	if (params) {
		params->quit = 1;

		prussdrv_pru_wait_event(PRU_EVTOUT_0);
		prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);        
		prussdrv_pru_disable(0);
		prussdrv_exit();
	}
	system("echo > 1 /sys/devices/virtual/graphics/fbcon/cursor_blink");
	exit(0);
} 
   
int main(int argc, char **argv) {  

    system("echo > 0 /sys/devices/virtual/graphics/fbcon/cursor_blink");

    signal(SIGINT, intHandler);    

    init_fb();

#if 1
#else
    if (!spi_open()) {
        printf("spi_open() failed\n");  
        return 1;  
    }
#endif
    
    TouchInit();

    gpioCD = gpio.getPin("P9_15"); 

    load_font();
    load_metric();

    // If this segfaults, make sure you're executing as root.  
    prussdrv_init();  
    if (prussdrv_open(PRU_EVTOUT_0) == -1) {  
        printf("prussdrv_open() failed\n");  
        return 1;  
    }  

    tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;  
    prussdrv_pruintc_init(&pruss_intc_initdata);  

    void *pru_data_mem = 0;
    prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pru_data_mem );

    params = (Params *)pru_data_mem;
    memset(params, 0, sizeof(Params));    

    params->motorFactorHigh = 2;
    params->oversampleFactor = 8;

    prussdrv_exec_program(0, "pru0.bin");  
    
    usleep(100000);

    int32_t CurrentSpindlePosition = params->spindlePosition;
    int32_t CurrentMotorPosition = params->motorPosition;

    time(&startTime);
    
    time_t nowTime = { 0 };

    time(&nowTime);

    JiffyClock = difftime(nowTime, startTime);
    
    for (;;) {
                static uint16_t color = 0;
                
                SetMillimetersPerRotation(params, pitch);

		int32_t x = 0;
		int32_t y = 0;
		int32_t t = 0;
		TouchCalc(x, y, t);

		double OldJiffyClock = JiffyClock;

                time(&nowTime);
                JiffyClock = difftime(nowTime, startTime);

		if (!inTouch && t) {
			inTouch = true;
			tap_screen(x / FONT_WIDTH, y / FONT_HEIGHT);	
		}

		if (!t) {
			inTouch = false;
		}

		update_stats();
	
		frame(&fill_span);

		int32_t OldSpindlePosition = CurrentSpindlePosition;
		CurrentSpindlePosition = params->spindlePosition;
		
		int32_t OldMotorPosition = CurrentMotorPosition;
		CurrentMotorPosition = params->motorPosition;

		posz -= (double(CurrentMotorPosition - OldMotorPosition) / double(MOTOR_STEPS_PER_REV)) * MOTOR_PITCH_MM;
		posc -= (double(CurrentSpindlePosition - OldSpindlePosition) * 360.) / double(ENCODER_STEPS_PER_REV);            	
                post -=	 double(CurrentSpindlePosition - OldSpindlePosition);
		if (JiffyClock != OldJiffyClock) {		
        	        rpm = double(post) / (JiffyClock - OldJiffyClock);                
        		rpm = (rpm / double(ENCODER_STEPS_PER_REV)) * 60.;
        		post = 0;
                }
                
		usleep(1000000/100);
    }
    
    params->quit = 1;
    
    prussdrv_pru_wait_event(PRU_EVTOUT_0);  
    prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
    prussdrv_pru_disable(0);  
    prussdrv_exit();  
   
    return 0;  
}
