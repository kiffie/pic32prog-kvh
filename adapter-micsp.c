/*
 * ICSP (2 wire) interface using FT2232-based USB adapter
 *
 * Copyright (C) 2019 Kiffie van Haash, License: GPLv2
 *
 * This file is based on work by Serge Vakulenko in the
 * PIC32PROG project. See the accompanying file "COPYING" for more details.
 *
 * Electrical connection to the target:
 *
 *     TCK ------- PGEC
 *     TDO -+----- PGED
 *          |
 *     TDI -+
 *     TMS ------- MCLR\
 *
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <libusb-1.0/libusb.h>

#include "adapter.h"
#include "pic32.h"


#define FTDI_DIR_HIGH	0x0f
#define FTDI_INIT_HIGH	0x0f

#define FTDI_BIT_TCK	0x01
#define FTDI_BIT_TDI	0x02
#define FTDI_BIT_TDO	0x04
#define FTDI_BIT_TMS	0x08

#define FTDI_DIR_LOW	(FTDI_BIT_TMS|FTDI_BIT_TDI|FTDI_BIT_TCK)
#define FTDI_INIT_LOW	(FTDI_BIT_TMS|FTDI_BIT_TCK)

#define RXBUF_SIZE  512

typedef struct {
    /* Common part */
    adapter_t adapter;
    const char *name;

    /* Device handle for libusb. */
    libusb_device_handle *usbdev;
    libusb_context *context;

    /* Transmit buffer for MPSSE packet. */
    unsigned char output [256*16];
    int bytes_to_write;

    /* Receive buffer */
    unsigned char rxbuf[RXBUF_SIZE];
    int rxbuf_len;
    int rxbuf_start;
    int bytes_to_read;

    //unsigned mhz;
    unsigned use_executive;
    unsigned serial_execution_mode;
} micsp_adapter_t;

static const char adapter_name[]="FTDI ICSP (2 wire)";

/*
 * Identifiers of USB adapter.
 */
#define OLIMEX_VID              0x15ba
#define OLIMEX_ARM_USB_TINY     0x0004  /* ARM-USB-Tiny */
#define OLIMEX_ARM_USB_TINY_H   0x002a	/* ARM-USB-Tiny-H */
#define OLIMEX_ARM_USB_OCD_H    0x002b	/* ARM-USB-OCD-H */
#define OLIMEX_MIPS_USB_OCD_H   0x0036	/* MIPS-USB-OCD-H */

#define DP_BUSBLASTER_VID       0x0403
#define DP_BUSBLASTER_PID       0x6010  /* Bus Blaster v2 */

/*
 * USB endpoints.
 */
#define IN_EP                   0x02
#define OUT_EP                  0x81

/* Requests */
#define SIO_RESET               0 /* Reset the port */
#define SIO_MODEM_CTRL          1 /* Set the modem control register */
#define SIO_SET_FLOW_CTRL       2 /* Set flow control register */
#define SIO_SET_BAUD_RATE       3 /* Set baud rate */
#define SIO_SET_DATA            4 /* Set the data characteristics of the port */
#define SIO_POLL_MODEM_STATUS   5
#define SIO_SET_EVENT_CHAR      6
#define SIO_SET_ERROR_CHAR      7
#define SIO_SET_LATENCY_TIMER   9
#define SIO_GET_LATENCY_TIMER   10
#define SIO_SET_BITMODE         11
#define SIO_READ_PINS           12
#define SIO_READ_EEPROM         0x90
#define SIO_WRITE_EEPROM        0x91
#define SIO_ERASE_EEPROM        0x92

/* MPSSE commands. */
#define CLKWNEG                 0x01
#define BITMODE                 0x02
#define CLKRNEG                 0x04
#define LSB                     0x08
#define WTDI                    0x10
#define RTDO                    0x20
#define WTMS                    0x40
#define SET_BITS_LOW            0x80
#define SET_BITS_HIGH           0x82
#define TCK_DIVISOR             0x86

/*
 * Calculate checksum.
 */
static unsigned calculate_crc (unsigned crc, unsigned char *data, unsigned nbytes)
{
    static const unsigned short crc_table [16] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    };
    unsigned i;

    while (nbytes--) {
        i = (crc >> 12) ^ (*data >> 4);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        i = (crc >> 12) ^ (*data >> 0);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        data++;
    }
    return crc & 0xffff;
}

static int bulk_write(micsp_adapter_t *a, unsigned char *output, int nbytes)
{
    int bytes_written;
    assert(nbytes > 0);

    if (debug_level > 2) {
        int i;
        fprintf(stderr, "USB bulk write %d bytes", nbytes);
        if(debug_level > 3){
            fprintf(stderr, ":");
            for (i=0; i<nbytes; i++)
                fprintf(stderr, "%c%02x", i ? '-' : ' ', output[i]);
        }
        fprintf(stderr, "\n");
    }

    int ret = libusb_bulk_transfer(a->usbdev, IN_EP, (unsigned char*) output,
        nbytes, &bytes_written, 3000);

    if (ret != 0) {
        fprintf(stderr, "USB bulk write failed: %s.\n", libusb_strerror(ret));
        exit(EXIT_FAILURE);
    }
    if (bytes_written != nbytes){
        fprintf(stderr,
                "USB bulk written %d bytes of %d", bytes_written, nbytes);
    }
    return bytes_written;
}

static int bulk_read(micsp_adapter_t *a, unsigned char *input, int len)
{
    int ret, l, nbytes = len;

    if(debug_level > 2){
        fprintf(stderr, "bulk_read(len=%d)\n", len);
    }
    while(nbytes > 0){
        l = a->rxbuf_len - a->rxbuf_start;
        if(l > nbytes){
            l = nbytes;
        }
        if( l > 0){
            memcpy(input, &a->rxbuf[a->rxbuf_start], l);
            a->rxbuf_start += l;
            nbytes -= l;
            input += l;
        }
        if(nbytes != 0 && a->rxbuf_start == a->rxbuf_len){
            a->rxbuf_start = 2;
            ret = libusb_bulk_transfer(a->usbdev,
                                       OUT_EP,
                                       a->rxbuf, RXBUF_SIZE, &a->rxbuf_len,
                                       2000);
            if(ret != 0 ){
                fprintf(stderr, "USB bulk read failed: %s\n", libusb_strerror(ret));
                exit(EXIT_FAILURE);
            }
            if(debug_level > 2){
                fprintf(stderr, "USB bulk read: %d bytes.\n", a->rxbuf_len);
            }
        }
    }
    return len;
}

static void set_clock_frequency(micsp_adapter_t *a, int khz)
{
    unsigned char output [3];
    unsigned const MHZ = 6;
    int divisor = (MHZ * 2000 / khz + 1) / 2 - 1;

    if (divisor < 0)
        divisor = 0;
    if (debug_level)
    	fprintf(stderr, "%s: divisor: %u\n", a->name, divisor);

    if (MHZ > 6) {
        /* Use 60MHz master clock (disable divide by 5). */
        output [0] = 0x8A;

        /* Turn off adaptive clocking. */
        output [1] = 0x97;

        /* Disable three-phase clocking. */
        output [2] = 0x8D;
        bulk_write(a, output, 3);
    }

    /* Command "set TCK divisor". */
    output [0] = TCK_DIVISOR;
    output [1] = divisor;
    output [2] = divisor >> 8;
    bulk_write(a, output, 3);

    if (debug_level) {
        khz = (MHZ * 2000 / (divisor + 1) + 1) / 2;
        fprintf(stderr, "%s: clock rate %.1f MHz\n", a->name, khz / 1000.0);
    }
}

static void set_reset(micsp_adapter_t *a, int reset, int clk){

	uint8_t bits= 0;
	if( reset )
		bits|= FTDI_BIT_TMS;
	if( clk )
		bits|= FTDI_BIT_TCK;

	uint8_t buf[]= {SET_BITS_LOW, bits, FTDI_DIR_LOW};
	int result= bulk_write(a, buf, sizeof(buf));
	if( result != 3 ){
		fprintf(stderr, "set_reset_pin: bulk_write failed\n");
		exit(EXIT_FAILURE);
	}
}


static void enter_icsp_mode(micsp_adapter_t *a){
	set_reset(a, 0, 0);
	usleep(10);
	set_reset(a, 1, 0);
        usleep(10);
	set_reset(a, 0, 0);
	usleep(10);

        uint8_t cmd_buf[]= {WTDI|CLKWNEG, 3, 0,
		0x4d, 0x43, 0x48, 0x50};

	int res= bulk_write(a, cmd_buf, sizeof(cmd_buf));
	if( res != sizeof(cmd_buf) ){
		fprintf(stderr, "enter_icsp_mode: bulk_write failed\n");
		exit(EXIT_FAILURE);
	}
	usleep(1000);
	set_reset(a, 1, 0);
	usleep(10);
	set_reset(a, 1, 1);
	usleep(1000);
}


static void flush_output(micsp_adapter_t *a){
                if(a->bytes_to_write == 0){
                    return;
                }
		int res= bulk_write(a, a->output, a->bytes_to_write);
		if( res != a->bytes_to_write ){
			fprintf(stderr, "bulk_write failed\n");
			exit(EXIT_FAILURE);
		}
		a->bytes_to_write= 0;
}

static void write_command(micsp_adapter_t *a, uint8_t *buf, int len){
	while(len>0){
		int avail= sizeof(a->output)-a->bytes_to_write;
		assert(avail >= 0);
		if( avail == 0 ){
			flush_output(a);
		}
		int cpylen= len<=avail? len : avail;
		uint8_t *p= &a->output[a->bytes_to_write];
		memcpy(p, buf, cpylen);
		a->bytes_to_write+= cpylen;
		p+= cpylen;
		buf+= cpylen;
		len-= cpylen;
	}
}

static void icsp_4phase(micsp_adapter_t *a, int tdi, int tms, int do_read){
	uint8_t bits= 0;
	//printf("icsp_4phase: tdi=%d, tms=%d, do_read=%d\n", tdi, tms, do_read);
	if( tdi )
		bits|= 0x80;
	if( tms )
		bits|= 0x40;
	uint8_t cmd_buf[]= {

		SET_BITS_LOW, FTDI_INIT_LOW, FTDI_DIR_LOW,
		WTDI|BITMODE, 1, bits,

		SET_BITS_LOW, FTDI_INIT_LOW, FTDI_DIR_LOW & ~FTDI_BIT_TDI,
		WTDI|(do_read?RTDO:0)|BITMODE, 1, 0
	};
	write_command(a, cmd_buf, sizeof(cmd_buf));
	if( do_read ){
		a->bytes_to_read++;
	}
}

static unsigned icsp_readback(micsp_adapter_t *a, int no_bits){

	uint8_t buf[256];
	int read_ctr=0, retry_ctr=0;
	unsigned res= 0;

	assert(no_bits <= sizeof(buf));
	if( no_bits > a->bytes_to_read ){
		printf("internal error: trying to read too many bits %d > %d\n",
						no_bits, a->bytes_to_read);
		exit(EXIT_FAILURE);
	}
	flush_output(a);

	while( read_ctr < no_bits ){

		res= bulk_read(a, &buf[read_ctr], no_bits-read_ctr);
		//printf("  res=%d, no_bits=%d, read_ctr=%d\n", res, no_bits, read_ctr);

		if( res <= 0 ){
			retry_ctr++;
		}else{
			read_ctr+= res;
			retry_ctr= 0;
		}

		if( retry_ctr >100 ){
			fprintf(stderr, "icsp_readback: ftdi_read_data failed.\n");
			exit(EXIT_FAILURE);
		}
	}
	a->bytes_to_read-=no_bits;
	assert(read_ctr==no_bits);
	res= 0;
	int i;
	for(i=0; i<no_bits; i++){
		//printf("buf[%d]= %02x\n", i, buf[i]);
		res>>=1;
		if( buf[i]&0x2 ){
			res|= 0x80000000;
		}
	}
	res>>=(32-no_bits);
	//printf("icsp_readback: res=%08x\n", res);
	return res;
}



static void set_mode(micsp_adapter_t *a, unsigned int pattern, int len){
	int i;
	for(i=0; i< len; i++){
		int bit= pattern&0x01;
		icsp_4phase(a, 0, bit, 0);
		pattern>>=1;
	}
}


static void send_command(micsp_adapter_t *a, unsigned int command){
	icsp_4phase(a, 0, 1, 0);
	icsp_4phase(a, 0, 1, 0);
	icsp_4phase(a, 0, 0, 0);
	icsp_4phase(a, 0, 0, 0);
	int i;
	for(i=0; i<5; i++){
		int tdi= command&0x01;
		command>>=1;
		int tms= i==4? 1 : 0;
		icsp_4phase(a, tdi, tms, 0);
	}
	icsp_4phase(a, 0, 1, 0);
	icsp_4phase(a, 0, 0, 0);
}


static unsigned xfer_data(micsp_adapter_t *a, unsigned data, unsigned len){
	assert(len>0 && len <=32);
	icsp_4phase(a, 0, 1, 0);
	icsp_4phase(a, 0, 0, 0);
	icsp_4phase(a, 0, 0, 1);
	int i;
	for(i=0; i<len; i++){
		int tdi= data&0x01;
		data>>=1;
		if( i< (len-1) ){
			icsp_4phase(a, tdi, 0, 1);
		}else{
			icsp_4phase(a, tdi, 1, 0);
		}
	}
	icsp_4phase(a, 0, 1, 0);
	icsp_4phase(a, 0, 0, 0);
	return icsp_readback(a, len);
}

static inline uint8_t xfer_data8(micsp_adapter_t *a, uint8_t data){
    return xfer_data(a, data, 8);
}

static inline uint32_t xfer_data32(micsp_adapter_t *a, uint32_t data){
    return xfer_data(a, data, 32);
}

static uint32_t xfer_fast_data_sw(micsp_adapter_t *a, uint32_t data, int do_read, int swtap){
    uint32_t result;
    int pr_acc;
    icsp_4phase(a, 0, 1, 0);
    icsp_4phase(a, 0, 0, 0);
    icsp_4phase(a, 0, 0, 1); /* receive PrAcc */
    icsp_4phase(a, 0, 0, do_read); /* send PrAcc==0, receive LSB */

    int i;
    for(i=0; i<32; i++){
        int tdi= data&0x01;
        data>>=1;
        int tms= i==31?1:0; /* set TMS=1 for last bit */
        int read= (i<31)&&do_read?1:0; /* read during first 31 bits only */
        icsp_4phase(a, tdi, tms, read);
    }
    icsp_4phase(a, 0, 1, 0);
    icsp_4phase(a, 0, 0, 0);
    if(swtap){
        send_command(a, TAP_SW_MTAP);
        flush_output(a);
        usleep(500);
        send_command(a, TAP_SW_ETAP);
    }
    pr_acc= icsp_readback(a, 1);
    if( pr_acc == 0 ){
        printf("error: pr_acc==0\n");
    }
    if( do_read ){
        result= icsp_readback(a, 32);
    }else{
        result= 0;
    }
    return result;
}

static inline uint32_t xfer_fast_data(micsp_adapter_t *a, uint32_t data, int do_read){
    return xfer_fast_data_sw(a, data, do_read, 0);
}


static void xfer_instruction(micsp_adapter_t *a, uint32_t instruction){

    unsigned ctl;

    if (debug_level > 1)
        fprintf (stderr, "%s: xfer instruction %08x\n", a->name, instruction);

    // Select Control Register
    send_command(a, ETAP_CONTROL);

    // Wait until CPU is ready
    // Check if Processor Access bit (bit 18) is set
    do {
		ctl= xfer_data32(a, CONTROL_PRACC|CONTROL_PROBEN|CONTROL_PROBTRAP|CONTROL_EJTAGBRK);
    } while (! (ctl & CONTROL_PRACC));

    // Select Data Register
    // Send the instruction
    send_command(a, ETAP_DATA);
    xfer_data32(a, instruction);

    // Tell CPU to execute instruction
    send_command(a, ETAP_CONTROL);
    xfer_data32(a, CONTROL_PROBEN | CONTROL_PROBTRAP);
}

static uint8_t get_device_status(micsp_adapter_t *a){

    uint8_t status;

    set_mode(a , 0x1f, 6);
    send_command(a, 0x04); /* MTAP_SW_MTAP */
    send_command(a, 0x07 ); /* MTAP_COMMAND */
    status= xfer_data8(a, 0x00); /* MCHP_STATUS */
    return status;
}


static void micsp_close(adapter_t *adapter, int power_on)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;

    flush_output(a);
    set_reset(a, 0, 0);
    usleep(10);
    uint8_t cmd_buf[]= {WTDI|CLKWNEG|BITMODE, 0, 0};
    int res= bulk_write(a, cmd_buf, sizeof(cmd_buf));
    if( res != sizeof(cmd_buf) ){
        fprintf(stderr, "mpsse_close: bulk_write failed\n");
        exit(EXIT_FAILURE);
    }
    usleep(10);
    set_reset(a, 1, 0);
    uint8_t cmd_highz[]= {SET_BITS_LOW, 0x00, 0x00,
                          SET_BITS_HIGH, 0x00, 0x00};
    res= bulk_write(a, cmd_highz, sizeof(cmd_highz));
    if( res != sizeof(cmd_highz) ){
        fprintf(stderr, "mpsse_close: bulk_write failed\n");
        exit(EXIT_FAILURE);
    }

    libusb_release_interface(a->usbdev, 0);
    libusb_close(a->usbdev);
    free (a);
}

/*
 * Read the Device Identification code
 */
static unsigned micsp_get_idcode (adapter_t *adapter)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;

    set_mode(a, 0x1f, 6);
    return xfer_data32(a, 0);
}


/*
 * Put device to serial execution mode.
 */
static void serial_execution (micsp_adapter_t *a)
{
    if (a->serial_execution_mode)
        return;
    a->serial_execution_mode = 1;

    /* Enter serial execution. */
    if (debug_level > 0)
        fprintf (stderr, "%s: enter serial execution\n", a->name);

    send_command(a, TAP_SW_MTAP);
    send_command(a, MTAP_COMMAND);
    unsigned status= xfer_data8(a, MCHP_STATUS);
    if (debug_level > 0)
        fprintf (stderr, "%s: status %02x\n", a->name, status);
    if ((status & MCHP_STATUS_CPS) == 0) {
        fprintf (stderr, "%s: invalid status = %02x (reset)\n", a->name, status);
        exit (-1);
    }
    xfer_data8(a, MCHP_ASSERT_RST);
    send_command(a, TAP_SW_ETAP);
    send_command(a, ETAP_EJTAGBOOT);
    send_command(a, TAP_SW_MTAP);
    send_command(a, MTAP_COMMAND);
    xfer_data8(a, MCHP_DEASSERT_RST);
    xfer_data8(a, MCHP_FLASH_ENABLE);
    send_command(a, TAP_SW_ETAP);
}

static unsigned get_pe_response (micsp_adapter_t *a)
{
    unsigned ctl, response;

    // Select Control Register
    send_command(a, ETAP_CONTROL);

    // Wait until CPU is ready
    // Check if Processor Access bit (bit 18) is set
    do {
        ctl= xfer_data32(a, CONTROL_PRACC |
                            CONTROL_PROBEN |
                            CONTROL_PROBTRAP |
                            CONTROL_EJTAGBRK);
    } while (! (ctl & CONTROL_PRACC));

    // Select Data Register
    // Send the instruction
    send_command(a, ETAP_DATA);
	response= xfer_data32(a, 0);

    // Tell CPU to execute NOP instruction
    send_command(a, ETAP_CONTROL);
    xfer_data32(a, CONTROL_PROBEN | CONTROL_PROBTRAP);
    if (debug_level > 1)
        fprintf (stderr, "%s: get PE response %08x\n", a->name, response);
    return response;
}

/*
 * Read a word from memory (without PE).
 */
static unsigned micsp_read_word_classic(adapter_t *adapter, unsigned addr)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;
    unsigned addr_lo = addr & 0xFFFF;
    unsigned addr_hi = (addr >> 16) & 0xFFFF;

    serial_execution (a);

    //fprintf (stderr, "%s: read word from %08x\n", a->name, addr);
    xfer_instruction (a, 0x3c13ff20);           // lui s3, 0xFF20
    xfer_instruction (a, 0x3c080000 | addr_hi); // lui t0, addr_hi
    xfer_instruction (a, 0x35080000 | addr_lo); // ori t0, addr_lo
    xfer_instruction (a, 0x8d090000);           // lw t1, 0(t0)
    xfer_instruction (a, 0xae690000);           // sw t1, 0(s3)
    xfer_instruction (a, 0x00000000);           // nop

    send_command(a, ETAP_FASTDATA);
    unsigned word= xfer_fast_data(a, 0, 1);
    if (debug_level > 0)
        fprintf (stderr, "%s: read word (classic) at %08x -> %08x\n",
                 a->name, addr, word);
    return word;
}


static unsigned micsp_read_word_micromips(adapter_t *adapter, unsigned addr)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;
    unsigned addr_lo = (addr << 16);
    unsigned addr_hi = addr & 0xFFFF0000;

    serial_execution (a);

    //fprintf (stderr, "%s: read word from %08x\n", a->name, addr);
    xfer_instruction (a, 0xFF2041B3);           // lui s3, 0xFF20
    xfer_instruction (a, 0x000041A8 | addr_hi); // lui t0, addr_hi
    xfer_instruction (a, 0x00005108 | addr_lo); // ori t0, addr_lo
    xfer_instruction (a, 0x0000FD28);           // lw t1, 0(t0)
    xfer_instruction (a, 0x0000F933);           // sw t1, 0(s3)
    xfer_instruction (a, 0x00000000);           // nop

    send_command(a, ETAP_FASTDATA);
    unsigned word= xfer_fast_data(a, 0, 1);
    if (debug_level > 0)
        fprintf (stderr, "%s: read word (micromips) at %08x -> %08x\n",
                 a->name, addr, word);
    return word;
}

static int micsp_is_micromips(adapter_t *adapter){
    assert(adapter->family_name != NULL);
    return strcmp(adapter->family_name, "mm") == 0 ? 1 : 0;
}

static unsigned micsp_read_word(adapter_t *adapter, unsigned addr){
    unsigned result;
    if(micsp_is_micromips(adapter)){
        result =  micsp_read_word_micromips(adapter, addr);
    }else{
        result = micsp_read_word_classic(adapter, addr);
    }
    return result;
}

/*
 * Read a memory block.
 */
static void micsp_read_data (adapter_t *adapter,
    unsigned addr, unsigned nwords, unsigned *data)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;
    unsigned words_read, i;

    //fprintf (stderr, "%s: read %d bytes from %08x\n", a->name, nwords*4, addr);
    if (! a->use_executive) {
        /* Without PE. */
        for (; nwords > 0; nwords--) {
            *data++ = micsp_read_word (adapter, addr);
            addr += 4;
        }
        return;
    }

    /* Use PE to read memory. */
    for (words_read = 0; words_read < nwords; words_read += 32) {

        send_command (a, ETAP_FASTDATA);
        xfer_fast_data(a, PE_READ << 16 | 32, 0);      /* Read 32 words */
        xfer_fast_data(a, addr, 0);                    /* Address */

        unsigned response = get_pe_response (a);    /* Get response */
        if (response != PE_READ << 16) {
            fprintf (stderr, "%s: bad READ response = %08x, expected %08x\n",
                a->name, response, PE_READ << 16);
            exit (-1);
        }
        for (i=0; i<32; i++) {
            *data++ = get_pe_response (a);          /* Get data */
        }
        addr += 32*4;
    }
}

/*
 * Download programming executive (PE).
 */
static void micsp_load_executive_classic(adapter_t *adapter,
    const unsigned *pe, unsigned nwords, unsigned pe_version)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;

    a->use_executive = 1;
    serial_execution (a);

    if (debug_level > 0)
        fprintf (stderr, "%s: download PE loader\n", a->name);

    /* Step 1. */
    xfer_instruction (a, 0x3c04bf88);   // lui a0, 0xbf88
    xfer_instruction (a, 0x34842000);   // ori a0, 0x2000 - address of BMXCON
    xfer_instruction (a, 0x3c05001f);   // lui a1, 0x1f
    xfer_instruction (a, 0x34a50040);   // ori a1, 0x40   - a1 has 001f0040
    xfer_instruction (a, 0xac850000);   // sw  a1, 0(a0)  - BMXCON initialized

    /* Step 2. */
    xfer_instruction (a, 0x34050800);   // li  a1, 0x800  - a1 has 00000800
    xfer_instruction (a, 0xac850010);   // sw  a1, 16(a0) - BMXDKPBA initialized

    /* Step 3. */
    xfer_instruction (a, 0x8c850040);   // lw  a1, 64(a0) - load BMXDMSZ
    xfer_instruction (a, 0xac850020);   // sw  a1, 32(a0) - BMXDUDBA initialized
    xfer_instruction (a, 0xac850030);   // sw  a1, 48(a0) - BMXDUPBA initialized

    /* Step 4. */
    xfer_instruction (a, 0x3c04a000);   // lui a0, 0xa000
    xfer_instruction (a, 0x34840800);   // ori a0, 0x800  - a0 has a0000800

    /* Download the PE loader. */
    int i;
    for (i=0; i<PIC32_PE_LOADER_LEN; i+=2) {
        /* Step 5. */
        unsigned opcode1 = 0x3c060000 | pic32_pe_loader[i];
        unsigned opcode2 = 0x34c60000 | pic32_pe_loader[i+1];

        xfer_instruction (a, opcode1);      // lui a2, PE_loader_hi++
        xfer_instruction (a, opcode2);      // ori a2, PE_loader_lo++
        xfer_instruction (a, 0xac860000);   // sw  a2, 0(a0)
        xfer_instruction (a, 0x24840004);   // addiu a0, 4
    }

    /* Jump to PE loader (step 6). */
    xfer_instruction (a, 0x3c19a000);   // lui t9, 0xa000
    xfer_instruction (a, 0x37390800);   // ori t9, 0x800  - t9 has a0000800
    xfer_instruction (a, 0x03200008);   // jr  t9
    xfer_instruction (a, 0x00000000);   // nop

    /* Switch from serial to fast execution mode. */
    send_command (a, TAP_SW_ETAP);
    set_mode(a, 0x1f, 6);             /* TMS 1-1-1-1-1-0 */

    /* Send parameters for the loader (step 7-A).
     * PE_ADDRESS = 0xA000_0900,
     * PE_SIZE */
    send_command(a, ETAP_FASTDATA);
    xfer_fast_data (a, 0xa0000900, 0);
    xfer_fast_data (a, nwords, 0);

    /* Download the PE itself (step 7-B). */
    if (debug_level > 0)
        fprintf (stderr, "%s: download PE\n", a->name);
    for (i=0; i<nwords; i++) {
        xfer_fast_data (a, *pe++, 0);
    }
    mdelay (10);

    /* Download the PE instructions. */
    xfer_fast_data (a, 0, 0);                       /* Step 8 - jump to PE. */
    xfer_fast_data (a, 0xDEAD0000, 0);
    mdelay (10);
    xfer_fast_data (a, PE_EXEC_VERSION << 16, 0);

    unsigned version = get_pe_response (a);
    if (version != (PE_EXEC_VERSION << 16 | pe_version)) {
        fprintf (stderr, "%s: bad PE version = %08x, expected %08x\n",
            a->name, version, PE_EXEC_VERSION << 16 | pe_version);
        exit (-1);
    }
    if (debug_level > 0)
        fprintf (stderr, "%s: PE version = %04x\n",
            a->name, version & 0xffff);
   a->serial_execution_mode= 0;
}

static void micsp_load_executive_micromips(adapter_t *adapter,
    const unsigned *pe, unsigned nwords, unsigned pe_version)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;

    a->use_executive = 1;
    serial_execution (a);

    if (debug_level > 0)
        fprintf (stderr, "%s: download PE loader (micromips), nwords = %u\n",
                 a->name, nwords);

    /* Step 1 */
    xfer_instruction (a, 0xA00041A4);   // lui a0, 0xa000
    xfer_instruction (a, 0x02005084);   // ori a0, 0x0800 - address of BMXCON

    /* Download the PE loader (Step2) */
    int i;
    for(i=0; i<PIC32_PEMM_LOADER_LEN; i+=2){
        /* Step 5. */
        uint32_t opcode1 = 0x41A6 | (pic32_pemm_loader[i+1] << 16); /* hi */
        uint32_t opcode2 = 0x50C6 | (pic32_pemm_loader[i] << 16); /* lo */
        xfer_instruction (a, opcode1);      // lui a2, PE_loader_hi++
        xfer_instruction (a, opcode2);      // ori a2, PE_loader_lo++
        xfer_instruction (a, 0x6E42EB40);   // sw  a2, 0(a0)
                                            // addiu a0, 4
    }

    /* Jump to PE loader (step 3) */
    xfer_instruction (a, 0xA00041B9);
    xfer_instruction (a, 0x02015339);
    xfer_instruction (a, 0x0c004599);
    xfer_instruction (a, 0x0c000c00);
    xfer_instruction (a, 0x0c000c00);

    /* Load PE (step 4) */
    /* Switch from serial to fast execution mode. */
    send_command (a, TAP_SW_ETAP);
    set_mode(a, 0x1f, 6);             /* TMS 1-1-1-1-1-0 */

    /* Send parameters for the loader (step 7-A).
     * PE_ADDRESS = 0xA000_0900,
     * PE_SIZE */
    send_command(a, ETAP_FASTDATA);

    xfer_fast_data (a, 0xa0000300, 0);
    xfer_fast_data (a, nwords, 0);

    /* Download the PE itself (step 7-B). */
    if (debug_level > 0)
        fprintf (stderr, "%s: download PE (micromips)\n", a->name);
    for (i=0; i<nwords; i++) {
        xfer_fast_data (a, *pe++, 0);
    }

    /* Download the PE instructions. */
    xfer_fast_data (a, 0, 0);                       /* Step 5 - jump to PE. */
    xfer_fast_data (a, 0xDEAD0000, 0);

    xfer_fast_data (a, PE_EXEC_VERSION << 16, 0);

    unsigned version = get_pe_response (a);
    if (version != (PE_EXEC_VERSION << 16 | pe_version)) {
        fprintf (stderr, "%s: bad PE version = %08x, expected %08x\n",
            a->name, version, PE_EXEC_VERSION << 16 | pe_version);
        exit (-1);
    }
    if (debug_level > 0)
        fprintf (stderr, "%s: PE version = %04x\n",
            a->name, version & 0xffff);
   a->serial_execution_mode= 0;
}

static void micsp_load_executive(adapter_t *adapter,
    const unsigned *pe, unsigned nwords, unsigned pe_version)
{
    if(micsp_is_micromips(adapter)){
        micsp_load_executive_micromips(adapter, pe, nwords, pe_version);
    }else{
        micsp_load_executive_classic(adapter, pe, nwords, pe_version);
    }
}

/*
 * Erase all flash memory.
 */
static void micsp_erase_chip (adapter_t *adapter)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;

    send_command(a, TAP_SW_MTAP);
    send_command(a, MTAP_COMMAND);
    xfer_data8(a, MCHP_ERASE);
    xfer_data8(a, MCHP_DEASSERT_RST);
    //flush_output(a); /* not needed because xfer_data8 reads result */
    mdelay (400);

    /* Leave it in ETAP mode. */
    send_command(a, TAP_SW_ETAP);
}

/*
 * Write a word to flash memory.
 */
static void micsp_program_word (adapter_t *adapter,
    unsigned addr, unsigned word)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;

    if (debug_level > 0)
        fprintf (stderr, "%s: program word at %08x: %08x\n", a->name, addr, word);
    if (! a->use_executive) {
        /* Without PE. */
        fprintf (stderr, "%s: slow flash write not implemented yet.\n", a->name);
        exit (-1);
    }

    /* Use PE to write flash memory. */
    send_command(a, ETAP_FASTDATA);
    xfer_fast_data(a, PE_WORD_PROGRAM << 16 | 2, 0);
    flush_output(a);
    xfer_fast_data(a, addr, 0);                    /* Send address. */
    flush_output(a);
    xfer_fast_data (a, word, 0);                   /* Send word. */

    unsigned response = get_pe_response (a);
    if (response != (PE_WORD_PROGRAM << 16)) {
        fprintf (stderr, "%s: failed to program word %08x at %08x, reply = %08x\n",
            a->name, word, addr, response);
        exit (-1);
    }
}

#if 0
static void micsp_program_dword (adapter_t *adapter,
    unsigned addr, unsigned word0, unsigned word1)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;

    if (debug_level > 0)
        fprintf (stderr, "%s: program double word at %08x: %08x:%08x\n",
                 a->name, addr, word0, word1);
    if (! a->use_executive) {
        /* Without PE. */
        fprintf (stderr, "%s: slow flash write not implemented yet.\n", a->name);
        exit (-1);
    }

    /* Use PE to write flash memory. */
    send_command(a, ETAP_FASTDATA);
    xfer_fast_data(a, PE_DOUBLE_WORD_PGRM << 16 | 2, 0);
    flush_output(a);
    xfer_fast_data(a, addr, 0);                    /* Send address. */
    flush_output(a);
    xfer_fast_data (a, word0, 0);                   /* Send word 0. */
    xfer_fast_data (a, word1, 0);                   /* Send word 1. */


    unsigned response = get_pe_response (a);
    if (response != (PE_DOUBLE_WORD_PGRM << 16)) {
        fprintf (stderr, "%s: failed to program double word %08x:%08x at %08x, reply = %08x\n",
            a->name, word0, word1, addr, response);
        exit (-1);
    }
    send_command(a, TAP_SW_MTAP);
    flush_output(a);
    usleep(1000);
    send_command(a, TAP_SW_ETAP);
    flush_output(a);

}
#endif


/*
 * Flash write row of memory.
 */
static void micsp_program_row (adapter_t *adapter, unsigned addr,
    unsigned *data, unsigned words_per_row)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;
    int i;

    if (debug_level > 0)
        fprintf (stderr, "%s: row program %u words at %08x\n",
                 a->name, words_per_row, addr);
    if (! a->use_executive) {
        /* Without PE. */
        fprintf (stderr, "%s: slow flash write not implemented yet.\n", a->name);
        exit (-1);
    }

    /* Use PE to write flash memory. */
    send_command(a, ETAP_FASTDATA);
    xfer_fast_data (a, PE_ROW_PROGRAM << 16 | words_per_row, 0);
    xfer_fast_data (a, addr, 0);                    /* Send address. */

    /* Download data. */
    for (i = 0; i < words_per_row; i++) {
        /* quirk: fast TAP switch during flashing of configuration words
         * needed for PIC32MM (for JTAGEN bit). We do the TAP switch after
         * submitting each complete program row command, i.e. after transmission
         * of last word of the row.
         */
        int swtap = (i == words_per_row-1);
        xfer_fast_data_sw (a, *data++, 0, swtap);          /* Send word. */
    }

    unsigned response = get_pe_response (a);
    if (response != (PE_ROW_PROGRAM << 16)) {
        fprintf (stderr, "%s: failed to program row at %08x, reply = %08x\n",
            a->name, addr, response);
        exit (-1);
    }
}

/*
 * Verify a block of memory.
 */
static void micsp_verify_data (adapter_t *adapter,
    unsigned addr, unsigned nwords, unsigned *data)
{
    micsp_adapter_t *a = (micsp_adapter_t*) adapter;
    unsigned data_crc, flash_crc;

    //fprintf (stderr, "%s: verify %d words at %08x\n", a->name, nwords, addr);
    if (! a->use_executive) {
        /* Without PE. */
        fprintf (stderr, "%s: slow verify not implemented yet.\n", a->name);
        exit (-1);
    }

    /* Use PE to get CRC of flash memory. */
    send_command(a, ETAP_FASTDATA);
    xfer_fast_data(a, PE_GET_CRC << 16, 0);
    xfer_fast_data (a, addr, 0);                    /* Send address. */
    xfer_fast_data (a, nwords * 4, 0);              /* Send length. */

    unsigned response = get_pe_response (a);
    if (response != (PE_GET_CRC << 16)) {
        fprintf (stderr, "%s: failed to verify %d words at %08x, reply = %08x\n",
            a->name, nwords, addr, response);
        exit (-1);
    }
    flash_crc = get_pe_response (a) & 0xffff;

    data_crc = calculate_crc (0xffff, (unsigned char*) data, nwords * 4);
    if (flash_crc != data_crc) {
        fprintf (stderr, "%s: checksum failed at %08x: sum=%04x, expected=%04x\n",
            a->name, addr, flash_crc, data_crc);
        //exit (-1);
    }
}

/*
 * Initialize adapter F2232.
 * Return a pointer to a data structure, allocated dynamically.
 * When adapter not found, return 0.
 */

adapter_t *adapter_open_micsp(int vid, int pid, const char *serial)
{
    micsp_adapter_t *a;

    a = calloc (1, sizeof (*a));
    if (! a) {
        fprintf (stderr, "Out of memory\n");
        return 0;
    }
    a->name= adapter_name;
    a->rxbuf_len = 0;
    a->rxbuf_start = 0;
    a->bytes_to_read = 0;
    a->bytes_to_write = 0;
    a->context = NULL;

    int ret = libusb_init(&a->context);
    if (ret != 0) {
        fprintf(stderr, "libusb init failed: %d: %s\n",
            ret, libusb_strerror(ret));
        exit(-1);
    }

    a->usbdev = libusb_open_device_with_vid_pid(a->context, vid, pid);
    if(a->usbdev == NULL){
        fprintf(stderr, "Cannot access USB device.\n");
        free(a);
        return NULL;
    }

    ret = libusb_detach_kernel_driver(a->usbdev, 0);
    if (ret != 0 && ret != LIBUSB_ERROR_NOT_FOUND) {
        fprintf(stderr, "Error detaching kernel driver: %d: %s\n",
            ret, libusb_strerror(ret));
        libusb_close(a->usbdev);
        exit(-1);
    }

    libusb_claim_interface(a->usbdev, 0);

    /* Reset the ftdi device. */
    if (libusb_control_transfer(a->usbdev,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
        SIO_RESET, 0, 1, 0, 0, 1000) != 0)
    {
        if (errno == EPERM)
            fprintf(stderr, "%s: superuser privileges needed.\n", a->name);
        else
            fprintf(stderr, "%s: FTDI reset failed\n", a->name);
failed: libusb_release_interface(a->usbdev, 0);
        libusb_close(a->usbdev);
        free(a);
        return 0;
    }

    /* MPSSE mode. */
    if (libusb_control_transfer(a->usbdev,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
        SIO_SET_BITMODE, 0x20b, 1, 0, 0, 1000) != 0) {
        fprintf(stderr, "%s: can't set sync mpsse mode\n", a->name);
        goto failed;
    }

    /* Optimal latency timer is 1 for slow mode and 0 for fast mode. */
    const unsigned MHZ = 6;
    unsigned latency_timer = (MHZ > 6) ? 0 : 1;
    if (libusb_control_transfer(a->usbdev,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
        SIO_SET_LATENCY_TIMER, latency_timer, 1, 0, 0, 1000) != 0) {
        fprintf(stderr, "%s: unable to set latency timer\n", a->name);
        goto failed;
    }
    if (libusb_control_transfer(a->usbdev,
        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN,
        SIO_GET_LATENCY_TIMER, 0, 1, (unsigned char*) &latency_timer, 1, 1000) != 1) {
        fprintf(stderr, "%s: unable to get latency timer\n", a->name);
        goto failed;
    }
    if (debug_level)
        fprintf(stderr, "%s: latency timer: %u usec\n", a->name, latency_timer);

    set_clock_frequency(a, 3000); /* 3 MHz */

    enter_icsp_mode(a);
    unsigned status = get_device_status(a);
    if (debug_level > 0)
        fprintf (stderr, "%s: status %02x\n", a->name, status);
    if ((status & ~MCHP_STATUS_DEVRST) !=
        (MCHP_STATUS_CPS | MCHP_STATUS_CFGRDY))
    {
        fprintf (stderr, "%s: invalid status = %02x\n", a->name, status);
        //mpsse_reset (a, 0, 0, 0);
        free(a);
        return 0;
    }
    printf ("      Adapter: %s\n", a->name);
    a->adapter.flags = (AD_PROBE | AD_ERASE | AD_READ | AD_WRITE);

    /* User functions. */
    a->adapter.close = micsp_close;
    a->adapter.get_idcode = micsp_get_idcode;
    a->adapter.load_executive = micsp_load_executive;
    a->adapter.read_word = micsp_read_word;
    a->adapter.read_data = micsp_read_data;
    a->adapter.verify_data = micsp_verify_data;
    a->adapter.erase_chip = micsp_erase_chip;
    a->adapter.program_word = micsp_program_word;
    a->adapter.program_row = micsp_program_row;
    return &a->adapter;
}
