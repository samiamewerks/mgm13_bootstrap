/***************************************************************************//**
 *
 * @file
 * @brief Enel x MGM13S SPI bootstrap
 *
 * Loads program code from the SPI using the bluetooth SPI packet protocol and
 * flashes that to internal flash memory, then starts it.
 *
 *******************************************************************************
 * # License
 * <b>Copyright (C) 2019 Enel x</b>
 *******************************************************************************
 *
 ******************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_msc.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/* Application header */
#include "app.h"

/* SPI driver */
#include "spidrv.h"

/* location of resident program magic */
#define PROGRAM_MAGIC_LOC (USERDATA_BASE+0)

/* location of main program load */
#define PROGRAM_LOAD_LOC (0x800)

#define CMDLEN 5        /* length of standard command */
#define PKTLEN (2*1024) /* length of max packet + overhead */

/* magic number for resident program: ENLX */
#define MAGIC 0x454e4c58

#define FLSSIZ 1024 /* basic flash packet size */

/* GPIO Port A pin for IRQ to master */
#define IRQPIN 4

/* configuration lock word */
#define CLW0_ADDR 0xfe041e8

/* base address of bootstrap */
#define BOOTSTRAP_BASE 0x70000

/* define this if you want to override the startup of a valid resident program
   based on the presence of the magic number and download a resident program. */
//#define BURNMAGIC /* define this to burn the magic number */

/* SPI protocol commands */
typedef enum {

    spicmd_noop,       /* zero code, unused */
    spicmd_mts_len,    /* 1: master to slave packet length */
    spicmd_mts_pkt,    /* 2: master to slave packet transfer */
    spicmd_stm_len,    /* 3: slave to master packet length */
    spicmd_stm_pkt,    /* 4: slave to master packet transfer */
	spicmd_mts_gbs,    /* 5: go to bootstrap */
	spicmd_mts_sps,    /* 6: master to slave, start reprogram sequence */
	spicmd_mts_ppd,    /* 7: master to slave, pass program data */
	spicmd_mts_pex,    /* 8: master to slave, execute program */
	spicmd_rsf = 0xfd, /* reprogram sequence fails */
	spicmd_nrp = 0xfe, /* slave to master status, no resident program */
    spicmd_err = 0xff  /* protocol faulted, reset */

} spicmd;

SPIDRV_HandleData_t handleDataSlave;
SPIDRV_Handle_t handleSlave = &handleDataSlave;

uint8_t pkttxbuf[PKTLEN]; /* SPI transmit packet buffer */
uint8_t pktrxbuf[PKTLEN]; /* SPI receive packet buffer */

int pktrxlen; /* receive (master to slave) length (pkt+protocol) */
int pkttxlen; /* transmit (slave to master) length (pkt+protocol) */

spicmd spistate; /* state of SPI transfers */

/* current flash program load address, must be mod 2kb */
uint8_t* fladdr;
unsigned long magicval = MAGIC; /* magic value for writing */

/* go address for resident program */
volatile int goaddr;

/*******************************************************************************

Calculate 16 bit CRC

Finds a 16 bit CRC according to CRC-CCIT-16 using a 0xFFFF starting word.
The block of bytes to be checked is given, with the length of the block.

*******************************************************************************/

unsigned short crc16(const unsigned char* data_p, unsigned length)

{

    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--){

        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^
              ((unsigned short)(x <<5)) ^ ((unsigned short)x);

    }

    return crc;

}

/*******************************************************************************

Calculate 32 bit CRC

Finds a 32 bit CRC using the same algorithm as Ethernet CRC. The block of bytes
to be checked is given, with the length of the block.

*******************************************************************************/

unsigned long crc32(const unsigned char* data, unsigned long length)

{

    unsigned int crc = 0xffffffff;	/* Initial value. */
    unsigned char current_octet;
    int bit;

    while(length-- > 0) {

	    current_octet = *data++;

	    for (bit = 8; --bit >= 0; current_octet >>= 1) {

	        if ((crc ^ current_octet) & 1) {

	            crc >>= 1;
	            crc ^= 0xedb88320U; /* ethernet polynomial */

	        } else crc >>= 1;

	    }

    }

    return (crc);

}

/*******************************************************************************

Receive bluetooth packet

Called when a 1kb block of program flash data is received. If the address given
by the flash memory pointer fladdr is divisible by 2kb, then that page is
erased. Regardless of position, a 1kb block in pktrxbuf is programmed. fladdr
is always aligned by 1kb increments (the lower 10 bits is all zeros). The input
buffer address is not aligned.

Returns non-zero on error.

*******************************************************************************/

int program_packet_receive(uint8_t* buff)

{

	msc_Return_TypeDef r;

	/* perform sector erase if on 2kb boundary */
    r = 0;
	if (!((uint32_t)fladdr & 0x7ff)) r = MSC_ErasePage((uint32_t*)fladdr);
	 /* write data block */
	if (!r)	r = MSC_WriteWord((uint32_t*)fladdr, buff, FLSSIZ);

	/* return with error status */
	return (!!r);

}

int spixfr(uint8_t* txbuf, uint8_t* rxbuf, int len);

/*******************************************************************************

Callback for SPI transfer

This routine is called on completion of SPI transfer exchanges (receive and
transmit). This routine and spixfr call each other, and this routine essentially
runs a state machine regulating the various transfer states between the master
and the slave.

*******************************************************************************/

void STransferCB(struct SPIDRV_HandleData *handle,
                 Ecode_t transferStatus,
                 int itemsTransferred)

{

    unsigned short crc, rcrc;
    unsigned long crcl, rcrcl;
    int len = 0;
    int r;

    len = 0; /* clear length */
    /* set length and buffer according to state */
    if (spistate == spicmd_mts_gbs || spistate == spicmd_mts_sps ||
        spistate == spicmd_nrp) len = 3;
    else len = FLSSIZ+1;
    crc = crc16(pktrxbuf, len); /* find received CRC */
    rcrc = pktrxbuf[len] << 8 | pktrxbuf[len+1];
    if (crc != rcrc) { /* CRCs don't match */

    	printf("CRC fail: was: %04x s/b: %04x\r\n", crc, rcrc);
    	spistate = spicmd_err;

    }
    if (pktrxbuf[0] < spicmd_mts_len ||
        (pktrxbuf[0] > spicmd_mts_pex && pktrxbuf[0] < spicmd_rsf))
    	spistate = spicmd_err;

    /* if we are in "no resident program" state, and the master wants to go to
       reprogram state, that is a valid transition */
    if (pktrxbuf[0] == spicmd_mts_sps && spistate == spicmd_nrp)
    	spistate = spicmd_mts_sps;

    /* if we are in "pass program" state, and the master wants to go to
       program execute state, that is a valid transition */
    if (pktrxbuf[0] == spicmd_mts_pex && spistate == spicmd_mts_ppd)
    	spistate = spicmd_mts_pex;

    /* if we are in the "start program state", and master wants "go bootstrap"
       state, we got this because master does not know we are already in
       bootstrap. The state is a no-op, but this is allowed */
    if (pktrxbuf[0] == spicmd_mts_gbs &&
        (spistate == spicmd_mts_sps || spistate == spicmd_nrp))
        spistate = spicmd_mts_gbs;

    if (pktrxbuf[0] != spistate) /* check expected state */
        spistate = spicmd_err; /* states/commands do not match */

    /* Now run the state machine. Remember, these are the states at the END
       of a transfer! */
    switch (spistate) {

        case spicmd_noop:
		    break;
        case spicmd_mts_gbs:
        	/* Go bootstrap: We are already in the bootstrap, so this is a
        	   no-op. We do this because the master needs to kick us out of
        	   any active resident program. */
        	spistate = spicmd_mts_sps; /* go to pass program data */
        	pkttxbuf[0] = spicmd_mts_sps;
        	r = spixfr(pkttxbuf, pktrxbuf, 3); /* transfer length */
            if (r) spistate = spicmd_err; /* go to error state */
            break;
        case spicmd_mts_sps:
        	/* start reprogram sequence */
        	spistate = spicmd_mts_ppd; /* go to pass program data */
        	pkttxbuf[0] = spicmd_mts_ppd;
        	r = spixfr(pkttxbuf, pktrxbuf, FLSSIZ+1); /* transfer 1kb */
            if (r) spistate = spicmd_err; /* go to error state */
            /* pulse master IRQ to signal armed and ready */
            GPIO_PinModeSet(gpioPortA, IRQPIN, gpioModePushPull, 0);
            GPIO_PinModeSet(gpioPortA, IRQPIN, gpioModePushPull, 1);
            break;
        /* don't merge these two cases, the compiler does that */
        case spicmd_mts_ppd:
        	if (fladdr >= (uint8_t*)BOOTSTRAP_BASE) spistate = spicmd_err;
        	else {

            	/* pass program data */
        	    program_packet_receive(pktrxbuf+1);
        	    fladdr += FLSSIZ;
        	    pkttxbuf[0] = spicmd_mts_ppd;
                r = spixfr(pkttxbuf, pktrxbuf, FLSSIZ+1); /* transfer 1kb */
                if (r) spistate = spicmd_err; /* go to error state */

        	}
            /* pulse master IRQ to signal armed and ready */
            GPIO_PinModeSet(gpioPortA, IRQPIN, gpioModePushPull, 0);
            GPIO_PinModeSet(gpioPortA, IRQPIN, gpioModePushPull, 1);
        	break;
        case spicmd_mts_pex: /* execute resident program */
            printf("Program flashed, check CRC: length: %d\r\n",
            	   fladdr-(uint8_t*)PROGRAM_LOAD_LOC);
        	/* find crc for entire program in flash */
        	crcl = crc32((unsigned char*)PROGRAM_LOAD_LOC,
        			     fladdr-(uint8_t*)PROGRAM_LOAD_LOC);
        	/* get crc from master */
        	rcrcl = pktrxbuf[1] << 24 | pktrxbuf[2] << 16 | pktrxbuf[3] << 8 | pktrxbuf[4];
        	if (crcl != rcrcl) {

        		printf("Program fails CRC\r\n");
        		spistate = spicmd_rsf; /* flag program failure */

        	} else {

        		/* place resident magic to indicate programmed */
                r = MSC_ErasePage((uint32_t*)USERDATA_BASE);
        		if (!r) r = MSC_WriteWord((uint32_t*)USERDATA_BASE, &magicval, sizeof(uint32_t));
        		if (r) {

        			spistate = spicmd_rsf; /* fails */
        			printf("Unable to program magic number\r\n");

        		}
        		else {

            		MSC_Deinit(); /* shut off flash programming */
                    printf("Executing resident program\r\n");
            		/* go program */;
            		goaddr = 1;

        		}

        	}
        	break;
        /* standard bluetooth packet codes, these are all errors here */
        case spicmd_mts_len/*1*/:
        case spicmd_mts_pkt/*2*/:
        case spicmd_stm_len/*3*/:
        case spicmd_stm_pkt/*4*/:
        	spistate = spicmd_err;
        /* error states */
        case spicmd_rsf:
        case spicmd_nrp:
        case spicmd_err:
        	/* In error state, we continually send error back to master.
        	   We never leave error state, but instead get reset by the
        	   master. */
        	pkttxbuf[0] = spistate;
        	spixfr(pkttxbuf, pktrxbuf, 3); /* set transfer */
		    break;

    }

}

/*******************************************************************************

Run SPI transfer

Runs a transfer on the SPI bus. A SPI transfer is a simultaneous receive and
transmit operation. The dual DMAs are programmed, one for each of the slave
receive and transmit buffers, and with a matching length. The length is the
maximum length. If the CS ends from the master, the transfer terminates.

*******************************************************************************/

int spixfr(uint8_t* txbuf, uint8_t* rxbuf, int len)

{

    unsigned short crc;
    Ecode_t r;

    // Start a SPI slave receive transfer.
    crc = crc16(txbuf, len); /* calculate CRC on payload */
    txbuf[len] = crc >> 8; /* place CRC */
    txbuf[len+1] = crc & 0xff;
    /* start transfer with callback */
    r = SPIDRV_STransfer(handleSlave, txbuf, rxbuf, len+2, STransferCB, 0);

    return (!!r); /* return with error code */

}

/*******************************************************************************

Start resident program

Shuts down the clocks, loads the SP from the vector at 0x800, and then branches
to the vector at 0x804.

*******************************************************************************/

void start_resident(void)

{

	/* bring the clocks to original state */
	deinitMcu();

	/* execute the resident program by the vector table */
	__ASM (
	    "mov r3,0x800\n\t"
	    "ldr r4,[r3]\n\t"
	    "mov sp,r4\n\t"
	    "add r3,#4\n\t"
	    "ldr r3,[r3]\n\t"
	    "bx r3\n\t"
	);

	/* just for paranoia, this should never execute */
	while (1);

}

/**
 * @brief  Main function
 */
int main(void)

{

	int r;
	uint32_t *addr_clw0 = (uint32_t*) CLW0_ADDR;
	uint32_t clear_boot_enable = ~0x2;

    /* Initialize device */
    initMcu();

    printf("\r\nBluetooth bootstrap program vs. 1.3\r\n");

    printf("Configuration lock word: %08lx\r\n", *addr_clw0);

    if (*addr_clw0 == 0xffffffff) {

    	printf("Programming boot bit\r\n");
        r = MSC_WriteWord(addr_clw0, &clear_boot_enable, 4);
        if (r) printf("Programming boot enable bit fails\r\n");
        printf("Configuration lock word after programming: %08lx\r\n", *addr_clw0);

    }

    /* Initialize board */
    initBoard();
    /* Initialize application */
    initApp();

    /* set IRQ pin to WIFI inactive (low true) */
    GPIO_PinModeSet(gpioPortA, IRQPIN, gpioModePushPull, 1);

    // SPIDRV_Init_t
    // PC6 - MOSI
    // PC7 - MISO
    // PC8 - CLK  connect to the CLK of the master
    // PC9 - CS
    SPIDRV_Init_t initDataSlave = SPIDRV_SLAVE_USART1;

    // Initialize the USART1 a SPI slave
    SPIDRV_Init( handleSlave, &initDataSlave );

    /* open the MSC/flash package */
    MSC_Init();

/* burn this page if you need to not exec the program image */
#ifdef BURNMAGIC
    MSC_ErasePage((uint32_t*)USERDATA_BASE);
#endif

    /* set up flash program load address */
    fladdr = (uint8_t*)PROGRAM_LOAD_LOC;

    /*
     * Start the first SPI transfer. The SPI engine is driven by completions
     * after that.
     *
     */

    /* check resident program exists */
    if (*((unsigned long*)USERDATA_BASE) == MAGIC) {

    	printf("Resident program detected, executing at: %08x", PROGRAM_LOAD_LOC);
		/* go program */;
		start_resident();

    }

  	/* set no resident program */
   	spistate = spicmd_nrp;
    pkttxbuf[0] = spistate;
    r = spixfr(pkttxbuf, pktrxbuf, 3); /* set DMAs to run */
    if (r) spistate = spicmd_err; /* failed, put in error state */

    /* halt until go command is seen */
    while (!goaddr);
    start_resident();

}
