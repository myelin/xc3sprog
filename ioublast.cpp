/*
 *   Driver for USB-JTAG, Altera USB-Blaster and compatibles
 *
 *   Ported from OpenOCD.
 *   Inspired from original code from Kolja Waschk's USB-JTAG project
 *   (http://www.ixo.de/info/usb_jtag/), and from openocd project.
 *
 *   Copyright (C) 2018 Google LLC philpearson@google.com
 *   Copyright (C) 2013 Franck Jullien franck.jullien@gmail.com
 *   Copyright (C) 2012 Robert Jarzmik robert.jarzmik@free.fr
 *   Copyright (C) 2011 Ali Lown ali@lown.me.uk
 *   Copyright (C) 2009 Catalin Patulea cat@vv.carleton.ca
 *   Copyright (C) 2006 Kolja Waschk usbjtag@ixo.de
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
 * The following information is originally from Kolja Waschk's USB-JTAG,
 * where it was obtained by reverse engineering an Altera USB-Blaster.
 * See http://www.ixo.de/info/usb_jtag/ for USB-Blaster block diagram and
 * usb_jtag-20080705-1200.zip#usb_jtag/host/openocd for protocol.
 *
 * The same information is also on the UrJTAG mediawiki, with some additional
 * notes on bits marked as "unknown" by usb_jtag.
 * (http://sourceforge.net/apps/mediawiki/urjtag/index.php?
 *    title=Cable_Altera_USB-Blaster)
 *
 * USB-JTAG, Altera USB-Blaster and compatibles are typically implemented as
 * an FTDIChip FT245 followed by a CPLD which handles a two-mode protocol:
 *
 *            _________
 *           |         |
 *           | AT93C46 |
 *           |_________|
 *            __|__________    _________
 *           |             |  |         |
 *      USB__| FTDI 245BM  |__| EPM7064 |__JTAG (B_TDO,B_TDI,B_TMS,B_TCK)
 *           |_____________|  |_________|
 *            __|__________    _|___________
 *           |             |  |             |
 *           | 6 MHz XTAL  |  | 24 MHz Osc. |
 *           |_____________|  |_____________|
 *
 * USB-JTAG, Altera USB-Blaster II are typically implemented as a Cypress
 * EZ-USB FX2LP followed by a CPLD.
 *            _____________    _________
 *           |             |  |         |
 *      USB__| EZ-USB FX2  |__| EPM570  |__JTAG (B_TDO,B_TDI,B_TMS,B_TCK)
 *           |_____________|  |_________|
 *            __|__________
 *           |             |
 *           | 24 MHz XTAL |
 *           |_____________|
 */

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#include "ioublast.h"
#include "ublast_access.h"

/* Size of USB endpoint max packet size, ie. 64 bytes */
#define MAX_PACKET_SIZE 64
/*
 * Size of data buffer that holds bytes in byte-shift mode.
 * This buffer can hold multiple USB packets aligned to
 * MAX_PACKET_SIZE bytes boundaries.
 * BUF_LEN must be grater than or equal MAX_PACKET_SIZE.
 */
#define BUF_LEN 4096

/* USB-Blaster II specific command */
#define CMD_COPY_TDO_BUFFER	0x5F

enum gpio_steer {
	FIXED_0 = 0,
	FIXED_1,
	SRST,
	TRST,
};

struct ublast_info {
	enum gpio_steer pin6;
	enum gpio_steer pin8;
	int tms;
	int tdi;
	bool trst_asserted;
	bool srst_asserted;
	uint8_t buf[BUF_LEN];
	int bufidx;

	char *lowlevel_name;
	struct ublast_lowlevel *drv;
	char *ublast_device_desc;
	uint16_t ublast_vid, ublast_pid;
	uint16_t ublast_vid_uninit, ublast_pid_uninit;
	int flags;
	char *firmware_path;
};

/*
 * Global device control
 */
static struct ublast_info info = {
	.ublast_vid = 0x09fb, /* Altera */
	.ublast_pid = 0x6001, /* USB-Blaster */
	.lowlevel_name = NULL,
	.srst_asserted = false,
	.trst_asserted = false,
	.pin6 = FIXED_1,
	.pin8 = FIXED_1,
};

/*
 * Available lowlevel drivers (FTDI, FTD2xx, ...)
 */
struct drvs_map {
	const char *name;
	struct ublast_lowlevel *(*drv_register)(void);
};

static struct drvs_map lowlevel_drivers_map[] = {
#if BUILD_USB_BLASTER
	{ .name = "ftdi", .drv_register = ublast_register_ftdi },
#endif
#if BUILD_USB_BLASTER_2
	{ .name = "ublast2", .drv_register = ublast2_register_libusb },
#endif
	{ NULL, NULL },
};

IOUblast::IOUblast()
{
}

/*
 * Access functions to lowlevel driver, agnostic of libftdi/libftdxx
 */
static char *hexdump(uint8_t *buf, unsigned int size)
{
	unsigned int i;
	char *str = (char *)calloc(size * 2 + 1, 1);

	for (i = 0; i < size; i++)
		sprintf(str + 2*i, "%02x", buf[i]);
	return str;
}

static int ublast_buf_read(uint8_t *buf, unsigned size, uint32_t *bytes_read)
{
	int ret = info.drv->read(info.drv, buf, size, bytes_read);
	char *str = hexdump(buf, *bytes_read);

	DEBUG_JTAG_IO("(size=%d, buf=[%s]) -> %u", size, str,
		      *bytes_read);
	free(str);
	return ret;
}

static int ublast_buf_write(uint8_t *buf, int size, uint32_t *bytes_written)
{
	int ret = info.drv->write(info.drv, buf, size, bytes_written);
	char *str = hexdump(buf, *bytes_written);

	DEBUG_JTAG_IO("(size=%d, buf=[%s]) -> %u", size, str,
		      *bytes_written);
	free(str);
	return ret;
}

static int nb_buf_remaining(void)
{
	return BUF_LEN - info.bufidx;
}

static void ublast_flush_buffer(void)
{
	unsigned int retlen;
	int nb = info.bufidx, ret = ERROR_OK;

	while (ret == ERROR_OK && nb > 0) {
		ret = ublast_buf_write(info.buf, nb, &retlen);
		nb -= retlen;
	}
	info.bufidx = 0;
}

/*
 * Actually, the USB-Blaster offers a byte-shift mode to transmit up to 504 data
 * bits (bidirectional) in a single USB packet. A header byte has to be sent as
 * the first byte in a packet with the following meaning:
 *
 *   Bit 7 (0x80): Must be set to indicate byte-shift mode.
 *   Bit 6 (0x40): If set, the USB-Blaster will also read data, not just write.
 *   Bit 5..0:     Define the number N of following bytes
 *
 * All N following bytes will then be clocked out serially on TDI. If Bit 6 was
 * set, it will afterwards return N bytes with TDO data read while clocking out
 * the TDI data. LSB of the first byte after the header byte will appear first
 * on TDI.
 */

/* Simple bit banging mode:
 *
 *   Bit 7 (0x80): Must be zero (see byte-shift mode above)
 *   Bit 6 (0x40): If set, you will receive a byte indicating the state of TDO
 *                 in return.
 *   Bit 5 (0x20): Output Enable/LED.
 *   Bit 4 (0x10): TDI Output.
 *   Bit 3 (0x08): nCS Output (not used in JTAG mode).
 *   Bit 2 (0x04): nCE Output (not used in JTAG mode).
 *   Bit 1 (0x02): TMS Output.
 *   Bit 0 (0x01): TCK Output.
 *
 * For transmitting a single data bit, you need to write two bytes (one for
 * setting up TDI/TMS/TCK=0, and one to trigger TCK high with same TDI/TMS
 * held). Up to 64 bytes can be combined in a single USB packet.
 * It isn't possible to read a data without transmitting data.
 */

#define TCK		(1 << 0)
#define TMS		(1 << 1)
#define NCE		(1 << 2)
#define NCS		(1 << 3)
#define TDI		(1 << 4)
#define LED		(1 << 5)
#define READ		(1 << 6)
#define SHMODE		(1 << 7)
#define READ_TDO	(1 << 0)

/**
 * ublast_queue_byte - queue one 'bitbang mode' byte for USB Blaster
 * @abyte: the byte to queue
 *
 * Queues one byte in 'bitbang mode' to the USB Blaster. The byte is not
 * actually sent, but stored in a buffer. The write is performed once
 * the buffer is filled, or if an explicit ublast_flush_buffer() is called.
 */
static void ublast_queue_byte(uint8_t abyte)
{
	if (nb_buf_remaining() < 1)
		ublast_flush_buffer();
	info.buf[info.bufidx++] = abyte;
	if (nb_buf_remaining() == 0)
		ublast_flush_buffer();
	DEBUG_JTAG_IO("(byte=0x%02x)", abyte);
}

/**
 * ublast_compute_pin - compute if gpio should be asserted
 * @steer: control (ie. TRST driven, SRST driven, of fixed)
 *
 * Returns pin value (1 means driven high, 0 mean driven low)
 */
bool ublast_compute_pin(enum gpio_steer steer)
{
	switch (steer) {
	case FIXED_0:
		return 0;
	case FIXED_1:
		return 1;
	case SRST:
		return !info.srst_asserted;
	case TRST:
		return !info.trst_asserted;
	default:
		return 1;
	}
}

/**
 * ublast_build_out - build bitbang mode output byte
 * @type: says if reading back TDO is required
 *
 * Returns the compute bitbang mode byte
 */
static uint8_t ublast_build_out(enum scan_type type)
{
	uint8_t abyte = 0;

	abyte |= info.tms ? TMS : 0;
	abyte |= ublast_compute_pin(info.pin6) ? NCE : 0;
	abyte |= ublast_compute_pin(info.pin8) ? NCS : 0;
	abyte |= info.tdi ? TDI : 0;
	abyte |= LED;
	if (type == SCAN_IN || type == SCAN_IO)
		abyte |= READ;
	return abyte;
}

/**
 * ublast_clock_tms - clock a TMS transition
 * @tms: the TMS to be sent
 *
 * Triggers a TMS transition (ie. one JTAG TAP state move).
 */
static void ublast_clock_tms(int tms)
{
	uint8_t out;

	DEBUG_JTAG_IO("(tms=%d)", !!tms);
	info.tms = !!tms;
	info.tdi = 0;
	out = ublast_build_out(SCAN_OUT);
	ublast_queue_byte(out);
	ublast_queue_byte(out | TCK);
}

/**
 * ublast_idle_clock - put back TCK to low level
 *
 * See ublast_queue_tdi() comment for the usage of this function.
 */
static void ublast_idle_clock(void)
{
	uint8_t out = ublast_build_out(SCAN_OUT);

	DEBUG_JTAG_IO(".");
	ublast_queue_byte(out);
}

/**
 * ublast_clock_tdi - Output a TDI with bitbang mode
 * @tdi: the TDI bit to be shifted out
 * @type: scan type (ie. does a readback of TDO is required)
 *
 * Output a TDI bit and assert clock to push it into the JTAG device :
 *  - writing out TCK=0, TMS=<old_state>=0, TDI=<tdi>
 * - writing out TCK=1, TMS=<new_state>, TDI=<tdi> which triggers the JTAG
 *    device aquiring the data.
 *
 * If a TDO is to be read back, the required read is requested (bitbang mode),
 * and the USB Blaster will send back a byte with bit0 reprensenting the TDO.
 */
static void ublast_clock_tdi(int tdi, enum scan_type type)
{
	uint8_t out;

	DEBUG_JTAG_IO("(tdi=%d)",  !!tdi);
	info.tdi = !!tdi;

	out = ublast_build_out(SCAN_OUT);
	ublast_queue_byte(out);

	out = ublast_build_out(type);
	ublast_queue_byte(out | TCK);
}

/**
 * ublast_clock_tdi_flip_tms - Output a TDI with bitbang mode, change JTAG state
 * @tdi: the TDI bit to be shifted out
 * @type: scan type (ie. does a readback of TDO is required)
 *
 * This function is the same as ublast_clock_tdi(), but it changes also the TMS
 * while outputing the TDI. This should be the last TDI output of a TDI
 * sequence, which will change state from :
 *   - IRSHIFT -> IREXIT1
 *   - or DRSHIFT -> DREXIT1
 */
static void ublast_clock_tdi_flip_tms(int tdi, enum scan_type type)
{
	uint8_t out;

	DEBUG_JTAG_IO("(tdi=%d)", !!tdi);
	info.tdi = !!tdi;
	info.tms = !info.tms;

	out = ublast_build_out(SCAN_OUT);
	ublast_queue_byte(out);

	out = ublast_build_out(type);
	ublast_queue_byte(out | TCK);

	out = ublast_build_out(SCAN_OUT);
	ublast_queue_byte(out);
}

/**
 * ublast_queue_bytes - queue bytes for the USB Blaster
 * @bytes: byte array
 * @nb_bytes: number of bytes
 *
 * Queues bytes to be sent to the USB Blaster. The bytes are not
 * actually sent, but stored in a buffer. The write is performed once
 * the buffer is filled, or if an explicit ublast_flush_buffer() is called.
 */
static void ublast_queue_bytes(const uint8_t *bytes, int nb_bytes)
{
	if (info.bufidx + nb_bytes > BUF_LEN) {
		LOG_ERROR("buggy code, should never queue more that %d bytes",
			  info.bufidx + nb_bytes);
		exit(-1);
	}
	DEBUG_JTAG_IO("(nb_bytes=%d, bytes=[0x%02x, ...])", nb_bytes,
		      bytes ? bytes[0] : 0);
	if (bytes)
		memcpy(&info.buf[info.bufidx], bytes, nb_bytes);
	else
		memset(&info.buf[info.bufidx], 0, nb_bytes);
	info.bufidx += nb_bytes;
	if (nb_buf_remaining() == 0)
		ublast_flush_buffer();
}

/**
 * ublast_tms_seq - write a TMS sequence transition to JTAG
 * @bits: TMS bits to be written (bit0, bit1 .. bitN)
 * @nb_bits: number of TMS bits (between 1 and 8)
 *
 * Write a serie of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=<new_state>, TDI=<???>
 *  - writing out TCK=1, TMS=<new_state>, TDI=<???> which triggers the transition
 * The function ensures that at the end of the sequence, the clock (TCK) is put
 * low.
 */
static void ublast_tms_seq(const uint8_t *bits, int nb_bits)
{
	int i;

	DEBUG_JTAG_IO("(bits=%02x..., nb_bits=%d)", bits[0], nb_bits);
	for (i = 0; i < nb_bits; i++)
		ublast_clock_tms((bits[i / 8] >> (i % 8)) & 0x01);
	ublast_idle_clock();
}

void IOUblast::tx_tms(unsigned char *pat, int length, int force)
{
	ublast_tms_seq(pat, length);
	if (force)
		ublast_flush_buffer();
}

/**
 * ublast_read_byteshifted_tdos - read TDO of byteshift writes
 * @buf: the buffer to store the bits
 * @nb_bits: the number of bits
 *
 * Reads back from USB Blaster TDO bits, triggered by a 'byteshift write', ie. eight
 * bits per received byte from USB interface, and store them in buffer.
 *
 * As the USB blaster stores the TDO bits in LSB (ie. first bit in (byte0,
 * bit0), second bit in (byte0, bit1), ...), which is what we want to return,
 * simply read bytes from USB interface and store them.
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read error occured
 */
static int ublast_read_byteshifted_tdos(uint8_t *buf, int nb_bytes)
{
	unsigned int retlen;
	int ret = ERROR_OK;

	DEBUG_JTAG_IO("%s(buf=%p, num_bits=%d)", __func__, buf, nb_bytes * 8);
	ublast_flush_buffer();
	while (ret == ERROR_OK && nb_bytes > 0) {
		ret = ublast_buf_read(buf, nb_bytes, &retlen);
		nb_bytes -= retlen;
	}
	return ret;
}

/**
 * ublast_read_bitbang_tdos - read TDO of bitbang writes
 * @buf: the buffer to store the bits
 * @nb_bits: the number of bits
 *
 * Reads back from USB Blaster TDO bits, triggered by a 'bitbang write', ie. one
 * bit per received byte from USB interface, and store them in buffer, where :
 *  - first bit is stored in byte0, bit0 (LSB)
 *  - second bit is stored in byte0, bit 1
 *  ...
 *  - eight bit is sotred in byte0, bit 7
 *  - ninth bit is sotred in byte1, bit 0
 *  - etc ...
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read error occured
 */
static int ublast_read_bitbang_tdos(uint8_t *buf, int nb_bits)
{
	int nb1 = nb_bits;
	int i, ret = ERROR_OK;
	unsigned int retlen;
	uint8_t tmp[8];

	DEBUG_JTAG_IO("%s(buf=%p, num_bits=%d)", __func__, buf, nb_bits);
	if (!buf) {
		LOG_ERROR("%s called with NULL buf", __func__);
		exit(1);
	}

	/*
	 * Ensure all previous bitbang writes were issued to the dongle, so that
	 * it returns back the read values.
	 */
	ublast_flush_buffer();

	ret = ublast_buf_read(tmp, nb1, &retlen);
	for (i = 0; ret == ERROR_OK && i < nb1; i++)
		if (tmp[i] & READ_TDO)
			*buf |= (1 << i);
		else
			*buf &= ~(1 << i);
	return ret;
}

/**
 * ublast_queue_tdi - short description
 * @bits: bits to be queued on TDI (or NULL if 0 are to be queued)
 * @nb_bits: number of bits
 * @scan: scan type (ie. if TDO read back is required or not)
 *
 * Outputs a serie of TDI bits on TDI.
 * As a side effect, the last TDI bit is sent along a TMS=1, and triggers a JTAG
 * TAP state shift if input bits were non NULL.
 *
 * In order to not saturate the USB Blaster queues, this method reads back TDO
 * if the scan type requests it, and stores them back in bits.
 *
 * As a side note, the state of TCK when entering this function *must* be
 * low. This is because byteshift mode outputs TDI on rising TCK and reads TDO
 * on falling TCK if and only if TCK is low before queuing byteshift mode bytes.
 * If TCK was high, the USB blaster will queue TDI on falling edge, and read TDO
 * on rising edge !!!
 */
void IOUblast::txrx_block(const unsigned char *bits, unsigned char *tdos,
                           int nb_bits, bool last)
{
	int nb8 = nb_bits / 8;
	int nb1 = nb_bits % 8;
	int nbfree_in_packet, i, trans = 0, read_tdos;
	static uint8_t byte0[BUF_LEN];
	const enum scan_type scan = (tdos ? SCAN_IO : SCAN_OUT);

	DEBUG_JTAG_IO("%s(bits=%p, tdos=%p, num_bits=%d)", __func__, bits, tdos, nb_bits);

	/*
	 * As the last TDI bit should always be output in bitbang mode in order
	 * to activate the TMS=1 transition to EXIT_?R state. Therefore a
	 * situation where nb_bits is a multiple of 8 is handled as follows:
	 * - the number of TDI shifted out in "byteshift mode" is 8 less than
	 *   nb_bits
	 * - nb1 = 8
	 * This ensures that nb1 is never 0, and allows the TMS transition.
	 */
	if (nb8 > 0 && nb1 == 0) {
		nb8--;
		nb1 = 8;
	}

	read_tdos = (scan == SCAN_IN || scan == SCAN_IO);
	for (i = 0; i < nb8; i += trans) {
		/*
		 * Calculate number of bytes to fill USB packet of size MAX_PACKET_SIZE
		 */
		nbfree_in_packet = (MAX_PACKET_SIZE - (info.bufidx%MAX_PACKET_SIZE));
		trans = MIN(nbfree_in_packet - 1, nb8 - i);

		/*
		 * Queue a byte-shift mode transmission, with as many bytes as
		 * is possible with regard to :
		 *  - current filling level of write buffer
		 *  - remaining bytes to write in byte-shift mode
		 */
		if (read_tdos)
			ublast_queue_byte(SHMODE | READ | trans);
		else
			ublast_queue_byte(SHMODE | trans);
		if (bits)
			ublast_queue_bytes(&bits[i], trans);
		else
			ublast_queue_bytes(byte0, trans);
		if (read_tdos) {
			if (info.flags & COPY_TDO_BUFFER)
				ublast_queue_byte(CMD_COPY_TDO_BUFFER);
			ublast_read_byteshifted_tdos(&tdos[i], trans);
		}
	}

	/*
	 * Queue the remaining TDI bits in bitbang mode.
	 */
	for (i = 0; i < nb1; i++) {
		int tdi = bits ? bits[nb8 + i / 8] & (1 << i) : 0;
		if (bits && i == nb1 - 1 && last)
			ublast_clock_tdi_flip_tms(tdi, scan);
		else
			ublast_clock_tdi(tdi, scan);
	}
	if (nb1 && read_tdos) {
		if (info.flags & COPY_TDO_BUFFER)
			ublast_queue_byte(CMD_COPY_TDO_BUFFER);
		ublast_read_bitbang_tdos(&tdos[nb8], nb1);
	}

	/*
	 * Ensure clock is in lower state
	 */
	ublast_idle_clock();
}

static void ublast_initial_wipeout(void)
{
	static uint8_t tms_reset = 0xff;
	uint8_t out_value;
	uint32_t retlen;
	int i;

	out_value = ublast_build_out(SCAN_OUT);
	for (i = 0; i < BUF_LEN; i++)
		info.buf[i] = out_value | ((i % 2) ? TCK : 0);

	/*
	 * Flush USB-Blaster queue fifos
	 *  - empty the write FIFO (128 bytes)
	 *  - empty the read FIFO (384 bytes)
	 */
	ublast_buf_write(info.buf, BUF_LEN, &retlen);
	/*
	 * Put JTAG in RESET state (five 1 on TMS)
	 */
	ublast_tms_seq(&tms_reset, 5);
}

/**
 * ublast_init - Initialize the Altera device
 *
 * Initialize the device :
 *  - open the USB device
 *  - pretend it's initialized while actual init is delayed until first jtag command
 *
 * Returns ERROR_OK if USB device found, error if not.
 */
int IOUblast::Init(struct cable_t *cable, const char *dev, unsigned int freq)
{
	int ret, i;

	for (i = 0; lowlevel_drivers_map[i].name; i++) {
		if (info.lowlevel_name) {
			if (!strcmp(lowlevel_drivers_map[i].name, info.lowlevel_name)) {
				info.drv = lowlevel_drivers_map[i].drv_register();
				if (!info.drv) {
					LOG_ERROR("Error registering lowlevel driver \"%s\"",
						  info.lowlevel_name);
					return ERROR_JTAG_INIT_FAILED;
				}
				break;
			}
		} else {
			info.drv = lowlevel_drivers_map[i].drv_register();
			if (info.drv) {
				info.lowlevel_name = strdup(lowlevel_drivers_map[i].name);
				LOG_INFO("No lowlevel driver configured, using %s", info.lowlevel_name);
				break;
			}
		}
	}

	if (!info.drv) {
		LOG_ERROR("No lowlevel driver available");
		return ERROR_JTAG_INIT_FAILED;
	}

	/*
	 * Register the lowlevel driver
	 */
	info.drv->ublast_vid = info.ublast_vid;
	info.drv->ublast_pid = info.ublast_pid;
	info.drv->ublast_vid_uninit = info.ublast_vid_uninit;
	info.drv->ublast_pid_uninit = info.ublast_pid_uninit;
	info.drv->ublast_device_desc = info.ublast_device_desc;
	info.drv->firmware_path = info.firmware_path;

	info.flags |= info.drv->flags;

	ret = info.drv->open(info.drv);

	// Reset TAP
	ublast_initial_wipeout();

	return ret;
}

void IOUblast::flush()
{
	ublast_flush_buffer();
}

/**
 * ublast_quit - Release the Altera device
 *
 * Releases the device :
 *   - put the device pins in 'high impedance' mode
 *   - close the USB device
 *
 * Returns always ERROR_OK
 */
IOUblast::~IOUblast()
{
	uint8_t byte0 = 0;
	unsigned int retlen;

	ublast_buf_write(&byte0, 1, &retlen);
	info.drv->close(info.drv);
}
