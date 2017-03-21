/* JTAG low-level I/O to JLink

Using the libjaylink library

Copyright (C) 2005-2011 Uwe Bonnes bon@elektron.ikp.physik.tu-darmstadt.de
Copyright (C) 2017 Google Inc, philpearson@google.com

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/time.h>

#include <string.h>
#include <cstdio>

#include "iojlink.h"
#include "io_exception.h"
#include "utilities.h"

// #define IOJLINK_VERBOSE

IOJLink::IOJLink()
:  IOBase(), bptr(0), calls_rd(0) , calls_wr(0), context(NULL)
{
}

int IOJLink::Init(struct cable_t *cable, char const *serial, unsigned int freq)
{
  int rc;
  fprintf(stderr, "Connecting to J-Link\n");

  rc = jaylink_init(&context);
  if (rc != JAYLINK_OK) {
    fprintf(stderr, "jaylink_init: %s\n", jaylink_strerror_name(rc));
    return 1;
  }

  struct jaylink_device **devices;
  ssize_t n_devices = jaylink_get_device_list(context, &devices);
  if (!n_devices) {
    fprintf(stderr, "No J-Link devices found\n");
    return 1;
  }

  device = NULL;
  for (int i = 0; devices[i] != NULL; ++i) {
    rc = jaylink_open(devices[i], &device);
    if (rc != JAYLINK_OK) {
      fprintf(stderr, "jaylink_open: %s\n", jaylink_strerror_name(rc));
      return 1;
    }
  }

  jaylink_free_device_list(devices, 1);

  if (device == NULL) {
    fprintf(stderr, "Unable to find attached JLink device.\n");
    return 1;
  }

  rc = jaylink_select_interface(device, JAYLINK_TIF_JTAG, NULL);
  if (rc != JAYLINK_OK) {
    fprintf(stderr, "jaylink_select_interface: %s\n", jaylink_strerror_name(rc));
    return 1;
  }

  uint32_t jl_freq;
  uint16_t jl_div;
  rc = jaylink_get_speeds(device, &jl_freq, &jl_div);
  if (rc != JAYLINK_OK) {
    fprintf(stderr, "jaylink_get_speeds: %s\n", jaylink_strerror_name(rc));
    return 1;
  }
  fprintf(stderr, "J-Link speed caps: freq=%d, divider=%d, i.e. max speed=%d kHz\n", jl_freq, jl_div, jl_freq / jl_div / 1000);
  if (jl_freq / jl_div < freq) {
    freq = jl_freq / jl_div;
  }

  fprintf(stderr, "J-Link: setting speed to %d kHz\n", freq / 1000);
  rc = jaylink_set_speed(device, freq / 1000);
  if (rc != JAYLINK_OK) {
    fprintf(stderr, "jaylink_set_speed: %s\n", jaylink_strerror_name(rc));
    return 1;
  }

  return 0;
}

IOJLink::~IOJLink()
{
  jaylink_exit(context);
}

void IOJLink::txrx_block(const unsigned char *tdi, unsigned char *tdo,
		       int length, bool last)
{
#ifdef IOJLINK_VERBOSE
  fprintf(stderr, "jlink txrx_block %d bits last=%d:", length, last);
  if (tdi == NULL)
    fprintf(stderr, " (zeros)");
  else {
    for (int pos = 0; pos < (length + 7) / 8; ++pos)
      fprintf(stderr, " %02x", tdi[pos]);
  }
  fprintf(stderr, "\n");
#endif // IOJLINK_VERBOSE

  // TMS should be 0 while we're shifting into IR or DR
  int tms_length = (length + 7) / 8;
  uint8_t tms[tms_length];
  memset(tms, 0, tms_length);

  // Throwaway buffers used when the caller doesn't care about TDI/TDO
  uint8_t dummy_tdi[tms_length];
  memset(dummy_tdi, 0, tms_length);
  if (tdi == NULL)
    tdi = dummy_tdi;
  uint8_t dummy_tdo[tms_length];
  if (tdo == NULL)
    tdo = dummy_tdo;

  // Except for a final 1 bit if we're exiting the SHIFT state
  if (last) {
    int last_pos = length - 1;
    tms[last_pos/8] = 1 << (last_pos % 8);
  }

  // Actually perform the JTAG I/O operation...
  int rc = jaylink_jtag_io(device, tms, tdi, tdo, length, JAYLINK_JTAG_V3);
  if (rc != JAYLINK_OK) {
    fprintf(stderr, "jaylink_jtag_io: %s\n", jaylink_strerror_name(rc));
    exit(1);
  }

#ifdef IOJLINK_VERBOSE
  fprintf(stderr, "--> tdo:");
  for (int pos = 0; pos < (length + 7) / 8; ++pos)
    fprintf(stderr, " %02x", tdo[pos]);
  fprintf(stderr, "\n");
#endif // IOJLINK_VERBOSE
}

void IOJLink::tx_tms(unsigned char *pat, int length, int force)
{
#ifdef IOJLINK_VERBOSE
  fprintf(stderr, "jlink tx_tms %d bits", length);
  for (int pos = 0; pos < (length + 7) / 8; ++pos)
    fprintf(stderr, " %02x", pat[pos]);
  fprintf(stderr, "\n");
#endif // IOJLINK_VERBOSE

  // TDI and TDO are irrelevant, but libjaylink expects them, so give it some zeroes
  int tms_length = (length + 7) / 8;
  uint8_t tdi[tms_length];
  memset(tdi, 0, tms_length);
  uint8_t tdo[tms_length];

  // Actually perform the JTAG I/O operation...
  int rc = jaylink_jtag_io(device, pat, tdi, tdo, length, JAYLINK_JTAG_V3);
  if (rc != JAYLINK_OK) {
    fprintf(stderr, "jaylink_jtag_io: %s\n", jaylink_strerror_name(rc));
    exit(1);
  }
}
