/* JTAG low-level I/O to J-Link

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
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef IOJLINK_H
#define IOJLINK_H

#if defined (__WIN32__)
#include <windows.h>
#include <libusb/usb.h>
#else
#include <usb.h>
#endif

extern "C" {
#include <libjaylink.h>
}

#include "iobase.h"
#include "cabledb.h"

class IOJLink : public IOBase
{
 protected:
  int bptr, calls_rd, calls_wr;

 public:
  IOJLink();
  int Init(struct cable_t *cable, char const *serial, unsigned int freq);
  ~IOJLink();

  void txrx_block(const unsigned char *tdi, unsigned char *tdo, int length, bool last);
  void tx_tms(unsigned char *pat, int length, int force);

 private:
  struct jaylink_context *context;
  struct jaylink_device_handle *device;
};

#endif // IOJLink_H
