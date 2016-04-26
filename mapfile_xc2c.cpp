/* 
 * Warp Coolrunner II Jedecfile to Bitfiles suitable for programming 
 * and vice- versa
 *
 * Needs access to the Xilinx supplied .map files for transformation
 *
 * Copyright Uwe Bonnes 2009 bon@elektron.ikp.physik.tu-darmstadt.de
 *
*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "mapfile_xc2c.h"

#ifndef MAPDIR
#if defined (__linux__) || defined(__FreeBSD__) || defined(__MACH__)
#define MAPDIR "/opt/Xilinx/12.4/ISE_DS/ISE/xbr/data"
#elif defined(__WIN32__)
#define MAPDIR "c:"
#endif
#endif

#define MAP_ZERO  -1
#define MAP_ONE   -2
#define MAP_SPARE -3
#define MAP_SEC   -10 /* sec_0 = -10, ... , sec_6 = -4 */
#define MAP_DONE  -12 /* done_0 = -12, done_1 = -11 */
#define MAP_USER  -44 /* user_0 = -44, ..., user_31 = -13 */

MapFile_XC2C::MapFile_XC2C()
{
  map = 0;
}

MapFile_XC2C::~MapFile_XC2C()
{
  if (map)
    free(map);
}

int MapFile_XC2C::readmap(FILE *fp)
{
  unsigned int i=0, j=0, k=0;
  int num = 0;
  char buffer[8];
  int x;
  int empty = 2;
  while ((x = fgetc(fp)) != EOF)
    {
      switch (x)
	{
	case 0x09:
	case '\n':
	  /* Lines with all TABs mark transfer bits and those bits should be
	  set 0 other empty places should be set 1 */
	  if     (empty == 2) num = MAP_ZERO; /* Empty line (for now) */
	  else if(empty == 1) num = MAP_ONE;  /* Empty place */
	  else empty = 1; /* If not empty reset the value for next token */
	  if(k > 0) /* String value */
	    {
	      if     (strncmp(buffer, "spare", k) == 0) num = MAP_SPARE;
	      else if(strncmp(buffer, "sec_", k) == 0) num += MAP_SEC;
	      else if(strncmp(buffer, "done_", k) == 0) num += MAP_DONE;
	      else if(strncmp(buffer, "user_", k) == 0) num += MAP_USER;
	      k = 0;
	    }
	  /* Store value */
	  map[i*block_num+j] = num;
	  num = 0;
	  j++;
	  if(x == '\n')
	    {
	      j = 0;
	      i++;
	      empty = 2;
	    }
	  break;
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
	  num = 10 * num + (x-'0');
	  empty = 0;
	  break;
	case '\r':
	  break;
	case '_':
	case 'a' ... 'z':
	  if(k < sizeof(buffer)) buffer[k++] = x;
	  empty = 0;
	break;
	}
    }
  return 0;
}

int MapFile_XC2C::loadmapfile(const char *mapdir, const char *device)
{
  FILE *fp;
  const char * mapfile;
  
  if (strncasecmp(device, "XC2C32", 6) == 0)
    {
      block_size = 260;
      block_num  = 48;
      if (strncasecmp(device, "XC2C32A", 7) == 0)
	mapfile = "xc2c32a";
      else
	mapfile = "xc2c32";
    }
  else if (strncasecmp(device, "XC2C64", 6) == 0)
    {
      block_size = 274;
      block_num  = 96;
      if (strncasecmp(device, "XC2C64A", 7) == 0)
	mapfile = "xc2c64a";
      else
	mapfile = "xc2c64";
    }
  else if (strncasecmp(device, "XC2C128", 7) == 0)
    {
      block_size = 752;
      block_num  = 80;
      mapfile = "xc2c128";
    }
  else if (strncasecmp(device, "XC2C256", 7) == 0)
    {
      block_size = 1364;
      block_num  = 96;
      mapfile = "xc2c256";
    }
  else if (strncasecmp(device, "XC2C384", 7) == 0)
    {
      block_size = 1868;
      block_num  = 120;
      mapfile = "xc2c384";
    }
  else if (strncasecmp(device, "XC2C512", 7) == 0)
    {
      block_size = 1980;
      block_num  = 160;
      mapfile = "xc2c512";
    }
  
  /* There are two extra rows for security/done and usercode bits*/
  block_num += 2;

  if (!mapdir)
    if(!(mapdir = getenv("XC_MAPDIR")))  
      mapdir = MAPDIR;
  
  mapfilename = (char *) malloc(strlen(mapdir)+strlen(mapfile)+6);
  if (mapfilename)
    {
      strcpy(mapfilename, mapdir);
      strcat(mapfilename, "/");
      strcat(mapfilename, mapfile);
      strcat(mapfilename, ".map");
    }
  fp = fopen(mapfilename, "rb");
  free(mapfilename);

  if (fp == NULL)
    {
      fprintf(stderr,"Mapfile %s/%s.map not found: %s\n", mapdir, mapfile,
	      strerror(errno));
       return 1;
    }

  if(map)
    free (map);
  map = (int *) malloc(block_size * (block_num) * sizeof(unsigned int));
  if (map == NULL)
    return 2;
  memset(map, 0, block_size * (block_num) * sizeof(unsigned int));
  readmap(fp);
  fclose(fp);
  return 0;
}

void MapFile_XC2C::jedecfile2bitfile(uint32_t usercode, JedecFile *fuses, BitFile  *bits)
{
  int i, j;
  bits->setLength(block_size*block_num);
  for (i=0; i<block_num; i++)
    {
      for (j=0; j<block_size; j++)
	{
	  int bitnum = (i+1)*block_size -j -1;
	  int fuse_idx = map[j*block_num +i];
	  int fuse = 1;
	  switch(fuse_idx)
	    {
	      case MAP_ZERO:
	        fuse = 0;
		break;
	      case MAP_ONE:
	      case MAP_SPARE:
	      case MAP_SEC ... MAP_SEC+6:
	      case MAP_DONE ... MAP_DONE+1:
	        fuse = 1;
		break;
	      case MAP_USER ... MAP_USER+31:
		fuse = usercode >> (fuse_idx-MAP_USER) & 1;
		break;
	      default:
	        /* xc2c32a.map from 10.1 contain 0 .. 12278 versus 0..12277 */
	        if (fuse_idx < (int)fuses->getLength())
		  fuse = fuses->get_fuse(fuse_idx);
	    }
	  bits->set_bit(bitnum, fuse);
	}
    }
}

void MapFile_XC2C::bitfile2jedecfile(BitFile  *bits, JedecFile *fuses)
{
  int i, j;
  fuses->setLength(block_size*block_num);
  int maxnum =0;
  for (i=0; i<block_num; i++)
    {
      for (j=0; j<block_size; j++)
	{
	  int bitnum = (i+1)*block_size -j -1;
	  int bit = 1;
	  int fuse_idx= map[j*block_num +i];
	  bit = bits->get_bit(bitnum);
	  if (fuse_idx>= 0)
	    {
	      if (fuse_idx > maxnum)
		maxnum = fuse_idx;
	      fuses->set_fuse(fuse_idx, bit);
	    }
	}
    }
  fuses->setLength(maxnum+1);
}
