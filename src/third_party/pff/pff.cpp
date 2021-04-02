/*----------------------------------------------------------------------------/
/  Petit FatFs - FAT file system module  R0.03a
/-----------------------------------------------------------------------------/
/
/ Copyright (C) 2019, ChaN, all right reserved.
/
/ Petit FatFs module is an open source software. Redistribution and use of
/ Petit FatFs in source and binary forms, with or without modification, are
/ permitted provided that the following condition is met:
/
/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.
/
/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/-----------------------------------------------------------------------------/
/ Jun 15,'09  R0.01a  First release.
/
/ Dec 14,'09  R0.02   Added multiple code page support.
/                     Added write funciton.
/                     Changed stream read mode interface.
/ Dec 07,'10  R0.02a  Added some configuration options.
/                     Fixed fails to open objects with DBCS character.

/ Jun 10,'14  R0.03   Separated out configuration options to pffconf.h.
/                     Added _USE_LCC option.
/                     Added _FS_FAT16 option.
/
/ Jan 30,'19  R0.03a  Supported stdint.h for C99 and later.
/                     Removed _WORD_ACCESS option.
/                     Changed prefix of configuration options, _ to PF_.
/                     Added some code pages.
/                     Removed some code pages actually not valid.
/----------------------------------------------------------------------------*/

#include "third_party/pff/pff.h" /* Petit FatFs configurations and declarations */
#include "driver/gpio.h"

/*--------------------------------------------------------------------------

   Module Private Definitions

---------------------------------------------------------------------------*/

#if PF_DEFINED != 8088 /* Revision ID */
#error Wrong include file (pff.h).
#endif

#if PF_FS_FAT32
#if !PF_FS_FAT16 && !PF_FS_FAT12
#define _FS_32ONLY 1
#else
#define _FS_32ONLY 0
#endif
#else
#if !PF_FS_FAT16 && !PF_FS_FAT12
#error Wrong PF_FS_FATxx setting.
#endif
#define _FS_32ONLY 0
#endif

#define ABORT(err)    \
	{                 \
		fs->flag = 0; \
		return err;   \
	}

namespace ThirdParty
{
pff::pff() : FatFs(nullptr),									 //
			 spi(getGpioConfig(System::Pinout::Name::kGPIO_SD_B1_05, false, true)), //
			 spo(getGpioConfig(System::Pinout::Name::kGPIO_SD_B1_03, true, false)), //
			 clk(getGpioConfig(System::Pinout::Name::kGPIO_SD_B1_04, true, false)), //
			 cs(getGpioConfig(System::Pinout::Name::kGPIO_SD_B1_00, true, true)), //
			 csToSckDelay(2500), sckDelay(3000) {
	spo.Set(true);
	clk.Set(false);
	cs.Set(true);

	System::Systick::DelayMS(10);

	BYTE res;
	FATFS fs;			/* File system object */
	// DIR dir;			/* Directory object */
	// FILINFO fno;		/* File information */

	// Initialize physical drive
	res = disk_initialize();
	put_drc(res);

	csToSckDelay = 40;
	sckDelay = 60;

	// Mount the volume
	put_rc(pf_mount(&fs));

	printf("End of pff init\n");
}

/*--------------------------------------------------------------------------

   Private Functions

---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Load multi-byte word in the FAT structure                             */
/*-----------------------------------------------------------------------*/

WORD pff::ld_word(const BYTE *ptr) /*	 Load a 2-byte little-endian word */
{
	WORD rv;

	rv = ptr[1];
	rv = rv << 8 | ptr[0];
	return rv;
}

DWORD pff::ld_dword(const BYTE *ptr) /* Load a 4-byte little-endian word */
{
	DWORD rv;

	rv = ptr[3];
	rv = rv << 8 | ptr[2];
	rv = rv << 8 | ptr[1];
	rv = rv << 8 | ptr[0];
	return rv;
}

/*-----------------------------------------------------------------------*/
/* String functions                                                      */
/*-----------------------------------------------------------------------*/

/* Fill memory block */
void pff::mem_set(void *dst, int val, int cnt)
{
	char *d = (char *)dst;
	while (cnt--)
		*d++ = (char)val;
}

/* Compare memory block */
int pff::mem_cmp(const void *dst, const void *src, int cnt)
{
	const char *d = (const char *)dst, *s = (const char *)src;
	int r = 0;
	while (cnt-- && (r = *d++ - *s++) == 0)
		;
	return r;
}

/*-----------------------------------------------------------------------*/
/* FAT access - Read value of a FAT entry                                */
/*-----------------------------------------------------------------------*/

CLUST pff::get_fat(			  /* 1:IO error, Else:Cluster status */
				   CLUST clst /* Cluster# to get the link information */
)
{
	BYTE buf[4];
	FATFS *fs = FatFs;
#if PF_FS_FAT12
	UINT wc, bc, ofs;
#endif

	if (clst < 2 || clst >= fs->n_fatent)
		return 1; /* Range check */

	switch (fs->fs_type)
	{
#if PF_FS_FAT12
	case FS_FAT12:
	{
		bc = (UINT)clst;
		bc += bc / 2;
		ofs = bc % 512;
		bc /= 512;
		if (ofs != 511)
		{
			if (disk_readp(buf, fs->fatbase + bc, ofs, 2))
				break;
		}
		else
		{
			if (disk_readp(buf, fs->fatbase + bc, 511, 1))
				break;
			if (disk_readp(buf + 1, fs->fatbase + bc + 1, 0, 1))
				break;
		}
		wc = ld_word(buf);
		return (clst & 1) ? (wc >> 4) : (wc & 0xFFF);
	}
#endif
#if PF_FS_FAT16
	case FS_FAT16:
		if (disk_readp(buf, fs->fatbase + clst / 256, ((UINT)clst % 256) * 2, 2))
			break;
		return ld_word(buf);
#endif
#if PF_FS_FAT32
	case FS_FAT32:
		if (disk_readp(buf, fs->fatbase + clst / 128, ((UINT)clst % 128) * 4, 4))
			break;
		return ld_dword(buf) & 0x0FFFFFFF;
#endif
	}

	return 1; /* An error occured at the disk I/O layer */
}

/*-----------------------------------------------------------------------*/
/* Get sector# from cluster# / Get cluster field from directory entry    */
/*-----------------------------------------------------------------------*/

DWORD pff::clust2sect(			 /* !=0: Sector number, 0: Failed - invalid cluster# */
					  CLUST clst /* Cluster# to be converted */
)
{
	FATFS *fs = FatFs;

	clst -= 2;
	if (clst >= (fs->n_fatent - 2))
		return 0; /* Invalid cluster# */
	return (DWORD)clst * fs->csize + fs->database;
}

CLUST pff::get_clust(
	BYTE *dir /* Pointer to directory entry */
)
{
	FATFS *fs = FatFs;
	CLUST clst = 0;

	if (_FS_32ONLY || (PF_FS_FAT32 && fs->fs_type == FS_FAT32))
	{
		clst = ld_word(dir + DIR_FstClusHI);
		clst <<= 16;
	}
	clst |= ld_word(dir + DIR_FstClusLO);

	return clst;
}

/*-----------------------------------------------------------------------*/
/* Directory handling - Rewind directory index                           */
/*-----------------------------------------------------------------------*/

pff::FRESULT pff::dir_rewind(
	DIR *dj /* Pointer to directory object */
)
{
	CLUST clst;
	FATFS *fs = FatFs;

	dj->index = 0;
	clst = dj->sclust;
	if (clst == 1 || clst >= fs->n_fatent)
	{ /* Check start cluster range */
		return FR_DISK_ERR;
	}
	if (PF_FS_FAT32 && !clst && (_FS_32ONLY || fs->fs_type == FS_FAT32))
	{ /* Replace cluster# 0 with root cluster# if in FAT32 */
		clst = (CLUST)fs->dirbase;
	}
	dj->clust = clst;												  /* Current cluster */
	dj->sect = (_FS_32ONLY || clst) ? clust2sect(clst) : fs->dirbase; /* Current sector */

	return FR_OK; /* Seek succeeded */
}

/*-----------------------------------------------------------------------*/
/* Directory handling - Move directory index next                        */
/*-----------------------------------------------------------------------*/

pff::FRESULT pff::dir_next(		   /* FR_OK:Succeeded, FR_NO_FILE:End of table */
						   DIR *dj /* Pointer to directory object */
)
{
	CLUST clst;
	WORD i;
	FATFS *fs = FatFs;

	i = dj->index + 1;
	if (!i || !dj->sect)
		return FR_NO_FILE; /* Report EOT when index has reached 65535 */

	if (!(i % 16))
	{				/* Sector changed? */
		dj->sect++; /* Next sector */

		if (dj->clust == 0)
		{ /* table */
			if (i >= fs->n_rootdir)
				return FR_NO_FILE; /* Report EOT when end of table */
		}
		else
		{ /* Dynamic table */
			if (((i / 16) & (fs->csize - 1)) == 0)
			{							   /* Cluster changed? */
				clst = get_fat(dj->clust); /* Get next cluster */
				if (clst <= 1)
					return FR_DISK_ERR;
				if (clst >= fs->n_fatent)
					return FR_NO_FILE; /* Report EOT when it reached end of dynamic table */
				dj->clust = clst;	  /* Initialize data for new cluster */
				dj->sect = clust2sect(clst);
			}
		}
	}

	dj->index = i;

	return FR_OK;
}

/*-----------------------------------------------------------------------*/
/* Directory handling - Find an object in the directory                  */
/*-----------------------------------------------------------------------*/

pff::FRESULT pff::dir_find(
	DIR *dj,  /* Pointer to the directory object linked to the file name */
	BYTE *dir /* 32-byte working buffer */
)
{
	FRESULT res;
	BYTE c;

	res = dir_rewind(dj); /* Rewind directory object */
	if (res != FR_OK)
		return res;

	do
	{
		res = disk_readp(dir, dj->sect, (dj->index % 16) * 32, 32) /* Read an entry */
				  ? FR_DISK_ERR
				  : FR_OK;
		if (res != FR_OK)
			break;
		c = dir[DIR_Name]; /* First character */
		if (c == 0)
		{
			res = FR_NO_FILE;
			break;
		} /* Reached to end of table */
		if (!(dir[DIR_Attr] & AM_VOL) && !mem_cmp(dir, dj->fn, 11))
			break;			/* Is it a valid entry? */
		res = dir_next(dj); /* Next entry */
	} while (res == FR_OK);

	return res;
}

/*-----------------------------------------------------------------------*/
/* Read an object from the directory                                     */
/*-----------------------------------------------------------------------*/
#if PF_USE_DIR
pff::FRESULT pff::dir_read(
	DIR *dj,  /* Pointer to the directory object to store read object name */
	BYTE *dir /* 32-byte working buffer */
)
{
	FRESULT res;
	BYTE a, c;

	res = FR_NO_FILE;
	while (dj->sect)
	{
		res = disk_readp(dir, dj->sect, (dj->index % 16) * 32, 32) /* Read an entry */
				  ? FR_DISK_ERR
				  : FR_OK;
		if (res != FR_OK)
			break;
		c = dir[DIR_Name];
		if (c == 0)
		{
			res = FR_NO_FILE;
			break;
		} /* Reached to end of table */
		a = dir[DIR_Attr] & AM_MASK;
		if (c != 0xE5 && c != '.' && !(a & AM_VOL))
			break;			/* Is it a valid entry? */
		res = dir_next(dj); /* Next entry */
		if (res != FR_OK)
			break;
	}

	if (res != FR_OK)
		dj->sect = 0;

	return res;
}
#endif

/*-----------------------------------------------------------------------*/
/* Pick a segment and create the object name in directory form           */
/*-----------------------------------------------------------------------*/

pff::FRESULT pff::create_name(
	DIR *dj,		  /* Pointer to the directory object */
	const char **path /* Pointer to pointer to the segment in the path string */
)
{
	BYTE c, d, ni, si, i, *sfn;
	const char *p;
#if PF_USE_LCC && defined(_EXCVT)
	const BYTE cvt[] = _EXCVT;
#endif

	/* Create file name in directory form */
	sfn = dj->fn;
	mem_set(sfn, ' ', 11);
	si = i = 0;
	ni = 8;
	p = *path;
	for (;;)
	{
		c = p[si++];
		if (c <= ' ' || c == '/')
			break; /* Break on end of segment */
		if (c == '.' || i >= ni)
		{
			if (ni != 8 || c != '.')
				break;
			i = 8;
			ni = 11;
			continue;
		}
#if PF_USE_LCC && defined(_EXCVT)
		if (c >= 0x80)
			c = cvt[c - 0x80]; /* To upper extended char (SBCS) */
#endif
		if (IsDBCS1(c) && i < ni - 1)
		{				 /* DBC 1st byte? */
			d = p[si++]; /* Get 2nd byte */
			sfn[i++] = c;
			sfn[i++] = d;
		}
		else
		{ /* Single byte code */
			if (PF_USE_LCC && IsLower(c))
				c -= 0x20; /* toupper */
			sfn[i++] = c;
		}
	}
	*path = &p[si]; /* Rerurn pointer to the next segment */

	sfn[11] = (c <= ' ') ? 1 : 0; /* Set last segment flag if end of path */

	return FR_OK;
}

/*-----------------------------------------------------------------------*/
/* Get file information from directory entry                             */
/*-----------------------------------------------------------------------*/
#if PF_USE_DIR
void pff::get_fileinfo(				/* No return code */
					   DIR *dj,		/* Pointer to the directory object */
					   BYTE *dir,   /* 32-byte working buffer */
					   FILINFO *fno /* Pointer to store the file information */
)
{
	BYTE i, c;
	char *p;

	p = fno->fname;
	if (dj->sect)
	{
		for (i = 0; i < 8; i++)
		{ /* Copy file name body */
			c = dir[i];
			if (c == ' ')
				break;
			if (c == 0x05)
				c = 0xE5;
			*p++ = c;
		}
		if (dir[8] != ' ')
		{ /* Copy file name extension */
			*p++ = '.';
			for (i = 8; i < 11; i++)
			{
				c = dir[i];
				if (c == ' ')
					break;
				*p++ = c;
			}
		}
		fno->fattrib = dir[DIR_Attr];			   /* Attribute */
		fno->fsize = ld_dword(dir + DIR_FileSize); /* Size */
		fno->fdate = ld_word(dir + DIR_WrtDate);   /* Date */
		fno->ftime = ld_word(dir + DIR_WrtTime);   /* Time */
	}
	*p = 0;
}
#endif /* PF_USE_DIR */

/*-----------------------------------------------------------------------*/
/* Follow a file path                                                    */
/*-----------------------------------------------------------------------*/

pff::FRESULT pff::follow_path(				   /* FR_OK(0): successful, !=0: error code */
							  DIR *dj,		   /* Directory object to return last directory and found object */
							  BYTE *dir,	   /* 32-byte working buffer */
							  const char *path /* Full-path string to find a file or directory */
)
{
	FRESULT res;

	while (*path == ' ')
		path++; /* Strip leading spaces */
	if (*path == '/')
		path++;		/* Strip heading separator if exist */
	dj->sclust = 0; /* Set start directory (always root dir) */

	if ((BYTE)*path < ' ')
	{ /* Null path means the root directory */
		res = dir_rewind(dj);
		dir[0] = 0;
	}
	else
	{ /* Follow path */
		for (;;)
		{
			res = create_name(dj, &path); /* Get a segment */
			if (res != FR_OK)
				break;
			res = dir_find(dj, dir); /* Find it */
			if (res != FR_OK)
				break; /* Could not find the object */
			if (dj->fn[11])
				break; /* Last segment match. Function completed. */
			if (!(dir[DIR_Attr] & AM_DIR))
			{ /* Cannot follow path because it is a file */
				res = FR_NO_FILE;
				break;
			}
			dj->sclust = get_clust(dir); /* Follow next */
		}
	}

	return res;
}

/*-----------------------------------------------------------------------*/
/* Check a sector if it is an FAT boot record                            */
/*-----------------------------------------------------------------------*/

BYTE pff::check_fs(			  /* 0:The FAT boot record, 1:Valid boot record but not an FAT, 2:Not a boot record, 3:Error */
				   BYTE *buf, /* Working buffer */
				   DWORD sect /* Sector# (lba) to check if it is an FAT boot record or not */
)
{
	if (disk_readp(buf, sect, 510, 2))
	{ /* Read the boot record */
		return 3;
	}
	if (ld_word(buf) != 0xAA55)
	{ /* Check record signature */
		return 2;
	}

	if (!_FS_32ONLY && !disk_readp(buf, sect, BS_FilSysType, 2) && ld_word(buf) == 0x4146)
	{ /* Check FAT12/16 */
		return 0;
	}
	if (PF_FS_FAT32 && !disk_readp(buf, sect, BS_FilSysType32, 2) && ld_word(buf) == 0x4146)
	{ /* Check FAT32 */
		return 0;
	}
	return 1;
}

/*--------------------------------------------------------------------------

   Public Functions

--------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Mount/Unmount a Locical Drive                                         */
/*-----------------------------------------------------------------------*/

pff::FRESULT pff::pf_mount(
	FATFS *fs /* Pointer to new file system object */
)
{
	BYTE fmt, buf[36];
	DWORD bsect, fsize, tsect, mclst;

	FatFs = 0;

	if (disk_initialize() & STA_NOINIT)
	{ /* Check if the drive is ready or not */
		return FR_NOT_READY;
	}

	/* Search FAT partition on the drive */
	bsect = 0;
	fmt = check_fs(buf, bsect); /* Check sector 0 as an SFD format */
	if (fmt == 1)
	{ /* Not an FAT boot record, it may be FDISK format */
		/* Check a partition listed in top of the partition table */
		if (disk_readp(buf, bsect, MBR_Table, 16))
		{ /* 1st partition entry */
			fmt = 3;
		}
		else
		{
			if (buf[4])
			{								/* Is the partition existing? */
				bsect = ld_dword(&buf[8]);  /* Partition offset in LBA */
				fmt = check_fs(buf, bsect); /* Check the partition */
			}
		}
	}
	if (fmt == 3)
		return FR_DISK_ERR;
	if (fmt)
		return FR_NO_FILESYSTEM; /* No valid FAT patition is found */

	/* Initialize the file system object */
	if (disk_readp(buf, bsect, 13, sizeof(buf)))
		return FR_DISK_ERR;

	fsize = ld_word(buf + BPB_FATSz16 - 13); /* Number of sectors per FAT */
	if (!fsize)
		fsize = ld_dword(buf + BPB_FATSz32 - 13);

	fsize *= buf[BPB_NumFATs - 13];							  /* Number of sectors in FAT area */
	fs->fatbase = bsect + ld_word(buf + BPB_RsvdSecCnt - 13); /* FAT start sector (lba) */
	fs->csize = buf[BPB_SecPerClus - 13];					  /* Number of sectors per cluster */
	fs->n_rootdir = ld_word(buf + BPB_RootEntCnt - 13);		  /* Nmuber of root directory entries */
	tsect = ld_word(buf + BPB_TotSec16 - 13);				  /* Number of sectors on the file system */
	if (!tsect)
		tsect = ld_dword(buf + BPB_TotSec32 - 13);
	mclst = (tsect /* Last cluster# + 1 */
			 - ld_word(buf + BPB_RsvdSecCnt - 13) - fsize - fs->n_rootdir / 16) /
				fs->csize +
			2;
	fs->n_fatent = (CLUST)mclst;

	fmt = 0; /* Determine the FAT sub type */
	if (PF_FS_FAT12 && mclst < 0xFF7)
		fmt = FS_FAT12;
	if (PF_FS_FAT16 && mclst >= 0xFF8 && mclst < 0xFFF7)
		fmt = FS_FAT16;
	if (PF_FS_FAT32 && mclst >= 0xFFF7)
		fmt = FS_FAT32;
	if (!fmt)
		return FR_NO_FILESYSTEM;
	fs->fs_type = fmt;

	if (_FS_32ONLY || (PF_FS_FAT32 && fmt == FS_FAT32))
	{
		fs->dirbase = ld_dword(buf + (BPB_RootClus - 13)); /* Root directory start cluster */
	}
	else
	{
		fs->dirbase = fs->fatbase + fsize; /* Root directory start sector (lba) */
	}
	fs->database = fs->fatbase + fsize + fs->n_rootdir / 16; /* Data start sector (lba) */

	fs->flag = 0;
	FatFs = fs;

	return FR_OK;
}

/*-----------------------------------------------------------------------*/
/* Open or Create a File                                                 */
/*-----------------------------------------------------------------------*/

pff::FRESULT pff::pf_open(
	const char *path /* Pointer to the file name */
)
{
	FRESULT res;
	DIR dj;
	BYTE sp[12], dir[32];
	FATFS *fs = FatFs;

	if (!fs)
		return FR_NOT_ENABLED; /* Check file system */

	fs->flag = 0;
	dj.fn = sp;
	res = follow_path(&dj, dir, path); /* Follow the file path */
	if (res != FR_OK)
		return res; /* Follow failed */
	if (!dir[0] || (dir[DIR_Attr] & AM_DIR))
		return FR_NO_FILE; /* It is a directory */

	fs->org_clust = get_clust(dir);			  /* File start cluster */
	fs->fsize = ld_dword(dir + DIR_FileSize); /* File size */
	fs->fptr = 0;							  /* File pointer */
	fs->flag = FA_OPENED;

	return FR_OK;
}

/*-----------------------------------------------------------------------*/
/* Read File                                                             */
/*-----------------------------------------------------------------------*/
#if PF_USE_READ

pff::FRESULT pff::pf_read(
	void *buff, /* Pointer to the read buffer (NULL:Forward data to the stream)*/
	UINT btr,   /* Number of bytes to read */
	UINT *br	/* Pointer to number of bytes read */
)
{
	DRESULT dr;
	CLUST clst;
	DWORD sect, remain;
	UINT rcnt;
	BYTE cs, *rbuff = (BYTE *)buff;
	FATFS *fs = FatFs;

	*br = 0;
	if (!fs)
		return FR_NOT_ENABLED; /* Check file system */
	if (!(fs->flag & FA_OPENED))
		return FR_NOT_OPENED; /* Check if opened */

	remain = fs->fsize - fs->fptr;
	if (btr > remain)
		btr = (UINT)remain; /* Truncate btr by remaining bytes */

	while (btr)
	{ /* Repeat until all data transferred */
		if ((fs->fptr % 512) == 0)
		{												   /* On the sector boundary? */
			cs = (BYTE)(fs->fptr / 512 & (fs->csize - 1)); /* Sector offset in the cluster */
			if (!cs)
			{ /* On the cluster boundary? */
				if (fs->fptr == 0)
				{ /* On the top of the file? */
					clst = fs->org_clust;
				}
				else
				{
					clst = get_fat(fs->curr_clust);
				}
				if (clst <= 1)
					ABORT(FR_DISK_ERR);
				fs->curr_clust = clst; /* Update current cluster */
			}
			sect = clust2sect(fs->curr_clust); /* Get current sector */
			if (!sect)
				ABORT(FR_DISK_ERR);
			fs->dsect = sect + cs;
		}
		rcnt = 512 - (UINT)fs->fptr % 512; /* Get partial sector data from sector buffer */
		if (rcnt > btr)
			rcnt = btr;
		dr = disk_readp(rbuff, fs->dsect, (UINT)fs->fptr % 512, rcnt);
		if (dr)
			ABORT(FR_DISK_ERR);
		fs->fptr += rcnt; /* Advances file read pointer */
		btr -= rcnt;
		*br += rcnt; /* Update read counter */
		if (rbuff)
			rbuff += rcnt; /* Advances the data pointer if destination is memory */
	}

	return FR_OK;
}
#endif

/*-----------------------------------------------------------------------*/
/* Write File                                                            */
/*-----------------------------------------------------------------------*/
#if PF_USE_WRITE

pff::FRESULT pff::pf_write(
	const void *buff, /* Pointer to the data to be written */
	UINT btw,		  /* Number of bytes to write (0:Finalize the current write operation) */
	UINT *bw		  /* Pointer to number of bytes written */
)
{
	CLUST clst;
	DWORD sect, remain;
	const BYTE *p = (BYTE *)buff;
	BYTE cs;
	UINT wcnt;
	FATFS *fs = FatFs;

	*bw = 0;
	if (!fs)
		return FR_NOT_ENABLED; /* Check file system */
	if (!(fs->flag & FA_OPENED))
		return FR_NOT_OPENED; /* Check if opened */

	if (!btw)
	{ /* Finalize request */
		if ((fs->flag & FA__WIP) && disk_writep(0, 0))
			ABORT(FR_DISK_ERR);
		fs->flag &= ~FA__WIP;
		return FR_OK;
	}
	else
	{ /* Write data request */
		if (!(fs->flag & FA__WIP))
		{ /* Round-down fptr to the sector boundary */
			fs->fptr &= 0xFFFFFE00;
		}
	}
	remain = fs->fsize - fs->fptr;
	if (btw > remain)
		btw = (UINT)remain; /* Truncate btw by remaining bytes */

	while (btw)
	{ /* Repeat until all data transferred */
		if ((UINT)fs->fptr % 512 == 0)
		{												   /* On the sector boundary? */
			cs = (BYTE)(fs->fptr / 512 & (fs->csize - 1)); /* Sector offset in the cluster */
			if (!cs)
			{ /* On the cluster boundary? */
				if (fs->fptr == 0)
				{ /* On the top of the file? */
					clst = fs->org_clust;
				}
				else
				{
					clst = get_fat(fs->curr_clust);
				}
				if (clst <= 1)
					ABORT(FR_DISK_ERR);
				fs->curr_clust = clst; /* Update current cluster */
			}
			sect = clust2sect(fs->curr_clust); /* Get current sector */
			if (!sect)
				ABORT(FR_DISK_ERR);
			fs->dsect = sect + cs;
			if (disk_writep(0, fs->dsect))
				ABORT(FR_DISK_ERR); /* Initiate a sector write operation */
			fs->flag |= FA__WIP;
		}
		wcnt = 512 - (UINT)fs->fptr % 512; /* Number of bytes to write to the sector */
		if (wcnt > btw)
			wcnt = btw;
		if (disk_writep(p, wcnt))
			ABORT(FR_DISK_ERR); /* Send data to the sector */
		fs->fptr += wcnt;
		p += wcnt; /* Update pointers and counters */
		btw -= wcnt;
		*bw += wcnt;
		if ((UINT)fs->fptr % 512 == 0)
		{
			if (disk_writep(0, 0))
				ABORT(FR_DISK_ERR); /* Finalize the currtent secter write operation */
			fs->flag &= ~FA__WIP;
		}
	}

	return FR_OK;
}
#endif

/*-----------------------------------------------------------------------*/
/* Seek File R/W Pointer                                                 */
/*-----------------------------------------------------------------------*/
#if PF_USE_LSEEK

pff::FRESULT pff::pf_lseek(
	DWORD ofs /* File pointer from top of file */
)
{
	CLUST clst;
	DWORD bcs, sect, ifptr;
	FATFS *fs = FatFs;

	if (!fs)
		return FR_NOT_ENABLED; /* Check file system */
	if (!(fs->flag & FA_OPENED))
		return FR_NOT_OPENED; /* Check if opened */

	if (ofs > fs->fsize)
		ofs = fs->fsize; /* Clip offset with the file size */
	ifptr = fs->fptr;
	fs->fptr = 0;
	if (ofs > 0)
	{
		bcs = (DWORD)fs->csize * 512; /* Cluster size (byte) */
		if (ifptr > 0 &&
			(ofs - 1) / bcs >= (ifptr - 1) / bcs)
		{										 /* When seek to same or following cluster, */
			fs->fptr = (ifptr - 1) & ~(bcs - 1); /* start from the current cluster */
			ofs -= fs->fptr;
			clst = fs->curr_clust;
		}
		else
		{						  /* When seek to back cluster, */
			clst = fs->org_clust; /* start from the first cluster */
			fs->curr_clust = clst;
		}
		while (ofs > bcs)
		{						  /* Cluster following loop */
			clst = get_fat(clst); /* Follow cluster chain */
			if (clst <= 1 || clst >= fs->n_fatent)
				ABORT(FR_DISK_ERR);
			fs->curr_clust = clst;
			fs->fptr += bcs;
			ofs -= bcs;
		}
		fs->fptr += ofs;
		sect = clust2sect(clst); /* Current sector */
		if (!sect)
			ABORT(FR_DISK_ERR);
		fs->dsect = sect + (fs->fptr / 512 & (fs->csize - 1));
	}

	return FR_OK;
}
#endif

/*-----------------------------------------------------------------------*/
/* Create a Directroy Object                                             */
/*-----------------------------------------------------------------------*/
#if PF_USE_DIR

pff::FRESULT pff::pf_opendir(
	DIR *dj,		 /* Pointer to directory object to create */
	const char *path /* Pointer to the directory path */
)
{
	FRESULT res;
	BYTE sp[12], dir[32];
	FATFS *fs = FatFs;

	if (!fs)
	{ /* Check file system */
		res = FR_NOT_ENABLED;
	}
	else
	{
		dj->fn = sp;
		res = follow_path(dj, dir, path); /* Follow the path to the directory */
		if (res == FR_OK)
		{ /* Follow completed */
			if (dir[0])
			{ /* It is not the root dir */
				if (dir[DIR_Attr] & AM_DIR)
				{ /* The object is a directory */
					dj->sclust = get_clust(dir);
				}
				else
				{ /* The object is not a directory */
					res = FR_NO_FILE;
				}
			}
			if (res == FR_OK)
			{
				res = dir_rewind(dj); /* Rewind dir */
			}
		}
	}

	return res;
}

/*-----------------------------------------------------------------------*/
/* Read Directory Entry in Sequense                                      */
/*-----------------------------------------------------------------------*/

pff::FRESULT pff::pf_readdir(
	DIR *dj,	 /* Pointer to the open directory object */
	FILINFO *fno /* Pointer to file information to return */
)
{
	FRESULT res;
	BYTE sp[12], dir[32];
	FATFS *fs = FatFs;

	if (!fs)
	{ /* Check file system */
		res = FR_NOT_ENABLED;
	}
	else
	{
		dj->fn = sp;
		if (!fno)
		{
			res = dir_rewind(dj);
		}
		else
		{
			res = dir_read(dj, dir); /* Get current directory item */
			if (res == FR_NO_FILE)
				res = FR_OK;
			if (res == FR_OK)
			{								/* A valid entry is found */
				get_fileinfo(dj, dir, fno); /* Get the object information */
				res = dir_next(dj);			/* Increment read index for next */
				if (res == FR_NO_FILE)
					res = FR_OK;
			}
		}
	}

	return res;
}

// diskio.cpp content
// moved to here for easier formation of class

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

pff::DSTATUS pff::disk_initialize(void)
{
	BYTE n, cmd, ty, ocr[4];
	UINT tmr;

#if _USE_WRITE
	if (CardType && IS_CS_LOW) disk_writep(0, 0);	/* Finalize write process if it is in progress */
#endif
	// init_spi();		/* Initialize ports to control SDC/MMC */
	// yim: no need

	CS_HIGH();
	// yim: spo might need high here
	for (n = 10; n; n--) rcv_spi();	/* 80 Dummy clocks with CS=H */

	ty = 0;
	if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
		if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2 */
			for (n = 0; n < 4; n++) ocr[n] = rcv_spi();		/* Get trailing return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) {			/* The card can work at vdd range of 2.7-3.6V */
				for (tmr = 10000; tmr && send_cmd(ACMD41, 1UL << 30); tmr--) dly_100us();	/* Wait for leaving idle state (ACMD41 with HCS bit) */
				if (tmr && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
					for (n = 0; n < 4; n++) ocr[n] = rcv_spi();
					ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 (HC or SC) */
				}
			}
		} else {							/* SDv1 or MMCv3 */
			if (send_cmd(ACMD41, 0) <= 1) 	{
				ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
			} else {
				ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
			}
			for (tmr = 10000; tmr && send_cmd(cmd, 0); tmr--) dly_100us();	/* Wait for leaving idle state */
			if (!tmr || send_cmd(CMD16, 512) != 0) {		/* Set R/W block length to 512 */
				ty = 0;
			}
		}
	}
	CardType = ty;
	CS_HIGH();
	rcv_spi();

	return ty ? 0 : STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Read Partial Sector                                                   */
/*-----------------------------------------------------------------------*/

pff::DRESULT pff::disk_readp(
	BYTE *buff,   /* Pointer to the destination object */
	DWORD sector, /* Sector number (LBA) */
	UINT offset,  /* Offset in the sector */
	UINT count	/* Byte count (bit15:destination) */
)
{
	DRESULT res;
	BYTE rc;
	UINT bc;

	if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

	res = RES_ERROR;
	if (send_cmd(CMD17, sector) == 0) {		/* READ_SINGLE_BLOCK */

		bc = 40000;
		do {							/* Wait for data packet */
			rc = rcv_spi();
		} while (rc == 0xFF && --bc);

		if (rc == 0xFE) {				/* A data packet arrived */
			bc = 514 - offset - count;

			/* Skip leading bytes */
			if (offset) {
				do rcv_spi(); while (--offset);
			}

			/* Receive a part of the sector */
			if (buff) {	/* Store data to the memory */
				do {
					*buff++ = rcv_spi();
				} while (--count);
			} else {	/* Forward data to the outgoing stream (depends on the project) */
				// yim: originally forward is used to send data out for other purpose,
				// e.g. uart out, but in our case, it would mean there's no buffer assigned
				// and it's treated as an error

				assert(true);

				// do {
				// 	FORWARD(rcv_spi());
				// } while (--count);
			}

			/* Skip remaining bytes and CRC */
			do { rcv_spi(); } while (--bc);

			res = RES_OK;
		}
	}

	CS_HIGH();
	rcv_spi();

	return res;
}

/*-----------------------------------------------------------------------*/
/* Write Partial Sector                                                  */
/*-----------------------------------------------------------------------*/

pff::DRESULT pff::disk_writep(
	const BYTE *buff, /* Pointer to the data to be written, NULL:Initiate/Finalize write operation */
	DWORD sc		  /* Sector number (LBA) or Number of bytes to send */
)
{
	DRESULT res;
	UINT bc;
	static WORD wc;

	res = RES_ERROR;

	if (buff) {		/* Send data bytes */
		bc = (WORD)sc;
		while (bc && wc) {		/* Send data bytes to the card */
			xmit_spi(*buff++);
			wc--; bc--;
		}
		res = RES_OK;
	} else {
		if (sc) {	/* Initiate sector write process */
			if (!(CardType & CT_BLOCK)) sc *= 512;	/* Convert to byte address if needed */
			if (send_cmd(CMD24, sc) == 0) {			/* WRITE_SINGLE_BLOCK */
				xmit_spi(0xFF); xmit_spi(0xFE);		/* Data block header */
				wc = 512;							/* Set byte counter */
				res = RES_OK;
			}
		} else {	/* Finalize sector write process */
			bc = wc + 2;
			while (bc--) xmit_spi(0);	/* Fill left bytes and CRC with zeros */
			if ((rcv_spi() & 0x1F) == 0x05) {	/* Receive data resp and wait for end of write process in timeout of 500ms */
				for (bc = 5000; rcv_spi() != 0xFF && bc; bc--) dly_100us();	/* Wait ready */
				if (bc) res = RES_OK;
			}
			CS_HIGH();
			rcv_spi();
		}
	}

	return res;

}

/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for Petit FatFs (C)ChaN, 2014      */
/*-----------------------------------------------------------------------*/

const Driver::GPIO::Config pff::getGpioConfig(System::Pinout::Name pinName, bool isOutput, bool isPullup)
{
	Driver::GPIO::Config config;
	config.pin = pinName;
	config.pin_config.pull_config = System::Pinout::Config::PinConfig::PullConfig::k22kPullUp;
	config.pin_config.pull_keep_config = isPullup ? System::Pinout::Config::PinConfig::PullKeepConfig::kKeep : System::Pinout::Config::PinConfig::PullKeepConfig::kPull;
	config.gpio_dir = isOutput ? Driver::GPIO::Direction::kDigitalOutput : Driver::GPIO::Direction::kDigitalInput;
	return config;
}

BYTE pff::send_cmd (
	BYTE cmd,		/* 1st byte (Start + Index) */
	DWORD arg		/* Argument (32 bits) */
)
{
	BYTE n, res;

	if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	/* Select the card */
	CS_HIGH();
	rcv_spi();
	CS_LOW();
	rcv_spi();

	/* Send a command packet */
	xmit_spi(cmd);						/* Start + Command index */
	xmit_spi((BYTE)(arg >> 24));		/* Argument[31..24] */
	xmit_spi((BYTE)(arg >> 16));		/* Argument[23..16] */
	xmit_spi((BYTE)(arg >> 8));			/* Argument[15..8] */
	xmit_spi((BYTE)arg);				/* Argument[7..0] */
	n = 0x01;							/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;			/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;			/* Valid CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive a command response */
	n = 10;								/* Wait for a valid response in timeout of 10 attempts */
	do {
		res = rcv_spi();
	} while ((res & 0x80) && --n);

	return res;			/* Return with the response value */
}
}
#endif /* PF_USE_DIR */
