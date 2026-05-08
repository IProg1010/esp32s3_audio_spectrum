/* SPDX-License-Identifier: GPL-2.0-only */
/* SPDX-FileCopyrightText: 2022 Jules Maselbas */
#include <unistd.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <errno.h>
#include <limits.h>
//#include <libusb.h>
#include "../dev_iface.h"

#include "arg.h"

//#define __noreturn __attribute__((noreturn))
//#define __unused __attribute__((unused))
#define __printf __attribute__((format(printf,1,2)))

//typedef uint8_t uint8_t;
//typedef uint16_t uint16_t;
//typedef uint32_t uint32_t;

#define BIT(x)		(1UL << (x))
#define MIN(a, b)	((a) < (b) ? (a) : (b))
#define MAX(a, b)	((a) < (b) ? (b) : (a))
#define LEN(a)		(sizeof(a) / sizeof(*a))
#define ALIGN(x, a)	(((x) + ((a) - 1)) & ~((a) - 1))
#define MASK(u,l)	(BIT((u) + 1) - BIT(l))

#define MAX_PACKET_SIZE 64
#define SECTOR_SIZE  1024

/*
 *  All readable and writable registers.
 *  - `RDPR`: Read Protection
 *  - `USER`: User Config Byte (normally in Register Map datasheet)
 *  - `WPR`:  Write Protection Mask, 1=unprotected, 0=protected
 *
 *  | BYTE0  | BYTE1  | BYTE2  | BYTE3  |
 *  |--------|--------|--------|--------|
 *  | RDPR   | nRDPR  | USER   | nUSER  |
 *  | DATA0  | nDATA0 | DATA1  | nDATA1 |
 *  | WPR0   | WPR1   | WPR2   | WPR3   |
 */
#define CFG_MASK_USERCONF 0x07 /* 12 bytes, see table above */
#define CFG_MASK_BTVER 0x08 /* Bootloader version, in the format of `[0x00, major, minor, 0x00]` */
#define CFG_MASK_UID 0x10 /* Device Unique ID */
#define CFG_MASK_ALL 0x1f /* All mask bits of CFGs */

#define WCH_ISP_REQ_HEADER_1     0x57
#define WCH_ISP_REQ_HEADER_2     0xAB
#define WCH_ISP_RESP_HEADER_1    0x55
#define WCH_ISP_RESP_HEADER_2    0xAA

#define CMD_IDENTIFY	0xa1
#define CMD_ISP_END	0xa2
#define CMD_ISP_KEY	0xa3
#define CMD_ERASE	0xa4
#define CMD_PROGRAM	0xa5
#define CMD_VERIFY	0xa6
#define CMD_READ_CONFIG	0xa7
#define CMD_WRITE_CONFIG	0xa8
#define CMD_DATA_ERASE	0xa9
#define CMD_DATA_PROGRAM	0xaa
#define CMD_DATA_READ	0xab
#define CMD_WRITE_OTP	0xc3
#define CMD_READ_OTP	0xc4
#define CMD_SET_BAUD	0xc5

#define ISP_VID 0x4348
#define ISP_PID 0x55e0
#define ISP_EP_OUT (2 | LIBUSB_ENDPOINT_OUT)
#define ISP_EP_IN (2 | LIBUSB_ENDPOINT_IN)
#define VERSION "1.0.0"

struct db;
struct isp_dev {
	uint8_t id;
	uint8_t type;
	uint8_t uid[8];
	char uid_str[3 * 8];
	uint16_t btver;
	uint8_t xor_key[8];
	//libusb_device_handle *usb_dev;
	const device_conf* dev_func;
	unsigned int kernel;
	/* info filled from db */
	const struct db *db; /* device family */
	const char *name;
	uint32_t flash_size;
	uint32_t eeprom_size;
	uint32_t flash_sector_size;
};

#include "devices.h"


static struct isp_dev glob_dev;
static struct isp_dev *dev_list;
static size_t dev_count;

//static libusb_context *usb;
static uint8_t isp_key[30]; /* all zero key */

static int dbg_enable;
static int do_progress;
static int do_reset;
static int do_verify = 1;
static const char *do_match;

/*__noreturn */static void die(const char *errstr, ...) __printf;
/*__noreturn*/ static void version(void);
/*__noreturn*/ static void usage(int help);
static void *xcalloc(size_t nmemb, size_t size);

static void dbg_isp_cmd(const char *dir, uint8_t cmd, uint16_t len, const uint8_t *data);
void isp_init(struct isp_dev *dev);
void isp_key_init(struct isp_dev *dev);
void isp_fini(struct isp_dev *dev);
size_t isp_send_cmd(struct isp_dev *dev, uint8_t cmd, uint16_t len, const uint8_t *data);
size_t isp_recv_cmd(struct isp_dev *dev, uint8_t cmd, uint16_t len, uint8_t *data);

const device_conf* glob_dev_conf;

void set_function(const device_conf* dev_conf)
{
	glob_dev_conf = dev_conf;
	glob_dev.dev_func = dev_conf;
}

uint8_t isp_calculate_crc(const uint8_t *data, size_t len) 
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) 
    {
        crc += data[i]; // Переполнение произойдет автоматически (wrapping)
    }
    return crc;
}

static void die(const char *errstr, ...)
{
	va_list ap;

	va_start(ap, errstr);
	vfprintf(stderr, errstr, ap);
	va_end(ap);

	//exit(1);
}

static int ctz_uint32_t(uint32_t x)
{
	return __builtin_ctz(x);
}

void dbg_isp_cmd(const char *dir, uint8_t cmd, uint16_t len, const uint8_t *data)
{
	uint16_t i;

	if (!dbg_enable)
		return;

	fprintf(stderr, "isp %s cmd %.2x len %.4x : ", dir, cmd, len);
	for (i = 0; i < len; i++)
		fprintf(stderr, "%.2x", data[i]);
	fprintf(stderr, "\n");
}

void *xcalloc(size_t nmemb, size_t size)
{
	void *p = calloc(nmemb, size);
	if (p == NULL)
		die("calloc: %s\n", strerror(errno));
	return p;
}

int streq(const char *s1, const char *s2)
{
	return strcmp(s1, s2) == 0;
}

static uint32_t reg_read_le(const uint8_t *src, size_t len)
{
	uint32_t reg = 0;
	while (len-- > 0) {
		reg <<= 8;
		reg |= src[len];
	}
	return reg;
}

static void reg_write_le(uint8_t *dst, size_t len, uint32_t val)
{
	while (len-- > 0) {
		*dst++ = val;
		val >>= 8;
	}
}

size_t isp_send_cmd(struct isp_dev *dev, uint8_t cmd, uint16_t len, const uint8_t *data)
{
	uint8_t buf[64];
	int ret, got = 0;

	if ((size_t)(len + 3) > sizeof(buf))
		die("isp_send_cmd: invalid argument, length %d\n", len);


    buf[0] = WCH_ISP_REQ_HEADER_1;
    buf[1] = WCH_ISP_REQ_HEADER_2; 

	buf[2] = cmd;
	/* length is sent in little endian... but it doesn't really matter
	 * as the usb maxpacket size is 64, thus len should never be greater
	 * than 61 (64 minus the 3 bytes header). */
	buf[3] = (len >> 0) & 0xff;
	buf[4] = (len >> 8) & 0xff;
	if (len != 0)
		memcpy(&buf[5], data, len);

	buf[len+5] = isp_calculate_crc(&buf[2], len+3);	
	/*printf("send:");
	for(int i = 0; i < len+6; i++)
	{
		printf("%02X", buf[i]);
	}
	printf("\n");*/

	ret = dev->dev_func->write(buf, len + 6);//libusb_bulk_transfer(dev->usb_dev, ISP_EP_OUT, buf, len + 3, &got, 10000);
	//if (ret)
		//die("isp_send_cmd: %s\n", libusb_strerror(ret));
	return got;
}

size_t isp_recv_cmd(struct isp_dev *dev, uint8_t cmd, uint16_t len, uint8_t *data)
{
	uint8_t buf[69];
	int ret, got = 0;
	uint16_t hdrlen;

	if ((size_t)(len + 4) > sizeof(buf))
		die("isp_recv_cmd: invalid argument, length %d\n", len);

	len = (len + 7); 	
	//printf("len =%d\n", len);
	ret = dev->dev_func->read(buf, len, &got);//libusb_bulk_transfer(dev->usb_dev, ISP_EP_IN, buf, len + 4, &got, 10000);
	int read_len = got;
	//printf("read:");
	/*for(int i = 0; i < read_len; i++)
	{
		printf("%02X", buf[i]);
	}*/
	while(read_len < len)
	{
		ret = dev->dev_func->read(&buf[read_len], len-read_len, &got);
		read_len += got;
		if(got == 0)
		{
			break;
		}
		//printf("read in cycle\n");
		//printf("read_len =%d\n", read_len);
	}
	/*printf("read_len =%d\n", read_len);
	printf("read:");
	for(int i = 0; i < read_len; i++)
	{
		printf("%02X", buf[i]);
	}
	printf("\n");
	if (ret)
		//die("isp_recv_cmd: %s\n", libusb_strerror(ret));
		printf("isp_recv_cmd: %s\n", "sd");


	printf("ok\n");*/
	if(buf[0] != WCH_ISP_RESP_HEADER_1 && buf[1] != WCH_ISP_RESP_HEADER_2)
	{
		die("isp_recv_cmd: header not wrong");
	}	
	if (read_len < 4)
		die("isp_recv_cmd: not enough data recv\n");
	if (buf[2] != cmd)
		die("isp_recv_cmd: got wrong command %#.x (exp %#.x)\n", buf[0], cmd);
	//if (buf[1])
	//	die("isp_recv_cmd: cmd error %#.x\n", buf[1]);

	//printf("ok\n");
	read_len -= 7;
	hdrlen = (buf[3] << 8) | (buf[4]);
	if (hdrlen != read_len)
		die("isp_recv_cmd: length mismatch, got %#.x (hdr %#.x)\n", got, hdrlen);
	
	//printf("len = %d, got = %d\n", len, got);
	len = MIN(len, read_len);
	//printf("len = %d\n", len);

	//printf("ok\n");
	if (data != NULL)
		memcpy(data, buf + 6, len);

	//printf("ok\n");
	uint8_t crc = isp_calculate_crc(&buf[2], read_len+4);
	if(crc != buf[read_len+6])
	{
		die("isp_recv_cmd: crc not correct, crc = %#.x (crc_read = %#.x)\n", crc, buf[read_len+6]);
	}	

	//dbg_isp_cmd("recv", cmd, len, data);

	return read_len;
}

void isp_cmd_identify(struct isp_dev *dev, uint8_t *dev_id, uint8_t *dev_type)
{
	const uint8_t buf[] = "\0\0MCU ISP & WCH.CN";
	uint8_t ids[2];

	/* do not send the terminating nul byte, hence sizeof(buf) - 1 */
	isp_send_cmd(dev, CMD_IDENTIFY, sizeof(buf) - 1, buf);
	isp_recv_cmd(dev, CMD_IDENTIFY, sizeof(ids), ids);

	dev->id = ids[0];
	printf("dev_id = %x\n", dev->id);
	dev->type = ids[1];
	printf("dev_type = %x\n", dev->type);
}

void isp_cmd_identify_glob()
{
	isp_cmd_identify(&glob_dev, &(glob_dev.id), &(glob_dev.type));
}


void
isp_cmd_isp_key(struct isp_dev *dev, size_t len, uint8_t *key, uint8_t *sum)
{
	uint8_t rsp[2];

	isp_send_cmd(dev, CMD_ISP_KEY, len, key);
	isp_recv_cmd(dev, CMD_ISP_KEY, sizeof(rsp), rsp);
	if (sum)
		*sum = rsp[0];
}

void
isp_cmd_isp_end(struct isp_dev *dev, uint8_t reason)
{
	uint8_t buf[2];

	isp_send_cmd(dev, CMD_ISP_END, sizeof(reason), &reason);
	isp_recv_cmd(dev, CMD_ISP_END, sizeof(buf), buf);
}

void
isp_cmd_erase(struct isp_dev *dev, uint32_t sectors)
{
	uint8_t sec[4];
	uint8_t rsp[2];

	sec[0] = (sectors >>  0) & 0xff;
	sec[1] = (sectors >>  8) & 0xff;
	sec[2] = (sectors >> 16) & 0xff;
	sec[3] = (sectors >> 24) & 0xff;

	isp_send_cmd(dev, CMD_ERASE, sizeof(sec), sec);
	isp_recv_cmd(dev, CMD_ERASE, 2, rsp);

	if (rsp[0] != 0 || rsp[1] != 0)
		die("Fail to erase, error: %.2x %.2x\n", rsp[0], rsp[1]);
}

size_t
isp_cmd_program(struct isp_dev *dev, uint32_t addr, size_t len, const uint8_t *data, const uint8_t key[8])
{
	uint8_t unk[61];
	uint8_t rsp[2];
	size_t i;

	unk[0] = (addr >>  0) & 0xff;
	unk[1] = (addr >>  8) & 0xff;
	unk[2] = (addr >> 16) & 0xff;
	unk[3] = (addr >> 24) & 0xff;
	unk[4] = 0; /* carefully choosen random number */

	len = MIN(sizeof(unk) - 5, len);
	for (i = 0; i < len; i++)
		unk[5 + i] = data[i] ^ key[i % 8];

	isp_send_cmd(dev, CMD_PROGRAM, len + 5, unk);
	isp_recv_cmd(dev, CMD_PROGRAM, sizeof(rsp), rsp);

	if (rsp[0] != 0 || rsp[1] != 0)
		die("Fail to program chunk @ %#x error: %.2x %.2x\n", addr, rsp[0], rsp[1]);

	return len;
}

size_t
isp_cmd_verify(struct isp_dev *dev, uint32_t addr, size_t len, const uint8_t *data, const uint8_t key[8])
{
	uint8_t unk[61];
	uint8_t rsp[2];
	size_t i;

	unk[0] = (addr >>  0) & 0xff;
	unk[1] = (addr >>  8) & 0xff;
	unk[2] = (addr >> 16) & 0xff;
	unk[3] = (addr >> 24) & 0xff;
	unk[4] = 0; /* carefully choosen random number */

	len = MIN(sizeof(unk) - 5, len);
	for (i = 0; i < len; i++)
		unk[5 + i] = data[i] ^ key[i % 8];

	isp_send_cmd(dev, CMD_VERIFY, len + 5, unk);
	isp_recv_cmd(dev, CMD_VERIFY, sizeof(rsp), rsp);

	if (rsp[0] != 0 || rsp[1] != 0)
		die("Fail to verify chunk @ %#x error: %.2x %.2x\n", addr, rsp[0], rsp[1]);

	return len;
}

size_t
isp_cmd_read_conf(struct isp_dev *dev, uint16_t cfgmask, size_t len, uint8_t *cfg)
{
	uint8_t buf[60];
	uint8_t req[2];
	uint16_t mask;
	size_t got;

	req[0] = (cfgmask >> 0) & 0xff;
	req[1] = (cfgmask >> 8) & 0xff;

	isp_send_cmd(dev, CMD_READ_CONFIG, sizeof(req), req);
	got = isp_recv_cmd(dev, CMD_READ_CONFIG, sizeof(buf), buf);
	if (got < 2)
		die("read conf fail: not received enough bytes\n");
	mask = buf[0] | (buf[1] << 8);
	if (cfgmask != mask)
		die("read conf fail: received conf does not match\n");
	len = MIN(got - 2, len);
	memcpy(cfg, &buf[2], len);

	return len;
}

void
isp_cmd_write_conf(struct isp_dev *dev, uint16_t cfgmask, size_t len, uint8_t *cfg)
{
	uint8_t req[60];
	uint8_t rsp[2];

	req[0] = (cfgmask >>  0) & 0xff;
	req[1] = (cfgmask >>  8) & 0xff;

	len = MIN(sizeof(req) - 2, len);
	memcpy(&req[2], cfg, len);

	isp_send_cmd(dev, CMD_WRITE_CONFIG, len + 2, req);
	isp_recv_cmd(dev, CMD_WRITE_CONFIG, sizeof(rsp), rsp);
}

uint16_t read_btver(struct isp_dev *dev)
{
	uint8_t buf[4];
	size_t len;

	/* format: [0x00, major, minor, 0x00] */
	len = isp_cmd_read_conf(dev, CFG_MASK_BTVER, sizeof(buf), buf);
	if (len != 4)
		return 0xffff;

	return (buf[1] << 8) | buf[2];
}

static size_t db_flash_size(struct isp_dev *dev)
{
	return dev->flash_size;
}

static size_t db_flash_sector_size(struct isp_dev *dev)
{
	return dev->flash_sector_size;
}

static void get_cur_flash_size(struct isp_dev *dev)
{
	uint8_t cfg[16];
	uint8_t mod;
	size_t len;

	len = isp_cmd_read_conf(dev, CFG_MASK_USERCONF, sizeof(cfg), cfg);
	if (len < 12)
		die("config: invalid length\n");

	mod = (cfg[2] >> 6) & 0x3;
	if (dev->db && dev->db->flash_cfg)
		dev->flash_size = (*dev->db->flash_cfg)[mod].code;
	else
		dev->flash_size = SZ_UNKNOWN;
}

static void
isp_init_from_db(struct isp_dev *dev)
{
	const struct db *db = NULL;
	const struct db_dev *db_dev = NULL;
	size_t i;

	dev->flash_sector_size = SECTOR_SIZE;
	dev->name = "unknown";
	dev->flash_size = SZ_UNKNOWN;
	dev->eeprom_size = 0;

	for (i = 0; i < LEN(devices); i++) {
		if (devices[i].type == dev->type) {
			dev->db = db = &devices[i];
			break;
		}
	}
	if (db) {
		for (db_dev = db->devs; db_dev->id != 0; db_dev++) {
			if (db_dev->id == dev->id)
				break;
		}
		if (db_dev->id == 0)
			db_dev = NULL;
	}

	if (db) {
		dev->flash_sector_size = db->flash_sector_size;
	}
	if (db_dev) {
		dev->name = db_dev->name;
		dev->flash_size = db_dev->flash_size;
		if (dev->flash_size == SZ_FROM_CONF)
			get_cur_flash_size(dev);
		dev->eeprom_size = db_dev->eeprom_size;
	}
}

void
isp_init(struct isp_dev *dev)
{
	size_t i;

	/* get the device type and id */
	isp_cmd_identify(dev, &dev->id, &dev->type);
	/* match the detected device */
	isp_init_from_db(dev);
	/* get the bootloader version */
	dev->btver = read_btver(dev);

	/* get the device uid */
	isp_cmd_read_conf(dev, CFG_MASK_UID, sizeof(dev->uid), dev->uid);

	for (i = 0; i < sizeof(dev->uid); i++) {
		snprintf(dev->uid_str + 3 * i, sizeof(dev->uid_str) - 3 * i,
			 "%.2x-", dev->uid[i]);
	}
}

void
isp_key_init(struct isp_dev *dev)
{
	size_t i;
	uint8_t sum;
	uint8_t rsp;

	/* initialize xor_key */
	for (sum = 0, i = 0; i < sizeof(dev->uid); i++)
		sum += dev->uid[i];
	memset(dev->xor_key, sum, sizeof(dev->xor_key));
	dev->xor_key[7] = dev->xor_key[0] + dev->id;

	/* send the isp key */
	isp_cmd_isp_key(dev, sizeof(isp_key), isp_key, &rsp);

	/* The bootloader send back a checksum of xor_key. This response is
	 * to make sure that we are in sync. */
	for (sum = 0, i = 0; i < sizeof(dev->xor_key); i++)
		sum += dev->xor_key[i];

	/* Workaround for CH56x family which reply with 0
	 * (only tested on CH569) */
	if (dev->type == 0x10)
		sum = 0;

	if (rsp != sum)
		die("failed set isp key, wrong reply, got %x (exp %x)\n", rsp, sum);
}

void
progress_bar(const char *act, size_t current, size_t total)
{
	const char *f = "####################################################";
	const char *e = "                                                    ";
	size_t l = strlen(f);
	size_t n = total > 0 ? (l * current) / total : l;

	if (!do_progress)
		return;

	printf("\r[%s%s] %s %zu/%zu\n", &f[l-n] , &e[n], act, current, total);
	if (current == total)
		puts("");
	fflush(stdout);
}

static void
isp_flash(struct isp_dev *dev, size_t size, const uint8_t *data)
{
	size_t sector_size = db_flash_sector_size(dev);
	uint32_t nr_sectors = ALIGN(size, sector_size) / sector_size;
	size_t off = 0;
	size_t rem = size;
	size_t len;

	isp_cmd_erase(dev, nr_sectors);

	while (off < size) {
		//progress_bar("write", off, size);

		len = isp_cmd_program(dev, off, rem, data + off, dev->xor_key);
		off += len;
		rem -= len;
	}
	isp_cmd_program(dev, off, 0, NULL, dev->xor_key);
	//progress_bar("write", size, size);
}

void
isp_verify(struct isp_dev *dev, size_t size, const uint8_t *data)
{
	size_t off = 0;
	size_t rem = size;
	size_t len;

	while (off < size) {
		//progress_bar("verify", off, size);

		len = isp_cmd_verify(dev, off, rem, data + off, dev->xor_key);
		off += len;
		rem -= len;
	}
	//progress_bar("verify", size, size);
}

void
isp_fini(struct isp_dev *dev)
{
	if (do_reset)
		isp_cmd_isp_end(dev, 1);
}

void
file_read_all(const char *name, size_t *size_p, void **bin_p)
{
	FILE *f;
	size_t len, size;
	void *bin;
	int ret;

	f = fopen(name, "rb");
	if (f == NULL)
		die("%s: %s\n", name, strerror(errno));

	ret = fseek(f, 0, SEEK_END);
	if (ret == -1)
		die("fseek: %s\n", strerror(errno));

	len = ftell(f);
	ret = fseek(f, 0, SEEK_SET);
	if (ret == -1)
		die("fseek: %s\n", strerror(errno));

	/* binary image needs to be aligned to a 64 bytes boundary */
	size = ALIGN(len, 64);
	bin = calloc(1, size);
	if (bin == NULL)
		die("calloc: %s\n", strerror(errno));

	if (len > 0) {
		ret = fread(bin, len, 1, f);
		if (ret != 1)
			die("fread: %s\n", strerror(errno));
	}

	fclose(f);

	*size_p = size;
	*bin_p = bin;
}

void cmd_write_flash_bin(const uint8_t* bin, size_t size)
{
	const char *name;

	if (size > db_flash_size(&glob_dev))
		printf("bin_data too big, flash size is\n");

	isp_flash(&glob_dev, size, bin);
	isp_key_init(&glob_dev);
	if (do_verify)
		isp_verify(&glob_dev, size, bin);

	//free(bin);
}

 
void cmd_write_flash(struct isp_dev *dev, int argc, char **argv)
{

	//free(bin);
}

 void cmd_verify_flash_bin(const uint8_t* bin, size_t size)
{
	if (size > db_flash_size(&glob_dev))
		die("bin flash too big, flash size is %zd\n", db_flash_size(&glob_dev));

	isp_verify(&glob_dev, size, bin);

	//free(bin);
}

 
void cmd_verify_flash(struct isp_dev *dev, int argc, char **argv)
{

	//free(bin);
}

/**
 * fmtb formating function to print a binary number to a char buf
 * b the output buffer
 * n the buffer size
 * p the "precision", how many 'bit' will be printed
 * v the value to be printed
 */
static char *
fmtb(char *b, size_t n, int p, uint32_t v)
{
	char *s = b + n;

	if ((size_t)p > 32) p = 32;
	if ((size_t)p > n)  p = n;

	*--s = '\0';
	for (; b < s && v ; v >>= 1, p--)
		*--s = (v & 1) ? '1' : '0';
	while (b < s && p-- > 0)
		*--s = '0';
	return s;
}

void
cmd_config_show(struct isp_dev *dev, __unused int argc, __unused char **argv)
{
	const struct userconf_reg *cfg_reg = NULL;
	uint8_t cfg[16];
	size_t len, i;

	len = isp_cmd_read_conf(dev, CFG_MASK_USERCONF, sizeof(cfg), cfg);

	printf("raw:");
	for (i = 0; i < len; i++)
		printf(" %.2x", cfg[i]);
	puts("");

	if (dev->db)
		cfg_reg = dev->db->userconf_regs;
	for (; cfg_reg && cfg_reg->size; cfg_reg++) {
		const struct userconf_bit *bit = cfg_reg->bits;
		char bitstr[40];
		const char *name = cfg_reg->name;
		size_t addr = cfg_reg->addr;
		size_t size = cfg_reg->size;
		uint32_t reg = 0;

		if ((addr + size) > len) {
			printf("%.2zx: %s: fail decode: no enough bytes\n", addr, name);
			continue;
		}
		if (size > sizeof(reg)) {
			fprintf(stderr, "bad config: %s size %zd clamped to %zd\n", name, size, sizeof(reg));
			size = sizeof(reg);
		}

		reg = reg_read_le(cfg + addr, size);
		printf("%.2zx: %s: 0x%.*x\n", addr, name, (int)(2 * size), reg);
		for (; bit && bit->mask; bit++) {
			uint32_t lsb = ctz_uint32_t(bit->mask);
			uint32_t cnt = ctz_uint32_t(~(bit->mask >> lsb));
			uint32_t hsb = lsb + cnt - 1;
			uint32_t val = (reg & bit->mask) >> lsb;

			if (hsb == lsb)
				printf("      [%d]", lsb);
			else
				printf("    [%d:%d]", hsb, lsb);
			if (bit->name)
				printf(" %s", bit->name);
			printf(": ");
			if (cnt > 1)
				printf("0x%x ", val);
			else
				printf("%x ", val);
			if (bit->tostr) {
				printf("(%s)", bit->tostr(dev, bitstr, sizeof(bitstr), val));
			} else {
				printf("(0b%s)", fmtb(bitstr, sizeof(bitstr), cnt, val));
			}
			puts("");
		}
	}
}

static uint32_t
parse_bit(const char *str, uint8_t reg_size, char **end)
{
	char *e;
	unsigned long val;
	uint32_t lsb, msb;

	if (*str != '[')
		die("syntax error: %s\n", "expected '['");

	val = strtoul(str + 1, &e, 0);
	if ((str + 1) == e)
		die("syntax error: %s\n", "missing bit index");
	if (val == ULONG_MAX)
		die("syntax error: %s\n", "value overflow");
	if (val >= (8 * reg_size))
		die("syntax error: %s\n", "bit index greater than register width");
	msb = lsb = val;
	if (*e == ']')
		goto out;

	if (*e != ':')
		die("syntax error: %s\n", "expected ':'");

	str = e;
	val = strtoul(str + 1, &e, 0);
	if ((str + 1) == e)
		die("syntax error: %s\n", "missing bit index");
	if (val == ULONG_MAX)
		die("syntax error: %s\n", "value overflow");
	if (val >= (8 * reg_size))
		die("syntax error: %s\n", "bit index greater than register width");
	lsb = val;
	if (*e != ']')
		die("syntax error: %s\n", "expected ']'");

out:
	if (msb < lsb)
		die("syntax error: %s\n", "invalid bit mask");
	if (end)
		*end = e;
	return MASK(msb, lsb);
}

static const struct userconf_reg *
find_userconf_reg(const struct userconf_reg *reg, const char *name, size_t len)
{
	if (!len)
		len = strlen(name) + 1;
	for (; reg && reg->size; reg++) {
		if (!strncmp(name, reg->name, len))
			return reg;
	}
	return NULL;
}

static const struct userconf_bit *
find_userconf_bit(const struct userconf_bit *bit, const char *name, size_t len)
{
	if (!len)
		len = strlen(name) + 1;
	for (; bit && bit->mask; bit++) {
		if (!strncmp(name, bit->name, len))
			return bit;
	}
	return NULL;
}

static uint32_t
cfg_parse_field(const struct userconf_reg *reg, const char *arg)
{
	char *end;
	size_t len;
	uint32_t msk = -1u;

	/* FIELD is optional */
	if (*arg != ':' && *arg != '[')
		return -1u;

	if (*arg == ':')
		arg++;

	if (*arg == '[') {
		msk = parse_bit(arg, reg->size, &end);
		if (*end != ']')
			die("syntax error: %s\n", "exected '='");
	} else {
		const struct userconf_bit *bit;
		end = strchr(arg, '=');
		len = end - arg;
		if (!end)
			die("syntax error: %s\n", "exected '='");
		if (len == 0)
			die("syntax error: %s\n", "exected a bit-field name");
		bit = find_userconf_bit(reg->bits, arg, len);
		if (!bit)
			die("error: no bit-field named '%.*s' found\n", (int)len, arg);
		msk = bit->mask;
	}

	return msk;
}

static uint32_t
cfg_parse_value(const struct userconf_reg *reg, uint32_t msk, const char *arg)
{
	char *end;
	uint32_t lsb = ctz_uint32_t(msk);
	uint32_t val = strtoul(arg, &end, 0);

	if (*arg == '\0')
		die("syntax error: %s\n", "missing value");
	if (*end != '\0')
		die("syntax error: %s\n", "value has garbage at the end");
	if (val == ULONG_MAX || val >= BIT(8 * reg->size))
		die("syntax error: %s\n", "value overflow");

	val <<= lsb;
	return val & msk;
}

static void
cmd_config_set(struct isp_dev *dev, int argc, char **argv)
{
	const struct userconf_reg *cfg_reg = NULL;
	uint8_t oldcfg[16];
	uint8_t newcfg[16];
	size_t len, i;

	if (argc <= 1)
		die("config-set: missing argument\n");

	len = isp_cmd_read_conf(dev, CFG_MASK_USERCONF, sizeof(oldcfg), oldcfg);
	memcpy(newcfg, oldcfg, len);

	if (dev && dev->db)
		cfg_reg = dev->db->userconf_regs;

	for (i = 1; i < argc; i++) {
		const char *arg = argv[i];
		char *end;
		size_t len;
		uint32_t reg, val, msk;

		/* parse REG_NAME */
		end = strchr(arg, ':');
		end = end ? end : strchr(arg, '[');
		end = end ? end : strchr(arg, '=');
		if (!end)
			die("syntax error: '%s': %s\n", arg, "expected ':' or '='");
		len = end - arg;

		cfg_reg = find_userconf_reg(cfg_reg, arg, len);
		if (!cfg_reg)
			die("error: no config register named '%.*s' found\n", (int)len, arg);

		/* parse BIT_FIELD */
		msk = cfg_parse_field(cfg_reg, end);
		end = strchr(end, '=');
		if (*end != '=')
			die("syntax error: %s\n", "exected '='");
		/* parse VALUE */
		val = cfg_parse_value(cfg_reg, msk, end + 1);

		reg = reg_read_le(newcfg + cfg_reg->addr, cfg_reg->size);
		reg &= ~msk;
		reg |= val;
		reg_write_le(newcfg + cfg_reg->addr, cfg_reg->size, reg);
	}

	if (memcmp(oldcfg, newcfg, len) == 0) {
		printf("nothing changed, skipping write\n");
		return;
	}
	printf("writing new config\n");
	isp_cmd_write_conf(dev, CFG_MASK_USERCONF, len, newcfg);
}

static void
cmd_remove_wp(struct isp_dev *dev, __unused int argc, __unused char **argv)
{
	uint8_t cfg[16];
	size_t len;

	len = isp_cmd_read_conf(dev, CFG_MASK_USERCONF, sizeof(cfg), cfg);
	if (cfg[0] == 0xa5) {
		printf("write protection already off\n");
	} else {
		cfg[0] = 0xa5;
		isp_cmd_write_conf(dev, CFG_MASK_USERCONF, len, cfg);
		printf("write protection disabled\n");
	}
}

static void
cmd_remove_wp_bin(struct isp_dev *dev)
{
	uint8_t cfg[16];
	size_t len;

	len = isp_cmd_read_conf(dev, CFG_MASK_USERCONF, sizeof(cfg), cfg);
	if (cfg[0] == 0xa5) {
		printf("write protection already off\n");
	} else {
		cfg[0] = 0xa5;
		isp_cmd_write_conf(dev, CFG_MASK_USERCONF, len, cfg);
		printf("write protection disabled\n");
	}
}

static void
cmd_erase_all(struct isp_dev *dev, __unused int argc, __unused char **argv)
{
	size_t size = db_flash_size(dev);
	size_t sector_size = db_flash_sector_size(dev);
	uint32_t nr_sectors = size / sector_size;
	isp_cmd_erase(dev, nr_sectors);
}

static void cmd_erase_all_bin(struct isp_dev *dev)
{
	size_t size = db_flash_size(dev);
	size_t sector_size = db_flash_sector_size(dev);
	uint32_t nr_sectors = size / sector_size;
	isp_cmd_erase(dev, nr_sectors);
}

static void
cmd_reset(struct isp_dev *dev, __unused int argc, __unused char **argv)
{
	isp_cmd_isp_end(dev, 1);
}

static void
print_dev(struct isp_dev *dev)
{
	printf("BTVER v%d.%d UID %s [0x%.2x%.2x] %s",
	       dev->btver >> 8, dev->btver & 0xff,
	       dev->uid_str, dev->type, dev->id,
	       dev->name);
	if (dev->flash_size != SZ_UNKNOWN)
		printf(" (flash %dK)", dev->flash_size / SZ_1K);
	else
		printf(" (flash size unknown)");
}

static void
list_devices(void)
{
	size_t i;

	for (i = 0; i < dev_count; i++) {
		printf("%zd: ", i);
		print_dev(&dev_list[i]);
		printf("\n");
	}
}

static void
cmd_list_devices(__unused struct isp_dev *dev, __unused int argc, __unused char **argv)
{
	list_devices();
}

static struct isp_dev *
dev_by_uid(const char *uid)
{
	size_t i;

	for (i = 0; i < dev_count; i++)
		if (streq(uid, dev_list[i].uid_str))
			return &dev_list[i];

	return NULL;
}

static struct isp_dev *
dev_by_index(const char *s)
{
	char *e = NULL;
	unsigned long i;

	i = strtoul(s, &e, 0);
	if (i == ULONG_MAX || (e != NULL && *e != '\0'))
		return NULL;

	if (i < dev_count)
		return &dev_list[i];

	return NULL;
}

char *argv0;

static void
usage(int help)
{
	printf("usage: %s [-VDnpr] [-d <uid>] COMMAND [ARG ...]\n", argv0);
	printf("       %s [-VDnpr] [-d <uid>] [flash|write|verify|reset] FILE\n", argv0);
	printf("       %s [-VDnpr] [-d <uid>] [erase|config|remove-wp]\n", argv0);
	printf("       %s [-VDnpr] [-d <uid>] config-set <REG_NAME>[:<FIELD_SEL>]=<VALUE>\n", argv0);
	printf("       %s [-VDnpr] [list]\n", argv0);
	if (!help)
		exit(1);

	printf("options:\n");
	printf("  -d <uid> Select the usb device that matches by uid first, else by index\n");
	printf("  -n       No verify after writing to flash, done by default\n");
	printf("  -p       Print a progress-bar during command operation\n");
	printf("  -r       Reset after command completed\n");
	printf("  -D       Print raw isp command (for debug)\n");
	printf("  -V       Print version and exit\n");

	exit(0);
}

static void
version(void)
{
	printf("%s %s\n", argv0, VERSION);
	exit(0);
}

static const struct {
	const char *name;
	void (*func)(struct isp_dev *dev, int argc, char **argv);
} cmds[] = {
	{ "list", cmd_list_devices },
	{ "flash", cmd_write_flash },
	{ "write", cmd_write_flash },
	{ "verify", cmd_verify_flash },
	{ "erase", cmd_erase_all },
	{ "reset", cmd_reset },
	{ "config", cmd_config_show },
	{ "config-set", cmd_config_set },
	{ "remove-wp", cmd_remove_wp },
};

static int
is_valid_cmd(const char *name)
{
	size_t i;
	for (i = 0; i < LEN(cmds); i++) {
		if (streq(name, cmds[i].name))
			return 1;
	}
	return 0;
}

int
main_wchisp(int argc, char **argv)
{
	struct isp_dev *dev;
	size_t i;

	ARGBEGIN {
	case 'p':
		do_progress = 1;
		break;
	case 'r':
		do_reset = 1;
		break;
	case 'v':
		do_verify = 1;
		break;
	case 'n':
		do_verify = 0;
		break;
	case 'd':
		do_match = EARGF(usage(0));
		break;
	case 'D':
		dbg_enable = 1;
		break;
	case 'V':
		version();
	case 'h':
		usage(1);
	default:
		usage(0);
	} ARGLONG {
		fprintf(stderr, "unknown option '%s'\n", ARGLC());
		usage(0);
	} ARGEND;

	if (argc > 0 && !is_valid_cmd(argv[0]))
		die("%s: invalid command\n", argv[0]);

	//usb_init();

	if (dev_count == 0)
		die("no device detected\n");

	for (i = 0; i < dev_count; i++)
		isp_init(&dev_list[i]);

	if (argc < 1 || streq(argv[0], "list")) {
		list_devices();
		goto out;
	}

	/* by default select the first device */
	dev = &dev_list[0];
	if (do_match) {
		dev = dev_by_uid(do_match);
		if (!dev)
			dev = dev_by_index(do_match);
		if (!dev)
			die("no device match for '%s'\n", do_match);
	}

	print_dev(dev);
	printf("\n");
	isp_key_init(dev);

	for (i = 0; i < LEN(cmds); i++) {
		if (streq(argv[0], cmds[i].name)) {
			cmds[i].func(dev, argc, argv);
			printf("%s done\n", cmds[i].name);
			break;
		}
	}
	if (i == LEN(cmds))
		die("%s: invalid command\n", argv[0]);

	isp_fini(dev);
out:
	//usb_fini();

	return 0;
}


int program_wchisp_algo()
{
	printf("\n");
	isp_cmd_identify_glob();
	isp_init(&glob_dev);
	isp_key_init(&glob_dev);
	//isp_cmd_erase(&glob_dev, 500);
	//cmd_erase_all_bin(&glob_dev);
	cmd_remove_wp_bin(&glob_dev);
	//isp_fini(&glob_dev);
	print_dev(&glob_dev);
	printf("\n");

	return 0;
}
