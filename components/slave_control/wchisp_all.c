static size_t isp_cmd_read_conf(uint16_t cfgmask, size_t len, uint8_t *cfg)
{
	u8 buf[60];
	u8 req[2];
	u16 mask;
	size_t got;

	req[0] = (cfgmask >> 0) & 0xff;
	req[1] = (cfgmask >> 8) & 0xff;

	isp_send_cmd(dev, CMD_READ_CONFIG, sizeof(req), req);
	got = isp_recv_cmd(dev, CMD_READ_CONFIG, sizeof(buf), buf);
	if (got < 2)
		printf("read conf fail: not received enough bytes\n");
	mask = buf[0] | (buf[1] << 8);
	if (cfgmask != mask)
		printf("read conf fail: received conf does not match\n");
	len = MIN(got - 2, len);
	memcpy(cfg, &buf[2], len);

	return len;
}

static void isp_cmd_write_conf(uint16_t cfgmask, size_t len, uint8_t *cfg)
{
	u8 req[60];
	u8 rsp[2];

	req[0] = (cfgmask >>  0) & 0xff;
	req[1] = (cfgmask >>  8) & 0xff;

	len = MIN(sizeof(req) - 2, len);
	memcpy(&req[2], cfg, len);

	isp_send_cmd(dev, CMD_WRITE_CONFIG, len + 2, req);
	isp_recv_cmd(dev, CMD_WRITE_CONFIG, sizeof(rsp), rsp);
}