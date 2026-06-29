#include "../cmnds/cmd_public.h"
#include "../hal/hal_wifi.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"
#include "../new_common.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "agriht_license.h"
#include "drv_deye_solarman.h"

#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"
#ifndef WINDOWS
#include "lwip/netdb.h"
#include <lwip/dns.h>
#endif
#include "errno.h"

#define DEYE_MODBUS_MAX_LEN 16
#define DEYE_V5_MAX_LEN 64
#define DEYE_RX_MAX_LEN 256

static char g_deye_ip[16] = "192.168.1.104";
static int g_deye_port = 8899;
static uint32_t g_deye_logger_serial = 3928316555u;
static int g_deye_slave_id = 1;
static int g_deye_register = 40;
static int g_deye_fixed_percent = 50;
static int g_deye_update_seconds = 2;
static int g_deye_fixed_enabled = 0;
static int g_deye_auto_enabled = 1;
static int g_deye_seconds_until_send = 0;
static int g_deye_grid_invert = 0;
static int g_deye_voltage_channel = -1;
static int g_deye_current_channel = -1;
static int g_deye_grid_channel = -1;
static int g_deye_freq_channel = -1;
static int g_deye_pf_channel = -1;
static int g_deye_auto_detect_channels = 1;
static int g_deye_rated_w = 800;
static int g_deye_target_grid_w = 30;
static int g_deye_export_limit_w = -20;
static int g_deye_min_percent = 0;
static int g_deye_max_percent = 100;
static int g_deye_gain_percent = 100;
static int g_deye_max_step_percent = 20;
static int g_deye_w_per_percent = 100;
static int g_deye_deadband_w = 5;
static int g_deye_auto_percent = 50;
static int g_deye_last_grid_w = 0;
static int g_deye_last_voltage_v = 0;
static int g_deye_last_current_ma = 0;
static int g_deye_last_apparent_w = 0;
static float g_deye_last_freq_hz = 0.0f;
static float g_deye_last_pf = 0.0f;
static int g_deye_discover_active = 0;
static int g_deye_discover_host = 1;
static int g_deye_discover_found = 0;
static int g_deye_ip_confirmed = 0;
static byte g_deye_sequence = 1;
static int g_deye_last_rc = 0;
static int g_deye_last_rx_len = 0;
static int g_deye_last_percent = -1;
static unsigned int g_deye_send_count = 0;
static int g_deye_discover_seconds_until_next = 0;
static int g_deye_discover_try_saved_ip = 0;
static int g_deye_discover_saved_retry_seconds = 0;
static int g_deye_fail_count = 0;
static int g_deye_verify_seconds = 0;
static int g_deye_verify_expected = -1;
static int g_deye_verify_readback = -1;
static int g_deye_verify_grid_before_w = 0;
static int g_deye_filtered_grid_w = 0;
static int g_deye_filter_initialized = 0;
static int g_deye_last_control_direction = 0;
static int g_deye_stable_grid_w = 0;
static int g_deye_grid_sign_initialized = 0;
static int g_deye_pending_grid_sign = 0;
static int g_deye_pending_grid_sign_count = 0;

static int Deye_HTTP_GetJson(http_request_t *request);
static int Deye_HTTP_Discover(http_request_t *request);

static const char *Deye_RcText(int rc) {
	switch (rc) {
	case 0: return "OK";
	case -1: return "build Modbus failed";
	case -2: return "build Solarman V5 failed";
	case -3: return "socket failed";
	case -4: return "TCP connect failed";
	case -5: return "TCP send failed";
	case -6: return "TCP recv timeout/error";
	case -7: return "TCP probe connect failed";
	case -8: return "invalid Solarman/Modbus response";
	case -9: return "Modbus exception";
	case -10: return "license required";
	default: return "unknown";
	}
}

static const char *Deye_LinkText(void) {
	if (!AgriHTLicense_IsActive()) return "LOCKED";
	if (!g_deye_ip_confirmed) return g_deye_discover_active ? "SEARCHING" : "WAIT_IP";
	if (g_deye_last_rc == 0) return "OK";
	return "ERROR";
}

static int Deye_EstimatedOutputW(void) {
	int pct = g_deye_last_percent >= 0 ? g_deye_last_percent : g_deye_auto_percent;
	if (g_deye_rated_w <= 0) return 0;
	return (g_deye_rated_w * pct) / 100;
}

static uint32_t Deye_ParseU32(const char *s, uint32_t def) {
	uint32_t v = 0;
	int any = 0;
	while (*s >= '0' && *s <= '9') {
		v = (v * 10u) + (uint32_t)(*s - '0');
		s++;
		any = 1;
	}
	return any ? v : def;
}

static int Deye_ClampPercent(int percent) {
	if (percent < g_deye_min_percent) return g_deye_min_percent;
	if (percent > g_deye_max_percent) return g_deye_max_percent;
	return percent;
}

static int Deye_FeedbackStep(int absGridW) {
	if (absGridW < 30) return 1;
	if (absGridW < 100) return 3;
	if (absGridW < 300) return 7;
	return 15;
}

static const char *Deye_FlowText(void) {
	if (g_deye_last_grid_w > g_deye_target_grid_w) return "IMPORT";
	if (g_deye_last_grid_w < g_deye_export_limit_w) return "EXPORT";
	return "HOLD";
}

static int Deye_Clamp0To100(int percent) {
	if (percent < 0) return 0;
	if (percent > 100) return 100;
	return percent;
}

static void Deye_LogHex(const char *prefix, const byte *data, int len) {
	char line[96];
	char *p = line;
	int left = sizeof(line);
	int i;

	ADDLOG_INFO(LOG_FEATURE_DRV, "%s len=%i", prefix, len);
	for (i = 0; i < len; i++) {
		int written = snprintf(p, left, "%02X%s", data[i], ((i % 24) == 23 || i == len - 1) ? "" : " ");
		if (written < 0 || written >= left) {
			line[0] = 0;
			p = line;
			left = sizeof(line);
			continue;
		}
		p += written;
		left -= written;
		if ((i % 24) == 23 || i == len - 1) {
			ADDLOG_INFO(LOG_FEATURE_DRV, "%s", line);
			line[0] = 0;
			p = line;
			left = sizeof(line);
		}
	}
}

static void Deye_LogConfig(const char *prefix, int percent) {
	ADDLOG_INFO(LOG_FEATURE_DRV,
		"%s ip=%s port=%i loggerSerial=%u slave=%i reg=%i percent=%i seq=%u",
		prefix, g_deye_ip, g_deye_port, (unsigned int)g_deye_logger_serial,
		g_deye_slave_id, g_deye_register, percent, (unsigned int)g_deye_sequence);
}

static int Deye_TCPConnectTimeoutMs(int sock, struct sockaddr_in *addr, int timeoutMs) {
	fd_set wfds;
	struct timeval tv;
	int ret;
	int err = 0;
	socklen_t errLen = sizeof(err);

	if (timeoutMs < 50) timeoutMs = 50;
	lwip_fcntl(sock, F_SETFL, O_NONBLOCK);
	ret = connect(sock, (struct sockaddr *)addr, sizeof(*addr));
	if (ret == 0) {
		lwip_fcntl(sock, F_SETFL, 0);
		return 0;
	}
	if (errno != EINPROGRESS && errno != EWOULDBLOCK && errno != EAGAIN) {
		lwip_fcntl(sock, F_SETFL, 0);
		return -1;
	}
	FD_ZERO(&wfds);
	FD_SET(sock, &wfds);
	tv.tv_sec = timeoutMs / 1000;
	tv.tv_usec = (timeoutMs % 1000) * 1000;
	ret = select(sock + 1, NULL, &wfds, NULL, &tv);
	if (ret <= 0 || !FD_ISSET(sock, &wfds)) {
		lwip_fcntl(sock, F_SETFL, 0);
		return -2;
	}
	if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &errLen) < 0 || err != 0) {
		lwip_fcntl(sock, F_SETFL, 0);
		return -3;
	}
	lwip_fcntl(sock, F_SETFL, 0);
	return 0;
}

static uint16_t Deye_ModbusCRC(const byte *data, int len) {
	uint16_t crc = 0xFFFF;
	int i, bit;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (bit = 0; bit < 8; bit++) {
			if (crc & 1) {
				crc = (crc >> 1) ^ 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}

static int Deye_BuildWriteRegister40Modbus(byte *out, int outLen, int percent) {
	uint16_t crc;
	int reg = g_deye_register;

	if (outLen < 11) return -1;
	percent = Deye_ClampPercent(percent);

	out[0] = (byte)g_deye_slave_id;
	out[1] = 0x10;
	out[2] = (byte)((reg >> 8) & 0xFF);
	out[3] = (byte)(reg & 0xFF);
	out[4] = 0x00;
	out[5] = 0x01;
	out[6] = 0x02;
	out[7] = (byte)((percent >> 8) & 0xFF);
	out[8] = (byte)(percent & 0xFF);

	crc = Deye_ModbusCRC(out, 9);
	out[9] = (byte)(crc & 0xFF);
	out[10] = (byte)((crc >> 8) & 0xFF);
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman: Modbus RTU write reg=%i qty=1 value=%i crc=%04X",
		reg, percent, crc);
	return 11;
}

static int Deye_BuildReadRegisterModbus(byte *out, int outLen) {
	uint16_t crc;
	int reg = g_deye_register;

	if (outLen < 8) return -1;
	out[0] = (byte)g_deye_slave_id;
	out[1] = 0x03;
	out[2] = (byte)((reg >> 8) & 0xFF);
	out[3] = (byte)(reg & 0xFF);
	out[4] = 0x00;
	out[5] = 0x01;
	crc = Deye_ModbusCRC(out, 6);
	out[6] = (byte)(crc & 0xFF);
	out[7] = (byte)((crc >> 8) & 0xFF);
	return 8;
}

static int Deye_ParseModbusResponse(const byte *rx, int rxLen, int function, byte expectedSeq, int *value) {
	int i;
	int payloadLen;
	int checksum = 0;
	if (rxLen < 13 || rx[0] != 0xA5 || rx[rxLen - 1] != 0x15) goto invalid;
	payloadLen = rx[1] | ((int)rx[2] << 8);
	if (rxLen != payloadLen + 13 || rx[4] != 0x15 || rx[5] != expectedSeq ||
		rx[7] != (byte)(g_deye_logger_serial & 0xFF) ||
		rx[8] != (byte)((g_deye_logger_serial >> 8) & 0xFF) ||
		rx[9] != (byte)((g_deye_logger_serial >> 16) & 0xFF) ||
		rx[10] != (byte)((g_deye_logger_serial >> 24) & 0xFF) || rx[11] != 0x02) {
		goto invalid;
	}
	for (i = 1; i < rxLen - 2; i++) checksum = (checksum + rx[i]) & 0xFF;
	if (rx[rxLen - 2] != (byte)checksum) goto invalid;

	/* Solarman V5 responses have a 14-byte prefix after the 11-byte header. */
	for (i = 25; i + 5 <= rxLen - 2; i++) {
		int len;
		uint16_t gotCrc;
		uint16_t expectedCrc;
		if (rx[i] != (byte)g_deye_slave_id) continue;
		if (rx[i + 1] == (byte)(function | 0x80)) {
			ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman Modbus exception function=0x%02X code=0x%02X",
				rx[i + 1], rx[i + 2]);
			return -9;
		}
		if (rx[i + 1] != (byte)function) continue;
		if (function == 0x03) {
			if (rx[i + 2] != 2) continue;
			len = 7;
		} else if (function == 0x10) {
			len = 8;
		} else {
			continue;
		}
		if (i + len > rxLen) continue;
		gotCrc = (uint16_t)rx[i + len - 2] | ((uint16_t)rx[i + len - 1] << 8);
		expectedCrc = Deye_ModbusCRC(rx + i, len - 2);
		if (gotCrc != expectedCrc) continue;
		if (function == 0x03 && value) {
			*value = ((int)rx[i + 3] << 8) | rx[i + 4];
		}
		if (function == 0x10) {
			int reg = ((int)rx[i + 2] << 8) | rx[i + 3];
			int qty = ((int)rx[i + 4] << 8) | rx[i + 5];
			if (reg != g_deye_register || qty != 1) continue;
		}
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman Modbus RX valid function=0x%02X offset=%i", function, i);
		return 0;
	}
	ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman response has no valid Modbus function=0x%02X payload", function);
	return -8;
invalid:
	ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman invalid V5 response length/seq/serial/checksum");
	return -8;
}

static int Deye_BuildSolarmanV5Frame(const byte *modbus, int modbusLen, byte *out, int outLen) {
	int payloadLen = 15 + modbusLen;
	int pos = 0;
	int checksum = 0;
	int i;
	uint32_t serial = g_deye_logger_serial;

	if (outLen < payloadLen + 13) return -1;

	out[pos++] = 0xA5;
	out[pos++] = (byte)(payloadLen & 0xFF);
	out[pos++] = (byte)((payloadLen >> 8) & 0xFF);
	out[pos++] = 0x10;
	out[pos++] = 0x45;
	out[pos++] = g_deye_sequence++;
	out[pos++] = 0x00;
	out[pos++] = (byte)(serial & 0xFF);
	out[pos++] = (byte)((serial >> 8) & 0xFF);
	out[pos++] = (byte)((serial >> 16) & 0xFF);
	out[pos++] = (byte)((serial >> 24) & 0xFF);
	out[pos++] = 0x02;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;
	out[pos++] = 0x00;

	for (i = 0; i < modbusLen; i++) {
		out[pos++] = modbus[i];
	}
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman V5 encode payloadLen=%i modbusOffset=%i",
		payloadLen, pos - modbusLen);

	for (i = 1; i < pos; i++) {
		checksum = (checksum + out[i]) & 0xFF;
	}
	out[pos++] = (byte)checksum;
	out[pos++] = 0x15;
	return pos;
}

static int Deye_Transaction(const byte *modbus, int modbusLen, int function, int *value) {
	byte frame[DEYE_V5_MAX_LEN];
	byte rx[DEYE_RX_MAX_LEN];
	int frameLen, recvLen;
	int sock;
	int rc;
	struct sockaddr_in addr;
	struct timeval tv;
	uint32_t resolved;

	g_deye_last_rx_len = 0;
	Deye_LogHex("DeyeSolarman Modbus TX", modbus, modbusLen);

	frameLen = Deye_BuildSolarmanV5Frame(modbus, modbusLen, frame, sizeof(frame));
	if (frameLen <= 0) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman: failed to build V5 frame");
		g_deye_last_rc = -2;
		g_deye_fail_count++;
		return -2;
	}
	Deye_LogHex("DeyeSolarman V5 TX", frame, frameLen);

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman: socket() failed");
		g_deye_last_rc = -3;
		g_deye_fail_count++;
		return -3;
	}

	tv.tv_sec = 3;
	tv.tv_usec = 0;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

	resolved = inet_addr(g_deye_ip);
	if (resolved == 0xFFFFFFFF) {
		struct hostent *he = gethostbyname(g_deye_ip);
		if (he && he->h_addr_list && he->h_addr_list[0]) {
			resolved = *(uint32_t *)he->h_addr_list[0];
		}
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons((uint16_t)g_deye_port);
	addr.sin_addr.s_addr = resolved;

	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman TCP: connecting to %s:%i timeout=1000ms", g_deye_ip, g_deye_port);
	if (Deye_TCPConnectTimeoutMs(sock, &addr, 1000) < 0) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman TCP: connect failed rc=-4 (%s)", Deye_RcText(-4));
		closesocket(sock);
		g_deye_last_rc = -4;
		g_deye_fail_count++;
		return -4;
	}
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman TCP: connected");

	if (send(sock, frame, frameLen, 0) != frameLen) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman TCP: send failed rc=-5 (%s)", Deye_RcText(-5));
		closesocket(sock);
		g_deye_last_rc = -5;
		g_deye_fail_count++;
		return -5;
	}
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman TCP: sent %i bytes", frameLen);

	recvLen = recv(sock, rx, sizeof(rx), 0);
	if (recvLen <= 0) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman TCP: recv timeout/error %i rc=-6 (%s)", recvLen, Deye_RcText(-6));
		closesocket(sock);
		g_deye_last_rc = -6;
		g_deye_fail_count++;
		return -6;
	}
	g_deye_last_rx_len = recvLen;
	Deye_LogHex("DeyeSolarman RX", rx, recvLen);
	closesocket(sock);
	rc = Deye_ParseModbusResponse(rx, recvLen, function, frame[5], value);
	if (rc != 0) {
		g_deye_last_rc = rc;
		g_deye_fail_count++;
		return rc;
	}
	g_deye_last_rc = 0;
	g_deye_fail_count = 0;
	return 0;
}

static int Deye_SendPercent(int percent) {
	byte modbus[DEYE_MODBUS_MAX_LEN];
	int modbusLen;
	int rc;

	if (!AgriHTLicense_IsActive()) return -10;
	percent = Deye_ClampPercent(percent);
	if (percent == g_deye_last_percent && g_deye_last_rc == 0) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman WRITE R40 skipped unchanged value=%i%%", percent);
		return 0;
	}
	g_deye_send_count++;
	g_deye_last_percent = percent;
	Deye_LogConfig("DeyeSolarman WRITE R40 begin", percent);
	modbusLen = Deye_BuildWriteRegister40Modbus(modbus, sizeof(modbus), percent);
	if (modbusLen <= 0) {
		g_deye_last_rc = -1;
		g_deye_fail_count++;
		return -1;
	}
	rc = Deye_Transaction(modbus, modbusLen, 0x10, NULL);
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman WRITE R40 done value=%i rc=%i (%s) rxLen=%i count=%u",
		percent, rc, Deye_RcText(rc), g_deye_last_rx_len, g_deye_send_count);
	return rc;
}

static int Deye_ReadRegister40(int *value) {
	byte modbus[DEYE_MODBUS_MAX_LEN];
	int modbusLen;
	int rc;

	if (!AgriHTLicense_IsActive()) return -10;
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman READ R40 begin ip=%s serial=%u",
		g_deye_ip, (unsigned int)g_deye_logger_serial);
	modbusLen = Deye_BuildReadRegisterModbus(modbus, sizeof(modbus));
	if (modbusLen <= 0) return -1;
	rc = Deye_Transaction(modbus, modbusLen, 0x03, value);
	if (rc == 0) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman READ R40 done value=%i%%", *value);
	} else {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman READ R40 failed rc=%i (%s)", rc, Deye_RcText(rc));
	}
	return rc;
}

static int Deye_ProbeTCPAt(const char *ip, int timeoutSeconds) {
	int sock;
	struct sockaddr_in addr;
	struct timeval tv;

	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman PROBE begin ip=%s port=%i timeout=%is", ip, g_deye_port, timeoutSeconds);
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman PROBE: socket failed rc=-3 (%s)", Deye_RcText(-3));
		g_deye_last_rc = -3;
		return -3;
	}
	tv.tv_sec = timeoutSeconds;
	tv.tv_usec = 0;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons((uint16_t)g_deye_port);
	addr.sin_addr.s_addr = inet_addr(ip);

	if (Deye_TCPConnectTimeoutMs(sock, &addr, timeoutSeconds * 1000) < 0) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman PROBE: TCP connect failed rc=-7 (%s)", Deye_RcText(-7));
		closesocket(sock);
		g_deye_last_rc = -7;
		return -7;
	}
	closesocket(sock);
	g_deye_last_rc = 0;
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman PROBE: TCP connect OK");
	return 0;
}

static int Deye_ProbeTCP(void) {
	return Deye_ProbeTCPAt(g_deye_ip, 3);
}

static int Deye_CheckSerialAt(const char *ip, int connectTimeoutMs) {
	byte modbus[DEYE_MODBUS_MAX_LEN];
	byte frame[DEYE_V5_MAX_LEN];
	byte rx[DEYE_RX_MAX_LEN];
	int modbusLen, frameLen, recvLen;
	int sock;
	struct sockaddr_in addr;
	struct timeval tv;
	uint32_t serialInReply;

	modbusLen = Deye_BuildReadRegisterModbus(modbus, sizeof(modbus));
	if (modbusLen <= 0) return -1;
	frameLen = Deye_BuildSolarmanV5Frame(modbus, modbusLen, frame, sizeof(frame));
	if (frameLen <= 0) return -2;

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) return -3;

	tv.tv_sec = 0;
	tv.tv_usec = 250000;
	setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
	setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons((uint16_t)g_deye_port);
	addr.sin_addr.s_addr = inet_addr(ip);

	if (Deye_TCPConnectTimeoutMs(sock, &addr, connectTimeoutMs) < 0) {
		closesocket(sock);
		return -4;
	}
	if (send(sock, frame, frameLen, 0) != frameLen) {
		closesocket(sock);
		return -5;
	}
	recvLen = recv(sock, rx, sizeof(rx), 0);
	closesocket(sock);
	if (recvLen < 11) {
		return -6;
	}
	serialInReply = (uint32_t)rx[7] |
		((uint32_t)rx[8] << 8) |
		((uint32_t)rx[9] << 16) |
		((uint32_t)rx[10] << 24);
	if (serialInReply != g_deye_logger_serial) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman DISCOVER ip=%s serial mismatch got=%u want=%u",
			ip, (unsigned int)serialInReply, (unsigned int)g_deye_logger_serial);
		return -8;
	}
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman DISCOVER ip=%s serial OK=%u rxLen=%i",
		ip, (unsigned int)serialInReply, recvLen);
	return 0;
}

static void Deye_BuildSubnetIP(int host, char *out, int outLen) {
	const char *myip = HAL_GetMyIPString();
	int a = 0, b = 0, c = 0, d = 0;
	if (myip && sscanf(myip, "%i.%i.%i.%i", &a, &b, &c, &d) == 4) {
		snprintf(out, outLen, "%i.%i.%i.%i", a, b, c, host);
	} else {
		snprintf(out, outLen, "192.168.1.%i", host);
	}
}

static void Deye_SetProductNameFromSN(void) {
	char name[32];
	snprintf(name, sizeof(name), "ZN-%u", (unsigned int)g_deye_logger_serial);
	CFG_SetShortDeviceName(name);
	CFG_SetDeviceName(name);
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman product name=%s", name);
}

static void Deye_StartDiscover(void) {
	if (!AgriHTLicense_IsActive()) {
		g_deye_discover_active = 0;
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman discovery locked deviceId=%s",
			AgriHTLicense_GetDeviceId());
		return;
	}
	g_deye_discover_active = 1;
	g_deye_discover_found = 0;
	g_deye_ip_confirmed = 0;
	g_deye_discover_host = 1;
	g_deye_discover_try_saved_ip = g_deye_ip[0] ? 1 : 0;
	g_deye_discover_saved_retry_seconds = 0;
	g_deye_discover_seconds_until_next = 0;
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman discover started on subnet of %s for serial=%u",
		HAL_GetMyIPString(), (unsigned int)g_deye_logger_serial);
}

static int Deye_IsVoltageType(int type) {
	return type == ChType_Voltage_div10 || type == ChType_Voltage_div100;
}

static int Deye_IsCurrentType(int type) {
	return type == ChType_Current_div10 || type == ChType_Current_div100 ||
		type == ChType_Current_div1000;
}

static void Deye_AutoDetectMeasurementChannels(void) {
	int ch;
	int activePowerCh = -1;
	int voltageCh = -1;
	int currentCh = -1;
	int freqCh = -1;
	int pfCh = -1;

	if (!g_deye_auto_detect_channels) return;
	for (ch = 0; ch < CHANNEL_MAX; ch++) {
		int type = CHANNEL_GetType(ch);
		if (type == ChType_ActivePower && activePowerCh < 0) {
			activePowerCh = ch;
		}
		if (voltageCh < 0 && Deye_IsVoltageType(type)) {
			voltageCh = ch;
		}
		if (currentCh < 0 && Deye_IsCurrentType(type)) {
			currentCh = ch;
		}
		if (freqCh < 0 && type == ChType_Frequency_div100) {
			freqCh = ch;
		}
		if (pfCh < 0 && type == ChType_PowerFactor_div1000) {
			pfCh = ch;
		}
	}
	if (g_deye_grid_channel < 0 && activePowerCh >= 0) {
		g_deye_grid_channel = activePowerCh;
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman AUTO-DETECT signed active power channel=%i", g_deye_grid_channel);
	}
	if (voltageCh >= 0 && voltageCh != g_deye_voltage_channel) {
		g_deye_voltage_channel = voltageCh;
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman AUTO-DETECT voltage channel=%i", g_deye_voltage_channel);
	}
	if (currentCh >= 0 && currentCh != g_deye_current_channel) {
		g_deye_current_channel = currentCh;
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman AUTO-DETECT current channel=%i", g_deye_current_channel);
	}
	if (freqCh >= 0 && freqCh != g_deye_freq_channel) {
		g_deye_freq_channel = freqCh;
	}
	if (pfCh >= 0 && pfCh != g_deye_pf_channel) {
		g_deye_pf_channel = pfCh;
	}
}

static int Deye_ReadGridPowerW(void) {
	float voltage = 0.0f;
	float current = 0.0f;
	int gridW = 0;

	Deye_AutoDetectMeasurementChannels();
	if (g_deye_voltage_channel >= 0) {
		voltage = CHANNEL_GetFinalValue(g_deye_voltage_channel);
		g_deye_last_voltage_v = (int)voltage;
	}
	if (g_deye_current_channel >= 0) {
		current = CHANNEL_GetFinalValue(g_deye_current_channel);
		g_deye_last_current_ma = (int)(current * 1000.0f);
	}
	if (g_deye_freq_channel >= 0) {
		g_deye_last_freq_hz = CHANNEL_GetFinalValue(g_deye_freq_channel);
	}
	if (g_deye_pf_channel >= 0) {
		g_deye_last_pf = CHANNEL_GetFinalValue(g_deye_pf_channel);
	}
	if (g_deye_grid_channel >= 0) {
		gridW = (int)CHANNEL_GetFinalValue(g_deye_grid_channel);
	}
	if (voltage > 1.0f && current > 0.001f) {
		g_deye_last_apparent_w = (int)(voltage * current);
	}
	if (g_deye_grid_invert) gridW = -gridW;
	g_deye_last_grid_w = gridW;
	return gridW;
}

static int Deye_StabilizeGridDirection(int rawGridW) {
	int rawSign;
	int stableSign;

	if (rawGridW > 0) rawSign = 1;
	else if (rawGridW < 0) rawSign = -1;
	else rawSign = 0;

	if (!g_deye_grid_sign_initialized) {
		g_deye_stable_grid_w = rawGridW;
		g_deye_grid_sign_initialized = 1;
		return rawGridW;
	}

	if (g_deye_stable_grid_w > 0) stableSign = 1;
	else if (g_deye_stable_grid_w < 0) stableSign = -1;
	else stableSign = 0;

	if (rawSign == 0 || stableSign == 0 || rawSign == stableSign ||
		rawGridW >= 150 || rawGridW <= -150) {
		g_deye_stable_grid_w = rawGridW;
		g_deye_pending_grid_sign = 0;
		g_deye_pending_grid_sign_count = 0;
		return rawGridW;
	}

	if (rawSign != g_deye_pending_grid_sign) {
		g_deye_pending_grid_sign = rawSign;
		g_deye_pending_grid_sign_count = 1;
	} else {
		g_deye_pending_grid_sign_count++;
	}

	if (g_deye_pending_grid_sign_count >= 2) {
		g_deye_stable_grid_w = rawGridW;
		g_deye_pending_grid_sign = 0;
		g_deye_pending_grid_sign_count = 0;
		return rawGridW;
	}

	ADDLOG_INFO(LOG_FEATURE_DRV,
		"DeyeSolarman SIGN guard raw=%iW held=%iW pendingSign=%i count=%i",
		rawGridW, g_deye_stable_grid_w, g_deye_pending_grid_sign,
		g_deye_pending_grid_sign_count);
	return g_deye_stable_grid_w;
}

static void Deye_RunAutoControl(void) {
	int rawGridW;
	int stableGridW;
	int controlGridW;
	int errorW;
	int delta;
	int direction;
	int targetPercent;
	int rc;

	rawGridW = Deye_ReadGridPowerW();
	stableGridW = Deye_StabilizeGridDirection(rawGridW);
	if (!g_deye_filter_initialized) {
		g_deye_filtered_grid_w = stableGridW;
		g_deye_filter_initialized = 1;
	} else {
		/* Follow large load changes immediately; smooth small meter jitter near target. */
		int jumpW = stableGridW - g_deye_filtered_grid_w;
		if (jumpW > 120 || jumpW < -120) {
			g_deye_filtered_grid_w = stableGridW;
		} else {
			g_deye_filtered_grid_w = (g_deye_filtered_grid_w * 2 + stableGridW) / 3;
		}
	}
	controlGridW = g_deye_filtered_grid_w;
	if (!g_deye_ip_confirmed) {
		if (!g_deye_discover_active) {
			Deye_StartDiscover();
		}
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman AUTO waiting for Deye IP, grid=%iW V=%i I=%imA apparent=%iW",
			rawGridW, g_deye_last_voltage_v, g_deye_last_current_ma, g_deye_last_apparent_w);
		return;
	}
	if (g_deye_grid_channel < 0 && g_deye_last_apparent_w <= 0) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman AUTO waiting for metering channels");
		return;
	}

	delta = 0;
	direction = 0;
	errorW = 0;
	if (controlGridW > g_deye_target_grid_w) {
		errorW = controlGridW - g_deye_target_grid_w;
		delta = Deye_FeedbackStep(errorW);
		direction = 1;
	} else if (controlGridW < g_deye_export_limit_w) {
		errorW = g_deye_export_limit_w - controlGridW;
		delta = -Deye_FeedbackStep(errorW);
		/* Export is the unsafe direction, so reduce one step faster. */
		if (errorW >= 30 && delta > -15) delta--;
		direction = -1;
	}
	if (direction && g_deye_last_control_direction &&
		direction != g_deye_last_control_direction && errorW < 100) {
		delta = direction;
	}

	targetPercent = Deye_ClampPercent(g_deye_auto_percent + delta);
	ADDLOG_INFO(LOG_FEATURE_DRV,
		"DeyeSolarman FEEDBACK raw=%iW stable=%iW filtered=%iW band=%i..%iW error=%iW step=%i%% limit=%i%%->%i%% V=%i I=%imA chP=%i",
		rawGridW, stableGridW, controlGridW, g_deye_export_limit_w, g_deye_target_grid_w,
		errorW, delta,
		g_deye_auto_percent, targetPercent,
		g_deye_last_voltage_v, g_deye_last_current_ma, g_deye_grid_channel);

	if (targetPercent == g_deye_auto_percent && g_deye_send_count > 0 && g_deye_last_rc == 0) {
		return;
	}
	if (direction) g_deye_last_control_direction = direction;
	g_deye_auto_percent = targetPercent;
	rc = Deye_SendPercent(g_deye_auto_percent);
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman AUTO send rc=%i (%s)", rc, Deye_RcText(rc));
	if (rc != 0 && g_deye_fail_count >= 3) {
		ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman AUTO lost Deye IP after %i failures, restarting discovery", g_deye_fail_count);
		g_deye_ip_confirmed = 0;
		g_deye_discover_found = 0;
		g_deye_fail_count = 0;
		Deye_StartDiscover();
	}
}

static void Deye_RunDiscoverStep(void) {
	char ip[16];
	int rc;
	int attempts;
	const char *myIp = HAL_GetMyIPString();

	if (!myIp || !myIp[0] || !strcmp(myIp, "0.0.0.0")) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman DISCOVER waiting for LAN IP");
		return;
	}

	for (attempts = 0; attempts < 4 && g_deye_discover_active; attempts++) {
		if (g_deye_discover_try_saved_ip || g_deye_discover_saved_retry_seconds <= 0) {
			g_deye_discover_try_saved_ip = 0;
			g_deye_discover_saved_retry_seconds = 5;
			rc = Deye_CheckSerialAt(g_deye_ip, 800);
			if (rc == 0) {
				g_deye_discover_active = 0;
				g_deye_discover_found = 1;
				g_deye_ip_confirmed = 1;
				ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman DISCOVER reused saved ip=%s serial=%u",
					g_deye_ip, (unsigned int)g_deye_logger_serial);
			}
			continue;
		}

		if (g_deye_discover_host > 256) {
			g_deye_discover_active = 0;
			ADDLOG_ERROR(LOG_FEATURE_DRV, "DeyeSolarman DISCOVER done: no logger found on subnet");
			return;
		}
		Deye_BuildSubnetIP(g_deye_discover_host, ip, sizeof(ip));
		g_deye_discover_host++;
		if (!strcmp(ip, HAL_GetMyIPString())) continue;

		rc = Deye_CheckSerialAt(ip, 250);
		if (rc == 0) {
			snprintf(g_deye_ip, sizeof(g_deye_ip), "%s", ip);
			g_deye_discover_active = 0;
			g_deye_discover_found = 1;
			g_deye_ip_confirmed = 1;
			ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman DISCOVER found ip=%s serial=%u",
				g_deye_ip, (unsigned int)g_deye_logger_serial);
		}
	}
}

static commandResult_t Deye_CMD_SetIP(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	snprintf(g_deye_ip, sizeof(g_deye_ip), "%s", Tokenizer_GetArg(0));
	g_deye_ip_confirmed = 1;
	g_deye_discover_active = 0;
	g_deye_discover_found = 1;
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman IP=%s", g_deye_ip);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_SetPort(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	g_deye_port = Tokenizer_GetArgIntegerDefault(0, g_deye_port);
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman port=%i", g_deye_port);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_SetLoggerSerial(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	g_deye_logger_serial = Deye_ParseU32(Tokenizer_GetArg(0), g_deye_logger_serial);
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman logger serial=%u", (unsigned int)g_deye_logger_serial);
	Deye_SetProductNameFromSN();
	g_deye_auto_enabled = 1;
	g_deye_fixed_enabled = 0;
	Deye_StartDiscover();
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_SetSlave(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	g_deye_slave_id = Tokenizer_GetArgIntegerRange(0, 1, 247);
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman slave=%i", g_deye_slave_id);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_SetRegister(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	g_deye_register = Tokenizer_GetArgIntegerRange(0, 0, 65535);
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman register=%i", g_deye_register);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_SetFixedPercent(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	g_deye_fixed_percent = Deye_Clamp0To100(Tokenizer_GetArgIntegerDefault(0, g_deye_fixed_percent));
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman fixed percent=%i", g_deye_fixed_percent);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_SetPeriodMs(const void *context, const char *cmd, const char *args, int flags) {
	int ms;
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	ms = Tokenizer_GetArgIntegerDefault(0, g_deye_update_seconds * 1000);
	if (ms < 2000) ms = 2000;
	g_deye_update_seconds = (ms + 999) / 1000;
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman period=%i seconds", g_deye_update_seconds);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_EnableFixed(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	g_deye_fixed_enabled = Tokenizer_GetArgIntegerDefault(0, 0) ? 1 : 0;
	if (g_deye_fixed_enabled) g_deye_auto_enabled = 0;
	g_deye_seconds_until_send = 0;
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman fixed mode=%i", g_deye_fixed_enabled);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_SendFixed(const void *context, const char *cmd, const char *args, int flags) {
	int rc = Deye_SendPercent(g_deye_fixed_percent);
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman send fixed rc=%i (%s)", rc, Deye_RcText(rc));
	return rc == 0 ? CMD_RES_OK : CMD_RES_ERROR;
}

static commandResult_t Deye_CMD_ReadR40(const void *context, const char *cmd, const char *args, int flags) {
	int value = -1;
	int rc = Deye_ReadRegister40(&value);
	if (rc == 0) g_deye_verify_readback = value;
	return rc == 0 ? CMD_RES_OK : CMD_RES_ERROR;
}

static commandResult_t Deye_CMD_VerifyR40(const void *context, const char *cmd, const char *args, int flags) {
	int rc;
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	g_deye_verify_expected = Deye_Clamp0To100(Tokenizer_GetArgIntegerDefault(0, g_deye_fixed_percent));
	g_deye_fixed_enabled = 0;
	g_deye_auto_enabled = 0;
	g_deye_verify_readback = -1;
	g_deye_verify_grid_before_w = Deye_ReadGridPowerW();
	ADDLOG_INFO(LOG_FEATURE_CMD,
		"DeyeSolarman VERIFY start R40=%i%% gridBefore=%iW; auto paused; readback scheduled in 60s",
		g_deye_verify_expected, g_deye_verify_grid_before_w);
	rc = Deye_SendPercent(g_deye_verify_expected);
	if (rc != 0) {
		g_deye_verify_seconds = 0;
		ADDLOG_ERROR(LOG_FEATURE_CMD, "DeyeSolarman VERIFY write failed rc=%i (%s)", rc, Deye_RcText(rc));
		return CMD_RES_ERROR;
	}
	g_deye_verify_seconds = 60;
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_Status(const void *context, const char *cmd, const char *args, int flags) {
	ADDLOG_INFO(LOG_FEATURE_CMD, "AgriHT license=%s deviceId=%s",
		AgriHTLicense_IsActive() ? "ACTIVE" : "LOCKED", AgriHTLicense_GetDeviceId());
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman ip=%s port=%i serial=%u slave=%i reg=%i fixed=%i fixedOn=%i autoOn=%i period=%is lastRc=%i(%s) lastPercent=%i lastRxLen=%i sends=%u verifyIn=%is verifyExpected=%i verifyReadback=%i",
		g_deye_ip, g_deye_port, (unsigned int)g_deye_logger_serial, g_deye_slave_id,
		g_deye_register, g_deye_fixed_percent, g_deye_fixed_enabled, g_deye_auto_enabled, g_deye_update_seconds,
		g_deye_last_rc, Deye_RcText(g_deye_last_rc), g_deye_last_percent, g_deye_last_rx_len,
		g_deye_send_count, g_deye_verify_seconds, g_deye_verify_expected, g_deye_verify_readback);
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman zeroExport grid=%iW target=%iW V=%i I=%imA apparent=%iW rated=%iW est=%iW autoLimit=%i%% min=%i%% max=%i%% gain=%i wPerPct=%i step=%i deadband=%i chP=%i chV=%i chI=%i autoDetect=%i invert=%i discover=%i found=%i ipOK=%i fail=%i",
		g_deye_last_grid_w, g_deye_target_grid_w, g_deye_last_voltage_v,
		g_deye_last_current_ma, g_deye_last_apparent_w, g_deye_rated_w,
		Deye_EstimatedOutputW(), g_deye_auto_percent, g_deye_min_percent, g_deye_max_percent,
		g_deye_gain_percent, g_deye_w_per_percent, g_deye_max_step_percent,
		g_deye_deadband_w, g_deye_grid_channel, g_deye_voltage_channel,
		g_deye_current_channel, g_deye_auto_detect_channels, g_deye_grid_invert,
		g_deye_discover_active, g_deye_discover_found, g_deye_ip_confirmed,
		g_deye_fail_count);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_Probe(const void *context, const char *cmd, const char *args, int flags) {
	int rc = Deye_ProbeTCP();
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman probe rc=%i (%s)", rc, Deye_RcText(rc));
	return rc == 0 ? CMD_RES_OK : CMD_RES_ERROR;
}

static void Deye_TrimLicense(char *dst, const char *src, int maxLen) {
	int i = 0;
	while (*src && i < maxLen - 1) {
		if (*src > ' ' && *src <= '~') {
			dst[i++] = *src;
		}
		src++;
	}
	dst[i] = '\0';
}

static void Deye_ApplySetupFromTokenizer(void) {
	int setupPower;
	g_deye_logger_serial = Deye_ParseU32(Tokenizer_GetArg(0), g_deye_logger_serial);
	g_deye_target_grid_w = Tokenizer_GetArgIntegerDefault(1, g_deye_target_grid_w);
	g_deye_min_percent = Deye_Clamp0To100(Tokenizer_GetArgIntegerDefault(2, g_deye_min_percent));
	g_deye_max_percent = Deye_Clamp0To100(Tokenizer_GetArgIntegerDefault(3, g_deye_max_percent));
	setupPower = Tokenizer_GetArgIntegerDefault(4, g_deye_rated_w);
	if (setupPower >= 200) {
		g_deye_rated_w = setupPower;
		g_deye_w_per_percent = g_deye_rated_w / 100;
		if (g_deye_w_per_percent < 1) g_deye_w_per_percent = 1;
	} else if (setupPower > 0) {
		g_deye_w_per_percent = setupPower;
		if (g_deye_rated_w <= 0) g_deye_rated_w = 800;
	}
	if (Tokenizer_GetArgsCount() >= 6) {
		const char *ipArg = Tokenizer_GetArg(5);
		if (ipArg && ipArg[0] && strcmp(ipArg, "0")) {
			snprintf(g_deye_ip, sizeof(g_deye_ip), "%s", ipArg);
			g_deye_ip_confirmed = 1;
			g_deye_discover_active = 0;
			g_deye_discover_found = 1;
		} else {
			g_deye_ip_confirmed = 0;
		}
	}
	if (Tokenizer_GetArgsCount() >= 7) {
		g_deye_gain_percent = Tokenizer_GetArgIntegerRange(6, 1, 500);
	}
	if (Tokenizer_GetArgsCount() >= 8) {
		g_deye_max_step_percent = Tokenizer_GetArgIntegerRange(7, 1, 100);
	}
	if (Tokenizer_GetArgsCount() >= 9) {
		g_deye_deadband_w = Tokenizer_GetArgIntegerRange(8, 0, 1000);
	}
	if (Tokenizer_GetArgsCount() >= 10) {
		int periodMs = Tokenizer_GetArgIntegerRange(9, 2000, 60000);
		g_deye_update_seconds = (periodMs + 999) / 1000;
	}
	if (Tokenizer_GetArgsCount() >= 11) {
		g_deye_export_limit_w = Tokenizer_GetArgIntegerRange(10, -1000, 0);
	}
	if (Tokenizer_GetArgsCount() >= 12) {
		const char *licenseArg = Tokenizer_GetArg(11);
		if (licenseArg) {
			char cleanKey[32];
			Deye_TrimLicense(cleanKey, licenseArg, sizeof(cleanKey));
			if (strlen(cleanKey) == 16) {
				AgriHTLicense_Activate(cleanKey);
			}
		}
	}
	if (Tokenizer_GetArgsCount() >= 13) {
		g_deye_grid_invert = Tokenizer_GetArgIntegerDefault(12, 0) ? 1 : 0;
	}
	if (g_deye_w_per_percent < 1) g_deye_w_per_percent = 1;
	if (g_deye_min_percent > g_deye_max_percent) {
		int t = g_deye_min_percent;
		g_deye_min_percent = g_deye_max_percent;
		g_deye_max_percent = t;
	}
	g_deye_fixed_enabled = 0;
	g_deye_auto_enabled = 1;
	/* PM01_A002 HLW8112: ch5=Power A signed, ch6=Power B, ch7=apparent. */
	g_deye_grid_channel = 5;
	g_deye_auto_detect_channels = 1;
	Deye_SetProductNameFromSN();
	if (!g_deye_ip_confirmed) {
		Deye_StartDiscover();
	}
}

static void Deye_BuildStartupCommand(char *out, int outLen) {
	snprintf(out, outLen, "DeyeSetup %u %i %i %i %i %s %i %i %i %i %i %s %i",
		(unsigned int)g_deye_logger_serial, g_deye_target_grid_w,
		g_deye_min_percent, g_deye_max_percent, g_deye_rated_w,
		g_deye_ip, g_deye_gain_percent, g_deye_max_step_percent,
		g_deye_deadband_w, g_deye_update_seconds * 1000,
		g_deye_export_limit_w,
		AgriHTLicense_IsActive() ? AgriHTLicense_GetKey() : "0",
		g_deye_grid_invert);
}

static commandResult_t Deye_CMD_Activate(const void *context, const char *cmd, const char *args, int flags) {
	char startup[160];
	char cleanKey[32];
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	Deye_TrimLicense(cleanKey, Tokenizer_GetArg(0), sizeof(cleanKey));
	if (!AgriHTLicense_Activate(cleanKey)) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "AgriHT activation rejected deviceId=%s",
			AgriHTLicense_GetDeviceId());
		return CMD_RES_ERROR;
	}
	Deye_BuildStartupCommand(startup, sizeof(startup));
	CFG_SetShortStartupCommand(startup);
	CFG_Save_IfThereArePendingChanges();
	g_deye_auto_enabled = 1;
	g_deye_seconds_until_send = 0;
	if (!g_deye_ip_confirmed) Deye_StartDiscover();
	ADDLOG_INFO(LOG_FEATURE_CMD, "AgriHT activation accepted deviceId=%s",
		AgriHTLicense_GetDeviceId());
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_Setup(const void *context, const char *cmd, const char *args, int flags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	Deye_ApplySetupFromTokenizer();
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman setup SN=%u target=%iW min=%i%% max=%i%% rated=%iW ip=%s gain=%i step=%i%% deadband=%iW period=%is ipOK=%i",
		(unsigned int)g_deye_logger_serial, g_deye_target_grid_w,
		g_deye_min_percent, g_deye_max_percent, g_deye_rated_w,
		g_deye_ip, g_deye_gain_percent, g_deye_max_step_percent,
		g_deye_deadband_w, g_deye_update_seconds, g_deye_ip_confirmed);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_Save(const void *context, const char *cmd, const char *args, int flags) {
	char new_deye[128];
	char final_cmd[256];
	Deye_BuildStartupCommand(new_deye, sizeof(new_deye));
	const char *current = CFG_GetShortStartupCommand();
	if (current && strstr(current, "startDriver HLW8112SPI")) {
		snprintf(final_cmd, sizeof(final_cmd), "startDriver HLW8112SPI; HLW8112_SetResistorGain 1.0 0.2267 1.0; %s", new_deye);
	} else {
		strncpy(final_cmd, new_deye, sizeof(final_cmd));
	}
	CFG_SetShortStartupCommand(final_cmd);
	CFG_Save_IfThereArePendingChanges();
	ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman saved startup: %s", final_cmd);
	return CMD_RES_OK;
}

static commandResult_t Deye_CMD_Main(const void *context, const char *cmd, const char *args, int flags) {
	const char *sub;

	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_GetArgsCount() < 1) {
		Deye_CMD_Status(context, cmd, args, flags);
		ADDLOG_INFO(LOG_FEATURE_CMD, "Deye usage: Deye verify <percent> writes R40 then reads it after 60s; Deye read reads R40 now. Setup: setup <SN> [targetW min%% max%% maxPowerW ip], save");
		return CMD_RES_OK;
	}

	sub = Tokenizer_GetArg(0);
	if (!stricmp(sub, "status")) {
		return Deye_CMD_Status(context, cmd, args, flags);
	}
	if (!stricmp(sub, "send")) {
		return Deye_CMD_SendFixed(context, cmd, args, flags);
	}
	if (!stricmp(sub, "read") || !stricmp(sub, "read40")) {
		return Deye_CMD_ReadR40(context, cmd, args, flags);
	}
	if (!stricmp(sub, "verify") || !stricmp(sub, "verify40")) {
		return Deye_CMD_VerifyR40(context, cmd, Tokenizer_GetArgFrom(1), flags);
	}
	if (!stricmp(sub, "probe") || !stricmp(sub, "tcp")) {
		return Deye_CMD_Probe(context, cmd, args, flags);
	}
	if (!stricmp(sub, "discover")) {
		Deye_StartDiscover();
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "setup")) {
		return Deye_CMD_Setup(context, cmd, Tokenizer_GetArgFrom(1), flags);
	}
	if (!stricmp(sub, "save")) {
		return Deye_CMD_Save(context, cmd, args, flags);
	}
	if (!stricmp(sub, "ip")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		snprintf(g_deye_ip, sizeof(g_deye_ip), "%s", Tokenizer_GetArg(1));
		g_deye_ip_confirmed = 1;
		g_deye_discover_active = 0;
		g_deye_discover_found = 1;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman IP=%s", g_deye_ip);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "pct") || !stricmp(sub, "percent")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_fixed_percent = Deye_Clamp0To100(Tokenizer_GetArgIntegerDefault(1, g_deye_fixed_percent));
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman fixed percent=%i", g_deye_fixed_percent);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "enable") || !stricmp(sub, "on")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_fixed_enabled = Tokenizer_GetArgIntegerDefault(1, 0) ? 1 : 0;
		if (g_deye_fixed_enabled) g_deye_auto_enabled = 0;
		g_deye_seconds_until_send = 0;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman fixed mode=%i", g_deye_fixed_enabled);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "auto")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_auto_enabled = Tokenizer_GetArgIntegerDefault(1, 0) ? 1 : 0;
		if (g_deye_auto_enabled) g_deye_fixed_enabled = 0;
		g_deye_seconds_until_send = 0;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman zero export auto=%i", g_deye_auto_enabled);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "autodetect")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_auto_detect_channels = Tokenizer_GetArgIntegerDefault(1, 0) ? 1 : 0;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman measurement auto-detect=%i", g_deye_auto_detect_channels);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "rated")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_rated_w = Tokenizer_GetArgIntegerRange(1, 1, 50000);
		g_deye_w_per_percent = g_deye_rated_w / 100;
		if (g_deye_w_per_percent < 1) g_deye_w_per_percent = 1;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman rated power=%iW", g_deye_rated_w);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "wperpct")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_w_per_percent = Tokenizer_GetArgIntegerRange(1, 1, 5000);
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman watts per percent step=%i", g_deye_w_per_percent);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "target")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_target_grid_w = Tokenizer_GetArgIntegerDefault(1, g_deye_target_grid_w);
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman target grid=%iW", g_deye_target_grid_w);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "min")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_min_percent = Deye_Clamp0To100(Tokenizer_GetArgIntegerDefault(1, g_deye_min_percent));
		if (g_deye_max_percent < g_deye_min_percent) g_deye_max_percent = g_deye_min_percent;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman min percent=%i", g_deye_min_percent);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "max")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_max_percent = Deye_Clamp0To100(Tokenizer_GetArgIntegerDefault(1, g_deye_max_percent));
		if (g_deye_min_percent > g_deye_max_percent) g_deye_min_percent = g_deye_max_percent;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman max percent=%i", g_deye_max_percent);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "gain")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_gain_percent = Tokenizer_GetArgIntegerRange(1, 1, 500);
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman gain=%i", g_deye_gain_percent);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "step")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_max_step_percent = Tokenizer_GetArgIntegerRange(1, 1, 100);
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman max step=%i%%", g_deye_max_step_percent);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "deadband")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_deadband_w = Tokenizer_GetArgIntegerRange(1, 0, 1000);
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman deadband=%iW", g_deye_deadband_w);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "gridch")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_grid_channel = Tokenizer_GetArgIntegerRange(1, 0, CHANNEL_MAX - 1);
		g_deye_auto_detect_channels = 0;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman grid channel=%i", g_deye_grid_channel);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "invert")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_grid_invert = Tokenizer_GetArgIntegerDefault(1, 0) ? 1 : 0;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman grid invert=%i", g_deye_grid_invert);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "name")) {
		Deye_SetProductNameFromSN();
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "port")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_port = Tokenizer_GetArgIntegerDefault(1, g_deye_port);
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman port=%i", g_deye_port);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "serial") || !stricmp(sub, "sn")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_logger_serial = Deye_ParseU32(Tokenizer_GetArg(1), g_deye_logger_serial);
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman logger serial=%u", (unsigned int)g_deye_logger_serial);
		Deye_SetProductNameFromSN();
		g_deye_auto_enabled = 1;
		g_deye_fixed_enabled = 0;
		Deye_StartDiscover();
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "slave")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_slave_id = Tokenizer_GetArgIntegerRange(1, 1, 247);
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman slave=%i", g_deye_slave_id);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "reg") || !stricmp(sub, "register")) {
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		g_deye_register = Tokenizer_GetArgIntegerRange(1, 0, 65535);
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman register=%i", g_deye_register);
		return CMD_RES_OK;
	}
	if (!stricmp(sub, "period")) {
		int ms;
		if (Tokenizer_GetArgsCount() < 2) return CMD_RES_NOT_ENOUGH_ARGUMENTS;
		ms = Tokenizer_GetArgIntegerDefault(1, g_deye_update_seconds * 1000);
		if (ms < 2000) ms = 2000;
		g_deye_update_seconds = (ms + 999) / 1000;
		ADDLOG_INFO(LOG_FEATURE_CMD, "DeyeSolarman period=%i seconds", g_deye_update_seconds);
		return CMD_RES_OK;
	}

	ADDLOG_ERROR(LOG_FEATURE_CMD, "DeyeSolarman unknown subcommand %s. Use: Deye setup <SN> [targetW min%% max%% maxPowerW ip], save, status|probe|send|discover|ip|name|pct|enable|auto|autodetect|rated|wperpct|target|min|max|gain|step|deadband|gridch|invert|port|slave|reg|period", sub);
	return CMD_RES_UNKNOWN_COMMAND;
}

void DeyeSolarman_Init(void) {
	int pinsChanged = 0;
	AgriHTLicense_Init();
	Deye_SetProductNameFromSN();
	if (PIN_GetPinRoleForPinIndex(6) == IOR_None) {
		PIN_SetPinRoleForPinIndex(6, IOR_LED_WIFI_n);
		pinsChanged = 1;
	}
	if (PIN_GetPinRoleForPinIndex(8) == IOR_None) {
		PIN_SetPinRoleForPinIndex(8, IOR_Button);
		PIN_SetPinChannelForPinIndex(8, CHANNEL_MAX - 1);
		pinsChanged = 1;
	}
	if (pinsChanged) {
		CFG_Save_IfThereArePendingChanges();
		ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman PM01 pins configured: P6=WifiLED_n P8=Btn");
	}
	CMD_RegisterCommand("Deye", Deye_CMD_Main, NULL);
	CMD_RegisterCommand("DeyeSetup", Deye_CMD_Setup, NULL);
	CMD_RegisterCommand("DeyeSave", Deye_CMD_Save, NULL);
	CMD_RegisterCommand("DeyeIP", Deye_CMD_SetIP, NULL);
	CMD_RegisterCommand("DeyePort", Deye_CMD_SetPort, NULL);
	CMD_RegisterCommand("DeyeSN", Deye_CMD_SetLoggerSerial, NULL);
	CMD_RegisterCommand("DeyeSlave", Deye_CMD_SetSlave, NULL);
	CMD_RegisterCommand("DeyeReg", Deye_CMD_SetRegister, NULL);
	CMD_RegisterCommand("DeyePct", Deye_CMD_SetFixedPercent, NULL);
	CMD_RegisterCommand("DeyePeriod", Deye_CMD_SetPeriodMs, NULL);
	CMD_RegisterCommand("DeyeOn", Deye_CMD_EnableFixed, NULL);
	CMD_RegisterCommand("DeyeSend", Deye_CMD_SendFixed, NULL);
	CMD_RegisterCommand("DeyeRead40", Deye_CMD_ReadR40, NULL);
	CMD_RegisterCommand("DeyeVerify40", Deye_CMD_VerifyR40, NULL);
	CMD_RegisterCommand("DeyeProbe", Deye_CMD_Probe, NULL);
	CMD_RegisterCommand("DeyeStatus", Deye_CMD_Status, NULL);
	CMD_RegisterCommand("AgriHTActivate", Deye_CMD_Activate, NULL);
	CMD_RegisterCommand("Deye_SetIP", Deye_CMD_SetIP, NULL);
	CMD_RegisterCommand("Deye_SetPort", Deye_CMD_SetPort, NULL);
	CMD_RegisterCommand("Deye_SetLoggerSerial", Deye_CMD_SetLoggerSerial, NULL);
	CMD_RegisterCommand("Deye_SetSlave", Deye_CMD_SetSlave, NULL);
	CMD_RegisterCommand("Deye_SetRegister", Deye_CMD_SetRegister, NULL);
	CMD_RegisterCommand("Deye_SetFixedPercent", Deye_CMD_SetFixedPercent, NULL);
	CMD_RegisterCommand("Deye_SetPeriodMs", Deye_CMD_SetPeriodMs, NULL);
	CMD_RegisterCommand("Deye_EnableFixed", Deye_CMD_EnableFixed, NULL);
	CMD_RegisterCommand("Deye_SendFixed", Deye_CMD_SendFixed, NULL);
	CMD_RegisterCommand("Deye_Probe", Deye_CMD_Probe, NULL);
	CMD_RegisterCommand("Deye_Status", Deye_CMD_Status, NULL);
	HTTP_RegisterCallback("/deye", HTTP_GET, Deye_HTTP_GetJson, 1);
	HTTP_RegisterCallback("/zn-discover", HTTP_GET, Deye_HTTP_Discover, 1);
	ADDLOG_INFO(LOG_FEATURE_DRV, "DeyeSolarman initialized");
}

void DeyeSolarman_RunEverySecond(void) {
	if (!AgriHTLicense_IsActive()) return;
	if (g_deye_verify_seconds > 0) {
		g_deye_verify_seconds--;
		if (g_deye_verify_seconds == 0) {
			int rc;
			int value = -1;
			int gridAfterW = Deye_ReadGridPowerW();
			rc = Deye_ReadRegister40(&value);
			if (rc == 0) g_deye_verify_readback = value;
			ADDLOG_INFO(LOG_FEATURE_DRV,
				"DeyeSolarman VERIFY result expected=%i%% readback=%i%% match=%i gridBefore=%iW gridAfter60s=%iW deltaGrid=%iW rc=%i (%s)",
				g_deye_verify_expected, value, rc == 0 && value == g_deye_verify_expected,
				g_deye_verify_grid_before_w, gridAfterW, gridAfterW - g_deye_verify_grid_before_w,
				rc, Deye_RcText(rc));
			ADDLOG_INFO(LOG_FEATURE_DRV,
				"DeyeSolarman VERIFY note: correlate inverter/app F07/F13 timestamp with this 60s test; no unverified alarm register is read");
		}
		return;
	}
	if (g_deye_discover_active) {
		if (g_deye_discover_saved_retry_seconds > 0) {
			g_deye_discover_saved_retry_seconds--;
		}
		if (g_deye_discover_seconds_until_next > 0) {
			g_deye_discover_seconds_until_next--;
		} else {
			Deye_RunDiscoverStep();
			g_deye_discover_seconds_until_next = 1;
		}
	}
	if (!g_deye_fixed_enabled && !g_deye_auto_enabled) return;
	if (g_deye_seconds_until_send > 0) {
		g_deye_seconds_until_send--;
		return;
	}
	if (g_deye_auto_enabled) {
		Deye_RunAutoControl();
	} else {
		Deye_SendPercent(g_deye_fixed_percent);
	}
	g_deye_seconds_until_send = g_deye_update_seconds - 1;
}

void DeyeSolarman_Stop(void) {
	g_deye_fixed_enabled = 0;
	g_deye_auto_enabled = 0;
	g_deye_discover_active = 0;
}

static void Deye_HTTP_PrintCss(http_request_t *request) {
	poststr(request, "<style>");
	poststr(request, "body{margin:0;background:#eef5ff;color:#101828;font-family:Arial,Helvetica,sans-serif}#main{max-width:920px;margin:0 auto;padding:14px}.zetop{display:flex;justify-content:space-between;align-items:center;margin:6px 0 12px}.zetop h1{font-size:20px;margin:0;color:#0f172a}.zetop a{color:#2563eb;text-decoration:none}");
	poststr(request, ".ze{border:1px solid #bfdbfe;border-radius:8px;padding:16px;margin:8px 0 14px;background:#f8fbff;color:#101828;box-shadow:0 6px 20px #0f172a18}");
	poststr(request, ".zehead{display:flex;justify-content:space-between;gap:10px;align-items:flex-start}.ze h2{margin:0 0 3px;font-size:21px}.ze small{color:#475467}");
	poststr(request, ".zebadge,.zeflow{border-radius:999px;padding:6px 11px;font-size:12px;font-weight:800;white-space:nowrap}.zebadge{background:#e0f2fe}.zeflow{display:inline-block;margin-top:8px;background:#eef2ff}");
	poststr(request, ".zegrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(110px,1fr));gap:9px;margin-top:12px}.zem{background:#fff;border:1px solid #dbeafe;border-radius:7px;padding:10px}.zel{font-size:11px;color:#667085;text-transform:uppercase}.zev{font-size:19px;font-weight:800}.zebig .zev{font-size:26px}");
	poststr(request, ".zeok,.zeimport{color:#047857}.zeerr,.zeexport{color:#b42318}.zewait,.zehold{color:#b54708}.zef{display:grid;grid-template-columns:repeat(auto-fit,minmax(135px,1fr));gap:8px;margin-top:14px}");
	poststr(request, ".zef label{font-size:12px;font-weight:700;color:#344054}.zef input,.zef select{width:100%;box-sizing:border-box;margin-top:3px;padding:7px;border-radius:6px;border:1px solid #cbd5e1;background:#fff}");
	poststr(request, ".zeb{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}.zeb button,.zeb a{padding:9px 12px;border-radius:6px;border:1px solid #94a3b8;background:#fff;color:#111827;text-decoration:none;font-size:14px}.zeb button:first-child{background:#166534;color:#fff;border-color:#166534}.zest{font-size:12px;color:#475467;margin-top:8px}.zewm{text-align:center;border-top:1px solid #dbeafe;margin-top:14px;padding-top:10px;font-size:11px;color:#64748b}</style>");
}

static void Deye_HTTP_PrintMetric(http_request_t *request, const char *id, const char *label, const char *fmt, int value, int big) {
	hprintf255(request, "<div class='zem%s'><div class='zel'>%s</div><div class='zev'>", big ? " zebig" : "", label);
	if (id) hprintf255(request, "<span id='%s'>", id);
	hprintf255(request, fmt, value);
	if (id) poststr(request, "</span>");
	poststr(request, "</div></div>");
}

static void Deye_HTTP_PrintSetupForm(http_request_t *request) {
	hprintf255(request, "<div class='zef'><label>Device ID<input id='ze_device' value='%s' readonly></label>",
		AgriHTLicense_GetDeviceId());
	poststr(request, "<label>Activation Key<input id='ze_key' maxlength='16' placeholder='16-character key'></label></div>");
	poststr(request, "<div class='zeb'><button onclick='zeActivate()'>Activate &amp; Save</button></div>");
	poststr(request, "<div class='zef'>");
	hprintf255(request, "<label>Logger SN<input id='ze_sn' value='%u'></label>", (unsigned int)g_deye_logger_serial);
	hprintf255(request, "<label>Deye IP optional<input id='ze_dip' value='%s'></label>", g_deye_ip_confirmed ? g_deye_ip : "");
	hprintf255(request, "<label>Import high W<input id='ze_target' value='%i'></label>", g_deye_target_grid_w);
	hprintf255(request, "<label>Export low W<input id='ze_export' value='%i'></label>", g_deye_export_limit_w);
	hprintf255(request, "<label>Max Power W<input id='ze_rated' value='%i'></label>", g_deye_rated_w);
	hprintf255(request, "<label>Min %%<input id='ze_min' type='number' min='0' max='100' value='%i'></label>", g_deye_min_percent);
	hprintf255(request, "<label>Max %%<input id='ze_max' value='%i'></label>", g_deye_max_percent);
	hprintf255(request, "<label>Update ms<input id='ze_period' type='number' min='2000' max='60000' step='1000' value='%i'></label>", g_deye_update_seconds * 1000);
	hprintf255(request, "<label>Đảo chiều CT<select id='ze_invert'><option value='0'%s>Không</option><option value='1'%s>Có</option></select></label>",
		g_deye_grid_invert ? "" : " selected", g_deye_grid_invert ? " selected" : "");
	poststr(request, "</div><div class='zeb'>");
	poststr(request, "<button onclick='zeApply(0)'>Apply now</button>");
	poststr(request, "<button onclick='zeApply(1)'>Save startup</button>");
	poststr(request, "<button onclick='zeCmd(\"Deye discover\")'>Find Deye IP</button>");
	poststr(request, "<button onclick='zeCmd(\"Deye status\")'>Log status</button>");
	poststr(request, "<a href='/cfg_wifi'>Cài WiFi</a>");
	poststr(request, "<a href='/ota'>Cập nhật firmware</a>");
	poststr(request, "</div><div id='ze_msg' class='zest'></div>");
	poststr(request, "<script>");
	poststr(request, "function zeVal(i){return document.getElementById(i).value}");
	poststr(request, "function zeSet(i,v){var e=document.getElementById(i);if(e)e.innerText=v}");
	poststr(request, "function zeCmd(c){fetch('/cm?cmnd='+encodeURIComponent(c)).then(r=>r.text()).then(t=>{ze_msg.innerText=c+' OK';zePoll()}).catch(e=>{ze_msg.innerText=e})}");
	poststr(request, "function zeActivate(){var k=zeVal('ze_key').trim();fetch('/cm?cmnd='+encodeURIComponent('AgriHTActivate '+k)).then(r=>r.text()).then(t=>{ze_msg.innerText=t.indexOf('Error')>=0?'Activation failed':'Activated and saved';zePoll()}).catch(e=>{ze_msg.innerText=e})}");
	poststr(request, "function zeApply(s){var ip=zeVal('ze_dip')||'0';var c='Deye setup '+zeVal('ze_sn')+' '+zeVal('ze_target')+' '+zeVal('ze_min')+' '+zeVal('ze_max')+' '+zeVal('ze_rated')+' '+ip+' 100 5 0 '+zeVal('ze_period')+' '+zeVal('ze_export')+' 0 '+zeVal('ze_invert');");
	poststr(request, "fetch('/cm?cmnd='+encodeURIComponent(c)).then(()=>s?fetch('/cm?cmnd='+encodeURIComponent('Deye save')):0).then(()=>{ze_msg.innerText=s?'Saved':'Applied';zePoll()})}");
	poststr(request, "function zePoll(){fetch('/deye').then(r=>r.json()).then(j=>{");
	poststr(request, "zeSet('ze_grid',j.grid+' W');zeSet('ze_target_m',j.target+' W');zeSet('ze_limit',j.limit+' %');zeSet('ze_v',j.voltage+' V');zeSet('ze_i',j.current+' mA');");
	poststr(request, "zeSet('ze_app',j.apparent+' VA');zeSet('ze_freq',j.freq+' Hz');zeSet('ze_pf',j.pf);zeSet('ze_rated_m',j.rated+' W');zeSet('ze_link',j.link);zeSet('ze_badge',j.link);zeSet('ze_flow',j.flow);zeSet('ze_ip',j.ip+':'+j.port);zeSet('ze_lastrc',j.lastRc+' '+j.lastText);zeSet('ze_sends',j.sends);zeSet('ze_fail',j.fail);");
	poststr(request, "zeSet('ze_license',j.licensed?'ACTIVE':'LOCKED');");
	poststr(request, "var l=document.getElementById('ze_link'),b=document.getElementById('ze_badge'),f=document.getElementById('ze_flow');var c=j.link=='OK'?'zeok':(j.link=='ERROR'?'zeerr':'zewait');if(l)l.className=c;if(b)b.className='zebadge '+c;if(f)f.className='zeflow ze'+j.flow.toLowerCase()})}");
	poststr(request, "setInterval(zePoll,1000);zePoll();");
	poststr(request, "</script>");
}

static void Deye_HTTP_PrintPanel(http_request_t *request) {
	Deye_HTTP_PrintCss(request);
	poststr(request, "<section class='ze'><div class='zehead'><div><h2>AgriHT Zero Export</h2><small>PM01 + Deye Solarman V5 load following</small><br>");
	hprintf255(request, "<span id='ze_flow' class='zeflow ze%s'>%s</span></div>", Deye_FlowText(), Deye_FlowText());
	hprintf255(request, "<div id='ze_badge' class='zebadge'>%s</div></div><div class='zegrid'>", Deye_LinkText());
	hprintf255(request, "<div class='zem'><div class='zel'>License</div><div class='zev'><span id='ze_license'>%s</span></div></div>",
		AgriHTLicense_IsActive() ? "ACTIVE" : "LOCKED");
	Deye_HTTP_PrintMetric(request, "ze_grid", "Grid Active", "%i W", g_deye_last_grid_w, 1);
	Deye_HTTP_PrintMetric(request, "ze_target_m", "Target", "%i W", g_deye_target_grid_w, 0);
	Deye_HTTP_PrintMetric(request, "ze_limit", "Limit", "%i %%", g_deye_auto_percent, 1);
	Deye_HTTP_PrintMetric(request, "ze_rated_m", "Max Power", "%i W", g_deye_rated_w, 0);
	Deye_HTTP_PrintMetric(request, "ze_v", "Voltage", "%i V", g_deye_last_voltage_v, 0);
	Deye_HTTP_PrintMetric(request, "ze_i", "Current", "%i mA", g_deye_last_current_ma, 0);
	Deye_HTTP_PrintMetric(request, "ze_app", "Apparent", "%i VA", g_deye_last_apparent_w, 0);
	hprintf255(request, "<div class='zem'><div class='zel'>Frequency</div><div class='zev'><span id='ze_freq'>%.2f Hz</span></div></div>", g_deye_last_freq_hz);
	hprintf255(request, "<div class='zem'><div class='zel'>Power Factor</div><div class='zev'><span id='ze_pf'>%.3f</span></div></div>", g_deye_last_pf);
	hprintf255(request, "<div class='zem'><div class='zel'>Deye Link</div><div class='zev'><span id='ze_link'>%s</span></div></div>", Deye_LinkText());
	poststr(request, "</div>");
	hprintf255(request, "<div class='zest'>Deye <span id='ze_ip'>%s:%i</span> SN %u, auto=%i, ipOK=%i, discover=%i, chP=%i chV=%i chI=%i, last=<span id='ze_lastrc'>%i %s</span>, sends=<span id='ze_sends'>%u</span>, fail=<span id='ze_fail'>%i</span></div>",
		g_deye_ip, g_deye_port, (unsigned int)g_deye_logger_serial,
		g_deye_auto_enabled, g_deye_ip_confirmed, g_deye_discover_active,
		g_deye_grid_channel, g_deye_voltage_channel, g_deye_current_channel,
		g_deye_last_rc, Deye_RcText(g_deye_last_rc), g_deye_send_count,
		g_deye_fail_count);
	Deye_HTTP_PrintSetupForm(request);
	poststr(request, "<div class='zewm'>Made by AgriHT &copy; 2026</div>");
	poststr(request, "</section>");
}

static int Deye_HTTP_GetJson(http_request_t *request) {
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"grid\":%i,\"target\":%i,\"exportLimit\":%i,\"limit\":%i,\"voltage\":%i,\"current\":%i,",
		g_deye_last_grid_w, g_deye_target_grid_w, g_deye_export_limit_w, g_deye_auto_percent,
		g_deye_last_voltage_v, g_deye_last_current_ma);
	hprintf255(request, "\"apparent\":%i,\"lastPercent\":%i,\"rated\":%i,\"freq\":%.2f,\"pf\":%.3f,",
		g_deye_last_apparent_w, g_deye_last_percent, g_deye_rated_w, g_deye_last_freq_hz, g_deye_last_pf);
	hprintf255(request, "\"lastRc\":%i,\"sends\":%u,\"sn\":%u,\"fail\":%i,",
		g_deye_last_rc, g_deye_send_count, (unsigned int)g_deye_logger_serial,
		g_deye_fail_count);
	hprintf255(request, "\"port\":%i,\"auto\":%i,\"ipOK\":%i,\"discover\":%i,",
		g_deye_port, g_deye_auto_enabled, g_deye_ip_confirmed, g_deye_discover_active);
	hprintf255(request, "\"gain\":%i,\"step\":%i,\"deadband\":%i,\"periodMs\":%i,",
		g_deye_gain_percent, g_deye_max_step_percent, g_deye_deadband_w,
		g_deye_update_seconds * 1000);
	hprintf255(request, "\"invert\":%i,\"licensed\":%i,\"deviceId\":\"%s\",\"ip\":\"%s\",\"link\":\"%s\",\"flow\":\"%s\",\"lastText\":\"%s\"}",
		g_deye_grid_invert,
		AgriHTLicense_IsActive(), AgriHTLicense_GetDeviceId(), g_deye_ip,
		Deye_LinkText(), Deye_FlowText(), Deye_RcText(g_deye_last_rc));
	poststr(request, NULL);
	return 0;
}

static int Deye_HTTP_Discover(http_request_t *request) {
	http_setup(request, httpMimeTypeJson);
	hprintf255(request,
		"{\"product\":\"AgriHT-ZN\",\"name\":\"%s\",\"deviceId\":\"%s\","
		"\"sn\":%u,\"licensed\":%i,\"link\":\"%s\",\"moduleIp\":\"%s\"}",
		CFG_GetShortDeviceName(), AgriHTLicense_GetDeviceId(),
		(unsigned int)g_deye_logger_serial, AgriHTLicense_IsActive(),
		Deye_LinkText(), HAL_GetMyIPString());
	poststr(request, NULL);
	return 0;
}

void DeyeSolarman_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState) {
	if (!bPreState) return;
	Deye_HTTP_PrintPanel(request);
}

int DeyeSolarman_HTTPIndexPage(http_request_t *request) {
	http_setup(request, httpMimeTypeHTML);
	poststr(request, "<!DOCTYPE html><html><head><title>AgriHT Zero Export</title>");
	poststr(request, "<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'><meta name='robots' content='none'>");
	poststr(request, "</head><body><div id='main'>");
	poststr(request, "<div class='zetop'><h1>AgriHT Zero Export</h1><a href='/cfg_wifi'>Cài WiFi</a></div>");
	Deye_HTTP_PrintPanel(request);
	poststr(request, "<div class='zeb'><a href='/cfg_wifi'>Đổi WiFi</a><a href='/ota'>OTA firmware</a><a href='/index?restart=1' onclick=\"return confirm('Khởi động lại thiết bị?')\">Khởi động lại</a></div>");
	poststr(request, "</div></body></html>");
	poststr(request, NULL);
	return 0;
}
