#include "../hal/hal_wifi.h"
#include "../new_common.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_public.h"
#include "agriht_license.h"

/* Keep synchronized with tools/generate_license.py in the private repo. */
static const byte g_license_secret[16] = {
	0xA1, 0x42, 0xAE, 0xAA, 0x6D, 0x61, 0xF6, 0x76,
	0xB6, 0x84, 0xDA, 0xBA, 0x8F, 0x43, 0x72, 0x80
};
static char g_device_id[13];
static char g_license_key[AGRIHT_LICENSE_KEY_HEX_LEN + 1];
static int g_license_active;

static uint64_t rotl64(uint64_t v, int n) { return (v << n) | (v >> (64 - n)); }
static uint64_t load64le(const byte *p) {
	return ((uint64_t)p[0]) | ((uint64_t)p[1] << 8) | ((uint64_t)p[2] << 16) |
		((uint64_t)p[3] << 24) | ((uint64_t)p[4] << 32) | ((uint64_t)p[5] << 40) |
		((uint64_t)p[6] << 48) | ((uint64_t)p[7] << 56);
}
static void sipround(uint64_t *v0, uint64_t *v1, uint64_t *v2, uint64_t *v3) {
	*v0 += *v1; *v1 = rotl64(*v1, 13); *v1 ^= *v0; *v0 = rotl64(*v0, 32);
	*v2 += *v3; *v3 = rotl64(*v3, 16); *v3 ^= *v2;
	*v0 += *v3; *v3 = rotl64(*v3, 21); *v3 ^= *v0;
	*v2 += *v1; *v1 = rotl64(*v1, 17); *v1 ^= *v2; *v2 = rotl64(*v2, 32);
}
static uint64_t siphash24(const byte *data, int len) {
	uint64_t k0 = load64le(g_license_secret), k1 = load64le(g_license_secret + 8);
	uint64_t v0 = 0x736f6d6570736575ULL ^ k0, v1 = 0x646f72616e646f6dULL ^ k1;
	uint64_t v2 = 0x6c7967656e657261ULL ^ k0, v3 = 0x7465646279746573ULL ^ k1;
	uint64_t tail = ((uint64_t)len) << 56;
	int i;
	while (len >= 8) {
		uint64_t m = load64le(data);
		v3 ^= m; sipround(&v0, &v1, &v2, &v3); sipround(&v0, &v1, &v2, &v3); v0 ^= m;
		data += 8; len -= 8;
	}
	for (i = 0; i < len; i++) tail |= ((uint64_t)data[i]) << (8 * i);
	v3 ^= tail; sipround(&v0, &v1, &v2, &v3); sipround(&v0, &v1, &v2, &v3); v0 ^= tail;
	v2 ^= 0xff;
	for (i = 0; i < 4; i++) sipround(&v0, &v1, &v2, &v3);
	return v0 ^ v1 ^ v2 ^ v3;
}

#if defined(PLATFORM_BK7231T) || defined(PLATFORM_BK7231N) || defined(PLATFORM_BK7238) || defined(PLATFORM_BK7252) || defined(WINDOWS)
extern unsigned int flash_ctrl(unsigned int cmd, void *parm);
#define CMD_FLASH_GET_UID 54
typedef struct {
    unsigned char *buf;
    unsigned int addr;
    unsigned int len;
} temp_flash_otp_t;

static commandResult_t CMD_GetFlashUID(const void* context, const char* cmd, const char* args, int cmdFlags) {
	unsigned char uid[16] = {0};
	temp_flash_otp_t param;
	param.buf = uid;
	param.addr = 0;
	param.len = 16;
	unsigned int ret = flash_ctrl(CMD_FLASH_GET_UID, &param);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "Flash UID ret: %u, uid: %02X%02X%02X%02X", ret, uid[0], uid[1], uid[2], uid[3]);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "UID[4-7]: %02X%02X%02X%02X", uid[4], uid[5], uid[6], uid[7]);
	
	extern int wifi_get_mac_address_from_efuse(uint8_t *mac);
	uint8_t emac[6] = {0};
	int eret = wifi_get_mac_address_from_efuse(emac);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "eFuse ret: %d, MAC: %02X:%02X:%02X:%02X:%02X:%02X", eret, emac[0], emac[1], emac[2], emac[3], emac[4], emac[5]);
	
	return CMD_RES_OK;
}
#endif

void AgriHTLicense_Init(void) {
#if defined(PLATFORM_BK7231T) || defined(PLATFORM_BK7231N) || defined(PLATFORM_BK7238) || defined(PLATFORM_BK7252) || defined(WINDOWS)
	CMD_RegisterCommand("GetFlashUID", CMD_GetFlashUID, NULL);
#endif
	byte mac[6];
	int uid_valid = 0;
	uint8_t uid[16] = {0};

#if defined(PLATFORM_BK7231T) || defined(PLATFORM_BK7231N) || defined(PLATFORM_BK7238) || defined(PLATFORM_BK7252) || defined(WINDOWS)
	temp_flash_otp_t param;
	param.buf = uid;
	param.addr = 0;
	param.len = 16;
	if (flash_ctrl(CMD_FLASH_GET_UID, &param) == 16) {
		int i;
		for (i = 0; i < 16; i++) {
			if (uid[i] != 0x00 && uid[i] != 0xFF) {
				uid_valid = 1;
				break;
			}
		}
	}
	
	if (!uid_valid) {
		extern int wifi_get_mac_address_from_efuse(uint8_t *mac);
		uint8_t emac[6] = {0};
		if (wifi_get_mac_address_from_efuse(emac) > 0) {
			int i;
			for (i = 0; i < 6; i++) {
				if (emac[i] != 0x00 && emac[i] != 0xFF) {
					uid_valid = 1;
					break;
				}
			}
			if (uid_valid) {
				memcpy(uid, emac, 6);
				memset(uid + 6, 0, 10);
			}
		}
	}
#endif

	if (uid_valid) {
		uint64_t uid_hash = siphash24((const byte *)uid, 16);
		snprintf(g_device_id, sizeof(g_device_id), "%02X%02X%02X%02X%02X%02X",
			(unsigned int)((uid_hash >> 40) & 0xFF),
			(unsigned int)((uid_hash >> 32) & 0xFF),
			(unsigned int)((uid_hash >> 24) & 0xFF),
			(unsigned int)((uid_hash >> 16) & 0xFF),
			(unsigned int)((uid_hash >> 8) & 0xFF),
			(unsigned int)(uid_hash & 0xFF));
	} else {
		WiFI_GetMacAddress((char *)mac);
		snprintf(g_device_id, sizeof(g_device_id), "%02X%02X%02X%02X%02X%02X",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	}

	g_license_key[0] = 0;
	g_license_active = 0;
}
int AgriHTLicense_Activate(const char *key) {
	char expected[17];
	uint64_t tag;
	int diff = 0, i;
	if (!key || strlen(key) != 16) { g_license_active = 0; return 0; }
	tag = siphash24((const byte *)g_device_id, strlen(g_device_id));
	snprintf(expected, sizeof(expected), "%08X%08X", (unsigned int)(tag >> 32), (unsigned int)tag);
	for (i = 0; i < 16; i++) diff |= toupper((unsigned char)key[i]) ^ expected[i];
	g_license_active = diff == 0;
	if (g_license_active) snprintf(g_license_key, sizeof(g_license_key), "%s", key);
	return g_license_active;
}
int AgriHTLicense_IsActive(void) { return g_license_active; }
const char *AgriHTLicense_GetDeviceId(void) { return g_device_id; }
const char *AgriHTLicense_GetKey(void) { return g_license_key; }
