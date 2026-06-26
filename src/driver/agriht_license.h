#ifndef __AGRIHT_LICENSE_H__
#define __AGRIHT_LICENSE_H__

#define AGRIHT_LICENSE_KEY_HEX_LEN 16

void AgriHTLicense_Init(void);
int AgriHTLicense_Activate(const char *key);
int AgriHTLicense_IsActive(void);
const char *AgriHTLicense_GetDeviceId(void);
const char *AgriHTLicense_GetKey(void);

#endif
