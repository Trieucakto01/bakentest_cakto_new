#ifndef __DRV_DEYE_SOLARMAN_H__
#define __DRV_DEYE_SOLARMAN_H__

#include "../httpserver/new_http.h"

void DeyeSolarman_Init(void);
void DeyeSolarman_RunEverySecond(void);
void DeyeSolarman_Stop(void);
void DeyeSolarman_AppendInformationToHTTPIndexPage(http_request_t *request, int bPreState);
int DeyeSolarman_HTTPIndexPage(http_request_t *request);

#endif
