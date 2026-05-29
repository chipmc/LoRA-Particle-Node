#ifndef NODE_TIME_H
#define NODE_TIME_H

#include "Particle.h"

class AB1805;

constexpr time_t NODE_TIME_MIN_VALID_EPOCH = 1577836800; // 2020-01-01T00:00:00Z
constexpr time_t NODE_TIME_MAX_VALID_EPOCH = 2145916800; // 2038-01-01T00:00:00Z

void nodeTimeNoteBootRtcState(bool rtcRestored);
bool nodeTimeApplyGatewayAck(time_t epoch, AB1805 &rtc, const char *context = nullptr);
bool nodeTimeHasAuthoritativeAck();
bool nodeTimeIsProvisional();
bool nodeTimeIsSaneEpoch(time_t epoch);
time_t nodeTimeLastGatewayAckUtc();
time_t nodeTimeLastNodeUtcBeforeAck();
int32_t nodeTimeLastAckDriftSeconds();

#endif