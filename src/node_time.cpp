#include "node_time.h"

#include "AB1805_RK.h"

namespace {

bool timeProvisional = true;
bool authoritativeAckSeenThisBoot = false;

} // anonymous namespace

void nodeTimeNoteBootRtcState(bool rtcRestored) {
	timeProvisional = true;
	authoritativeAckSeenThisBoot = false;
	Log.info("Time: rtc=%s provisional=1", rtcRestored ? "ok" : "fail");
}

bool nodeTimeApplyGatewayAck(time_t epoch, AB1805 &rtc, const char *context) {
	if (!nodeTimeIsSaneEpoch(epoch)) {
		(void)context;
		Log.info("Time: ack invalid action=ignored");
		return false;
	}

	int32_t driftSeconds = 0;
	if (Time.isValid()) {
		driftSeconds = (int32_t)(epoch - Time.now());
	}

	Time.setTime(epoch);
	bool rtcUpdated = rtc.setRtcFromTime(epoch);
	timeProvisional = false;
	authoritativeAckSeenThisBoot = true;

	Log.info(
		"Time: ack sync epoch=%lu drift=%ld rtc=%s",
		(unsigned long)epoch,
		(long)driftSeconds,
		rtcUpdated ? "updated" : "failed");

	return true;
}

bool nodeTimeHasAuthoritativeAck() {
	return authoritativeAckSeenThisBoot;
}

bool nodeTimeIsProvisional() {
	return timeProvisional;
}

bool nodeTimeIsSaneEpoch(time_t epoch) {
	return epoch >= NODE_TIME_MIN_VALID_EPOCH && epoch <= NODE_TIME_MAX_VALID_EPOCH;
}