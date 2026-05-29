#include "node_time.h"

#include "AB1805_RK.h"

namespace {

bool timeProvisional = true;
bool authoritativeAckSeenThisBoot = false;
time_t lastGatewayAckUtc = 0;
time_t lastNodeUtcBeforeAck = 0;
int32_t lastAckDriftSeconds = 0;

} // anonymous namespace

void nodeTimeNoteBootRtcState(bool rtcRestored) {
	timeProvisional = true;
	authoritativeAckSeenThisBoot = false;
	lastGatewayAckUtc = 0;
	lastNodeUtcBeforeAck = 0;
	lastAckDriftSeconds = 0;
	Log.info("TimeSync: rtcBoot=%s provisional=1", rtcRestored ? "ok" : "fail");
}

bool nodeTimeApplyGatewayAck(time_t epoch, AB1805 &rtc, const char *context) {
	if (!nodeTimeIsSaneEpoch(epoch)) {
		(void)context;
		Log.info("TimeSync: ack invalid action=ignored");
		return false;
	}

	int32_t driftSeconds = 0;
	time_t nodeBefore = 0;
	if (Time.isValid()) {
		nodeBefore = Time.now();
		driftSeconds = (int32_t)(epoch - nodeBefore);
	}
	lastGatewayAckUtc = epoch;
	lastNodeUtcBeforeAck = nodeBefore;
	lastAckDriftSeconds = driftSeconds;

	Time.setTime(epoch);
	bool rtcUpdated = rtc.setRtcFromTime(epoch);
	timeProvisional = false;
	authoritativeAckSeenThisBoot = true;

	Log.info(
		"TimeSync: ack epoch=%lu drift=%ld rtc=%s context=%s",
		(unsigned long)epoch,
		(long)driftSeconds,
		rtcUpdated ? "updated" : "failed",
		context ? context : "ack");

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

time_t nodeTimeLastGatewayAckUtc() {
	return lastGatewayAckUtc;
}

time_t nodeTimeLastNodeUtcBeforeAck() {
	return lastNodeUtcBeforeAck;
}

int32_t nodeTimeLastAckDriftSeconds() {
	return lastAckDriftSeconds;
}