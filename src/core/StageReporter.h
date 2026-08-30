#ifndef USV_STAGE_REPORTER_H
#define USV_STAGE_REPORTER_H

#include <stdint.h>

#include "Packets.h"
#include "core/MissionFsm.h"

struct StageReportContext {
    BoatState previousState;
    BoatState currentState;
    FsmInputs fsm;
    bool      initialReport;
    bool      manualMode;
    bool      uploadComplete;
    bool      homeSet;
    uint8_t   waypointCount;
    uint8_t   waypointIndex;
    uint8_t   expectedBatchIndex;
};

class StageReporter {
public:
    static StageStatusPacket build(const StageReportContext& ctx);

    static bool sameStatus(const StageStatusPacket& a,
                           const StageStatusPacket& b);

private:
    static uint8_t deriveIntent(const StageReportContext& ctx);
    static uint8_t deriveNextStage(const StageReportContext& ctx);
    static uint8_t deriveReason(const StageReportContext& ctx);
    static uint8_t deriveFlags(const StageReportContext& ctx);
};

#endif // USV_STAGE_REPORTER_H
