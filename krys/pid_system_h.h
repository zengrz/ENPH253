#ifndef PID_SYSTEM_H_
#define PID_SYSTEM_H_

#include "configuration_h.h" // for type definition (uint16 etc.)

namespace PID
{
    //PID TAPE
    extern float QRDLastError;
    extern uint16 QRDErrorTime;
    void QRDUpdateLastError();
    void QRDReset();
    float GetQRDError();
    int16 GetQRDAdjustment();
    void QRDUpdate(int vel = 0);

    //PID OP
    extern float OPLastError, OPTotalError;
    extern uint16 OPMaxIntegral;
    void OPReset();
    void ResetOPOffsets();
    int16 GetOPError(bool front, int16 lastError,  float rearLeftOPOffsetGain, float rearRightOPOffsetGain);
    int16 GetOPAdjustment(bool front, uint16 diff, float p, float i, float d, float rearLeftOPOffsetGain, float rearRightOPOffsetGain);
    void OPUpdate(bool frontOP, uint16 diff, float p, float i, float d, int16 vel = 0,  float rearLeftOPOffsetGain = 1.0f, float rearRightOPOffsetGain = 1.0f);
    bool TargOPAdjustCompleted();
    bool RearOPAdjustCompleted(float leftGain = 1, float rightGain = 1);

    void OPDriveUpdate(uint16 diff, uint16 goalRearIRThresh);
    bool OPDriveAdjustCompleted(uint16 goalRearIRThresh, float leftGain = 1, float rightGain = 1);
}
#endif