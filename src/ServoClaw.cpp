#include "ServoClaw.h"
#include "BoardPins.h"

void ServoClaw::begin() {
    if (_attached) return;

    // Servo::attach() maps 0..180 deg onto the default 1000..2000 us pulses.
    _left.attach(SERVO_CLAW_LEFT_PIN);
    _right.attach(SERVO_CLAW_RIGHT_PIN);
    _pickupServo.attach(SERVO_PICKUP_PIN);
    _attached = _left.attached() && _right.attached() && _pickupServo.attached();

    // Deterministic power-on pose: jaws closed, pickup arm stowed up.
    close();
    drop();
}

void ServoClaw::open() {
    writeAngle(SERVO_LEFT, _openL);
    writeAngle(SERVO_RIGHT, _openR);
}

void ServoClaw::close() {
    writeAngle(SERVO_LEFT, _closeL);
    writeAngle(SERVO_RIGHT, _closeR);
}

void ServoClaw::pickup() {
    writeAngle(SERVO_PICKUP, _pick);
}

void ServoClaw::drop() {
    writeAngle(SERVO_PICKUP, _drop);
}

void ServoClaw::resetDefaults() {
    _openL  = DEF_OPEN_L;
    _openR  = DEF_OPEN_R;
    _closeL = DEF_CLOSE_L;
    _closeR = DEF_CLOSE_R;
    _pick   = DEF_PICKUP;
    _drop   = DEF_DROP;
}

void ServoClaw::writeAngle(int servo1Based, int deg) {
    deg = clampDeg(deg);
    switch (servo1Based) {
        case SERVO_LEFT:   _left.write(deg);        _curL = deg; break;
        case SERVO_RIGHT:  _right.write(deg);       _curR = deg; break;
        case SERVO_PICKUP: _pickupServo.write(deg); _curP = deg; break;
        default: break;
    }
}

bool ServoClaw::moveRaw(int servo1Based, int deg) {
    if (servo1Based < SERVO_LEFT || servo1Based > SERVO_PICKUP) return false;
    writeAngle(servo1Based, deg);
    return true;
}

int ServoClaw::angleOf(int servo1Based) const {
    switch (servo1Based) {
        case SERVO_LEFT:   return _curL;
        case SERVO_RIGHT:  return _curR;
        case SERVO_PICKUP: return _curP;
        default:           return -1;
    }
}

void ServoClaw::detachAll() {
    _seqStage = SEQ_IDLE;              // kill any running scoop sequence
    _piL = _piR = -1;
    _left.detach();
    _right.detach();
    _pickupServo.detach();
    _attached = false;
}

int ServoClaw::stepToward(int cur, int tgt, int step) {
    if (cur < tgt) {
        int d = tgt - cur;
        return (d < step) ? d : step;
    }
    if (cur > tgt) {
        int d = cur - tgt;
        return (d < step) ? -d : -step;
    }
    return 0;
}

void ServoClaw::startSeq(int piLeftDeg, int piRightDeg) {
    _seqDonePending = false;
    _piL = clampDeg(piLeftDeg);
    _piR = clampDeg(piRightDeg);
    // Step 1: full close. The remaining stages run out of service().
    _seqStage = SEQ_STAGE_CLOSE;
    _seqStageMs = millis();
    close();
}

void ServoClaw::service() {
    if (_seqStage == SEQ_IDLE) return;
    unsigned long now = millis();

    switch (_seqStage) {
        case SEQ_STAGE_CLOSE:
            if (now - _seqStageMs < SEQ_CLOSE_MS) return;
            if (_piL >= 0) {                    // Pi-supplied fine grip angle(s)
                writeAngle(SERVO_LEFT, _piL);
                _piR = (_piR >= 0) ? _piR : _curR;
                writeAngle(SERVO_RIGHT, _piR);
                _seqStage = SEQ_STAGE_ANGLE;
            } else {
                _seqStage = SEQ_STAGE_PICKUP;   // no Pi info: skip the adjust
            }
            _seqStageMs = now;
            return;

        case SEQ_STAGE_ANGLE:
            if (now - _seqStageMs < SEQ_ANGLE_MS) return;
            pickup();
            _seqStage = SEQ_STAGE_PICKUP;
            _seqStageMs = now;
            return;

        case SEQ_STAGE_PICKUP:
            if (now - _seqStageMs < SEQ_PICK_MS) return;
            _seqStage = SEQ_STAGE_OPEN;
            _seqStageMs = now;
            return;

        case SEQ_STAGE_OPEN: {
            // Creep the jaws open one small step at a time ("open carefully").
            if (now - _seqStageMs < SEQ_OPEN_MS) return;
            _seqStageMs = now;
            int dl = stepToward(_curL, _openL, SEQ_OPEN_STEP);
            int dr = stepToward(_curR, _openR, SEQ_OPEN_STEP);
            if (dl == 0 && dr == 0) {           // fully open: sequence complete
                _seqStage = SEQ_IDLE;
                _piL = _piR = -1;
                _seqDonePending = true;
                return;
            }
            if (dl) writeAngle(SERVO_LEFT, _curL + dl);
            if (dr) writeAngle(SERVO_RIGHT, _curR + dr);
            return;
        }

        default:
            _seqStage = SEQ_IDLE;
    }
}

const char *ServoClaw::seqStageName() const {
    switch (_seqStage) {
        case SEQ_STAGE_CLOSE:  return "CLOSE";
        case SEQ_STAGE_ANGLE:  return "ANGLE";
        case SEQ_STAGE_PICKUP: return "PICKUP";
        case SEQ_STAGE_OPEN:   return "OPEN";
        default:               return "IDLE";
    }
}

bool ServoClaw::pollSeqDone() {
    bool done = _seqDonePending;
    _seqDonePending = false;
    return done;
}

void ServoClaw::printStatus(Stream &s) const {
    s.print(F("CLAW: attached="));
    s.print(_attached ? 1 : 0);
    s.print(F(" open=(L="));
    s.print(_openL);
    s.print(F(",R="));
    s.print(_openR);
    s.print(F(") close=(L="));
    s.print(_closeL);
    s.print(F(",R="));
    s.print(_closeR);
    s.print(F(") pickup="));
    s.print(_pick);
    s.print(F(" drop="));
    s.print(_drop);
    if (_seqStage != SEQ_IDLE) {           // scoop sequence in progress
        s.print(F(" seq="));
        s.print(seqStageName());
    }
    s.print(F(" | now=(L="));
    s.print(_curL);
    s.print(F(",R="));
    s.print(_curR);
    s.print(F(",P="));
    s.print(_curP);
    s.println(')');
    // Paste-ready initialisers for ServoClaw.h (tune, then bake in).
    s.print(F("# defaults: OPEN_L="));
    s.print(_openL);
    s.print(F(" OPEN_R="));
    s.print(_openR);
    s.print(F(" CLOSE_L="));
    s.print(_closeL);
    s.print(F(" CLOSE_R="));
    s.print(_closeR);
    s.print(F(" PICKUP="));
    s.print(_pick);
    s.print(F(" DROP="));
    s.println(_drop);
}
