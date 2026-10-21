#include "Commands.h"

#include <Arduino.h>
#include <math.h>

#include "Claw.h"
#include "Config.h"
#include "Lights.h"
#include "Motion.h"
#include "Motors.h"
#include "Telemetry.h"
#include "Tof.h"

namespace {

const int MAX_ARGS = 5;

// Parses a whole token as a number within [lo, hi].
bool number(const char *text, float lo, float hi, float &out) {
    char *end;
    out = strtof(text, &end);
    return end != text && *end == '\0' && out >= lo && out <= hi;
}

bool word(const char *text, const char *expected) { return strcasecmp(text, expected) == 0; }

// Handlers return nullptr on success, or the reason for the ERR line.
// The dispatcher has already checked the argument count.

const char *setSetting(const char *key, const char *text) {
    const config::Setting *s = config::find(key);
    if (!s) return "unknown setting, see GET";
    float value;
    if (!number(text, s->min, s->max, value)) {
        static char reason[48];
        snprintf(reason, sizeof(reason), "value must be %g..%g", s->min, s->max);
        return reason;
    }
    *s->value = value;
    return nullptr;
}

const char *cmdPing(char **, int) { return nullptr; }

const char *cmdHelp(char **, int);

const char *cmdStatus(char **, int) {
    telemetry::printStatus();
    return nullptr;
}

const char *cmdImu(char **, int) {
    telemetry::printImu();
    return nullptr;
}

const char *cmdTof(char **args, int count) {
    if (count == 1) {
        if (!word(args[0], "SCAN")) return "use TOF or TOF SCAN";
        if (motion::mode() != motion::Mode::Idle) return "busy moving, STOP first";
        tof::scan();
    }
    tof::printStatus();
    return nullptr;
}

const char *cmdStream(char **args, int) { return setSetting("stream.hz", args[0]); }

const char *cmdDrive(char **args, int count) {
    float speed, steer, ms = 0;
    if (!number(args[0], -100, 100, speed)) return "speed must be -100..100";
    if (!number(args[1], -1, 1, steer)) return "steer must be -1..1";
    if (count > 2 && !number(args[2], 1, 600000, ms)) return "ms must be 1..600000";
    motion::drive(speed, steer, (uint32_t)ms);
    return nullptr;
}

const char *cmdStraight(char **args, int count) {
    float speed, ms = 0;
    if (!number(args[0], -100, 100, speed)) return "speed must be -100..100";
    if (count > 1 && !number(args[1], 1, 600000, ms)) return "ms must be 1..600000";
    if (!motion::straight(speed, (uint32_t)ms)) return "no heading from IMU";
    return nullptr;
}

const char *cmdTurn(char **args, int count) {
    float degrees, power = cfg.turnMaxPower;
    if (!number(args[0], -3600, 3600, degrees) || degrees == 0) return "degrees must be -3600..3600, not 0";
    if (count > 1 && !number(args[1], 1, 100, power)) return "power must be 1..100";
    if (!motion::turn(degrees, power)) return "no heading from IMU";
    return nullptr;
}

const char *cmdStop(char **, int) {
    motion::stop();
    return nullptr;
}

const char *cmdMotors(char **args, int) {
    float speeds[4];
    for (int i = 0; i < 4; i++) {
        if (!number(args[i], -100, 100, speeds[i])) return "speeds must be -100..100";
    }
    motion::stop();
    motors::setEach(speeds);
    return nullptr;
}

const char *cmdGrab(char **args, int) {
    claw::Side side;
    if (word(args[0], "L")) side = claw::Side::Left;
    else if (word(args[0], "C")) side = claw::Side::Center;
    else if (word(args[0], "R")) side = claw::Side::Right;
    else return "side must be L, C or R";
    if (claw::busy()) return "already grabbing";
    claw::grab(side);
    return nullptr;
}

const char *cmdArm(char **args, int) {
    if (claw::busy()) return "busy grabbing";
    if (word(args[0], "UP")) claw::armUp();
    else if (word(args[0], "DOWN")) claw::armDown();
    else return "use UP or DOWN";
    return nullptr;
}

const char *cmdJaw(char **args, int) {
    if (claw::busy()) return "busy grabbing";
    if (word(args[0], "OPEN")) claw::jawsOpen();
    else if (word(args[0], "NEUTRAL")) claw::jawsNeutral();
    else return "use OPEN or NEUTRAL";
    return nullptr;
}

const char *cmdServo(char **args, int) {
    float servo, degrees;
    if (!number(args[0], 1, 3, servo) || servo != floorf(servo)) return "servo must be 1, 2 or 3";
    if (!number(args[1], 0, 180, degrees)) return "degrees must be 0..180";
    if (claw::busy()) return "busy grabbing";
    claw::moveServo((int)servo, degrees);
    return nullptr;
}

const char *cmdLight(char **args, int count) {
    if (count == 1 && word(args[0], "OFF")) {
        lights::setBar(0, 0, 0);
        return nullptr;
    }
    float rgb[3];
    if (count != 3) return "use LIGHT <r> <g> <b> or LIGHT OFF";
    for (int i = 0; i < 3; i++) {
        if (!number(args[i], 0, 255, rgb[i])) return "colors must be 0..255";
    }
    lights::setBar(rgb[0], rgb[1], rgb[2]);
    return nullptr;
}

const char *cmdSet(char **args, int) { return setSetting(args[0], args[1]); }

const char *cmdGet(char **args, int count) {
    if (count == 0) {
        config::printAll();
        return nullptr;
    }
    const config::Setting *s = config::find(args[0]);
    if (!s) return "unknown setting, see GET";
    Serial.printf("CFG %s=%g\n", s->key, *s->value);
    return nullptr;
}

const char *cmdSave(char **, int) { return config::save() ? nullptr : "flash write failed"; }

const char *cmdDefaults(char **, int) {
    config::resetToDefaults();
    return nullptr;
}

const char *cmdBootload(char **, int) {
    motion::stop();
    Serial.println("OK BOOTLOAD");
    Serial.flush();
    delay(50);
    rp2040.rebootToBootloader();
    return nullptr;
}

struct Command {
    const char *name;
    int minArgs, maxArgs;
    const char *(*run)(char **args, int count);
    const char *usage;
};

const Command COMMANDS[] = {
    {"PING", 0, 0, cmdPing, "PING"},
    {"HELP", 0, 0, cmdHelp, "HELP"},
    {"STATUS", 0, 0, cmdStatus, "STATUS  one STAT line"},
    {"IMU", 0, 0, cmdImu, "IMU  one IMU line"},
    {"TOF", 0, 1, cmdTof, "TOF [SCAN]  sensor states, SCAN rediscovers them"},
    {"STREAM", 1, 1, cmdStream, "STREAM <hz>  IMU + STAT lines, 0 = off"},
    {"DRIVE", 2, 3, cmdDrive, "DRIVE <speed> <steer -1..1> [ms]"},
    {"STRAIGHT", 1, 2, cmdStraight, "STRAIGHT <speed> [ms]  hold the current heading"},
    {"TURN", 1, 2, cmdTurn, "TURN <deg, + = right> [power]"},
    {"STOP", 0, 0, cmdStop, "STOP  brake and end the current move"},
    {"MOTORS", 4, 4, cmdMotors, "MOTORS <m1> <m2> <m3> <m4>  raw test, -100..100"},
    {"GRAB", 1, 1, cmdGrab, "GRAB <L|C|R>  full pickup sequence"},
    {"ARM", 1, 1, cmdArm, "ARM <UP|DOWN>"},
    {"JAW", 1, 1, cmdJaw, "JAW <OPEN|NEUTRAL>"},
    {"SERVO", 2, 2, cmdServo, "SERVO <1=jaw L|2=jaw R|3=arm> <deg>"},
    {"LIGHT", 1, 3, cmdLight, "LIGHT <r> <g> <b> | LIGHT OFF"},
    {"SET", 2, 2, cmdSet, "SET <key> <value>  until reboot, SAVE to keep"},
    {"GET", 0, 1, cmdGet, "GET [key]"},
    {"SAVE", 0, 0, cmdSave, "SAVE  write settings to flash"},
    {"DEFAULTS", 0, 0, cmdDefaults, "DEFAULTS  restore defaults, SAVE to keep"},
    {"BOOTLOAD", 0, 0, cmdBootload, "BOOTLOAD  reboot into the UF2 bootloader"},
};

const char *cmdHelp(char **, int) {
    for (const Command &c : COMMANDS) Serial.printf("HELP %s\n", c.usage);
    return nullptr;
}

void run(char *line) {
    char *args[MAX_ARGS + 1];
    char *name = strtok(line, " \t");
    if (!name) return;
    int count = 0;
    char *token;
    while ((token = strtok(nullptr, " \t")) && count <= MAX_ARGS) args[count++] = token;

    for (const Command &c : COMMANDS) {
        if (!word(name, c.name)) continue;
        if (count < c.minArgs || count > c.maxArgs) {
            Serial.printf("ERR %s usage: %s\n", c.name, c.usage);
            return;
        }
        const char *error = c.run(args, count);
        if (error) Serial.printf("ERR %s %s\n", c.name, error);
        else Serial.printf("OK %s\n", c.name);
        return;
    }
    Serial.printf("ERR %s unknown command, try HELP\n", name);
}

char line[96];
size_t length = 0;
bool tooLong = false;

}  // namespace

namespace commands {

void poll() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c != '\n' && c != '\r') {
            if (length < sizeof(line) - 1) line[length++] = c;
            else tooLong = true;
            continue;
        }
        line[length] = '\0';
        if (tooLong) Serial.println("ERR LINE too long");
        else run(line);
        length = 0;
        tooLong = false;
    }
}

}  // namespace commands
