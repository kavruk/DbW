#include <EEPROM.h>
#include <PID_v1.h>

// Pins H bridge
const int HB_FORWARD = 9; // H bridge Forward
const int HB_REVERSE = 5; // H bridge Reverse
const int HB_ENABLE = 6;  // H bridge Enable
// Inputs
const int PM_SERVO_1 = A2; // Potentiometru servo 1
const int PM_INPUT_1 = A4; // Potentiometru input 1

// EEPROM Addresses
const int EE_BASE_ADDR = 0x10;
const int EE_MAGIC_ADDR = EE_BASE_ADDR;
const int EE_SERVO_MIN_ADDR = EE_BASE_ADDR + 0x02;
const int EE_SERVO_MAX_ADDR = EE_BASE_ADDR + 0x04;
const int EE_SERVO_POS_MIN_ADDR = EE_BASE_ADDR + 0x06;
const int EE_SERVO_POS_MAX_ADDR = EE_BASE_ADDR + 0x08;
const int EE_INPUT_MIN_ADDR = EE_BASE_ADDR + 0x0A;
const int EE_INPUT_MAX_ADDR = EE_BASE_ADDR + 0x0C;
const int EE_PID_PRESET_ADDR = EE_BASE_ADDR + 0x0E;
const int EE_PID_PRESETS_BASE_ADDR = EE_BASE_ADDR + 0x10;

const int PID_CONF_SIZE = 26;

// Configuration
// Servo
int servoMin = 0;
int servoMax = 1023;
int servoPosMin = 0;
int servoPosMax = 255;

// Input
int inputMin = 0;
int inputMax = 1023;

// PID
int pidPreset = 0;
double pidKp = 0;
double pidKi = 0;
double pidKd = 0;
int pidPWM = 1;

// Runtime configuration
bool showInfo = false;

// PID variables
double Input, Output, SetPoint;
PID servoPID(&Input, &Output, &SetPoint, pidKp, pidKi, pidKd, DIRECT);

void ee_write_block(int addr, char *buf, int size) {
    for (int i = 0; i < size; i++) {
        EEPROM.write(addr++, *buf++);
    }
}

void ee_read_block(int addr, char *buf, int size) {
    for (int i = 0; i < size; i++) {
        *buf++ = EEPROM.read(addr++);
    }
}

void ee_write_int(int addr, int val) {
    ee_write_block(addr, (char *)&val, sizeof(val));
}

int ee_read_int(int addr) {
    int val = 0;
    ee_read_block(addr, (char *)&val, sizeof(val));
    return val;
}

void ee_write_double(int addr, double val) {
    ee_write_block(addr, (char *)&val, sizeof(val));
}

double ee_read_double(int addr) {
    double val = 0;
    ee_read_block(addr, (char *)&val, sizeof(val));
    return val;
}

void ee_write_pid(int preset, double Kp, double Ki, double Kd, int pwm) {
    int base_addr = EE_PID_PRESETS_BASE_ADDR + (preset * PID_CONF_SIZE);
    ee_write_double(base_addr, Kp);
    ee_write_double(base_addr + 8, Ki);
    ee_write_double(base_addr + 16, Kd);
    ee_write_int(base_addr + 24, pwm);
}

void ee_read_pid(int preset, double *Kp, double *Ki, double *Kd, int *pwm) {
    int base_addr = EE_PID_PRESETS_BASE_ADDR + (preset * PID_CONF_SIZE);
    *Kp = ee_read_double(base_addr);
    *Ki = ee_read_double(base_addr + 8);
    *Kd = ee_read_double(base_addr + 16);
    *pwm = ee_read_int(base_addr + 24);
}

void setupEEPROM() {
    // If EEPROM was not initialized, initialize with default values
    int magic = ee_read_int(EE_MAGIC_ADDR);
    if (magic != 0xB00B) {
        ee_write_int(EE_MAGIC_ADDR, 0xB00B);
        ee_write_int(EE_SERVO_MIN_ADDR, servoMin);
        ee_write_int(EE_SERVO_MAX_ADDR, servoMax);
        ee_write_int(EE_SERVO_POS_MIN_ADDR, servoPosMin);
        ee_write_int(EE_SERVO_POS_MAX_ADDR, servoPosMax);
        ee_write_int(EE_INPUT_MIN_ADDR, inputMin);
        ee_write_int(EE_INPUT_MAX_ADDR, inputMax);
        ee_write_int(EE_PID_PRESET_ADDR, pidPreset);
        ee_write_pid(0, pidKp, pidKi, pidKd, pidPWM);
        ee_write_pid(1, pidKp, pidKi, pidKd, pidPWM);
        ee_write_pid(2, pidKp, pidKi, pidKd, pidPWM);
        ee_write_pid(3, pidKp, pidKi, pidKd, pidPWM);
        return;
    }

    // Read value from EEPROM
    servoMin = ee_read_int(EE_SERVO_MIN_ADDR);
    servoMax = ee_read_int(EE_SERVO_MAX_ADDR);
    servoPosMin = ee_read_int(EE_SERVO_POS_MIN_ADDR);
    servoPosMax = ee_read_int(EE_SERVO_POS_MAX_ADDR);

    inputMin = ee_read_int(EE_INPUT_MIN_ADDR);
    inputMax = ee_read_int(EE_INPUT_MAX_ADDR);

    pidPreset = ee_read_int(EE_PID_PRESET_ADDR);
    ee_read_pid(pidPreset, &pidKp, &pidKi, &pidKd, &pidPWM);
}

char *readLine() {
    static char buff[11];
    char *pos = buff;
    char *end = buff + sizeof(buff) / sizeof(buff[0]) - 1;

    for(;;) {
        int chr = Serial.read();
        if (chr == -1) continue;
        if (chr == '\n' || chr == '\r') break;

        *pos++ = chr;
        if (pos == end) break;
    }

    *pos = 0;
    return buff;
}

int readInt(const char *what, int min, int max) {
    for(;;) {
        Serial.print(what);
        Serial.print(" (");
        Serial.print(min);
        Serial.print("-");
        Serial.print(max);
        Serial.print("): ");

        char *line = readLine();
        int val = atoi(line);
        if (val < min) {
            Serial.print("Invalid value, must be between ");
            Serial.print(min);
            Serial.print(" and ");
            Serial.print(max);
            Serial.println();
            continue;
        }

        if (val > max) {
            Serial.print("Invalid value, must be between ");
            Serial.print(min);
            Serial.print(" and ");
            Serial.print(max);
            Serial.println();
            continue;
        }

        return val;
    }

    return 0;
}

int readDouble(const char *what, double min, double max) {
    for(;;) {
        Serial.print(what);
        Serial.print(" (");
        Serial.print(min);
        Serial.print("-");
        Serial.print(max);
        Serial.print("): ");

        char *line = readLine();
        double val = atof(line);
        if (val < min) {
            Serial.print("Invalid value, must be between ");
            Serial.print(min);
            Serial.print(" and ");
            Serial.print(max);
            Serial.println();
            continue;
        }

        if (val > max) {
            Serial.print("Invalid value, must be between ");
            Serial.print(min);
            Serial.print(" and ");
            Serial.print(max);
            Serial.println();
            continue;
        }

        return val;
    }

    return 0;
}

void menuItem(const char *option, const char *name, const int currentVal) {
    Serial.print(option);
    Serial.print(") ");
    Serial.print(name);
    Serial.print(" = ");
    Serial.print(currentVal);
    Serial.println();
}

void menuServo() {
    for (;;) {
        Serial.print("Select the value you want to change\n");

        menuItem("1", "Minimum servo sensor value (0-1023)", servoMin);
        menuItem("2", "Maximum servo sensor value (1-1023)", servoMax);
        menuItem("3", "Minimum servo position value (0-255)", servoPosMin);
        menuItem("4", "Maximum servo position value (0-255)", servoPosMax);
        Serial.print("5) Print servo sensor value\n");
        Serial.print("6) Print servo position value\n");
        Serial.print("e) Exit menu\n");

        int option = -1;
        while(option == -1) option = Serial.read();

        int servoPos;
        switch(option) {
            case 'e': return;

            case '1':
                servoMin = readInt("Minimum servo sensor value", 0, servoMax - 1);
                ee_write_int(EE_SERVO_MIN_ADDR, servoMin);
                break;

            case '2':
                servoMax = readInt("Minimum servo sensor value", servoMin + 1, 1023);
                ee_write_int(EE_SERVO_MAX_ADDR, servoMax);
                break;

            case '3':
                servoPosMin = readInt("Minimum servo position value", 0, servoPosMax - 1);
                ee_write_int(EE_SERVO_POS_MIN_ADDR, servoPosMin);
                break;

            case '4':
                servoPosMax = readInt("Minimum servo position value", servoPosMin + 1, 255);
                ee_write_int(EE_SERVO_POS_MAX_ADDR, servoPosMax);
                break;

            case '5':
                Serial.print("Current servo sensor value: ");
                Serial.print(analogRead(PM_SERVO_1));
                Serial.println();
                break;

            case '6':
                Serial.print("Current servo position value: ");
                Serial.print(readPot(PM_SERVO_1, servoMin, servoMax, 0, 255));
                Serial.println();
                break;

            default:
                Serial.print("Invalid option: ");
                Serial.print((char)option);
                Serial.println();
                break;
        }
    }
}
void menuInput() {
    for (;;) {
        Serial.print("Select the value you want to change\n");

        menuItem("1", "Minimum input sensor value (0-1023)", inputMin);
        menuItem("2", "Maximum input sensor value (0-1023)", inputMax);
        Serial.print("3) Print input sensor value\n");
        Serial.print("e) Exit menu\n");

        int option = -1;
        while(option == -1) option = Serial.read();

        switch(option) {
            case 'e': return;

            case '1':
                inputMin = readInt("Minimum input sensor value", 0, inputMax - 1);
                ee_write_int(EE_INPUT_MIN_ADDR, inputMin);
                break;

            case '2':
                inputMax = readInt("Maximum input sensor value", inputMin + 1, 1023);
                ee_write_int(EE_INPUT_MAX_ADDR, inputMax);
                break;

            case '3':
                Serial.print("Current input sensor value: ");
                Serial.print(analogRead(PM_INPUT_1));
                Serial.println();
                break;

            default:
                Serial.print("Invalid option: ");
                Serial.print((char)option);
                Serial.println();
                break;
        }
    }
}

void menuPIDPreset(int preset) {
    int base_addr = EE_PID_PRESETS_BASE_ADDR + (preset * PID_CONF_SIZE);

    double pKp, pKi, pKd;
    int pPWM;

    if (preset != pidPreset) {
        ee_read_pid(preset, &pKp, &pKi, &pKd, &pPWM);
    } else {
        pKp = pidKp;
        pKi = pidKi;
        pKd = pidKd;
        pPWM = pidPWM;
    }

    for (;;) {
        Serial.print("Editing preset #");
        Serial.print(preset + 1);
        Serial.println();

        Serial.print("1) Kp (>= 0) = "); Serial.print(pKp); Serial.println();
        Serial.print("2) Ki (>= 0)= "); Serial.print(pKi); Serial.println();
        Serial.print("3) Kd (>= 0) = "); Serial.print(pKd); Serial.println();
        Serial.print("4) PWM (1-255) = "); Serial.print(pPWM); Serial.println();

        if (preset != pidPreset) {
            Serial.print("5) Set as active\n");
        }

        Serial.print("e) Exit menu\n");

        int option = -1;
        while(option == -1) option = Serial.read();

        switch(option) {
            case 'e': return;

            case '1':
                pKp = readDouble("Kp", 0, INFINITY);
                ee_write_double(base_addr, pKp);
                break;

            case '2':
                pKi = readDouble("Kp", 0, INFINITY);
                ee_write_double(base_addr + 8, pKi);
                break;

            case '3':
                pKd = readDouble("Kp", 0, INFINITY);
                ee_write_double(base_addr + 16, pKd);
                break;

            case '4':
                pPWM = readInt("PWM", 1, 255);
                ee_write_int(base_addr + 24, pPWM);
                break;

            case '5':
                pidPreset = preset;
                ee_write_int(EE_PID_PRESET_ADDR, pidPreset);
                break;

            default:
                Serial.print("Invalid option: ");
                Serial.print((char)option);
                Serial.println();
                break;
        }

        if (preset == pidPreset) {
            pidKp = pKp;
            pidKi = pKi;
            pidKd = pKd;
            pidPWM = pPWM;
            servoPID.SetTunings(pidKp, pidKi, pidKd);
        }
    }
}

void menuPIDPresets() {
    for (;;) {
        Serial.print("Select PID preset\n");

        Serial.print("1) Preset 1\n");
        Serial.print("2) Preset 2\n");
        Serial.print("3) Preset 3\n");
        Serial.print("4) Preset 4\n");
        Serial.print("e) Exit menu\n");

        int option = -1;
        while(option == -1) option = Serial.read();

        int preset;
        switch(option) {
            case 'e': return;

            case '1':
            case '2':
            case '3':
            case '4':
                preset = option - '1';
                menuPIDPreset(preset);
                break;

            default:
                Serial.print("Invalid option: ");
                Serial.print((char)option);
                Serial.println();
                break;
        }
    }
}

void menuMain() {
    for (;;) {
        Serial.print("Select submenu\n");

        Serial.print("1) Servo settings\n");
        Serial.print("2) Input settings\n");
        Serial.print("3) PID settings\n");
        Serial.print("e) Exit menu\n");

        int option = -1;
        while(option == -1) option = Serial.read();

        switch(option) {
            case 'e': return;

            case '1':
                menuServo();
                break;

            case '2':
                menuInput();
                break;

            case '3':
                menuPIDPresets();
                break;

            default:
                Serial.print("Invalid option: ");
                Serial.print((char)option);
                Serial.println();
                break;
        }
    }
}

// the setup routine runs once when you press reset:
void setup() {
    // Setup serial
  Serial.begin(9600);

    // Setup EEPROM data
    setupEEPROM();

    // Setup pins
  pinMode(HB_REVERSE, OUTPUT);
  pinMode(HB_FORWARD, OUTPUT);
  pinMode(HB_ENABLE, OUTPUT);

    // Setup pid
  servoPID.SetSampleTime(10);
  servoPID.SetMode(AUTOMATIC);
    servoPID.SetTunings(pidKp, pidKi, pidKd);
}

// the loop routine runs over and over again forever:
void loop() {
    if (Serial.available()) {
        switch (Serial.read()) {
            case 'm':
                menuMain();
                break;
            case 't':
                showInfo = !showInfo;
                break;
        }
    }

    // Read potentiometer values
    Input = readPot(PM_SERVO_1, servoMin, servoMax, servoPosMin, servoPosMax);
    SetPoint = readPot(PM_INPUT_1, inputMin, inputMax, 0, 255);
  servoPID.Compute();

    setHBridgePWM();

    if (showInfo) {
        printInfo();
    }

    // Delay
    delay(10);
}

int readPot(int pin, int minVal, int maxVal, int minPos, int maxPos) {
    int val = analogRead(pin);
    int pos = (long)255 * (val - minVal) / (maxVal - minVal);

    // Ensure we're within position limits
    if (pos < minPos) return minPos;
    if (pos > maxPos) return maxPos;

    return pos;
}

void setHBridgePWM() {
    if (Input < Output) {
        digitalWrite(HB_FORWARD, HIGH);
        digitalWrite(HB_REVERSE, LOW);
    } else {
        digitalWrite(HB_FORWARD, LOW);
        digitalWrite(HB_REVERSE, HIGH);
    }

    analogWrite(HB_ENABLE, pidPWM * abs(Input - Output) / 255);
}

void printInfo() {
    // Print PID values
    Serial.print("\r");
    Serial.print("Input: ");
    Serial.print(Input);
    Serial.print(", Output: ");
    Serial.print(Output);
    Serial.print(", SetPoint: ");
    Serial.print(SetPoint);
    Serial.print("                     "); // Override leftovers
}
