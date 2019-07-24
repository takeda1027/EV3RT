//
//  crew.hpp
//  aflac2019
//
//  Created by Wataru Taniguchi on 2019/04/28.
//  Copyright © 2019 Ahiruchan Koubou. All rights reserved.
//

#ifndef crew_hpp
#define crew_hpp

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#include <cinttypes>
#include <cmath>
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
using namespace ev3api;
#include "utility.hpp"

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE          60  /* 白色の光センサ値 */
#define LIGHT_BLACK           3  /* 黒色の光センサ値 */
#define HSV_V_WHITE         100
#define HSV_V_LOST           90  // threshold to determine "line lost"
#define HSV_V_FOUND          60  // threshold to determine "line found"
// Note that lost_flag does NOT flip when between HSV_V_LOST and HSV_V_FOUND
#define HSV_V_BLACK           0
#define HSV_V_BLUE           35
#define RGB_HIST_ARRAY_SIZE  30
#define RGB_COMP_LEN         10
#define FINAL_APPROACH_LEN  100  // final approch length in milimater
#define SONAR_ALERT_DISTANCE 30  /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP  90  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */

#define TIRE_DIAMETER    100.0F  // diameter of tire in milimater
#define WHEEL_TREAD      175.0F  // distance between the right and left wheels
#define P_CONST           0.38L  // PID constants determined by Ultimate Gain method
#define I_CONST           0.06L
#define D_CONST          0.027L

//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START_R     'R' // R-mode start command
#define CMD_START_r     'r' // R-mode start command
#define CMD_START_L     'L' // L-mode start command
#define CMD_START_l     'l' // L-mode start command
#define CMD_DANCE_D     'D'
#define CMD_DANCE_d     'd'
#define CMD_CRIMB_C     'C'
#define CMD_CRIMB_c     'c'
#define CMD_STOP_S      'S'
#define CMD_STOP_s      's'

// machine state
#define ST_takingOff    0
#define ST_tracing_L    1
#define ST_crimbing     2
#define ST_tracing_R    3
#define ST_dancing      4
#define ST_stopping_L   5
#define ST_stopping_R   6
#define ST_landing      7

#define ST_NAME_LEN     20  // maximum number of characters for a machine state name
const char stateName[][ST_NAME_LEN] = {
    "ST_takingOff",
    "ST_tracing_L",
    "ST_crimbing",
    "ST_tracing_R",
    "ST_dancing",
    "ST_stopping_L",
    "ST_stopping_R",
    "ST_landing"
};

// event
#define EVT_cmdStart_L      0
#define EVT_cmdStart_R      1
#define EVT_touch_On        2
#define EVT_touch_Off       3
#define EVT_sonar_On        4
#define EVT_sonar_Off       5
#define EVT_backButton_On   6
#define EVT_backButton_Off  7
#define EVT_bk2bl           8
#define EVT_bl2bk           9
#define EVT_cmdDance        10
#define EVT_cmdCrimb        11
#define EVT_cmdStop         12
#define EVT_line_lost       13
#define EVT_line_found      14
#define EVT_dist_reached    15

#define EVT_NAME_LEN        20  // maximum number of characters for an event name
const char eventName[][EVT_NAME_LEN] = {
    "EVT_cmdStart_L",
    "EVT_cmdStart_R",
    "EVT_touch_On",
    "EVT_touch_Off",
    "EVT_sonar_On",
    "EVT_sonar_Off",
    "EVT_backButton_On",
    "EVT_backButton_Off",
    "EVT_bk2bl",
    "EVT_bl2bk",
    "EVT_cmdDance",
    "EVT_cmdCrimb",
    "EVT_cmdStop",
    "EVT_line_lost",
    "EVT_line_found",
    "EVT_dist_reached"
};

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

#define PERIOD_TRACE_MSG    500    /* Trace message in every 1000 ms */
#define M_2PI    (2.0 * M_PI)

class Radioman {
private:
    FILE*           bt;      /* Bluetoothファイルハンドル */
public:
    Radioman();
    void operate(); // method to invoke from the task handler
    ~Radioman();
};

class Observer {
private:
    Motor*          leftMotor;
    Motor*          rightMotor;
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    GyroSensor*     gyroSensor;
    ColorSensor*    colorSensor;
    double distance, azimuth, locX, locY;
    int16_t traceCnt;
    int32_t prevAngL, prevAngR, notifyDistance;
    rgb_raw_t cur_rgb;
    hsv_raw_t cur_hsv;
    bool touch_flag, sonar_flag, backButton_flag, lost_flag, blue_flag;
    uint16_t rgb_hist_r[RGB_HIST_ARRAY_SIZE], rgb_hist_b[RGB_HIST_ARRAY_SIZE];
    long double rgb_avg_r_new, rgb_avg_r_old, rgb_avg_b_new, rgb_avg_b_old;
    int rgb_hist_index;
    bool check_touch(void);
    bool check_sonar(void);
    bool check_backButton(void);
    bool check_lost(void);
    bool check_found(void);
    bool check_blue(void);
    bool check_black(void);
protected:
public:
    Observer();
    Observer(Motor* lm, Motor* rm, TouchSensor* ts, SonarSensor* ss, GyroSensor* gs, ColorSensor* cs);
    void goOnDuty();
    void reset();
    void notifyOfDistance(int32_t delta);
    int32_t getDistance();
    int16_t getAzimuth();
    int32_t getLocX();
    int32_t getLocY();
    void operate(); // method to invoke from the cyclic handler
    void goOffDuty();
    ~Observer();
};

class Navigator {
private:
    long double kp, ki, kd;   /* PID constant */
    int16_t diff[2];
    long double integral;
protected:
    int8_t forward;      /* 前後進命令 */
    int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */
    int16_t         trace_pwmT, trace_pwmLR;
    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*          tailMotor;
    GyroSensor*     gyroSensor;
    ColorSensor*    colorSensor;
    void cancelBacklash(int8_t lpwm, int8_t rpwm, int32_t *lenc, int32_t *renc);
    void controlTail(int32_t angle);
    void setPIDconst(long double p, long double i, long double d);
    int16_t math_limit(int16_t input, int16_t min, int16_t max);
    int16_t computePID(int16_t sensor, int16_t target);
public:
    Navigator();
    void goOnDuty();
    virtual void haveControl() = 0;
    virtual void operate() = 0;
    void goOffDuty();
    virtual ~Navigator();
};

class AnchorWatch : public Navigator {
private:
protected:
public:
    AnchorWatch();
    AnchorWatch(Motor* tm);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~AnchorWatch();
};

class LineTracer : public Navigator {
private:
    int32_t motor_ang_l, motor_ang_r;
    int32_t gyro, volt;
    bool    frozen;
protected:
public:
    LineTracer();
    LineTracer(Motor* lm, Motor* rm, Motor* tm, GyroSensor* gs, ColorSensor* cs);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    void freeze();
    void unfreeze();
    ~LineTracer();
};

#include "SeesawCrimber.hpp"
#include "LimboDancer.hpp"

class Captain {
private:
    TouchSensor*    touchSensor;
    SonarSensor*    sonarSensor;
    ColorSensor*    colorSensor;
    GyroSensor*     gyroSensor;
    Motor*          leftMotor;
    Motor*          rightMotor;
    Motor*          tailMotor;
    AnchorWatch*    anchorWatch;
    LineTracer*     lineTracer;
    SeesawCrimber*  seesawCrimber;
    LimboDancer*    limboDancer;
protected:
public:
    Captain();
    void takeoff();
    void decide(uint8_t event);
    void triggerLanding();
    void land();
    ~Captain();
};

extern Captain*     captain;
extern Observer*    observer;
extern Navigator*   activeNavigator;
extern Clock*       clock;
extern uint8_t      state;

#endif /* crew_hpp */
