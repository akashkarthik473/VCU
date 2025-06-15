#include <stdlib.h>
#include <math.h>
#include "IO_RTC.h"
#include "IO_DIO.h"
#include "launchControl.h"
#include "wheelSpeeds.h"
#include "mathFunctions.h"
#include "initializations.h"
#include "sensors.h"
#include "torqueEncoder.h"
#include "brakePressureSensor.h"
#include "motorController.h"
#include "sensorCalculations.h"
#include "drs.h"
#include "PID.h"
#include "IO_Driver.h" //Includes datatypes, constants, etc - should be included in every c file

extern Sensor Sensor_LCButton;
extern Sensor Sensor_RightKnob;

//LC Status Flags
//nibble 1
static const ubyte1 LC_ready = 1;
static const ubyte1 LC_active = 2;
static const ubyte1 LC_initalCurve = 4;
static const ubyte1 LC_overtorque = 8;
//nibble 2
static const ubyte1 LC_speedMode = 0x10;
static const ubyte1 LC_constantSpeedOverride = 0x20;
static const ubyte1 LC_belowSlipTarget = 0x40;
static const ubyte1 LC_aboveSlipTarget = 0x80;

//Initial Torque Setpoints
static const sbyte2 Low = 105;
static const sbyte2 Standard = 110;
static const sbyte2 Medium_High = 115;
static const sbyte2 High = 120;
static const sbyte2 Rain_Guess_1 = 40;
static const sbyte2 Rain_Guess_2 = 50;


LaunchControl *LaunchControl_new(){
    LaunchControl* me = (LaunchControl*)malloc(sizeof(struct _LaunchControl));
    // malloc returns NULL if it fails to allocate memory
    if (me == NULL)
        return NULL;

    //Torque Mode Settings for LC
    me->pidTorque = PID_new(20, 1, 0, 0, 1000); //Divide by 1000 needed to offset the x 1000 used to scale up our calculated slip ratio.
    PID_updateSettings(me->pidTorque, setpoint, 325); // Having a statically coded slip ratio may not be the best. this requires knowing that this is both a) the best slip ratio for the track, and b) that our fronts are not in any way slipping / entirely truthful regarding the groundspeed of the car. Using accel as a target is perhaps better, but needs to be better understood.
    PID_updateSettings(me->pidTorque, frequency, 1);
    PID_updateSettings(me->pidTorque, sensorInput, 1);
    me->lcTorqueCommand = NULL;
    me->initialTorque = Standard;

    me->constA = 7;
    me->constB = 20;

    //Slip Ratio
    me->slipRatio = 0;
    me->slipRatioThreeDigits = 0;

    //Flags and Timer
    me->safteyTimer = 0;
    me->lcReady = FALSE;
    me->lcActive = FALSE;
    me->initialCurve = FALSE;
    me->overTorque = FALSE;
    me->flags = 0x00;

    //Speed Mode Settings for LC
    me->pidSpeed =  PID_new(200, 2, 0, 0, 100); //No saturation point to see what the behavior of the PID is, will need a saturation value somewhere to prevent wind-up of the pid in the future
    me->lcSpeedCommand = NULL;

    //me->speedMode //should be reflected from mcm->speedControlValidity & other flags. this may also be delayed by a VCU cycle, so a bug exists here

    /** Variables for constantSpeedTestOverride Function. 
     * Enabling this mode disabled Launch Control & 
     * changes button function to act as a cruise control targeting a specified speed */
    me->constantSpeedTestOverride = FALSE;
    me->overrideTestSpeedCommand = 3000; // CONSTANT SPEED TARGET

    me->buttonDebug = 0; // This exists as a holdover piece of code to what I presume is debugging which button was which on the steering wheel. should remove / place elsewhere
    return me;
}

#ifdef LAUNCHCONTROL_ENABLE

void LaunchControl_calculateSlipRatio(LaunchControl *me, MotorController *mcm, WheelSpeeds *wss){
    //Non interpolated Wheelspeeds always read 0 for some reason, so we are forced to use the interpolated speeds.
    if( (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, TRUE) + 0.5) != 0 ) {
        ubyte2 RearR = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, RR, TRUE) + 0.5);
        ubyte2 FrontL = (ubyte2)(WheelSpeeds_getWheelSpeedRPM(wss, FL, TRUE) + 0.5);
        ubyte4 calcs = (RearR * 1000) / FrontL;
        me->slipRatioThreeDigits = (ubyte2) calcs;
    }
        me->slipRatioThreeDigits = (ubyte2) calcs; // Why does the system respond well when we didnt subtract 1?
    }
}

void LaunchControl_initialTorqueCurve(LaunchControl* me, MotorController* mcm){
    me->lcTorqueCommand = (sbyte2) me->initialTorque + ( MCM_getMotorRPM(mcm) * 7 / 20 ); // Tunable Values will be the inital Torque Request @ 0 and the scalar factor
}

void LaunchControl_calculateTorqueCommand(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs){
    if( MCM_get_LC_activeStatus(mcm) )
    {
        if( MCM_getGroundSpeedKPH(mcm) < 5 ) {
            me->initialCurve = TRUE;
            LaunchControl_initialTorqueCurve(me, mcm);
        } else {
            me->initialCurve = FALSE;
            PID_addSensorInput(me->pidTorque, (sbyte2) MCM_getCommandedTorque(mcm));
            me->lcTorqueCommand = PID_computeOutput(me->pidTorque, me->slipRatioThreeDigits);
        }

        // Update launch control torque command in mcm struct
        if(me->lcTorqueCommand > 231) {
            me->lcTorqueCommand = 231;
            me->overTorque = TRUE;
        }
        else { me->overTorque = FALSE; }

        if(MCM_getMotorRPM(mcm) >= 3500) {
            me->lcTorqueCommand = 225; // Mini-Torque Derate, might help with field weakening as well as staying under 80 kwh threshold
        }
        MCM_update_LC_torqueCommand(mcm, me->lcTorqueCommand);
    }
}

void LaunchControl_initialRPMCurve(LaunchControl* me, MotorController* mcm){
    me->lcSpeedCommand = (sbyte2) 100 + ( MCM_getMotorRPM(mcm) * 10 ); // Tunable Values will be the inital Speed Request @ 0 and the scalar factor
}

void LaunchControl_calculateSpeedCommand(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs){
    if(me->lcActive && !me->constantSpeedTestOverride){
        if(MCM_getGroundSpeedKPH(mcm) < 3){
            LaunchControl_initialRPMCurve(me,mcm);
        }
        else{
            me->slipRatioThreeDigits = (sbyte2) (me->slipRatio * 100);
            PID_computeOutput(me->pidSpeed, me->slipRatioThreeDigits);
            me->lcSpeedCommand = MCM_getMotorRPM(mcm)+ PID_getOutput(me->pidSpeed);
        }
        // Tune
        if(MCM_getGroundSpeedKPH(mcm) == 30 ){ DRS_open(drs); }
        // Update launch control speed command in mcm struct
        MCM_update_LC_speedCommand(mcm, me->lcSpeedCommand);
    }
    // constantSpeedTestOverride
    else if(me->constantSpeedTestOverride && Sensor_LCButton.sensorValue == TRUE){
        MCM_update_LC_speedCommand(mcm, me->overrideTestSpeedCommand);
        MCM_update_LC_activeStatus(mcm, TRUE); //We fake the "active" status to enable speed mode in the mcm
    }
}

void LaunchControl_checkState(LaunchControl *me, TorqueEncoder *tps, BrakePressureSensor *bps, MotorController *mcm, DRS *drs){
    sbyte2 speedKph         = MCM_getGroundSpeedKPH(mcm);

    /* LC STATUS CONDITIONS *//*
     * lcReady = FALSE && lcActive = FALSE -> NOTHING HAPPENS
     * lcReady = TRUE  && lcActive = FALSE -> We are in the prep stage for lc, and all entry conditions for being in prep stage have and continue to be monitored
     * lcReady = FALSE && lcActive = TRUE  -> We have left the prep stage by releasing the lc button on the steering wheel, stay in Launch until exit conditions are met
     * AT ALL TIMES, EXIT CONDITIONS ARE CHECKED FOR BOTH STATES
    */

    /**
     * Launch Control Pre-Staging Operations:
     * If the car is near 0 kph (in case of wss float issues) & the Launch Button is pressed,
     * we initialise a 0.5 second timer to confirm a valid Launch attempt
     * Once this timer reaches maturity, we are now in "ready" state
     * The driver can now fully press TPS/APPS without moving car
     * 
     * Upon button release, we are now in "active" state and will proceed with our launch as intended -> car go eeeeeeeee (e-motor sounds)
     * 
     * At any time, an exit condition can be triggered to reset this staging operation and cancel our launch attempt
     */
    // Handle the LC Active Entry first so we don't need to displace the lcReady = FALSE trigger to a differnet place (if a button is released, lcReady would otherwise report false before we try to enter lcActive)
    if( me->lcReady == TRUE && Sensor_LCButton.sensorValue == FALSE ){
        PID_updateSettings(me->pidTorque, totalError, 170); // Error should be set here, so for every launch we reset our error to this value (check if this is the best value)
        me->lcActive = TRUE;
        me->lcReady = FALSE;
    }
    
    if(Sensor_LCButton.sensorValue == TRUE && speedKph < 1 && bps->percent < 0.05 ) {
        if (me->safteyTimer == 0){
            IO_RTC_StartTime(&me->safteyTimer);
            // DRS_open(drs); // Visual Indicator of LC staging
        }
        else if (IO_RTC_GetTimeUS(me->safteyTimer) >= 50000) {
            me->lcReady = TRUE;
            // DRS_close(drs); // Visual Indicator of LC staging
            me->safteyTimer = 0; // We don't need to track the time anymore
        }
    } else { me->lcReady = FALSE; }

    if( tps->tps0_percent < 0.90 || tps->tps0_percent < 0.90 || bps->percent > 0.05 ){
        me->lcActive = FALSE;
        me->lcTorqueCommand = NULL;
    }
    
    MCM_update_LC_activeStatus(mcm, (bool)me->lcActive);
    MCM_update_LC_readyStatus(mcm, (bool)me->lcReady);
}

ubyte1 LaunchControl_getStatus(LaunchControl *me){ 
    //Ready;
    if (me->lcReady == TRUE)
    {
        me->flags |= LC_ready;
    }
    else
    {
        me->flags &= ~LC_ready;
    }
    //Active;
    if (me->lcActive == TRUE)
    {
        me->flags |= LC_active;
    }
    else
    {
        me->flags &= ~LC_active;
    }
    //Predetermined Torque Curve;
    if (me->initialCurve == TRUE)
    {
        me->flags |= LC_initalCurve;
    }
    else
    {
        me->flags &= ~LC_initalCurve;
    }
    //LC Requesting above MaxTorque;
    if (me->overTorque == TRUE)
    {
        me->flags |= LC_overtorque;
    }
    else
    {
        me->flags &= ~LC_overtorque;
    }
    //LC In Speed Mode;
    if (me->lcSpeedCommand != 0)
    {
        me->flags |= LC_speedMode;
    }
    else
    {
        me->flags &= ~LC_speedMode;
    }
    //LC Used for Constant Speed Tests;
    if (me->constantSpeedTestOverride == TRUE)
    {
        me->flags |= LC_constantSpeedOverride;
    }
    else
    {
        me->flags &= ~LC_constantSpeedOverride;
    }
    //Reports if undershooting Slip Target;
    if (me->pidTorque->setpoint >= me->slipRatioThreeDigits)
    {
        me->flags |= LC_belowSlipTarget;
    }
    else
    {
        me->flags &= ~LC_belowSlipTarget;
    }
    //Reports if overshooting Slip Target;
    if (me->pidTorque->setpoint <= me->slipRatioThreeDigits)
    {
        me->flags |= LC_aboveSlipTarget;
    }
    else
    {
        me->flags &= ~LC_aboveSlipTarget;
    }
    return me->flags; 
}

sbyte2 LaunchControl_getTorqueCommand(LaunchControl *me){ return me->lcTorqueCommand; }

float LaunchControl_getSlipRatio(LaunchControl *me){ return me->slipRatio; }

sbyte2 LaunchControl_getSlipRatioThreeDigits(LaunchControl *me){ return me->slipRatioThreeDigits; }

ubyte1 LaunchControl_getButtonDebug(LaunchControl *me) { return me->buttonDebug; }

void LaunchControl_checkRotary(LaunchControl *me)
{
    switch(Rotary_getLeftPosition()){
        case ROTARY_POS_0:
            PID_updateSettings(me->pidTorque, setpoint, 325);
            PID_updateSettings(me->pidTorque, frequency, 1);
            me->initialTorque = Standard;
            me->constA = 7;
            me->constB = 20;
            break;

        case ROTARY_POS_1:
            PID_updateSettings(me->pidTorque, setpoint, 325);
            PID_updateSettings(me->pidTorque, frequency, 1);
            me->initialTorque = Standard;
            me->constA = 13;
            me->constB = 40;
            break;

        case ROTARY_POS_2:
            PID_updateSettings(me->pidTorque, setpoint, 315);
            PID_updateSettings(me->pidTorque, frequency, 1);
            me->initialTorque = Standard;
            me->constA = 7;
            me->constB = 20;
            break;

        case ROTARY_POS_3:
            PID_updateSettings(me->pidTorque, setpoint, 300);
            PID_updateSettings(me->pidTorque, frequency, 1);
            me->initialTorque = Medium_High;
            me->constA = 13;
            me->constB = 40;
            break;

        case ROTARY_POS_4:
            PID_updateSettings(me->pidTorque, setpoint, 325);
            PID_updateSettings(me->pidTorque, frequency, 1);
            me->initialTorque = High;
            me->constA = 19;
            me->constB = 60;
            break;

        case ROTARY_POS_5: // Make this a full hard-coded launch?
            PID_updateSettings(me->pidTorque, setpoint, 325);
            PID_updateSettings(me->pidTorque, frequency, 1);
            me->initialTorque = Standard;
            me->constA = 7;
            me->constB = 20;
            break;

        case ROTARY_POS_6:
            PID_updateSettings(me->pidTorque, setpoint, 150);
            PID_updateSettings(me->pidTorque, frequency, 2);
            me->initialTorque = Rain_Guess_2;
            me->constA = 3;
            me->constB = 10;
            break;
    }
}

#endif