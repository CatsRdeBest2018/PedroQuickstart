package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class BobConstants {

    // ==================== SAVING AUTO POSITION ====================
    public static double LAST_X = 0;
    public static double LAST_Y = 0;
    public static double LAST_HEADING = 0;


    // ==================== SHOOTER ====================
    public static double TICKS_PER_REV_SHOOTER = 28;
    public static int RPM_ZONE1 = 2570;
    public static int RPM_ZONE1_AUTO = 2500;
    public static int RPM_ZONE2 = 3200;
    public static int RPM_OFF = 0;


    // ==================== PIDF SHOOTER ====================
    public static double P = 2;
    public static double I = 0.4;
    public static double D = 0;
    public static double F = 1;
    public static double TARGET_RPM = 0;

    // ==================== PIDF TURRET ====================
    public static double TICKS_PER_REV_TURRET = 1600;
    public static double KALMAN_TURRET = 0.1;
    public static double tP = 0.018;
    public static double tI = 0.02;
    public static double tD = 0.0014;
    public static double tF = 1;
    public static double DISTANCE_FROM_TARGET = 0;

    // ==================== HOOD CONTROL ====================
    public static double HOOD_STARTING_POS = 0.5;

    // ==================== STOPPER CONTROL ====================
    public static double STOPPER_STARTING_POS = 0.7;
    public static double STOPPER_STOP = 0.95;

    // ==================== SPINDEXER ====================
    public static double TICKS_PER_REV_SPINDEXER = 8192;
    public static double SPINDEX_KP = 0.00032;
    public static double SPINDEX_KI = 0;
    public static double SPINDEX_KD = 0.00005;

    public static double SPINDEX_KP_A = 0.00014;
    public static double SPINDEX_KI_A = 0.000009;
    public static double SPINDEX_KD_A = 0.000035;

    public static double SPINDEX_EMERGENCY_POWER_LIMIT = 0.6;
    public static double SPINDEX_EMERGENCY_POWER_LIMIT_A = 0.3;


    // ==================== TRANSFER ====================
    public static double TRANSFER_UP = 0.6;
    public static double TRANSFER_DOWN = 0.1;
    public static double TRANSFER_PULSE_TIME = 300; // milliseconds

    // ==================== INTAKE ====================
    public static double INTAKE_POWER_IN = 1;
    public static double INTAKE_POWER_OUT = -1;
    public static double INTAKE_POWER_OFF = 0;
    public static double BALL_PROX = 20;

    // ==================== PTO ====================

    public static boolean PTO_ENGAGED = false;
    public static boolean PTO_MESSAGE = false;
    public static boolean PTO_RUNNING = false;

    // ==================== MISC ====================
    public static int INFINITY = 2000000000;
    public static double LIGHT3 = 1.0;
    public static double LIGHT2 = 0.66;
    public static double LIGHT1 = 0.33;
    public static double LIGHT0 = 0.0;
    public static double LSERVO = 0.0;

    // LIFT
    public static double SERVO_ENGAGED_LEFT = 0.05;
    public static double SERVO_ENGAGED_RIGHT = 0;

    public static double taP = 0.65;
    public static double taI = 0.5;
    public static double taD = 0;
    public static double taF = 0;
}