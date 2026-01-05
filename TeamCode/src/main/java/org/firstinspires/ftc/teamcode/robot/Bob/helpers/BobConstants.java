package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class BobConstants {


    // ==================== SHOOTER ====================
    public static double TICKS_PER_REV_SHOOTER = 28;
    public static int RPM_ZONE1 = 2570;
    public static int RPM_ZONE1_AUTO = 2500;
    public static int RPM_ZONE2 = 3200;
    public static int RPM_OFF = 0;
    // ZONE 1 PID Values
    public static double SHOOTER_P_Z1 = 0.012;
    public static double SHOOTER_I_Z1 = 0;
    public static double SHOOTER_D_Z1 = 0.0000004;
    // ZONE 2 PID Values
    public static double SHOOTER_P_Z2 = 0.014;
    public static double SHOOTER_I_Z2 = 0.0006;
    public static double SHOOTER_D_Z2 = 0.0000004;

    // PIDF SHOOTER VALUES
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;
    public static double TARGET_RPM = 0;


    // ==================== SPINDEXER ====================
    public static double TICKS_PER_REV_SPINDEXER = 8192;
    public static double SPINDEX_KP = 0.00032;
    public static double SPINDEX_KI = 0;
    public static double SPINDEX_KD = 0.00005;
//    public static double SPINDEX_KP = 0.0003;
//    public static double SPINDEX_KI = 0.00003;
//    public static double SPINDEX_KD = 0.00007;

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
    public static double INTAKE_POWER_IN = -0.7;
    public static double INTAKE_POWER_OUT = 0.7;
    public static double INTAKE_POWER_OFF = 0;
    public static double BALL_PROX = 20;


    // ==================== MISC ====================
    public static int INFINITY = 2000000000;
    public static double LIGHT3 = 1.0;
    public static double LIGHT2 = 0.66;
    public static double LIGHT1 = 0.33;
    public static double LIGHT0 = 0.0;
    public static double LSERVO = 0.0;
}