package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.HOOD_STARTING_POS;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.tD;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.tF;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.tI;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.tP;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class BobConfigure {
    @Configurable
    public static class Position{
        public static boolean SHOW_POSITION = false;
        public static boolean LL_LOCALIZATION = false;
        public static double DISTANCE_TO_TARGET = 0;
    }
    @Configurable
    public static class Shooter{
        public static boolean SHOOTER_ON = false;
        public static boolean USE_DISTANCE = false;
        public static double P = BobConstants.P;
        public static double I = BobConstants.I;
        public static double D = BobConstants.D;
        public static double F = BobConstants.F;
        public static double TARGET_RPM = 0;

    }
    @Configurable
    public static class Turret{
        public static boolean TURRET_ON = false;
        public static double P = tP;
        public static double I = tI;
        public static double D = tD;
        public static double F = tF;
    }
    @Configurable
    public static class Hood{
        public static boolean HOOD_ON = false;
        public static boolean USE_DISTANCE = false;
        public static double HOOD_POS = HOOD_STARTING_POS;

    }
    @Configurable
    public static class Intake{
        public static boolean INTAKE_ON = false;
        public static double INTAKE_POWER = 0;

    }
}