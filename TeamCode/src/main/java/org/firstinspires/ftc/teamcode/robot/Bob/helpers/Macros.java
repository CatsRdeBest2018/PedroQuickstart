package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.*;

public class Macros {
    // In Macros.java - Auto versions with absolute angles
    public static final BobState SHOOT_ALL_THREE_AUTO_FINAL = new BobState(null, 60.0, null, null, null, null);
    public static final BobState SHOOT_ALL_THREE_AUTO8 = new BobState(null, null, null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE_AUTO_FINAL));
    public static final BobState SHOOT_ALL_THREE_AUTO7 = new BobState(null, null, null, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 350, SHOOT_ALL_THREE_AUTO8));
    public static final BobState SHOOT_ALL_THREE_AUTO6 = new BobState(null, 120.0, null, null, null, new LinkedState(Link.LinkType.WAIT, 700, SHOOT_ALL_THREE_AUTO7));  // Absolute 240°
    public static final BobState SHOOT_ALL_THREE_AUTO5 = new BobState(null, null, null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE_AUTO6));
    public static final BobState SHOOT_ALL_THREE_AUTO4 = new BobState(null, null, null, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 350, SHOOT_ALL_THREE_AUTO5));
    public static final BobState SHOOT_ALL_THREE_AUTO3 = new BobState(null, 120.0, null, null, null, new LinkedState(Link.LinkType.WAIT, 700, SHOOT_ALL_THREE_AUTO4));  // Absolute 120°
    public static final BobState SHOOT_ALL_THREE_AUTO2 = new BobState(null, null, null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE_AUTO3));
    public static final BobState SHOOT_ALL_THREE_AUTO__ = new BobState(null, null, null, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 350, SHOOT_ALL_THREE_AUTO2));
    public static final BobState SHOOT_ALL_THREE_AUTO = new BobState(null, null, null, null, null, new LinkedState(Link.LinkType.WAIT, 50, SHOOT_ALL_THREE_AUTO__));

    public static final BobState SHOOT_ALL_THREE_FINAL = new BobState(0, 60.0,null, null, null, null);
    // shoot 3 ball
    public static final BobState SHOOT_ALL_THREE8 = new BobState(null, null,null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE_FINAL));
    public static final BobState SHOOT_ALL_THREE7 = new BobState(null, null, null,TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 340, SHOOT_ALL_THREE8));
    // spin to next ball
    public static final BobState SHOOT_ALL_THREE6 = new BobState(null, 120.0, null,null, null, new LinkedState(Link.LinkType.WAIT, 600, SHOOT_ALL_THREE7));
    //shoot 2 ball
    public static final BobState SHOOT_ALL_THREE5 = new BobState(null, null, null,TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE6));
    public static final BobState SHOOT_ALL_THREE4 = new BobState(null, null, null,TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 340, SHOOT_ALL_THREE5));
    // spin to next ball
    public static final BobState SHOOT_ALL_THREE3 = new BobState(null, 120.0,null, null, null, new LinkedState(Link.LinkType.WAIT, 600, SHOOT_ALL_THREE4));
    // shoot 1 ball
    public static final BobState SHOOT_ALL_THREE2 = new BobState(null, null,null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE3));
    public static final BobState SHOOT_ALL_THREE = new BobState(null, null,false, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 340, SHOOT_ALL_THREE2));

    // spin shooter zone1
    public static final BobState SHOOTER_ZONE1 = new BobState(RPM_ZONE1, null, null);
    //spin zone1 w/ shooter mode
    public static final BobState SHOOTER_ZONE2_MATIC = new BobState(RPM_ZONE2, 60.0, null);
    public static final BobState SHOOTER_ZONE1_MATIC = new BobState(RPM_ZONE1, 60.0, null);

    public static final BobState SHOOTER_ZONE1_AUTO = new BobState(RPM_ZONE1_AUTO, null, null);

    public static final BobState SHOOTER_ZONE1_AUTO_2 = new BobState(RPM_ZONE1_AUTO, -120.0, null);

    public static final BobState SHOOTER_ZONE1_AUTO_3 = new BobState(RPM_ZONE1_AUTO, 120.0, null);


    public static final BobState SHOOTER_ZONE1_AUTO_BOMBA = new BobState(RPM_ZONE1_AUTO, 60.0, null);

    public static final BobState SHOOTER_ZONE1_AUTO_2_BOMBA = new BobState(RPM_ZONE1_AUTO, -120.0+60.0, null);

    public static final BobState SHOOTER_ZONE1_AUTO_3_BOMBA = new BobState(RPM_ZONE1_AUTO, 120.0+60.0, null);

    // spin shooter zone2
    public static final BobState SHOOTER_ZONE2 = new BobState(RPM_ZONE2, null, null);

    // shooter off
    public static final BobState SHOOTER_OFF = new BobState(RPM_OFF, null, null);

    //spin right
    public static final BobState SPINDEXER_RIGHT = new BobState(null, 120.0, null);
    //spin left
    public static final BobState SPINDEXER_LEFT = new BobState(null, -120.0, null);
    //toggle shooter vs. intake
    public static final BobState SPINDEXER_SIXTY = new BobState(null, 60.0, null);

    //intake
    public static final BobState INTAKE_IN = new BobState(null, null, null,null, INTAKE_POWER_IN, null);
    //outtake
    public static final BobState INTAKE_OUT = new BobState(null, null, null,null, INTAKE_POWER_OUT, null);
    //stop intake
    public static final BobState INTAKE_STOP = new BobState(null, null,null, null, INTAKE_POWER_OFF, null);


    public static final BobState testy2 = new BobState(RPM_OFF, null, null);
    public static final BobState testy = new BobState(RPM_ZONE1, null, null,null,null, new LinkedState(Link.LinkType.WAIT, 1000, testy2));




    //    //intake macro
//    public static final BobState INTAKE_THEN_SPIN12 = new BobState(null, 60.0,null, null, INTAKE_POWER_OFF,null);
//    public static final BobState INTAKE_THEN_SPIN11 = new BobState(null, 120.0,null, null, null,new LinkedState(Link.LinkType.WAIT,750, INTAKE_THEN_SPIN12));
//    public static final BobState INTAKE_THEN_SPIN10 = new BobState(null, 120.0,null, null, null,new LinkedState(Link.LinkType.WAIT,750, INTAKE_THEN_SPIN11));
//    public static final BobState INTAKE_THEN_SPIN9 = new BobState(null, 60.0,null, null, INTAKE_POWER_IN-.1,new LinkedState(Link.LinkType.WAIT,750, INTAKE_THEN_SPIN10));
//    //spike mark 2
//    public static final BobState INTAKE_THEN_SPIN8 = new BobState(null, 60.0,null, null, INTAKE_POWER_OFF,new LinkedState(Link.LinkType.WAIT,9000, INTAKE_THEN_SPIN9));
//    public static final BobState INTAKE_THEN_SPIN7 = new BobState(null, 120.0,null, null, null,new LinkedState(Link.LinkType.WAIT,750, INTAKE_THEN_SPIN8));
//    public static final BobState INTAKE_THEN_SPIN6 = new BobState(null, 120.0,null, null, null,new LinkedState(Link.LinkType.WAIT,750, INTAKE_THEN_SPIN7));
//    public static final BobState INTAKE_THEN_SPIN5 = new BobState(null, 60.0,null, null, INTAKE_POWER_IN-.1,new LinkedState(Link.LinkType.WAIT,750, INTAKE_THEN_SPIN6));
    //spike mark 1
    public static final BobState INTAKE_THEN_SPIN4 = new BobState(null, 60.0,null, null, INTAKE_POWER_OFF,null/*new LinkedState(Link.LinkType.WAIT,9000, INTAKE_THEN_SPIN5)*/);
    public static final BobState INTAKE_THEN_SPIN3 = new BobState(null, 120.0,null, null, null, new LinkedState(Link.LinkType.WAIT,750,INTAKE_THEN_SPIN4));
    public static final BobState INTAKE_THEN_SPIN2 = new BobState(null, 120.0,null, null, null, new LinkedState(Link.LinkType.WAIT,750,INTAKE_THEN_SPIN3));
    public static final BobState INTAKE_THEN_SPIN = new BobState(null, null, null,null, INTAKE_POWER_IN-.1, new LinkedState(Link.LinkType.WAIT, 600, INTAKE_THEN_SPIN2));


    public static final BobState SHOOT_ALL_THREE_AUTO_FINAL2 = new BobState(null, 60.0, true, null, null, null);
    public static final BobState SHOOT_ALL_THREE_AUTO82 = new BobState(null, null, null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE_AUTO_FINAL2));
    public static final BobState SHOOT_ALL_THREE_AUTO72 = new BobState(null, null, null, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 300, SHOOT_ALL_THREE_AUTO82));
    public static final BobState SHOOT_ALL_THREE_AUTO62 = new BobState(null, 120.0, true, null, null, new LinkedState(Link.LinkType.WAIT, 500, SHOOT_ALL_THREE_AUTO72));  // Absolute 240°
    public static final BobState SHOOT_ALL_THREE_AUTO52 = new BobState(null, null, null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE_AUTO62));
    public static final BobState SHOOT_ALL_THREE_AUTO42 = new BobState(null, null, null, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 300, SHOOT_ALL_THREE_AUTO52));
    public static final BobState SHOOT_ALL_THREE_AUTO32 = new BobState(null, 120.0, true, null, null, new LinkedState(Link.LinkType.WAIT, 500, SHOOT_ALL_THREE_AUTO42));  // Absolute 120°
    public static final BobState SHOOT_ALL_THREE_AUTO22 = new BobState(null, null, null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE_AUTO32));
    public static final BobState SHOOT_ALL_THREE_AUTO_2 = new BobState(null, null, null, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 300, SHOOT_ALL_THREE_AUTO22));



}