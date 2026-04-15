package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.*;

public class Macros {

    public static final BobState SHOOT_THREE_DONE = new BobState(STOPPER_STOP,INTAKE_POWER_OFF,null);
    public static final BobState SHOOT_THREE = new BobState(STOPPER_STARTING_POS,INTAKE_POWER_IN,new LinkedState(Link.LinkType.WAIT, 1500, SHOOT_THREE_DONE));



}