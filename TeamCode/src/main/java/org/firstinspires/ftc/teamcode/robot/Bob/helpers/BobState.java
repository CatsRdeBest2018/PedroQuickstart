package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

public class BobState {
    public Integer shooterRPM;
    public Double intakePower;
    public Double stopper;
    public LinkedState linkedState;

    public BobState(
                    Double stopper,
                    Double intakePower,
                    LinkedState linkedState) {
        this.stopper = stopper;
        this.intakePower = intakePower;
        this.linkedState = linkedState;
    }

}