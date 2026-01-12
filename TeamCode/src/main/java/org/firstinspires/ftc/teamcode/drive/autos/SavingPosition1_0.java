package org.firstinspires.ftc.teamcode.drive.autos;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_X;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_Y;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.INTAKE_THEN_SPIN;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_ALL_THREE_AUTO;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;
import org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants;

@Autonomous
public class SavingPosition1_0 extends OpMode {


    private Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)); // Start Pose of our robot.



    @Override
    public void loop() {
        follower.update();
        updateLastPosition();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {

        pathState = 0;
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    private void updateLastPosition(){
        Pose currentPose = follower.getPose();
        LAST_X = currentPose.getX();
        LAST_Y = currentPose.getY();
        LAST_HEADING = currentPose.getHeading();
        telemetry.addLine( "x: "+ LAST_X);
        telemetry.addLine( "y: "+ LAST_Y);
        telemetry.addLine( "heading: "+ LAST_HEADING);
        telemetry.update();
    }
}

