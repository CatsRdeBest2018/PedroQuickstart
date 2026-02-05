package org.firstinspires.ftc.teamcode.drive.autos;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

@Disabled
@Autonomous
public class SK_Auto extends OpMode {


    private Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)); // Start Pose of our robot.

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path3_5;
    public PathChain Path3_5_5;
    public PathChain Path4;

    public void buildPaths() {

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(57.253, 64.688))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.253, 64.688),
                                new Pose(71.628, 98.892),
                                new Pose(104.840, 103.105)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                .build();
    
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;
            case 2:
                follower.followPath(Path2);
                pathState = 3;
                break;

            case 3:
                if (!follower.isBusy()) {
                    pathState = 3;
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
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

}

