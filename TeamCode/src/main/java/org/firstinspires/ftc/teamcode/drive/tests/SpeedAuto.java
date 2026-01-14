package org.firstinspires.ftc.teamcode.drive.tests;

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
public class SpeedAuto extends OpMode {


    private Follower follower;
    private int pathState;

    private final Pose startPose = new Pose(117.32413793103447, 128.9103448275862, Math.toRadians(45)); // Start Pose of our robot.

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(117.324, 128.910),

                                new Pose(84.703, 92.028)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.703, 92.028),
                                new Pose(102.879, 83.069),
                                new Pose(103.152, 83.386),
                                new Pose(120.607, 83.421)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(120.607, 83.421),
                                new Pose(102.352, 81.838),
                                new Pose(84.483, 92.131)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.483, 92.131),
                                new Pose(90.355, 54.290),
                                new Pose(92.852, 57.821),
                                new Pose(132.283, 58.214)
                        )
                ).setTangentHeadingInterpolation()

                .build();
        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.283, 58.214),
                                new Pose(102.848, 73.145),
                                new Pose(84.310, 91.566)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.310, 91.566),
                                new Pose(101.348, 71.345),
                                new Pose(111.817, 54.690),
                                new Pose(125.614, 59.579)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.614, 59.579),
                                new Pose(109.400, 53.738),
                                new Pose(92.786, 82.317),
                                new Pose(84.524, 91.593)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
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
                    follower.followPath(Path2);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(Path3);
                    pathState = 3;
                }
                break;

            case 3:

                if (!follower.isBusy()) {
                    follower.followPath(Path4);
                    pathState = 4;
                }
                break;
            case 4:

                if (!follower.isBusy()) {
                    follower.followPath(Path5);
                    pathState = 5;
                }

                break;
            case 5:

                if (!follower.isBusy()) {
                    follower.followPath(Path6);
                    pathState = 6;
                }

                break;
            case 6:

                if (!follower.isBusy()) {
                    follower.followPath(Path7);
                    pathState = 5;
                }

                break;

            default:
                break;
        }
    }
    @Override
    public void loop() {
        autonomousPathUpdate();
        follower.update();
        updateLastPosition();

    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
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

