package org.firstinspires.ftc.teamcode.drive.autos;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.BALL_PROX;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_2;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_2_BOMBA;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_3;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_3_BOMBA;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_BOMBA;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_ALL_THREE_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_RIGHT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_SIXTY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;
import org.firstinspires.ftc.teamcode.robot.RobotContext;

import java.util.List;

@Autonomous(name = "RedAuto", group = "DecodeQual1")
public class Auto_1_1 extends OpMode {
    private final Bob bob = new Bob();

    private int greenBallTarget = 1;
    private boolean waiting = false;
    Limelight3A limelight;
    private boolean isSpike1 = true;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int intakeState = 0;
    private boolean waiting2 = false;
    private final Pose startPose = new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)); // Start Pose of our robot.
    private boolean finished = false;

    public static double offset = 0;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path3_5;
    public PathChain Path3_5_5;
    public PathChain Path4;
    public PathChain SpikeMark2;
    public PathChain SpikeMark21;
    public PathChain SpikeMark22;
    public PathChain SpikeMark23;
    public PathChain SpikeMark24;
    public PathChain park;
    public PathChain Path100;

    private void endAuto() {
        bob.shooterController.setRPM(0);
        bob.cancelMacros();
        if (pathState != -1) setP(20);
        if (opmodeTimer.getElapsedTimeSeconds() > 29.9 || pathState == -1) savePose();
    }
    private void savePose() {
        if (finished) return;  // make it idempotent

        finished = true;
        RobotContext.lastPose = follower.getPose();
    }

    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.646, 9.354),
                                new Pose(85, 85)) //(72, 72)

                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85, 85), //(72, 72)
                                new Pose(83.938, 82.708),
                                new Pose(95, 83.692)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(95, 83.692),
                                new Pose(107, 83.692)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path3_5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(107, 83.692),
                                new Pose(112, 83.692)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Path3_5_5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(112, 83.692),
                                new Pose(125, 83.692)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(125, 83.692),
                                new Pose(89.600, 83.446),
                                new Pose(85, 85)
                        )
                )
                .setBrakingStrength(5)

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();
        SpikeMark2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(85, 85), new Pose(95, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();

        SpikeMark21 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(95, 60.000), new Pose(107.000, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        SpikeMark22 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(107.000, 60.000), new Pose(112.000, 60.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        SpikeMark23 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(112.000, 60.000), new Pose(130, 55))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        SpikeMark24 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130, 55), new Pose(85, 85))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(85, 85), new Pose(102.8923076923077, 66.21538461538462))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-15))
                .build();


    }

    public void waitSpike(double seconds) {
        if (!waiting2) {
            actionTimer.resetTimer();
            waiting2 = true;
        }
        if (seconds == 0 || actionTimer.getElapsedTimeSeconds() > seconds) {
            waiting2 = false;
            setI(intakeState + 1);
        }
    }

    public void waitThenRun(double seconds) {
        if (!waiting) {
            actionTimer.resetTimer();
            waiting = true;
        }
        if (seconds == 0 || actionTimer.getElapsedTimeSeconds() > seconds) {
            waiting = false;
            setP(getP() + 1);
        }
    }

    public void intakeSpikeMarks() {
        switch (intakeState) {
            case 0:
                if (isSpike1) follower.followPath(Path3, .35, true);
                else follower.followPath(SpikeMark21, .35, true);
                setI(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (bob.isBall() || actionTimer.getElapsedTimeSeconds() > .7) {
                        bob.runMacro(SPINDEXER_RIGHT);
                        setI(2);
                    }

                }
                break;
            case 2:
            case 5:
                waitSpike(.5);
                break;
            case 3:
                if (isSpike1) follower.followPath(Path3_5, .5, true);
                else follower.followPath(SpikeMark22, .5, true);
                setI(4);
                actionTimer.resetTimer();
                break;
            case 4:
                if (!follower.isBusy()) {
                    if (bob.isBall() || actionTimer.getElapsedTimeSeconds() > .7) {
                        bob.runMacro(SPINDEXER_RIGHT);
                        setI(5);
                    }
                }
                break;
            case 6:
                if (isSpike1) follower.followPath(Path3_5_5, .5, true);
                else follower.followPath(SpikeMark23, .5, true);
                setI(7);
                actionTimer.resetTimer();
                break;
            case 7:
                if (!follower.isBusy()) {
                    if (bob.isBall() || actionTimer.getElapsedTimeSeconds() > .7) {
                        bob.intakeController.stopIntake();
                        if (isSpike1) {
                            pathState = 7;
                        } else {
                            pathState = 15;
                        }
                        setI(-1);
                    }

                }
                break;

        }
    }

    public void autoMain() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1, 1, true);
                setP(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    bob.runMacro(SHOOT_ALL_THREE_AUTO);
                    setP(2);
                }
                break;
            case 2:
            case 11:
                waitThenRun(3.3);
                break;

            case 3:
                follower.followPath(Path2);
                bob.intakeController.intake();
                setP(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    waitThenRun(.5);
                }

                break;
            case 5:
            case 14:
                intakeSpikeMarks();
                break;


            case 7:
                if (!follower.isBusy()) {
                    switch (greenBallTarget) {
                        case 1:
                            bob.runMacro(SHOOTER_ZONE1_AUTO_3_BOMBA);
                            break;
                        case 2:
                            bob.runMacro(SHOOTER_ZONE1_AUTO_BOMBA);
                            break;
                        case 3:
                            bob.runMacro(SHOOTER_ZONE1_AUTO_2_BOMBA);
                            break;
                    }
                    follower.followPath(Path4, 1, true);
                    setP(8);

                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    setP(9);
                }
                break;
            case 9:
                waitThenRun(.5);

                break;
            case 10:
                if (!follower.isBusy()) {
                    bob.runMacro(SHOOT_ALL_THREE_AUTO);
                    setP(11);
                }
                break;

            case 12:
                follower.followPath(SpikeMark2);
                bob.intakeController.intake();
                setP(13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    intakeState = 0;
                    isSpike1 = false;
                    waitThenRun(.5);
                }

                break;


            case 15:
                if (!follower.isBusy()) {

                    switch (greenBallTarget) {
                        case 1:
                            bob.runMacro(SHOOTER_ZONE1_AUTO_BOMBA);
                            break;
                        case 2:
                            bob.runMacro(SHOOTER_ZONE1_AUTO_2_BOMBA);
                            break;
                        case 3:
                            bob.runMacro(SHOOTER_ZONE1_AUTO_3_BOMBA);
                            break;
                    }
                    follower.followPath(SpikeMark24, 1, true);
                    setP(16);

                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    setP(17);
                }
                break;
            case 17:
                waitThenRun(.5);

                break;
            case 18:
                bob.runMacro(SHOOT_ALL_THREE_AUTO);
                setP(19);

                break;
            case 19:
                waitThenRun(3.3);
                break;
            case 20:
                follower.followPath(park);
                setP(-1);


            default:
                break;
        }
    }


    public void setI(int i) {
        intakeState = i;
    }

    public void setP(int p) {
        pathState = p;
        pathTimer.resetTimer();
    }

    public int getP() {
        return pathState;
    }

    public void bigTick() {
        follower.update();
        autoMain();
        bob.tick();

    }

    @Override
    public void loop() {

        bigTick();
        if (pathState == -1 || opmodeTimer.getElapsedTimeSeconds() > 29) {
            endAuto();
        }

        telemetry.addData("there is ball: ", bob.isBall());
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        bob.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        bob.follower = follower;

        buildPaths();

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        obelisk();
        telemetry.addData("shoot green ball: ", greenBallTarget);
        telemetry.update();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        switch (greenBallTarget) {
            case 1:
                bob.runMacro(SHOOTER_ZONE1_AUTO);
                break;
            case 2:
                bob.runMacro(SHOOTER_ZONE1_AUTO_2);
                break;
            case 3:
                bob.runMacro(SHOOTER_ZONE1_AUTO_3);
                break;
        }


        bob.transferController.setDown();
        opmodeTimer.resetTimer();
        setP(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        endAuto();
    }

    private void obelisk() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                telemetry.addData("id: ", id);

                switch (id) {
                    case 21:
                        greenBallTarget = 1;
                        telemetry.addData("greenyINS: ", greenBallTarget);
                        break;
                    case 22:
                        greenBallTarget = 2;
                        telemetry.addData("greenyINS: ", greenBallTarget);
                        break;
                    case 23:
                        greenBallTarget = 3;
                        telemetry.addData("greenyINS: ", greenBallTarget);
                        break;
                }
            }
        }
    }
}
