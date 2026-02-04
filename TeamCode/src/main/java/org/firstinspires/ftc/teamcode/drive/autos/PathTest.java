package org.firstinspires.ftc.teamcode.drive.autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.BALL_PROX;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KD;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KD_A;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KI;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KI_A;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KP;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KP_A;
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

@Autonomous(name = "PathTest")
public class PathTest extends OpMode {
    private final Bob bob = new Bob();

    private double ramDistance = 20.0;
    private int checkpoint3 = 1;
    private boolean shootingAllThree1 = false;
    private int greenBallTarget = 1;
    private boolean waiting = false;
    Limelight3A limelight;
    private boolean isSpike1 = true;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, correctionTimer;
    private int pathState;
    private int intakeState = 0;
    private int ram = 0;
    private boolean waiting2 = false;
    private final Pose startPose = new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)); // Start Pose of our robot.
    private boolean finished = false;

    public static double offset = 0;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain park;

    private void endAuto() {
        //bob.shooterController.setRPM(0);
        bob.cancelMacros();
        if (pathState != -1) setP(20);
        if (opmodeTimer.getElapsedTimeSeconds() > 29.9 || pathState == -1) savePose();
    }
    private void savePose() {
        if (finished) return;

        finished = true;
        RobotContext.lastPose = follower.getPose();
    }

    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(86.892, 9.354),

                                new Pose(90.462, 83.692)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(48))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.462, 83.692),

                                new Pose(103.772, 83.692)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(103.772, 83.692),

                                new Pose(125.000, 83.692)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.000, 83.692),

                                new Pose(90.462, 83.692)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.462, 83.692),

                                new Pose(102.662, 59.007)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(102.662, 59.007),

                                new Pose(129.200, 59.007)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.200, 59.007),

                                new Pose(90.462, 83.692)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))

                .build();

        park = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.462, 83.692),

                                new Pose(97.345, 77.083)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(48))

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
                if (isSpike1) follower.followPath(Path3, 0.25, true);
                else follower.followPath(Path6, 0.25, true);
                actionTimer.resetTimer();
                setI(1);
                break;

            case 1:
                    if (bob.isBall() || actionTimer.getElapsedTimeSeconds() > 2) {
                        bob.runMacro(SPINDEXER_RIGHT);
                        actionTimer.resetTimer();
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
                    follower.followPath(Path2, 1, true);
                    setP(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(Path3, 0.25, true);
                    setP(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Path4, 1, true);
                    setP(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path5, 1, true);
                    setP(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Path6, 1, true);
                    setP(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(Path7, 0.25, true);
                    setP(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(park, 1, true);
                    setP(8);
                }
                break;

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
        telemetry.update();

    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        correctionTimer = new Timer();

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
