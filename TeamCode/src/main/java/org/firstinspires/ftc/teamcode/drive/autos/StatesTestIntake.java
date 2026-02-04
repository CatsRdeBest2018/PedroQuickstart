package org.firstinspires.ftc.teamcode.drive.autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KD;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KD_A;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KI;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KI_A;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KP;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KP_A;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_ALL_THREE_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_2;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_2_BOMBA;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_3;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_3_BOMBA;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_AUTO_BOMBA;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_RIGHT;

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

@Autonomous(name = "intake test")
public class StatesTestIntake extends OpMode {
    private final Bob bob = new Bob();

    private boolean secondSpike = false;
    private double ramDistance = 10.0;
    private int checkpoint3 = 1;
    private boolean shootingAllThree1 = false;
    private int greenBallTarget = 1;
    private boolean waiting = false;
    Limelight3A limelight;
    private boolean isSpike1 = true;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private int intakeState = 0;
    private int ram = 0;
    private boolean waiting2 = false;
    private final Pose startPose = new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)); // Start Pose of our robot.
    private boolean finished = false;

    public static double offset = 0;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path4;
    public PathChain SpikeMark1;
    public PathChain SpikeMark2;
    public PathChain park;
    public PathChain ramPath;

    public PathChain SpikeCreep1;
    public PathChain SpikeCreep1_5;
    public PathChain SpikeCreep2;
    public PathChain SpikeCreep2_5;// (95,60.0)   -> (130,55)

    private void endAuto() {
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

        SpikeCreep1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(103.772, 83.692),

                                new Pose(118.000, 83.692)
                        )
                ).setTangentHeadingInterpolation()

                .build();
        SpikeCreep1_5 = follower.pathBuilder().addPath(
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


        SpikeMark1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(90.462, 83.692),

                                new Pose(102.662, 59.007)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))

                .build();
        SpikeCreep2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(102.662, 59.007),

                                new Pose(118, 59.007)
                        )
                ).setTangentHeadingInterpolation()

                .build();
        SpikeCreep2_5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(102.662, 59.007),

                                new Pose(127.46206896551725, 59.007)
                        )
                ).setTangentHeadingInterpolation()

                .build();


        SpikeMark2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.462, 59.007),

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

    public void ramThatFucker() {
        switch (ram) {
            case 0:
                follower.followPath(ramPath, 1, true);
                ram++;
                break;
            case 1:
                if (!follower.isBusy()) {
                    pathState = checkpoint3;
                }
                break;
        }
    }

    public void intakeSpikeMarks() {
        switch (intakeState) {
            case 0:
                // Start creeping once
                if (isSpike1) follower.followPath(SpikeCreep1, 0.4, true);
                else follower.followPath(SpikeCreep2, 0.4, true);

                actionTimer.resetTimer();
                secondSpike = true;
                setI(1);
                break;

            case 1:
                if (bob.isBall() || actionTimer.getElapsedTimeSeconds() > 2.5) {
                    bob.runMacro(SPINDEXER_RIGHT);
                    setI(2);
                }
                if (!follower.isBusy() && secondSpike) {
                    secondSpike = false;
                    if (isSpike1) follower.followPath(SpikeCreep1_5, 1, true);
                    else follower.followPath(SpikeCreep2_5, 1, true);
                }
                break;

            case 2:
            case 5:

                bob.spindexerController.setConsts(SPINDEX_KP, SPINDEX_KI, SPINDEX_KD);
                waitSpike(.5);
                if (!follower.isBusy() && secondSpike) {
                    secondSpike = false;
                    if (isSpike1) follower.followPath(SpikeCreep1_5, 1, true);
                    else follower.followPath(SpikeCreep2_5, 1, true);
                }
                break;

            case 3:

                if (bob.isBall() || actionTimer.getElapsedTimeSeconds() > 2.5) {
                    bob.runMacro(SPINDEXER_RIGHT);
                    setI(5);
                }
                if (!follower.isBusy() && secondSpike) {
                    secondSpike = false;
                    if (isSpike1) follower.followPath(SpikeCreep1_5, 1, true);
                    else follower.followPath(SpikeCreep2_5, 1, true);
                }
                break;

            case 6:

                if (bob.isBall() || actionTimer.getElapsedTimeSeconds() > 2.5) {
                    bob.runMacro(SPINDEXER_RIGHT);
                    setI(7);
                }
                if (!follower.isBusy() && secondSpike) {
                    secondSpike = false;
                    if (isSpike1) follower.followPath(SpikeCreep1_5, 1, true);
                    else follower.followPath(SpikeCreep2_5, 1, true);
                }
                break;

            case 7:

                if (isSpike1) {
                    pathState = 7;
                } else {
                    pathState = 15;
                }
                setI(-1);
                if (!follower.isBusy() && secondSpike) {
                    secondSpike = false;
                    if (isSpike1) follower.followPath(SpikeCreep1_5, 1, true);
                    else follower.followPath(SpikeCreep2_5, 1, true);
                }
                break;

            default:
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
                    shootingAllThree1 = true;
                    checkpoint3 = 1;
                    bob.runMacro(SHOOT_ALL_THREE_AUTO);
                    setP(2);
                }
                break;

            case 2:
            case 11:
            case 19:
                waitThenRun(3.3);
                break;

            case 3:
                shootingAllThree1 = false;
                ram = 0;
                follower.followPath(Path2);
                bob.intakeController.intake();
                setP(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    waitThenRun(.5);
                    bob.spindexerController.setConsts(SPINDEX_KP_A, SPINDEX_KI_A, SPINDEX_KD_A);
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

            case 17:
                waitThenRun(.5);
                break;

            case 10:
                if (!follower.isBusy()) {
                    shootingAllThree1 = true;
                    checkpoint3 = 10;
                    bob.runMacro(SHOOT_ALL_THREE_AUTO);
                    setP(11);
                }
                break;

            case 12:
                shootingAllThree1 = false;
                follower.followPath(SpikeMark1);
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
                    follower.followPath(SpikeMark2, 1, true);
                    setP(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    setP(17);
                }
                break;

            case 18:
                shootingAllThree1 = true;
                checkpoint3 = 18;
                bob.runMacro(SHOOT_ALL_THREE_AUTO);
                setP(19);
                break;

            case 20:
                shootingAllThree1 = false;
                follower.followPath(park);
                setP(-1);
                break;

            case 21:
                ramThatFucker();
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

        if (false) {
            Pose current = follower.getPose();
            Pose expected = new Pose(85, 85, Math.toRadians(45));
            double x = current.getX() - expected.getX();
            double y = current.getY() - expected.getY();
            double heading = Math.abs(current.getHeading() - expected.getHeading());
            boolean pushed = (Math.hypot(x, y) > 10.0) || heading > Math.toRadians(10.0);
            if (pushed) {
                bob.cancelMacros();
                waiting = false;

                double L = Math.sqrt((x * x) + (y * y));
                double newX = current.getX() + ramDistance * (x) / L;
                double newY = current.getY() + ramDistance * (y) / L;

                double newAngle = Math.tanh(y / x);

                double newX2 = 85 + ramDistance * ((85 - current.getX()) / L);
                double newY2 = 85 + ramDistance * ((85 - current.getY()) / L);

                ramPath = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(current.getX(), current.getY()), new Pose(newX, newY)))
                        .setLinearHeadingInterpolation(Math.toRadians(current.getHeading()), newAngle)
                        .addPath(new BezierLine(new Pose(newX, newY), new Pose(newX2, newY2)))
                        .setTangentHeadingInterpolation().setReversed()
                        .addPath(new BezierLine(new Pose(newX2, newY2), new Pose(85.000, 85.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(newAngle), Math.toRadians(45))
                        .build();

                pathState = 21;
            }
        }
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

    @Override
    public void init_loop() {
        obelisk();
        telemetry.addData("shoot green ball: ", greenBallTarget);
        telemetry.update();
    }

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
