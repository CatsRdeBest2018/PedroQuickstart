package org.firstinspires.ftc.teamcode.drive.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;
import org.firstinspires.ftc.teamcode.robot.RobotContext;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.INTAKE_POWER_IN;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.BALL_PROX;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_OFF;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_MATIC;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE2;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE2_MATIC;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_ALL_THREE;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_LEFT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_RIGHT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_SIXTY;

import java.util.Deque;
import java.util.LinkedList;
import java.util.Objects;

@Disabled
@Configurable
@TeleOp(name = "Pedro in teleOp")
public class Tele_1_2 extends OpMode {

    Bob bob = new Bob();

    TelemetryManager telemetryM;

    Limelight3A limelight;
    private Timer macroTimer, actionTimer, opmodeTimer;
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    private PID rotationPID = new PID(2,0,.05);
    private boolean rotationCorrectionOn = false;
    private double rotationPower = 0;
    private double currentAngle = 0;
    private boolean transfer = false;
    private boolean intake = false;
    private double rotationTarget = 0;
    private double rotationErrorThresh = 0.05;
    private double rotationDerivativeThresh = 0.1;

    // Pedro
    private Pose startPose;
    private Follower follower;
    private Supplier<PathChain> pathChain;
    private boolean automatedDrive = false;

    // teleOp State
    private boolean intakeOn = false;
    private boolean isZoneOne = true;
    private boolean isMacroing = false;
    private int numBalls = 0;



    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        bob.init(hardwareMap);
        rotationPID.init(0);
        rotationPID.setTarget(rotationTarget);
        rotationPID.setDoneThresholds(rotationErrorThresh, rotationDerivativeThresh);
        actionTimer = new Timer();
        macroTimer = new Timer();

        startPose = Objects.requireNonNullElseGet(RobotContext.lastPose, () -> new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        bob.follower = follower;
    }

    @Override
    public void start() {
        bob.transferController.setDown();
        limelight.start();
        macroTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;

        follower.update();

        Pose currentPose = follower.getPose();
        telemetryM.debug("Pedro Pose:  "+String.format("x=%.2f in, y=%.2f in, h=%.1f deg", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));



//        telemetry.addData("current RPM:    ", bob.newShooterController.getCurrentRPM());
//        telemetry.addData("target RPM:   ", bob.newShooterController.getTargetRPM());
//        telemetry.update();

        telemetryM.debug("current RPM:    "+ bob.newShooterController.getCurrentRPM());
        telemetryM.debug("target RPM:   "+ bob.newShooterController.getTargetRPM());
        telemetryM.debug("Follower Busy: "+follower.isBusy());
        telemetryM.update(telemetry);
        // limelight tracking
        updateRotationCorrection();

        if (!automatedDrive) {
            drive();
        }


        //TODO: GAMEPAD1 CONTROLS (DRIVER)

        if (intakeOn && numBalls < 3) bob.intakeController.intake();
        else bob.intakeController.stopIntake();

        if (gamepad2.right_bumper && !lastGamepad2.right_bumper) {
            intakeOn = !intakeOn;
        }
        if (bob.isBall() &&
                bob.intakeController.getIntake() == INTAKE_POWER_IN &&
                actionTimer.getElapsedTimeSeconds() > .5 &&
                numBalls < 3
        ){
            numBalls++;
            actionTimer.resetTimer();
            bob.runMacro(SPINDEXER_RIGHT);
        }
        if (numBalls == 3){
            numBalls = 4;
            if (isZoneOne) bob.runMacro(SHOOTER_ZONE1_MATIC);
            else bob.runMacro(SHOOTER_ZONE2_MATIC);
        }
        //shooting all 3 balls
        if (gamepad1.y && !lastGamepad1.y) {
            bob.runMacro(SHOOT_ALL_THREE);
            Pose pose = follower.getPose();

            double targetX = 132;
            double targetY = 132;

            double targetHeading = Math.atan2(
                    targetY - pose.getY(),
                    targetX - pose.getX()
            );

            PathChain chain = follower.pathBuilder()
                    .addPath(
                            new BezierPoint(pose)
                    )
                    .setConstantHeadingInterpolation(targetHeading)
                    .build();

            follower.followPath(chain);
            automatedDrive = true;
            macroTimer.resetTimer();
            isMacroing = true;
        }
        if (macroTimer.getElapsedTimeSeconds() > 3) {
            isMacroing = false;
            automatedDrive = false;
            follower.breakFollowing();
            numBalls = 0;
        }
        if (!isMacroing) macroTimer.resetTimer();


        // TODO: GAMEPAD2 CONTROLS (GUNNER)
        //zone 1
        if (gamepad2.a && !lastGamepad2.a) isZoneOne = true;

        //zone 2
        if (gamepad2.y && !lastGamepad2.y) isZoneOne = false;

        if (gamepad1.dpad_up) {
            automatedDrive = true;
        } else if (lastGamepad1.dpadUpWasReleased()){
            automatedDrive = false;
            follower.breakFollowing();
        }

        if (gamepad1.dpad_up && !lastGamepad1.dpad_up) {
            Pose pose = follower.getPose();

            double targetX = 132;
            double targetY = 132;

            double targetHeading = Math.atan2(
                    targetY - pose.getY(),
                    targetX - pose.getX()
            );

            PathChain chain = follower.pathBuilder()
                    .addPath(
                            new BezierPoint(pose)
                    )
                    .setConstantHeadingInterpolation(targetHeading)
                    .build();

            follower.followPath(chain);
            automatedDrive = true;
        }



        bob.tick();
        gamepad1History.add(gamepad1);
        gamepad2History.add(gamepad2);
        if (gamepad1History.size() > 100) {
            gamepad1History.removeLast();
            gamepad2History.removeLast();
        }
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

    private void updateRotationCorrection() {
        if (rotationCorrectionOn) {

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                currentAngle = Math.toRadians(result.getTx());
                rotationPower = rotationPID.tick(currentAngle);
            } else {
                rotationPower = 0;
            }
        } else {
            rotationPower = 0;
        }

        telemetry.update();
    }
    private void drive(){
        if (!gamepad1.right_bumper && gamepad1.right_trigger <= 0.1) {
            // normal driving
            bob.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            rotationCorrectionOn = false;
        }
        else if (gamepad1.right_bumper) {
            // slow
            bob.motorDriveXYVectors(0.7 * gamepad1.left_stick_x, 0.7 * -gamepad1.left_stick_y, 0.3 * gamepad1.right_stick_x);
            rotationCorrectionOn = false;
        }
        else if (gamepad1.right_trigger > 0.1) {
            // limelight
            rotationCorrectionOn = true;
            bob.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, -rotationPower);

        }
    }

}
