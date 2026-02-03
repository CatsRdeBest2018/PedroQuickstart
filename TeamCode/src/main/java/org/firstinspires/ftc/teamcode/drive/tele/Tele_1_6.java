package org.firstinspires.ftc.teamcode.drive.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;
import org.firstinspires.ftc.teamcode.robot.RobotContext;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.INTAKE_POWER_IN;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.BALL_PROX;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LSERVO;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.RPM_ZONE1;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.RPM_ZONE2;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KD;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KD_A;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KI;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KI_A;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KP;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.SPINDEX_KP_A;
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
import java.util.function.Supplier;

@TeleOp(name = "States TeleOp (Red)")
public class Tele_1_6 extends OpMode {

    TelemetryManager telemetryM;
    private final Bob bob = new Bob();
    Limelight3A limelight;
    private Timer macroTimer, actionTimer, opmodeTimer;
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    private PID rotationPID = new PID(1.5,0,.08);
    private boolean rotationCorrectionOn = false;
    private double rotationPower = 0;
    private double currentAngle = 0;
    private boolean transfer = false;
    private boolean intake = false;
    private double rotationTarget = 0;
    private double rotationErrorThresh = 0.05;
    private double rotationDerivativeThresh = 0.1;

    // teleOp State
    private boolean intakeOn = false;
    private boolean isZoneOne = true;
    private boolean isMacroing = false;
    private int numBalls = 0;

    // Pedro
    private Pose startPose;
    private Follower follower;

    private Supplier<PathChain> pathChain;
    private boolean automatedDrive = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(6);

        rotationPID.init(0);
        rotationPID.setTarget(rotationTarget);
        rotationPID.setDoneThresholds(rotationErrorThresh, rotationDerivativeThresh);
        actionTimer = new Timer();
        macroTimer = new Timer();

        // pedro
        //  startPose = Objects.requireNonNullElseGet(RobotContext.lastPose, () -> new Pose(95, 37, Math.toRadians(0)));
        startPose = Objects.requireNonNullElseGet(null, () ->
                new Pose(102.8923076923077, 66.21538461538462, Math.toRadians(-15)));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        bob.follower = follower;

        bob.init(hardwareMap, false);
    }

    public void init_loop() {
        telemetryM.debug("LastPose: "+RobotContext.lastPose);
        telemetryM.update();
    }
    @Override
    public void start() {
        bob.spindexerController.setConsts(SPINDEX_KP_A,SPINDEX_KI_A,SPINDEX_KD_A);
        bob.transferController.setDown();
        limelight.start();
        macroTimer.resetTimer();
        bob.spindexerController.incrementAngle(60);
    }

    @Override
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;

        follower.update();
        Pose currentPose = follower.getPose();
        telemetryM.debug("c1: "+ bob.c.getDistance(DistanceUnit.MM));
        telemetryM.debug("c2: "+ bob.c2.getDistance(DistanceUnit.MM));
        telemetryM.debug("c3: "+ bob.c3.getDistance(DistanceUnit.MM));
        telemetryM.debug("Pedro Pose:  "+String.format("x=%.2f in, y=%.2f in, h=%.1f deg", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));


        telemetryM.update(telemetry);


        updateRotationCorrection();
        drive();
        if (!gamepad2.right_bumper) autoControl();
        else manualControl();

        bob.tick();
        gamepadUpdate();
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
        if (!gamepad1.right_bumper && gamepad1.right_trigger <= 0.1 && gamepad1.left_trigger<=0.1) {
            // normal driving
            bob.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            rotationCorrectionOn = false;
        }
        else if (gamepad1.right_bumper && gamepad1.left_trigger <= 0.1) {
            // slow
            bob.motorDriveXYVectors(0.85 * gamepad1.left_stick_x, 0.85 * -gamepad1.left_stick_y, 0.4 * gamepad1.right_stick_x);
            rotationCorrectionOn = false;
        }
        else if (gamepad1.left_trigger >= 0.1) {
            // slow
            bob.motorDriveXYVectors(0.3 * gamepad1.left_stick_x, 0.3 * -gamepad1.left_stick_y, 0.3 * gamepad1.right_stick_x);
            rotationCorrectionOn = false;
        }
        else if (gamepad1.right_trigger > 0.1) {
            // limelight
            rotationCorrectionOn = true;
            bob.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, rotationPower);

        }
    }
    private void manualControl(){
        if (gamepad2.left_bumper && !lastGamepad2.left_bumper) {
            transfer = !transfer;
            if (transfer) bob.transferController.setUp();
            else bob.transferController.setDown();
        }

        if (gamepad2.x) bob.intakeController.intake();
        else if (gamepad2.b) bob.intakeController.outtake();
        else bob.intakeController.stopIntake();

        // TODO: GAMEPAD2 CONTROLS (GUNNER)
        //zone 1
        if (gamepad2.a && !lastGamepad2.a) bob.runMacro(SHOOTER_ZONE1);
        //zone 2
        if (gamepad2.y && !lastGamepad2.y) bob.runMacro(SHOOTER_ZONE2);

        //stop shooter
        if (gamepad2.b && !lastGamepad2.b) bob.runMacro(SHOOTER_OFF);
        // spin 120
        if (gamepad2.dpad_right && !lastGamepad2.dpad_right) bob.runMacro(SPINDEXER_RIGHT);
        // spin -120
        if (gamepad2.dpad_left && !lastGamepad2.dpad_left) bob.runMacro(SPINDEXER_LEFT);
        // spin 60
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up) bob.runMacro(SPINDEXER_SIXTY);

    }
    private void autoControl(){

        bob.updateLight(numBalls);
        bob.lservo.setPosition(LSERVO);
        //TODO: GAMEPAD1 CONTROLS (DRIVER)

        if (intakeOn && numBalls < 3 && !gamepad2.b) bob.intakeController.intake();
        else if (gamepad2.b) bob.intakeController.outtake();
        else bob.intakeController.stopIntake();

        if (gamepad2.x && !lastGamepad2.x) {
            intakeOn = !intakeOn;
        }
        if (bob.isBall() &&
                bob.intakeController.getIntake() == INTAKE_POWER_IN &&
                actionTimer.getElapsedTimeSeconds() > .5 &&
                numBalls < 3
        ){
            bob.spindexerController.setConsts(SPINDEX_KP,SPINDEX_KI,SPINDEX_KD);
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
        if (((gamepad2.left_bumper && !lastGamepad2.left_bumper) || (gamepad1.left_bumper && !lastGamepad1.left_bumper)) &&
                numBalls < 3) {
            numBalls = 3;
        }
        if (((gamepad2.left_bumper && !lastGamepad2.left_bumper) || (gamepad1.left_bumper && !lastGamepad1.left_bumper)) &&
                numBalls == 4) {
            bob.runMacro(SHOOT_ALL_THREE);
            macroTimer.resetTimer();
            isMacroing = true;
        }
        if (macroTimer.getElapsedTimeSeconds() > 3) {
            isMacroing = false;
            numBalls = 0;
            bob.spindexerController.setConsts(SPINDEX_KP_A,SPINDEX_KI_A,SPINDEX_KD_A);
        }
        if (!isMacroing) macroTimer.resetTimer();

        // TODO: GAMEPAD2 CONTROLS (GUNNER)
        //zone 1
        if (gamepad2.a && !lastGamepad2.a) {
            isZoneOne = true;
            if (bob.newShooterController.getTargetRPM() == RPM_ZONE2){
                bob.newShooterController.setRPM(RPM_ZONE1);
            }
        }

        //zone 2
        if (gamepad2.y && !lastGamepad2.y) {
            isZoneOne = false;
            if (bob.newShooterController.getTargetRPM() == RPM_ZONE1){
                bob.newShooterController.setRPM(RPM_ZONE2);
            }
        }
    }
    private void gamepadUpdate(){
        gamepad1History.add(gamepad1);
        gamepad2History.add(gamepad2);
        if (gamepad1History.size() > 100) {
            gamepad1History.removeLast();
            gamepad2History.removeLast();
        }
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }


}
