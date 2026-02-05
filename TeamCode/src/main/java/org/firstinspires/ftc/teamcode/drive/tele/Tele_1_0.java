package org.firstinspires.ftc.teamcode.drive.tele;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

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

@Disabled
@TeleOp(name = "1.0 - normal control")
public class Tele_1_0 extends OpMode {

    Bob bob = new Bob();

    Limelight3A limelight;
    private Timer pathTimer, actionTimer, opmodeTimer;
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

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        bob.init(hardwareMap);
        rotationPID.init(0);
        rotationPID.setTarget(rotationTarget);
        rotationPID.setDoneThresholds(rotationErrorThresh, rotationDerivativeThresh);
        actionTimer = new Timer();
    }

    @Override
    public void start() {
        bob.transferController.setDown();
        limelight.start();

    }

    @Override
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;
        // limelight tracking
        updateRotationCorrection();

        //TODO: GAMEPAD1 CONTROLS (DRIVER)

        if (gamepad2.x && !lastGamepad2.x) {
            transfer = !transfer;

            if (transfer) bob.transferController.setUp();
            else bob.transferController.setDown();
        }
        if (gamepad2.right_bumper && !lastGamepad2.right_bumper) {
            intake = !intake;

            if (intake) bob.intakeController.intake();
            else bob.intakeController.stopIntake();
        }
        if (bob.isBall() &&
                bob.intakeController.getIntake() == INTAKE_POWER_IN &&
                actionTimer.getElapsedTimeSeconds() > .5
        ){
            actionTimer.resetTimer();
            bob.runMacro(SPINDEXER_RIGHT);
        }

        if (!gamepad1.right_bumper && gamepad1.right_trigger <= 0.1) {
            // normal driving
            bob.motorDriveXYVectors(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            rotationCorrectionOn = false;
        }
        else if (gamepad1.right_bumper) {
            // slow
            bob.motorDriveXYVectors(0.7 * -gamepad1.left_stick_x, 0.7 * gamepad1.left_stick_y, 0.3 * -gamepad1.right_stick_x);
            rotationCorrectionOn = false;
        }
        else if (gamepad1.right_trigger > 0.1) {
            // limelight
            rotationCorrectionOn = true;
            bob.motorDriveXYVectors(-gamepad1.left_stick_x, gamepad1.left_stick_y, rotationPower);

        }


        //shooting all 3 balls
        if (gamepad1.y && !lastGamepad1.y) bob.runMacro(SHOOT_ALL_THREE);
        if (gamepad1.left_bumper && !lastGamepad1.left_bumper) bob.runMacro(SHOOTER_ZONE1_MATIC);
        if (gamepad1.dpad_up && !lastGamepad1.dpad_up) bob.runMacro(SHOOTER_ZONE2_MATIC);



        //we got ball into spindexer
        if (gamepad1.x && !lastGamepad1.x) bob.runMacro(SPINDEXER_RIGHT);

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
}