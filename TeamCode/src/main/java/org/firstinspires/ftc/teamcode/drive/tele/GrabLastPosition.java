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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_X;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_Y;
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
@Disabled
@TeleOp(name = "Grab Last Position From Auto")
public class GrabLastPosition extends OpMode {

    TelemetryManager telemetryM;
    private final Bob bob = new Bob();
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();

    private Pose startPose;
    private Follower follower;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        startPose = new Pose(LAST_X,LAST_Y,LAST_HEADING);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        bob.follower = follower;

        bob.init(hardwareMap, false);
    }

    public void init_loop() {
        telemetryM.debug("start x: " + LAST_X);
        telemetryM.debug("start y: " + LAST_Y);
        telemetryM.debug("start heading: " + LAST_HEADING);
        telemetryM.update();
    }
    @Override
    public void start() {
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


        drive();
        bob.tick();
        gamepadUpdate();
    }

    private void drive(){
        if (!gamepad1.right_bumper && gamepad1.right_trigger <= 0.1 && gamepad1.left_trigger<=0.1) {
            // normal driving
            bob.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
        else if (gamepad1.right_bumper && gamepad1.left_trigger <= 0.1) {
            // slow
            bob.motorDriveXYVectors(0.85 * gamepad1.left_stick_x, 0.85 * -gamepad1.left_stick_y, 0.4 * gamepad1.right_stick_x);
        }
        else if (gamepad1.left_trigger >= 0.1) {
            // slow
            bob.motorDriveXYVectors(0.3 * gamepad1.left_stick_x, 0.3 * -gamepad1.left_stick_y, 0.3 * gamepad1.right_stick_x);
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
