package org.firstinspires.ftc.teamcode.drive.limelight;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

import java.util.List;

@Configurable
@TeleOp
public class TurretTele extends OpMode {
    Bob bob = new Bob();

    private double cameraHeight = 0; // inches
    private double tagHeight = 14; // inches
    private double heightDif = tagHeight-cameraHeight;
    private double distanceFromTag; // inches
    Limelight3A limelight;

    private Follower follower;
    GoBildaPinpointDriver pinpoint;
    public static int wait = 5;

    public static double startX = 108.939;
    public static double startY = 137.322;
    public static double startHeadingDeg = 270;

    public void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    @Override
    // runs on init press
    public void init() {
        bob.init(hardwareMap);
        // define and init robot
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        follower.update();

    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions
        limelight.start();
    }

    ElapsedTime lastRelocalized = new ElapsedTime();

    @SuppressLint("DefaultLocale")
    @Override
    // loops after start
    // press
    public void loop() {
        pinpoint.update();
        follower.update();

        Pose currentPose = follower.getPose();
        telemetry.addData("Pedro Pose", String.format("x=%.2f in, y=%.2f in, h=%.1f deg", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        Drawing.drawPoseHistory(follower.getPoseHistory());
        drawCurrent();

        if (!gamepad1.right_bumper && gamepad1.right_trigger <= 0.1) {
            bob.motorDriveXYVectors(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        }
        else if (gamepad1.right_bumper) {
            bob.motorDriveXYVectors(0.7 * -gamepad1.left_stick_x, 0.7 * gamepad1.left_stick_y, 0.3 * -gamepad1.right_stick_x);
        }
        double ppYaw = pinpoint.getHeading(AngleUnit.DEGREES);

        telemetry.addData("ppYaw", ppYaw);
        limelight.updateRobotOrientation(ppYaw+90);

        if (gamepad1.left_bumper){
            follower.setPose(new Pose(128.13769363166955,71.62822719449225,follower.getHeading()));
        }

        // LIMELIGHT
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            bob.turretController.update(result.getTx());
//            if (Math.abs(result.getTx()) < 0.5){
//                updateDistanceTag(result.getTy());
//            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();
    }
//    private void updateDistanceTag(double ty){
//        distanceFromTag = heightDif/Math.tan(ty);
//
//    }
}
