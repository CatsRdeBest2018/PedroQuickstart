package org.firstinspires.ftc.teamcode.drive.limelight;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.KALMAN_TURRET;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;


@Configurable
@TeleOp
public class TurretTele extends OpMode {
    Bob bob = new Bob();

    private final double cameraHeight = 11.25; // inches
    private final double tagHeight = 29.5; // inches
    private final double heightDif = tagHeight-cameraHeight;
    Limelight3A limelight;

    private Follower follower;
    GoBildaPinpointDriver pinpoint;


    public void drawCurrent() {
        try {
//            Drawing.drawRobot(follower.getPose());
//            Drawing.sendPacket();
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
        follower.setStartingPose(new Pose(86.89230769230768, 9.353846153846153, Math.toRadians(90)));
        follower.update();

    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions
        limelight.start();
    }


    @SuppressLint("DefaultLocale")
    @Override
    // loops after start
    // press
    public void loop() {
        pinpoint.update();
        follower.update();

        Pose currentPose = follower.getPose();
        telemetry.addData("Pedro Pose", String.format("x=%.2f in, y=%.2f in, h=%.1f deg", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        //Drawing.drawPoseHistory(follower.getPoseHistory());
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
            if (Math.abs(result.getTx()) < 0.5){
//                updatePose(result.getTy())
                updatePose2(result.getBotposeAvgDist());
            }

        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();
    }
    private void updatePose(double ty){
        try {
            Pose currentFollowerPose = follower.getPose();
            double distanceFromTag = heightDif / Math.tan(Math.toRadians(ty));
            double trueAngle = 90-Math.toDegrees(follower.getHeading()) + bob.turretController.getTurretAngle();
            trueAngle = Math.toRadians(trueAngle);
            telemetry.addLine("limelight distance to target: " + distanceFromTag);
            telemetry.addLine("RELATIVE turret angle: " + bob.turretController.getTurretAngle());
            telemetry.addLine("RELATIVE turret ticks: " + bob.turretController.getTurretTicks());
            telemetry.addLine("pinpoint angle: " + Math.toDegrees(follower.getHeading()));
            telemetry.addLine("TRUE turret angle: " + trueAngle);
            if (trueAngle != 90) {
                double visionY = Math.sin(trueAngle) * distanceFromTag;
                double visionX = Math.cos(trueAngle) * distanceFromTag;
                visionX = 127.628 - visionX;
                visionY = 131.669 - visionY;
                double finalX = currentFollowerPose.getX() + KALMAN_TURRET * (visionX - currentFollowerPose.getX());
                double finalY = currentFollowerPose.getY() + KALMAN_TURRET * (visionY - currentFollowerPose.getY());
                follower.setPose(new Pose(finalX, finalY, currentFollowerPose.getHeading()));
            }
        }
        catch (Exception e) {
            telemetry.addData("Limelight error", e.getMessage());
        }
    }
    private void updatePose2(double dist){
        try {
            Pose currentFollowerPose = follower.getPose();
            double distanceFromTag = dist;
            double trueAngle = Math.toDegrees(follower.getHeading()) + bob.turretController.getTurretAngle();
            trueAngle = Math.toRadians(trueAngle);
            telemetry.addLine("limelight distance to target: " + distanceFromTag);
            telemetry.addLine("RELATIVE turret angle: " + bob.turretController.getTurretAngle());
            telemetry.addLine("RELATIVE turret ticks: " + bob.turretController.getTurretTicks());
            telemetry.addLine("pinpoint angle: " + Math.toDegrees(follower.getHeading()));
            telemetry.addLine("TRUE turret angle: " + trueAngle);
            if (trueAngle != 90) {
                double visionY = Math.sin(trueAngle) * distanceFromTag;
                double visionX = Math.cos(trueAngle) * distanceFromTag;
                visionX = 127.628 - visionX;
                visionY = 131.669 - visionY;
                double finalX = currentFollowerPose.getX() + KALMAN_TURRET * (visionX - currentFollowerPose.getX());
                double finalY = currentFollowerPose.getY() + KALMAN_TURRET * (visionY - currentFollowerPose.getY());
                follower.setPose(new Pose(finalX, finalY, currentFollowerPose.getHeading()));
            }
        }
        catch (Exception e) {
            telemetry.addData("Limelight error", e.getMessage());
        }
    }
}
