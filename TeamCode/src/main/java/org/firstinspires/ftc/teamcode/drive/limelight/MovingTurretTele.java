package org.firstinspires.ftc.teamcode.drive.limelight;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.KALMAN_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_X;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_Y;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
public class MovingTurretTele extends OpMode {
    Bob bob = new Bob();
    TelemetryManager telemetryM;
    private double targetAngle = 0;
    private double distanceToTarget = 0;
    private double horiVeliToTarget = 0;
    private double veliTowardTarget = 0;
    private double difX = 0;
    private final double cameraDistFromCenter = 3.6875;
    private double difY = 0;
    private double targetX = 134.64543889845095;
    private double targetY = 139.24612736660927;
    private double xPos = 0;
    private double yPos = 0;
    private double xVel = 0;
    private double yVel = 0;
    private final double timeInAir = 0.8;
    private final double cameraHeight = 11.25; // inches
    private final double tagHeight = 29.5;  // inches
    private final double heightDif = tagHeight-cameraHeight;
    Limelight3A limelight;

    private Follower follower;
    GoBildaPinpointDriver pinpoint;

    private Pose startPose;
    public static double startX = 108.939;
    public static double startY = 137.322;
    public static double startHeadingDeg = 270;

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
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        bob.init(hardwareMap);
        // define and init robot
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        startPose = new Pose(LAST_X,LAST_Y,LAST_HEADING);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

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
        drawCurrent();



        double ppYaw = pinpoint.getHeading(AngleUnit.DEGREES);

//        telemetry.addData("ppYaw", ppYaw);
        limelight.updateRobotOrientation(ppYaw+90);

        if (gamepad1.left_bumper){
            follower.setPose(new Pose(128.13769363166955,71.62822719449225,follower.getHeading()));
        }


        updateTargetAngle();
        // LIMELIGHT
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            bob.turretController.update(result.getTx());
            if (Math.abs(result.getTx()) < 0.5){
                updatePose3(result.getBotposeAvgDist());
            }

        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();
        telemetryM.update();
    }
    private void updatePose3(double dist){
        try {
            Pose currentFollowerPose = follower.getPose();
            double distanceFromTag = Math.sqrt(((dist*39.3701)*(dist*39.3701))- (heightDif*heightDif));
            double trueAngle = Math.toDegrees(follower.getHeading()) + bob.turretController.getTurretAngle();
            trueAngle = Math.toRadians(trueAngle);
            telemetryM.debug("limelight distance to target: " + distanceFromTag);
            // telemetry.addLine("RELATIVE turret angle: " + bob.turretController.getTurretAngle());
            // telemetry.addLine("RELATIVE turret ticks: " + bob.turretController.getTurretTicks());
            //  telemetry.addLine("pinpoint angle: " + Math.toDegrees(follower.getHeading()));
            // telemetry.addLine("TRUE turret angle: " + Math.toDegrees(trueAngle));
            if (Math.toDegrees(trueAngle) != 90) {
                double visionY = Math.sin(trueAngle) * distanceFromTag;
                double visionX = Math.cos(trueAngle) * distanceFromTag;
                visionX += Math.cos(trueAngle)*cameraDistFromCenter;
                visionY += Math.sin(trueAngle)*cameraDistFromCenter;
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
    private void updateTargetAngle(){
        xPos = follower.getPose().getX();
        yPos = follower.getPose().getY();
        xVel = follower.getVelocity().getXComponent();
        yVel = follower.getVelocity().getYComponent();
        difX = targetX - xPos;
        difY = targetY - yPos;
        distanceToTarget = Math.sqrt(difX*difX + difY*difY);
        horiVeliToTarget = xVel * (-difY / distanceToTarget) + (yVel * (difX / distanceToTarget));
        targetAngle = Math.toDegrees(Math.atan((-horiVeliToTarget*timeInAir)/distanceToTarget));
        bob.turretController.setTargetAngle(targetAngle);

        // for adjustable hood:
        veliTowardTarget = xVel * (difX / distanceToTarget) + yVel * (difY / distanceToTarget);
        telemetry.addLine("xVel: " + xVel);
        telemetry.addLine("yVel: " + yVel);
        telemetry.addLine("sideways velocity to target: " + horiVeliToTarget);
        telemetry.addLine("target angle: " + targetAngle);

        telemetryM.debug("sideways velocity to target: " + horiVeliToTarget);
        telemetryM.debug("target angle: " + targetAngle);

    }
}
