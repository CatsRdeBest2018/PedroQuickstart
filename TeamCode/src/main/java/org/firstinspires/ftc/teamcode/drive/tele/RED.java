package org.firstinspires.ftc.teamcode.drive.tele;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.PTO.FrontTwoWheelsPower;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.PTO.PTO_In_Left;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.PTO.PTO_In_Right;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.KALMAN_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_X;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_Y;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.PTO_ENGAGED;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.PTO_MESSAGE;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.PTO_RUNNING;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.PTO_STOP;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.STOPPER_STOP;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_THREE;

import android.annotation.SuppressLint;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;



@TeleOp
public class RED extends OpMode {
    Bob bob = new Bob();
    TelemetryManager telemetryM;
    private double xPos = 0;
    private double yPos = 0;
    private double xVel = 0;
    private double yVel = 0;
    private final double timeInAir = 0.6;
    private double horiVeliToTarget = 0;
    private double veliTowardTarget = 0;
    private double targetAngle = 0;
    private double distanceToTarget = 0;
    private double targetX = 134.64543889845095;
    private double targetY = 139.24612736660927;
    private double odoDistance = 0;
    private double difX = 0;

    private double difY = 0;
    private boolean isShooting = false;
    private double shootingSpeed = 1.0;
    private Timer shootTimer, PTOTimer, begin, limelightTimer;
    private double currentRPM = 0;
    Limelight3A limelight;
    private Follower follower;
    private Pose startPose;
    private double limeDist = 1;
    @Override
    public void init() {
        bob.init(hardwareMap);
        bob.stopper.setPosition(STOPPER_STOP);
        PTO_MESSAGE = false;
        PTO_ENGAGED = false;
        PTO_RUNNING = false;
        PTO_STOP = false;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        startPose = new Pose(LAST_X,LAST_Y,LAST_HEADING);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        shootTimer = new Timer();
        PTOTimer = new Timer();
        begin = new Timer();
        limelightTimer = new Timer();
        begin.resetTimer();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

    }
    @Override
    public void start() {
        limelight.start();
        follower.update();
        follower.startTeleopDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override

    public void loop() {
        bob.tickMacros();

       // Position();
        Turret();
        updateTargetAngle();
        Shooter();
        Hood();
        Stopper();
        Intake();
        PTO();
        FrontTwoWheels();


        if (!gamepad1.right_bumper) follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        else follower.setTeleOpDrive(-gamepad1.left_stick_y*0.5, -gamepad1.left_stick_x*0.5, -gamepad1.right_stick_x*0.5, true);
        follower.update();
        telemetryM.update();
    }
    private void updatePose3(double dist){
        final double cameraDistFromCenter = 3.6875;
        final double cameraHeight = 10.76; // inches
        final double tagHeight = 29.5; // inches
        final double heightDif = tagHeight-cameraHeight;
        try {
            Pose currentFollowerPose = follower.getPose();
            double distanceFromTag = Math.sqrt(((dist*39.3701)*(dist*39.3701))- (heightDif*heightDif));
            double trueAngle = Math.toDegrees(follower.getHeading()) + bob.turretController.getTurretAngle();
            trueAngle = Math.toRadians(trueAngle);
            telemetryM.debug("limelight distance to target: " + distanceFromTag);
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
            telemetryM.addData("Limelight error", e.getMessage());
        }
    }

    private void Position(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (Math.abs(result.getTx()) < 0.5) {
                updatePose3(result.getBotposeAvgDist());
                telemetryM.debug("Limelight distance to target: " + result.getBotposeAvgDist());
            }
        } else {
            telemetryM.addData("Limelight", "No Targets");
        }
    }

    private void Turret(){
        bob.turretController.configureConsts();
        double turretAngle = bob.turretController.getTurretAngle();
        telemetryM.debug("Current Turret Angle: "+ turretAngle);
        telemetryM.debug("Current Turret Ticks: "+ bob.turretController.getTurretTicks());
        telemetryM.debug("limelight distance: "+ limeDist);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            limelightTimer.resetTimer();
            limeDist = 39.37*result.getBotposeAvgDist();
            if ((turretAngle > 90.0 && result.getTx() > 0)
                    ||
                    (turretAngle < -135.0 && result.getTx() < 0)
            ) bob.turretController.update(0, 0);
            else bob.turretController.update(result.getTx(), follower.getAngularVelocity());
            // else bob.turretController.update(result.getTx(), 0);
        } else {
            telemetryM.addData("Limelight", "No Targets");
           // bob.turretController.update(0,0);
            bob.turretController.manual(gamepad2.left_stick_x);
        }
    }


    private void Shooter(){
        currentRPM = bob.shooterController.getCurrentRPM();
        if (limelightTimer.getElapsedTimeSeconds() < 3) {
            bob.shooterController.setRPMWithDistance(limeDist - 4);
        } else {
            bob.shooterController.setRPMWithDistance(distanceToTarget-9);
        }
        bob.shooterController.configureConsts();
        bob.shooterController.update();
    }

    private void Hood(){
        if (limelightTimer.getElapsedTimeSeconds() < 3) {
            bob.hoodController.setHoodPosWithDistance(limeDist-4,currentRPM);
        }
        else {
            bob.hoodController.setHoodPosWithDistance(distanceToTarget-9,currentRPM);
        }
    }
    private void Stopper(){
        //if (gamepad2.x) bob.stopperController.setStopperPos(STOPPER_STOP);
        if (gamepad2.x) {
            shootingSpeed = 0.6;
            bob.runMacro(SHOOT_THREE);
            shootTimer.resetTimer();
        }

        if (gamepad2.y) {
            shootingSpeed = 1.0;
            bob.runMacro(SHOOT_THREE);
            shootTimer.resetTimer();
        }
        isShooting = !(shootTimer.getElapsedTimeSeconds() > 1.5);
    }

    private void Intake(){
        if (!isShooting) {
            if (gamepad2.a) {
                bob.intakeController.setIntake(1);
            } else if (gamepad2.b) {
                bob.intakeController.setIntake(-1);
            } else {
                bob.intakeController.setIntake(0);
            }
        }
        if (isShooting && begin.getElapsedTimeSeconds() > 1.6){
            bob.intakeController.setIntake(shootingSpeed);
        }
    }

    // PTO

    public void FrontTwoWheels() {
        if (PTO_RUNNING) {
            PTOTimer.resetTimer();
            bob.frontTwoWheels.setFrontTwoWheelsPower(FrontTwoWheelsPower);
            bob.frontTwoWheels.runFrontTwoWheels();
            PTO_RUNNING = false;
            PTO_STOP = true;
        } else if (PTOTimer.getElapsedTimeSeconds() > 3 && PTO_STOP){
            bob.frontTwoWheels.setFrontTwoWheelsPower(0);
            bob.frontTwoWheels.runFrontTwoWheels();
        }
    }
    public void PTO() {
        if (gamepad2.right_bumper && gamepad2.left_bumper) {
            PTOTimer.resetTimer();
            bob.ptoServos.setPTOPosition(PTO_In_Left, PTO_In_Right);
            PTO_MESSAGE = true;
            PTO_ENGAGED = false;
        }
        if (PTO_MESSAGE) {
            if (PTOTimer.getElapsedTimeSeconds() > 1) {
                PTO_RUNNING = true;
                PTO_MESSAGE = false;
            }
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
    //    bob.turretController.setTargetAngle(targetAngle);

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
