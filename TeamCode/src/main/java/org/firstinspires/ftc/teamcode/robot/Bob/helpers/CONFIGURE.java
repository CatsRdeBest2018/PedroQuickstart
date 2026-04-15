package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.AngularTuning.ANGULAR_VEL_TUN;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.AngularTuning.F_TUNE;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.AngularTuning.TARGET_ANG_VEL_1;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.AngularTuning.TURRET_F_TUN;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Hood.HOOD_ON;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Hood.HOOD_POS;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Intake.INTAKE_ON;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Intake.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.PTO.FrontTwoWheelsPower;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.PTO.PTO_In_Left;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.PTO.PTO_In_Right;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.PTO.PTO_Position;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Position.DISTANCE_TO_TARGET;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Position.DRIVE;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Position.LL_LOCALIZATION;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Position.SHOW_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Shooter.SHOOTER_ON;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Shooter.USE_DISTANCE;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Stopper.STOPPER_ON;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Stopper.STOPPER_POS;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Turret.TURRET_ON;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.DISTANCE_FROM_TARGET;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.KALMAN_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_X;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_Y;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.TARGET_RPM;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;


@Configurable
@TeleOp
public class CONFIGURE extends OpMode {
    Bob bob = new Bob();
    TelemetryManager telemetryM;
    private double odoDistance = 0;
    private double currentRPM = 0;
    Limelight3A limelight;
    private Follower follower;
    private Pose startPose;
    private double limeDist = 1;
    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        startPose = new Pose(LAST_X,LAST_Y,LAST_HEADING);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        bob.init(hardwareMap);;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

    }
    @Override
    public void start() {
        limelight.start();
        follower.update();
        if (!ANGULAR_VEL_TUN) follower.startTeleopDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override

    public void loop() {

        Position();
        Turret();
        AngularTuning();
        Shooter();
        Hood();
        Stopper();
        Intake();
        FrontTwoWheels();
        PTO();

        if (!ANGULAR_VEL_TUN) follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
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
//        if (DRIVE){
//            if (!follower.isTeleopDrive()) follower.startTeleopDrive();
//            else follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//        }
        if (SHOW_POSITION){
            Pose currentPose = follower.getPose();
            telemetryM.debug("Pedro Pose", String.format("x=%.2f in, y=%.2f in, h=%.1f deg", currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

            odoDistance = Math.sqrt(Math.pow((127.628 - follower.getPose().getX()), 2) + Math.pow((131.669 - follower.getPose().getY()), 2));
            telemetryM.debug("odo distance to target: " + odoDistance);
            telemetryM.update();
        }
        if (LL_LOCALIZATION){
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
    }

    private void Turret(){
        if (TURRET_ON){
            bob.turretController.configureConsts();
            double turretAngle = bob.turretController.getTurretAngle();
            telemetryM.debug("Current Turret Angle: "+ turretAngle);
            telemetryM.debug("Current Turret Ticks: "+ bob.turretController.getTurretTicks());
            telemetryM.debug("limelight distance: "+ limeDist);
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                limeDist = 39.37*result.getBotposeAvgDist();
                if ((turretAngle > 90.0 && result.getTx() > 0)
                        ||
                        (turretAngle < -135.0 && result.getTx() < 0)
                ) bob.turretController.update(0, 0);
               else bob.turretController.update(result.getTx(), follower.getAngularVelocity());
               // else bob.turretController.update(result.getTx(), 0);
            } else {
                telemetryM.addData("Limelight", "No Targets");
                bob.turretController.update(0,0);
            }

        }
    }
    private void AngularTuning(){
        if (ANGULAR_VEL_TUN){
            telemetryM.debug("Current Ang Vel: "+follower.getAngularVelocity());
            telemetryM.debug("real power "+bob.angTuningController.getPower());
            double error = TARGET_ANG_VEL_1 - follower.getAngularVelocity();
            bob.angTuningController.configureConsts();
            bob.angTuningController.update(error);
        }
        if (TURRET_F_TUN){
            double turretAngle = bob.turretController.getTurretAngle();
            telemetryM.debug("Turret Angle: "+ turretAngle);
            telemetryM.debug("Turret Ticks: "+ bob.turretController.getTurretTicks());
            telemetryM.debug("Angular Vel: "+ follower.getAngularVelocity());

            boolean atLimit = (turretAngle > 90.0 || turretAngle < -135.0);
            if (atLimit) bob.turretController.setPower(0);
            else bob.turretController.setPower(F_TUNE);
        }
    }

    private void Shooter(){
        if (SHOOTER_ON){
            currentRPM = bob.shooterController.getCurrentRPM();
            if (!USE_DISTANCE) bob.shooterController.setRPM(BobConfigure.Shooter.TARGET_RPM);
            else bob.shooterController.setRPMWithDistance(limeDist);
            bob.shooterController.configureConsts();
            bob.shooterController.update();
            telemetryM.debug("Current RPM: "+bob.shooterController.getCurrentRPM());
            telemetryM.debug("Target RPM "+BobConfigure.Shooter.TARGET_RPM);
            telemetryM.debug("real power "+bob.shooterController.getPower());
        }
    }

    private void Hood(){
        if (HOOD_ON){
            if (!BobConfigure.Hood.USE_DISTANCE) bob.hoodController.setHoodPos(HOOD_POS);
            else bob.hoodController.setHoodPosWithDistance(limeDist,currentRPM);
        }
    }
    private void Stopper(){
        if (STOPPER_ON)bob.stopperController.setStopperPos(STOPPER_POS);

    }

    private void Intake(){
        if (INTAKE_ON){
            bob.intakeController.setIntake(INTAKE_POWER);
        }
    }

    public void FrontTwoWheels() {
//        if (false) {
//            bob.frontTwoWheels.setFrontTwoWheelsPower(FrontTwoWheelsPower);
//            bob.frontTwoWheels.runFrontTwoWheels();
//        } else {
//            bob.frontTwoWheels.setFrontTwoWheelsPower(0);
//            bob.frontTwoWheels.runFrontTwoWheels();
//        }
    }

    public void PTO() {
//        if (false) {
//            bob.ptoServos.setPTOPosition(PTO_In_Left, PTO_In_Right);
//        }
    }
}
