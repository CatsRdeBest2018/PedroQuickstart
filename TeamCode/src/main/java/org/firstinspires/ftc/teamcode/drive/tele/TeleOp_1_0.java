package org.firstinspires.ftc.teamcode.drive.tele;

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
public class TeleOp_1_0 extends OpMode {
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
        follower.startTeleopDrive();
    }

    @SuppressLint("DefaultLocale")
    @Override

    public void loop() {

        Position();
        Turret();
        Shooter();
        Hood();
        Stopper();
        Intake();


        if (!gamepad1.right_bumper) follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        else follower.setTeleOpDrive(-gamepad1.left_stick_y*0.7, -gamepad1.left_stick_x*0.7, -gamepad1.right_stick_x*0.3, true);
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


    private void Shooter(){
            currentRPM = bob.shooterController.getCurrentRPM();
            bob.shooterController.setRPMWithDistance(limeDist);
            bob.shooterController.configureConsts();
            bob.shooterController.update();
    }

    private void Hood(){
          bob.hoodController.setHoodPosWithDistance(limeDist,currentRPM);
    }
    private void Stopper(){
        if (gamepad1.y) bob.stopperController.setStopperPos(STOPPER_POS);

        else  bob.stopperController.setStopperPos(0.95);
    }

    private void Intake(){
        if (gamepad1.a){
            bob.intakeController.setIntake(1);
        }
        else if (gamepad1.b){
            bob.intakeController.setIntake(-1);
        }
        else{
            bob.intakeController.setIntake(0);
        }
    }
}
