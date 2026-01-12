package org.firstinspires.ftc.teamcode.drive.limelight;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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


@Configurable
@TeleOp
public class LimeLightLocalization extends OpMode {
    DcMotorEx motorFrontLeft;
    DcMotorEx motorBackLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackRight;
    Limelight3A limelight;

    private Follower follower;
    GoBildaPinpointDriver pinpoint;

    public static int wait = 5;
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
        // define and init robot
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("br");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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

        //Drawing.drawPoseHistory(follower.getPoseHistory());
        drawCurrent();

//       if (gamepad2.start || gamepad1.start) return;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (!gamepad1.right_bumper){
            motorFrontLeft.setPower(y + x + rx);
            motorBackLeft.setPower(y - x + rx);
            motorFrontRight.setPower(y - x - rx);
            motorBackRight.setPower(y + x - rx);
        }
        else {
            motorFrontLeft.setPower(0.3 * (y + x + rx));
            motorBackLeft.setPower(0.3 * (y - x + rx));
            motorFrontRight.setPower(0.3 * (y - x - rx));
            motorBackRight.setPower(0.3 * (y + x - rx));
        }

        double ppYaw = pinpoint.getHeading(AngleUnit.DEGREES);

        telemetry.addData("ppYaw", ppYaw);
        limelight.updateRobotOrientation(ppYaw+90);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            telemetry.addData("Botpose_MT2 exists", botpose_mt2 != null);

            if (botpose_mt2 != null) {
                double a = botpose_mt2.getPosition().x;
                double b = botpose_mt2.getPosition().y;
                double z = botpose_mt2.getPosition().z;

                telemetry.addData("MT2 Location:", "(" + a + ", " + b + ", " + z + ")");

                try {
                    Pose currentFollowerPose = follower.getPose();

                    double LPxInches = a * 39.3701;
                    double LPyInches = b * 39.3701;

                    double PxInches = LPxInches+72;
                    double PyInches = -LPyInches+72;

                    boolean clearView = true;

                    if (lastRelocalized.seconds() >= wait) {
                        if (clearView) {
                            telemetry.addData("Re-localized", String.format("x=%.2f in, y=%.2f in, h=%.1f deg", PxInches, PyInches, currentFollowerPose.getHeading()));
                            follower.setPose(new Pose(PxInches, PyInches, currentFollowerPose.getHeading()));
                            lastRelocalized.reset();
                        } else {
                            telemetry.addData("April Tag not in clear view", String.format("wait=%.2f, x=%.2f in, y=%.2f in, h=%.1f deg", lastRelocalized.seconds(), PxInches, PyInches, currentFollowerPose.getHeading()));
                        }
                    } else {
                        telemetry.addData("Waiting to Re-localized, ", String.format("wait=%.2f, x=%.2f in, y=%.2f in, h=%.1f deg", lastRelocalized.seconds(), PxInches, PyInches, currentFollowerPose.getHeading()));
                    }

                } catch (Exception e) {
                    telemetry.addData("Limelight error", e.getMessage());
                }
            } else {
                telemetry.addData("Botpose_MT2", "NULL - Check pipeline configuration");
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();
    }
}
