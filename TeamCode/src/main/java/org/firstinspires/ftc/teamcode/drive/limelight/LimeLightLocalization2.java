package org.firstinspires.ftc.teamcode.drive.limelight;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

@Configurable
@TeleOp
public class LimeLightLocalization2 extends OpMode {
    DcMotorEx motorFrontLeft;
    DcMotorEx motorBackLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackRight;

    Limelight3A limelight;
    private Follower follower;
    GoBildaPinpointDriver pinpoint;

    // start pose
    public static double startX = 108.939;
    public static double startY = 137.322;
    public static double startHeadingDeg = 270;

    // how much time in between relocalizing
    public static double waitSeconds = 0.0;

    // kalman filter
    public static double kVision = 0.30;      // 0.15–0.50 typical

    // max distance to relocalize to
    public static double maxJumpIn = 24.0;

    // max image capture delay
    public static double maxLatencyMs = 1400.0;

    // pose history update interval
    public static double poseHistoryDtMs = 50.0;

    // field conversion shi
    public static double limelightYawOffsetDeg = 90.0;
    public static double fieldHalfIn = 72.0;
    public static boolean flipY = true; // if true: Py = -LLy + 72, else Py = +LLy + 72

    public static double metersToInches = 39.3701;

    private final ElapsedTime lastRelocalized = new ElapsedTime();

    public void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    @Override
    public void init() {
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
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startX, startY, Math.toRadians(startHeadingDeg)));
        follower.update();

        lastRelocalized.reset();
    }

    @Override
    public void start() {
        limelight.start();
        lastRelocalized.reset();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        pinpoint.update();
        follower.update();

        Pose currentPose = follower.getPose();
        telemetry.addData("Pedro Pose",
                String.format("x=%.2f in, y=%.2f in, h=%.1f deg",
                        currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        Drawing.drawPoseHistory(follower.getPoseHistory());
        drawCurrent();

        // Driver control
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double scale = gamepad1.right_bumper ? 0.3 : 1.0;
        motorFrontLeft.setPower(scale * (y + x + rx));
        motorBackLeft.setPower(scale * (y - x + rx));
        motorFrontRight.setPower(scale * (y - x - rx));
        motorBackRight.setPower(scale * (y + x - rx));

        // Feed Limelight yaw (deg)
        double ppYawDeg = pinpoint.getHeading(AngleUnit.DEGREES);
        double pedroYawDeg = Math.toDegrees(follower.getPose().getHeading());

        telemetry.addData("ppYaw (deg)", ppYawDeg);
        telemetry.addData("pedroYaw (deg)", pedroYawDeg);

        limelight.updateRobotOrientation(pedroYawDeg + limelightYawOffsetDeg);

        // Read Limelight result
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            telemetry.addData("Botpose_MT2 exists", botpose_mt2 != null);
            if (botpose_mt2 != null) {
                // Raw MT2 meters
                double a = botpose_mt2.getPosition().x;
                double b = botpose_mt2.getPosition().y;

                // Convert meters -> inches
                double llXIn = a * metersToInches;
                double llYIn = b * metersToInches;

                // Convert -> Pedro inches (your tested mapping)
                double visionX = llXIn + fieldHalfIn;
                double visionY = (flipY ? (-llYIn) : llYIn) + fieldHalfIn;

                telemetry.addData("Vision Pedro (in)", String.format("(%.1f, %.1f)", visionX, visionY));
                
                // if you can relocalize
                boolean okTime = (waitSeconds <= 0.0) || (lastRelocalized.seconds() >= waitSeconds);

                // total delay time
                double captureLatencyMs = result.getCaptureLatency();
                double targetingLatencyMs = result.getTargetingLatency();
                double parseLatencyMs = result.getParseLatency();
                double stalenessMs = result.getStaleness();

                double captureLatencyUsedMs = Double.isFinite(captureLatencyMs) ? captureLatencyMs : 0.0;
                double targetingLatencyUsedMs = Double.isFinite(targetingLatencyMs) ? targetingLatencyMs : 0.0;
                double parseLatencyUsedMs = Double.isFinite(parseLatencyMs) ? parseLatencyMs : 0.0;
                double stalenessUsedMs = Double.isFinite(stalenessMs) ? stalenessMs : 0.0;
                double totalLatencyMs = captureLatencyUsedMs + targetingLatencyUsedMs + parseLatencyUsedMs + stalenessUsedMs;

                telemetry.addData("LL capture latency (ms)", String.format("%.0f", captureLatencyMs));
                telemetry.addData("LL targeting latency (ms)", String.format("%.0f", targetingLatencyMs));
                telemetry.addData("LL parse latency (ms)", String.format("%.0f", parseLatencyMs));
                telemetry.addData("LL staleness (ms)", String.format("%.0f", stalenessMs));
                telemetry.addData("LL total latency (ms)", String.format("%.0f", totalLatencyMs));
                
                // if delay time is ok
                boolean captureFinite = Double.isFinite(captureLatencyMs);
                boolean targetingFinite = Double.isFinite(targetingLatencyMs);
                boolean parseFinite = Double.isFinite(parseLatencyMs);
                boolean stalenessFinite = Double.isFinite(stalenessMs);
                boolean latencyComponentsFinite = captureFinite && targetingFinite && parseFinite && stalenessFinite;

                boolean okLatency = totalLatencyMs >= 0 && totalLatencyMs <= maxLatencyMs;

                boolean canCorrect = okTime && okLatency;
                String rejectReason = null;

                if (!okTime) {
                    rejectReason = String.format("waiting %.2f/%.2f s", lastRelocalized.seconds(), waitSeconds);
                } else if (!okLatency) {
                    rejectReason = "latency too high for history buffer";
                }

                if (canCorrect) {
                    // check pos at past time
                    double[] hx = follower.getPoseHistory().getXPositionsArray();
                    double[] hy = follower.getPoseHistory().getYPositionsArray();
                    if (hx == null || hy == null || hx.length == 0 || hy.length == 0) {
                        canCorrect = false;
                        rejectReason = "pose history unavailable";
                    } else {
                        // estimate average pos between pos type
                        int maxIndex = Math.min(hx.length, hy.length) - 1;

                        double idx = totalLatencyMs / poseHistoryDtMs;
                        int i0 = (int) Math.floor(idx);
                        int i1 = i0 + 1;
                        if (i0 < 0) i0 = 0;
                        if (i0 > maxIndex) i0 = maxIndex;
                        if (i1 < 0) i1 = 0;
                        if (i1 > maxIndex) i1 = maxIndex;
                        double frac = idx - Math.floor(idx);
                        if (frac < 0) frac = 0;
                        if (frac > 1) frac = 1;

                        double pastX = hx[i0] * (1.0 - frac) + hx[i1] * frac;
                        double pastY = hy[i0] * (1.0 - frac) + hy[i1] * frac;

                        // Error at capture time
                        double dxPast = visionX - pastX;
                        double dyPast = visionY - pastY;
                        double jump = Math.hypot(dxPast, dyPast);

                        telemetry.addData("History idx", String.format("idx=%.2f i0=%d i1=%d frac=%.2f", idx, i0, i1, frac));
                        telemetry.addData("Past pose (in)", String.format("(%.1f, %.1f)", pastX, pastY));
                        telemetry.addData("Vision delta@past (in)",
                                String.format("(dx=%.1f, dy=%.1f) jump=%.1f", dxPast, dyPast, jump));
                        
                        // if jump not too big
                        if (jump <= maxJumpIn) {
                            // Apply correction to the current pose (blended)
                            Pose cur = follower.getPose();
                            double newX = cur.getX() + kVision * dxPast;
                            double newY = cur.getY() + kVision * dyPast;

                            follower.setPose(new Pose(newX, newY, cur.getHeading()));
                            lastRelocalized.reset();

                            telemetry.addData("Vision applied",
                                    String.format("k=%.2f -> (%.2f, %.2f)", kVision, newX, newY));
                        } else {
                            canCorrect = false;
                            rejectReason = String.format("jump %.1f > %.1f", jump, maxJumpIn);
                        }
                    }
                }
                if (!canCorrect && rejectReason != null) {
                    telemetry.addData("Vision rejected", rejectReason);
                }
                if (!latencyComponentsFinite) {
                    telemetry.addData("LL latency finite",
                            String.format("cap=%b targ=%b parse=%b stale=%b",
                                    captureFinite, targetingFinite, parseFinite, stalenessFinite));
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
