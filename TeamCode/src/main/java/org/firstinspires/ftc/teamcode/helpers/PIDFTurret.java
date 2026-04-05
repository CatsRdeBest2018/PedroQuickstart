package org.firstinspires.ftc.teamcode.helpers;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.TICKS_PER_REV_TURRET;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFTurret {

    private double kP, kI, kD, kF;

    private double targetAngle = 0;
    private double turretAngle = 0;

    private double integralSum = 0;
    private double lastError = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private static final double MAX_INTEGRAL = 1.0; // in output units, not ticks

    public PIDFTurret(double kP, double kI, double kD, double kF) {
        setConsts(kP, kI, kD, kF);
        reset();
    }

    public void setConsts(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public double getTurretAngle(double currentTicks) {
        double rev = (currentTicks / (double) TICKS_PER_REV_TURRET) % 1.0;
        turretAngle = rev * 360;
        return turretAngle;
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    private double wrapError(double error) {
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    public double update(double currentAngle, double r) {

        double dt = timer.seconds();
        timer.reset();
// clamp
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        double error = wrapError(targetAngle - currentAngle);

        if (Math.abs(error) > 0.5) {
            integralSum += error * dt;
            integralSum = Range.clip(integralSum, -MAX_INTEGRAL / kI, MAX_INTEGRAL / kI);
        } else {
            integralSum = 0;
        }
        double derivative = (error - lastError) / dt;
        lastError = error;

        double abs_r = Math.abs(r);
        double feedForward = Math.signum(r) * (
                0
                + 0.158122 * abs_r
                +  -0.0259028 * abs_r*abs_r
                + 0.00578704 * abs_r*abs_r*abs_r
                +  -0.000625 * abs_r*abs_r*abs_r*abs_r);
        feedForward = -feedForward * kF;

        double output = (kP * error)
                + (kI * integralSum)
                + (kD * derivative)
                + feedForward;

        return Range.clip(output, -1.0, 1.0);
    }
}