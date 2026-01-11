package org.firstinspires.ftc.teamcode.helpers;

import static org.firstinspires.ftc.teamcode.pedroPathing.Robot.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.TICKS_PER_REV_TURRET;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFTurret {

    // PIDF gains
    private double kP, kI, kD, kF;

    // Target position (encoder ticks)
    private double targetAngle = 0;

    private double turretAngle = 0;
    // PID state
    private double integralSum = 0;
    private double lastError = 0;

    private final ElapsedTime timer = new ElapsedTime();

    private static final double maxIntegral = 3000;

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

    public double getTurretAngle(double currentTicks){
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


    public double update(double currentAngle) {

        double error = targetAngle - currentAngle;
        integralSum += error;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);

        if (Math.abs(error) < 0.5) integralSum = 0;

        double derivative = error - lastError;
        lastError = error;

        double feedForward = kF * Math.signum(error);

        double output = kP * error + kI * integralSum + kD * derivative + feedForward;

        return Range.clip(output, -1.0, 1.0);
    }
}
