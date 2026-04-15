package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class BLPIDFShooter {

    private double kP, kI, kD, kF;

    private final double ticksPerRev;
    private final double maxRPM;

    private double targetRPM    = 0;
    private double filteredRPM  = 0;
    private double rawRPM       = 0;
    private double lastTicks    = 0;
    private boolean firstSample = true;

    private static final double RPM_ALPHA    = 0.15;
    private static final double MAX_I_OUTPUT = 0.15;
    private static final double RPM_TOLERANCE = 200;

    private double integralSum = 0;
    private double lastError   = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public BLPIDFShooter(double ticksPerRev, double kP, double kI, double kD, double kF) {
        this(ticksPerRev, 6000, kP, kI, kD, kF);
    }

    public BLPIDFShooter(double ticksPerRev, double maxRPM, double kP, double kI, double kD, double kF) {
        this.ticksPerRev = ticksPerRev;
        this.maxRPM      = maxRPM;
        setConsts(kP, kI, kD, kF);
    }

    public void setConsts(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset(double currentTicks) {
        integralSum = 0;
        lastError   = 0;
        filteredRPM = 0;
        rawRPM      = 0;
        lastTicks   = currentTicks;
        firstSample = true;
        timer.reset();
    }

    public void setTargetRPM(double rpm)  { this.targetRPM = rpm; }
    public double getTargetRPM()          { return targetRPM; }
    public double getCurrentRPM()         { return filteredRPM; }
    public double getRawRPM()             { return rawRPM; }

    public boolean isAtSpeed() {
        return targetRPM > 0 && Math.abs(targetRPM - filteredRPM) < RPM_TOLERANCE;
    }

    private void updateRPM(double currentTicks, double dt) {
        if (firstSample) {
            firstSample = false;
            lastTicks   = currentTicks;
            return;
        }
        double deltaTicks = currentTicks - lastTicks;
        lastTicks         = currentTicks;
        rawRPM            = (deltaTicks / ticksPerRev) / dt * 60.0;
        filteredRPM       = RPM_ALPHA * rawRPM + (1.0 - RPM_ALPHA) * filteredRPM;
    }

    private double getFeedforward(double rpm) {
        double x = Range.clip(rpm / 1000.0, 0, 5.73);
        return -0.00158144  * Math.pow(x, 4)
                + 0.0214144 * Math.pow(x, 3)
                - 0.0924621 * Math.pow(x, 2)
                + 0.298742 * x
                + 0;
    }

    // NEED P to be 4 if RPM is above 4000

    public double update(double currentTicks) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        updateRPM(currentTicks, dt);

        if (targetRPM == 0) {
            integralSum = 0;
            lastError   = 0;
            return 0;
        }

        if (firstSample) {
            return Range.clip(kF * getFeedforward(targetRPM), 0, 1);
        }

        double normalizedTarget  = targetRPM  / maxRPM;
        double normalizedCurrent = filteredRPM / maxRPM;
        double error             = normalizedTarget - normalizedCurrent;

        double ffOutput = kF * getFeedforward(targetRPM);

        double pOutput = kP * error;

        boolean saturated = Math.abs(ffOutput + pOutput) >= 1.0;
        boolean windingUp = (error > 0 && integralSum > 0) || (error < 0 && integralSum < 0);
        if (!saturated || !windingUp) {
            integralSum += error * dt;
        }
        double iOutput = Range.clip(kI * integralSum, -MAX_I_OUTPUT, MAX_I_OUTPUT);

        double derivative = (error - lastError) / dt;
        lastError = error;
        double dOutput = kD * derivative;

        return Range.clip(ffOutput + pOutput + iOutput + dOutput, 0, 1);
    }
}