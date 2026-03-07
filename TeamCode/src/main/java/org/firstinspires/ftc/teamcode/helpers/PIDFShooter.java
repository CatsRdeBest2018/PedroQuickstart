package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFShooter {

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
    private static final double RPM_TOLERANCE = 50;
    private static final double RPM_DROP_THRESHOLD = 100;
    private static final double BOOST_OUTPUT = 0.15;

    private double integralSum = 0;
    private double lastError   = 0;

    private final ElapsedTime timer = new ElapsedTime();

    // --- Constructors — same signatures as before ---

    public PIDFShooter(double ticksPerRev, double kP, double kI, double kD, double kF) {
        this(ticksPerRev, 6000, kP, kI, kD, kF); // default maxRPM, override with other constructor
    }

    public PIDFShooter(double ticksPerRev, double maxRPM, double kP, double kI, double kD, double kF) {
        this.ticksPerRev = ticksPerRev;
        this.maxRPM      = maxRPM;
        setConsts(kP, kI, kD, kF);
    }

    // --- Config — unchanged ---

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

    // --- Getters/Setters — unchanged ---

    public void setTargetRPM(double rpm)  { this.targetRPM = rpm; }
    public double getTargetRPM()          { return targetRPM; }
    public double getCurrentRPM()         { return filteredRPM; }
    public double getRawRPM()             { return rawRPM; }

    public boolean isAtSpeed() {
        return targetRPM > 0 && Math.abs(targetRPM - filteredRPM) < RPM_TOLERANCE;
    }

    // --- Internal ---

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

    // --- Main update — unchanged signature ---
    private double getFeedforward(double rpm) {
        double x = Range.clip(rpm / 1000.0, 1.5, 3.5); // clamp to your measured range
        return 1.86449  * Math.pow(x, 4)
                - 18.64883 * Math.pow(x, 3)
                + 69.41847 * Math.pow(x, 2)
                - 114.13428 * x
                + 71.00255;
    }
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

        // Feedforward — polynomial only, kF trims it
        double ffOutput = kF * getFeedforward(targetRPM) * normalizedTarget;

        // Proportional
        double pOutput = kP * error;

        // Integral with anti-windup
        boolean saturated = Math.abs(ffOutput + pOutput) >= 1.0;
        boolean windingUp = (error > 0 && integralSum > 0) || (error < 0 && integralSum < 0);
        if (!saturated || !windingUp) {
            integralSum += error * dt;
        }
        double iOutput = Range.clip(kI * integralSum, -MAX_I_OUTPUT, MAX_I_OUTPUT);

        // Derivative
        double derivative = (error - lastError) / dt;
        lastError = error;
        double dOutput = kD * derivative;

        // Ball drop boost — immediate extra power when RPM drops hard
        double boostOutput = 0;
        if ((targetRPM - filteredRPM) > RPM_DROP_THRESHOLD) {
            boostOutput = BOOST_OUTPUT;
        }

        return Range.clip(ffOutput + pOutput + iOutput + dOutput + boostOutput, 0, 1);
    }
}