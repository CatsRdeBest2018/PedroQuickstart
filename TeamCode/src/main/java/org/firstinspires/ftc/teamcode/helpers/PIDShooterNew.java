package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDShooterNew {

    // PID gains
    private double kP, kI, kD, kF;
    private double currentRPM = 0;

    // Motor/encoder geometry
    private final double ticksPerRev;

    // Target RPM
    private double targetRPM = 0;

    // PID state
    private double integralSum = 0;
    private double lastError = 0;

    // RPM calculation state
    private double lastTicks = 0;
    private boolean firstSample = true;

    private final ElapsedTime timer = new ElapsedTime();

    private static final double maxIntegral = 2000;

    // --- Constructors ---

    public PIDShooterNew(double ticksPerRev) {
        this(ticksPerRev, 0, 0, 0, 0);
    }

    public PIDShooterNew(double ticksPerRev, double kP, double kI, double kD, double kF) {
        this.ticksPerRev = ticksPerRev;
        setConsts(kP, kI, kD, kF);
        reset(0);
    }

    // --- Config ---

    public void setConsts(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** Reset PID state and lock RPM calculation to current encoder ticks. */
    public void reset(double currentTicks) {
        integralSum = 0;
        lastError = 0;

        lastTicks = currentTicks;
        firstSample = true;

        targetRPM = 0;
        timer.reset();
    }

    // --- Target Setting ---

    /** Set the target RPM. */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    /** Returns the current target RPM. */
    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    // --- RPM Calculation ---

    private double computeCurrentRPM(double currentTicks) {
        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3;

        timer.reset();

        if (firstSample) {
            firstSample = false;
            lastTicks = currentTicks;
            currentRPM = 0;
            return currentRPM;
        }

        double deltaTicks = currentTicks - lastTicks;
        lastTicks = currentTicks;

        double revs = deltaTicks / ticksPerRev;
        currentRPM = (revs / dt) * 60.0;

        return currentRPM;
    }


    // --- Main PID update ---

    /**
     * Call every loop with current encoder ticks.
     * Returns a power in [-1, 1] to send to the shooter motor.
     */
    public double update(double currentTicks) {

        // Compute RPM from ticks
        double currentRPM = computeCurrentRPM(currentTicks);

        double error = targetRPM - currentRPM;

        // integrate
        double dt = Math.max(timer.seconds(), 1e-3);
        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);

        // derivative
        double derivative = (error - lastError) / dt;
        lastError = error;

        // PID output
        double output = kF * targetRPM + kP * error + kI * integralSum + kD * derivative;

        return Range.clip(output, -1, 1);
    }
}
