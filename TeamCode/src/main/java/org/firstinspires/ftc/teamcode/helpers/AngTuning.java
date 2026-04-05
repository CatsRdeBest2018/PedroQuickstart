package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class AngTuning {

    private double kP, kI, kD, kF;

    private double targetAngle = 0;

    private double integralSum = 0;
    private double lastError = 0;
    private double filteredDerivative = 0;
    private double filteredInput = 0;
    private double filteredOutput = 0;
    private boolean firstLoop = true;

    private static final double DERIVATIVE_FILTER = 0.25;
    private static final double INPUT_FILTER = 0.3;
    private static final double OUTPUT_FILTER = 0.2; // lower = smoother, more lag
    private static final double MAX_INTEGRAL = 1.0;

    private final ElapsedTime timer = new ElapsedTime();

    public AngTuning(double kP, double kI, double kD, double kF) {
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
        filteredDerivative = 0;
        filteredInput = 0;
        filteredOutput = 0;
        firstLoop = true;
        timer.reset();
    }

    public void setTargetAngle(double angle) { this.targetAngle = angle; }
    public double getTargetAngle()           { return targetAngle; }

    public double update(double currentAngle, double robotAngularVel) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        if (firstLoop) {
            filteredInput = currentAngle;
            filteredOutput = 0;
            firstLoop = false;
        } else {
            filteredInput = INPUT_FILTER * currentAngle + (1.0 - INPUT_FILTER) * filteredInput;
        }

        double error = targetAngle - filteredInput;

        double pOut = kP * error;
        boolean saturated = Math.abs(pOut) >= 1.0;
        boolean windingUp = (error > 0 && integralSum > 0) || (error < 0 && integralSum < 0);
        if (!saturated || !windingUp) {
            integralSum += error * dt;
        }
        double maxI = kI > 0 ? MAX_INTEGRAL / kI : MAX_INTEGRAL;
        integralSum = Range.clip(integralSum, -maxI, maxI);

        double rawDerivative = (error - lastError) / dt;
        filteredDerivative = DERIVATIVE_FILTER * rawDerivative + (1.0 - DERIVATIVE_FILTER) * filteredDerivative;
        lastError = error;

        double feedForward = kF * -robotAngularVel;

        double rawOutput = pOut
                + (kI * integralSum)
                + (kD * filteredDerivative)
                + feedForward;

        // Smooth the final output
        filteredOutput = OUTPUT_FILTER * rawOutput + (1.0 - OUTPUT_FILTER) * filteredOutput;

        return Range.clip(filteredOutput, -1.0, 1.0);
    }
}