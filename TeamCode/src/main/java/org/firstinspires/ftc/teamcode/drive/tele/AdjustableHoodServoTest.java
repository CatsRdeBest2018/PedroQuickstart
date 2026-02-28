package org.firstinspires.ftc.teamcode.drive.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Configurable
@TeleOp(name = "Servo Step Control", group = "Test")
public class AdjustableHoodServoTest extends LinearOpMode {

    ServoImplEx hood;
    CRServoImplEx intakeRight;
    CRServoImplEx intakeLeft;
    DcMotorImplEx intake;

    public static double position = 0;
    public static double power = 0;
    TelemetryManager t;


    @Override
    public void runOpMode() {

        hood = hardwareMap.get(ServoImplEx.class, "hood");
        intakeRight = hardwareMap.get(CRServoImplEx.class,"intakeRight");
        intakeLeft = hardwareMap.get(CRServoImplEx.class,"intakeLeft");
        intake = hardwareMap.get(DcMotorImplEx.class,"intake");

        intake.setDirection(DcMotorImplEx.Direction.REVERSE);

        hood.setPosition(position);
        intakeRight.setPower(power);
        intakeLeft.setPower(-power);
        intake.setPower(power);
        t = PanelsTelemetry.INSTANCE.getTelemetry();


        waitForStart();

        while (opModeIsActive()) {
            t.update();

            hood.setPosition(position);
            intakeRight.setPower(power);
            intakeLeft.setPower(-power);
            intake.setPower(power);
            telemetry.addData("Servo Position", position);
            telemetry.update();

        }
    }
}
