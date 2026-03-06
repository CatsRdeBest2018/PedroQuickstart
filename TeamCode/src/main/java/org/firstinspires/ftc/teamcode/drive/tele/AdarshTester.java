package org.firstinspires.ftc.teamcode.drive.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants;

@Configurable
@TeleOp(name = "ThroughbotQuickTest", group = "Test")
public class AdarshTester extends LinearOpMode {

    ServoImplEx hood;
    CRServoImplEx intakeRight;
    CRServoImplEx intakeLeft;
    DcMotorImplEx intake;
    DcMotorImplEx sl;
    DcMotorImplEx sr;
    ServoImplEx ballStop;

    public static double hoodPosition = 0;
    public static double intakePower = 0;
    public static double shooterPower = 0;
    public static double stopperPos = 0;
    TelemetryManager t;


    @Override
    public void runOpMode() {

        hood = hardwareMap.get(ServoImplEx.class, "hood");
        intakeRight = hardwareMap.get(CRServoImplEx.class,"intakeRight");
        intakeLeft = hardwareMap.get(CRServoImplEx.class,"intakeLeft");
        intake = hardwareMap.get(DcMotorImplEx.class,"intake");
        sr = hardwareMap.get(DcMotorImplEx.class, "sr");
        sl = hardwareMap.get(DcMotorImplEx.class, "sl");
        ballStop = hardwareMap.get(ServoImplEx.class, "ballStop");

        intake.setDirection(DcMotorImplEx.Direction.REVERSE);

        hood.setPosition(hoodPosition);
        intakeRight.setPower(intakePower);
        intakeLeft.setPower(-intakePower);
        intake.setPower(intakePower);
        sl.setPower(-BobConstants.RPM_ZONE1_AUTO);
        sr.setPower(BobConstants.RPM_ZONE1_AUTO);
        ballStop.setPosition(stopperPos);
        t = PanelsTelemetry.INSTANCE.getTelemetry();


        waitForStart();

        while (opModeIsActive()) {
            t.update();

            hood.setPosition(hoodPosition);
            intakeRight.setPower(intakePower);
            intakeLeft.setPower(-intakePower);
            intake.setPower(intakePower);
            sl.setPower(-BobConstants.RPM_ZONE1_AUTO);
            sr.setPower(BobConstants.RPM_ZONE1_AUTO);
            ballStop.setPosition(stopperPos);
            telemetry.addData("Servo Position", hoodPosition);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("ballStop position", stopperPos);
            telemetry.update();

        }
    }
}
