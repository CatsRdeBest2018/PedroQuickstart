package org.firstinspires.ftc.teamcode.drive.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Single Servo TeleOp", group = "Test")
public class AdjustableHoodServoTest extends LinearOpMode {

    Servo hood;

    double position = 0.5;

    @Override
    public void runOpMode() {


        hood = hardwareMap.get(Servo.class, "hood");

        hood.setPosition(position);

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.a) {
                position+=0.01;
            }

            if (gamepad1.b) {
                position-=0.01;
            }


            position += -gamepad1.left_stick_y * 0.01;


            position = Math.max(0.0, Math.min(1.0, position));

            hood.setPosition(position);

            telemetry.addData("Servo Position", position);
            telemetry.update();
        }
    }
}
