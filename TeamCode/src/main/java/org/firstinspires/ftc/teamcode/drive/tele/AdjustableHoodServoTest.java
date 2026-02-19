package org.firstinspires.ftc.teamcode.drive.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Step Control", group = "Test")
public class AdjustableHoodServoTest extends LinearOpMode {

    Servo myServo;

    double position = 0.5;


    boolean lastA = false;
    boolean lastB = false;

    @Override
    public void runOpMode() {

        myServo = hardwareMap.get(Servo.class, "servo");
        myServo.setPosition(position);

        waitForStart();

        while (opModeIsActive()) {

            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;


            if (currentA && !lastA) {
                position += 0.01;
            }


            if (currentB && !lastB) {
                position -= 0.01;
            }


            position = Math.max(0.0, Math.min(1.0, position));

            myServo.setPosition(position);

            telemetry.addData("Servo Position", position);
            telemetry.update();

            lastA = currentA;
            lastB = currentB;
        }
    }
}
