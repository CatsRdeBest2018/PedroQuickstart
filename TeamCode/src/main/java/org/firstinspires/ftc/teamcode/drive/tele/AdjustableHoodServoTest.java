package org.firstinspires.ftc.teamcode.drive.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
@TeleOp(name = "Servo Step Control", group = "Test")
public class AdjustableHoodServoTest extends LinearOpMode {

    Servo hood;

    public static double position = 0.5;
    TelemetryManager t;


    public boolean lastA = false;
    public boolean lastB = false;

    @Override
    public void runOpMode() {

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(position);
        t = PanelsTelemetry.INSTANCE.getTelemetry();


        waitForStart();

        while (opModeIsActive()) {
            t.update();
            boolean currentA = gamepad1.a;
            boolean currentB = gamepad1.b;


            if (currentA && !lastA) {
                position += 0.01;
            }


            if (currentB && !lastB) {
                position -= 0.01;
            }


            position = Math.max(0.0, Math.min(1.0, position));

            hood.setPosition(position);

            telemetry.addData("Servo Position", position);
            telemetry.update();
            lastA = currentA;
            lastB = currentB;
        }
    }
}
