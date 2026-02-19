package org.firstinspires.ftc.teamcode.drive.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Configurable
@TeleOp(name = "Servo Step Control", group = "Test")
public class AdjustableHoodServoTest extends LinearOpMode {

    ServoImplEx hood;

    public static double position = 0.5;
    TelemetryManager t;


    @Override
    public void runOpMode() {

        hood = hardwareMap.get(ServoImplEx.class, "hood");
        hood.setPosition(position);
        t = PanelsTelemetry.INSTANCE.getTelemetry();


        waitForStart();

        while (opModeIsActive()) {
            t.update();

            hood.setPosition(position);

            telemetry.addData("Servo Position", position);
            telemetry.update();

        }
    }
}
