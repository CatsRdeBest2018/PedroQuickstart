package org.firstinspires.ftc.teamcode.drive.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

public class ServoTests extends OpMode {

    private final Bob bob = new Bob();

    @Override
    public void init() {
        bob.init(hardwareMap, false);

    }

    @Override
    public void loop() {

        if (gamepad2.a) {
            bob.spindexer.setPower(1);
        }

    }
}
