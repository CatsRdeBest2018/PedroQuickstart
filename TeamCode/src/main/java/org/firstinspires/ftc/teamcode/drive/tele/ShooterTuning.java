package org.firstinspires.ftc.teamcode.drive.tele;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.DISTANCE_FROM_TARGET;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.TARGET_RPM;
import java.util.Deque;
import java.util.LinkedList;

@TeleOp(name = "Shooter Tuning")
public class ShooterTuning extends OpMode {

    TelemetryManager telemetryM;
    private final Bob bob = new Bob();
    Limelight3A limelight;
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();


    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        bob.init(hardwareMap);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;

        telemetryM.debug("Current RPM: "+bob.shooterController.getCurrentRPM());
        telemetryM.debug("Target RPM "+TARGET_RPM);
        telemetryM.update(telemetry);
        //telemetryM.update();
        // set target rpm
        if (gamepad1.left_bumper) bob.shooterController.setRPMWithDistance(DISTANCE_FROM_TARGET);
        if (gamepad1.right_bumper) bob.shooterController.setRPM(TARGET_RPM);

        // intake
        if (gamepad1.a) bob.intakeController.intake();
        else bob.intakeController.stopIntake();

        bob.tick();
        gamepadUpdate();
    }

    private void gamepadUpdate(){
        gamepad1History.add(gamepad1);
        gamepad2History.add(gamepad2);
        if (gamepad1History.size() > 100) {
            gamepad1History.removeLast();
            gamepad2History.removeLast();
        }
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }


}
