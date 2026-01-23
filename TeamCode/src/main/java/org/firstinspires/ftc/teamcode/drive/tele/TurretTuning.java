package org.firstinspires.ftc.teamcode.drive.tele;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.KALMAN_TURRET;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_X;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_Y;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;


@Configurable
@TeleOp
public class TurretTuning extends OpMode {
    Bob bob = new Bob();
    TelemetryManager telemetryM;
    Limelight3A limelight;

    @Override
    // runs on init press
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        bob.init(hardwareMap);;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions
        limelight.start();

    }


    @SuppressLint("DefaultLocale")
    @Override

    public void loop() {

        telemetryM.debug("current angle: "+bob.turretController.getTurretAngle());
        telemetryM.debug("target angle: "+ bob.turretController.getTargetAngle());
        telemetryM.update();

        bob.turretController.setTurretConsts();
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            bob.turretController.update(result.getTx());
        }

        telemetry.update();

    }
}
