package org.firstinspires.ftc.teamcode.robot.Bob;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PIDFShooter;
import org.firstinspires.ftc.teamcode.helpers.PIDFTurret;
import org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure;
import org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobState;
import org.firstinspires.ftc.teamcode.robot.Bob.helpers.Link;

@Configurable
public class Bob implements Robot {
    protected HardwareMap hw = null;

    // Controllers
    public IntakeController intakeController = new IntakeController();
    public ShooterController shooterController = new ShooterController();
    public TurretController turretController = new TurretController();
    public HoodController hoodController = new HoodController();
   // public StopperController stopperController = new StopperController();

    public DcMotorEx intake;
    public DcMotorEx shooterRight;
    public DcMotorEx shooterLeft;
    public DcMotorEx turret;
    public CRServoImplEx intakeRight;
    public CRServoImplEx intakeLeft;
    public Servo hood;
    public Servo stopper;

    Limelight3A limelight;

    public boolean manualReset = false;
    public double manualPower = 0;
    // Pedro Pathing

    public Telemetry tele;
    public boolean inited = false;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(HardwareMap hardwareMap) {
        initHardwareMap(hardwareMap);
    }

    public void initHardwareMap(HardwareMap hardwareMap) {

        // HOOD
        hood = hardwareMap.servo.get("hood");

        // STOPPER
       // stopper = hardwareMap.servo.get("ballstop");

        // SHOOTERS
        shooterRight = (DcMotorEx) hardwareMap.dcMotor.get("sr");
        shooterRight.setZeroPowerBehavior(FLOAT);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft = (DcMotorEx) hardwareMap.dcMotor.get("sl");
        shooterLeft.setZeroPowerBehavior(FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // TURRET
        turret = (DcMotorEx) hardwareMap.dcMotor.get("turret");
        turret.setZeroPowerBehavior(BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);



        // INTAKE
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setZeroPowerBehavior(BRAKE);
        intakeLeft = hardwareMap.get(CRServoImplEx.class, "intakeLeft");

        intakeRight = hardwareMap.get(CRServoImplEx.class, "intakeRight");
        intakeLeft.setDirection(CRServoImplEx.Direction.REVERSE);

        // LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        shooterController.start();
        turretController.start();
        hoodController.start();
        //stopperController.start();

        hw = hardwareMap;
        runtime.reset();
        inited = true;
    }

    public void tick() {
        tickMacros();
        intakeController.intakeTick();
        shooterController.update();
    }



    public class ShooterController {
        double powerT;
        double ticksT;
        double leftPos;
        double rightPos;
        private PIDFShooter shootPID;
        public void start() {
            shootPID = new PIDFShooter(TICKS_PER_REV_SHOOTER,3000, P, I, D,F);
            shootPID.reset(0);
        }

        public void configureConsts() {
            shootPID.setConsts(BobConfigure.Shooter.P,BobConfigure.Shooter.I,BobConfigure.Shooter.D,BobConfigure.Shooter.F);
        }

        public void update(){
            double currentTicks = (shooterLeft.getCurrentPosition() + shooterRight.getCurrentPosition()) / 2.0;
          //  if (shootPID.getTargetRPM() == RPM_OFF)shootPID.setConsts(0, 0, 0,F);
          //  else shootPID.setConsts(P, I, D,F);

            double power = shootPID.update(currentTicks);
            ticksT = currentTicks;
            powerT = power;
            leftPos = shooterLeft.getCurrentPosition();
            rightPos = shooterRight.getCurrentPosition();
            shooterLeft.setPower(power);
            shooterRight.setPower(power);
        }
        public double getPower(){
            return powerT;
        }
        public double getTicks(){
            return ticksT;
        }
        public double getRightPos(){
            return rightPos;
        }
        public double getLeftPos(){
            return leftPos;
        }
        public void setRPM(double rpm) {
            shootPID.setTargetRPM(rpm);
        }
        public void setRPMWithDistance(double distance) {
            double rpm = (0.00642857 * distance + 1.9) * 1000;
            shootPID.setTargetRPM(rpm);
        }

        // getters
        public double getCurrentRPM() {
            return shootPID.getCurrentRPM();
        }
        public double getTargetRPM(){
            return shootPID.getTargetRPM();
        }
    }

    public class HoodController {
        public void start() {
            hood.setPosition(HOOD_STARTING_POS);
        }
        public double getHoodPos(){
            return hood.getPosition();
        }
        public void setHoodPos(double pos){
            hood.setPosition(pos);
        }
        public void setHoodPosWithDistance(double distance){
            double pos = (-59.88069 + 30.36486 * Math.log(distance)) / 100.0;
            hood.setPosition(Range.clip(pos, 0.1, 0.7));
        }
    }
//    public class StopperController {
//        public void start() {
//            stopper.setPosition(STOPPER_STARTING_POS);
//        }
//        public double getStopperPos(){
//            return stopper.getPosition();
//        }
//        public void setStopperPos(double pos) {
//            stopper.setPosition(pos);
//        }
//    }

    public class TurretController {
        private PIDFTurret turretPIDF;
        public void start() {
            turretPIDF = new PIDFTurret(tP, tI, tD,tF);
            turretPIDF.reset();
        }
        public double getTurretAngle(){
            return turretPIDF.getTurretAngle(turret.getCurrentPosition());
        }
        public double getTurretTicks(){
            return turret.getCurrentPosition();
        }
        public void setTurretConsts(){
            turretPIDF.setConsts(tP,tI,tD,tF);
        }
        public void configureTurretConsts(){
            turretPIDF.setConsts(
                    BobConfigure.Turret.P,BobConfigure.Turret.I,BobConfigure.Turret.D,BobConfigure.Turret.F
            );
        }


        public void update(double currentAngle, double angVel){
            double power = turretPIDF.update(currentAngle, angVel);
            turret.setPower(-power);
        }
        public void setTargetAngle(double angle) {
            turretPIDF.setTargetAngle(angle);
        }
        public double getTargetAngle(){
            return turretPIDF.getTargetAngle();
        }

    }


    public class IntakeController {
        private double intakePower = 0;
        public void intake() {
            intakePower = INTAKE_POWER_IN;
        }
        public void outtake() {
            intakePower = INTAKE_POWER_OUT;
        }
        public void stopIntake() {
            intakePower = INTAKE_POWER_OFF;
        }
        public void intakeTick() {
            intake.setPower(intakePower);
            intakeLeft.setPower(intakePower);
            intakeRight.setPower(intakePower);
        }
        public void setIntake(double pow) {
            intake.setPower(pow);
            intakeLeft.setPower(pow);
            intakeRight.setPower(pow);
        }
    }


    public BobState macroState = null;
    public boolean MACROING = false;
    public ElapsedTime macroTimer = new ElapsedTime();
    public int macroTimeout = INFINITY;

    public void runMacro(BobState m) {
        if (macroTimer.milliseconds() < macroTimeout)
            macroTimeout = INFINITY;
        macroState = m;
        MACROING = true;
    }

    public void cancelMacros() {
        MACROING = false;
        macroTimeout = INFINITY;
    }
    public void tickMacros() {
        if (!MACROING && macroTimer.milliseconds() > macroTimeout && macroTimeout != INFINITY) {
            macroTimeout = INFINITY;
            MACROING = true;
        }

        if (MACROING) {
            BobState m = macroState;

            if (m.shooterRPM != null) shooterController.setRPM(m.shooterRPM);

            // Handle spindexer angle - absolute or increment


            if (m.intakePower != null) {
                if (m.intakePower == INTAKE_POWER_IN) {
                    intakeController.intake();
                } else if (m.intakePower == INTAKE_POWER_OUT) {
                    intakeController.outtake();
                } else {
                    intakeController.stopIntake();
                }
            }

            if (m.linkedState != null && m.linkedState.type == Link.LinkType.WAIT) {
                macroTimer.reset();
                macroTimeout = m.linkedState.trigger;
                macroState = m.linkedState.nextState;
            }

            MACROING = false;
        }
    }


}
