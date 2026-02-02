package org.firstinspires.ftc.teamcode.robot.Bob;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helpers.PIDFShooter;
import org.firstinspires.ftc.teamcode.helpers.PIDFTurret;
import org.firstinspires.ftc.teamcode.helpers.PIDShooter;
import org.firstinspires.ftc.teamcode.helpers.PIDSpindexer;
import org.firstinspires.ftc.teamcode.robot.Bob.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobState;
import org.firstinspires.ftc.teamcode.robot.Bob.helpers.Link;

@Configurable
public class Bob extends Meccanum implements Robot {
    protected HardwareMap hw = null;

    // Controllers
    public SpindexerController spindexerController = new SpindexerController();
    public IntakeController intakeController = new IntakeController();
    public TransferController transferController = new TransferController();
    public ProximityController proximityController = new ProximityController();
    public NewShooterController newShooterController = new NewShooterController();
    public TurretController turretController = new TurretController();


    public RevColorSensorV3 c;
    public RevColorSensorV3 c2;
    public RevColorSensorV3 c3;
    // Motors
    public DcMotorEx intake;
    public DcMotorEx shooterRight;
    public DcMotorEx shooterLeft;

    //public DcMotorEx turret;

    // Servos
    public CRServo spindexer;
    public DcMotorEx spincoder; // Motor used as encoder for spindexer
    public Servo transfer;
    public Servo light;
    public Servo lservo;

    public boolean manualReset = false;

    public double manualPower = 0;
    // Pedro Pathing
    public Follower follower;

    public Telemetry tele;
    public boolean inited = false;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    public void init(HardwareMap hardwareMap, boolean resetSpindexer) {
        super.init(hardwareMap);

        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("br");


        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(BRAKE);

        shooterRight = (DcMotorEx) hardwareMap.dcMotor.get("sr");
        shooterLeft = (DcMotorEx) hardwareMap.dcMotor.get("sl");

     //   turret = (DcMotorEx) hardwareMap.dcMotor.get("turret");

        shooterLeft.setZeroPowerBehavior(FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

       // turret.setZeroPowerBehavior(BRAKE);
       // turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //turret.setDirection(DcMotorSimple.Direction.REVERSE);


        shooterRight.setZeroPowerBehavior(FLOAT);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(BRAKE);


        c = hardwareMap.get(RevColorSensorV3.class, "color");
        c2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        c3 = hardwareMap.get(RevColorSensorV3.class, "color3");

        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        spincoder = hardwareMap.get(DcMotorEx.class, "spincoder"); // Using back left as encoder
        if (resetSpindexer) {
            spincoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        spincoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = hardwareMap.get(Servo.class, "transfer");

        light = hardwareMap.get(Servo.class, "light");
        lservo = hardwareMap.get(Servo.class, "lservo");

       // shooterController.start();
        newShooterController.start();
        turretController.start();
        spindexerController.start();
        intakeController.start();
        transferController.start();

        hw = hardwareMap;
        runtime.reset();
        inited = true;
    }

    public void tick() {
        tickMacros();
        intakeController.intakeTick();
        transferController.transferTick();
        proximityController.proximityTick();
        spindexerController.spindexerTick();
        newShooterController.update();
    }
    public void updateLight(int n) {
        switch(n){
            case 0:
                light.setPosition(LIGHT0);
                break;
            case 1:
                light.setPosition(LIGHT1);
                break;
            case 2:
                light.setPosition(LIGHT2);
                break;
            case 3:
            case 4:
                light.setPosition(LIGHT3);
                break;
            default:
                break;
        }
    }


    public class ProximityController {
        private boolean isBall;
        public void proximityTick(){
            isBall = c.getDistance(DistanceUnit.MM) < BALL_PROX
                    || c2.getDistance(DistanceUnit.MM) < BALL_PROX
                    || c3.getDistance(DistanceUnit.MM) < BALL_PROX;
        }
        public boolean isBall(){
            return isBall;
        }
    }
    // TODO: SPINDEXER SHIT

    public class SpindexerController {
        private PIDSpindexer spinPID;
        private double targetAngle = 0;
        private boolean emergencyMode = false;

        public void start() {
            spinPID = new PIDSpindexer(TICKS_PER_REV_SPINDEXER, SPINDEX_KP, SPINDEX_KI, SPINDEX_KD);
            spinPID.reset(0);
        }
        public void setConsts(double p, double i, double d) {
            spinPID.setConsts(p,i,d);
        }

        public void setTargetAngle(double angle) {
            targetAngle = angle;
            double currentTicks = spincoder.getCurrentPosition();
            spinPID.setTargetAngle(targetAngle, currentTicks);
        }
        public double getTargetAngle() {
            return (targetAngle/360.0) * TICKS_PER_REV_SPINDEXER;
        }

        public void incrementAngle(double increment) {
            setTargetAngle(targetAngle + increment);
        }

        public void manualPower(double p){
            manualPower = p;
        }
       public void resetEncoder(){
           spincoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        public void spindexerTick() {
            double currentTicks = spincoder.getCurrentPosition();
            spinPID.setTargetAngle(targetAngle, currentTicks);

            double power = -spinPID.update(currentTicks);

            if (!manualReset) {
                if (emergencyMode && (power > 0.1 || power < -0.1)) {
                    spindexer.setPower(power * SPINDEX_EMERGENCY_POWER_LIMIT);
                } else {
                    spindexer.setPower(power);
                }
            }
            else{
                spindexer.setPower(manualPower);
            }

            double wrappedTicks = currentTicks % TICKS_PER_REV_SPINDEXER;
            if (wrappedTicks < 0) wrappedTicks += TICKS_PER_REV_SPINDEXER;
            double currentAngle = (wrappedTicks / TICKS_PER_REV_SPINDEXER) * 360.0;

        }

        public double getCurrentAngle() {
            double currentTicks = spincoder.getCurrentPosition();
            double wrappedTicks = currentTicks % TICKS_PER_REV_SPINDEXER;
            if (wrappedTicks < 0) wrappedTicks += TICKS_PER_REV_SPINDEXER;
            return (wrappedTicks / TICKS_PER_REV_SPINDEXER) * 360.0;
        }
    }


// TODO: NEW SHOOTER PID

    public class NewShooterController {
        private PIDFShooter shootPID;
        public void start() {
            shootPID = new PIDFShooter(TICKS_PER_REV_SHOOTER, P, I, D,F);
            shootPID.reset(0);
        }

        public void update(){
            double currentTicks = (shooterLeft.getCurrentPosition() + shooterRight.getCurrentPosition()) / 2.0;
            if (shootPID.getTargetRPM() == RPM_OFF)shootPID.setConsts(0, 0, 0,F);
            // else if (shootPID.getTargetRPM() == RPM_ZONE1) shootPID.setConsts(SHOOTER_P_Z1, SHOOTER_I_Z1, SHOOTER_D_Z1);
          //  else if (shootPID.getTargetRPM() == RPM_ZONE2) shootPID.setConsts(SHOOTER_P_Z2, SHOOTER_I_Z2, SHOOTER_D_Z2);
            else shootPID.setConsts(P, I, D,F);

            double power = shootPID.update(currentTicks);

            shooterLeft.setPower(power);
            shooterRight.setPower(power);
        }
        public void setRPM(double rpm) {
            shootPID.setTargetRPM(rpm);
        }
        public double getCurrentRPM() {
            return shootPID.getCurrentRPM();
        }
        public double getTargetRPM(){
            return shootPID.getTargetRPM();
        }

    }

    // TODO:  TURRET PIDF

    public class TurretController {
        private PIDFTurret turretPIDF;
        public void start() {
            turretPIDF = new PIDFTurret(tP, tI, tD,tF);
            turretPIDF.reset();
        }
        public double getTurretAngle(){
            return 0.0;
                   // return turretPIDF.getTurretAngle(turret.getCurrentPosition());
        }
        public double getTurretTicks(){
            return 0.0;
           // return turret.getCurrentPosition();
        }

        public void update(double currentAngle){
            double power = turretPIDF.update(currentAngle);
            //turret.setPower(power);
        }
        public void setTargetAngle(double angle) {
            turretPIDF.setTargetAngle(angle);
        }
        public double getTargetAngle(){
            return turretPIDF.getTargetAngle();
        }

    }

    // TODO: INTAKE SHIT

    public class IntakeController {
        private double intakePower = 0;

        public void start() {
        }

        public void intake() {
            intakePower = INTAKE_POWER_IN;
        }
        public double getIntake(){
            return intakePower;
        }

        public void outtake() {
            intakePower = INTAKE_POWER_OUT;
        }

        public void stopIntake() {
            intakePower = INTAKE_POWER_OFF;
        }

        public void intakeTick() {
            intake.setPower(intakePower);
        }
    }

    // TODO: TRANSFER SHIT

    public class TransferController {
        private boolean transferPulse = false;
        private long transferTimer = 0;
        private double transferPosition = TRANSFER_DOWN;

        public void start() {
            transfer.setPosition(TRANSFER_DOWN);
        }

        public void pulse() {
            transferPulse = true;
            transferTimer = System.currentTimeMillis();
            transferPosition = TRANSFER_UP;
        }

        public void setUp() {
            transferPosition = TRANSFER_UP;
            transferPulse = false;
        }

        public void setDown() {
            transferPosition = TRANSFER_DOWN;
            transferPulse = false;
        }

        public void transferTick() {
            if (transferPulse) {
                if (System.currentTimeMillis() - transferTimer > TRANSFER_PULSE_TIME) {
                    transferPosition = TRANSFER_DOWN;
                    transferPulse = false;
                }
            }

            transfer.setPosition(transferPosition);

        }

        public boolean isPulsing() {
            return transferPulse;
        }
    }


    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motorBackLeft.setZeroPowerBehavior(zeroPowerBehavior);
        motorBackRight.setZeroPowerBehavior(zeroPowerBehavior);
        motorFrontRight.setZeroPowerBehavior(zeroPowerBehavior);
        motorFrontLeft.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void strafe(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(power);
    }

    // TODO: MACRO SHIT

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

            if (m.shooterRPM != null) newShooterController.setRPM(m.shooterRPM);

            // Handle spindexer angle - absolute or increment
            if (m.spindexerAngle != null) {
                if (m.spindexerAbsolute != null && m.spindexerAbsolute) {
                    spindexerController.setTargetAngle(m.spindexerAngle);  // Absolute
                } else {
                    spindexerController.incrementAngle(m.spindexerAngle);  // Increment (default)
                }
            }

            if (m.transferPosition != null) {
                if (m.transferPosition == TRANSFER_UP) {
                    transferController.setUp();
                } else {
                    transferController.setDown();
                }
            }

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
    public void tickWithMacros() {
        tickMacros();
       // shooterController.shooterTick();
        spindexerController.spindexerTick();
        intakeController.intakeTick();
        transferController.transferTick();


    }
    public boolean isBall(){
        return proximityController.isBall();
    }
}
