package org.firstinspires.ftc.teamcode.drive.tests;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Hood.HOOD_ON;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Hood.HOOD_POS;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Shooter.SHOOTER_ON;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Shooter.USE_DISTANCE;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Stopper.STOPPER_ON;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Stopper.STOPPER_POS;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure.Turret.TURRET_ON;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.I;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_X;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_Y;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.STOPPER_STOP;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;
import org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConfigure;

@Autonomous
public class OrganizedAuto extends OpMode {
    private final Bob bob = new Bob();
    private Timer shootTimer, correctionTimer, opmodeTimer, intakeTimer;
    Limelight3A limelight;
    private Follower follower;
    private double currentRPM = 0;
   private boolean shoot = true;

    private double setupTime = 0.1;
    private double timeForShooting3 = 1.2;
    private double timeForIntakingGate = 2;

    private final Pose startPose = new Pose(115.446, 125.988, Math.toRadians(45)); // Start Pose of our robot.

    private BotState botState = BotState.INIT;
    private PathState pathState = PathState.INIT;
    private Shooting shooting = Shooting.START_SHOOTING;

    private Intaking intaking = Intaking.START_INTAKING;

    public PathChain Shot1;
    public PathChain Intake1;
    public PathChain Intake1Setup;
    public PathChain Shot2;
    public PathChain Intake2Setup;
    public PathChain Intake2Setup2;
    public PathChain Intake2;
    public PathChain Shot3;
    public PathChain Intake3;

    public PathChain Intake4Setup;
    public PathChain Intake4;
    public PathChain Shot4;
    public PathChain park;
    public PathChain ramPath;
    private void endAuto() {
        bob.cancelMacros();
    }

    public void buildPaths() {
        Shot1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(115.446, 125.988),
                                new Pose(84.703, 92.028)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();
        Intake1Setup = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.338, 83.054),
                                new Pose(93, 59)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
        Intake1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(93, 57),
                                new Pose(126.819, 57)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Shot2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126.819, 56.378),
                                new Pose(87.231, 82.871)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Intake2Setup = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.230, 82.871),
                                new Pose(99.893, 61.449),
                                new Pose(126.089, 55.86)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), 0.66)
                .build();
        Intake2Setup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.230, 82.871),
                                new Pose(99.893, 61.449),
                                new Pose(123, 61.9)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(10))
                .build();
        Intake2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126.089, 55.86),
                                new Pose(126.3, 56)
                        )
                )
                .setLinearHeadingInterpolation(0.66, 0.66)
                .build();


        // angle = 28.65

        Shot3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(126.3, 56),
                                new Pose(95.428, 63.623),
                                new Pose(90.231, 82.871)
                        )
                )
                .setLinearHeadingInterpolation(0.66, Math.toRadians(0))
                .build();
        Intake3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.231, 82.454),
                                new Pose(114, 81.914)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(114, 81.914),
                                new Pose(90.301, 82.336)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Intake4Setup = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(90.301, 82.336),
                                new Pose(99.893, 61.449),
                                new Pose(123, 61.9)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Intake4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(121.530, 58),
                                new Pose(127.88, 57.45)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32.09))
                .build();

        Shot4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129.5, 59),
                                new Pose(95.428, 63.623),
                                new Pose(90.231, 82.871)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(28.65), Math.toRadians(0))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.231, 82.871),
                                new Pose(95, 75)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }

    public enum BotState{
        INIT,
        SHOOTING,
        PATHING,
        WAITING,
        INTAKING
    }
    public enum PathState{
        INIT,
        SHOOT1,
        INTAKE1SETUP,
        INTAKE1,
        SHOOT2,
        INTAKE2SETUP,
        INTAKE2SETUP2,
        INTAKE2,
        SHOOT3,
        INTAKE3,
        INTAKE4SETUP,

        INTAKE4,
        SHOOT4,
        PARK
    }
    public enum Shooting{
        SETUP,
        START_SHOOTING,
        END_SHOOTING
    }
    public enum Intaking{
        START_INTAKING,
        END_INTAKING
    }

    public void Intaking(){
        switch (intaking){
            case START_INTAKING:
                intake();
           //     bob.intakeController.setIntake(.7);
                intakeTimer.resetTimer();
                intaking = Intaking.END_INTAKING;
                break;
            case END_INTAKING:
                if (intakeTimer.getElapsedTimeSeconds() >= timeForIntakingGate){
                    stopIntake();
                    intaking = Intaking.START_INTAKING;
                    setBotState(BotState.PATHING);
                }
                break;
        }
    }
    public void Shooting(){
        switch (shooting){
            case SETUP:
                shootTimer.resetTimer();
                shooting = Shooting.START_SHOOTING;
                break;
            case START_SHOOTING:
                if (shootTimer.getElapsedTimeSeconds() >= setupTime) {
                    intake();
                    bob.stopperController.open();
                    shootTimer.resetTimer();
                    shooting = Shooting.END_SHOOTING;
                }
                break;
            case END_SHOOTING:
                if (shootTimer.getElapsedTimeSeconds() >= timeForShooting3){
                    stopIntake();
                    bob.stopperController.close();
                    shooting = Shooting.SETUP;
                    setBotState(BotState.PATHING);
                }
                break;
        }
    }
    public void PathEnd(){
        switch(pathState){
            case SHOOT1: // shoot 1 ends
                setBotState(BotState.SHOOTING);
                setPathState(PathState.INTAKE1SETUP);
                break;
            case INTAKE1SETUP:
                setBotState(BotState.PATHING);
                setPathState(PathState.INTAKE1);
                break;
            case INTAKE1:
                setBotState(BotState.PATHING);
                setPathState(PathState.SHOOT2);
                break;
            case SHOOT2:
                setBotState(BotState.SHOOTING);
                setPathState(PathState.INTAKE2SETUP);
                break;
            case INTAKE2SETUP:
                setBotState(BotState.PATHING);
               // setPathState(PathState.INTAKE2SETUP2);
                setPathState(PathState.INTAKE2);
                break;
            case INTAKE2SETUP2:
                setBotState(BotState.PATHING);
                setPathState(PathState.INTAKE2);
                break;
            case INTAKE2:
                setBotState(BotState.INTAKING);
                setPathState(PathState.SHOOT3);
                break;
            case SHOOT3:
                setBotState(BotState.SHOOTING);
                setPathState(PathState.INTAKE3);
                break;
            case INTAKE3:
                setBotState(BotState.SHOOTING);
                setPathState(PathState.INTAKE4SETUP);
                break;
            case INTAKE4SETUP:
                setBotState(BotState.PATHING);
                setPathState(PathState.INTAKE4);
                break;
            case INTAKE4:
                setBotState(BotState.INTAKING);
                setPathState(PathState.SHOOT4);
                break;
            case SHOOT4:
                setBotState(BotState.SHOOTING);
                setPathState(PathState.PARK);
                break;
            case PARK:
                endAuto();
                break;
        }
    }
    public void WaitingForPath(){
        if (!follower.isBusy()){
            PathEnd();
        }
    }
    public void PathBegin(){
        switch(pathState){
            case SHOOT1:
                followWait(Shot1);
                break;
            case INTAKE1SETUP:
                shoot = false;
                followSlowWait(Intake1Setup,1);
                break;
            case INTAKE1:
                followSlowIntakeWait(Intake1,1);
                //0.7
                break;
            case SHOOT2:
                shoot = true;
                followIntakeWait(Shot2);
                break;
            case INTAKE2SETUP:
                shoot = false;
                followWait(Intake2Setup);
                break;
            case INTAKE2SETUP2:
                shoot = false;
                followSlowWait(Intake2Setup2,1);
                break;
            case INTAKE2:
                shoot = false;
                followIntakeWait(Intake2);
                break;
            case SHOOT3:
                shoot = true;
                followIntakeWait(Shot3);
                break;
            case INTAKE3:
                shoot = false;
                followIntakeWait(Intake3);
                break;
            case INTAKE4SETUP:
                shoot = false;
                followWait(Intake2Setup);
                break;
            case INTAKE4:
                shoot = false;
                followSlowIntakeWait(Intake2,1);
                break;
            case SHOOT4:
                shoot = true;
                followIntakeWait(Shot4);
                break;
            case PARK:
                shoot = false;
                followWait(park);
                break;
        }
    }

    public void StateUpdate() {
            switch(botState){
                case SHOOTING:
                    Shooting();
                    break;
                case PATHING:
                    PathBegin();
                    break;
                case WAITING:
                    WaitingForPath();
                    break;
                case INTAKING:
                    Intaking();
                    break;
            }
        }


    public void followWait(PathChain path){
        stopIntake();
        follower.followPath(path);
        setBotState(BotState.WAITING);

    }
    public void followIntakeWait(PathChain path){
        follower.followPath(path);
        intake();
        setBotState(BotState.WAITING);
    }
    public void followSlowWait(PathChain path, double speed){
        stopIntake();
        follower.followPath(path, speed, true);
        setBotState(BotState.WAITING);
    }
    public void followSlowIntakeWait(PathChain path, double speed){
        follower.followPath(path, speed, true);
        intake();
        setBotState(BotState.WAITING);
    }
    public void followThenIntake(PathChain path){
        follower.followPath(path);
        setBotState(BotState.WAITING);
    }

    public void setBotState(BotState bs){
        botState = bs;
    }
    public void setPathState(PathState ps){
        pathState = ps;
    }
    public void intake(){
        bob.intakeController.intake();
    }
    public void stopIntake(){
        bob.intakeController.stopIntake();
    }

    public void telemetry(){
        telemetry.addLine("BotState: "+ botState);
        telemetry.addLine("PathState: "+ pathState);
        telemetry.addLine("ShootState: "+ botState);
        telemetry.update();
    }
    @Override
    public void loop() {

        if (shoot){
            Hood();
            Shooter();
        }
        Turret();
        StateUpdate();
        follower.update();
        bob.tick();

        updateLastPosition();
    }

    @Override
    public void init() {
        // init map
        bob.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);

        //timers
        shootTimer = new Timer();
        intakeTimer = new Timer();
        opmodeTimer = new Timer();
        correctionTimer = new Timer();

        // pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // state
        botState = BotState.PATHING;
        pathState = PathState.SHOOT1;
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        limelight.start();
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        endAuto();
    }
    public void safetyChecks(){
        // auto time --> 29.5 seconds we gotta to park
        park();

        // TODO: robot pushed us --> ram

        // TODO: we are stuck mid path

    }
    public void park(){
        if (opmodeTimer.getElapsedTimeSeconds() > 29.5){
            setBotState(BotState.PATHING);
            setPathState(PathState.PARK);
        }
    }
    private void updateLastPosition(){
        Pose currentPose = follower.getPose();
        LAST_X = currentPose.getX();
        LAST_Y = currentPose.getY();
        LAST_HEADING = currentPose.getHeading();
        telemetry.addLine( "x: "+ LAST_X);
        telemetry.addLine( "y: "+ LAST_Y);
        telemetry.addLine( "heading: "+ LAST_HEADING);
        telemetry.update();
    }
    private void Turret(){
            bob.turretController.configureConsts();
            double turretAngle = bob.turretController.getTurretAngle();

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
//                limeDist = 39.37*result.getBotposeAvgDist();
                if ((turretAngle > 90.0 && result.getTx() > 0)
                        ||
                        (turretAngle < -135.0 && result.getTx() < 0)
                ) bob.turretController.update(0, 0);
                else bob.turretController.update(result.getTx(), follower.getAngularVelocity());
                // else bob.turretController.update(result.getTx(), 0);
            } else {
                bob.turretController.update(0,0);
            }
    }
    private void Shooter(){
        currentRPM = bob.shooterController.getCurrentRPM();
            bob.shooterController.setRPMWithDistance(48);
            bob.shooterController.configureConsts();
            bob.shooterController.update();

    }

    private void Hood(){

            bob.hoodController.setHoodPosWithDistance(48,3483);

    }


}
