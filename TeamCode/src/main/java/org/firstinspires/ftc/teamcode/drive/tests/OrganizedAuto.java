package org.firstinspires.ftc.teamcode.drive.tests;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_HEADING;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_X;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.LAST_Y;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

@Autonomous
public class OrganizedAuto extends OpMode {
    private final Bob bob = new Bob();
    private Timer shootTimer, correctionTimer, opmodeTimer;
    Limelight3A limelight;
    private Follower follower;



    private double timeForShooting3 = 1.0;

    private final Pose startPose = new Pose(117.32413793103447, 128.9103448275862, Math.toRadians(45)); // Start Pose of our robot.

    private BotState botState = BotState.INIT;
    private PathState pathState = PathState.INIT;
    private Shooting shooting = Shooting.START_SHOOTING;


    public PathChain Shot1;
    public PathChain Intake1;
    public PathChain Shot2;
    public PathChain Intake2;
    public PathChain Shot3;
    public PathChain park;
    public PathChain ramPath;
    private void endAuto() {
        bob.cancelMacros();
    }

    public void buildPaths() {

        Shot1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(117.324, 128.910),

                                new Pose(84.703, 92.028)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        Intake1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.703, 92.028),
                                new Pose(102.879, 83.069),
                                new Pose(103.152, 83.386),
                                new Pose(123.000, 83.421)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Shot2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(123.000, 83.421),
                                new Pose(102.352, 81.838),
                                new Pose(84.483, 92.131)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Intake2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.483, 92.131),
                                new Pose(90.355, 54.290),
                                new Pose(92.852, 57.821),
                                new Pose(132.283, 58.214)
                        )
                ).setTangentHeadingInterpolation()

                .build();
        Shot3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.283, 58.214),
                                new Pose(102.848, 73.145),
                                new Pose(84.310, 91.566)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(85, 85), new Pose(102.8923076923077, 66.21538461538462))
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-15))
                .build();
    }

    public enum BotState{
        INIT,
        SHOOTING,
        PATHING,
        WAITING
    }
    public enum PathState{
        INIT,
        SHOOT1,
        INTAKE1,
        SHOOT2,
        INTAKE2,
        SHOOT3,
        PARK
    }
    public enum Shooting{
        START_SHOOTING,
        END_SHOOTING
    }

    public void Shooting(){
        switch (shooting){
            case START_SHOOTING:
                intake();
                //bob.stopperController.open();
                shootTimer.resetTimer();
                shooting = Shooting.END_SHOOTING;
                break;
            case END_SHOOTING:
                if (shootTimer.getElapsedTimeSeconds() >= timeForShooting3){
                    stopIntake();
                    //bob.stopperController.close();
                    shooting = Shooting.START_SHOOTING;
                    setBotState(BotState.PATHING);
                }
                break;
        }


    }
    public void PathEnd(){
        switch(pathState){
            case SHOOT1: // shoot 1 ends
                setBotState(BotState.SHOOTING);
                setPathState(PathState.INTAKE1);
                break;
            case INTAKE1:
                setBotState(BotState.PATHING);
                setPathState(PathState.SHOOT2);
                break;
            case SHOOT2:
                setBotState(BotState.SHOOTING);
                setPathState(PathState.INTAKE2);
                break;
            case INTAKE2:
                setBotState(BotState.PATHING);
                setPathState(PathState.SHOOT3);
                break;
            case SHOOT3:
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
            case INTAKE1:
                followIntakeWait(Intake1);
                break;
            case SHOOT2:
                followWait(Shot2);
                break;
            case INTAKE2:
                followIntakeWait(Intake2);
                break;
            case SHOOT3:
                followWait(Shot3);
                break;
            case PARK:
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

        StateUpdate();
        follower.update();
        bob.tick();

        updateLastPosition();
    }

    @Override
    public void init() {
        // init map
        bob.init(hardwareMap);

        //timers
        shootTimer = new Timer();
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

}
