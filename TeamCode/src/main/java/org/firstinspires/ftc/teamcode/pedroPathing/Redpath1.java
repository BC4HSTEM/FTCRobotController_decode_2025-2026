package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

@Autonomous (name = "Red Path 1", group = "Red")
public class Redpath1 extends OpMode {

    /** This is the enum where we store the state of our auto.
     * It is used by the pathUpdate method. */
    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS
    }
    /** Create an instance of the enum with a variable to track the path state */
    private PathState pathState;

    /** Launcher Setup */
    private Launcher shooter;

    /** Variable to track the number of triggered shots*/
    private boolean shotsTriggered = false;

    /** Setup pedro pathing follower */
    private Follower follower;

    /** Setup variables for timers used to track actions - IF NEEDED, no currently using */
    private Timer pathTimer, actionTimer, opModeTimer;

    // TODO : Update the pose coordinate and heading values based on Izeyah and Majesty's work
    /** Set up variables for the poses : Left, Right, Heading (degrees to radians */
    private final Pose startPose = new Pose(123.597, 122.500, Math.toRadians(45));
    private final Pose scorePose = new Pose(84.822, 84.164, Math.toRadians(45));
    private final Pose endPose = new Pose(85.918, 102.575, Math.toRadians(45));

    /** Setup the PathChains for each "set or group" of movements */
    private PathChain driveStartPosShootPos; // Drive from the start position to the shoot position
    private PathChain driveShootPosToEndPos; // Drive from the shoot Position to the end position

    public void buildPaths() {

        // coordinates for starting pose > shoot pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        // coordinates for shoot pose to end pose
        driveShootPosToEndPos = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    // Launch preloads
                    if(!shotsTriggered){
                        shooter.fireShots(3);
                        shotsTriggered = true;
                        telemetry.addLine("Done Path 1");
                    }
                    else if (shotsTriggered && !shooter.isBusy()){
                        // shots are done, transition
                        follower.followPath(driveShootPosToEndPos, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                    }
                }
                break;
            case DRIVE_SHOOTPOS_ENDPOS:
                // all done
                if(!follower.isBusy()){
                    telemetry.addLine("Done all paths.");
                }
                break;
            default:
                telemetry.addLine("No State Commander");
                break;

        }
    }

    /* Method to set the pathstate, reset the timer, */
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }


    /* This method overrides the FTC code for init */
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);


        // Combine the default driver station telemetry with the FTC Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // TODO add in any other init mechanisms
        shooter = new Launcher(telemetry, hardwareMap);
        shooter.init();

        buildPaths();
        follower.setPose(startPose);
    }


    /* This method overrides the FTC code for start (executes once when you press start.
    *  Here we are setting the pathState and the timer */
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    /* This method overrides the FTC code for loop (after hitting play)
    *  Here we are updating the follower, shooter, and statePathUpdate so they execute on each loop
    *  as well as outputting telemetry. */
    @Override
    public void loop() {
        follower.update();
        shooter.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose(). getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
    }
}
