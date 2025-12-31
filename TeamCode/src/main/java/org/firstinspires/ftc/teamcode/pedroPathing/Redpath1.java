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

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS
    }


    // Launcher Setup
    private Launcher shooter;
    private boolean shotsTriggered = false;

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private PathState pathState;

    private final Pose startPose = new Pose(129.054, 127.631, Math.toRadians(31));
    private final Pose scorePose = new Pose(78.049, 76.863, Math.toRadians(45));
    private final Pose endPose = new Pose(23.723, 96.554, Math.toRadians(45));

    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosToEndPos;

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

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shotsTriggered = false;
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);


        // Combine the default driver station telemetry with the FTC Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //TODO add in any other init mechanisms
        shooter = new Launcher(telemetry, hardwareMap);
        shooter.init();

        buildPaths();
        follower.setPose(startPose);
    }


    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

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
