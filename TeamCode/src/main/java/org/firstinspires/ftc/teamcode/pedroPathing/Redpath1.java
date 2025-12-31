package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous (name = "Red Path 1", group = "Red")
public class Redpath1 extends OpMode {
    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;

    private PathState pathState;

    private final Pose startPose = new Pose(129.054, 127.631, Math.toRadians(31));
    private final Pose scorePose = new Pose(78.049, 76.863, Math.toRadians(45));
    private final Pose endPose = new Pose(23.723, 96.554, Math.toRadians(45));

    private PathChain driveStartPosShootPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
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
                    // TODO add logic to flywheel shooter
                    telemetry.addLine("Done Path 1");
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
    }
    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechsnicms

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
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose(). getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time", pathTimer.getElapsedTimeSeconds());
    }
}
