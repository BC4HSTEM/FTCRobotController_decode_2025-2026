package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    Flywheel Launcher logic is based on the following video
    https://www.youtube.com/watch?v=p6g8CbNj0eM&t=46s
    The code does differ because we do not have a "gate" to track.
*/

public class Launcher {
    private Telemetry telemetry;
    private HardwareMap hwMap;

    // Constructor to receive the Telemetry and HardwareMap objects
    // A Constructor is called when an object of a class is created
    public Launcher(Telemetry telemetry, HardwareMap hwMap) {
        this.telemetry = telemetry;
        this.hwMap = hwMap;
    }

    public static double FEED_TIME_SECONDS = 1.0; // The feeder servos run this long when a shot is requested.
    public static double STOP_SPEED = 0.0; // We send this power to the servos when we want them to stop.
    public static double FULL_SPEED = 1.0; // We send this power to the servos when we want them to spin.

    // Robot Starts close, in front of Goal
    public static double LAUNCHER_TARGET_VELOCITY = 1625;
    public static double LAUNCHER_MIN_VELOCITY = 1275;

    private int shotsRemaining = 0;

    // Declare OpMode members.
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime stateTimer = new ElapsedTime();

    /*
     * Launch States
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private LaunchState launchState;

    public void init(){
        launchState = LaunchState.IDLE;

        launcher = hwMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFeeder = hwMap.get(CRServo.class, "grabber_left");
        rightFeeder = hwMap.get(CRServo.class, "grabber_right");

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");

    }

    public void update() {
        switch (launchState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    stateTimer.reset();
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                    stateTimer.reset();
                }
                break;
            case LAUNCH:
                shotsRemaining--;
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                stateTimer.reset();
                launchState = LaunchState.LAUNCHING;

                telemetry.addData("launch speed",launcher.getVelocity());
                telemetry.addData("launch speed target",LAUNCHER_TARGET_VELOCITY);
                telemetry.addData("launcher speed min",LAUNCHER_MIN_VELOCITY);

                telemetry.update();
                break;
            case LAUNCHING:
                if (stateTimer.seconds() > FEED_TIME_SECONDS ) {
                    if(shotsRemaining == 0){
                        launchState = LaunchState.IDLE;
                        leftFeeder.setPower(STOP_SPEED);
                        rightFeeder.setPower(STOP_SPEED);
                        launcher.setVelocity(0);
                    } else {
                        launchState = LaunchState.SPIN_UP;
                    }
                }
                break;
        }
    }

    public void fireShots(int numOfShots){
        if(launchState == LaunchState.IDLE){
            shotsRemaining = numOfShots;
        }
    }

    public boolean isBusy(){
        return launchState != LaunchState.IDLE;
    }
}
