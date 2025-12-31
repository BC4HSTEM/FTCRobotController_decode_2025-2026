package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Launcher {
    private Telemetry telemetry;
    private HardwareMap hwMap;

    // Constructor to receive the Telemetry and HardwareMap objects
    public Launcher(Telemetry telemetry, HardwareMap hwMap) {
        this.telemetry = telemetry;
        this.hwMap = hwMap;
    }

    public static double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    public static double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public static double FULL_SPEED = 1.0;

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
        LAUNCHING,
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

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

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
