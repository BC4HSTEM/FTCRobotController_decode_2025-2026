package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

import java.util.Set;

@Config
@Autonomous(name = "Near shot", group = "Autonomous")
public class AutoShootNear extends LinearOpMode {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    private DcMotor FL, FR, BL, BR;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    public static double targetVelocity;
    public static double minVelocity;
    public static double drivePower;
    public static int feederTime;
    public static int feederPause;

    public static int flywheelP = 300;
    public static int flywheelI = 0;
    public static int flywheelD = 0;
    public static int flywheelF = 10;

    private Launcher shooter;

    @Override
    public void runOpMode() {

        // Initialize motors using your configuration names
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        // Reverse left side motors (standard mecanum setup)
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        drivePower = 0.2;

        //set up launchers
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF));
        leftFeeder = hardwareMap.get(CRServo.class,"grabber_left");
        rightFeeder = hardwareMap.get(CRServo.class, "grabber_right");

        // leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        feederTime = 250;
        feederPause = 3500;

        shooter = new Launcher(telemetry, hardwareMap);
        shooter.init();

        targetVelocity = 1490;
        minVelocity=1485;

        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        if(gamepad1.x){
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriver.setPattern(pattern);
        } else if (gamepad1.x){
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(pattern);
        }

        waitForStart();


        if (opModeIsActive()) {

            // Drive backward at 50% power
            FL.setPower(-drivePower);
            FR.setPower(-drivePower);
            BL.setPower(-drivePower);
            BR.setPower(-drivePower);

            sleep(2050); // 2 second

            // Stop all motors
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            // start flywheel
            launcher.setVelocity(targetVelocity);
            sleep(feederPause);

            // launch first ball
            //wait for velocity for the first ball
            while (launcher.getVelocity() < minVelocity) {
                telemetry.addData("Launch state", "checking velocity");
                telemetry.update();
            }
            setLauncher(-1.0, 1.0);

            sleep(feederPause);

            telemetry.addData("launch state","start launch");
            telemetry.addData("launch velocity",launcher.getVelocity());
            telemetry.update();


            sleep(feederPause);

            //launch second ball
            //wait for velocity for the second ball
            while (launcher.getVelocity() < minVelocity) {
                telemetry.addData("Launch state", "checking velocity");
                telemetry.update();
            }
            setLauncher(-1.0, 1.0);

            sleep(feederPause);

            //launch third ball
            //wait for velocity for the third ball
            while (launcher.getVelocity() < minVelocity) {
                telemetry.addData("Launch state", "checking velocity");
                telemetry.update();
            }
            setLauncher(-1.0, 1.0);

            sleep(feederTime);


            //sleep(700);

            // stop flywheel
            //launcher.setVelocity(0);

            //sleep(3000);

            telemetry.addData("launch state","stop launch");
            telemetry.update();

        }
    }

    private void setLauncher(double lFeederPower, double rFeederPower){
        telemetry.addData("feeder","feeders called");
        telemetry.update();

        leftFeeder.setPower(lFeederPower);
        rightFeeder.setPower(rFeederPower);
        telemetry.addData("launch velocity",launcher.getVelocity());
        telemetry.addData("feeder","feeders running");

        telemetry.update();
        sleep(feederTime);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

    }
}
