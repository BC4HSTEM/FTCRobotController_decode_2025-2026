package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

import java.util.Set;

@Config
@Autonomous(name = "Near shot", group = "Autonomous")
public class AutoShootNear extends LinearOpMode {

    private DcMotor FL, FR, BL, BR;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    public static double targetVelocity;
    public static double minVelocity;
    public static double drivePower;
    public static int feederTime;
    public static int feederPause;

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
         launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(300, 0, 0, 10));
         leftFeeder = hardwareMap.get(CRServo.class,"grabber_left");
         rightFeeder = hardwareMap.get(CRServo.class, "grabber_right");
         leftFeeder.setPower(0);
         rightFeeder.setPower(0);
         feederTime = 1000;
         feederPause = 250;

         shooter = new Launcher(telemetry, hardwareMap);
         shooter.init();

         targetVelocity = 1496;
         minVelocity=1446;

telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

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

            // launch first ball
            sleep(1000);
            setLauncher(1.0, 1.0);
            telemetry.addData("launch state","start launch");
            telemetry.update();

            //launch second ball
            sleep(feederPause);
            setLauncher(1.0, 1.0);

            //launch third ball
            sleep(feederPause);
           setLauncher(1.0, 1.0);


            sleep(700);

            // stop flywheel
            launcher.setVelocity(0);

            sleep(3000);

            telemetry.addData("launch state","stop launch");
            telemetry.update();

        }
    }

    private void setLauncher(double lFeederPower, double rFeederPower){

        //launch third ball
        if (launcher.getVelocity() > minVelocity) {
            leftFeeder.setPower(lFeederPower);
            rightFeeder.setPower(rFeederPower);
            sleep(feederTime);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
        }
    }
}
