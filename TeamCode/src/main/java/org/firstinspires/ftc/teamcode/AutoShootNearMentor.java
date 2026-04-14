package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

@Config
@Autonomous(name = "Near Shot Mentor")
public class AutoShootNearMentor extends OpMode {

    private DcMotorEx launcher = null;

    public double highVelocity = 1500;
    public double lowVelocity = 900;
    public double curTargetVelocity = highVelocity;
    public double F = 0;
    public double P = 0;

    public double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Init Complete");
        telemetry.update();

    }

    @Override
    public void loop(){
        telemetry.addData("Status","After Start Loop");
        telemetry.update();

        if(gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // set velocity
        launcher.setVelocity(curTargetVelocity);

        double curVelocity = launcher.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);

        telemetry.addLine("-----------------------------------");


    }
}
