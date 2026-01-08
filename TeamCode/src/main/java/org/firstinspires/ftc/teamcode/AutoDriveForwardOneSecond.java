package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Drive Forward 1 Second", group = "Autonomous")
public class AutoDriveForwardOneSecond extends LinearOpMode {

    private DcMotor FL, FR, BL, BR;

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

        waitForStart();

        if (opModeIsActive()) {

            // Drive forward at 50% power
            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);

            sleep(1000); // 1 second

            // Stop all motors
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }
}
