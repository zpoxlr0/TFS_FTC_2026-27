package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Unicorn Auton A")
public class Unicorn_Auton_A extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    @Override
    public void runOpMode() {
        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive =
                hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive =
                hardwareMap.get(DcMotor.class, "right_back_drive");

        // Same directions as your ProgrammingBoard.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{
                leftFrontDrive, leftBackDrive,
                rightFrontDrive, rightBackDrive
        }) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        waitForStart();

        if (isStopRequested()) return;

        try {
            // Forward.
            setDrivePower(0.6);
            sleep(2200);

            // Stop for 20 seconds.
            setDrivePower(0);
            sleep(24000);

            if (!opModeIsActive()) return;

            // Reverse for slightly less time.
            setDrivePower(-0.6);
            sleep(2000);
        } finally {
            setDrivePower(0);
        }
    }

    private void setDrivePower(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
}