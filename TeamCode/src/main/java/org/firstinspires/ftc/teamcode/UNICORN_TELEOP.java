package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="UNICORN TELEOP", group="Linear OpMode")

public class UNICORN_TELEOP extends LinearOpMode {

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Declare the four drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Connect the Java variables to the motors
        // These names MUST match the names in the robot configuration.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Motors on opposite sides of the robot face opposite directions.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait until the driver presses START
        waitForStart();
        runtime.reset();

        // Keep running until STOP is pressed
        while (opModeIsActive()) {

            double leftPower;
            double rightPower;

            // Left stick controls forward/backward movement
            double drive = -gamepad1.left_stick_y;

            // Right stick controls turning
            double turn = gamepad1.right_stick_x;

            // Calculate power for each side
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Both motors on the left get the same power
            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);

            // Both motors on the right get the same power
            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);

            // Display useful information on the Driver Station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.update();
        }
    }
}
//
