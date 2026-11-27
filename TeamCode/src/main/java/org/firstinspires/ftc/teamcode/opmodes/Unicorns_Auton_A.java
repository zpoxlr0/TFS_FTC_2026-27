
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Unicorns Auton A", group = "Unicorns")
public class Unicorns_Auton_A extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    @Override
    public void runOpMode() {

        // HARDWARE SETUP
        leftFrontDrive = hardwareMap.get(
                DcMotor.class, "left_front_drive");

        leftBackDrive = hardwareMap.get(
                DcMotor.class, "left_back_drive");

        rightFrontDrive = hardwareMap.get(
                DcMotor.class, "right_front_drive");

        rightBackDrive = hardwareMap.get(
                DcMotor.class, "right_back_drive");

        // MOTOR DIRECTIONS
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // MOTOR CONFIGURATION
        for (DcMotor motor : new DcMotor[]{
                leftFrontDrive,
                leftBackDrive,
                rightFrontDrive,
                rightBackDrive
        }) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );
        }

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        try {

//            // STEP 1: FORWARD FOR 2.2 SECONDS
//            telemetry.addData("Stage", "FORWARD");
//            telemetry.update();

            setDrivePower(0.6);
            sleep(500);

            if (!opModeIsActive()) return;
//
//            // STEP 2: STOP FOR 1 SECOND
//            telemetry.addData("Stage", "WAITING");
//            telemetry.update();
//
//            setDrivePower(0);
//            sleep(1000);
//
//            if (!opModeIsActive()) return;
//
//            // STEP 3: REVERSE FOR 1.8 SECONDS
//            telemetry.addData("Stage", "REVERSING");
//            telemetry.update();
//
//            setDrivePower(-0.6);
//            sleep(1800);

        } finally {

            // STOP ALL DRIVE MOTORS
            setDrivePower(0);

            telemetry.addData("Stage", "FINISHED");
            telemetry.update();
        }
    }

    private void setDrivePower(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }
}
