
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "NARWHALS TELEOP")
public class NARWHALS_TELEOP extends LinearOpMode {

    // DRIVE MOTORS
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    // INTAKE
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private DcMotor intakeMotor;

    // OUTTAKE AND TRANSFER
    private DcMotor outtakeMotor;
    private DcMotor transferMotor;

    // INTAKE TOGGLE
    private boolean intakeOn = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {

        // =========================
        // HARDWARE SETUP
        // =========================

        leftFrontDrive = hardwareMap.get(
                DcMotor.class, "left_front_drive");

        leftBackDrive = hardwareMap.get(
                DcMotor.class, "left_back_drive");

        rightFrontDrive = hardwareMap.get(
                DcMotor.class, "right_front_drive");

        rightBackDrive = hardwareMap.get(
                DcMotor.class, "right_back_drive");

        leftIntakeServo = hardwareMap.get(
                CRServo.class, "left_intake_servo");

        rightIntakeServo = hardwareMap.get(
                CRServo.class, "right_intake_servo");

        intakeMotor = hardwareMap.get(
                DcMotor.class, "intake_motor");

        outtakeMotor = hardwareMap.get(
                DcMotor.class, "outtake_motor");

        transferMotor = hardwareMap.get(
                DcMotor.class, "transfer_motor");

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        leftFrontDrive.setDirection(
                DcMotor.Direction.REVERSE);

        leftBackDrive.setDirection(
                DcMotor.Direction.FORWARD);

        rightFrontDrive.setDirection(
                DcMotor.Direction.FORWARD);

        rightBackDrive.setDirection(
                DcMotor.Direction.REVERSE);

        // Intake servos spin opposite ways
        leftIntakeServo.setDirection(
                DcMotorSimple.Direction.FORWARD);

        rightIntakeServo.setDirection(
                DcMotorSimple.Direction.REVERSE);

        outtakeMotor.setDirection(
                DcMotor.Direction.REVERSE);

        // =========================
        // INITIALIZATION
        // =========================

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // =========================
        // MAIN LOOP
        // =========================

        while (opModeIsActive()) {

            // DRIVING
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double strafe = gamepad1.right_stick_x;

            // Hold B for full speed
            double speed = gamepad1.b ? 1.0 : 0.7;

            double scale = Math.max(
                    1.0,
                    Math.abs(forward)
                            + Math.abs(strafe)
                            + Math.abs(turn)
            );

            leftFrontDrive.setPower(
                    (forward + strafe + turn) / scale * speed);

            leftBackDrive.setPower(
                    (forward - strafe + turn) / scale * speed);

            rightFrontDrive.setPower(
                    (forward - strafe - turn) / scale * speed);

            rightBackDrive.setPower(
                    (forward + strafe - turn) / scale * speed);

            // =========================
            // INTAKE: LEFT BUMPER TOGGLE
            // =========================

            boolean leftBumper = gamepad1.left_bumper;

            if (leftBumper && !lastLeftBumper) {
                intakeOn = !intakeOn;
            }

            lastLeftBumper = leftBumper;

            if (intakeOn) {
                leftIntakeServo.setPower(1.0);
                rightIntakeServo.setPower(1.0);
                intakeMotor.setPower(1.0);
            } else {
                leftIntakeServo.setPower(0);
                rightIntakeServo.setPower(0);
                intakeMotor.setPower(0);
            }

            // =========================
            // TRANSFER: RIGHT BUMPER
            // =========================

            if (gamepad1.right_bumper) {
                transferMotor.setPower(1.0);
            } else {
                transferMotor.setPower(0);
            }

            // =========================
            // OUTTAKE: RIGHT TRIGGER
            // =========================

            if (gamepad1.right_trigger > 0.1) {
                outtakeMotor.setPower(0.55);
            } else {
                outtakeMotor.setPower(0);
            }

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData(
                    "Intake",
                    intakeOn ? "ON" : "OFF");

            telemetry.addData(
                    "Transfer",
                    gamepad1.right_bumper ? "ON" : "OFF");

            telemetry.addData(
                    "Outtake",
                    gamepad1.right_trigger > 0.1
                            ? "ON" : "OFF");

            telemetry.addData(
                    "Speed",
                    gamepad1.b ? "100%" : "70%");

            telemetry.update();
        }

        // STOP EVERYTHING
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        transferMotor.setPower(0);
    }
}
