package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="UNICORN TELEOP", group="Linear OpMode")

public class UNICORN_TELEOP extends LinearOpMode {

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // ====================
    // DRIVE MOTORS
    // ====================

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    // ====================
    // INTAKE SERVOS
    // ====================

    // Main intake/outtake servo
    private CRServo intakeServo = null;

    // Two horizontal helper servos
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;


    // Intake toggle variables
    private boolean intakeOn = false;
    private boolean previousLeftBumper = false;


    @Override
    public void runOpMode() {

        // ====================
        // HARDWARE SETUP
        // ====================

        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "left_front_drive");

        leftBackDrive =
                hardwareMap.get(DcMotor.class, "left_back_drive");

        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "right_front_drive");

        rightBackDrive =
                hardwareMap.get(DcMotor.class, "right_back_drive");


        // Main intake servo
        intakeServo =
                hardwareMap.get(CRServo.class, "intake_servo");

        // Helper intake servos
        leftIntakeServo =
                hardwareMap.get(CRServo.class, "left_intake_servo");

        rightIntakeServo =
                hardwareMap.get(CRServo.class, "right_intake_servo");


        // ====================
        // DRIVE DIRECTIONS
        // ====================

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for START
        waitForStart();
        runtime.reset();


        // ====================
        // MAIN TELEOP LOOP
        // ====================

        while (opModeIsActive()) {


            // ====================
            // DRIVING
            // ====================

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double leftPower =
                    Range.clip(drive + turn, -1.0, 1.0);

            double rightPower =
                    Range.clip(drive - turn, -1.0, 1.0);


            leftFrontDrive.setPower(leftPower);
            leftBackDrive.setPower(leftPower);

            rightFrontDrive.setPower(rightPower);
            rightBackDrive.setPower(rightPower);


            // ====================
            // INTAKE TOGGLE
            // Left Bumper
            // ====================

            boolean currentLeftBumper = gamepad1.left_bumper;

            // Detect a NEW press
            if (currentLeftBumper && !previousLeftBumper) {

                intakeOn = !intakeOn;
            }

            previousLeftBumper = currentLeftBumper;


            // ====================
            // INTAKE / OUTTAKE
            // ====================

            if (gamepad1.right_trigger > 0.1) {

                // ----------------
                // OUTTAKE
                // ----------------

                // Main intake reverses
                intakeServo.setPower(-1.0);

                // Helper servos stop
                leftIntakeServo.setPower(0.0);
                rightIntakeServo.setPower(0.0);

            }

            else if (intakeOn) {

                // ----------------
                // INTAKE
                // ----------------

                // Main intake
                intakeServo.setPower(1.0);

                // Helper servos turn on
                leftIntakeServo.setPower(1.0);
                rightIntakeServo.setPower(-1.0);

            }

            else {

                // ----------------
                // EVERYTHING OFF
                // ----------------

                intakeServo.setPower(0.0);

                leftIntakeServo.setPower(0.0);
                rightIntakeServo.setPower(0.0);
            }


            // ====================
            // TELEMETRY
            // ====================

            telemetry.addData(
                    "Status",
                    "Run Time: " + runtime.toString()
            );

            telemetry.addData(
                    "Drive",
                    "Left: %.2f | Right: %.2f",
                    leftPower,
                    rightPower
            );

            telemetry.addData(
                    "Intake Toggle",
                    intakeOn ? "ON" : "OFF"
            );

            if (gamepad1.right_trigger > 0.1) {

                telemetry.addData(
                        "Intake System",
                        "OUTTAKE"
                );

            }

            else if (intakeOn) {

                telemetry.addData(
                        "Intake System",
                        "INTAKE"
                );

            }

            else {

                telemetry.addData(
                        "Intake System",
                        "OFF"
                );
            }

            telemetry.update();
        }
    }
}