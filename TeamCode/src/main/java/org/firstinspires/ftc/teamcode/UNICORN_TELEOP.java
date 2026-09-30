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


    // =========================
    // DRIVE MOTORS
    // =========================

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    // =========================
    // BALL SYSTEM
    // =========================

    // Two horizontal CR servos that collect balls
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;

    // CR servo that moves balls farther inside the robot
    private CRServo intermediateServo = null;

    // Motor used for the final outtake
    private DcMotor outtakeMotor = null;


    // =========================
    // INTAKE TOGGLE
    // =========================

    private boolean intakeOn = false;
    private boolean previousLeftBumper = false;


    @Override
    public void runOpMode() {

        // =========================
        // HARDWARE SETUP
        // =========================

        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "left_front_drive");

        leftBackDrive =
                hardwareMap.get(DcMotor.class, "left_back_drive");

        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "right_front_drive");

        rightBackDrive =
                hardwareMap.get(DcMotor.class, "right_back_drive");


        // Intake CR servos
        leftIntakeServo =
                hardwareMap.get(CRServo.class, "left_intake_servo");

        rightIntakeServo =
                hardwareMap.get(CRServo.class, "right_intake_servo");


        // Intermediate CR servo
        intermediateServo =
                hardwareMap.get(CRServo.class, "intermediate_servo");


        // Outtake motor
        outtakeMotor =
                hardwareMap.get(DcMotor.class, "outtake_motor");


        // =========================
        // DRIVE MOTOR DIRECTIONS
        // =========================

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // =========================
        // INITIALIZATION
        // =========================

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();


        // =========================
        // MAIN TELEOP LOOP
        // =========================

        while (opModeIsActive()) {


            // =====================================
            // DRIVING
            // =====================================

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


            // =====================================
            // INTAKE
            //
            // LEFT BUMPER = Toggle ON/OFF
            // =====================================

            boolean currentLeftBumper = gamepad1.left_bumper;

            // Only toggle when the bumper is first pressed
            if (currentLeftBumper && !previousLeftBumper) {
                intakeOn = !intakeOn;
            }

            previousLeftBumper = currentLeftBumper;


            if (intakeOn) {

                // Opposite directions so both servos
                // pull balls toward the center
                leftIntakeServo.setPower(1.0);
                rightIntakeServo.setPower(-1.0);

            } else {

                leftIntakeServo.setPower(0.0);
                rightIntakeServo.setPower(0.0);
            }


            // =====================================
            // INTERMEDIATE SERVO
            //
            // HOLD RIGHT BUMPER
            // =====================================

            if (gamepad1.right_bumper) {

                intermediateServo.setPower(1.0);

            } else {

                intermediateServo.setPower(0.0);
            }


            // =====================================
            // OUTTAKE MOTOR
            //
            // HOLD RIGHT TRIGGER
            // =====================================

            if (gamepad1.right_trigger > 0.1) {

                outtakeMotor.setPower(1.0);

            } else {

                outtakeMotor.setPower(0.0);
            }


            // =====================================
            // TELEMETRY
            // =====================================

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
                    "Intake",
                    intakeOn ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Intermediate",
                    gamepad1.right_bumper ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Outtake",
                    gamepad1.right_trigger > 0.1 ? "ON" : "OFF"
            );

            telemetry.update();
        }
    }
}