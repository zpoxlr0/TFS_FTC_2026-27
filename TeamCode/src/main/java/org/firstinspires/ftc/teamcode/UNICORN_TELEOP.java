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

    // Drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Intake / Outtake continuous rotation servo
    private CRServo intakeServo = null;

    // Intake toggle variables
    private boolean intakeOn = false;
    private boolean previousLeftBumper = false;

    @Override
    public void runOpMode() {

        // --------------------
        // HARDWARE SETUP
        // --------------------

        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "left_front_drive");

        leftBackDrive =
                hardwareMap.get(DcMotor.class, "left_back_drive");

        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "right_front_drive");

        rightBackDrive =
                hardwareMap.get(DcMotor.class, "right_back_drive");

        // Dual-mode servo in continuous rotation mode
        intakeServo =
                hardwareMap.get(CRServo.class, "intake_servo");


        // --------------------
        // MOTOR DIRECTIONS
        // --------------------

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for START
        waitForStart();
        runtime.reset();


        // --------------------
        // MAIN TELEOP LOOP
        // --------------------

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
            // Left bumper
            // ====================

            boolean currentLeftBumper = gamepad1.left_bumper;

            // Detect a NEW bumper press
            if (currentLeftBumper && !previousLeftBumper) {
                intakeOn = !intakeOn;
            }

            previousLeftBumper = currentLeftBumper;


            // ====================
            // INTAKE / OUTTAKE SERVO
            // ====================

            if (gamepad1.right_trigger > 0.1) {

                // OUTTAKE
                // Right trigger overrides intake
                intakeServo.setPower(-1.0);

            } else if (intakeOn) {

                // INTAKE
                intakeServo.setPower(1.0);

            } else {

                // STOP
                intakeServo.setPower(0.0);
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
                telemetry.addData("Servo", "OUTTAKE");
            } else if (intakeOn) {
                telemetry.addData("Servo", "INTAKE");
            } else {
                telemetry.addData("Servo", "STOPPED");
            }

            telemetry.update();
        }
    }
}
