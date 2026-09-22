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

    // Drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Intake and outtake motors
    private DcMotor intakeMotor = null;
    private DcMotor outtakeMotor = null;

    // Intake toggle variables
    private boolean intakeOn = false;
    private boolean previousLeftBumper = false;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // --------------------
        // HARDWARE SETUP
        // --------------------

        // Drive motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Intake and outtake
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtake_motor");

        // Drive motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

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
            // INTAKE
            // Left bumper = toggle
            // ====================

            boolean currentLeftBumper = gamepad1.left_bumper;

            // Only toggle when the button changes
            // from NOT pressed to pressed
            if (currentLeftBumper && !previousLeftBumper) {
                intakeOn = !intakeOn;
            }

            previousLeftBumper = currentLeftBumper;

            if (intakeOn) {
                intakeMotor.setPower(1.0);
            } else {
                intakeMotor.setPower(0.0);
            }


            // ====================
            // OUTTAKE
            // Hold right trigger
            // ====================

            if (gamepad1.right_trigger > 0.1) {
                outtakeMotor.setPower(1.0);
            } else {
                outtakeMotor.setPower(0.0);
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
                    "Intake",
                    intakeOn ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Outtake",
                    gamepad1.right_trigger > 0.1 ? "ON" : "OFF"
            );

            telemetry.update();
        }
    }
}