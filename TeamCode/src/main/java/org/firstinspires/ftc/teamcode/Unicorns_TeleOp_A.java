package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Unicorn TeleOp A")
public class Unicorns_TeleOp_A extends LinearOpMode {

    private final Unicorns_ProgrammingBoard board = new Unicorns_ProgrammingBoard();
    private final ElapsedTime runtime = new ElapsedTime();

    // Starting position for tracking
    private static final double START_X = 0;
    private static final double START_Y = 0;
    private static final double START_HEADING = 0;

    @Override
    public void runOpMode() {

        board.init(hardwareMap);

        try {

            // INITIALIZATION
            telemetry.addData("Status", "Initialized");
            telemetry.addLine("LB: Toggle intake ON/OFF");
            telemetry.addLine("RB: Hold transfer");
            telemetry.addLine("LT: Hold full speed");
            telemetry.addLine("B: Toggle outtake ON/OFF");
            telemetry.addLine("A: Reset calibration readings");
            telemetry.addLine("X: EMERGENCY // Reverse intake and transfer");
            telemetry.addLine("D-pad up/down: flywheel speed +/-");
            telemetry.addLine("D-pad right/left: counterroller speed +/-");

            board.reportTelemetry(telemetry);
            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            board.setFieldPosition(
                    START_X,
                    START_Y,
                    START_HEADING
            );

            board.resetCalibrationReadings();
            runtime.reset();

            boolean intakeOn = false;
            boolean leftBumperWasPressed = gamepad1.left_bumper;
            boolean outtakeOn = false;
            boolean bWasPressed = gamepad1.b;
            boolean aWasPressed = gamepad1.a;
            boolean upWasPressed = gamepad1.dpad_up;
            boolean downWasPressed = gamepad1.dpad_down;
            boolean rightWasPressed = gamepad1.dpad_right;
            boolean leftWasPressed = gamepad1.dpad_left;
            int outtakeAdjustments = 0;
            int counterspinAdjustments = 0;

            // MAIN TELEOP LOOP
            while (opModeIsActive()) {

                // Update robot position
                board.updateFieldPosition();

                // DRIVING
                double forward = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x;
                double strafe = gamepad1.right_stick_x;

                // RT = full speed (100%)
                // Otherwise 70%
                double speedScale =
                        gamepad1.left_trigger > 0.1 ? 1.0 : 0.7;

                board.drive(
                        forward,
                        strafe,
                        turn,
                        speedScale
                );

                // INTAKE: LEFT BUMPER TOGGLE
                boolean leftBumperPressed =
                        gamepad1.left_bumper;

                if (leftBumperPressed && !leftBumperWasPressed) {
                    intakeOn = !intakeOn;
                }

                leftBumperWasPressed = leftBumperPressed;

                // Controls both intake servos and intake motor
                board.setIntake(intakeOn);

                // TRANSFER: RIGHT BUMPER
                boolean transferOn = gamepad1.right_bumper;
                board.setTransfer(transferOn);

                // OUTTAKE: B BUTTON TOGGLE
                if (gamepad1.b && !bWasPressed) {
                    outtakeOn = !outtakeOn;
                }
                bWasPressed = gamepad1.b;

                int outtakeSteps = (gamepad2.dpad_up && !upWasPressed ? 1 : 0)
                        - (gamepad2.dpad_down && !downWasPressed ? 1 : 0);
                int counterspinSteps = (gamepad2.dpad_right && !rightWasPressed ? 1 : 0)
                        - (gamepad2.dpad_left && !leftWasPressed ? 1 : 0);
                upWasPressed = gamepad2.dpad_up;
                downWasPressed = gamepad2.dpad_down;
                rightWasPressed = gamepad2.dpad_right;
                leftWasPressed = gamepad2.dpad_left;
                if (board.adjustOuttakeVelocity(outtakeSteps)) {
                    outtakeAdjustments += outtakeSteps;
                }
                if (board.adjustCounterspinVelocity(counterspinSteps)) {
                    counterspinAdjustments += counterspinSteps;
                }
                board.setOuttake(outtakeOn);

                // CALIBRATION: A BUTTON
                if (gamepad1.a && !aWasPressed) {
                    board.resetCalibrationReadings();
                }

                // EMERGENCY REVERSE INTAKE/TRANSFER: X BUTTON
                if (gamepad1.x) {
                    board.setIntake(true, true);
                    board.setTransfer(true, true);
                }

                aWasPressed = gamepad1.a;

                // TELEMETRY
                telemetry.addData(
                        "Status",
                        "Run Time: " + runtime
                );

                telemetry.addData(
                        "Drive",
                        "Left: %.2f | Right: %.2f",
                        board.getLeftPower(),
                        board.getRightPower()
                );

                telemetry.addData(
                        "Intake",
                        intakeOn ? "ON" : "OFF"
                );

                telemetry.addData(
                        "Transfer",
                        transferOn ? "ON" : "OFF"
                );

                telemetry.addData(
                        "Outtake",
                        outtakeOn ? "ON" : "OFF"
                );

                telemetry.addData(
                        "Drive Speed",
                        speedScale == 1.0 ? "100%" : "70%"
                );

                telemetry.addData("Flywheel net adjustments", "%+d", outtakeAdjustments);
                telemetry.addData("Counterroller net adjustments", "%+d", counterspinAdjustments);
                board.reportTelemetry(telemetry);
                telemetry.update();

                idle();
            }

        } finally {
            // Stop all motors and servos
            board.stop();
        }
    }
}
