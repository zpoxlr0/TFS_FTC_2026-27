
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Unicorn TeleOp A")
public class UNICORN_TELEOP extends LinearOpMode {

    private final ProgrammingBoard board = new ProgrammingBoard();
    private final ElapsedTime runtime = new ElapsedTime();

    // Starting position for tracking
    private static final double START_X = 0;
    private static final double START_Y = 0;
    private static final double START_HEADING = 0;

    @Override
    public void runOpMode() {

        board.init(hardwareMap);

        try {

            // =========================
            // INITIALIZATION
            // =========================

            telemetry.addData("Status", "Initialized");
            telemetry.addLine("LB: Toggle intake ON/OFF");
            telemetry.addLine("RB: Hold outtake");
            telemetry.addLine("RT: Hold transfer");
            telemetry.addLine("B: Hold full speed");
            telemetry.addLine("A: Reset calibration readings");

            board.reportTelemetry(telemetry);
            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            // Initialize position tracking
            board.setFieldPosition(
                    START_X,
                    START_Y,
                    START_HEADING
            );

            board.resetCalibrationReadings();
            runtime.reset();

            // Intake toggle variables
            boolean intakeOn = false;
            boolean leftBumperWasPressed = gamepad1.left_bumper;

            // Calibration button variable
            boolean aWasPressed = gamepad1.a;

            // =========================
            // MAIN TELEOP LOOP
            // =========================

            while (opModeIsActive()) {

                // Update robot position
                board.updateFieldPosition();

                // =========================
                // DRIVING
                // =========================

                double forward = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x;
                double strafe = gamepad1.right_stick_x;

                // B = 100% speed
                // Otherwise = 70% speed
                double speedScale =
                        gamepad1.b ? 1.0 : 0.7;

                board.drive(
                        forward,
                        strafe,
                        turn,
                        speedScale
                );

                // =========================
                // INTAKE: LEFT BUMPER TOGGLE
                // =========================

                boolean leftBumperPressed =
                        gamepad1.left_bumper;

                // Toggle only once per press
                if (leftBumperPressed && !leftBumperWasPressed) {
                    intakeOn = !intakeOn;
                }

                leftBumperWasPressed = leftBumperPressed;

                // Controls:
                // Left intake servo
                // Right intake servo
                // Intake motor
                board.setIntake(intakeOn);

                // =========================
                // OUTTAKE: RIGHT BUMPER
                // =========================

                boolean outtakeOn =
                        gamepad1.right_bumper;

                board.setOuttake(outtakeOn);

                // =========================
                // TRANSFER: RIGHT TRIGGER
                // =========================

                boolean transferOn =
                        gamepad1.right_trigger > 0.1;

                board.setTransfer(transferOn);

                // =========================
                // CALIBRATION: A BUTTON
                // =========================

                // Reset once per button press
                if (gamepad1.a && !aWasPressed) {
                    board.resetCalibrationReadings();
                }

                aWasPressed = gamepad1.a;

                // =========================
                // TELEMETRY
                // =========================

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
                        "Outtake",
                        outtakeOn ? "ON" : "OFF"
                );

                telemetry.addData(
                        "Transfer",
                        transferOn ? "ON" : "OFF"
                );

                telemetry.addData(
                        "Drive Speed",
                        speedScale == 1.0 ? "100%" : "70%"
                );

                // Position tracking telemetry
                board.reportTelemetry(telemetry);

                telemetry.update();

                idle();
            }

        } finally {

            // =========================
            // STOP ALL MOTORS AND SERVOS
            // =========================

            board.stop();
        }
    }
}
