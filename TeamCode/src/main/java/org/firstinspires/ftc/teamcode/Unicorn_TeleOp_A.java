package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Unicorn_TeleOp_A extends LinearOpMode {

    private final ProgrammingBoard board = new ProgrammingBoard();
    private final ElapsedTime runtime = new ElapsedTime();

    // Local test origin at PLAY.
    // Later, these can represent your actual field starting position.
    private static final double START_X = 0;
    private static final double START_Y = 0;
    private static final double START_HEADING = 0;

    @Override
    public void runOpMode() {
        board.init(hardwareMap);

        try {
            telemetry.addData("Status", "Initialized");
            telemetry.addLine(
                    "A: zero calibration readings while stationary"
            );

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

            boolean aWasPressed = gamepad1.a;

            while (opModeIsActive()) {
                board.updateFieldPosition();

                // Driving inputs: same assignments as your original code.
                double forward = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x;
                double strafe = gamepad1.right_stick_x;
                double speedScale = gamepad1.right_bumper ? 1.0 : 0.7;

                board.drive(forward, strafe, turn, speedScale);

                // Intake
                board.setIntake(gamepad1.left_bumper);

                // Outtake
                boolean outtakeOn = gamepad1.b;
                board.setOuttake(outtakeOn);

                // Transfer
                board.setTransfer(gamepad1.x);

                // One reset per A-button press.
                if (gamepad1.a && !aWasPressed) {
                    board.resetCalibrationReadings();
                }

                aWasPressed = gamepad1.a;

                telemetry.addData(
                        "Status",
                        "Run Time: " + runtime
                );

                telemetry.addData(
                        "Inputs (forward/right/CW)",
                        "%.2f / %.2f / %.2f",
                        forward,
                        strafe,
                        turn
                );

                telemetry.addData(
                        "Drive",
                        "Left: %.2f | Right: %.2f",
                        board.getLeftPower(),
                        board.getRightPower()
                );

                telemetry.addData(
                        "Intake",
                        board.isIntakeOn() ? "ON" : "OFF"
                );

                telemetry.addData(
                        "Outtake",
                        outtakeOn ? "ON" : "OFF"
                );

                board.reportTelemetry(telemetry);
                telemetry.update();

                idle();
            }
        } finally {
            board.stop();
        }
    }
}