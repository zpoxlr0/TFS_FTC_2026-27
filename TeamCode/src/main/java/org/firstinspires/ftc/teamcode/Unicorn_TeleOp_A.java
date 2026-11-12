
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Unicorn TeleOp A")
public class Unicorn_TeleOp_A extends LinearOpMode {

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

            // INITIALIZATION
            telemetry.addData("Status", "Initialized");
            telemetry.addLine("LB: Toggle intake ON/OFF");
            telemetry.addLine("RB: Hold transfer");
            telemetry.addLine("RT: Toggle outtake ON/OFF");
            telemetry.addLine("B: Hold full speed");
            telemetry.addLine("A: Reset calibration readings");

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
            boolean rightTriggerWasPressed = gamepad1.right_trigger > 0.1;
            boolean aWasPressed = gamepad1.a;

            // MAIN TELEOP LOOP
            while (opModeIsActive()) {

                // Update robot position
                board.updateFieldPosition();

                // DRIVING
                double forward = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x;
                double strafe = gamepad1.right_stick_x;

                // B = full speed (100%)
                // Otherwise 70%
                double speedScale =
                        gamepad1.b ? 1.0 : 0.7;

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

                // OUTTAKE: RIGHT TRIGGER TOGGLE
                boolean rightTriggerPressed = gamepad1.right_trigger > 0.1;
                if (rightTriggerPressed && !rightTriggerWasPressed) {
                    outtakeOn = !outtakeOn;
                }
                rightTriggerWasPressed = rightTriggerPressed;
                board.setOuttake(outtakeOn);

                // CALIBRATION: A BUTTON
                if (gamepad1.a && !aWasPressed) {
                    board.resetCalibrationReadings();
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
