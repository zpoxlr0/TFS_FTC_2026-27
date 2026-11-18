package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Unicorn_TeleOp_AutoAim")
public class ExperimentalAutoAim extends LinearOpMode {

    private final ProgrammingBoard board = new ProgrammingBoard();
    private final ElapsedTime aimTimer = new ElapsedTime();

    // Walls viewed from the audience.
    // The robot starts facing inward, away from the selected wall.
    private enum Wall {
        BOTTOM(0),
        LEFT(-90),
        TOP(180),
        RIGHT(90);

        final double heading;

        Wall(double heading) {
            this.heading = heading;
        }
    }

    private Wall startFromWall = Wall.BOTTOM;

    private boolean aiming = false;
    private double targetHeading;
    private String aimStatus = "Manual";

    // Fixed horizontal directions in our chosen heading frame.
    private static final double LEFT_HEADING = 90;
    private static final double RIGHT_HEADING = -90;

    private static final double AIM_TIMEOUT_SECONDS = 5.0;
    private static final double ANGLE_TOLERANCE = 2.0;

    // Starting tuning values.
    private static final double TURN_KP = 0.012;
    private static final double MIN_TURN_POWER = 0.10;
    private static final double MAX_TURN_POWER = 0.30;

    @Override
    public void runOpMode() {
        board.init(hardwareMap);

        try {
            boolean downWasPressed = false;

            // Select the starting wall before PLAY.
            while (opModeInInit()) {
                if (gamepad1.dpad_down && !downWasPressed) {
                    Wall[] walls = Wall.values();

                    startFromWall = walls[
                            (startFromWall.ordinal() + 1) % walls.length
                            ];
                }

                downWasPressed = gamepad1.dpad_down;

                telemetry.addData("Starting wall", startFromWall);
                telemetry.addLine("D-pad DOWN: change starting wall");
                telemetry.addLine("Face inward, then press PLAY");
                telemetry.update();

                idle();
            }

            waitForStart();

            if (isStopRequested()) return;

            // Establish the selected starting heading once.
            board.setFieldPosition(0, 0, startFromWall.heading);
            board.resetCalibrationReadings();

            boolean leftWasPressed = gamepad1.dpad_left;
            boolean rightWasPressed = gamepad1.dpad_right;
            boolean aWasPressed = gamepad1.a;

            while (opModeIsActive()) {
                board.updateFieldPosition();

                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                // Start a turn once per D-pad press.
                if (gamepad1.dpad_left && !leftWasPressed
                        && !gamepad1.dpad_right) {
                    startAim(LEFT_HEADING);

                } else if (gamepad1.dpad_right && !rightWasPressed
                        && !gamepad1.dpad_left) {
                    startAim(RIGHT_HEADING);
                }

                leftWasPressed = gamepad1.dpad_left;
                rightWasPressed = gamepad1.dpad_right;

                // Stick input immediately returns control to the driver.
                if (Math.abs(forward) > 0.15
                        || Math.abs(strafe) > 0.15
                        || Math.abs(turn) > 0.15) {
                    aiming = false;
                    aimStatus = "Manual";
                }

                if (aiming) {
                    // Rotate in place while aiming.
                    forward = 0;
                    strafe = 0;
                    turn = 0;

                    double heading = board.getHeading();

                    if (!Double.isFinite(heading)) {
                        aiming = false;
                        aimStatus = "IMU unavailable";

                    } else if (aimTimer.seconds()
                            >= AIM_TIMEOUT_SECONDS) {
                        aiming = false;
                        aimStatus = "Turn timed out";

                    } else {
                        turn = turnForHeading(
                                heading,
                                targetHeading
                        );

                        if (turn == 0) {
                            aiming = false;
                            aimStatus = "Aligned";
                        }
                    }
                }

                board.drive(forward, strafe, turn, aiming ? 1.0 : 0.5);

                // Your existing mechanism controls.
                board.setIntake(gamepad1.left_bumper);
                board.setOuttake(gamepad1.right_trigger > 0.1);
                board.setTransfer(gamepad1.right_bumper);

                // Preserve your calibration-reading reset.
                if (gamepad1.a && !aWasPressed) {
                    board.resetCalibrationReadings();
                }

                aWasPressed = gamepad1.a;

                telemetry.addData("Starting wall", startFromWall);
                telemetry.addData("Aim", aimStatus);

                if (aiming) {
                    telemetry.addData(
                            "Target heading",
                            targetHeading
                    );
                }

                board.reportTelemetry(telemetry);
                telemetry.update();

                idle();
            }
        } finally {
            board.stop();
        }
    }

    private void startAim(double heading) {
        if (!Double.isFinite(board.getHeading())) {
            aiming = false;
            aimStatus = "IMU unavailable: check hub configuration";
            return;
        }

        targetHeading = heading;
        aiming = true;
        aimStatus = "Turning";
        aimTimer.reset();
    }

    static double turnForHeading(double current, double target) {
        // Shortest signed angle, including across the -180/180 boundary.
        double error =
                ((target - current + 180) % 360 + 360) % 360 - 180;

        if (Math.abs(error) <= ANGLE_TOLERANCE) {
            return 0;
        }

        double power = Math.max(
                MIN_TURN_POWER,
                Math.min(
                        MAX_TURN_POWER,
                        Math.abs(error) * TURN_KP
                )
        );

        // Heading increases counterclockwise.
        // ProgrammingBoard.drive() expects clockwise-positive turn power.
        return -Math.copySign(power, error);
    }
}