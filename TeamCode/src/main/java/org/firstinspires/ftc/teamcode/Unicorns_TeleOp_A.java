package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "Unicorns TeleOp A", group = "Unicorns")
public class Unicorns_TeleOp_A extends LinearOpMode {

    private final Unicorns_ProgrammingBoard board = new Unicorns_ProgrammingBoard();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final ElapsedTime runtime = new ElapsedTime();

    // Starting position for tracking
    private static final double START_X = -72;
    private static final double START_Y = -72;
    private static final double START_HEADING = 0;

    @Override
    public void runOpMode() {

        board.init(hardwareMap);
        board.loadShooterPidForTuning();

        try {

            // INITIALIZATION
            telemetry.addData("Status", "Initialized");
            telemetry.addLine("LB: Toggle intake ON/OFF");
            telemetry.addLine("RB: Hold transfer");
            telemetry.addLine("LT: Hold full speed");
            telemetry.addLine("B: Toggle outtake ON/OFF");
            telemetry.addLine("A: Reset calibration readings");
            telemetry.addLine("X: EMERGENCY // Reverse intake and transfer");
            telemetry.addLine("P2: D-pad up/down: flywheel speed +/-");
            telemetry.addLine("P2: D-pad right/left: counterroller speed +/-");

            board.reportTelemetry(telemetry);
            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

//            board.setFieldPosition(
//                    START_X,
//                    START_Y,
//                    START_HEADING
//            );
            SavedPose.Pose saved = SavedPose.consume();

            if (saved != null) {
                board.setFieldPosition(saved.x, saved.y, saved.heading);
            } else {
                // Must match the robot's actual starting pose if autonomous was not run.
                board.setFieldPosition(START_X, START_Y, START_HEADING);
            }

            board.resetCalibrationReadings();
            runtime.reset();

            boolean intakeOn = false;
            boolean leftBumperWasPressed = gamepad1.left_bumper;
            boolean outtakeOn = false;
            boolean bWasPressed = gamepad1.b;
            boolean aWasPressed = gamepad1.a;
            boolean upWasPressed = gamepad2.dpad_up;
            boolean downWasPressed = gamepad2.dpad_down;
            boolean rightWasPressed = gamepad2.dpad_right;
            boolean leftWasPressed = gamepad2.dpad_left;
            int outtakeAdjustments = 0;
            int counterspinAdjustments = 0;
            String shooterSpeedStatus = "No Dashboard speed update yet";

            // MAIN TELEOP LOOP
            while (opModeIsActive()) {

                // Update robot position
                board.updateFieldPosition();

                // DRIVING
                double forward = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x;
                double strafe = gamepad1.right_stick_x;

                // LT = full speed (100%)
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
                if (ShooterPidTuning.apply) {
                    board.applyShooterPidTuning();
                    ShooterPidTuning.apply = false;
                }
                if (ShooterPidTuning.applySpeed) {
                    double flywheel = ShooterPidTuning.flywheelSpeed;
                    double counterroller = ShooterPidTuning.counterrollerSpeed;

                    if (Double.isFinite(flywheel) && flywheel >= 0 && flywheel <= 1
                            && Double.isFinite(counterroller)
                            && counterroller >= 0 && counterroller <= 1) {
                        board.setShooterSpeedFractions(flywheel, counterroller);
                        shooterSpeedStatus = "Dashboard speeds applied";
                    } else {
                        shooterSpeedStatus = "Error: both shooter speeds must be between 0 and 1";
                    }

                    ShooterPidTuning.applySpeed = false;
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

                // FTC DASHBOARD TELEMETRY
                TelemetryPacket packet = new TelemetryPacket();

                packet.fieldOverlay()
                        .drawImage("/images/eyqmnhza25ph1.png", 0, 0, 144, 144);

                if (board.isTrackingReady()) {
                    double x = board.getX();
                    double y = board.getY();
                    double heading = Math.toRadians(board.getHeading());
                    double radius = 9.0; // Adjust to roughly half your robot's width, in inches.

                    packet.fieldOverlay()
                            .setStroke("blue")
                            .strokeCircle(x, y, radius)
                            .strokeLine(
                                    x, y,
                                    x + radius * Math.cos(heading),
                                    y + radius * Math.sin(heading)
                            );

                    packet.put("robotX", x);
                    packet.put("robotY", y);
                    packet.put("robotHeadingDeg", board.getHeading());
                }

                packet.put("Shooter speed status", shooterSpeedStatus);
                packet.put("outtakeTarget", outtakeOn ? board.getOuttakeTargetVelocity() : 0);
                packet.put("outtakeActual", board.getOuttakeVelocity());
                packet.put("counterspinTarget", outtakeOn ? board.getCounterspinTargetVelocity() : 0);
                packet.put("counterspinActual", board.getCounterspinVelocity());

                dashboard.sendTelemetryPacket(packet);

                idle();
            }

        } finally {
            // Stop all motors and servos
            board.stop();
        }
    }
}
