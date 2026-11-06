package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Unicorn_TeleOp_A extends LinearOpMode {

    private final ProgrammingBoard board = new ProgrammingBoard();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        board.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        runtime.reset();

        while (opModeIsActive()) {
            // Driving inputs
            double forward = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double strafe = gamepad1.right_stick_x;

            board.drive(forward, turn, strafe);


            // Intake
            board.setIntake(gamepad1.left_bumper);

            // Run outtake while the right trigger is held.
            boolean outtakeOn = gamepad1.right_trigger > 0.1;
            board.setOuttake(outtakeOn);

            // Transfer
            board.setTransfer(gamepad1.right_bumper);

            // Telemetry
            telemetry.addData(
                    "Status",
                    "Run Time: " + runtime.toString()
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

            telemetry.update();
        }

        board.stop();
    }
}