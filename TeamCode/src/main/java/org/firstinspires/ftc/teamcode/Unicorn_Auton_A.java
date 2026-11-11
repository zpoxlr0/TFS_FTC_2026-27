package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Unicorn Auton A")
public class Unicorn_Auton_A extends LinearOpMode {

    private final ProgrammingBoard board = new ProgrammingBoard();

    @Override
    public void runOpMode() {
        board.init(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        try {
            board.drive(0.6, 0, 0);
            sleep(2000);
            board.stop();

            sleep(20000);

            if (!opModeIsActive()) return;

            board.drive(-0.6, 0, 0);
            sleep(1700);
        } finally {
            board.stop();
        }
    }
}