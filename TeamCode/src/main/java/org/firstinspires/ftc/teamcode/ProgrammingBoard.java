package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ProgrammingBoard {

    // Drivetrain
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    // Ball system
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    private DcMotor outtakeMotor;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private boolean intakeOn = false;

    public void init(HardwareMap hardwareMap) {
        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive =
                hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive =
                hardwareMap.get(DcMotor.class, "right_back_drive");

        leftIntakeServo =
                hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo =
                hardwareMap.get(CRServo.class, "right_intake_servo");

        outtakeMotor =
                hardwareMap.get(DcMotor.class, "outtake_motor");
        intakeMotor =
                hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor =
                hardwareMap.get(DcMotor.class, "transfer_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);

        stop();
    }

    public void drive(double forward, double turn, double strafe) {
        double scale = Math.max(
                1.0, Math.abs(forward) + Math.abs(turn) + Math.abs(strafe)
        );

        leftFrontDrive.setPower((forward + turn + strafe) / scale);
        leftBackDrive.setPower((forward - turn + strafe) / scale);
        rightFrontDrive.setPower((forward - turn - strafe) / scale);
        rightBackDrive.setPower((forward + turn - strafe) / scale);
    }

    public void setIntake(boolean intakeOn) {
        this.intakeOn = intakeOn;

        leftIntakeServo.setPower(intakeOn ? -1.0 : 0.0);
        rightIntakeServo.setPower(intakeOn ? 1.0 : 0.0);
        intakeMotor.setPower(intakeOn ? 1.0 : 0.0);
    }

    public void setOuttake(boolean outtakeOn) {
        outtakeMotor.setPower(outtakeOn ? 0.8 : 0.0);
    }

    public void setTransfer(boolean transferOn) {
        transferMotor.setPower(transferOn ? 1.0 : 0.0);
    }

    public double getLeftPower() {
        return leftFrontDrive.getPower();
    }

    public double getRightPower() {
        return rightFrontDrive.getPower();
    }

    public boolean isIntakeOn() {
        return intakeOn;
    }

    public void stop() {
        drive(0.0, 0.0, 0.0);
        setIntake(false);
        setOuttake(false);
        setTransfer(false);
    }
}