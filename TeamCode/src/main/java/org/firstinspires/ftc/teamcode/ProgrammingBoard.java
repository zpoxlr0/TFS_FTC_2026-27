package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ProgrammingBoard {

    // Replace null after identifying the direction the broad REV-logo face points.
    public static final RevHubOrientationOnRobot.LogoFacingDirection
            HUB_LOGO_FACING =
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;

    // You confirmed that the USB ports face forward.
    public static final RevHubOrientationOnRobot.UsbFacingDirection
            HUB_USB_FACING =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
    private Stampede stampede;

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

        // Preserved from your original ProgrammingBoard.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);

        stop();

        // RUN_WITHOUT_ENCODER still lets us read encoder positions.
        // It preserves direct power control without adding velocity PID tuning.
        for (DcMotor motor : new DcMotor[]{
                leftFrontDrive,
                leftBackDrive,
                rightFrontDrive,
                rightBackDrive
        }) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU imu = null;

        if (HUB_LOGO_FACING != null) {
            imu = hardwareMap.get(IMU.class, "imu");

            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            HUB_LOGO_FACING,
                            HUB_USB_FACING
                    )
            ));
        }

        // Stampede reads the motors already owned by ProgrammingBoard.
        stampede = new Stampede(
                leftFrontDrive,
                leftBackDrive,
                rightFrontDrive,
                rightBackDrive,
                imu
        );
    }

    public void drive(double forward, double strafe, double turn, double speedScale) {
        double scale = Math.max(
                1.0,
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)
        );

        leftFrontDrive.setPower(
                (forward + strafe + turn) / scale * speedScale
        );
        leftBackDrive.setPower(
                (forward - strafe + turn) / scale * speedScale
        );
        rightFrontDrive.setPower(
                (forward - strafe - turn) / scale * speedScale
        );
        rightBackDrive.setPower(
                (forward + strafe - turn) / scale * speedScale
        );
    }

    public void setIntake(boolean intakeOn) {
        this.intakeOn = intakeOn;

        leftIntakeServo.setPower(intakeOn ? -1.0 : 0.0);
        rightIntakeServo.setPower(intakeOn ? 1.0 : 0.0);
        intakeMotor.setPower(intakeOn ? 1.0 : 0.0);
    }

    public void setOuttake(boolean outtakeOn) {
        outtakeMotor.setPower(outtakeOn ? 0.67 : 0.0);
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

    public void updateFieldPosition() {
        stampede.updateFieldPosition();
    }

    public void setFieldPosition(double x, double y, double heading) {
        stampede.setFieldPosition(x, y, heading);
    }

    public double getX() {
        return stampede.getX();
    }

    public double getY() {
        return stampede.getY();
    }

    public double getHeading() {
        return stampede.getHeading();
    }

    public boolean isTrackingReady() {
        return stampede.isTrackingReady();
    }

    public void resetCalibrationReadings() {
        stampede.resetCalibrationReadings();
    }

    public void reportTelemetry(Telemetry telemetry) {
        stampede.reportTelemetry(telemetry);

        telemetry.addData(
                "Wheel powers (LF/LB/RF/RB)",
                "%.2f / %.2f / %.2f / %.2f",
                leftFrontDrive.getPower(),
                leftBackDrive.getPower(),
                rightFrontDrive.getPower(),
                rightBackDrive.getPower()
        );
    }

    public void stop() {
        drive(0.0, 0.0, 0.0, 0.0);
        setIntake(false);
        setOuttake(false);
        setTransfer(false);
    }
}