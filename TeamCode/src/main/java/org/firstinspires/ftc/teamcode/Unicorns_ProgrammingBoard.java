package org.firstinspires.ftc.teamcode;

// Test github version control

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Unicorns_ProgrammingBoard {

    private static final double OUTTAKE_INITIAL_SPEED = 0.63;
    private static final double COUNTERSPIN_INITIAL_SPEED = 0.15;
    private static final double SHOOTER_SPEED_STEP = 0.05;
    private static final double MAX_HUB_VELOCITY = 32767.0;

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

    private DcMotorEx outtakeMotor;
    private DcMotorEx counterspinMotor;
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    private boolean intakeOn = false;
    private double outtakeMaxVelocity;
    private double counterspinMaxVelocity;
    private double outtakeTargetVelocity;
    private double counterspinTargetVelocity;

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
                hardwareMap.get(DcMotorEx.class, "outtake_motor");
        counterspinMotor =
                hardwareMap.get(DcMotorEx.class, "counterspin_motor");
        intakeMotor =
                hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor =
                hardwareMap.get(DcMotor.class, "transfer_motor");

        outtakeMaxVelocity = Math.min(MAX_HUB_VELOCITY,
                outtakeMotor.getMotorType().getAchieveableMaxTicksPerSecondRounded());
        counterspinMaxVelocity = Math.min(MAX_HUB_VELOCITY,
                counterspinMotor.getMotorType().getAchieveableMaxTicksPerSecondRounded());
        outtakeTargetVelocity = (int) (OUTTAKE_INITIAL_SPEED * outtakeMaxVelocity);
        counterspinTargetVelocity = (int) (COUNTERSPIN_INITIAL_SPEED * counterspinMaxVelocity);

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
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        counterspinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        turn = turn * (speedScale == 1 ? 1 : 0.67); // experimental thing that will hopefully make turning precision better

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

    public void setIntake(boolean intakeOn, boolean reverse) {
        this.intakeOn = intakeOn;

        leftIntakeServo.setPower(intakeOn ? -1.0 : 0.0);
        rightIntakeServo.setPower(intakeOn ? 1.0 : 0.0);

        if (reverse) {
            intakeMotor.setPower(intakeOn ? -1.0 : 0.0);
        } else {
            intakeMotor.setPower(intakeOn ? 1.0 : 0.0);
        }
    }

    public void setIntake(boolean intakeOn) {
        setIntake(intakeOn, false);
    }

    public void setOuttake(boolean outtakeOn) {
        outtakeMotor.setVelocity(outtakeOn ? outtakeTargetVelocity : 0.0);
        counterspinMotor.setVelocity(outtakeOn ? counterspinTargetVelocity : 0.0);
    }

    public boolean adjustOuttakeVelocity(int steps) {
        double previousVelocity = outtakeTargetVelocity;
        outtakeTargetVelocity = Math.max(0, Math.min(outtakeMaxVelocity,
                outtakeTargetVelocity + steps * SHOOTER_SPEED_STEP * outtakeMaxVelocity));
        return outtakeTargetVelocity != previousVelocity;
    }

    public boolean adjustCounterspinVelocity(int steps) {
        double previousVelocity = counterspinTargetVelocity;
        counterspinTargetVelocity = Math.max(0, Math.min(counterspinMaxVelocity,
                counterspinTargetVelocity + steps * SHOOTER_SPEED_STEP * counterspinMaxVelocity));
        return counterspinTargetVelocity != previousVelocity;
    }

    public double getOuttakeTargetVelocity() {
        return outtakeTargetVelocity;
    }

    public double getOuttakeVelocity() {
        return outtakeMotor.getVelocity();
    }

    public double getCounterspinTargetVelocity() {
        return counterspinTargetVelocity;
    }

    public double getCounterspinVelocity() {
        return counterspinMotor.getVelocity();
    }

    public void loadShooterPidForTuning() {
        PIDFCoefficients outtake =
                outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients counterspin =
                counterspinMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Change these values to the actual PIDF constants when we find them.
        ShooterPidTuning.outtakeP = outtake.p;
        ShooterPidTuning.outtakeI = outtake.i;
        ShooterPidTuning.outtakeD = outtake.d;
        ShooterPidTuning.outtakeF = outtake.f;

        ShooterPidTuning.counterspinP = counterspin.p;
        ShooterPidTuning.counterspinI = counterspin.i;
        ShooterPidTuning.counterspinD = counterspin.d;
        ShooterPidTuning.counterspinF = counterspin.f;
        ShooterPidTuning.apply = false;
    }

    public void applyShooterPidTuning() {
        outtakeMotor.setVelocityPIDFCoefficients(
                ShooterPidTuning.outtakeP,
                ShooterPidTuning.outtakeI,
                ShooterPidTuning.outtakeD,
                ShooterPidTuning.outtakeF
        );
        counterspinMotor.setVelocityPIDFCoefficients(
                ShooterPidTuning.counterspinP,
                ShooterPidTuning.counterspinI,
                ShooterPidTuning.counterspinD,
                ShooterPidTuning.counterspinF
        );
    }

    public void setTransfer(boolean transferOn, boolean reverse) {
        if (reverse) {
            transferMotor.setPower(transferOn ? -1.0 : 0.0);
        } else {
            transferMotor.setPower(transferOn ? 1.0 : 0.0);
        }
    }

    public void setTransfer(boolean transferOn) {
        setTransfer(transferOn, false);
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
        reportShooterTelemetry(telemetry);

        telemetry.addData(
                "Wheel powers (LF/LB/RF/RB)",
                "%.2f / %.2f / %.2f / %.2f",
                leftFrontDrive.getPower(),
                leftBackDrive.getPower(),
                rightFrontDrive.getPower(),
                rightBackDrive.getPower()
        );
    }

    public void reportShooterTelemetry(Telemetry telemetry) {
        telemetry.addData("Outtake target (ticks/s)", "%.1f", outtakeTargetVelocity);
        telemetry.addData("Outtake velocity (ticks/s)", "%.1f", outtakeMotor.getVelocity());
        telemetry.addData("Counterspin target (ticks/s)", "%.1f", counterspinTargetVelocity);
        telemetry.addData("Counterspin velocity (ticks/s)", "%.1f", counterspinMotor.getVelocity());
    }

    public void stop() {
        drive(0.0, 0.0, 0.0, 0.0);
        setIntake(false);
        setOuttake(false);
        setTransfer(false);
    }
}