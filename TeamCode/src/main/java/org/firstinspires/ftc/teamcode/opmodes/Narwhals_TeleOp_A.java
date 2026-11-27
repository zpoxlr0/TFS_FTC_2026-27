/*
 * MIT License
 * Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Narwhals TeleOp A", group = "Narwhals")
public class Narwhals_TeleOp_A extends OpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor intakeMotor;
    private DcMotorEx outtakeMotor;

    private CRServo transferServo;
    private CRServo leftIntake;
    private CRServo rightIntake;

    private static final int LAUNCHER_TARGET_VELOCITY = 1500; //test
    private static final int LAUNCHER_MIN_VELOCITY = 1400;

    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;

    private double intakePower;

    @Override
    public void init() {
        frontLeftMotor =
                hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor =
                hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightMotor =
                hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightMotor =
                hardwareMap.get(DcMotor.class, "back_right_motor");

        intakeMotor =
                hardwareMap.get(DcMotor.class, "intake_motor");
        outtakeMotor =
                hardwareMap.get(DcMotorEx.class, "outtake_motor");

        transferServo =
                hardwareMap.get(CRServo.class, "transfer_servo");
        leftIntake =
                hardwareMap.get(CRServo.class, "left_intake");
        rightIntake =
                hardwareMap.get(CRServo.class, "right_intake");

        // StarterBot direction convention.
        // Each motor can now be adjusted independently.

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[]{
                frontLeftMotor,
                backLeftMotor,
                frontRightMotor,
                backRightMotor
        }) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intakeMotor.setPower(0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Preserve the starter's encoder-based launcher control.
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5)
        );
        outtakeMotor.setVelocity(0);

        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        transferServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake.setPower(0);
        rightIntake.setPower(0);
        transferServo.setPower(0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        mecanumDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        // Right trigger intakes; left trigger reverses.
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // Right bumper runs launcher and feeds once it reaches speed.
        launch();

        intakePower = Math.max(-1.0, Math.min(1.0, intakePower));

        intakeMotor.setPower(intakePower);
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);

        telemetry.addData(
                "Front motors",
                "Left %.2f | Right %.2f",
                frontLeftPower,
                frontRightPower
        );
        telemetry.addData(
                "Back motors",
                "Left %.2f | Right %.2f",
                backLeftPower,
                backRightPower
        );
        telemetry.addData(
                "Outtake velocity",
                outtakeMotor.getVelocity()
        );
    }

    private void mecanumDrive(
            double forward,
            double strafe,
            double turn
    ) {
        double scale = Math.max(
                1.0,
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)
        );

        frontLeftPower = (forward + strafe + turn) / scale;
        backLeftPower = (forward - strafe + turn) / scale;
        frontRightPower = (forward - strafe - turn) / scale;
        backRightPower = (forward + strafe - turn) / scale;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void launch() {
        if (gamepad1.right_bumper) {
            outtakeMotor.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            outtakeMotor.setVelocity(0);
        }

        if (gamepad1.right_bumper
                && outtakeMotor.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            transferServo.setPower(1);
            intakePower += 0.5;
        } else {
            transferServo.setPower(0);
        }
    }

    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        intakeMotor.setPower(0);
        outtakeMotor.setVelocity(0);

        transferServo.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
}