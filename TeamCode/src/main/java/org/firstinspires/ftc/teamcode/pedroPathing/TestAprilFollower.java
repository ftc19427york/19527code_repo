package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class TestAprilFollower extends LinearOpMode {
    private DcMotorEx leftmotor;
    private DcMotorEx rightmotor;

    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     DRIVE_GEAR_REDUCTION    = 30.24;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 90.0 * 3.14;

    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;

    @Override
    public void runOpMode() {
        leftmotor = hardwareMap.get(DcMotorEx.class, "lr");
        rightmotor = hardwareMap.get(DcMotorEx.class, "rr");

        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // moves 2 ft forward
        // for teleops, instead of 610, we'll put inside whileloop and
        // move based on input received from gamepad
        int leftTarget = (int)(610 * COUNTS_PER_MM);
        int rightTarget = (int)(610 * COUNTS_PER_MM);
        double TPS = (175/ 60) * COUNTS_PER_WHEEL_REV;

        waitForStart();

        leftmotor.setTargetPosition(leftTarget);
        rightmotor.setTargetPosition(rightTarget);

        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftmotor.setVelocity(TPS);
        rightmotor.setVelocity(TPS);

        while (opModeIsActive() && (leftmotor.isBusy() && rightmotor.isBusy())) {
        }
    }
}