package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class MotorRPMControl extends OpMode {

    LLHardware robot = new LLHardware ();
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private final Pose redGoal = new Pose(142,125, Math.toRadians(0)); //138 133.5 0  142 125 38 test #s
                                                                            //142 130 0 does not move
    static final double TICKS_PER_REV = 28;

    double targetPower = 0;
    double hoodPosition = 0.5;
    double increment = 0.02;
    double hoodIncrement = 0.03;

    DcMotorEx flywheel;
    Servo hoodServo;

    private Pose currentGoal = redGoal;

    boolean isTimerActive = false;
    double timerUpperValue = 0.01; // amount of time timer is active for
    double delta = 0.01;        // amount of time passed between loop calls (ideally)
    double timerValue = 0.0;

    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastLeft = false;
    boolean lastRight = false;



    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        hoodServo = hardwareMap.get(Servo.class, "servo1");
        follower.setStartingPose(startingPose == null ? (new Pose(124, 116.5, Math.toRadians(38))) : startingPose);  //REDCLOSE: X:119 Y:98.5
        //follower.update();    // blue 39 117 142
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot.spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.spin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }


    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        follower.update();


    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.getPose();
        Pose errorPose= currentGoal.minus(currentPose);
        double fieldTargetAngle = Math.toDegrees(Math.atan2(errorPose.getY(), errorPose.getX()));

        /*
         * You can use the odometry to calculate the distance to target rather than using the limelight. using the Math.hypot();
         * should work well enough.
         */
        double robotHeading = Math.toDegrees(currentPose.getHeading());
        double targetAngle = fieldTargetAngle - robotHeading;

        double distanceToTarget = Math.hypot(errorPose.getY(), errorPose.getX());


        while (targetAngle > 180) targetAngle -= 360;
        while (targetAngle <= -180) targetAngle += 360;

        /*
         * If you have a max and min rotation distance you can use it here to set the min and max for the range clip
         * you can use two if statements to preemptively move the turret to the closest of the two maxes this means that
         * hopefully you turret will be on the other side of the dead zone before the robot finishes turning
         */

        double hoodTargetAngle = Range.clip(targetAngle, -90, 90);
        double ticksPerDegree = (537.6  * 5 ) / 360.0; // (TicksPerRev * gear ratio[100:20])/360
        int targetTicks = (int)(hoodTargetAngle * ticksPerDegree);
        robot.spin.setTargetPosition(targetTicks);
        robot.spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spin.setPower(1.0);




        //Call this once per loop
       // follower.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }


        /** OUTPUT & AIM*/

        targetPower = flywheelspeed(distanceToTarget);
        hoodPosition = tiltangle(distanceToTarget);


        if (gamepad1.dpad_up && !lastUp) {
            targetPower += increment;
        }
        if (gamepad1.dpad_down && !lastDown) {
            targetPower -= increment;
        }
        if (gamepad1.dpad_right && !lastRight) {
            hoodPosition += hoodIncrement;
        }
        // Increment down
        if (gamepad1.dpad_left && !lastLeft) {
            hoodPosition -= hoodIncrement;
        }
        // Save button states to prevent rapid-fire clicking
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        // Clip power between 0 and 1
        if (targetPower > 1.0) targetPower = 1.0;
        if (targetPower < 0.0) targetPower = 0.0;
        if (hoodPosition > 1.0) hoodPosition = 1.0;
        if (hoodPosition < 0.0) hoodPosition = 0.0;
        // 3. Set Motor Power
        flywheel.setPower(targetPower);
        hoodServo.setPosition(hoodPosition);



        /** INPUT */

        if (gamepad1.left_trigger > 0.5){
            robot.ip.setPower(-1); //in
        }
        else if (gamepad1.right_trigger > 0.5) {
            robot.ip.setPower(1); //out
        }
        else{
            robot.ip.setPower(0);
        }

        /** SPOON */


        if (gamepad1.right_bumper) {
            robot.lift.setPosition(0.47);//spoon up
            timerValue = 0;
            isTimerActive = true;
        }

        if(timerValue >= timerUpperValue){
            robot.lift.setPosition(0.05);
            timerValue = 0;
            isTimerActive = false;
        }

        if(isTimerActive){
            timerValue += delta;
        }

        /** TURRENT */



        double currentTicksPerSecond = flywheel.getVelocity();
        double currentRPM = (currentTicksPerSecond / TICKS_PER_REV) * 60.0;

        telemetry.addData("Target Power", "%.2f", targetPower);
        telemetry.addData("Velocity (RPM)", "%.1f", currentRPM);
        telemetry.addData("Velocity (Ticks/Sec)", "%.0f", currentTicksPerSecond);
        telemetry.addData("Hood Position", "%.2f", hoodPosition);
        telemetry.addLine("--- LIVE COORDINATES ---");
        telemetry.addData("X (Inches)", "%.2f in", follower.getPose().getX());
        telemetry.addData("Y (Inches)", "%.2f in", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", "%.2f°", robotHeading);
        telemetry.addData("Distance To Target", distanceToTarget);
        telemetry.update();



        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }

        //Slow Mode


        //Goal change


//        telemetry.addData("position", follower.getPose());
//        telemetry.addData("velocity", follower.getVelocity());
//        telemetry.addData("automatedDrive", automatedDrive);


//        if (five >= getRuntime() && isEndGame){
//            gamepad1.rumble(1);
//            gamepad2.rumble(1);
//        }



    }

    // Shooter Power Formula (Based on your power data)
    public static double flywheelspeed(double TD) {
        double power = (0.000013538 * Math.pow(TD, 2)) + (0.000025701 * TD) + 0.45686;
        return MathFunctions.clamp(power, 0.0, 1.0);
    }

    // Hood Angle Formula (Based on your new hood data)
    public static double tiltangle(double TD) {
        // This formula fits: 140in -> 0.77, 84in -> 0.50, 50in -> 0.50
        double angle = (0.00004674 * Math.pow(TD, 2)) - (0.005517 * TD) + 0.6538;
        // Clamped to 0.50 (minimum) and 0.80 (safe maximum)
        return MathFunctions.clamp(angle, 0.50, 0.80);
    }
}
