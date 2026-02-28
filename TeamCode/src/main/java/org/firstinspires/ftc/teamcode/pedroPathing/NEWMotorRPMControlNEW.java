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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class NEWMotorRPMControlNEW extends OpMode {

    LLHardware robot = new LLHardware ();
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private final Pose redGoal = new Pose(140.25,135.2, Math.toRadians(0)); // 141.75 135.2  lastest #s 140/135 138 133.5 0  142 125 38 test #s
    //142 130 0 does not move  blueGoal 0 135 0   ***134,135  142/130
    private final Pose blueGoal = new Pose  (8,126, Math.toRadians(0));  //8,135 13 137   8/135
    static final double TICKS_PER_REV = 28;

    // Manual turret offset variables
    private double turretOffsetDegrees = 0.0;
    private final double TURRET_OFFSET_SPEED = 30.0; // Degrees per second when trigger held
    private final double MAX_TURRET_OFFSET = 15.0; // Maximum offset in degrees
    private boolean lastDpadDown = false;

    double targetPower = 0;
    double hoodPosition = 0.5;

    double targetPower2 = 0;
    double hoodPosition2 = 0.5;
    double increment = 0.02;
    double hoodIncrement = 0.03;

    DcMotorEx flywheel;
    Servo hoodServo;
    DigitalChannel beamBreaker1; // First position (closest to intake)
    DigitalChannel beamBreaker2; // Second position (middle)
    DigitalChannel beamBreaker3; // Third position (closest to shooter)

    // Ball tracking variables
    private int ballCount = 0;
    private final int MAX_BALLS = 3;
    private boolean lastBeam1Broken = false;
    private boolean lastBeam2Broken = false;
    private boolean lastBeam3Broken = false;
    private boolean ballJustShot = false;
    private boolean beam1Broken = false;
    private boolean beam2Broken = false;
    private boolean beam3Broken = false;
    private boolean beam1Raw = false;
    private boolean beam2Raw = false;
    private boolean beam3Raw = false;
    int rawTrueCount = (beam1Raw ? 1 : 0) + (beam2Raw ? 1 : 0) + (beam3Raw ? 1 : 0);

    // Rumble control for no balls warning
    private boolean hasRumbledForEmpty = false; // Track if we've already rumbled for current empty state

    // Debounce for empty detection (prevent false positives from holes in balls)
    private double emptyDetectionTimer = 0.0;
    private final double EMPTY_CONFIRMATION_DELAY = 0.3; // 300ms - must be empty for this long before confirming
    private boolean confirmedEmpty = false;

    // Intake delay timer - 500ms delay after 3 balls detected before stopping intake
    private double intakeDelayTimer = 0.0;
    private final double INTAKE_STOP_DELAY = 0.1; // 500ms delay
    private boolean intakeDelayActive = false;

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
        follower = Constants2.createFollower(hardwareMap);
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        hoodServo = hardwareMap.get(Servo.class, "servo1");

        // Initialize 3 beam breaker sensors
        beamBreaker1 = hardwareMap.get(DigitalChannel.class, "beamBreaker1"); // Configure these names in robot configuration
        beamBreaker2 = hardwareMap.get(DigitalChannel.class, "beamBreaker2");
        beamBreaker3 = hardwareMap.get(DigitalChannel.class, "beamBreaker3");

        // Set all as inputs
        beamBreaker1.setMode(DigitalChannel.Mode.INPUT);
        beamBreaker2.setMode(DigitalChannel.Mode.INPUT);
        beamBreaker3.setMode(DigitalChannel.Mode.INPUT);
        follower.setStartingPose(startingPose == null ? (new Pose(8.125, 8.875, 0)): startingPose);
//        follower.setStartingPose(startingPose == null ? new Pose(38.5, 117.25, Math.toRadians(142)) : startingPose);  //REDCLOSE: X:119 Y:98.5
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
        robot.spin.setTargetPosition(0);
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        follower.update();


    }

    @Override
    public void loop() {
        follower.update();

        // Read all 3 beam breaker sensors
        // Active-HIGH: true (HIGH) means object detected, false (LOW) means no object
        beam1Raw = beamBreaker1.getState();
        beam2Raw = beamBreaker2.getState();
        beam3Raw = beamBreaker3.getState();

        // Enforce sensor dependency chain:
        // - Beam 2 can only be HIGH if Beam 1 is HIGH
        // - Beam 3 can only be HIGH if Beam 2 is HIGH
        beam1Broken = beam1Raw;
        beam2Broken = beam2Raw && beam1Broken; // Only valid if beam1 is also broken
        beam3Broken = beam3Raw && beam2Broken; // Only valid if beam2 is also broken

        // Count balls based on which sensors detect objects (after validation)
        ballCount = 0;
        if (beam1Broken) ballCount++;
        if (beam2Broken) ballCount++;
        if (beam3Broken) ballCount++;

        // Check if robot is completely empty (no balls at all)
        boolean noBalls = !beam1Broken && !beam2Broken && !beam3Broken;

        // Debounce the empty state - must be empty for EMPTY_CONFIRMATION_DELAY before confirming
        // This prevents false positives when beam shines through holes in balls
        if (noBalls) {
            // Robot appears empty - increment timer
            emptyDetectionTimer += delta;

            // Only confirm empty after timer exceeds delay threshold
            if (emptyDetectionTimer >= EMPTY_CONFIRMATION_DELAY) {
                confirmedEmpty = true;
            }
        } else {
            // Robot has at least one ball - reset timer and confirmed state
            emptyDetectionTimer = 0.0;
            confirmedEmpty = false;
            hasRumbledForEmpty = false; // Reset rumble flag when we have balls
        }

        // Only rumble when we have CONFIRMED empty state (not just momentary)
        // This if statement will only be true ONCE when the robot becomes empty
        if (confirmedEmpty && !hasRumbledForEmpty) {
            // Robot has been confirmed empty and we haven't rumbled yet - RUMBLE ONCE
            gamepad1.rumble(1500); // Rumble once for 1.5 seconds
            gamepad2.rumble(1500);
            hasRumbledForEmpty = true; // Set flag to prevent repeated rumbles
        }

        lastBeam1Broken = beam1Broken;
        lastBeam2Broken = beam2Broken;
        lastBeam3Broken = beam3Broken;

        Pose currentPose = follower.getPose();
        Pose errorPose= currentGoal.minus(currentPose);
        double fieldTargetAngle = Math.toDegrees(Math.atan2(errorPose.getY(), errorPose.getX()));
        double robotHeading = Math.toDegrees(currentPose.getHeading());
        double targetAngle = fieldTargetAngle - robotHeading;
        double distanceToTarget = Math.hypot(errorPose.getY(), errorPose.getX());


        while (targetAngle > 180) targetAngle -= 360;
        while (targetAngle <= -180) targetAngle += 360;

        // Manual turret offset controls (gamepad2 triggers)
        // Left trigger = turn left (negative offset)
        // Right trigger = turn right (positive offset)
        double offsetAdjustment = 0.0;
//
//        if (gamepad2.left_trigger > 0.1) {
//            offsetAdjustment += gamepad2.left_trigger * TURRET_OFFSET_SPEED * delta;
//        }
//        if (gamepad2.right_trigger > 0.1) {
//            offsetAdjustment -= gamepad2.right_trigger * TURRET_OFFSET_SPEED * delta;
//        }

        turretOffsetDegrees += offsetAdjustment;
        turretOffsetDegrees = Range.clip(turretOffsetDegrees, -MAX_TURRET_OFFSET, MAX_TURRET_OFFSET);

        // Reset offset to zero with dpad down
        if (gamepad2.dpad_down && !lastDpadDown) {
            turretOffsetDegrees = 0.0;
        }

        // Save button state
        lastDpadDown = gamepad2.dpad_down;

        /*
         * If you have a max and min rotation distance you can use it here to set the min and max for the range clip
         * you can use two if statements to preemptively move the turret to the closest of the two maxes this means that
         * hopefully you turret will be on the other side of the dead zone before the robot finishes turning
         */

        // Apply manual offset to the calculated target angle
        double adjustedTargetAngle = targetAngle + turretOffsetDegrees;
        double hoodTargetAngle = Range.clip(adjustedTargetAngle, -90, 90);
        double ticksPerDegree = (537.7  * 5 ) / 360.0; // (TicksPerRev * gear ratio[100:20])/360
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
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    gamepad1.left_stick_y * slowModeMultiplier,
                    gamepad1.left_stick_x * slowModeMultiplier,
                    gamepad1.right_stick_x * slowModeMultiplier,
                    false // Robot Centric
            );
        }


        /** OUTPUT & AIM*/

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
//        if (hoodPosition > 1.0) hoodPosition = 1.0;
        if (hoodPosition < 0.0) hoodPosition = 0.0;
        // 3. Set Motor Power
        flywheel.setPower(targetPower);
        hoodServo.setPosition(hoodPosition);



        double idealPower = flywheelspeed(distanceToTarget);
        double targetVelocityTicks = idealPower * 2520;
        double actualVelocityTicks = flywheel.getVelocity();
        double velocityError = Math.abs(targetVelocityTicks - actualVelocityTicks);
        double tolerance = 100;

        double idealPower2 = flywheelspeedfar(distanceToTarget);
        double targetVelocityTicks2 = idealPower2 * 2520;
        double actualVelocityTicks2 = flywheel.getVelocity();
        double velocityError2 = Math.abs(targetVelocityTicks2 - actualVelocityTicks2);
        double tolerance2 = 115;


        /** INPUT */

        // Intake control with 500ms delay after 3 balls detected
        // Start delay timer when beam3 becomes broken (3 balls detected)
        if (beam3Broken && !intakeDelayActive) {
            intakeDelayActive = true;
            intakeDelayTimer = 0.0;
        }

        // Reset delay if beam3 is no longer broken (ball was shot or removed)
        if (!beam3Broken) {
            intakeDelayActive = false;
            intakeDelayTimer = 0.0;
        }

        // Increment timer if delay is active
        if (intakeDelayActive) {
            intakeDelayTimer += delta;
        }

        // Check if delay period has elapsed
        boolean intakeDisabled = intakeDelayActive && (intakeDelayTimer >= INTAKE_STOP_DELAY);

        // Intake control - can only spin if delay hasn't elapsed or beam3 is not broken
        if (gamepad1.left_trigger > 0.5){
            robot.ip.setPower(-1); //in
        }
        else if (gamepad1.right_trigger > 0.5) {
            robot.ip.setPower(1); //out (reverse always works)
        }
        else{
            robot.ip.setPower(0);
        }

        /** SPOON */

// Logic to run every loop cycle

        if (follower.getPose().getY() > 46) {
            if (velocityError <= tolerance) {
                robot.light.setPosition(0.50); // Green / Ready
            } else {
                robot.light.setPosition(0.0);  // Red / Not Ready
            }
        }else if (follower.getPose().getY() <= 46) {
            if (velocityError2 <= tolerance2) {
                robot.light.setPosition(0.50); // Green / Ready
            } else {
                robot.light.setPosition(0.0);  // Red / Not Ready
            }
        }
        /** BALL COUNT INDICATOR LIGHT */
        // Second light shows ball count status based on RAW sensor readings
        // Count how many raw sensors are true
        int rawTrueCount = 0;
        if (beam1Raw) rawTrueCount++;
        if (beam2Raw) rawTrueCount++;
        if (beam3Raw) rawTrueCount++;

        // Set light position based on raw count
        if (rawTrueCount == 0) {
            robot.light2.setPosition(0.0); // All false - Off/Red
        } else if (rawTrueCount == 1) {
            robot.light2.setPosition(0.28); // 1 true  red
        } else if (rawTrueCount == 2) {
            robot.light2.setPosition(0.388); // 2 true  yellow
        } else if (rawTrueCount == 3) {
            robot.light2.setPosition(0.500); // 3 true  green
        }

/** SPOON (LIFT) **/

        if (follower.getPose().getY() > 46) {
            if (gamepad1.right_bumper && velocityError <= tolerance ) {
                robot.lift.setPosition(0.47);
                timerValue = 0;
                isTimerActive = true;
            }
        }else if (follower.getPose().getY() <= 46){
            if (gamepad1.right_bumper && velocityError2 <= tolerance2) {
                robot.lift.setPosition(0.47);
                timerValue = 0;
                isTimerActive = true;
            }
        }
        if(timerValue >= timerUpperValue){
            robot.lift.setPosition(0.05);
            timerValue = 0;
            isTimerActive = false;
            // Ball count is now automatically tracked by sensors - no manual decrement needed
        }

        if(isTimerActive){
            timerValue += delta;
        }

        /** TURRENT */

        if(gamepad2.xWasPressed()) {
            currentGoal = blueGoal;
        }

        if(gamepad2.bWasPressed()) {
            currentGoal = redGoal;
        }



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
//    public static double flywheelspeed(double TD) {
//        double power = (0.000013538 * Math.pow(TD, 2)) + (0.000025701 * TD) + 0.45686;
//        return MathFunctions.clamp(power, 0.0, 1.0);
//    }

    //    public static double flywheelspeed(double TD) {
//        // Formula derived from 12 new data points (35in to 135in)
//        double power = (0.000019925 * Math.pow(TD, 2)) - (0.0007064 * TD) + 0.41205;
//        return MathFunctions.clamp(power, 0.0, 1.0);
//    }
    public static double flywheelspeed(double TD) {
        // Formula derived from 12 new data points (35in to 135in)
        double power = (0.000019616 * Math.pow(TD, 2)) - (0.000650882 * TD) + 0.410147;
        return MathFunctions.clamp(power, 0.0, 1.0);
    }

    public static double flywheelspeedfar(double TD) {
        // Formula derived from 12 new data points (35in to 135in)
        double power = (-0.0000428571 * Math.pow(TD, 2)) + (0.0141429 * TD) - 0.497714;
        return MathFunctions.clamp(power, 0.0, 1.0);
    }
    // Hood Angle Formula (Based on your new hood data)
//    public static double tiltangle(double TD) {
//        // This formula fits: 140in -> 0.77, 84in -> 0.50, 50in -> 0.50
//        double angle = (0.00004674 * Math.pow(TD, 2)) - (0.005517 * TD) + 0.6538;
//        // Clamped to 0.50 (minimum) and 0.80 (safe maximum)
//        return MathFunctions.clamp(angle, 0.25, 0.77);
//    }
//    public static double tiltangle(double TD) {
//        // Formula derived from 12 new data points (35in to 135in)
//        double angle = (-0.000046587 * Math.pow(TD, 2)) + (0.01135 * TD) + 0.26114;
//        return MathFunctions.clamp(angle, 0.50, 1.0);
//    }


    public static double tiltangle(double TD) {
        // Formula derived from 12 new data points (35in to 135in)
        double angle = (-0.0000332551 * Math.pow(TD, 2)) + (0.0088676 * TD) + 0.378357;
        return MathFunctions.clamp(angle, 0.50, 1.0);
    }
    public static double tiltangle2(double TD) {
        // Formula derived from 12 new data points (35in to 135in)
        double angle = (-0.0000285714 * Math.pow(TD, 2)) + (0.00782857 * TD) + 0.466857;
        return MathFunctions.clamp(angle, 0.50, 1.0);
    }
}