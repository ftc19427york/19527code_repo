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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class turretFollowing extends OpMode {

    LLHardware robot = new LLHardware ();
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private final Pose redGoal = new Pose(138,133.5, Math.toRadians(0));
    private final Pose blueGoal = new Pose (7,137, Math.toRadians(0));

    private final Pose redPose = new Pose(119,98.5, Math.toRadians(0));

    private final Pose bluePose = new Pose(23.5,95, Math.toRadians(180));

    private final Pose zeroPose = new Pose(8.125, 8.875, Math.toRadians(0));

    private final Pose zeroBlue = new Pose(134.5,8.5, Math.toRadians(180));

    private final Pose leftPose = new Pose(34,6, Math.toRadians(90));

    private final Pose rightPose = new Pose(106.5,6, Math.toRadians(90));
    private Pose currentPose = redPose;

    private Pose currentGoal = redGoal;

    boolean isTimerActive = false;
    double timerUpperValue = 0.6; // amount of time timer is active for
    double delta = 0.01;        // amount of time passed between loop calls (ideally)
    double timerValue = 0.0;

    double endGameStart;

    double five;

    double four;

    double three;

    double two;

    double one;

    double end;

    boolean isEndGame;

    @Override
    public void init() {
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        if(gamepad2.x) {
            currentPose = bluePose;
        }

        if(gamepad2.b) {
            currentPose = redPose;
        }

        if(gamepad2.y) {
            currentPose = zeroPose;
        }

        if(gamepad2.a) {
            currentPose = zeroBlue;
        }

        if(gamepad2.left_bumper) {
            currentPose = leftPose;
        }

        if(gamepad2.right_bumper) {
            currentPose = rightPose;
        }


        follower.setStartingPose(startingPose == null ? currentPose : startingPose);  //REDCLOSE: X:119 Y:98.5
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(8.125, 8.875, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }
    

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        endGameStart = getRuntime() + 100;

        five = getRuntime() + 115;
        four = getRuntime() + 116;
        three = getRuntime() + 117;
        two = getRuntime() + 118;
        one = getRuntime() + 119;
        end = getRuntime() + 120;

    }

    @Override
    public void loop() {



        Pose errorPose = currentGoal.minus(follower.getPose());

        double fieldTargetAngle = Math.toDegrees(Math.atan2(errorPose.getY(), errorPose.getX()));

        /*
         * You can use the odometry to calculate the distance to target rather than using the limelight. using the Math.hypot();
         * should work well enough.
         */
        double distanceToTarget = Math.hypot(errorPose.getY(), errorPose.getX());

        double robotHeading = Math.toDegrees(follower.getPose().getHeading());

        double ticksPerDegree = (537.6  * 5 ) / 360.0; // (TicksPerRev * gear ratio[100:20])/360

        double targetAngle = fieldTargetAngle - robotHeading;

        /*
         * If you have a max and min rotation distance you can use it here to set the min and max for the range clip
         * you can use two if statements to preemptively move the turret to the closest of the two maxes this means that
         * hopefully you turret will be on the other side of the dead zone before the robot finishes turning
         */

        double hoodTargetAngle = Range.clip(targetAngle, -50, 60.0);





        //Call this once per loop
        follower.update();
        telemetry.update();

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



        if (gamepad2.right_bumper){
            robot.op.setPower(flywheelspeed(distanceToTarget));
            robot.tilt.setPosition(tiltangle(distanceToTarget));
        } else {
            robot.op.setPower(0);
        }


        telemetry.addData("Fly Wheel: ", flywheelspeed(distanceToTarget));
        telemetry.addData("Tilt: ", tiltangle(distanceToTarget));


        /** INPUT */

        if (gamepad2.left_trigger > 0.5){
            robot.ip.setPower(-1); //in
        }
        else if (gamepad2.right_trigger > 0.5) {
            robot.ip.setPower(1); //out
        }
        else{
            robot.ip.setPower(0);
        }

        /** SPOON */


        if (gamepad2.dpad_up) {
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


        /** SLOW MODE */

        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        } else if (gamepad1.leftBumperWasPressed()) {
            slowMode = false;
        }



        /** TURRENT */
        if(targetAngle < -52) {
            hoodTargetAngle = -52;
        }

        if(targetAngle > 63.0) {
            hoodTargetAngle = 63;
        }


        telemetry.addData("Timer: ", getRuntime());
        telemetry.addData("dist: ", distanceToTarget);
        telemetry.addData("angle: ", hoodTargetAngle);
        telemetry.addData("Heading: ", robotHeading);
        telemetry.addData("X: ",   follower.getPose().getX());
        telemetry.addData("Y: ",   follower.getPose().getY());
        telemetry.addData("delta X: ",   errorPose.getX());
        telemetry.addData("delta Y: ",   errorPose.getY());
        telemetry.addData("tiltangle", tiltangle(distanceToTarget));
        telemetry.addData("robot angle",follower.getPose().getHeading());

        int targetTicks = (int)(hoodTargetAngle * ticksPerDegree);

                robot.spin.setTargetPosition(targetTicks);
                robot.spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.spin.setPower(1.0);




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
        if(gamepad1.xWasPressed()) {
            currentGoal = blueGoal;
        }

        if(gamepad1.bWasPressed()) {
            currentGoal = redGoal;
        }

//        telemetry.addData("position", follower.getPose());
//        telemetry.addData("velocity", follower.getVelocity());
//        telemetry.addData("automatedDrive", automatedDrive);

        if (endGameStart >= getRuntime() && !isEndGame){
            gamepad1.rumble(0.5, 0.5, 500);
            gamepad2.rumble(0.5, 0.5, 500);
            isEndGame = true;
        }
//        if (five >= getRuntime() && isEndGame){
//            gamepad1.rumble(1);
//            gamepad2.rumble(1);
//        }



    }

    public static double flywheelspeed(double TD) {
        return MathFunctions.clamp(-0.00000136581 * Math.pow(TD, 2) + 0.00242865 * TD + 0.350574, 0, 1);
    }

    public static double tiltangle(double TD) {
        return MathFunctions.clamp(((0.73-0.25)/(193)*TD) + 0.25, 0.25, 0.73);
    }
}
