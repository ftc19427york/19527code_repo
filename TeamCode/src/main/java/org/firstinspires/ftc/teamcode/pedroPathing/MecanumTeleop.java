package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import java.util.function.Supplier;

@Configurable
@TeleOp
public class MecanumTeleop extends OpMode {

    LLHardware robot = new LLHardware ();

    public Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;


    boolean isTimerActive = false;
    double timerUpperValue = 0.6; // amount of time timer is active for
    double delta = 0.01;        // amount of time passed between loop calls (ideally)
    double timerValue = 0.0;





    @Override
    public void init() {

        robot.init(hardwareMap);
//        robot.imu.resetYaw();
        follower = Constants2.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose())))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {

        robot.lift.setPosition(0.05);
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
            //If you don't pass anything in, it uses the default (false)
            follower.startTeleopDrive();
}

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

//        double deltaX = 144 -  follower.getPose().getX();
//        double deltaY = 144 -  follower.getPose().getY();
//
//
//        double fieldTargetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
//
//        double robotHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//
//        double targetAngle = (0) - robotHeading;
//
//        double ticksPerDegree = (537.6  * 5 ) / 360.0; // (TicksPerRev * gear ratio[100:20])/360
//
//        double hoodTargetAngle = fieldTargetAngle - robotHeading;
//
//        telemetry.addData("angle: ", targetAngle);
//        telemetry.addData("Heading: ", robotHeading);
//        telemetry.addData("X: ",   follower.getPose().getX());
//        telemetry.addData("Y: ",   follower.getPose().getY());
//
//        int targetTicks = (int)(hoodTargetAngle * ticksPerDegree);
//
//        robot.spin.setTargetPosition(targetTicks);
//        robot.spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.spin.setPower(0.5); // Adjust for speed







        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors


//            Orientation angles = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
//            double angle = angles.firstAngle;
//
//            //gets squared values from the driver's stick input**/
//            double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
//            //finds the desired angle that the driver wants to move the robot**/
//            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//            //sets the movement angle by finding the difference of the robots angle, the input angle and the offset value
//            // the offset value is set by the the driver if the imu does not reset after auto*/
//            robotAngle = robotAngle - Math.toRadians(angle);
//
//            double rightX = gamepad1.right_stick_x;
//
//            final double v1 = r * Math.cos(robotAngle) + rightX;
//            final double v2 = r * Math.sin(robotAngle) - rightX;
//            final double v3 = r * Math.sin(robotAngle) + rightX;
//            final double v4 = r * Math.cos(robotAngle) - rightX;
//
//            // Output the safe vales to the motor drives.++
//
//            robot.lf.setPower(v4);    //motor3
//            robot.rf.setPower(v2);   //motor2
//            robot.lr.setPower(v3);     //motor1
//            robot.rr.setPower(v1);    //motor
//
//            telemetry.addData("sticks", "%.2f , %.2f, %.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, angle);


//            //This is the normal version to use in the TeleOp
            double slowModeMultiplier = 0.4;
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad2.left_stick_y,
                    -gamepad2.left_stick_x,
                    -gamepad2.right_stick_x,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad2.left_stick_y * slowModeMultiplier,
                    -gamepad2.left_stick_x * slowModeMultiplier,
                    -gamepad2.right_stick_x * slowModeMultiplier,
                    false // Robot Centric
            );
        }

        /** OUTPUT & AIM*/
if(gamepad1.a){
    robot.tilt.setPosition(0.25);
}else if(gamepad1.y){
            robot.tilt.setPosition(0.75);
        }else if(gamepad1.x){
    robot.tilt.setPosition(0);
}


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

//        if (gamepad2.dpad_down) {
//            robot.lift.setPosition(0.05); //spoon down
//        }
        if (gamepad2.dpad_up) {
            robot.lift.setPosition(0.42);//spoon up
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


        /** ELSE */







        /*
        boolean isTimerActive = false;
        double timerUpperValue = 1 // amount of time timer is active for
        double delta = 0.01         // amount of time passed between loop calls (ideally)
        double timerValue = 0.0     // current value of timer

        when button pressed down:
            do action
            timerValue = 0
            isTimerActive = true

        if timerValue >= timerUpperValue:
            do inverse of action
            timerValue = 0
            isTimerActive = false

        if isTimerActive:
            timerValue += delta
         */






        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }

        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
//
        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        } else if (gamepad1.leftBumperWasPressed()) {
            slowMode = false;
        }

        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad2.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
