package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import static org.firstinspires.ftc.teamcode.pedroPathing.turretFollowing.flywheelspeed;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ABF_2Set", group = "Blue Far")
public class AutoBlueFar_2Set extends OpMode {
    LLHardware robot = new LLHardware ();
    private Timer pathTimer, opmodeTimer;

    //    private ElapsedTime runtime = new ElapsedTime();
//    private boolean moving = false;
//    boolean isTimerActive = false;
//    double timerUpperValue = 1; // amount of time timer is active for
//    double delta = 0.01;        // amount of time passed between loop calls (ideally)
//    double timerValue = 0.0;
//
//    boolean isWaitActive = false;
//    double waitUpper = 1; // amount of time timer is active for
//    double beta = 0.01;        // amount of time passed between loop calls (ideally)
//    double wait = 0.0;
    DcMotorEx flywheel;
//    double targetPower = 0;

    double velocityError = 0.0;

    double tolerance = 50;
    private final Pose redGoal = new Pose(134.5,126, Math.toRadians(0)); //138 133.5 0  142 125 38 test #s134.5/126

    private Pose currentGoal = redGoal;
    static final double TICKS_PER_REV = 28;



    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
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

//
//        while (targetAngle > 180) targetAngle -= 360;
//        while (targetAngle <= -180) targetAngle += 360;

//        double idealPower = flywheelspeed(distanceToTarget);
//        targetPower = flywheelspeed(distanceToTarget);

//        double targetVelocityTicks = idealPower * 2520;
        double actualVelocityTicks = flywheel.getVelocity();
//        velocityError = Math.abs(targetVelocityTicks - actualVelocityTicks);
        double currentTicksPerSecond = flywheel.getVelocity();
        double currentRPM = (currentTicksPerSecond / TICKS_PER_REV) * 60.0;
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Target Power", "%.2f", flywheel.getPower());
        telemetry.addData("Hood Position", "%.2f", robot.tilt.getPosition());
        telemetry.addData("Distance To Target", distanceToTarget);
        telemetry.addData("Velocity (RPM)", "%.1f", currentRPM);
        telemetry.addData("Velocity (Ticks/Sec)", "%.0f", currentTicksPerSecond);
//        telemetry.addData("Target Velocity", targetVelocityTicks);
//        telemetry.addData("Velocity Error", velocityError);
        telemetry.addData("Turrent Position", robot.spin.getCurrentPosition());

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        robot.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot.lift.setPosition(0.05);

        robot.spin.setTargetPosition(0);
        robot.spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.spin.setPower(1.0);

        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");

        follower = Constants2.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    private Follower follower;

    private int pathState;

    //95.5
    private final Pose startPose = new Pose(67, 8.88, Math.toRadians(90)); // Start Pose of our robot. 68.5  6.5
    private final Pose score = new Pose(67, 16, Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //    private final Pose move = new Pose(111.5, 98.5, Math.toRadians(38)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose score2 = new Pose(65, 18, Math.toRadians(91));

    private final Pose park = new Pose(67, 30, Math.toRadians(90)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup1 = new Pose(19.5, 38, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose gotopickup1 = new Pose(67, 38, Math.toRadians(180));

    private final Pose pickup2 = new Pose(134, 52.5, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose gotopickup2 = new Pose(100, 52.5, Math.toRadians(0));



    private Path scorePreload;
    private PathChain ScorePreLoad, scorePickup1, scorePickup2, goscorePickup2, shake2, end, gograbPickup1, grabPickup1, gograbPickup2, grabPickup2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ScorePreLoad = follower.pathBuilder()
                .addPath(new BezierLine(startPose, score))
                .setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading())
                .build();

        gograbPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(score, gotopickup1))
                .setLinearHeadingInterpolation(score.getHeading(), gotopickup1.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gotopickup1, pickup1))
                .setLinearHeadingInterpolation(gotopickup1.getHeading(), pickup1.getHeading())
                .build();

        scorePickup1 =follower.pathBuilder()
                .addPath(new BezierLine(pickup1, score2))
                .setLinearHeadingInterpolation(pickup1.getHeading(), score2.getHeading())
                .build();

        end = follower.pathBuilder()
                .addPath(new BezierLine(score2, park))
                .setLinearHeadingInterpolation(score2.getHeading(), park.getHeading())
                .build();


//
//        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup1Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup3Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup3Pose, scorePose))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.ge*tHeading())
//                .build();
    }

    public void autonomousPathUpdate() {


        switch (pathState) {
            case 0:
                flywheel.setPower(0.62); //70
                opmodeTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if (opmodeTimer.getElapsedTimeSeconds() >= 1) {
                    robot.ip.setPower(-1);
                    robot.tilt.setPosition(1.0);
                    robot.spin.setTargetPosition(155);
                    follower.followPath(ScorePreLoad);
                    opmodeTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2: //shoot 1
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1) && (velocityError <= tolerance)){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.75 ) {
                    robot.lift.setPosition(0.05);
                    opmodeTimer.resetTimer();//spoon down
                    setPathState(4);
                }
                break;

            case 4: //shoot 2
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.75) && (velocityError <= tolerance) ){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.25 ) {
                    robot.lift.setPosition(0.05);//spoon down
                    opmodeTimer.resetTimer();
                    setPathState(6);
                }
                break;
            case 6: //shoot 3
                if ((opmodeTimer.getElapsedTimeSeconds() >=1.25) && (velocityError <= tolerance) ){
                    robot.lift.setPosition(0.47);
                    opmodeTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7: //go grab 1
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1)) {
                    follower.followPath(gograbPickup1);
                    robot.lift.setPosition(0.05);
                    opmodeTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8: // grabbing
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=0.25)) {
                    follower.followPath(grabPickup1);
                    opmodeTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9: //go to score
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1)){
                    follower.followPath(scorePickup1);
                    opmodeTimer.resetTimer();
                    setPathState(10);
                }
                break;


            case 10: //shoot 1
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.5) && (velocityError <= tolerance)){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.75 ) {
                    robot.lift.setPosition(0.05);
                    opmodeTimer.resetTimer();//spoon down
                    setPathState(12);
                }
                break;

            case 12: //shoot 2
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.75) && (velocityError <= tolerance) ){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.25 ) {
                    robot.lift.setPosition(0.05);//spoon down
                    opmodeTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14: //shoot 3
                if ((opmodeTimer.getElapsedTimeSeconds() >=1.25) && (velocityError <= tolerance) ){
                    robot.lift.setPosition(0.47);
                    opmodeTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15: //shoot 3
                if ((opmodeTimer.getElapsedTimeSeconds() >=1.25)  ){
                    follower.followPath(end);
                    robot.spin.setTargetPosition(0);
                    flywheel.setPower(0);
                    robot.ip.setPower(0);
                    robot.lift.setPosition(0.05);
                    opmodeTimer.resetTimer();
                    setPathState(-1);
                }
                break;




            //                opmodeTimer.resetTimer();
//                while (opmodeTimer.getElapsedTimeSeconds() >= 3) {
//
//
//                    robot.lift.setPosition(0.42);//spoon up
//                    robot.lift.setPosition(0.05);//spoon down
//                }
//                opmodeTimer.resetTimer();
//                while (opmodeTimer.getElapsedTimeSeconds() >= 3) {
//
//
//                    robot.lift.setPosition(0.42);//spoon up
//                    robot.lift.setPosition(0.05);//spoon down
//                }














//            case 1:
//
//            /* You could check for
//            - Follower State: "if(!follower.isBusy()) {}"
//            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//            - Robot Position: "if(follower.getPose().getX() > 36) {}"
//            */
//
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Score Preload */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup1, true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup1, true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup2, true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup2, true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup3, true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if (!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup3, true);
//                    setPathState(7);
//                }
//                break;
//            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if (!follower.isBusy()) {
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//                break;
        }
        Pose finalAutoPose = follower.getPose();;// Save it to our storage class
        PoseStorage.currentPose = finalAutoPose;

        telemetry.addLine("Final Pose Saved!");
        telemetry.addData("Final X (in)", "%.2f", finalAutoPose.getX());
        telemetry.addData("Final Y (in)", "%.2f", finalAutoPose.getY());
        telemetry.addData("Final Heading", "%.2f°", finalAutoPose.getHeading());
        telemetry.update();

    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}