package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "12 Red Close", group = "Red Close")
public class Auto12Red extends OpMode {
    LLHardware robot = new LLHardware ();
    private Timer pathTimer, opmodeTimer;

    private ElapsedTime runtime = new ElapsedTime();
    private boolean moving = false;
    boolean isTimerActive = false;
    double timerUpperValue = 1; // amount of time timer is active for
    double delta = 0.01;        // amount of time passed between loop calls (ideally)
    double timerValue = 0.0;

    boolean isWaitActive = false;
    double waitUpper = 1; // amount of time timer is active for
    double beta = 0.01;        // amount of time passed between loop calls (ideally)
    double wait = 0.0;


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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

        follower = Constants2.createFollower(hardwareMap);
        buildPaths();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        follower.setStartingPose(startPose);
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    private Follower follower;

    private int pathState;

    private final Pose startPose = new Pose(125, 115, Math.toRadians(38)); //888125 115888 123/115.5Start Pose of our robot. 68.5  6.5
    private final Pose score = new Pose(100.5, 83.5, Math.toRadians(50)); // y93 Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //    private final Pose move = new Pose(111.5, 98.5, Math.toRadians(38)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose park = new Pose(94, 106, Math.toRadians(38)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup1 = new Pose(128, 77, Math.toRadians(0)); // 76.5Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose gotopickup1 = new Pose(100, 77, Math.toRadians(0));

    private final Pose pickup2 = new Pose(136, 52, Math.toRadians(0)); // 59.5Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose gotopickup2 = new Pose(100, 52, Math.toRadians(0));
    private final Pose pickup3 = new Pose(136, 29, Math.toRadians(0)); // 59.5Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose gotopickup3 = new Pose(100, 29, Math.toRadians(0));


    private Path scorePreload;
    private PathChain backup, scorePickup1, scorePickup2, scorePickup3, goscorePickup2, shake2, end, gograbPickup1, grabPickup1, gograbPickup2, grabPickup2, gograbPickup3, grabPickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        backup = follower.pathBuilder()
                .addPath(new BezierLine(startPose, score))
                .setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, score))
                .setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(gotopickup2, score))
                .setLinearHeadingInterpolation(gotopickup2.getHeading(), score.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(gotopickup3, score))
                .setLinearHeadingInterpolation(gotopickup3.getHeading(), score.getHeading())
                .build();
        goscorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, gotopickup2))
                .setLinearHeadingInterpolation(pickup2.getHeading(), gotopickup2.getHeading())
                .build();


        gograbPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(score, gotopickup1))
                .setLinearHeadingInterpolation(score.getHeading(), gotopickup1.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(gotopickup1, pickup1))
                .setLinearHeadingInterpolation(gotopickup1.getHeading(), pickup1.getHeading())
                .build();

        gograbPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(score, gotopickup2))
                .setLinearHeadingInterpolation(score.getHeading(), gotopickup2.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(gotopickup2, pickup2))
                .setLinearHeadingInterpolation(gotopickup2.getHeading(), pickup2.getHeading())
                .build();

        gograbPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(score, gotopickup3))
                .setLinearHeadingInterpolation(score.getHeading(), gotopickup3.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(gotopickup3, pickup3))
                .setLinearHeadingInterpolation(gotopickup3.getHeading(), pickup3.getHeading())
                .build();
        end = follower.pathBuilder()
                .addPath(new BezierLine(score, park))
                .setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
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
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.ip.setPower(-1);
                robot.op.setPower(0.35);
                robot.tilt.setPosition(0.86);
                follower.followPath(backup);
                opmodeTimer.resetTimer();
                setPathState(1);
                break;

            case 1: //shoot 1
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.0 )){
                    robot.lift.setPosition(0.42);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(2);
                }
                break;
            case 2:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.5 ) {
                    robot.op.setPower(0.37);
                    robot.lift.setPosition(0.05);
                    opmodeTimer.resetTimer();//spoon down
                    setPathState(3);
                }
                break;

            case 3: //shoot 2
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.0 )){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(4);
                }
                break;

            case 4:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.5 ) {
//                    follower.followPath(shake);
//                    follower.followPath(shake2);
                    robot.op.setPower(0.45);
                    robot.lift.setPosition(0.05);//spoon down
                    opmodeTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5: //shoot 3
                if (opmodeTimer.getElapsedTimeSeconds() >=1){
                    robot.lift.setPosition(0.47);
                    opmodeTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=0.5)) {
                    robot.lift.setPosition(0.05);
                    opmodeTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7: //grab
                if (opmodeTimer.getElapsedTimeSeconds() >=0.5) {
                    //robot.lift.setPosition(0.05);
                    follower.followPath(gograbPickup1);
                    robot.op.setPower(0.47);
                    setPathState(8);
                }
                break;

            case 8:

                if ((!follower.isBusy())) {
                   // robot.lift.setPosition(0.5);
                    follower.followPath(grabPickup1);
                    setPathState(9);
                }
                break;

            case 9:

                if ((!follower.isBusy())) {
                  //  robot.lift.setPosition(0.05);
                    follower.followPath(scorePickup1);
                    opmodeTimer.resetTimer();
//                    follower.followPath(end);
                    setPathState(10);
                }
                break;//
//COPY
            case 10: //shoot 1
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.1 )){
                    robot.lift.setPosition(0.47);//spoon up
                    robot.op.setPower(0.45);
                    opmodeTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.5 ) {
                    robot.lift.setPosition(0.05);
                    opmodeTimer.resetTimer();//spoon down
                    setPathState(12);
                }
                break;

            case 12: //shoot 2
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.0 )){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(13);
                }
                break;

            case 13:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.5 ) {
//                    follower.followPath(shake);
//                    follower.followPath(shake2);
                    robot.lift.setPosition(0.05);//spoon down
                    robot.op.setPower(0.47);
                    opmodeTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14: //shoot 3
                if (opmodeTimer.getElapsedTimeSeconds() >=1){
                    robot.lift.setPosition(0.47);
                    opmodeTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=0.5)) {
                    robot.lift.setPosition(0.05);

                    opmodeTimer.resetTimer();
                    setPathState(16);
                }
                break;

            case 16:
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=0.5)) {
                    follower.followPath(gograbPickup2);
                    opmodeTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                if ((!follower.isBusy())) {
                    follower.followPath(grabPickup2);
                    opmodeTimer.resetTimer();
                    setPathState(19);
                }
                break;

//            case 18:
//                if ((!follower.isBusy())) {
//                    follower.followPath(goscorePickup2);
//                    opmodeTimer.resetTimer();
//                    setPathState(19);
//                }
//                break;

            case 19:
                if ((!follower.isBusy())) {
                    follower.followPath(scorePickup2);
                    opmodeTimer.resetTimer();
                    setPathState(20);
                }
                break;

            case 20: //shoot 1
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.1)){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(21);
                }
                break;
            case 21:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.5 ) {
                    robot.op.setPower(0.42);
                    robot.lift.setPosition(0.05);
                    opmodeTimer.resetTimer();//spoon down
                    setPathState(22);
                }
                break;

            case 22: //shoot 2
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.0 )){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(23);
                }
                break;

            case 23:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.5 ) {
//                    follower.followPath(shake);
//                    follower.followPath(shake2);
                    robot.lift.setPosition(0.05);//spoon down
                    opmodeTimer.resetTimer();
                    setPathState(24);
                }
                break;
            case 24: //shoot 3
                if (opmodeTimer.getElapsedTimeSeconds() >=1){
                    robot.lift.setPosition(0.47);
                    opmodeTimer.resetTimer();
                    setPathState(25);
                }
                break;

            case 25:
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=0.5)) {
                    robot.lift.setPosition(0.05);

                    opmodeTimer.resetTimer();
                    setPathState(26);
                }
                break;
            case 26:
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=0.5)) {
                    follower.followPath(gograbPickup3);
                    opmodeTimer.resetTimer();
                    setPathState(27);
                }
                break;

            case 27:
                if ((!follower.isBusy())) {
                    follower.followPath(grabPickup3);
                    opmodeTimer.resetTimer();
                    setPathState(28);
                }
                break;

            case 28:
                if ((!follower.isBusy())) {
                    follower.followPath(scorePickup3);
                    opmodeTimer.resetTimer();
                    setPathState(30);
                }
                break;


            case 30: //shoot 1
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.0 )){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(31);
                }
                break;
            case 31:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.5 ) {
                    robot.op.setPower(0.44);
                    robot.lift.setPosition(0.05);
                    opmodeTimer.resetTimer();//spoon down
                    setPathState(32);
                }
                break;

            case 32: //shoot 2
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=1.0 )){
                    robot.lift.setPosition(0.47);//spoon up
                    opmodeTimer.resetTimer();
                    setPathState(33);
                }
                break;

            case 33:
                if (opmodeTimer.getElapsedTimeSeconds() >=0.5 ) {
//                    follower.followPath(shake);
//                    follower.followPath(shake2);
                    robot.lift.setPosition(0.05);//spoon down
                    opmodeTimer.resetTimer();
                    setPathState(34);
                }
                break;
            case 34: //shoot 3
                if (opmodeTimer.getElapsedTimeSeconds() >=1){
                    robot.lift.setPosition(0.47);
                    opmodeTimer.resetTimer();
                    setPathState(36);
                }
                break;

//            case 35:
//                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=0.25)) {
//                    robot.lift.setPosition(0.05);
//
//                    opmodeTimer.resetTimer();
//                    setPathState(36);
//                }
//                break;
// CASE 26 JUST ADDED
            case 36:
                if ((!follower.isBusy()) && (opmodeTimer.getElapsedTimeSeconds() >=0.25)) {
                    robot.ip.setPower(0);
                    robot.op.setPower(0);
                    robot.lift.setPosition(0.05);
                    follower.followPath(end);
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