package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;

import java.util.List;


@Autonomous(name = "Rotate With April Tag", group = "Examples")

public class AprilTagFollower extends OpMode {


    private Path scorePreload;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private Limelight3A limelight3A;
    LLHardware robot = new LLHardware ();
    private PathChain turn;
    private double distanceRaw;
    private Pose Pose1 = new Pose(0, 0, Math.toRadians(0));
    private Pose Pose2 = new Pose(0,0, Math.toRadians(0));
    private double tx;


    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants2.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        robot.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

    }


    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        LLResult llResult = limelight3A.getLatestResult();
        distanceRaw = getDistanceFromTag(llResult.getTa());

        double limelightMountAngleDegrees = 0;    // how many degrees back is your limelight rotated from perfectly vertical?
        double targetOffsetAngle_Vertical = llResult.getTy();
        double limelightLensHeightInches = 0;     // distance from the CENTER of the Limelight LENS to the floor
        double goalHeightInches = 0;              // distance from the target to the floor
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double DistanceRobotToGoal = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Target x", llResult.getTx());
        telemetry.addData("Target y", llResult.getTy());
        telemetry.addData("Distance Raw", distanceRaw);
        telemetry.addData("Distance Robot To Goal", DistanceRobotToGoal);


        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducialResults) {

            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

        telemetry.update();

        //Pose2 = Pose2.setHeading(/*updated heading in radians as double */0.0);
    }

    public void buildPaths() {

        turn = follower.pathBuilder()
                .addPath(new BezierLine(Pose1, Pose2))
                .setLinearHeadingInterpolation(Pose1.getHeading(), Pose2.getHeading())

            .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //loop
                LLResult llResult = limelight3A.getLatestResult();


                setPathState(0);
                break;
        }
    }

    public double getDistanceFromTag(double Ta){
        double scale = 32571.17;
        double distance = Math.sqrt(scale/(Ta));
        return distance;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

}
