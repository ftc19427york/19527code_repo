package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Pinpoint Live Tracker")
public class PinpointTracker extends LinearOpMode {

    LLHardware robot = new LLHardware ();

    @Override
    public void runOpMode() {
        // Initialize the hardware

        // Reset at start so current position is (0,0,0)
        robot.init(hardwareMap);
        Follower follower = Constants2.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8.125, 8.875, 0));

        telemetry.addLine("Ready! Press Play to see coordinates.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // IMPORTANT: You must call update() to get new data from the sensors
            follower.update();
            Pose currentPose = follower.getPose();
            double robotHeading = Math.toDegrees(follower.getPose().getHeading());

            // Display the data
            telemetry.addLine("--- LIVE COORDINATES ---");
            telemetry.addData("X (Inches)", "%.2f in", follower.getPose().getX());
            telemetry.addData("Y (Inches)", "%.2f in", follower.getPose().getY());
            telemetry.addData("Heading (Deg)", "%.2f°", robotHeading);

            telemetry.update();
        }
    }
}
