package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PIDF Velocity Control")
public class PIDFVelocityControl extends LinearOpMode {

    LLHardware robot = new LLHardware();

    // PIDF coefficients
    private double kP = 50.0;  // Proportional gain (velocity adjustment per inch of error)
    private double kI = 0.0;   // Integral gain
    private double kD = 5.0;   // Derivative gain

    // Goal position
    private double goalX = 133;  // Goal X position in inches
    private double goalY = 133;  // Goal Y position in inches

    // PIDF variables
    private double integral = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Motor control
    private DcMotorEx controlMotor;  // The motor you want to control

    // Velocity limits
    private double maxVelocity = 3500;  // Max motor velocity in ticks per second
    private double minVelocity = 1000;   // Min motor velocity (motor always runs at least this fast)
    private double baseVelocity = 3471; // Base velocity when at target distance

    // Distance threshold
    private double targetDistance = 156;  // Target distance from goal in inches
    private double tolerance = 2.0;        // Acceptable error in inches

    @Override
    public void runOpMode() {
        // Initialize hardware
        robot.init(hardwareMap);
        Follower follower = Constants.createFollower(hardwareMap);

        // Initialize the motor you want to control (change "motorName" to your motor's name)
        controlMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        controlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set starting pose
        follower.setStartingPose(new Pose(120.78, 121.70, 38));

        telemetry.addLine("PIDF Velocity Control Ready!");
        telemetry.addData("Goal Position", "X: %.2f, Y: %.2f", goalX, goalY);
        telemetry.addData("Target Distance", "%.2f in", targetDistance);
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // Update odometry
            follower.update();
            Pose currentPose = follower.getPose();

            // Calculate distance to goal
            double distanceToGoal = calculateDistance(
                    currentPose.getX(),
                    currentPose.getY(),
                    goalX,
                    goalY
            );

            // Calculate error (difference from target distance)
            // Positive error = too far from goal (need to speed up)
            // Negative error = too close to goal (need to slow down)
            double error = distanceToGoal - targetDistance;

            // Calculate PIDF velocity adjustment
            double velocityAdjustment = calculatePIDF(error);

            // Apply adjustment to base velocity
            double velocity = baseVelocity + velocityAdjustment;

            // Clamp velocity to limits (motor always runs between min and max)
            velocity = Math.max(minVelocity, Math.min(maxVelocity, velocity));

            // Set motor velocity
            controlMotor.setVelocity(velocity);

            // Display telemetry
            telemetry.addLine("--- PIDF VELOCITY CONTROL ---");
            telemetry.addData("Current Position", "X: %.2f, Y: %.2f",
                    currentPose.getX(), currentPose.getY());
            telemetry.addData("Distance to Goal", "%.2f in", distanceToGoal);
            telemetry.addData("Error", "%.2f in", error);
            telemetry.addLine();
            telemetry.addData("Base Velocity", "%.0f ticks/sec", baseVelocity);
            telemetry.addData("Velocity Adjustment", "%+.0f ticks/sec", velocityAdjustment);
            telemetry.addData("Final Motor Velocity", "%.0f ticks/sec", velocity);
            telemetry.addData("At Target Distance", Math.abs(error) < tolerance);
            telemetry.addLine();
            telemetry.addData("P Term", "%.2f", kP * error);
            telemetry.addData("I Term", "%.2f", kI * integral);
            telemetry.addData("D Term", "%.2f", kD * (error - lastError) / timer.seconds());
            telemetry.update();
        }

        // Stop motor when op mode ends
        controlMotor.setVelocity(0);
    }

    /**
     * Calculate PIDF output based on error
     * Returns velocity adjustment from base velocity
     */
    private double calculatePIDF(double error) {
        double deltaTime = timer.seconds();
        timer.reset();

        // Proportional term
        double pTerm = kP * error;

        // Integral term (accumulated error over time)
        integral += error * deltaTime;
        // Anti-windup: clamp integral to prevent runaway
        integral = Math.max(-100, Math.min(100, integral));
        double iTerm = kI * integral;

        // Derivative term (rate of change of error)
        double derivative = (error - lastError) / deltaTime;
        double dTerm = kD * derivative;

        // Update last error
        lastError = error;

        // Return velocity adjustment (positive = speed up, negative = slow down)
        return pTerm + iTerm + dTerm;
    }

    /**
     * Calculate Euclidean distance between two points
     */
    private double calculateDistance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
