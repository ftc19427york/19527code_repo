package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Limelight Testing", group = "Sensor")

public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight3A;

    LLHardware robot = new LLHardware();
    private double distance;
    private IMU imu;



    @Override
    public void init() {
        robot.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class,  "limelight");
        limelight3A.pipelineSwitch(0);
        imu= hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    @Override
    public void start(){
        limelight3A.start();
    }
    @Override
    public void loop() {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight3A.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botpose  =   llResult.getBotpose_MT2();  //llResult.getBotpose_MT2()
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("BotPose", botpose.toString());
            telemetry.addData("Yaw", botpose.getOrientation().getYaw());
        }

    }
}
