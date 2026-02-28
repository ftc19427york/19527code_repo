/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "LimeLight1", group = "Sensor")
public class LLDistanceData extends OpMode {
    private Limelight3A limelight3A;
    LLHardware robot = new LLHardware (); //reference the hardware file name

    private double distance;

    @Override
    public void init() {
        robot.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

    }


    @Override
    public void loop() {

        YawPitchRollAngles orientation = robot.getOrientation();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {

            // Access general information
            Pose3D botpose = llResult.getBotpose_MT2();

            double captureLatency = llResult.getCaptureLatency();
            double targetingLatency = llResult.getTargetingLatency();
            double parseLatency = llResult.getParseLatency();

            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target y", llResult.getTy());
            telemetry.addData("txnc", llResult.getTxNC());
            telemetry.addData("tync", llResult.getTyNC());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botpose.toString());
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                telemetry.update();
            }
        }
    }
    public double getDistanceFromTag(double Ta){
        double scale = 32571.17;
        double distance = Math.sqrt(scale/(Ta));
        return distance;
    }


}