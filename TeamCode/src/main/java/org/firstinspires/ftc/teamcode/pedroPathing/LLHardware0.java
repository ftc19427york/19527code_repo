package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LLHardware0 {

    /* Copyright (c) 2017 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
     * promote products derived from this software without specific prior written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */




    Limelight3A limelight;
    public IMU imu;
    public DcMotor lr;
    public DcMotor rf;
    public DcMotor rr;
    public DcMotor lf;


    public Servo Torrent ;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        // Save reference to Hardware map
        lf = hwMap.get(DcMotor.class, "lf");
        lr = hwMap.get(DcMotor.class, "lr");
        rf = hwMap.get(DcMotor.class, "rf");
        rr = hwMap.get(DcMotor.class, "rr");

        lf.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);





//        limelight = hwMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100);
        imu= hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));


    }
    //Functions to set motor starting positions

    public void setAllMotorPower(Double power){
        lf.setPower(-power);
        lr.setPower(-power);
        rf.setPower(power);
        rr.setPower(power);
    }

    public YawPitchRollAngles getOrientation() {
        // Check if the IMU was successfully initialized.
        if (imu != null) {
            // Return the current orientation angles from the IMU.
            // The specific AxesOrder might need adjustment depending on how your IMU is mounted.
            return imu.getRobotYawPitchRollAngles();
        }
        // Return null if the IMU is not initialized (which is what your code was doing).
        return null;
    }
}


