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

package org.firstinspires.ftc.teamcode.pedroPathing; /**This is the folder path of where this file is located. */

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

//import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Locale;

public class HardwareMecanum {

    //Giving names to each device
    public DcMotor lift = null;
    public DcMotor tilt = null;
    public Servo panClaw = null;
    public Servo claw = null;
    public Servo twistClaw = null;
    public AnalogInput encoder;
    Limelight3A limelight;

    //  public DistanceSensor rangeFinder = null;
    //  public TouchSensor toucher = null;
    //  public RevColorSensorV3 colorSensor = null;

   //Defining the servo starting positions
    public final static double panClawARM_HOME = 0.15; // Starting point for Servo Arm 0.75
    public final static double clawARM_HOME = 0.39; // Starting point for Servo Arm
    public final static double twistClawARM_HOME = 0.75; // 0.6 Starting point for Servo Arm **twisting of the entire claw assembly
    double panPosition = panClawARM_HOME;  // servo's position
    final double panARM_SPEED = 0.10;  // set rate to move servo
    double twistPosition = twistClawARM_HOME;  // servo's position
    final double twistARM_SPEED = 0.005;  // set rate to move servo

    double clawPosition = clawARM_HOME;  // servo's position
    final double clawARM_SPEED = 0.005;  // set rate to move servo

    //HardwareMap hwMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {

        // Save reference to Hardware map
        lift = hwMap.get(DcMotor.class, "motor4");
        tilt = hwMap.get(DcMotor.class, "motor5");
        encoder = hwMap.get(AnalogInput.class, "encoder");
        twistClaw = hwMap.servo.get("servo"); // set equal to name of the servo motor in DS
        claw = hwMap.servo.get("servo1");
        panClaw = hwMap.servo.get("servo2");
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        lift.setDirection(DcMotor.Direction.FORWARD);// motor4
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tilt.setDirection(DcMotor.Direction.FORWARD);// motor5
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Initialize motor starting positions
       // setLiftPosition(100, 1); //Function is below to set starting position of motor
      //  setTiltPosition(100, 1);

        //Initialize servo starting positions
        panClaw.setPosition(panClawARM_HOME); //setPosition sets the servo's position and moves it.
        claw.setPosition(clawARM_HOME); //setPosition are defined above.
        twistClaw.setPosition(twistClawARM_HOME);
    }
    //Functions to set motor starting positions
    public void setLiftPosition(int position, double power) {
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setTargetPosition(position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(power);
    }
    public void setTiltPosition(int position, double power) {
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setTargetPosition(position);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tilt.setPower(power);
    }


    public YawPitchRollAngles getOrientation() {
        return null;
    }
}


