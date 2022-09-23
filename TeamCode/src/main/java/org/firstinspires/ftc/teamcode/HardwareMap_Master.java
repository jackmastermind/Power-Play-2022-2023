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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
    MASTER HARDWARE MAP

    This is not an Op Mode. This file contains all the robots hardware,
    properties, and initialization settings. This file can be imported into
    all other class files, to automatically get access to the hardware map below

    HARDWARE MAP
 */

public class HardwareMap_Master
{
    // Mecanum Drive Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor collector;   // Gecko wheel rolling in freight
    public DcMotor spool;       // Spool controlling the height of the x-rail
    public DcMotor arm;         // Control the pivoting arm of the box
    public DcMotor carousel;    // The motor that spins the duck carousel

    public DcMotor[] motors;
    public Servo[] servos;

    public static final double WHEEL_CIRCUMFERENCE_INCHES = 9.276;
    public static final double TICKS_PER_ROTATION = 560;
    public static final double ROBOT_DIAMETER_INCHES = 24.456;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap)
    {
        init(ahwMap, false);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean chassisOnly) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Drive Motor Settings
        frontLeft = hwMap.get(DcMotor.class, "driveFL");
        frontRight = hwMap.get(DcMotor.class, "driveFR");
        backLeft = hwMap.get(DcMotor.class, "driveBL");
        backRight = hwMap.get(DcMotor.class, "driveBR");

        if (!chassisOnly) {
            collector = hwMap.get(DcMotor.class, "collector");
            spool = hwMap.get(DcMotor.class, "spool");
            arm = hwMap.get(DcMotor.class, "arm");
            carousel = hwMap.get(DcMotor.class, "carousel");
        }

        if (chassisOnly)
        {
            motors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight};
        }
        else {
            motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight, collector, spool, arm, carousel};
            servos = new Servo[]{};
        }

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        if (!chassisOnly) {
            carousel.setDirection(DcMotorSimple.Direction.REVERSE);
            collector.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Hardware Initialization
        for (DcMotor m: motors) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    
    public String filenameSelect(LinearOpMode currentOpMode)
    {
        Gamepad gamepad1 = currentOpMode.gamepad1;
        if (gamepad1.a)
        {
            return "recordingA.json";
        }
        else if (gamepad1.b)
        {
            return "recordingB.json";
        }
        else if (gamepad1.x)
        {
            return "recordingX.json";
        }
        else if (gamepad1.y)
        {
            return "recordingY.json";
        }
        else if (gamepad1.dpad_up)
        {
            return "recordingUP.json";
        }
        else if (gamepad1.dpad_left)
        {
            return "recordingLEFT.json";
        }
        else if (gamepad1.dpad_down)
        {
            return "recordingDOWN.json";
        }
        else if (gamepad1.dpad_right)
        {
            return "recordingRIGHT.json";
        }
        return null;
    }
 }

