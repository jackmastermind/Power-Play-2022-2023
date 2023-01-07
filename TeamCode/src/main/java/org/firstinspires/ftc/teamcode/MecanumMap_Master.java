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

public class MecanumMap_Master extends HardwareMap_Master
{
    // Mecanum Drive Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor susan;       // Lazy susan wheel
    public DcMotor spool;      // Linear slide motor 1
    public DcMotor arm;      // Linear slide motor 2

    public Servo clawWrist;
    public Servo clawServo;     // Servo to open & close claw

    //OLD DATA
    public static final double WHEEL_CIRCUMFERENCE_INCHES = 4 * Math.PI;
    public static final double TICKS_PER_ROTATION = 560;
    public static final double ROBOT_DIAMETER_INCHES = 24.456;

    /* Initialize standard Hardware interfaces */
    @Override
    public void init(HardwareMap ahwMap, boolean chassisOnly) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Drive Motor Settings
        frontLeft = hwMap.get(DcMotor.class, "driveFL");
        frontRight = hwMap.get(DcMotor.class, "driveFR");
        backLeft = hwMap.get(DcMotor.class, "driveBL");
        backRight = hwMap.get(DcMotor.class, "driveBR");

        if (!chassisOnly) {
            susan = hwMap.get(DcMotor.class, "susan");
            spool = hwMap.get(DcMotor.class, "spool");
            arm = hwMap.get(DcMotor.class, "arm");

            clawWrist = hwMap.get(Servo.class, "clawWrist");
            clawServo = hwMap.get(Servo.class, "clawServo");

        }

        if (chassisOnly)
        {
            motors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight};
        }
        else {
            motors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight,
                    susan, spool, arm};
            servos = new Servo[] {clawWrist, clawServo};
        }

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Hardware Initialization
        for (DcMotor m: motors) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
 }

