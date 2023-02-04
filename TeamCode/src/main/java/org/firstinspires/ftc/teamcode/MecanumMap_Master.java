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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public DcMotor[] driveMotors;

    public DcMotor susan;       // Lazy susan wheel
    public Servo clawWrist;
    public Servo clawServo;     // Servo to open & close claw


    public static final double WHEEL_CIRCUMFERENCE_INCHES = 4 * Math.PI;
    public static final double TICKS_PER_ROTATION = 537.7;
    public static final double ROBOT_DIAMETER_INCHES = 21.375;

    public static final double TILE_WIDTH = 23.5;
    public static final double AUTO_DRIVE_SPEED = 0.5;

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

        driveMotors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight};

        if (!chassisOnly) {
            susan = hwMap.get(DcMotor.class, "susan");
            clawWrist = hwMap.get(Servo.class, "clawWrist");
            clawServo = hwMap.get(Servo.class, "clawServo");
        }

        if (chassisOnly)
        {
            motors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight};
        }
        else {
            motors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight,
                    susan};
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
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void moveEncoder(double inches, double power) {
        //Gets the proper target position with some math & robot specs inherited from hwmap master.
        int target = (int) Math.round((inches / WHEEL_CIRCUMFERENCE_INCHES) * TICKS_PER_ROTATION);

        //Resets encoder ticks, sets the position, and gets the robot moving to the position.
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTargetPosition(target);
        setDrivePower(power);
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait for the robot to stop moving
        while (frontLeft.isBusy()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                break;
            }
        }

        //Stop the robot, reset encoder ticks, change run mode back to normal.
        setDrivePower(0);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveTiles(double tiles, double power)
    {
        moveEncoder(tiles * TILE_WIDTH, power);
    }

    public void moveTiles(double tiles)
    {
        if (tiles < 0)
        {
            moveTiles(tiles, -AUTO_DRIVE_SPEED);

        }
        else
        {
            moveTiles(tiles, AUTO_DRIVE_SPEED);
        }
    }

    public void turnEncoder(double degrees, double power) {
        int target = (int) Math.round((Math.PI * ROBOT_DIAMETER_INCHES / 360)   //Inches per degree
                * degrees                                 //times degrees
                * (1 / WHEEL_CIRCUMFERENCE_INCHES)        //times rotations per inch
                * TICKS_PER_ROTATION);                    //times ticks per rotation
        //equals ticks.
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        frontRight.setTargetPosition(-target);
        backRight.setTargetPosition(-target);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Wait for the robot to stop moving
        while (frontLeft.isBusy()) {
            try {
                Thread.sleep(100);
            }
            catch (InterruptedException e) {
                break;
            }
        }

        //Stop the robot, reset encoder ticks, change run mode back to normal.
        setDrivePower(0);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setDrivePower(double power) {
        for (DcMotor motor: driveMotors) {
            motor.setPower(power);
        }
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        for (DcMotor motor: motors) {
            motor.setMode(mode);
        }
    }

    public void setDriveTargetPosition(int position) {
        for (DcMotor motor: motors) {
            motor.setTargetPosition(position);
        }
    }

    public void drive(Gamepad gamepad, double speed, boolean useCubicControls)
    {
        double inputLX = gamepad.left_stick_x;
        double inputLY = gamepad.left_stick_y;
        double inputRX = gamepad.right_stick_x;

        double normalization = Math.max(Math.abs(inputLX) + Math.abs(inputLY) + Math.abs(inputRX),
                                        1.0);

        double flPower = ((-inputLX + inputLY) - inputRX)/ normalization;
        double blPower = ((inputLX + inputLY) - inputRX) / normalization;

        double frPower = ((inputLX + inputLY) + inputRX) / normalization;
        double brPower = ((-inputLX + inputLY) + inputRX)/ normalization;

        if (useCubicControls)
        {
            flPower = Math.pow(flPower, 3);
            frPower = Math.pow(frPower, 3);
            blPower = Math.pow(blPower, 3);
            brPower = Math.pow(brPower, 3);
        }

        flPower *= speed;
        frPower *= speed;
        blPower *= speed;
        brPower *= speed;

        frontLeft.setPower(flPower);
        frontRight.setPower(frPower);
        backLeft.setPower(blPower);
        backRight.setPower(brPower);
    }

    public void drive(Gamepad gamepad, double speed)
    {
        drive(gamepad, speed, true);
    }

    public void drive(Gamepad gamepad, boolean useCubicControls)
    {
        drive(gamepad, 1, useCubicControls);
    }

    public void drive(Gamepad gamepad)
    {
        drive(gamepad, 1);
    }

    public void LogValues(Telemetry telemetry)
    {
        telemetry.addData("fl", frontLeft.getPower());
        telemetry.addData("fr", frontRight.getPower());
        telemetry.addData("bl", backLeft.getPower());
        telemetry.addData("br", backRight.getPower());
    }
 }

