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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Master Autonomous [FOR COMPETITION]")
@SuppressWarnings("FieldCanBeLocal")
public class MasterAutonomous extends LinearOpMode {

    private final MecanumMap_Master mecanum = new MecanumMap_Master();
    private final ElapsedTime runtime = new ElapsedTime();

    private SlideController slide;
    private ClawController clawController;
    private SusanController susanController;
    private CameraController cameraController;

    private DcMotor[] motors;
    private Servo[] servos;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);

        mecanum.init(hardwareMap);
        slide           = new SlideController(hardwareMap);
        clawController  = new ClawController(hardwareMap);
        susanController = new SusanController(hardwareMap);
        cameraController = new CameraController(hardwareMap);

        motors = new DcMotor[] {mecanum.frontLeft, mecanum.frontRight,
                                mecanum.backLeft, mecanum.backRight,
                                slide.spoolMotor, susanController.susan};
        servos = new Servo[]   {clawController.claw, clawController.wrist};

        telemetry.addData("Status", "loading recordings...");

        MotorPlayback playback1 = new MotorPlayback("recording1.json", runtime, hardwareMap,
                                                    motors, servos, telemetry);
        MotorPlayback playback2 = new MotorPlayback("recording2.json", runtime, hardwareMap,
                                                    motors, servos, telemetry);
        MotorPlayback playback3 = new MotorPlayback("recording3.json", runtime, hardwareMap,
                                                    motors, servos, telemetry);

        telemetry.addData("Status", "initialized");
        telemetry.update();

        //RUNNING
        waitForStart();
        runtime.reset();
        cameraController.StartQRDetectThread(this);

        while (cameraController.qr == 0)
        {
            Thread.sleep(20);
        }

        if (cameraController.qr == 1)
        {
            playback1.playAll(this, false, false);
        }
        else if (cameraController.qr == 2)
        {
            playback2.playAll(this, false, false);
        }
        else
        {
            playback3.playAll(this, false, false);
        }
    }
}

