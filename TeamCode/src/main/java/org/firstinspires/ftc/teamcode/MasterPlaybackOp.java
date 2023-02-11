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

@Autonomous(name="Master Playback Op [FOR TESTING ONLY]")
@SuppressWarnings("FieldCanBeLocal")
public class MasterPlaybackOp extends LinearOpMode {

    private final MecanumMap_Master mecanum = new MecanumMap_Master();
    private final ElapsedTime runtime = new ElapsedTime();

    private SlideController slide;
    private ClawController clawController;
    private SusanController susanController;

    private DcMotor[] motors;
    private Servo[] servos;

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);

        mecanum.init(hardwareMap);
        slide           = new SlideController(hardwareMap);
        clawController  = new ClawController(hardwareMap);
        susanController = new SusanController(hardwareMap);

        motors = new DcMotor[] {mecanum.frontLeft, mecanum.frontRight,
                                mecanum.backLeft, mecanum.backRight,
                                slide.spoolMotor, susanController.susan};
        servos = new Servo[]   {clawController.claw, clawController.wrist};

        telemetry.addData("Status", "waiting for recording select...");
        telemetry.update();

        String filepath;
        while (true)
        {
            filepath = mecanum.filenameSelect(this);
            if (filepath != null) {
                telemetry.addData("Status", filepath + " selected");
                break;
            }
            try {
                //noinspection BusyWait
                Thread.sleep(10);
            } catch (InterruptedException e) {
                telemetry.addData("selection error", e.getMessage());
            }
        }
        telemetry.update();

        telemetry.addData("Status", "loading...");
        telemetry.update();

        MotorPlayback playback = new MotorPlayback(filepath, runtime, hardwareMap,
                                                   motors, servos, telemetry);

        telemetry.addData("Status", "all set!");
        telemetry.update();

        waitForStart();
        runtime.reset();
        playback.playAll(this, false, false);
    }
}

