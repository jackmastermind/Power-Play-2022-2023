package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Master Drive")
public class MasterDrive extends LinearOpMode
{
    private final MecanumMap_Master master = new MecanumMap_Master();
    private final SlideController slide = new SlideController();
    //Todo: Make Claw Controller Class
    //Todo: Make Suzan Controller Class


    //MOTOR VARIABLES
    double armSpeed = 1;
    double wristSpeed = -0.003;
    double clawSpeed = 1;
    double susanSpeed = 1;
    double wristTarget = 0.9;
    double armTarget = 0.0;

    boolean clawOpen = true;
    boolean aDown = false;


    private final ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {

        MotorRecorder recorder = new MotorRecorder(runtime, master, 0.01, telemetry);

        //INITIALIZE OBJECTS
        master.init(hardwareMap);
        slide.init(hardwareMap);

        slide.ignoreMinMax = true; //Todo: Remove this once min max are deterined

        waitForStart();
        telemetry.setAutoClear(true);
        runtime.reset();

        double lastRuntime = runtime.time();

        while(opModeIsActive())
        {
            double deltaTime = runtime.time() - lastRuntime;

            //DRIVE SECTION
            double powerMultiplier = 0.5;

            double inputLX = gamepad1.left_stick_x;
            double inputLY = gamepad1.left_stick_y;
            double inputRX = gamepad1.right_stick_x;

            double normalization = Math.max(Math.abs(inputLX) + Math.abs(inputLY) + Math.abs(inputRX), 1.0);

            double flPower = Math.pow(((-inputLX + inputLY) - inputRX)/ normalization, 3) * powerMultiplier;
            double blPower = Math.pow(((inputLX + inputLY) - inputRX)/ normalization, 3) * powerMultiplier;

            double frPower = Math.pow(((inputLX + inputLY) + inputRX)/ normalization, 3) * powerMultiplier;
            double brPower = Math.pow(((-inputLX + inputLY) + inputRX)/ normalization, 3) * powerMultiplier;

            //RECORDING DUMP
            if (gamepad1.right_trigger >= 0.75)
            {
                telemetry.addData("DUMPING MODE", "active");
                String filepath = master.filenameSelect(this);
                if (filepath != null)
                {
                    telemetry.addData("DUMPING MODE", "dumping...");
                    recorder.dumpData(filepath);
                    telemetry.addData("DUMPING MODE", "complete!");
                }
            }

            //OPERATING SECTION

            //Arm
            double armInput = gamepad2.left_stick_y;
            slide.MoveSlide(armInput, armSpeed);

            //Wrist
            double wristInput = gamepad2.right_stick_y;
            wristTarget += wristSpeed * wristInput;
            wristTarget = Math.min(1, Math.max(0, wristTarget));

            //Claw
            if (gamepad2.a && !aDown)
            {
                aDown = true;
                if (clawOpen)
                {
                    master.clawServo.setPosition(0.25);
                    clawOpen = false;
                }
                else
                {
                    master.clawServo.setPosition(0);
                    clawOpen = true;
                }
            }
            else if (!gamepad2.a){
                aDown = false;
            }

            //Susan
            double susanInput = gamepad2.right_trigger - gamepad2.left_trigger;
            double susanPower = Math.pow(susanInput * susanSpeed, 3);


            //SET POWER
            master.frontLeft.setPower(flPower);
            master.frontRight.setPower(frPower);
            master.backLeft.setPower(blPower);
            master.backRight.setPower(brPower);

            master.susan.setPower(susanPower);

            master.clawWrist.setPosition(wristTarget);


            //TELEMETRY
            telemetry.addLine("DRIVE MOTORS");
            telemetry.addData("fl", flPower);
            telemetry.addData("fr", frPower);
            telemetry.addData("bl", blPower);
            telemetry.addData("br", brPower);
            telemetry.addLine("OPERATOR MOTORS");
            telemetry.addData("susan", susanPower);
            telemetry.addData("armTarget", armTarget);
            //telemetry.addData("shoulder", arm.motorJoint1.getPower());
            //telemetry.addData("elbow", arm.motorJoint2.getPower());
            telemetry.addLine();
            telemetry.addData("claw position", master.clawServo.getPosition());
            telemetry.addData("wristTarget", wristTarget);
            telemetry.addData("wrist position", master.clawWrist.getPosition());
            telemetry.addLine();
            telemetry.addData("[DEBUG] aDown?", aDown);
            telemetry.addData("[DEBUG] clawOpen?", clawOpen);
            telemetry.update();

            lastRuntime = runtime.time();  //Set last runtime to current runtime
        }
    }
}
