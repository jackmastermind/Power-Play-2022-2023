package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.ArrayUtils;

import java.util.Arrays;

@TeleOp(name="Master Drive")
@SuppressWarnings("FieldCanBeLocal")
public class MasterDrive extends LinearOpMode
{
    private final MecanumMap_Master master = new MecanumMap_Master();
    private SlideController slide;
    private ClawController clawController;
    private SusanController susanController;

    private DcMotor[] motors;
    private Servo[] servos;

    //MOTOR VARIABLES
    double armSpeed = 1;
    double clawSpeed = -0.003;
    double susanSpeed = 0.5;

    boolean aDown = false;


    private final ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {
        //INITIALIZE OBJECTS
        master.init(hardwareMap);
        slide = new SlideController(hardwareMap);
        clawController = new ClawController(hardwareMap);
        susanController = new SusanController(hardwareMap);

        motors = new DcMotor[]{master.frontLeft, master.frontRight,
                master.backLeft, master.backRight, slide.spoolMotor, susanController.susan};
        servos = new Servo[]{clawController.claw, clawController.wrist};

        MotorRecorder recorder = new MotorRecorder(runtime, hardwareMap, motors,
                servos, 0.1, telemetry);

        slide.ignoreMinMax = true; //Todo: Remove this once min max are determined

        waitForStart();
        telemetry.setAutoClear(true);
        runtime.reset();

        double lastRuntime = runtime.time();
        slide.raiseLinear();
        clawController.closeClaw();
        clawController.moveWrist(0, 1); //Initializes to start pos

        while(opModeIsActive())
        {
            double deltaTime = runtime.time() - lastRuntime;

            //region DRIVE SECTION
            double powerMultiplier = 0.5;

            double inputLX = gamepad1.left_stick_x;
            double inputLY = gamepad1.left_stick_y;
            double inputRX = gamepad1.right_stick_x;

            double normalization = Math.max(Math.abs(inputLX) + Math.abs(inputLY) + Math.abs(inputRX), 1.0);

            double flPower = Math.pow(((-inputLX + inputLY) - inputRX)/ normalization, 3) * powerMultiplier;
            double blPower = Math.pow(((inputLX + inputLY) - inputRX)/ normalization, 3) * powerMultiplier;

            double frPower = Math.pow(((inputLX + inputLY) + inputRX)/ normalization, 3) * powerMultiplier;
            double brPower = Math.pow(((-inputLX + inputLY) + inputRX)/ normalization, 3) * powerMultiplier;
            //endregion

            //region RECORDING DUMP
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
            //endregion

            //region OPERATING SECTION

            //Arm
            slide.MoveSlide(gamepad2.left_stick_y, armSpeed);

            if (gamepad2.dpad_up)
            {
                slide.raiseLinear();
            }
            else if (gamepad2.dpad_down)
            {
                slide.lowerLinear();
            }

            //Wrist
            clawController.moveWrist(gamepad2.right_stick_y, clawSpeed);

            //Claw
            if (gamepad2.a && !aDown)
            {
               clawController.toggleClaw();
            }
            else if (!gamepad2.a){
                aDown = false;
            }

            //Susan
            susanController.moveSusan(gamepad2.right_trigger - gamepad2.left_trigger,
                    susanSpeed);
            //endregion


            //region SET POWER
            master.frontLeft.setPower(flPower);
            master.frontRight.setPower(frPower);
            master.backLeft.setPower(blPower);
            master.backRight.setPower(brPower);
            //endregion

            //region TELEMETRY
            telemetry.addLine("DRIVE MOTORS");
            telemetry.addData("fl", flPower);
            telemetry.addData("fr", frPower);
            telemetry.addData("bl", blPower);
            telemetry.addData("br", brPower);
            telemetry.addLine("OPERATOR MOTORS");
            clawController.LogValues(telemetry);
            slide.LogValues(telemetry);
            susanController.LogValues(telemetry);
            telemetry.update();
            //endregion

            lastRuntime = runtime.time();  //Set last runtime to current runtime
        }
    }
}
