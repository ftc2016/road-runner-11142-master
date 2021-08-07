package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous(name="P-Controller Single Motor control based on encoder", group="Exercises")
public class SingleMotorPController extends LinearOpMode {
    DcMotor arm;
    @Override
    public void runOpMode() throws InterruptedException {

        int encoderDegreesToAttain = 203;
        double minPower = 0.01;
        double maxPower = 0.625;
        PController pController = new PController(0.03);
        pController.setInputRange(50, 500);
        pController.setSetPoint(encoderDegreesToAttain);
        pController.setOutputRange(minPower, maxPower);

        arm = hardwareMap.dcMotor.get("arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("encoder position", arm.getCurrentPosition());
            telemetry.addData("power",
                    pController.getComputedOutput(arm.getCurrentPosition()));
            telemetry.update();
            if(arm.getCurrentPosition() <  encoderDegreesToAttain){
                arm.setPower(minPower +
                        pController.getComputedOutput(arm.getCurrentPosition()));
            } else {
                arm.setPower(minPower -
                        pController.getComputedOutput(arm.getCurrentPosition()));
            }

        }
        arm.setPower(0);
    }
}