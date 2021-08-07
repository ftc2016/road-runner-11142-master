package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@Disabled
@Autonomous(name = "ArmCode")
public class ArmCode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm;
        arm = hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double liftPosScale = 10, liftPowScale = 0.005;
        double liftPosCurrent = 0, liftPosDes = 0, liftPosError = 0, liftPow = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.dpad_down == true) {
                double turn = 386 / 4;
                int valueDown = arm.getCurrentPosition() + 110;
                arm.setTargetPosition(valueDown);
                telemetry.addData("Status", "Success", valueDown);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(1.0);
                // sleep(750);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (!arm.isBusy()) {
                    arm.setPower(0);
                }
                //drivetrain.arm.setPower(0.00);


                telemetry.addData("Status", "Success");
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                telemetry.addData("armPos", arm.getCurrentPosition());
                telemetry.update();
            }

//        liftPosCurrent = arm.getCurrentPosition();
//
//        liftPosDes += liftPosScale*gamepad2.left_stick_y;                //input scale factor
//        liftPosError = liftPosDes - liftPosCurrent;
////        integrater += liftPosError;                                           //unnecessary
//        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
//        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
//        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
//        arm.setPower(liftPow);

        }
    }
}
