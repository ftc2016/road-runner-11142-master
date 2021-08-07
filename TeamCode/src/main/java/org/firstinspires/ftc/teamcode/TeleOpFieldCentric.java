package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(name = "GM TeleOp FC")
public class TeleOpFieldCentric extends LinearOpMode {
    private Object PoseStorage;


    boolean isBlockerDown = false;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            gamepad1.right_trigger
                            -gamepad1.left_trigger
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            boolean indexRing = gamepad2.right_bumper;
            boolean takein = gamepad2.a;
            boolean takeout = gamepad2.b;
            boolean grabWobble = gamepad2.left_bumper;


            if(gamepad2.right_stick_y < 0) {
                drive.flywheelOne.setVelocity((2900 * 28) / 60);
                drive.flywheelTwo.setVelocity((2900 * 28) / 60);
            }
            if(gamepad2.right_stick_y > 0) {
                drive.flywheelOne.setVelocity(0);
                drive.flywheelTwo.setVelocity(0);
                drive.flywheelOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.flywheelTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (gamepad2.x == true) {
                drive.flywheelOne.setVelocity((2650 * 28) / 60);
                drive.flywheelTwo.setVelocity((2650 * 28) / 60);
            }

            if (gamepad2.dpad_up == true) {
                drive.block.setPosition(0.05);
            } else if(gamepad2.dpad_down == true) {
                drive.block.setPosition(0.80);
            }

            // Set intake power to 100% to take rings ito the robot and drop them in the indexer system
            if (takein == true) {
                drive.intake.setPower(1.00);
            } else if (takeout == true) {
                drive.intake.setPower(-1.00);
            } else {
                drive.intake.setPower(0.00);
            }

            // Sets servo position to push the bottom-most ring into the flywheelOne
            if (indexRing == true) {
                drive.index.setPosition(0.75);
            } else {
                drive.index.setPosition(1.00);
            }

            // Opens the wobble grabber servos when the correct button is held down
            // The wobble grabber servos are reset to a closed position when the button is not pressed
            if (grabWobble == true) {
                drive.grab.setPosition(0.50);
            } else {
                drive.grab.setPosition(1.00);
            }



            drive.arm.setPower(-gamepad2.left_stick_y);

            if (gamepad1.x == true) {

                drive.setPoseEstimate(startPose);
                drive.originalHeading = drive.absHeading;
            }




            if(gamepad1.y == true ) {

                double flypower = drive.flywheelOne.getPower();
                drive.flywheelOne.setPower(0.425);
                // The program will double check the power and make sure its is correct
                if(flypower > 0) {
                    double currentPower = drive.flywheelOne.getPower();
                    double error = (flypower - currentPower);
                    drive.flywheelOne.setPower(currentPower + error);
                }
                sleep(500);
                Trajectory shootPosition1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                        .lineToLinearHeading(new Pose2d(0, 48, Math.toRadians(0)))
                        .build();
                drive.followTrajectory(shootPosition1);
                drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                drive.absHeading = drive.angles.firstAngle;
                telemetry.addData("Heading", drive.absHeading);
                telemetry.update();
                for(int f = 1; f < 20; f++)
                {
                    drive.leftFront.setPower(-0.05 * (drive.absHeading - drive.originalHeading));
                    drive.leftRear.setPower(-0.05 * (drive.absHeading - drive.originalHeading));
                    drive.rightFront.setPower(0.05 * (drive.absHeading - drive.originalHeading));
                    drive.rightRear.setPower(0.05 * (drive.absHeading - drive.originalHeading));
                    drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    drive.absHeading = drive.angles.firstAngle;
                    telemetry.addData("Heading", drive.absHeading);
                    telemetry.update();
                }
                drive.leftFront.setPower(0);
                drive.leftRear.setPower(0);
                drive.rightFront.setPower(0);
                drive.rightRear.setPower(0);
                telemetry.update();
                // Push ring out with the servo arm
                drive.index.setPosition(0.75);
                sleep(250);
                // Pull servo arm back in
                drive.index.setPosition(1.00);
                // Reset flywheelOne to max power
                drive.flywheelOne.setPower(0.425);
                // The program will double check the power and make sure its is correct
                if(flypower > 0) {
                    double currentPower = drive.flywheelOne.getPower();
                    double error = (flypower - currentPower);
                    drive.flywheelOne.setPower(currentPower + error);
                }
                sleep(500);

                Trajectory shootPosition2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                        .lineToLinearHeading(new Pose2d(0, 16, Math.toRadians(0)))
                        .build();
                drive.followTrajectory(shootPosition2);
                drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                drive.absHeading = drive.angles.firstAngle;
                telemetry.addData("Heading", drive.absHeading);
                telemetry.update();
                for(int f = 1; f < 20; f++)
                {
                    drive.leftFront.setPower(-0.05 * (drive.absHeading - drive.originalHeading));
                    drive.leftRear.setPower(-0.05 * (drive.absHeading - drive.originalHeading));
                    drive.rightFront.setPower(0.05 * (drive.absHeading - drive.originalHeading));
                    drive.rightRear.setPower(0.05 * (drive.absHeading - drive.originalHeading));
                    drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    drive.absHeading = drive.angles.firstAngle;
                    telemetry.addData("Heading", drive.absHeading);
                    telemetry.update();
                }
                drive.leftFront.setPower(0);
                drive.leftRear.setPower(0);
                drive.rightFront.setPower(0);
                drive.rightRear.setPower(0);
                telemetry.update();
                // Push ring out with the servo arm
                drive.index.setPosition(0.75);
                sleep(250);
                // Pull servo arm back in
                drive.index.setPosition(1.00);
                // Reset flywheelOne to max power
                drive.flywheelOne.setPower(0.425);
                // The program will double check the power and make sure its is correct
                if(flypower > 0) {
                    double currentPower = drive.flywheelOne.getPower();
                    double error = (flypower - currentPower);
                    drive.flywheelOne.setPower(currentPower + error);
                }
                sleep(1000);

                Trajectory shootPosition3 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                        .lineToLinearHeading(new Pose2d(0, 16, Math.toRadians(0)))
                        .build();
                drive.followTrajectory(shootPosition3);
                drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                drive.absHeading = drive.angles.firstAngle;
                telemetry.addData("Heading", drive.absHeading);
                telemetry.update();
                for(int f = 1; f < 20; f++)
                {
                    drive.leftFront.setPower(-0.05 * (drive.absHeading - drive.originalHeading));
                    drive.leftRear.setPower(-0.05 * (drive.absHeading - drive.originalHeading));
                    drive.rightFront.setPower(0.05 * (drive.absHeading - drive.originalHeading));
                    drive.rightRear.setPower(0.05 * (drive.absHeading - drive.originalHeading));
                    drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    drive.absHeading = drive.angles.firstAngle;
                    telemetry.addData("Heading", drive.absHeading);
                    telemetry.update();
                }
                drive.leftFront.setPower(0);
                drive.leftRear.setPower(0);
                drive.rightFront.setPower(0);
                drive.rightRear.setPower(0);
                telemetry.update();
                // Push ring out with the servo arm
                drive.index.setPosition(0.75);
                sleep(250);
                // Pull servo arm back in
                drive.index.setPosition(1.00);
                // Reset flywheelOne to max power
                drive.flywheelOne.setPower(0.425);
                // The program will double check the power and make sure its is correct
                if(flypower > 0) {
                    double currentPower = drive.flywheelOne.getPower();
                    double error = (flypower - currentPower);
                    drive.flywheelOne.setPower(currentPower + error);
                }
                sleep(500);
                drive.flywheelOne.setPower(0.0);
                drive.flywheelOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

}