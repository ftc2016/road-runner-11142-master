package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// ^^ The required packages and classes are imported into the autonomous code ^^

@Autonomous(name = "GM-1 Spline")
public class GMSpline extends LinearOpMode {

    // Tensorflow labels are given to the ring stack size assets
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    // Import the variables from the util class for the motors and servos
    SampleMecanumDrive drive;
    // Reset the ring stack size variables for use in Tensorflow
    int zero = 0;
    int one= 0;
    int four = 0;
    List<Recognition> updatedRecognitions;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     */

    private static final String VUFORIA_KEY =
            "AZT5cbD/////AAABmfJnIeddQEINhStd867KmsE/U5DxLb3la9BlLgqlAj7pwpiD/JFpwc61NP8dil/k9TpMthRa3J0OFg2P1oaCjeRLb8s4ku8mWqY142NCUbQmrnMtzCDezbfhmeXOdgONV7+oW2Nu50zXzUwG/tkR8UgWiMkSU9M3ZEyZEhaG4sscMoY/tW23IWyq6PoMwIz8aQdtc+hm68hvEES4GYTPnxz0XvyPSNcGWmBPMGWYmKRy9i7ZGJG6/L29z3Y7AP2bnQ5gzHPuWWr6plIcxNP2jvaafqTPz8Nn2tjZ7yk3KPeHmf4MLjuseGCWV8wub/+sDKt2B5/vVntltf5/0gVAb25avPL9aNOGLDHEePembGAb";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first
        drive = new SampleMecanumDrive(hardwareMap);
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.0, 1.78);
        }
        drive.originalHeading = 0;
        /** Wait for the game to begin */
        waitForStart();

        //Autonomous begins
        if (opModeIsActive())
        {
            // Begin TFOD loop once autunomous starts
            // Tensorflow will run parameters 10 times to determine the ring stack size
            for(int ti = 1; ti < 10; ti++) {
                if (tfod != null) {
                    updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0) {
                            // Empty list, no objects recognized.
                            telemetry.addData("TFOD", "No items detected.");
                            telemetry.addData("Target Zone", "A");
                            telemetry.update();
                            zero++;
                            // No rings detected, robot may move to zone A
                        } else {
                            // If list is not empty:
                            // Step through the list of recognitions and display boundary info
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                // Check label to see which target zone to go after
                                if (recognition.getLabel().equals("Single")) {
                                    telemetry.addData("Target Zone", "B");
                                    telemetry.update();
                                    one++;
                                    // One ring was detected, robot may move to zone B
                                } else if (recognition.getLabel().equals("Quad")) {
                                    telemetry.addData("Target Zone", "C");
                                    telemetry.update();
                                    four++;
                                    // Four rings were detected, robot may move to zone C
                                } else {
                                    telemetry.addData("Target Zone", "UNKNOWN");
                                    telemetry.update();
                                    // If there was an error in detection, this caption will appear
                                }
                            }
                        }
                    }
                }
            }
            /*
             * After the TFOD program has looped 10 times, the occurrence of each ring position is stored
             * The ring position (stack size) that was detected the most times will be found
             * The other variables are then discarded and the chosen ring position is executed
             */
            if (zero > one && zero > four) {
                // If 0 rings were detected, these methods will be executed
                ZoneA();

            }
            else if (one > zero && one > four) {
                // If 1 ring was detected, these methods will be executed
                ZoneB();
            }
            else if (four > zero && four > one) {
                // If 4 rings were detected, these methods will be executed
                ZoneC();

            }
        }
        //Tensorflow will shut down after the OpMode is successfully executed
        if (tfod != null) {
            tfod.shutdown();
        }
        // End of Autonomous
    }

    public void gyroCorrect()
    {
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
    }

    // > Methods for larger lists of actions are written here <

    public void ZoneA()
    {
        drive.flywheelOne.setVelocity((2650 * 28) / 60);
        drive.flywheelTwo.setVelocity((2650 * 28) / 60);
        Trajectory positionPowershot = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(66, 12),  0)
                .build();
        Trajectory wobbleDrop1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-15, 65, Math.toRadians(0)))
                .build();
        Trajectory wobblePickup = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-50, -60, Math.toRadians(0)))
                .build();
        Trajectory wobbleGrab = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(15, 0, Math.toRadians(0)))
                .build();
        Trajectory wobbleDrop2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-10, -90, Math.toRadians(0)))
                .build();
        Trajectory wobbleForward = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .build();
        Trajectory linePark = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(0)))
                .build();
        drive.followTrajectory(positionPowershot);

        sleep(100);
        gyroCorrect();
        sleep(500);
        indexPowershot();
        drive.arm.setPower(0.14);
        drive.followTrajectory(wobbleDrop1);
        drive.arm.setPower(0);
        drive.grab.setPosition(0.375);
        sleep(100);
        drive.followTrajectory(wobblePickup);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(wobbleGrab);
        drive.grab.setPosition(1.00);
        sleep(100);
        drive.followTrajectory(wobbleDrop2);
        drive.followTrajectory(wobbleForward);
        drive.grab.setPosition(0.375);
        sleep(100);
        drive.followTrajectory(linePark);
       drive.turn(Math.toRadians(90));

    }
    public void ZoneB()
    {
        drive.flywheelOne.setVelocity((2650 * 28) / 60);
        drive.flywheelTwo.setVelocity((2650 * 28) / 60);
        Trajectory positionPowershot = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(66, 12),  0)
                .build();
        Trajectory collectStack = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-60, -6, Math.toRadians(0)))
                .build();
        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(30, -6, Math.toRadians(0)))
                .build();
        Trajectory wobbleDrop1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(25, 25, Math.toRadians(0)))
                .build();
        Trajectory wobblePickUp = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, -10, Math.toRadians(0)))
                .build();
        Trajectory wobbleDrop2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(0)))
                .build();
        Trajectory linePark = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-5, 0, Math.toRadians(0)))
                .build();

        drive.followTrajectory(positionPowershot);
        gyroCorrect();
        sleep(250);
        indexPowershot();
        drive.arm.setPower(0.175);
        drive.followTrajectory(wobbleDrop1);
        drive.arm.setPower(0.00);
        drive.grab.setPosition(0.375);
        sleep(100);
        drive.intake.setPower(1.00);
        drive.followTrajectory(collectStack);
        drive.turn(Math.toRadians(-170));
        drive.followTrajectory(wobblePickUp);
        drive.grab.setPosition(1.00);
        sleep(100);
        drive.intake.setPower(0);
        drive.flywheelOne.setVelocity((2900 * 28) / 60);
        drive.flywheelTwo.setVelocity((2900 * 28) / 60);
        drive.turn(Math.toRadians(-185));
        gyroCorrect();
        sleep(250);
        drive.followTrajectory(shootHigh);
        gyroCorrect();
        sleep(250);
        ringIndexOne();
        drive.followTrajectory(wobbleDrop2);
        drive.grab.setPosition(0.375);
        sleep(100);
        drive.followTrajectory(linePark);
    }
    public void ZoneC()
    {
        drive.flywheelOne.setVelocity((2650 * 28) / 60);
        drive.flywheelTwo.setVelocity((2650 * 28) / 60);
        Trajectory positionPowershot = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(66, 12),  0)
                .build();
        Trajectory collectStackStrafe = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 25, Math.toRadians(0)))
                .build();
        Trajectory collectStack = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-40, -32.5, Math.toRadians(0)))
                .build();
        Trajectory collectStackBack = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-50, 0, Math.toRadians(0)))
                .build();
        Trajectory collectStack2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-60, 0, Math.toRadians(0)))
                .build();
        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(35, -5, Math.toRadians(0)))
                .build();
        Trajectory shootHigh2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(60, 0, Math.toRadians(0)))
                .build();
        Trajectory wobbleDrop1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(45, 52.5, Math.toRadians(0)))
                .build();
        Trajectory wobbleDrop2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(0)))
                .build();
        Trajectory linePark = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(0)))
                .build();

        drive.followTrajectory(positionPowershot);
        gyroCorrect();
        sleep(250);
        indexPowershot();
        drive.arm.setPower(0.1);
        drive.followTrajectory(wobbleDrop1);
        drive.arm.setPower(0.00);
        drive.grab.setPosition(0.375);
        sleep(100);
        drive.intake.setPower(1.00);
        drive.followTrajectory(collectStack);
        drive.followTrajectory(collectStackBack);


        drive.flywheelOne.setVelocity((2900 * 28) / 60);
        drive.flywheelTwo.setVelocity((2900 * 28) / 60);
        drive.followTrajectory(shootHigh);
        gyroCorrect();
        drive.intake.setPower(0);
        sleep(1000);
        ringIndexAll();

        drive.intake.setPower(1.00);
        drive.followTrajectory(collectStack2);

        drive.flywheelOne.setVelocity((2900 * 28) / 60);
        drive.flywheelTwo.setVelocity((2900 * 28) / 60);
        drive.followTrajectory(shootHigh2);
        gyroCorrect();
        drive.intake.setPower(0);
        sleep(1000);
        drive.grab.setPosition(0.75);
        ringIndexAll();

    }
    public void indexPowershot()
    {
        Trajectory Strafe1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 11, Math.toRadians(0)))
                .build();
        Trajectory Strafe2 = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 13, Math.toRadians(0)))
                .build();
        // Push ring out with the servo arm
        drive.index.setPosition(0.75);
        sleep(100);
        // Pull servo arm back in
        drive.index.setPosition(1.0);
        // Reset flywheelOne to max power
        drive.flywheelOne.setVelocity((2650 * 28) / 60);
        drive.flywheelTwo.setVelocity((2650 * 28) / 60);
        drive.followTrajectory(Strafe1);
        gyroCorrect();
        sleep(250);

        drive.index.setPosition(0.75);
        sleep(100);
        drive.index.setPosition(1.0);
        drive.flywheelOne.setVelocity((2650 * 28) / 60);
        drive.flywheelTwo.setVelocity((2650 * 28) / 60);
        drive.followTrajectory(Strafe2);
        gyroCorrect();
        sleep(250);

        drive.index.setPosition(0.75);
        sleep(100);
        drive.index.setPosition(1.0);
        stopflywheel();
    }
    public void ringIndexOne()
    {
        // Push ring out with the servo arm
        drive.index.setPosition(0.75);
        sleep(100);
        drive.index.setPosition(1.0);
        stopflywheel();
    }
    public void ringIndexAll()
    {
        // Push ring out with the servo arm
        drive.index.setPosition(0.75);
        sleep(250);
        // Pull servo arm back in
        drive.index.setPosition(1.0);
        // Reset flywheelOne to max power
        drive.flywheelOne.setVelocity((2850 * 28) / 60);
        drive.flywheelTwo.setVelocity((2850 * 28) / 60);
        sleep(500);

        drive.index.setPosition(0.75);
        sleep(250);
        drive.index.setPosition(1.0);
        drive.flywheelOne.setVelocity((2850 * 28) / 60);
        drive.flywheelTwo.setVelocity((2850 * 28) / 60);
        sleep(500);

        drive.index.setPosition(0.75);
        sleep(250);
        drive.index.setPosition(1.0);
        stopflywheel();
        sleep(500);
    }
    public void stopflywheel() {

        drive.flywheelOne.setVelocity(0);
        drive.flywheelTwo.setVelocity(0);
        drive.flywheelOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.flywheelTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Initialize the Vuforia localization engine.
    private void initVuforia()
    {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    // Initialize the TensorFlow Object Detection engine.
    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}