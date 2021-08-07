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

@Autonomous(name = "GM-1 Auto")
public class GMAuto extends LinearOpMode {

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
                initialShooting();
               ZoneA();
             //  gyroCorrect();
            }
            else if (one > zero && one > four) {
                // If 1 ring was detected, these methods will be executed
                initialShooting();
                ringPickupB();
                ZoneB();
            }
            else if (four > zero && four > one) {
                // If 4 rings were detected, these methods will be executed
                initialShooting();
                ringPickupC();
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
    public void initialShooting()
    {
        Trajectory avoidStack = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(66, 0, Math.toRadians(0)))
                .build();
        drive.followTrajectory(avoidStack);
        startflywheelOne(0.45);
        Trajectory shootPosition = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 30, Math.toRadians(2.6)))
                .build();
        drive.followTrajectory(shootPosition);
        gyroCorrect();
        sleep(500);
        ringIndex();
    }
    public void ringPickupB()
    {
        drive.intake.setPower(1.00);
        Trajectory collectStack = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-46, -4, Math.toRadians(0)))
                .build();
        drive.followTrajectory(collectStack);

        Trajectory shootPositionTwo = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(44, 2, Math.toRadians(-2)))
                .build();
        drive.followTrajectory(shootPositionTwo);
        gyroCorrect();
        startflywheelOne(0.45);
        drive.intake.setPower(0.00);
        sleep(2500);
        ringIndex();
    }
    public void ringPickupC()
    {
        drive.intake.setPower(1.00);
        Trajectory collectStack = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-44, -4, Math.toRadians(0)))
                .build();
        drive.followTrajectory(collectStack);

        Trajectory shootPositionTwo = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(44, 2, Math.toRadians(-2)))
                .build();
        drive.followTrajectory(shootPositionTwo);
        gyroCorrect();
        startflywheelOne(0.4575);
        drive.intake.setPower(0.00);
        sleep(2500);
        ringIndex();
        sleep(500);

    }
    public void ZoneA()
    {
        drive.originalHeading += 91;
        drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        drive.absHeading = drive.angles.firstAngle;
        telemetry.addData("Heading", drive.absHeading);
        telemetry.update();
        for(int a = 1; a < 52.5; a++)
        {
            drive.leftFront.setPower(-0.0165 * (drive.absHeading - drive.originalHeading));
            drive.leftRear.setPower(-0.0165 * (drive.absHeading - drive.originalHeading));
            drive.rightFront.setPower(0.0165 * (drive.absHeading - drive.originalHeading));
            drive.rightRear.setPower(0.0165 * (drive.absHeading - drive.originalHeading));
            drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            drive.absHeading = drive.angles.firstAngle;
            telemetry.addData("Heading", drive.absHeading);
            telemetry.update();
        }
        drive.leftFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightRear.setPower(0);
        sleep(1000);
        telemetry.update();
        Trajectory wobble1DeliveryA = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(0)))
                .build();
        drive.followTrajectory(wobble1DeliveryA);
        telemetry.update();
        wobbleDrop();
        telemetry.update();

        Trajectory wobbleBackup = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-4,0 , Math.toRadians(0)))
                .build();

        drive.followTrajectory(wobbleBackup);

        drive.originalHeading += 91;
        drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        drive.absHeading = drive.angles.firstAngle;
        telemetry.addData("Heading", drive.absHeading);
        telemetry.update();
        for(int b = 1; b < 45; b++)
        {
            drive.leftFront.setPower(-0.0165 * (drive.absHeading - drive.originalHeading));
            drive.leftRear.setPower(-0.0165 * (drive.absHeading - drive.originalHeading));
            drive.rightFront.setPower(0.0165 * (drive.absHeading - drive.originalHeading));
            drive.rightRear.setPower(0.0165 * (drive.absHeading - drive.originalHeading));
            drive.angles = drive.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            drive.absHeading = drive.angles.firstAngle;
            telemetry.addData("Heading", drive.absHeading);
            telemetry.update();
            if (drive.absHeading <= 0)
            {
                break;
            }
        }
        drive.leftFront.setPower(0);
        drive.leftRear.setPower(0);
        drive.rightFront.setPower(0);
        drive.rightRear.setPower(0);

        sleep(1000);
        telemetry.update();

        //drive.turn(Math.toRadians(-85));
        Trajectory wobble2PickupA = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(40,-4 , Math.toRadians(0)))
                .build();

        drive.followTrajectory(wobble2PickupA);
        Trajectory wobble2PickupARight = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(6,-8 , Math.toRadians(0)))
                .build();

        //drive.followTrajectory(wobble2PickupARight);
        wobblePick();
        Trajectory wobble2DeliveryA = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-70,15 , Math.toRadians(0)))
                .build();
        drive.followTrajectory(wobble2DeliveryA);
        Trajectory wobble2DeliveryAStrafe = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0,-22 , Math.toRadians(0)))
                .build();
        drive.followTrajectory(wobble2DeliveryAStrafe);
        wobbleRelease();


        Trajectory lineParkAStrafe = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 20, Math.toRadians(0)))
                .build();
        drive.followTrajectory(lineParkAStrafe);

        Trajectory lineParkA = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(0)))
                .build();
        drive.followTrajectory(lineParkA);

    }
    public void ZoneB()
    {
        Trajectory wobble1DeliveryB = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .splineToLinearHeading(new Pose2d(24, 8, Math.toRadians(0)), Math.toRadians(0))
                .build();
        drive.followTrajectory(wobble1DeliveryB);
        wobbleDrop();

        Trajectory lineParkB = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-15, 50, Math.toRadians(0)))
                .build();
        drive.followTrajectory(lineParkB);
    }
    public void ZoneC()
    {
        Trajectory wobble1DeliveryC = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .splineToLinearHeading(new Pose2d(45, 32.5, Math.toRadians(0)), Math.toRadians(0))
                .build();
        drive.followTrajectory(wobble1DeliveryC);
        wobbleDrop();

        Trajectory lineParkC = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(-40, 0, Math.toRadians(0)))
                .build();
        drive.followTrajectory(lineParkC);
        Trajectory lineParkCStrafe = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(0, 10, Math.toRadians(0)))
                .build();
        drive.followTrajectory(lineParkCStrafe);
    }
    public void ringIndex()
    {
        // Push ring out with the servo arm
        drive.index.setPosition(0.75);
        sleep(250);
        // Pull servo arm back in
        drive.index.setPosition(1.00);
        // Reset flywheelOne to max power
        startflywheelOne(0.46);
        sleep(500);

        drive.index.setPosition(0.75);
        sleep(250);
        drive.index.setPosition(1.00);
        startflywheelOne(0.46);
        sleep(500);

        drive.index.setPosition(0.75);
        sleep(250);
        drive.index.setPosition(1.00);
        stopflywheelOne();
        sleep(500);
    }
    public void wobbleDrop()
    {
        // The wobble arm is brought down, then stopped
        drive.arm.setPower(1.00);
        sleep(1500);
        drive.arm.setPower(0.00);
        sleep(100);

        // The servo grabber releases the wobble
        drive.grab.setPosition(0.375);
        sleep(250);
    }
    public void wobblePick()
    {
        drive.grab.setPosition(1.0);
        sleep(500);
    }
    public void wobbleRelease()
    {
        drive.grab.setPosition(0.5);
        sleep(250);
    }
    // Runs the flywheelOne at any desired power
    public void startflywheelOne(double power)
    {
        drive.flywheelOne.setPower(power);
        // The program will double check the power and make sure its is correct
        if(power > 0) {
            double currentPower = drive.flywheelOne.getPower();
            double error = (power - currentPower);
            drive.flywheelOne.setPower(currentPower + error);
        }
    }
    public void stopflywheelOne() {
        drive.flywheelOne.setPower(0.0);
        drive.flywheelOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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