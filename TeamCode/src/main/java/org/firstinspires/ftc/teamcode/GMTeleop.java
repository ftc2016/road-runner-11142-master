package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// ^^ The required packages and classes are imported into the Teleop code ^^

@Disabled
@TeleOp(name = "GM TeleOp")
public class GMTeleop extends OpMode {
    DcMotorEx rightFront;
    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotor intake;
    DcMotor flywheel;
    DcMotor arm;
    Servo index;
    Servo grab;
    Servo block;

    // Define global variables called in the loop
    boolean toggleBlocker = true;
    boolean toggleFly = false;
   // double flyPow = 0.375;

    // above initializes all the aspects we need to make our robot function
    @Override
    public void init() {

        // defining all the hardware
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("lf");
        leftRear = (DcMotorEx) hardwareMap.dcMotor.get("lr");
        rightRear = (DcMotorEx) hardwareMap.dcMotor.get("rr");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rf");
        intake = (DcMotor) hardwareMap.dcMotor.get("intake");
        flywheel = (DcMotor) hardwareMap.dcMotor.get("flywheel");
        arm = (DcMotor) hardwareMap.dcMotor.get("arm");
        index = hardwareMap.servo.get("indexer");
        grab = hardwareMap.servo.get("grabber");
        block = hardwareMap.servo.get("blocker");


        // This puts rotated motors in reverse to ensure all motors go the same direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);


        // Enable encoders on motors that have them
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Teleop begins when button is clicked on the driver station
    // The robot will cotinuosly run inside this loop until the opMode is foce stopped by the driver station
    @Override
    public void loop() {

        // Display the flywheel power when lower power is toggled for the powershots
       // telemetry.addData("Flypow: ", flyPow);
        //telemetry.update();

        // Assign gamepad controls to the motor variables
        float x1 = -gamepad1.right_stick_x;
        float y1 = gamepad1.left_stick_y;
        float r1 = gamepad1.right_trigger;
        float r2 = gamepad1.left_trigger;
        float f1 = gamepad2.right_stick_y;

        // Boolean variables are used for toglleable switches
        boolean indexRing = gamepad2.right_bumper;
        boolean takein = gamepad2.a;
        boolean takeout = gamepad2.b;
        boolean grabWobble = gamepad2.left_bumper;

        // Reset variables
        float leftFrontPower = 0;
        float leftBackPower = 0;
        float rightFrontPower = 0;
        float rightBackPower = 0;

        // Create a button that sets flywheel power lower for shooting powershots
        /*
        if(gamepad2.x && !toggleFly) {
            if (flyPow == 0.375) {
                flyPow = 0.45;
            } else {
                flywheel.setPower(flyPow);
                toggleFly = true;
            }
        } else if (!gamepad2.x){
            toggleFly = false;
            flyPow = 0.375;
        }
*/

        if(gamepad1.a) {

            toggleBlocker = !toggleBlocker;

        }
        if(!toggleBlocker) {
            block.setPosition(0.975);
        }
        else if(toggleBlocker) {
            block.setPosition(0.575);
        }

        if(gamepad2.right_stick_y < 0) {
            flywheel.setPower(0.48);
        }
        if(gamepad2.right_stick_y > 0) {
            flywheel.setPower(0.00);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
       if (y1 > 0 && x1 == 0) {             // Backwards movement
           leftFrontPower -= y1;
           leftBackPower -= y1;
           rightFrontPower -= y1;
           rightBackPower -= y1;
       } else if (y1 < 0 && x1 == 0) {      // Forwards movement
           leftFrontPower -= y1;
           leftBackPower -= y1;
           rightFrontPower -= y1;
           rightBackPower -= y1;
       } else if (x1 > 0 && y1 == 0) {      // Strafe left
           leftFrontPower -= x1;
           leftBackPower += x1;
           rightFrontPower += x1;
           rightBackPower -= x1;
       } else if (x1 < 0 && y1 == 0) {      // Strafe Right
           leftFrontPower -= x1;
           leftBackPower += x1;
           rightFrontPower += x1;
           rightBackPower -= x1;
       } else if (x1 > 0 && y1 > 0) {       // Backleft diagonal movement
           rightBackPower -= y1;
           leftFrontPower -= y1;
       } else if (x1 < 0 && y1 < 0) {       // Forwardright diagonal movement
           rightBackPower -= y1;
           leftFrontPower -= y1;
       }else if (x1 > 0 && y1 < 0) {        // Forwardleft diagonal movement
           leftBackPower -= y1;
           rightFrontPower -= y1;
       } else if (x1 < 0 && y1 > 0) {       // Backright diagonal movement
           leftBackPower -= y1;
           rightFrontPower-= y1;
       } else {                             // Reset motor power when robot is not moving
           leftFrontPower = 0;
           leftBackPower = 0;
           rightFrontPower = 0;
           rightBackPower = 0;
       }

        // Handle clockwise turning movement
        // A multiplier is added to turning movement for slower and more controlled rotation
        leftFrontPower -= r1*0.625;
        leftBackPower -= r1*0.625;
        rightFrontPower += r1*0.625;
        rightBackPower += r1*0.625;

        // Handle counterclockwise turning movement
        // Rotation multiplier = 50% power
        leftFrontPower += r2*0.625;
        leftBackPower += r2*0.625;
        rightFrontPower -= r2*0.625;
        rightBackPower -= r2*0.625;

        // Set intake power to 100% to take rings ito the robot and drop them in the indexer system
        if (takein == true) {
            intake.setPower(1.00);
        } else if (takeout == true) {
            intake.setPower(-1.00);
        } else {
            intake.setPower(0.00);
        }

        // Sets servo position to push the bottom-most ring into the flywheel
        if (indexRing == true) {
            index.setPosition(0.75);
        } else {
            index.setPosition(1.00);
        }

        // Opens the wobble grabber servos when the correct button is held down
        // The wobble grabber servos are reset to a closed position when the button is not pressed
        if (grabWobble == true) {
            grab.setPosition(0.50);
        } else {
            grab.setPosition(1.00);

        }

        // The flywheel is set to a variable power. It is set to 87.5% power (5075 rpm) by default
        /*
        if (f1 < 0) {
            flywheel.setPower(flyPow);
        } else {
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (f1 > 0) {
            flywheel.setPower(0.00);
    }
         */

        // Wobble pickup arm receives direct motor power from joystick position values
        arm.setPower(gamepad2.left_stick_y);

        // Scale movement for the wheels
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));

        if (max > 1) {
            leftFrontPower = (float) Range.scale(leftFrontPower, -max, max, -.375, .375);
            leftBackPower = (float) Range.scale(leftBackPower, -max, max, -.375, .375);
            rightFrontPower = (float) Range.scale(rightFrontPower, -max, max, -.375, .375);
            rightBackPower = (float) Range.scale(rightBackPower, -max, max, -.375, .375);
        }
        leftRear.setPower(-leftBackPower);
        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(-rightFrontPower);
        rightRear.setPower(-rightBackPower);
    }
}