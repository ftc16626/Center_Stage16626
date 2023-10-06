package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "armSample") //ARI COMMENT: this name is going to show up on your robot controller
public class StrafeTeleOp extends LinearOpMode {


    //ARI COMMENT: This space here is where you should define all your local variables
    //For example, if I have a power variable I want to reference in the code multiple times, I would do it here
    /* EXAMPLE:

    int powerExample = 3;

     */


    //ALSO ARI COMMENT: you can define your local dcmotors and stuff here too. This won't break your code without it, but it makes it easier to reference
    //so for example, I could declare that there is a DcMotor called leftMotor here. However, this won't reference anything from the robot Controller.

    /*
    EXAMPLE:
    public DcMotor leftWheel;
     */


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LFMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LBMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RFMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RBMotor");

        // ARI COMMENT: You should define your motors for your arm here. I defined the LAMotor and RAMotor
        DcMotor LAMotor = hardwareMap.dcMotor.get("LAMotor");
        DcMotor RAMotor = hardwareMap.dcMotor.get("RAMotor");



        /*ARI COMMENT: These lines of code reverse the motors. If you run the code and the arm motors go backwards when they should
        be forward, then reverse them accordingly.
         */
        //These lines reverse the front and back motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /*ARI COMMENT: You're also going to want to do these reset lines for the arm encoders. This ensures that the encoders start
        at 0 every time you run the program
        */
        LAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //ARI COMMENT: This is all internal measuring unit stuff. If you want me to explain it lmk
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //ARI COMMENT: This is also where you would set the starting positions for your arms and servos, etc.

        //ARI COMMENT: this is where you press the start button on the robot controller. Everything after this is what runs when you're in the game
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
        while (opModeIsActive()) {// ARI COMMENT: I'm not going to remove this error because you should look at it and weep. There should only be one while opmodeisactive method
            // Assume that the motor is connected to the first port of the expansion hub
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "LAMotor");

            // "LAMotor" designates to the lower arm motor on our bot
            // Set the motor type to match the 310 rpm ftc motor
            MotorConfigurationType motorType = motor.getMotorType();
            motorType.setTicksPerRev(28);

            // The encoder counts 28 ticks per revolution
            motorType.setAchieveableMaxRPMFraction(.67);
            motor.setMotorType(motorType);
// The motor can reach its maximum rpm
            // Set the motor mode to run using encoder
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Get the gamepad triggers values
            double leftTrigger = gamepad2.left_trigger;
            double rightTrigger = gamepad2.right_trigger;

            // Calculate the desired velocity in ticks per second
            double velocity = (rightTrigger - leftTrigger) * motorType.getMaxRPM() * motorType.getTicksPerRev() / 60.0;

            // Set the motor power to the desired velocity
            // By setting these values to new Gamepad(), they will default to all
            // boolean values as false and all float values as 0
            motor.setVelocity(velocity);


        }
    }
}