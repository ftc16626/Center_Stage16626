//This is the exact onbot code from Allie

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name="derivedTeleop")
public class derivedTeleop extends LinearOpMode {
    //naming different motors
    //Drive base motors
    public DcMotor RFMotor;
    public DcMotor RBMotor;
    public DcMotor LFMotor;
    public DcMotor LBMotor;
    //Arm Motors
    public DcMotor RAMotor;
    public DcMotor LAMotor;
    //Servos
    public Servo ClawR; //Rotates Claw
    public Servo Drone; //servo that releases the tension to then shoot the drone
    public Servo ClawP;

    @Override
    public void runOpMode() throws InterruptedException {
        //Servos
        ClawR = hardwareMap.servo.get("ClawR");
        ClawR.setPosition(1);
        ClawP = hardwareMap.servo.get("ClawP");
        ClawP.setPosition(1);
        Drone = hardwareMap.servo.get("Drone");

        // Declare our motors
        // Make sure your ID's match your configuration
        //Arm Motors
        DcMotor LAMotor = hardwareMap.dcMotor.get("LAMotor");
        DcMotor RAMotor = hardwareMap.dcMotor.get("RAMotor");

        //Drive Base Motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LFMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LBMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RFMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RBMotor");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encoder Stuffs
        RAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RAMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LAMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            //Driver 2
            double rightStickY = -gamepad2.right_stick_y;
            double RAMotorPower = rightStickY;
            RAMotor.setPower(RAMotorPower);

            double yValue = -gamepad2.left_stick_y; // Negate the value if the servo moves in the opposite direction
            // Map the joystick value to the servo position range (adjust min and max as needed)
            double servoPosition = yValue;
            // Set the servo position
            ClawR.setPosition(ClawR.getPosition() + (yValue / 400)); //DEREK LOOK AT THIS -ARI
            //i dont like the sounds the servo makes but it works kinda -malachi

            int rightBumperY = 1; //Adjust these as necessary. If you want it go go faster, increase it
            int leftBumperY = 1;
            //int LAMotorPower = 0;

            //This controls both arm motors at the same time. Make sure both arms are at zero or its going to be walnky.
            if(gamepad2.left_bumper == true){
                RAMotor.setPower(-1);
                LAMotor.setPower(-1);
            }
            else if(gamepad2.right_bumper == true){
                RAMotor.setPower(1);
                LAMotor.setPower(1);
            }
            if (gamepad2.left_bumper == false && gamepad2.right_bumper == false) {
                LAMotor.setPower(0);
            }

            //Rotation of claw
            //brings claw back to home
            if (gamepad2.y) {
                ClawR.setPosition(0);// Make sure you are using 'clawRotation' for controlling the claw's position
            }
            //Controls ClawP
            if (gamepad2.x) { //opens
                ClawP.setPosition(0);
            }
            if (gamepad2.b) { //closes
                ClawP.setPosition(1);
            }

            //Driver 1
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; //Correct imperf strafing
            double rx = -gamepad1.right_stick_x;

            //Drone Servo Controls
            if (gamepad1.y){
                Drone.setPosition(-.2);
            }
            if (gamepad1.b){
                Drone.setPosition(0);
            }

            //set drone arm position
            if(gamepad1.x){
                LAMotor.setTargetPosition(2488);
                LAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LAMotor.setPower(1);
            }

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


            telemetry.addLine(String.valueOf(ClawR.getPosition()));
            telemetry.addLine(String.valueOf(RAMotor.getCurrentPosition()));
            telemetry.update();
        }
    }
}