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

@TeleOp
public class Plants extends LinearOpMode {
    //naming different motors
    public DcMotor RFMotor;
    public DcMotor RBMotor;
    public DcMotor LFMotor;
    public DcMotor LBMotor;
    public DcMotor RAMotor;
    public DcMotor LAMotor;
    public DcMotor IntaMotor; //Intake motor
    public Servo ClawR; //Rotates Claw
    public CRServo ClawP; //Closes (pinches) Claw
    public Servo Flip; //Servo that flips the intake
    public boolean isRunning = false;
    public ElapsedTime runtime = new ElapsedTime();
    public double power = 0.0; // Initial power setting

    @Override
    public void runOpMode() throws InterruptedException {

        RAMotor = hardwareMap.get(DcMotor.class, "RAMotor");
        IntaMotor = hardwareMap.get(DcMotor.class, "IntaMotor");
        Flip = hardwareMap.servo.get("Flip");
        ClawP = hardwareMap.crservo.get("ClawP");
        ClawR = hardwareMap.servo.get("ClawR");

        final double CLAWHOME = 0;
        ClawP.resetDeviceConfigurationForOpMode();
        IntaMotor.setPower(0);
        final double FLIPHOME = 0.6;
        Flip.setPosition(FLIPHOME);
        //ClawR.setPosition(CLAWHOME);
        ClawR.setPosition(0);
        RAMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Declare our motors
        // Make sure your ID's match your configuration
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

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; //Correct imperf strafing
            double rx = gamepad1.right_stick_x;

            double rightStickY = gamepad2.right_stick_y;
            double RAMotorPower = rightStickY;
            RAMotor.setPower(RAMotorPower);

            // Claw rotation controlled by the left joystick
            double yValue = -gamepad2.left_stick_y; // Negate the value if the servo moves in the opposite direction
            // Map the joystick value to the servo position range (adjust min and max as needed)
            double servoPosition = yValue;
            // Set the servo position //DEREK LOOK AT THIS -Ari


            //Controls for the intake
            //the spin spin thing
            if (gamepad2.x) {
                // If the X button is pressed, set the intake motor power to a positive value (e.g., 0.65 for power)
                IntaMotor.setPower(-0.6);
            } else {
                // If the X button is not pressed, stop the intake motor
                IntaMotor.setPower(0);
            }
            // Flip
            if (gamepad2.b) {
                Flip.setPosition(FLIPHOME);  // Adjust this value to match the actual servo position
            } else if (gamepad2.a) {
                Flip.setPosition(0.1);  // Set the servo back to its home position
            }
            //Rotation of claw
            //brings claw back to home
            if (gamepad2.y) {
                ClawR.setPosition(0);// Make sure you are using 'clawRotation' for controlling the claw's position
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
            ClawR.setPosition(servoPosition); //DEREK LOOK AT THIS -ARI

        }
    }
}
