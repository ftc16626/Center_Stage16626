package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.UartDevice;
import com.qualcomm.robotcore.hardware.UartDeviceConfig;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class auto {


    @Autonomous(name="HuskyLens AprilTag Reader", group="Sample")public class HuskyLensAprilTagReader extends LinearOpMode {

        private UartDevice huskyLensUart;
        private final ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize UART device
            huskyLensUart = hardwareMap.get(UartDevice.class, "huskyLensUart");
            UartDeviceConfig uartConfig = new UartDeviceConfig();
            uartConfig.baudRate = UartDeviceConfig.BaudRate.BAUD_RATE_9600;
            huskyLensUart.setConfig(uartConfig);

            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                // Read data from HuskyLens
                byte[] buffer = new byte[20];  // Adjust size based on expected data
                int read = huskyLensUart.read(buffer, buffer.length);

                if (read > 0) {
                    // Assuming the buffer now contains some data regarding the detected AprilTags
                    // Extract ID or relevant data
                    int aprilTagID = extractAprilTagID(buffer);
                    telemetry.addData("AprilTag ID", aprilTagID);

                    // For distance calculation, you'd need a method to process the tag's size or position
                    // double distance = calculateDistance(buffer);
                    // telemetry.addData("Distance", distance);
                }

                telemetry.update();
            }
        }

        // This method is a placeholder. Actual extraction depends on HuskyLens's data structure
        private int extractAprilTagID(byte[] data) {
            // Implement the extraction logic based on the HuskyLens's protocol for AprilTags
            return 0;  // Placeholder
        }

        public double calculateDistance(byte[] data) {
            // Implement distance calculation based on tag's size or position in the image
            // You'll need to know the data structure provided by HuskyLens for this calculation
            // For example, if the size of the tag is in the second byte of the data:
            if (data.length > 1) {
                double tagSize = data[1];  // Assuming tag size is in the second byte
                // Implement your distance calculation logic using tagSize
                // Example: distance = someFormula(tagSize);
                return distance;
            }
            return 0.0;  // Placeholder
    }
}

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 */

