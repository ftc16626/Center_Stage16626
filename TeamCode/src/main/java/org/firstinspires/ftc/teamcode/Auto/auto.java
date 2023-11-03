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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.teleop.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.RobotCentricSample;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

public class auto {


    @Autonomous(name = "HuskyLens AprilTag Reader", group = "Sample")
    public class HuskyLensAprilTagReader extends LinearOpMode {

        private UartDevice huskyLensUart;
        private final ElapsedTime runtime = new ElapsedTime();

        private HuskyLens initializeHuskyLens() {
            HuskyLens huskyLens = new HuskyLens(); // Instantiate the HuskyLens class
            // You may need to provide additional parameters or configurations here
            return huskyLens;
        }

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize UART device
            huskyLensUart = hardwareMap.get(UartDevice.class, "huskyLensUart");
            UartDeviceConfig uartConfig = new UartDeviceConfig();
            uartConfig.baudRate = UartDeviceConfig.BaudRate.BAUD_RATE_9600;
            huskyLensUart.setConfig(uartConfig);
            HuskyLens huskyLens = initializeHuskyLens();
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
}
// BELOW IS GOING TO BE CODE THAT SHOULD WORK ALONGSIDE WHAT IS ABOVE, HOWEVER THIS CODE IS FOR DETERMINING
// THE TEAM PROP LOCATION
package org.firstinspires.ftc.teamcode.Auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

    public class auto extends LinearOpMode {
        public int lastResult = 0;
        boolean test = false;

        private int position = 3;


        public boolean left;
        public boolean right;
        //VisionTest vt = new VisionTest();
        //public Rect[] boundRect = new Rect[0];
        private int width;

        public SignalReader(int width) {
            this.width = width;
        }



        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(230,90);
        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 50;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */

        Point region_pointA = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x,
                REGION_TOPLEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region;
        int avgRed, avgGreen, avgBlue;

        // Volatile since accessed by OpMode thread w/o synchronization


        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */



        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region = firstFrame.submat(new Rect(region_pointA, region_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
//        inputToCb(input);
//        Mat mat = new Mat();
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avgRed = (int) Core.mean(region).val[0];
            avgGreen = (int) Core.mean(region).val[1];
            avgBlue = (int) Core.mean(region).val[2];

            Imgproc.rectangle(input, region_pointA, region_pointB, BLUE, 2);

            return input;
        }
/*
    @Override
    public Mat processFrame(Mat input) {
        /*
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) return input;
        Scalar lowHSV = new Scalar(20, 100, 100);
        Scalar highHSV = new Scalar(30, 255, 255);
        Mat thresh = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);
        thresh.release();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        edges.release();
        hierarchy.release();
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
       Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        left = false;
        right = false;
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x) left = true;
            if (boundRect[i].x + boundRect[i].width > right_x) right = true;
            //if(boundRect[i].x > 0) test = true;
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }
        lastResult = 1;
        return mat;
    }
    */



        public int getLastResult() {
            return lastResult;
        }

        public int getPosition() {
            return position;
        }

        public boolean getRight() {
            return right;
        }

        public boolean getTest() {
            return test;
        }

        public int getAvgRed(){
            return avgRed;
        }
        public int getAvgGreen(){
            return avgGreen;
        }
        public int getAvgBlue(){
            return avgBlue;
        }
    }