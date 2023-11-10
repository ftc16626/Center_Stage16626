package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
/*import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.teleop.teleop;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
 */ //These imports are from a system called RoadRunner: https://learnroadrunner.com/installing.html#method-1-downloading-the-quickstart
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;


@Autonomous (name="Autonomous")

public class AutoTutorial extends LinearOpMode {

    // position variables
    private int liftPos;
    private double bucketPos;
    private double distanceToHub;
    private double bucketDumpPos; //Hi allie

    //define dcmotors
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor arm = null;
    DcMotor liftMotor1 = null;
    DcMotor liftMotor2 = null;

    public Servo claw;

    //drivetrain variables (would have been used in drive method at the bottom)
    int leftPos;
    int rightPos;

    //arm variable
    int armPos;


    @Override
    public void runOpMode() {
        //get motors from the hardware map (in the quotations are what the hardware objects are called in the configurations part of the driver station)
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        claw = hardwareMap.servo.get("claw");
        arm = hardwareMap.dcMotor.get("arm");
        liftMotor1 = hardwareMap.dcMotor.get("liftMotor1");
        liftMotor2 = hardwareMap.dcMotor.get("liftMotor2");

        //specifies that the devices are using encoders which allow for encoder (ticks)-based autonomous rather than timebased
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Because the motors are inverted (facing the center) on the robot, you have to reverse one side in order to make all the wheels move the same direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //these lines reset the encoders to 0 so that every time you use a command that uses encoders, you start from 0 rather than updating a value
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //one of the lift motors is reversed so that the lift can move up and down
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        //this calls the sample mecanum drive from roadrunner (check the drive package to the left)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        /* The generation of the trajectories start here. You can name them whatever you want, but for
        this autonomous I made trajectories from 0 on. */


        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d(-39.0, -63.0, Math.toRadians(180.0)))
                .forward(5)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .strafeLeft(34)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(43)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeRight(35)
                .build();

        /*Trajectory traj5 = drive.trajectoryBuilder(traj3.end())
                .strafeRight(0)
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    move(0.5, -1200, 700);
                    // Run your action in here!
                })
                .build();
         */


        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(-70))))
                .forward(8)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .back(10)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .forward(24)
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .back(30)
                .build();


        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .forward(10)
                .build();

        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .back(10)
                .build();


        // Red

        Trajectory traj10 = drive.trajectoryBuilder(traj9.end())
                .forward(26)
                .build();
        Trajectory traj11 = drive.trajectoryBuilder(traj10.end())
                .strafeLeft(20)
                .build();


        // Green

        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .back(14)
                .build();

        // Blue

        Trajectory traj13 = drive.trajectoryBuilder(traj11.end())
                .strafeRight(30)
                .build();

        Trajectory traj14 = drive.trajectoryBuilder(traj11.end())
                .back(14)
                .build();




        final int width = 320;
        final int height = 240;
        boolean test = false;


        // This has to do with the computer vision. The other file in this package called SignalReader is the camera configuration portion
        SignalReader detector = new SignalReader(width);
        OpenCvCamera camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        camera.setPipeline(detector);
        //camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
        // These statements below show up on the driver station and detect the average amounts of RGB in the frame created in SignalReader
        while (!isStarted()) {
            telemetry.addData("Avg Red in View", detector.getAvgRed());
            telemetry.addData("Avg Green in View", detector.getAvgGreen());
            telemetry.addData("Avg Blue in View", detector.getAvgBlue());
            telemetry.update();
        }


        //these set the position variables to 0 at the beginning
        int leftPos = 0;
        int rightPos = 0;

        int armPos = 0;

        int liftPos = 0;

        //closes the claw on the preloaded cone before the autonomous is initialized
        claw.setPosition(0);

        waitForStart();

        if (isStopRequested()) return width;


        //RED
        if (detector.getAvgRed() > detector.getAvgBlue() && detector.getAvgRed() > detector.getAvgGreen()) {
            drive.followTrajectory(traj0);
            move(0.5, -150, 0);
            claw.setPosition(0);
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            drive.turn(Math.toRadians(-59));
            move(0.5, -1000, 800); // -1200 900 lift before and worked
            sleep(500);
            //move arms and lift
            //moves forward to pole
            drive.followTrajectory(traj4);
            move(0.5, 300, 800);
            claw.setPosition(0.3);

            // raises up arm above pole
            move(0.5, -200, 0);
            //move back
            drive.followTrajectory(traj5);

            //lift goes down
            move(0.5, 300, -750);
            //turn
            drive.turn(Math.toRadians(202)); //200


            //drops lift and arm
            move(0.5, -100, -900);
            sleep(500);
            //moves forward
            drive.followTrajectory(traj6);
            //moves into arm
            move(0.5, 400, 0);
            claw.setPosition(0);
            sleep(250);
            // moves above arm
            move(0.5, -750, 0);
            //moves backward
            drive.followTrajectory(traj7);
            move(0.5, 0, 1200);
            sleep(350);
            //turns right
            drive.turn(Math.toRadians(-182));
            //moves forward
            drive.followTrajectory(traj8);
            //release cone
            claw.setPosition(0.3);
            //moves back
            drive.followTrajectory(traj9);
            //turns
            drive.turn(Math.toRadians(184));
            claw.setPosition(0);
            move(0.5, 1000, -1200);
            sleep(350);
            //RED PORTION
            drive.followTrajectory(traj10);
            drive.followTrajectory(traj11);




        }

        //BLUE
        if (detector.getAvgBlue() > detector.getAvgRed() && detector.getAvgBlue() > detector.getAvgGreen()) {
            drive.followTrajectory(traj0);
            move(0.5, -150, 0);
            claw.setPosition(0);
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            drive.turn(Math.toRadians(-59));
            move(0.5, -1000, 800); // -1200 900 lift before and worked
            sleep(500);
            //move arms and lift
            //moves forward to pole
            drive.followTrajectory(traj4);
            move(0.5, 300, 800);
            claw.setPosition(0.3);

            // raises up arm above pole
            move(0.5, -200, 0);
            //move back
            drive.followTrajectory(traj5);

            //lift goes down
            move(0.5, 300, -750);
            //turn
            drive.turn(Math.toRadians(202)); //200


            //drops lift and arm
            move(0.5, -100, -900);
            sleep(500);
            //moves forward
            drive.followTrajectory(traj6);
            //moves into arm
            move(0.5, 400, 0);
            claw.setPosition(0);
            sleep(250);
            // moves above arm
            move(0.5, -750, 0);
            //moves backward
            drive.followTrajectory(traj7);
            move(0.5, 0, 1200);
            sleep(350);
            //turns right
            drive.turn(Math.toRadians(-182));
            //moves forward
            drive.followTrajectory(traj8);
            //release cone
            claw.setPosition(0.3);
            //moves back
            drive.followTrajectory(traj9);
            //turns
            drive.turn(Math.toRadians(60));
            claw.setPosition(0);
            move(0.5, 1000, -1200);
            // Blue Park
            drive.followTrajectory(traj13);
            drive.followTrajectory(traj14);


        }

        //GREEN

        if (detector.getAvgGreen() > detector.getAvgRed() && detector.getAvgGreen() > detector.getAvgBlue())  {
            drive.followTrajectory(traj0);
            move(0.5, -150, 0);
            claw.setPosition(0);
            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
            drive.turn(Math.toRadians(-59));
            move(0.5, -1000, 800); // -1200 900 lift before and worked
            sleep(500);
            //move arms and lift
            //moves forward to pole
            drive.followTrajectory(traj4);
            move(0.5, 300, 800);
            claw.setPosition(0.3);

            // raises up arm above pole
            move(0.5, -200, 0);
            //move back
            drive.followTrajectory(traj5);

            //lift goes down
            move(0.5, 300, -750);
            //turn
            drive.turn(Math.toRadians(202)); //200


            //drops lift and arm
            move(0.5, -100, -900);
            sleep(500);
            //moves forward
            drive.followTrajectory(traj6);
            //moves into arm
            move(0.5, 400, 0);
            claw.setPosition(0);
            sleep(250);
            // moves above arm
            move(0.5, -750, 0);
            //moves backward
            drive.followTrajectory(traj7);
            move(0.5, 0, 1200);
            sleep(350);
            //turns right
            drive.turn(Math.toRadians(-182));
            //moves forward
            drive.followTrajectory(traj8);
            //release cone
            claw.setPosition(0.3);
            //moves back
            drive.followTrajectory(traj9);
            //turns
            drive.turn(Math.toRadians(60));
            claw.setPosition(0);
            move(0.5, 1000, -1200);
            // Green park
            drive.followTrajectory(traj12);
        }


        return width;
    }


    //private void drive(int leftTarget, int rightTarget, double speed, int armTarget, int liftTarget){
    // this is a custom method that updates the speed, the arm value, and the lift values during the autonomous (see above)
    private void move(double speed, int armTarget, int liftTarget){

        //leftPos += leftTarget;
        //rightPos += rightTarget;
        armPos += armTarget;
        liftPos += liftTarget;

        /*leftFront.setTargetPosition(leftPos);
        leftBack.setTargetPosition(leftPos);
        rightFront.setTargetPosition(rightPos);
        rightBack.setTargetPosition(rightPos);
         */
        arm.setTargetPosition(armPos);
        liftMotor1.setTargetPosition(liftPos);
        liftMotor2.setTargetPosition(liftPos);




        /*leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //lift motors
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
         */
        arm.setPower(speed);
        liftMotor1.setPower(speed);
        liftMotor2.setPower(speed);
    }
}