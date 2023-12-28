/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Auto;


/*I did this one for blue left position in field
You guys are going to have to tune this whole thing, but it should score your preloaded pixel.
As you make this more efficient hopefully you can go pick up more pixels and get max score
 */

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens.Block;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "testingAutonomous")

public class testingAutonomous extends LinearOpMode {
    //Variables for the code
    int armPos;

    //Drive motors
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    //Claw part of arm
    public Servo ClawR; //Rotates Claw
    public Servo ClawP;

    //Arm Motors
    public DcMotor armMotor;

    //The Stick Servo is the small stick claw in front
    public Servo Stick;
    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;

    @Override
    public void runOpMode()
    {

        //Drive motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LFMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LBMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RFMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RBMotor");


        huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens"); //HuskyLens
        Stick = hardwareMap.servo.get("Stick"); //Stick

        //Claw servos
        ClawR = hardwareMap.servo.get("ClawR"); //For rotation
        ClawR.setPosition(1);
        ClawP = hardwareMap.servo.get("ClawP"); //For pinching
        ClawP.setPosition(1);

        //Arm
        DcMotor armMotor = hardwareMap.dcMotor.get("RAMotor");

        //Reset encoders - specifies that the devices are using encoders which allow for encoder ticks
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reset encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Reverse Motors for Strafing
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);




        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }


        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.update();

        //TRAJECTORIES
        //Zone 1
        Trajectory t0 = drive.trajectoryBuilder(new Pose2d())
                .forward(2.5)
                .build();

        Trajectory t1 = drive.trajectoryBuilder(t0.end())
                .strafeLeft(3.8)  //Strafes back to beginning wall
                .build();

        Trajectory t2 = drive.trajectoryBuilder(t1.end())
                .forward(1.5) //Goes forward so it can go right and then score
                .build();

        Trajectory t3 = drive.trajectoryBuilder(t2.end())
                .strafeRight(2.5)
                .build();

        Trajectory t4 = drive.trajectoryBuilder(t3.end())
                .strafeLeft(2.5)
                .forward(2)
                .build(); //Parks robot



        //Zone 2
        //t0
        Trajectory t5 = drive.trajectoryBuilder(t4.end())
                .lineToSplineHeading(new Pose2d(0, -36, Math.toRadians(90))) //This is supposed to make it go back and turn left at the same time.
                .build();
        /*t5 might (probably) won't work so if not you can use this instead
        You will have to make a 90 degree turn left in the code before t5
        Trajectory t5 = drive.trajectoryBuilder(t4.end())
                .strafeLeft(2.5)
                .build();
                    */
        Trajectory t6 = drive.trajectoryBuilder(t5.end())
                .forward(1.5)
                .build();

        Trajectory t7 = drive.trajectoryBuilder(t6.end())
                .strafeRight(2.5)
                .build();

        Trajectory t8 = drive.trajectoryBuilder(t7.end())
                .strafeLeft(2.5)
                .forward(2)
                .build(); //Parks robot


        //ZONE 3

        Trajectory t9 = drive.trajectoryBuilder(t8.end())
                .strafeLeft(3.8)  //Strafes back to beginning wall
                .build();

        Trajectory t10 = drive.trajectoryBuilder(t9.end())
                .forward(1.5) //Goes forward so it can go right and then score
                .build();

        Trajectory t11 = drive.trajectoryBuilder(t10.end())
                .strafeRight(2.5)
                .build();

        Trajectory t12 = drive.trajectoryBuilder(t11.end())
                .strafeLeft(2.5)
                .forward(2)
                .build(); //Parks robot



        /*
        //  Zone 2
        Trajectory t0 = drive.trajectoryBuilder(new Pose2d())
                .forward(2.5)
                .build();

        Trajectory t1 = drive.trajectoryBuilder(t0.end())
                .back(2.3)
                .build();

        Trajectory t2 = drive.trajectoryBuilder(t1.end())
                .strafeLeft(8)
                .build();

        //Zone 1
        Trajectory t3 = drive.trajectoryBuilder(t2.end())
                .strafeLeft(3.8)
                .build();

        Trajectory t4 = drive.trajectoryBuilder(t3.end())
                .forward(5)
                .build();

        //Zone 3
        Trajectory t5 = drive.trajectoryBuilder(t4.end())
                .strafeRight(3.5)
                .build();

        Trajectory t6 = drive.trajectoryBuilder(t5.end())
                .back(5)
                .build();
        Trajectory t7 = drive.trajectoryBuilder(new Pose2d())
                .forward(2.7)
                .build();

         */

        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while(opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();
            
            final int AREAONE = 105;
            final int AREATWO = 210;
            final int AREATHREE = 211;
            int zone = 0;

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */

            // The following 6 lines of code took like 8 days to get working 
            
            Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            telemetry.addData("Blocks to string", blocks.toString());
            for (int i = 0; i < blocks.length; i++) {
                int blockX = blocks[i].x;
                telemetry.addData("Block X", blockX);
                
                if (blockX <= AREAONE) {
                    zone = 1;
                } else if (blockX <= AREATWO){
                    zone = 2;
                } else if (blockX >= AREATHREE){
                    zone = 3;
                }
                
                telemetry.addData("Zone", zone);
            }

            if (zone == 1) {
                Stick.setPosition(0);
                sleep(1000);
                drive.followTrajectory(t0); //forward
                drive.turn(Math.toRadians(5));
                Stick.setPosition(.8);
                sleep(1000);
                drive.followTrajectory(t1); //strafes left
                sleep(1000);
                drive.followTrajectory(t2); //goes forward
                sleep(1000000);
                drive.followTrajectory(t3); //strafes right to be in front of backdrop
                moveArm(0.5, 10000); //You're gonna have to tune this because idk how much the arm should go 💀
                //This should have scored
                moveArm(0.5, -10000); //resets arm to 0
                drive.followTrajectory(t4);
            }

            if (zone == 2) {
                Stick.setPosition(0);
                sleep(1000);
                drive.followTrajectory(t0); //moves forward
                Stick.setPosition(.8); //leaves pixel
                drive.followTrajectory(t5); //goes back and turns
                sleep(1000);
                drive.followTrajectory(t6); //goes forward
                sleep(10000);
                drive.followTrajectory(t7); //strafes right to backdrop
                sleep(1000);
                moveArm(0.5, 10000); //You're gonna have to tune this because idk how much the arm should go 💀
                //This should have scored
                moveArm(0.5, -10000); //resets arm to 0
                sleep(1000);
                drive.followTrajectory(t8); //Parks robot
            }

            if (zone == 3) {
                Stick.setPosition(0);
                sleep(1000);
                drive.followTrajectory(t0); //Goes forward
                drive.turn(Math.toRadians(-5.4)); //turns right to score in pos 3
                Stick.setPosition(.8); //scores pixel
                sleep(1000);
                drive.turn(Math.toRadians(10.8)); //turns about 180 degrees so it is facing the left not right
                drive.followTrajectory(t9); //strafes left to starting position
                sleep(10000);
                drive.followTrajectory(t10); //goes forward
                sleep(1000);
                drive.followTrajectory(t11); //strafes right to backdrop
                moveArm(0.5, 10000); //You're gonna have to tune this because idk how much the arm should go 💀
                //This should have scored
                moveArm(0.5, -10000); //resets arm to 0
                drive.followTrajectory(t12); //Parks
                sleep(1000000);
            }

            telemetry.update();
        }
    }

    private void moveArm(double speed, int armTarget){
        armPos += armTarget;

        armMotor.setTargetPosition(armPos);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(speed);

    }
}
