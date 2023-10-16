// package org.firstinspires.ftc.teamcode;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// @TeleOp
// public class Plants extends LinearOpMode {
//     //naming different motors
// private DcMotor RFMotor;
// private DcMotor RBMotor;
// private DcMotor LFMotor;
// private DcMotor LBMotor;
//
// @Override
// public void runOpMode(){
//     RFMotor = hardwareMap.get(DcMotor.class,"RFMotor");
//     RBMotor = hardwareMap.get(DcMotor.class,"RBMotor");
//     LFMotor = hardwareMap.get(DcMotor.class,"LFMotor");
//     LBMotor = hardwareMap.get(DcMotor.class,"LBMotor");
//
//     telemetry.addData("Status", "Initialized");
//         telemetry.update();
//         // Wait for the game to start (driver presses PLAY)
//         waitForStart();
//
//     telemetry.addData("Status","Running");
//     telemetry.update();
//
//     while (OpModeIsActive()) {
//         double RFtgtPower = 0;
//         RFtgtPower = gamepad1.right_stick_y;
//         RFMotor.setpower(RFtgtPower);
//         telemetry.addData("RF Target Power", RFtgtPower);
//
//         double RBtgtPower = 0;
//         RBtgtPower = gamepad1.right_stick_y;
//         RBMotor.setpower(RBtgtPower);
//         telemetry.addData("RB Target Power", RBtgtPower);
//
//         double LFtgtPower = 0;
//         LFtgtPower = gamepad1.right_stick_y;
//         LFMotor.setpower(RFtgtPower);
//         telemetry.addData("LF Target Power", LFtgtPower);
//
//         double LBtgtPower = 0;
//         LBtgtPower = gamepad1.right_stick_y;
//         LBMotor.setpower(RFtgtPower);
//         telemetry.addData("LB Target Power", LBtgtPower);
//
//     }
// }
// }
package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AliTeleOp extends LinearOpMode {
    //naming different motors
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    // private Servo   Inta;
    private CRServo Inta;
    private Servo ClawR; //Rotates Claw
    private CRServo ClawP; //Closes (pinches) Claw
    private Servo Flip;
    private boolean isRunning = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        Inta = hardwareMap.crservo.get("Inta");
        Flip = hardwareMap.servo.get("Flip");
        ClawP = hardwareMap.crservo.get("ClawP");
        ClawR = hardwareMap.servo.get("ClawR");
        Inta.resetDeviceConfigurationForOpMode();
        ClawP.resetDeviceConfigurationForOpMode();
        final double FLIPHOME = 0.1;
        final double CLAWHOME = 0;
        Flip.setPosition(FLIPHOME);
        ClawR.setPosition(CLAWHOME);
        double clawRotation = CLAWHOME;


        telemetry.addData("Status","Running");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            double leftAxis = -gamepad1.left_stick_y;
            double rightAxis = -gamepad1.right_stick_y;

            double RFtgtPower = rightAxis;
            double LFtgtPower = -leftAxis;

            LFMotor.setPower(LFtgtPower);
            RFMotor.setPower(LFtgtPower);
            LBMotor.setPower(LFtgtPower);
            RBMotor.setPower(LFtgtPower);
            ClawR.setPosition(clawRotation);


            if (gamepad1.a) {
                if (!isRunning) {
                    Inta.setPower(1); // Start the servo.
                    isRunning = true;
                    runtime.reset();
                }
            } else {
                Inta.setPower(0); // Stop the servo.
                isRunning = false;
            }

            if (gamepad2.a){
                Flip.setPosition(0.6);
            } else if (gamepad2.b){
                Flip.setPosition(FLIPHOME);
            }

            if (gamepad2.x && clawRotation <= 0.8){
                clawRotation += 0.02;
            } else if (gamepad2.y){
                clawRotation = CLAWHOME;
            }

            telemetry.update();

        /*
        RFMotor.setpower(RFtgtPower);
        telemetry.addData("RF Target Power", RFtgtPower);

        double RBtgtPower = 0;
        RBtgtPower = gamepad1.right_stick_y;
        RBMotor.setpower(RBtgtPower);
        telemetry.addData("RB Target Power", RBtgtPower);

        double LFtgtPower = 0;
        LFtgtPower = gamepad1.right_stick_y;
        LFMotor.setpower(RFtgtPower);
        telemetry.addData("LF Target Power", LFtgtPower);

        double LBtgtPower = 0;
        LBtgtPower = gamepad1.right_stick_y;
        LBMotor.setpower(RFtgtPower);
        telemetry.addData("LB Target Power", LBtgtPower);
        */

        }

    }
}
