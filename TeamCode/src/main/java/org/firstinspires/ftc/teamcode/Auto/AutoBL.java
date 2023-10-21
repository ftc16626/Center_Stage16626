package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name="Autonomous BL")

public class AutoBL extends LinearOpMode {
    //drivetrain motors
    DcMotor leftFront = null;
    DcMotor leftBack = null;
    DcMotor rightFront = null;
    DcMotor rightBack = null;

    //arm motors
    DcMotor armdown = null;
    DcMotor armup = null;


    int leftPos;
    int rightPos;
    double MotorPower = 0.25;

    //@Override
    public void runOpMode() throws InterruptedException {
//get motors from the hardware map (in the quotations are what the hardware objects are
// called in the configurations part of the driver station)
        leftFront = hardwareMap.dcMotor.get("LFMotor");
        leftBack = hardwareMap.dcMotor.get("LBMotor");
        rightFront = hardwareMap.dcMotor.get("RFMotor");
        rightBack = hardwareMap.dcMotor.get("RBMotor");

/*      RAMotor = hardwareMap.get(DcMotor.class, "RAMotor");
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
*/

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        // Straffing
        leftBack.setPower(MotorPower);
        leftFront.setPower(-MotorPower);
        rightBack.setPower(-MotorPower);
        rightFront.setPower(MotorPower);
        sleep(850);
    }
}

/*      leftBack.setPower(-MotorPower);
        leftFront.setPower(-MotorPower);
        rightFront.setPower(-MotorPower);
        rightBack.setPower(-MotorPower);/
        //sleep(500);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
}
         */
