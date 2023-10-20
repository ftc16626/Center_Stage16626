package TeamCode.src.main.java.org.firstinspires.ftc.teamcode.Auto;

// public class Auto2 {


package org.firstinspires.ftc.teamcode.Auto;

        import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import static java.lang.Thread.sleep;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class auto extends LinearOpMode {
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
    int MotorPower = 1;

    //@Override
    public void runOpMode() throws InterruptedException {
//get motors from the hardware map (in the quotations are what the hardware objects are called in the configurations part of the driver station)
        leftFront = hardwareMap.dcMotor.get("LFMotor");
        leftBack = hardwareMap.dcMotor.get("LBMotor");
        rightFront = hardwareMap.dcMotor.get("RFMotor");
        rightBack = hardwareMap.dcMotor.get("RBMotor");

//armup = hardwareMap.dcMotor.get("arm1");
        // armdown = hardwareMap.dcMotor.get("arm2");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // goes forward for 3 seconds then back and than raise's the arm
        leftBack.setPower(MotorPower);
        leftFront.setPower(-MotorPower);
        rightBack.setPower(-MotorPower);
        rightFront.setPower(MotorPower);
        sleep(850);
        /*leftBack.setPower(-MotorPower);
        leftFront.setPower(-MotorPower);
        rightFront.setPower(-MotorPower);
        rightBack.setPower(-MotorPower);/
        //sleep(500);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        //armup.setPower(1);
        sleep(1000);
    }
}
