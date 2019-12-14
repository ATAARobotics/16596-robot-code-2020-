package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="SkyStone Bot Rocks Auto 3", group="Autonomous")

public class Autonomus_3 extends LinearOpMode {
    private boolean armSwitch = false;
    private boolean clawSwitch = false;
    private boolean extenderSwitch = false;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor leftRearDrive = null;

    private DcMotor armDrive = null;
    private Servo armExtender = null;
    private Servo claw = null;

    private double maxDriveSpeed = 0.4;
    private double maxStrafeSpeed = 0.3;
    private double maxArmPower = 0.6;
    private double maxExtenderPower = 0.5;
    private double maxClawPower = 0.3;

    private int maxArmPosition = 375;
    private int minArmPosition = 0;

    private double maxExtender = 0.5;
    private double minExtender = 0.0;

    private double clawClosed = 0.0;
    private double clawOpen = 1.0;
    private boolean clawIsOpen = true;

    private int distance = 24;
    private double leftSpeed = 0.4;
    private double rightSpeed = leftSpeed;
    private double leftRearSpeed = leftSpeed;
    private double rightRearSpeed = rightSpeed;

    private RobotInterface robotui = null;

    public void runOpMode(){
        robotui = new RobotInterface(hardwareMap, telemetry,false,false,false, false);

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

      //  if(armSwitch) armDrive = hardwareMap.get(DcMotor.class, "lift");
      //  if(extenderSwitch) armExtender = hardwareMap.get(Servo.class, "extender");
      //  if(clawSwitch) claw = hardwareMap.get(Servo.class, "claw");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        rightDrive.setTargetPosition((int)(0.66*RobotInterface.INCHES_PER_ROTATION*RobotInterface.FR_TICKS_PER_INCH));
        leftDrive.setTargetPosition((int)(0.66*RobotInterface.INCHES_PER_ROTATION*RobotInterface.FL_TICKS_PER_INCH));
        rightRearDrive.setTargetPosition((int)(0.66*RobotInterface.INCHES_PER_ROTATION*RobotInterface.BR_TICKS_PER_INCH));
        leftRearDrive.setTargetPosition((int)(0.66*RobotInterface.INCHES_PER_ROTATION*RobotInterface.BL_TICKS_PER_INCH));

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Range.clip(leftSpeed,-maxDriveSpeed,maxDriveSpeed));
        rightDrive.setPower(Range.clip(rightSpeed,-maxDriveSpeed,maxDriveSpeed));
        leftRearDrive.setPower(Range.clip(leftRearSpeed,-maxDriveSpeed,maxDriveSpeed));
        rightRearDrive.setPower(Range.clip(rightRearSpeed,-maxDriveSpeed,maxDriveSpeed));

        while(opModeIsActive()) {
            telemetry.addData("Driving", "FL IPR %f TPI %f Target %f", RobotInterface.INCHES_PER_ROTATION, RobotInterface.FL_TICKS_PER_INCH, RobotInterface.INCHES_PER_ROTATION*RobotInterface.FL_TICKS_PER_INCH);
            telemetry.addData("Driving", "leftDrive from %o to %o", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Driving", "lrightDrive from %o to %o", rightDrive.getCurrentPosition(), rightDrive.getTargetPosition());
            telemetry.addData("Driving", "leftRearDrive from %o to %o", leftRearDrive.getCurrentPosition(), leftRearDrive.getTargetPosition());
            telemetry.addData("Driving", "rightRearDrive from %o to %o", rightRearDrive.getCurrentPosition(), rightRearDrive.getTargetPosition());
        //    telemetry.addData("Power levels", "leftDrive: %f rightDrive: %f", leftDrive.getPower(), rightDrive.getPower());
        //    telemetry.addData("Power levels", "leftRearDrive: %f rightRearDrive: %f", leftRearDrive.getPower(), rightRearDrive.getPower());
            telemetry.update();

        }


    }
}
