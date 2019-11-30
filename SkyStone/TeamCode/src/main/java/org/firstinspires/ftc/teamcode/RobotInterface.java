package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotInterface {


    private boolean armSwitch = false;
    private boolean clawSwitch = false;
    private boolean extenderSwitch = false;
    private boolean colorSensorSwitch = false;
    private boolean toothSwitch = false;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor leftRearDrive = null;

    private DcMotorSimple armDrive = null;
    private Servo armExtender = null;
    private Servo claw = null;
    private ColorSensor colorSensor = null;
    private AnalogInput armPotentiometer = null;
    private Servo tooth = null;

    private double maxDriveSpeed = 0.4;
    private double maxStrafeSpeed = 0.3;
    private double maxArmPower = 0.6;
    private double maxExtenderPower = 0.5;
    private double maxClawPower = 0.3;

    private final int maxArmPosition = 39;
    private final int minArmPosition = 30;


    private double maxExtender = 1.0;
    private double minExtender = 0.0;

    private double clawClosed = 0.0;
    private double clawOpen = 1.0;
    private boolean clawIsOpen = true;

    private double toothDeployed = 0.0;
    private double toothRaised = 1.0;
    private boolean toothIsDeployed = true;

    public static final double INCHES_PER_ROTATION = 9.42;
    public static final double FL_TICKS_PER_INCH = 443.0/INCHES_PER_ROTATION;
    public static final double FR_TICKS_PER_INCH = 453.0/INCHES_PER_ROTATION;
    public static final double BL_TICKS_PER_INCH = 435.0/INCHES_PER_ROTATION;
    public static final double BR_TICKS_PER_INCH = 456.0/INCHES_PER_ROTATION;
    public static final double WHEEL_BASE_CIRCUMFERENCE = 3.13 * 2.25;

    private final double FL_DRIVE_MODIFIER = 1.0;
    private final double FR_DRIVE_MODIFIER = 1.0;
    private final double RR_DRIVE_MODIFIER = 0.85;
    private final double RL_DRIVE_MODIFIER = 1.0;

    private Telemetry telemetry = null;
    private double DRIVE_ENCODER_ERROR = (FL_TICKS_PER_INCH + FR_TICKS_PER_INCH + BL_TICKS_PER_INCH + BR_TICKS_PER_INCH) / 8.0;
    public static final double ENCODER_TARGET_RATIO = 2.0/3.0;

    public double getMaxDriveSpeed() {
        return maxDriveSpeed;
    }

    public double getMaxStrafeSpeed() {
        return maxStrafeSpeed;
    }

    public double getMaxArmPower() {
        return maxArmPower;
    }

    public double getCurrentArmPower() { return armDrive.getPower(); }

    public double getMaxExtenderPower() {
        return maxExtenderPower;
    }

    public double getMaxClawPower() {
        return maxClawPower;
    }

    public boolean isClawIsOpen() {
        return clawIsOpen;
    }

    public boolean isToothDeployed() {
        return toothIsDeployed;
    }

    public double getExtenderPosition() {
        return armExtender.getPosition();
    }


    RobotInterface(HardwareMap hardwareMap, Telemetry telemetry, boolean armSwitch, boolean clawSwitch, boolean extenderSwitch, boolean toothSwitch) {
        this.armSwitch = armSwitch;
        this.clawSwitch = clawSwitch;
        this.extenderSwitch = extenderSwitch;
        this.telemetry = telemetry;
        this.toothSwitch = toothSwitch;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        if(armSwitch) {
            armPotentiometer = hardwareMap.get(AnalogInput.class,"poteniometer");
            armDrive = hardwareMap.get(DcMotorSimple.class, "lift");
        }
        if(extenderSwitch) armExtender = hardwareMap.get(Servo.class, "extender");
        if(clawSwitch) claw = hardwareMap.get(Servo.class, "claw");
        if(colorSensorSwitch) colorSensor = hardwareMap.get(ColorSensor.class, "scanner");
        if(toothSwitch) tooth =hardwareMap.get (Servo.class, "Tooth");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        if(armSwitch) {
            armDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            armDrive.setPower(0.0);
        }

        if(clawSwitch) claw.setPosition(clawOpen);
    }

    public void drive(double leftSpeed, double rightSpeed, boolean resetMode) {
        drive(leftSpeed,rightSpeed,leftSpeed,rightSpeed,resetMode);
    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive(leftSpeed,rightSpeed,true);
    }

    public void drive(double speed) {
        drive(speed,speed,true);
    }

    public void drive(double speed, boolean resetMode) {
        drive(speed,speed,resetMode);
    }

    public void drive(double leftSpeed, double rightSpeed, double leftRearSpeed, double rightRearSpeed) {
        drive(leftSpeed,rightSpeed,leftRearSpeed,rightRearSpeed,true);
    }
    public void drive(double leftSpeed, double rightSpeed, double leftRearSpeed, double rightRearSpeed, boolean resetMode) {
        if(resetMode) {
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        leftDrive.setPower(Range.clip(leftSpeed,-1 * maxDriveSpeed,maxDriveSpeed) * FL_DRIVE_MODIFIER);
        rightDrive.setPower(Range.clip(rightSpeed,-1 * maxDriveSpeed,maxDriveSpeed) * FR_DRIVE_MODIFIER);
        rightRearDrive.setPower(Range.clip(rightRearSpeed,-1 * maxDriveSpeed,maxDriveSpeed) * RR_DRIVE_MODIFIER);
        leftRearDrive.setPower(Range.clip(leftRearSpeed,-1 * maxDriveSpeed,maxDriveSpeed) * RL_DRIVE_MODIFIER);

        telemetry.addData("Driving with ", "Front left (%.2f), right (%.2f)", leftSpeed, rightSpeed);
        telemetry.addData("Driving with ", "Rear left (%.2f), right (%.2f)", leftRearSpeed, rightRearSpeed);
    }

    public void strafe(double speed) {
        drive(speed,speed,-speed,-speed);
    }

    public void drive(double leftSpeed, double rightSpeed, double distance) {
        if(distance <= 0.0) {
            leftSpeed *= -1;
            rightSpeed *= -1;
        }

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightDrive.setTargetPosition((int)(distance * FR_TICKS_PER_INCH * ENCODER_TARGET_RATIO));
        leftDrive.setTargetPosition((int)(distance * FL_TICKS_PER_INCH * ENCODER_TARGET_RATIO));
        rightRearDrive.setTargetPosition((int)(distance * BR_TICKS_PER_INCH * ENCODER_TARGET_RATIO));
        leftRearDrive.setTargetPosition((int)(distance * BL_TICKS_PER_INCH * ENCODER_TARGET_RATIO));

        drive(leftSpeed, rightSpeed,false);

        while(!AtTargetPosition(leftDrive) && !AtTargetPosition(rightRearDrive)) {
            telemetry.addData("Driving", "distance %f", distance);
            telemetry.addData("Driving", "leftDrive from %o to %o", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Driving", "leftRearDrive from %o to %o", rightRearDrive.getCurrentPosition(), rightRearDrive.getTargetPosition());
            telemetry.addData("Driving", "rightDrive from %o to %o", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Driving", "rightRearDrive from %o to %o", rightRearDrive.getCurrentPosition(), rightRearDrive.getTargetPosition());
            telemetry.addData("Power levels", "leftDrive: %f rightDrive: %f", leftDrive.getPower(), rightDrive.getPower());
            telemetry.addData("Power levels", "leftRearDrive: %f rightRearDrive: %f", leftRearDrive.getPower(), rightRearDrive.getPower());
            telemetry.update();
        }

        drive(0.0);
    }

    public void strafe(double frontSpeed, double distance) {
        if(distance < 0.0) return;
        if(frontSpeed < -maxDriveSpeed) frontSpeed = -maxDriveSpeed;
        if(frontSpeed > maxDriveSpeed) frontSpeed = maxDriveSpeed;
        double rearSpeed = -frontSpeed;

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setPower(frontSpeed*FL_DRIVE_MODIFIER);
        rightRearDrive.setPower(rearSpeed);

        rightRearDrive.setTargetPosition((int)(distance * BR_TICKS_PER_INCH * ENCODER_TARGET_RATIO));
        leftDrive.setTargetPosition((int)(distance * FL_TICKS_PER_INCH * ENCODER_TARGET_RATIO));
        leftRearDrive.setPower(rearSpeed);
        rightDrive.setPower(frontSpeed);

        while(!AtTargetPosition(leftDrive) && !AtTargetPosition(rightRearDrive)) {
            telemetry.addData("Driving", "distance %f", distance);
            telemetry.addData("Driving", "leftDrive from %o to %o", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Driving", "leftRearDrive from %o to %o", rightRearDrive.getCurrentPosition(), rightRearDrive.getTargetPosition());
            telemetry.addData("Driving", "rightDrive from %o to %o", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Driving", "rightRearDrive from %o to %o", rightRearDrive.getCurrentPosition(), rightRearDrive.getTargetPosition());
            telemetry.addData("Power levels", "leftDrive: %f rightDrive: %f", leftDrive.getPower(), rightDrive.getPower());
            telemetry.addData("Power levels", "leftRearDrive: %f rightRearDrive: %f", leftRearDrive.getPower(), rightRearDrive.getPower());
            telemetry.update();
        }

        drive(0.0,0.0);
        telemetry.addData("Finished Strafe", "Strafe Completed");
        telemetry.update();
    }

    public void turn(int angle) {
        double leftSpeed = 1.0;
        double rightSpeed = -1.0;
        if(angle < 0) {
            leftSpeed = -1.0;
            rightSpeed = 1.0;
        }

        double angleDistance = INCHES_PER_ROTATION*FL_TICKS_PER_INCH*ENCODER_TARGET_RATIO*((double)(angle))/(360.0*WHEEL_BASE_CIRCUMFERENCE);
        if(angleDistance < 0) {
            leftSpeed *= -1;
            rightSpeed *= -1;
        }
        drive(leftSpeed,rightSpeed,angleDistance);
    }

    public void extendArm() {
        extendArm(false);
    }

    public void extendArm(boolean wait) {
        if(extenderSwitch) {
            armExtender.setPosition(maxExtender);
            while (wait && armExtender.getPosition() < maxExtender) {
                // wait for extender if asked to
            }
        }
    }

    public void retractArm() {
        retractArm(false);
    }
    public void retractArm(boolean wait) {
        if(extenderSwitch) {
            armExtender.setPosition(minExtender);
            while (wait && armExtender.getPosition() > minExtender) {
                // wait for extender if asked to
            }
        }
    }

    public void stopExtender() {
        if(extenderSwitch) {
            armExtender.setPosition(0.5);
            //armExtender.setPosition(armExtender.getPosition());
        }
    }

    public void openClaw() {
        if(clawSwitch) {
            clawIsOpen = !clawIsOpen;
            if (clawIsOpen) {
                claw.setPosition(clawOpen);
            } else {
                claw.setPosition(clawClosed);
            }
        }
    }


    public void deployTooth() {
        if(toothSwitch) {
            toothIsDeployed = !toothIsDeployed;
            if (toothIsDeployed) {
                tooth.setPosition(toothDeployed);
            } else {
                tooth.setPosition(toothRaised);
            }
        }
    }

    public void liftArm() {
        if (armSwitch) {
            if (getCurrentArmPosition() > minArmPosition) {
                armDrive.setPower(maxArmPower);
            }
            else {
                stopArm();
            }
        }
    }

    public void lowerArm() {
        if (armSwitch) {
            if (getCurrentArmPosition() < maxArmPosition) {
                armDrive.setPower(-maxArmPower);
            }
            else {
                stopArm();
            }
        }
    }

    public void stopArm() {
        if (armSwitch) {
            armDrive.setPower(0.0);
        }
    }

    private boolean AtTargetPosition(DcMotor drive) {
        if((drive.getCurrentPosition() < drive.getTargetPosition() + DRIVE_ENCODER_ERROR) && (drive.getCurrentPosition() > drive.getTargetPosition() - DRIVE_ENCODER_ERROR)) return true;
        return false;
    }

    public int getCurrentArmPosition () {
        double maxResult = armPotentiometer.getMaxVoltage();
        double result = armPotentiometer.getVoltage();
        return (int)(100*result / maxResult);
    }
}
