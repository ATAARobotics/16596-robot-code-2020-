package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

import static java.lang.Thread.sleep;

public class RobotInterface {

    private ElapsedTime runtime = new ElapsedTime();

    private boolean armSwitch = false;
    private boolean clawSwitch = false;
    private boolean extenderSwitch = false;
    private boolean colorSensorSwitch = true;
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
    private DistanceSensor distanceSensor = null;
    private Servo park = null;
    private Servo pusher = null;

    public BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double setAngle = 0.0;
    private double globalAngle;
    private double correction;

    private double maxDriveSpeed = 0.5;
    private double maxStrafeSpeed = 0.4;
    private double maxArmPower = 0.262;
    private double maxExtenderPower = 1.0;
    private double maxClawPower = 0.3;

    public final int MAX_ARM_POSITION = 80;
    public final int MIN_ARM_POSITION = 50;
    public final int BRIDGE_ARM_POSITION = 65;
    public final int BLOCK_ARM_POSITION = 68;

    private double autoMaxExtender = 0.4;
    private double maxExtender = 1.0;
    private double minExtender = 0.0;

    private double clawClosed = 1.0;
    private double clawOpen = 0.0;
    private boolean clawIsOpen = false;

    private double toothDeployed = 0.0;
    private double toothRaised = 1.0;
    private boolean toothIsDeployed = true;

    public static final double INCHES_PER_ROTATION = 9.42;
    public static final double FL_TICKS_PER_INCH = 443.0/INCHES_PER_ROTATION;
    public static final double FR_TICKS_PER_INCH = 453.0/INCHES_PER_ROTATION;
    public static final double BL_TICKS_PER_INCH = 435.0/INCHES_PER_ROTATION;
    public static final double BR_TICKS_PER_INCH = 456.0/INCHES_PER_ROTATION;
    public static final double WHEEL_BASE_CIRCUMFERENCE = 3.13 * 2.25;

    private final int RED_MAXIMUM = 750;
    private final int GREEN_MAXIMUM = 1000;

    private final int MAX_DISTANCE = 50;
    private final int MIN_DISTANCE = 20;

    private final double FL_DRIVE_MODIFIER = 0.939;
    private final double FR_DRIVE_MODIFIER = 0.953;
    private final double RR_DRIVE_MODIFIER = 0.965;
    private final double RL_DRIVE_MODIFIER = 1.0;
    private final int LINE_RED = 220;
    private final int LINE_BLUE = 220;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

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
        if(clawSwitch) {
            claw = hardwareMap.get(Servo.class, "claw");
        }
        if(colorSensorSwitch) {
            distanceSensor = hardwareMap.get(DistanceSensor.class, "rangefinder");
            colorSensor = hardwareMap.get(ColorSensor.class, "scanner");
        }
        if(toothSwitch) tooth = hardwareMap.get (Servo.class, "Tooth");

        park = hardwareMap.get(Servo.class, "parking");
        pusher = hardwareMap.get(Servo.class, "pushing");

        // Set the park and pusher position so it doesn't start automatically.
        park.setPosition(0.5);
        pusher.setPosition(0.5);

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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 50);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        if(clawSwitch) claw.setPosition(clawClosed);
    }

    public void drive(double leftSpeed, double rightSpeed, boolean resetMode) {
        drive(leftSpeed,rightSpeed,leftSpeed,rightSpeed,resetMode);
    }

    public void drive(double leftSpeed, double rightSpeed) {
        drive(leftSpeed,rightSpeed,true);
    }

    public void driveWithCorrection(double speed) {
        correction = checkDirection();

        drive(speed-correction, speed+correction,false);

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
        if(leftSpeed < 0.0) leftSpeed *= -1;
        if(rightSpeed < 0.0) rightSpeed *= -1;

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
            correction = checkDirection();

            drive(leftSpeed-correction, rightSpeed+correction,false);

            telemetry.addData("IMU", "imu heading: %.2f, globalHeading %.2f, correction %.2f ",lastAngles.firstAngle);
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 global heading", globalAngle);
//            telemetry.addData("3 correction", correction);

            telemetry.addData("Driving", "distance %f inches", distance);
            telemetry.addData("Driving", "leftDrive from %d to %d", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Driving", "leftRearDrive from %d to %d", rightRearDrive.getCurrentPosition(), rightRearDrive.getTargetPosition());
            telemetry.addData("Driving", "rightDrive from %d to %d", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Driving", "rightRearDrive from %d to %d", rightRearDrive.getCurrentPosition(), rightRearDrive.getTargetPosition());
            telemetry.addData("Power levels", "leftDrive: %.2f rightDrive: %.2f", leftDrive.getPower(), rightDrive.getPower());
            telemetry.addData("Power levels", "leftRearDrive: %.2f rightRearDrive: %.2f", leftRearDrive.getPower(), rightRearDrive.getPower());
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
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setPower(frontSpeed*FL_DRIVE_MODIFIER);
        rightRearDrive.setPower(rearSpeed*RR_DRIVE_MODIFIER);
        leftRearDrive.setPower(rearSpeed*RL_DRIVE_MODIFIER);
        rightDrive.setPower(frontSpeed*FR_DRIVE_MODIFIER);

        rightRearDrive.setTargetPosition((int)(distance * BR_TICKS_PER_INCH * ENCODER_TARGET_RATIO));
        leftDrive.setTargetPosition((int)(distance * FL_TICKS_PER_INCH * ENCODER_TARGET_RATIO));
        leftRearDrive.setTargetPosition((int)(distance * BR_TICKS_PER_INCH * ENCODER_TARGET_RATIO));
        rightDrive.setTargetPosition((int)(distance * FL_TICKS_PER_INCH * ENCODER_TARGET_RATIO));

        while(!AtTargetPosition(leftDrive) && !AtTargetPosition(rightRearDrive)) {
            correction = checkDirection();
            if(frontSpeed < 0.0) correction *= -1.0;
            leftDrive.setPower((frontSpeed-correction)*FL_DRIVE_MODIFIER);
            rightRearDrive.setPower((rearSpeed+correction)*RR_DRIVE_MODIFIER);
            leftRearDrive.setPower((rearSpeed+correction)*RL_DRIVE_MODIFIER);
            rightDrive.setPower((frontSpeed-correction)*FR_DRIVE_MODIFIER);

            telemetry.addData("IMU", "imu heading: %.2f, globalHeading %.2f, correction %.2f ",lastAngles.firstAngle);
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 global heading", globalAngle);
//            telemetry.addData("3 correction", correction);

            telemetry.addData("Driving", "distance %f inches", distance);
            telemetry.addData("Power levels", "leftDrive: %.2f rightDrive: %.2f", leftDrive.getPower(), rightDrive.getPower());
            telemetry.addData("Power levels", "leftRearDrive: %.2f rightRearDrive: %.2f", leftRearDrive.getPower(), rightRearDrive.getPower());
            telemetry.addData("Driving", "leftDrive from %d to %d", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Driving", "leftRearDrive from %d to %d", rightRearDrive.getCurrentPosition(), rightRearDrive.getTargetPosition());
            telemetry.addData("Driving", "rightDrive from %d to %d", leftDrive.getCurrentPosition(), leftDrive.getTargetPosition());
            telemetry.addData("Driving", "rightRearDrive from %d to %d", rightRearDrive.getCurrentPosition(), rightRearDrive.getTargetPosition());
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
            leftSpeed *= -1.0;
            rightSpeed *= -1.0;
        }

        setAngle = setAngle + angle;
        while((getAngle() <= setAngle - 5) && (getAngle() >= setAngle + 5)) {
            telemetry.addData("Turn", "%d degrees", angle);
            telemetry.addData("IMU", "imu heading: %.2f, globalHeading %.2f, correction %.2f ",lastAngles.firstAngle);
            drive(leftSpeed, rightSpeed);
            telemetry.update();
        }

        drive(0.0);
        telemetry.addData("Turn complete","Finished!");
        telemetry.update();
    }

    public void extendArm() throws InterruptedException {
        extendArm(false);
    }

    public void retractArm(boolean wait) throws InterruptedException {
        if(extenderSwitch) {
            armExtender.setPosition(maxExtender);
            while (wait && armExtender.getPosition() < autoMaxExtender) {
                sleep(500);
            }
            if(wait) armExtender.setPosition(armExtender.getPosition());
        }
    }

    public void retractArm() throws InterruptedException {
        retractArm(false);
    }

    public void extendArm(boolean wait) throws InterruptedException {
        if(extenderSwitch) {
            armExtender.setPosition(minExtender);
            while (wait && armExtender.getPosition() > minExtender) {
                sleep(500);
            }
            if(wait) armExtender.setPosition(armExtender.getPosition());
        }
    }

    public void stopExtender() {
        if(extenderSwitch) {
            armExtender.setPosition(0.5);
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

    public void liftArm() throws InterruptedException {
        liftArm(false);
    }

    public void liftArm(boolean wait) throws InterruptedException {
        if (armSwitch) {
            if (getCurrentArmPosition() > MIN_ARM_POSITION) {
                armDrive.setPower(maxArmPower);
                while(wait && getCurrentArmPosition() > BRIDGE_ARM_POSITION) {
                    sleep(500);
                }
                if(wait) stopArm();
            }
            else {
                stopArm();
            }
        }
    }

    public void lowerArm() throws InterruptedException {
        lowerArm(0);
    }
    public void lowerArm(int targetPosition) throws InterruptedException {
        if (armSwitch) {
            if (getCurrentArmPosition() < MAX_ARM_POSITION) {
                armDrive.setPower(-maxArmPower);
                while(targetPosition == 1 && getCurrentArmPosition() < BRIDGE_ARM_POSITION) {
                    sleep(500);
                }
                while(targetPosition == 2 && getCurrentArmPosition() < BLOCK_ARM_POSITION) {
                    sleep(500);
                }
                if(targetPosition > 0) stopArm();
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

    public void sensorStart(){
        colorSensor.enableLed(true);
    }
    public void sensorStop(){
        colorSensor.enableLed(false);
    }

    public boolean skyStoneFound(){
        while(getDistance() < MIN_DISTANCE) {
            drive(0.5);
        }
        while(getDistance() > MAX_DISTANCE) {
            drive(-0.5);
        }
        drive(0.0);
        if ( (colorSensor.red() < RED_MAXIMUM) && (colorSensor.green() < GREEN_MAXIMUM)) {
            return (true);
        }
        return (false);
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.MM);
    }

    public ColorSensor getColorSensor() { return colorSensor; }

    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getEncoderDistance() {
        double distance = 0.0;
        distance = (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition() + leftRearDrive.getCurrentPosition() + rightRearDrive.getCurrentPosition()) / 4 ;
        return distance;
    }

    public void driveTest(double distance) {
        double leftTime = 0.0;
        double rightTime = 0.0;
        double leftRearTime = 0.0;
        double rightRearTime = 0.0;

        runtime.reset();

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

        drive(1.0, 1.0,false);

        while(!AtTargetPosition(leftDrive) || !AtTargetPosition(rightRearDrive) || !AtTargetPosition(rightDrive) || !AtTargetPosition(leftRearDrive)) {
            if(AtTargetPosition(leftRearDrive)) {
                leftRearTime = runtime.time();
                leftRearDrive.setPower(0.0);
            }
            if(AtTargetPosition(leftDrive)) {
                leftTime = runtime.time();
                leftDrive.setPower(0.0);
            }
            if(AtTargetPosition(rightRearDrive)) {
                rightRearTime = runtime.time();
                rightRearDrive.setPower(0.0);
            }
            if(AtTargetPosition(rightDrive)) {
                rightTime = runtime.time();
                rightDrive.setPower(0.0);
            }
            telemetry.addData("Run time", "%f", runtime.time());
            telemetry.addData("Encoders", "Left Front %o, Right Front %o, Left Back %o, Right Back %o", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), leftRearDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Run Times", "Left Front %f Right Front %f Left Rear %f Right Rear %f", leftTime,rightTime,leftRearTime,rightRearTime);
        telemetry.update();
    }

    double getClawPosition() {
        return claw.getPosition();
    }

    void strafeToLine(double speed) {
        while (!lineDetected()) {
            strafe(speed);
        }
        drive(0.0);
    }

    boolean lineDetected() {
        telemetry.addData("RED", "Seen %d   Target %d", colorSensor.red(), LINE_RED);
        telemetry.update();
        if(colorSensor.red() > LINE_RED || colorSensor.blue() > LINE_BLUE) return true;
        else return false;
    }

    boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    String getCalibrationStatus() {
        return imu.getCalibrationStatus().toString();
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == setAngle)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;


        //return correction;
        return Range.clip(correction,-0.2,0.2);
    }

    public void driveToBlock(double speed) {
        while (distanceSensor.getDistance(DistanceUnit.MM) < MIN_DISTANCE) {
            driveWithCorrection(-speed);
            telemetry.addData("Sensor", "Distance %f", distanceSensor.getDistance((DistanceUnit.CM)));
            telemetry.addData("Driving", "");
            telemetry.update();
        }
        while (distanceSensor.getDistance(DistanceUnit.MM) > MAX_DISTANCE) {
            driveWithCorrection(speed);
            telemetry.addData("Sensor", "Distance %f", distanceSensor.getDistance((DistanceUnit.CM)));
            telemetry.addData("Driving", "");
            telemetry.update();
        }
        drive(0.0);
    }

    public double getHeading() {
        return getAngle();
    }

    public Acceleration getAcceleration() {
        return imu.getLinearAcceleration();
    }

    public Velocity getVelocity() {
        return imu.getVelocity();
    }

    public Position getPosition() {
        return imu.getPosition();
    }

    public void parker() {
        park.setPosition(1);
    }
    public void blockpush(){pusher.setPosition(1);}
    public void unblockpush(){pusher.setPosition(0);}
}