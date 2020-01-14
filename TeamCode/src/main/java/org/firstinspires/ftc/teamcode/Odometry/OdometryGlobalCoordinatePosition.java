package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.RobotInterface;

public class OdometryGlobalCoordinatePosition implements Runnable{

    //Thread run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    private RobotInterface robotui;

    private Position currentPosition;
    private Acceleration previousAccel;
    private Position previousPosition;
    private Velocity previousVelocity;

    private double previousTick;

    private ElapsedTime runtime;
    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param robotui the robot interface.
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePosition(RobotInterface robotui, int threadSleepDelay){
        this.robotui = robotui;
        sleepTime = threadSleepDelay;
        currentPosition = new Position(DistanceUnit.MM,0.0,0.0,0.0,sleepTime);
        previousPosition = new Position(DistanceUnit.MM,0.0,0.0,0.0,sleepTime);
        previousAccel = new Acceleration(DistanceUnit.MM, 0.0, 0.0, 0.0, sleepTime);
        previousVelocity = new Velocity(DistanceUnit.MM, 0.0, 0.0, 0.0, sleepTime);
        runtime = new ElapsedTime();
        previousTick = 0.0;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        Double newTick = runtime.milliseconds() - previousTick;
        previousTick = runtime.milliseconds();
        double heading = robotui.getHeading();
        Acceleration newAcceleration = robotui.getAcceleration();
        Velocity newVelocity = new Velocity(DistanceUnit.MM, ((newAcceleration.xAccel - previousAccel.xAccel) / newTick) * Math.cos(heading) + previousVelocity.xVeloc, ((newAcceleration.yAccel - previousAccel.yAccel) / newTick) * Math.sin(heading) + previousVelocity.yVeloc, (newAcceleration.zAccel - previousAccel.zAccel) / newTick + previousVelocity.zVeloc, Double.doubleToLongBits(newTick));
        Position newDistance = new Position(DistanceUnit.MM, newVelocity.xVeloc / newTick + previousPosition.x, newVelocity.yVeloc / newTick + previousPosition.y, newVelocity.zVeloc / newTick + previousPosition.z, Double.doubleToLongBits(newTick));

        currentPosition.x = currentPosition.x + newDistance.x;
        currentPosition.y = currentPosition.y + newDistance.y;
        currentPosition.z = currentPosition.z + newDistance.z;

        previousVelocity.xVeloc = newVelocity.xVeloc;
        previousVelocity.yVeloc = newVelocity.yVeloc;
        previousVelocity.zVeloc = newVelocity.zVeloc;

        previousAccel.xAccel = newAcceleration.xAccel;
        previousAccel.yAccel = newAcceleration.yAccel;
        previousAccel.zAccel = newAcceleration.zAccel;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return currentPosition.x; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return currentPosition.y; }

    /**
     * Returns the robot's global z coordinate
     * @return global z coordinate
     */
    public double returnZCoordinate(){ return currentPosition.z; }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXSpeed(){ return previousVelocity.xVeloc; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYSpeed(){ return previousVelocity.yVeloc; }

    /**
     * Returns the robot's global z coordinate
     * @return global z coordinate
     */
    public double returnZSpeed(){ return previousVelocity.zVeloc; }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
