package OldCode2020.Odometry;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class OdometryGlobalCoordinatePosition implements Runnable{

    //Thread run condition
    private boolean isRunning = true;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    private RobotInterface robotui;

    private int runCount = 0;
    private final int maxSamples = 100;
    private double[] runTimes = new double[maxSamples];
    private Acceleration[] accelerations = new Acceleration[maxSamples];
    private Acceleration previousAcceleration;
    private Position currentPosition;
    private Position previousPosition;
    private Velocity previousVelocity;

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
        for(int i = 0; i< maxSamples; i++) {
            accelerations[i] = new Acceleration(DistanceUnit.MM, 0.0, 0.0, 0.0, sleepTime);
            runTimes[i] = 0.0;
        }
        previousAcceleration = new Acceleration(DistanceUnit.MM, 0.0, 0.0, 0.0, sleepTime);
        previousVelocity = new Velocity(DistanceUnit.MM, 0.0, 0.0, 0.0, sleepTime);
        runtime = new ElapsedTime();
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        runCount++;
        if(runCount >= maxSamples) {
            runCount = 0;
        }
        runTimes[runCount] = runtime.milliseconds();
        double heading = robotui.getHeading();
        Acceleration newAcceleration = robotui.getAcceleration();
        accelerations[runCount].xAccel = newAcceleration.xAccel * Math.cos(heading) + newAcceleration.yAccel * Math.sin(heading);
        accelerations[runCount].yAccel = newAcceleration.yAccel * Math.cos(heading) + newAcceleration.xAccel * Math.sin(heading);
        accelerations[runCount].zAccel = newAcceleration.zAccel;

        int previousCount = runCount + 1;
        if(previousCount >= maxSamples) {
            previousCount = 0;
        }

        if(previousCount <= 0) {
            Double newTick = runTimes[runCount] - runTimes[previousCount];
            newAcceleration.xAccel = 0.0;
            newAcceleration.yAccel = 0.0;
            newAcceleration.zAccel = 0.0;
            for(int i = 0; i < maxSamples; i++) {
                newAcceleration.xAccel += accelerations[i].xAccel;
                newAcceleration.yAccel += accelerations[i].yAccel;
                newAcceleration.zAccel += accelerations[i].zAccel;
            }
            newAcceleration.xAccel = newAcceleration.xAccel / maxSamples;
            newAcceleration.yAccel = newAcceleration.yAccel / maxSamples;
            newAcceleration.zAccel = newAcceleration.zAccel / maxSamples;

            Velocity newVelocity = new Velocity(DistanceUnit.MM, newAcceleration.xAccel * newTick + previousVelocity.xVeloc,
                                                                 newAcceleration.yAccel * newTick + previousVelocity.yVeloc,
                                                                 newAcceleration.zAccel * newTick + previousVelocity.zVeloc,
                                                                        Double.doubleToLongBits(newTick));
            Position newDistance = new Position(DistanceUnit.MM, newVelocity.xVeloc * newTick + previousPosition.x, newVelocity.yVeloc * newTick + previousPosition.y, newVelocity.zVeloc * newTick + previousPosition.z, Double.doubleToLongBits(newTick));

            currentPosition.x = currentPosition.x + newDistance.x;
            currentPosition.y = currentPosition.y + newDistance.y;
            currentPosition.z = currentPosition.z + newDistance.z;

            previousVelocity.xVeloc = newVelocity.xVeloc;
            previousVelocity.yVeloc = newVelocity.yVeloc;
            previousVelocity.zVeloc = newVelocity.zVeloc;
        }
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
                Thread.sleep(sleepTime/2);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
