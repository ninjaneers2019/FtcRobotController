package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Ninjabot
{

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public Servo claw     = null;
    public DcMotor motDrive = null;
    public DcMotor spinner = null;

    BNO055IMU gyro = null;

    static final int REV_ROBOTICS_HDHEX_MOTOR   = 28; // ticks per rotation
    static final int REV_ROBOTICS_HDHEX_20_to_1 = REV_ROBOTICS_HDHEX_MOTOR * 20;

    static final int DRIVE_MOTOR_TICK_COUNTS    = REV_ROBOTICS_HDHEX_20_to_1;
    static final double WHEEL_DIAMETER          = 4.0;

    static final int REV_ROBOTICS_COREHEX_MOTOR   = 4; // ticks per rotation
    static final int REV_ROBOTICS_COREHEX_72_to_1 = REV_ROBOTICS_COREHEX_MOTOR * 72;
    static final int LiftCounts                    = REV_ROBOTICS_COREHEX_72_to_1;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    LinearOpMode control        =  null;
    private ElapsedTime period  = new ElapsedTime();
    static final double     P_DRIVE_COEFF           = 0.0125;

    /* Constructor */
    public Ninjabot(HardwareMap map, LinearOpMode ctrl){
        init(map, ctrl);
    }
    public void init(HardwareMap ahwMap, LinearOpMode ctrl )
    {
        // Save reference to Hardware map
        hwMap   = ahwMap;
        control = ctrl;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "right");
        rightDrive = hwMap.get(DcMotor.class, "left");
        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        setDriveMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER, 0.0 );

        gyro = hwMap.get( BNO055IMU.class, "imu");
        //gyroLastAngles = new Orientation();
        //gyroGlobalAngle = 0.0;
    }


    public void setDriveTargets( int fr_dist, int fl_dist, int br_dist, int bl_dist )
    {
        leftDrive.setTargetPosition( leftDrive.getCurrentPosition() + fr_dist );
        rightDrive.setTargetPosition( rightDrive.getCurrentPosition()  + fl_dist );
    }

    public void driveTo(double distance,int dir)
    {
        int clicks = driveInchesToClicks( distance );
        switch(dir)
        {                     // displacements: f_right    f_left     b_right    b_left
            case FORWARD     : setDriveTargets( +clicks, +clicks, +clicks, +clicks); break;
            case RIGHT       : setDriveTargets( +clicks, -clicks, -clicks, +clicks); break;
            case BACKWARD    : setDriveTargets( -clicks, -clicks, -clicks, -clicks); break;
            case LEFT        : setDriveTargets( -clicks, +clicks, +clicks, -clicks); break;
            case ROTATE_LEFT : setDriveTargets( -clicks, +clicks, -clicks, +clicks); break;
            case ROTATE_RIGHT: setDriveTargets( +clicks, -clicks, +clicks, -clicks); break;
        }
    }

    public boolean targetReached()
    {   // get the average distance left to travel of all wheels (in ticks)
        int average = Math.abs(leftDrive.getTargetPosition() - leftDrive.getCurrentPosition());
        average += Math.abs(rightDrive.getTargetPosition() - rightDrive.getCurrentPosition());
        average = average / 2;
        // must be 'reached' if less than 50
        return (average < 50);
    }

    public boolean driveIsBusy()
    {
        return( leftDrive.isBusy() || rightDrive.isBusy());
    }


    public void setDriveMode( DcMotor.RunMode mode, double power )
    {
        leftDrive.setPower(power);
        leftDrive.setMode(mode);
        rightDrive.setPower(power);
        rightDrive.setMode(mode);
    }

    public void driveSetPower( double leftPower, double rightPower )
    {
        leftDrive.setPower(  leftPower );
        rightDrive.setPower( rightPower );
    }

    public int driveInchesToClicks( double dist )
    {
        double circumference   = 3.14 * WHEEL_DIAMETER;  //pi * diameter
        double rotationsNeeded = dist / circumference;
        return((int)(rotationsNeeded * DRIVE_MOTOR_TICK_COUNTS ));
    }

    public void gyroTurn(double speed, double angle){
        while (opModeIsActive() && !onHeading(speed, angle, 0.1)){
            telemetry.update();
        }
    }
}
