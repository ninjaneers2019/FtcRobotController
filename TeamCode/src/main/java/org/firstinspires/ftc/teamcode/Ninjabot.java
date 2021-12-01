package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



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

        gyro = hwMap.get( BNO055IMU.class, "imu");
        //gyroLastAngles = new Orientation();
        //gyroGlobalAngle = 0.0;
    }


    public void gyroTurn(double speed, double angle){
        while (control.opModeIsActive() && !gyroOnHeading(speed, angle, 0.1)){
            control.telemetry.update();
        }
    }

    boolean gyroOnHeading(double speed, double angle, double PCoeff)
    {
        double  error;
        double  steer;
        boolean onTarget = false;
        double  leftSpeed;
        double  rightSpeed;

        error = gyroGetError(angle);

        if (Math.abs(error) <= 1)
        {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else
        {
            steer = gyroGetSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        driveSetPower( leftSpeed, rightSpeed );

        control.telemetry.addData("Target", "%5.2f", angle);
        control.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        control.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double gyroGetError(double targetAngle)
    {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntergratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double gyroGetSteer(double error, double PCoeff)
    {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void driveSetPower( double leftPower, double rightPower )
    {
        rightDrive.setPower(rightPower);
        leftDrive.setPower(leftPower);
    }

}
