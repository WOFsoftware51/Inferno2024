package org.firstinspires.ftc.teamcode.Autons;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
@Autonomous(name = "Test Auton", group = "", preselectTeleOp = "Mecanum")
public class TestAuton extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorArmRight;
    DcMotor motorArmLeft;
    DcMotor motorElevator;
    Servo servoWrist;
    Servo servoIntake;
    IMU imu;
    YawPitchRollAngles myRobotOrientation;

    int elevatorEncoderMAX; //5500
    int elevatorEncoderMIN;
    int elevatorScore1;
    int elevatorScore2;
    int elevatorHigh;
    int elevatorFloor;
    int elevatorHP;
    int armHome;
    int armFloor;
    int armHP;
    int armScore1;
    int armHigh;
    int armMin;
    int armMax;
    double servoLow;
    double servoMid;
    double servoHigh;
    boolean score1;
    boolean score2;

    @Override
    public void runOpMode() //throws InterruptedException 
    {
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        motorBackLeft = hardwareMap.get(DcMotor.class, "backLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "backRight");
        motorArmRight = hardwareMap.get(DcMotor.class, "armRight"); //Expansion
        motorArmLeft = hardwareMap.get(DcMotor.class, "armLeft");
        motorElevator = hardwareMap.get(DcMotor.class, "elevator");
        servoWrist = hardwareMap.servo.get("wrist");
        servoIntake = hardwareMap.servo.get("intake");
        imu = hardwareMap.get(IMU.class, "imu");


        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorElevator.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double yaw = 0;
        imu.resetYaw();


        elevatorEncoderMAX = 5250; //5500
        elevatorEncoderMIN = 0;
        elevatorScore1 = 1600;
        elevatorScore2 = 1100;
        elevatorHigh = 5450;
        elevatorFloor = 1700;
        elevatorHP = 0;
        armHome = 600;
        armFloor = 2250;
        armHP = 2400;
        armScore1 = 375;
        armHigh = 200;
        armMin = 250;
        armMax = 2600;
        servoLow = 0.5;
        servoMid = 0.7;
        servoHigh = 0.85;
        score1 = false;
        score2 = false;


        motorArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorElevator.setMode(STOP_AND_RESET_ENCODER);
        motorArmLeft.setMode(STOP_AND_RESET_ENCODER);
        motorArmRight.setMode(STOP_AND_RESET_ENCODER);


        servoWrist.setPosition(servoLow);
        servoIntake.setPosition(1);


        resetOperatorEncoders();
        resetDriveEncoders();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        if (opModeIsActive()) { //24 inches = 1 block
            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -1000) { //about 24 inches/1 block
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);

            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -2200) { //about 24 inches/1 block
                drive(0.3, 0, 0);
            }
            drive(0, 0, 0);

            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -3300) { //about 24 inches/1 block
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -1050) {
                drive(0, 0, 0.3);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -450) {
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -2000) { //about 24 inches/1 block
                drive(0.3, 0, 0);
            }
            drive(0, 0, 0);
            sleep(100);
            while (motorFrontLeft.getCurrentPosition() < -200) { //about 24 inches/1 block
                drive(-0.3, 0, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -300) {
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -2000) { //about 24 inches/1 block
                drive(0.3, 0, 0);
            }  
            drive(0, 0, 0);
            sleep(100);
            while (motorFrontLeft.getCurrentPosition() < -200) { //about 24 inches/1 block
                drive(-0.3, 0, 0);
            }
            drive(0, 0, 0);
            resetDriveEncoders();

            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -250) {
                drive(0, 0.3, 0);
            }
            drive(0, 0, 0);

            sleep(100);
            while (motorFrontLeft.getCurrentPosition() > -2000) { //about 24 inches/1 block
                drive(0.3, 0, 0);
            }
            drive(0, 0, 0);
        }
    }

    public void drive(double x, double y , double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);
        telemetry.addData("Motor Front Left Encoder Pos", motorFrontLeft.getCurrentPosition());
        telemetry.update();
    }
    public void driveScore1(double x, double y , double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);
        goToPosition(elevatorScore1, motorElevator); //Elevator Score 1
        goToPosition(armScore1, motorArmLeft); //Left Arm
        goToPosition(armScore1, motorArmRight, motorArmLeft.getCurrentPosition()); //Right Arm
        telemetry.addData("Motor Front Left Encoder Pos", motorFrontLeft.getCurrentPosition());
            telemetry.update();
    }
    public void driveScore2(double x, double y , double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        servoIntake.setPosition(0);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);
        goToPosition(elevatorScore2, motorElevator); //Elevator Score 1
        servoIntake.setPosition(0);
        telemetry.addData("Motor Front Left Encoder Pos", motorFrontLeft.getCurrentPosition());
        telemetry.update();
    }
    public void driveHome(double x, double y , double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        goToPosition(armHome, motorArmLeft); //Left Arm
        goToPosition(armHome, motorArmRight, motorArmLeft.getCurrentPosition()); //Right Arm
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);
        goToPosition(elevatorEncoderMIN, motorElevator); //Elevator 0
        servoWrist.setPosition(servoMid); //Wrist Mid
        telemetry.addData("Motor Front Left Encoder Pos", motorFrontLeft.getCurrentPosition());
        telemetry.update();
    }


    public void stopMotors(){
        motorElevator.setPower(0);
        motorArmLeft.setPower(0);
        motorArmRight.setPower(0);
    }

    public void highBasket() {
        goToPosition(armHigh, motorArmLeft); //Left Arm
        goToPosition(armHigh, motorArmRight, motorArmLeft.getCurrentPosition()); //Right Arm
        goToPosition(elevatorHigh, motorElevator); //Elevator HP
        servoWrist.setPosition(servoHigh); //Servo High
    }
    public void score1() {
        goToPosition(armScore1, motorArmLeft); //Left Arm
        goToPosition(armScore1, motorArmRight, motorArmLeft.getCurrentPosition()); //Right Arm
        servoWrist.setPosition(servoMid); //Wrist Mid
        goToPosition(elevatorScore1, motorElevator); //Elevator Score 1

        if(((motorArmLeft.getCurrentPosition()+30)>armScore1) && (motorElevator.getCurrentPosition()+30)>elevatorScore1){
            score1=true;
        }
    }
    public void score2() {
        servoWrist.setPosition(servoLow); //Wrist Low
        goToPosition(elevatorScore2, motorElevator);
        if((motorElevator.getCurrentPosition()-30)<elevatorScore2){
            score2=true;
        } //Elevator Score 2
    }
    public void floorPickup() {
        goToPosition(armFloor, motorArmLeft); //Left Arm
        goToPosition(armFloor, motorArmRight, motorArmLeft.getCurrentPosition()); //Right Arm
        goToPosition(elevatorFloor, motorElevator); //Elevator Floor
        if(motorElevator.getCurrentPosition()>1365)
        {
            servoWrist.setPosition(servoLow); //Wrist Low
        }
    }
    public void home() {
        goToPosition(armHome, motorArmLeft); //Left Arm
        goToPosition(armHome, motorArmRight, motorArmLeft.getCurrentPosition()); //Right Arm
        goToPosition(elevatorEncoderMIN, motorElevator); //Elevator 0
        servoWrist.setPosition(servoMid); //Wrist Mid
    }


    public void openClaw() {
        servoIntake.setPosition(0);
    }
    public void closeClaw() {
        servoIntake.setPosition(1);
    }


    public void goToPosition(int target, DcMotor motor) {
        int encoder = motor.getCurrentPosition();
        int difference = (target - encoder);

        if(difference > 10000)
        {
            motor.setPower(1.0);
        }
        else if(difference > 500)
        {
            motor.setPower(1.0);
        }
        else if(difference > 300)
        {
            motor.setPower(0.5);
        }
        else if(difference > 200)
        {
            motor.setPower(0.25);
        }
        else if(difference > 30)
        {
            motor.setPower(0.20);
        }
        else if(difference > 25)
        {
            motor.setPower(0);
        }
        else if(difference > -25)
        {
            motor.setPower(0);
        }
        else if(difference > -30)
        {
            motor.setPower(-0.20);
        }
        else if(difference > -200)
        {
            motor.setPower(-0.25);
        }
        else if(difference > -300)
        {
            motor.setPower(-0.5);
        }
        else if(difference > -500)
        {
            motor.setPower(-1.0);
        }
        else if(difference > -10000)
        {
            motor.setPower(-1.0);
        }
    }

    public void goToPosition(int target, DcMotor motor, int encoder) {
        int difference = (target - encoder);

        if(difference > 10000)
        {
            motor.setPower(1.0);
        }
        else if(difference > 500)
        {
            motor.setPower(1.0);
        }
        else if(difference > 300)
        {
            motor.setPower(0.5);
        }
        else if(difference > 200)
        {
            motor.setPower(0.25);
        }
        else if(difference > 30)
        {
            motor.setPower(0.20);
        }
        else if(difference > 25)
        {
            motor.setPower(0);
        }
        else if(difference > -25)
        {
            motor.setPower(0);
        }
        else if(difference > -30)
        {
            motor.setPower(-0.20);
        }
        else if(difference > -200)
        {
            motor.setPower(-0.25);
        }
        else if(difference > -300)
        {
            motor.setPower(-0.5);
        }
        else if(difference > -500)
        {
            motor.setPower(-1.0);
        }
        else if(difference > -10000)
        {
            motor.setPower(-1.0);
        }
    }


    public void resetOperatorEncoders(){
        motorElevator.setMode(STOP_AND_RESET_ENCODER);
        motorArmLeft.setMode(STOP_AND_RESET_ENCODER);
        motorArmRight.setMode(STOP_AND_RESET_ENCODER);
        sleep(1);
        motorElevator.setMode(RUN_USING_ENCODER);
        motorArmLeft.setMode(RUN_USING_ENCODER);
        motorArmRight.setMode(RUN_USING_ENCODER);
    }

    public void resetDriveEncoders(){
        motorFrontLeft.setMode(STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(STOP_AND_RESET_ENCODER);
        sleep(1);
        motorFrontLeft.setMode(RUN_USING_ENCODER);
        motorFrontRight.setMode(RUN_USING_ENCODER);
        motorBackLeft.setMode(RUN_USING_ENCODER);
        motorBackRight.setMode(RUN_USING_ENCODER);
    }
}


