package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp

public class straferSummer2025 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



    private void setMotorPower( final double pFL, final double pBL, final double pFR, final double pBR){
            hardwareMap.dcMotor.get("frontLeft").setPower(pFL);
            hardwareMap.dcMotor.get("backRight").setPower(pBR);
            hardwareMap.dcMotor.get("frontRight").setPower(pFR);
            hardwareMap.dcMotor.get("backLeft").setPower(pBL);
        }

//    private void setHardware() {
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//    }


    private Servo initializeServo(){
        Servo servo = hardwareMap.servo.get("claw");
        servo.setPosition(0.35);
        return(servo);
    }

    private IMU initializeIMU(){
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        return imu;
    }




    private void distanceSensorDriveBack(double tgtDistanceFromObject, DistanceSensor backRightDistance, DistanceSensor backLeftDistance) {

            double motorPower = 0;
            double i = 0;
            double previous_error = 0;
            final double k_p = 0.05; //change later
            final double k_i = 0.0025; //change later
            final double k_d = 0; //change later
            final double max_i = 0.3; //change later
            final double maxPower = 0.5;


            double current_time = runtime.time();
            double startTime = current_time;
            double prev_time = current_time;

            double delta_time = 0;


            while (current_time - startTime < 10000) {

                double leftDistance = backLeftDistance.getDistance(DistanceUnit.INCH);
                double rightDistance = backRightDistance.getDistance(DistanceUnit.INCH);

                double avgDistance = (leftDistance + rightDistance) / 2;
                double current_error = avgDistance - tgtDistanceFromObject;


                telemetry.addData("left range", leftDistance);
                telemetry.addData("right range", rightDistance);
                telemetry.addData("current_error", current_error);


                current_time = runtime.time();
                delta_time = current_time - prev_time;

                telemetry.addData("loop time",delta_time);


                double p = k_p * current_error;
                i += k_i * (current_error * (delta_time));

                if (Math.abs(current_error) > 2) {
                    i = 0;
                }
                telemetry.addData("i value", i);

                if (i > max_i) {
                    i = max_i;
                } else if (i < -max_i) {
                    i = -max_i;
                }

                double d = k_d * (current_error - previous_error) / delta_time;

                motorPower = -(p + i + d);

                if (motorPower > maxPower) {
                    motorPower = maxPower;
                } else if (motorPower < -(maxPower)) {
                    motorPower = -maxPower;
                }
                setMotorPower(motorPower, motorPower, motorPower, motorPower);

                previous_error = current_error;
                prev_time = current_time;

                telemetry.update();
            }
            setMotorPower(0,0,0,0);
        }

//        public static final double inchToTick = 537.7/(3.14159265358*3.77); for motor encoders
        public static final double inchToTick = 505; //for dead wheel


        private void encoderDriveForward(double forwardIN,DcMotor frontRight, DcMotor frontLeft, DcMotor backRight){
//            int startingTicks = (frontLeft.getCurrentPosition()+frontRight.getCurrentPosition())/2;
            int startingTicks = -backRight.getCurrentPosition();
            //537.7 ticks/rotation, wheel diameter 3.75 inches
            //(547.7 ticks/rotation)*(1 rotation/pi*3.75 inches) = 45.6 ticks/inch
            int deltaTicks = (int) (forwardIN * inchToTick);
            int tgtTicks = startingTicks+deltaTicks;

            final double k_p = 0.0004;
//            double i = 0;
//            double k_i = 0;
//            double max_i = 0;
//            double k_d = 0;
            final double maxPower = 0.7;
            double current_time = runtime.time();
//            int prevError = (startingTicks-tgtTicks)/2;
            double prevMotorPower = 1;
            double prevTime = runtime.time();
            final double allowedError = 100;
            int atTgtCounter = 0;

            while (true){
//                current_time = runtime.time();
//                double delta_time = current_time - prevTime;
//                int currentError = (frontLeft.getCurrentPosition()+frontRight.getCurrentPosition())/2-tgtTicks;
                int currentError = -backRight.getCurrentPosition()-tgtTicks;

                if (Math.abs(currentError)<allowedError){ //&& Math.abs(prevMotorPower)<0.01){
                    atTgtCounter += 1;
                }else{
                    atTgtCounter = 0;}
                if (atTgtCounter>=5){
                    telemetry.addData("target ticks", tgtTicks);
                    telemetry.addData("current ticks",-backRight.getCurrentPosition());
                    break;
                }

                if (gamepad1.b){
                    break;
                }


                double p = k_p * currentError;
//                    i += k_i * (currentError * (delta_time));
//
//                    if (Math.abs(currentError)>270){
//                        i=0;
//                    }
//                    telemetry.addData("i value", i);
//
//                    if (i > max_i) {
//                        i = max_i;
//                    } else if (i < -max_i) {
//                        i = -max_i;
//                    }
//
//                    double d = k_d * (currentError - previous_error) / delta_time;

                    double motorPower = -(p);

                    if (motorPower > maxPower) {
                        motorPower = maxPower;
                    } else if (motorPower < -(maxPower)) {
                        motorPower = -maxPower;
                    }
                    setMotorPower(motorPower, motorPower, motorPower, motorPower);

//                    prevError = currentError;
//                    prevTime = current_time;
                    prevMotorPower = motorPower;

                    telemetry.addData("absolute value current error", Math.abs(currentError));
                    telemetry.addData("prev power", prevMotorPower);
                    telemetry.addData("target ticks", tgtTicks);
                    telemetry.addData("current ticks",-backRight.getCurrentPosition());
                    telemetry.update();


//                if (currentError<-45){
//                    setMotorPower(0.3,0.3,0.3,0.3);
//                }
//                else if (currentError>45){
//                    setMotorPower(-0.3,-0.3,-0.3,-0.3);
//                }
//                else {
//                    break;
                }
            setMotorPower(0,0,0,0);
            telemetry.addData("target ticks", tgtTicks);
            telemetry.addData("current ticks",-backRight.getCurrentPosition());
            telemetry.update();
    }






        @Override
        public void runOpMode() throws InterruptedException {


            final DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            final DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");

            final DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            final DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

            final DistanceSensor backRightDistance = hardwareMap.get(DistanceSensor.class, "backRightDistance");
            final DistanceSensor backLeftDistance = hardwareMap.get(DistanceSensor.class, "backLeftDistance");

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);


//            setHardware();

            IMU imu = initializeIMU();

            Servo servo = initializeServo();

            //initialize motor encoders,
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData("Starting at",  "%7d :%7d",
                    frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition());
            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;


            final double openPos = 0.35;
            final double closePos = 0.2;

            runtime.reset();
//            double period = 100;//time in millisecond
            while (opModeIsActive()) {


//                telemetry.addData("Starting at",  "%7d :%7d",
//                        frontLeft.getCurrentPosition(),
//                        frontRight.getCurrentPosition());


                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

//                telemetry.addData("bot heading", botHeading);
//
//                telemetry.update();


                if (gamepad1.a) {
                    encoderDriveForward(48,frontRight,frontLeft,backRight);
                } else if (gamepad1.y) {
                    distanceSensorDriveBack(12,backRightDistance,backLeftDistance);
                } else if (gamepad1.options) {
                    imu.resetYaw();
                } else if (gamepad1.b) {
                    servo.setPosition(openPos);
                } else if (gamepad1.x) {
                    servo.setPosition(closePos);
                } else {
                    double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                    double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                    double rx = gamepad1.right_stick_x;


                    // Rotate the movement direction counter to the bot's rotation
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

//                    telemetry.addData("rot X", rotX);
//                    telemetry.addData("rot Y", rotY);
//                    telemetry.update();

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio,
                    // but only if at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;
                    double backLeftPower = (rotY - rotX + rx) / denominator;
                    double frontRightPower = (rotY - rotX - rx) / denominator;
                    double backRightPower = (rotY + rotX - rx) / denominator;
                    setMotorPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

                }

    //                double new_time = runtime.time();
    //                while( new_time-current_time < period){
    //                    Thread.yield();
    //                    new_time = runtime.time();
            }


    //
    //
    //                if (gamepad1.a) {
    //                    telemetry.addData("a is pressed","");
    //                    double tgtDistance = 4;
    //                    double allowedError = 1;
    //                    double motorPower = 0.3;
    //                    while (true){
    //                        double leftDistance = backLeftDistance.getDistance(DistanceUnit.INCH);
    //                        double rightDistance = backRightDistance.getDistance(DistanceUnit.INCH);
    //                        double avgDistance = (leftDistance + rightDistance)/2;
    //                        double error = avgDistance-tgtDistance;
    //                        if (error>allowedError) {
    //                            setPowerAll(-motorPower,frontLeft, frontRight, backLeft, backRight);
    //                        } else if (error<-allowedError) {
    //                            setPowerAll(motorPower,frontLeft, frontRight, backLeft, backRight);
    //                        }
    //                        else {
    //                            setPowerAll(0,frontLeft, frontRight, backLeft, backRight);
    //                            break;
    //                        }
    //
    //                    }
    //





                }
            }


