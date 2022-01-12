package org.firstinspires.ftc.teamcode.techserpents.Autonomous.NewCompAutos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.techserpents.Autonomous.NewContourPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

// If you don't want to install dashboard you can delete @Config
@Config
@Autonomous(name="Fixed Red - Storage Side", group="TS")

public class NewRedStorage extends LinearOpMode {

    // Declaring camera variables
    private OpenCvCamera webcam;
    private NewContourPipeline pipeline;

    //Threshold variables - This is for hot pink
    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    // Variables for the rectangles that appear in our cmaera feed
    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

    // More variables
    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Declaring base motors
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    // Declaring subsystem motors
    DcMotorEx carouselMotor;
    DcMotorEx intakeMotor;
    DcMotorEx lift;

    // Declaring servos
    Servo pivotServo;


    // Autonomous robot variables
    Double width = 16.0; //inches
    Integer cpr = 538; //counts per rotation
    Integer gearratio = 1;
    Double diameter = 3.77953;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 1.0;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    Double conversion = cpi * bias;
    Boolean exit = false;

    // IMU variables
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    // Our three position booleans to know where the team element is located
    public boolean leftPos = false;
    public boolean midPos = false;
    public boolean rightPos = false;


    // Pressing the "Initialize" button
    @Override
    public void runOpMode()
    {
        // Initializing the gyroscope
        initGyro();

        // Base motor hardware map
        frontleft = hardwareMap.dcMotor.get("leftFront");
        frontright = hardwareMap.dcMotor.get("rightFront");
        backleft = hardwareMap.dcMotor.get("leftRear");
        backright = hardwareMap.dcMotor.get("rightRear");

        // Setting the zero power behavior for base motors; test FLOAT and BRAKE to see which one works better for you
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reversing the two right base motors
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializing carousel motor
        carouselMotor = hardwareMap.get(DcMotorEx.class, "cMotor");
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initializing the lift
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initializing intake motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializing pivot servo
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        pivotServo.setDirection(Servo.Direction.REVERSE);

        // Initializing OpenCV Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        // Initializing the pipeline
        pipeline = new NewContourPipeline(0.2, 0.2, 0.2, 0.2);
        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Don't touch this
        sleep(1000);

        if(pipeline.error){
            telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
        }

        // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
        // testing(pipeline);
        // Watch the YouTube Tutorial for the better explanation

        // Prints the are of the rectangle of the color you want
        double rectangleArea = pipeline.getRectArea();
        telemetry.addData("Rectangle Area", rectangleArea);

        // Looping when you press initialize but before you press start
        while (!opModeIsActive() && !isStopRequested()) {
            // Detects if there is enough color for it to matter
            if(rectangleArea > minRectangleArea){
                //Then check the location of the rectangle to see which barcode it is in.
                // If the rectangle is to the right of the middle, then it is in the RIGHT position
                if(pipeline.getRectMidpointX() > 400){
                    rightPos = true;
                    leftPos = false;
                    midPos = false;
                    telemetry.addData("AUTO:", "RIGHT");
                    telemetry.addData("LEFT:", leftPos);
                    telemetry.addData("MID: ", midPos);
                    telemetry.addData("RIGHT: ", rightPos);
                    telemetry.update();
                }
                // If the rectangle is to the left to the middle, then it is in the LEFT position.
                else if(pipeline.getRectMidpointX() < 200){
                    leftPos = true;
                    midPos = false;
                    rightPos = false;
                    telemetry.addLine("AUTO: LEFT");
                    telemetry.addData("LEFT:", leftPos);
                    telemetry.addData("MID: ", midPos);
                    telemetry.addData("RIGHT: ", rightPos);
                    telemetry.update();
                }
                // If none of the above, then it is in the MIDDLE position.
                else {
                    midPos = true;
                    rightPos = false;
                    leftPos = false;
                    telemetry.addData("AUTO:", "MIDDLE");
                    telemetry.addData("LEFT:", leftPos);
                    telemetry.addData("MID: ", midPos);
                    telemetry.addData("RIGHT: ", rightPos);
                    telemetry.update();
                }
            }
        }

        // Waiting until START is pressed
        waitForStart();

        // Code to run if its in the RIGHT position.
        if(rightPos){
            intakeMotor.setPower(0.5);
            moveToPosition(40, 0.5);
            moveToPosition(7, 0.5);
            intakeMotor.setPower(0);
            moveToPosition(-4, 0.5);
            turnWithGyro(80, -0.5);
            moveToPosition(4, 0.5);
            sleep(500);
            lift.setPower(0.8);
            sleep(800);
            lift.setPower(0);
            sleep(500);
            pivotServo.setPosition(0.1);
            sleep(500);
            moveToPosition(-10, 0.5);
            sleep(300);
            moveToPosition(5, 0.5);
            sleep(1000);
            lift.setPower(-0.8);
            sleep(700);
            lift.setPower(0);
            sleep(500);
            strafeToPosition(-7, 0.5);
            moveToPosition(22, 0.5);
            sleep(500);
            pivotServo.setPosition(0.85);
            sleep(500);
            turnWithGyro(165, 0.5);
            strafeToPosition(42, 0.3);
            sleep(500);
            carouselMotor.setPower(-0.5);
            sleep(4000);
            carouselMotor.setPower(0);
            sleep(1000);
            strafeToPosition(-23, 0.5);
            moveToPosition(-5, 0.3);
         //   moveToPosition(60, 0.5);
           // strafeToPosition(-12, 0.5);
            //moveToPosition(50, 0.5);
        }
        // Code to run if its in the MIDDLE position.
        else if(midPos){
            intakeMotor.setPower(0.5);
            moveToPosition(40, 0.5);
            moveToPosition(7, 0.5);
            intakeMotor.setPower(0);
            moveToPosition(-7, 0.5);
            turnWithGyro(80, -0.5);
            moveToPosition(4, 0.5);
            sleep(500);
            lift.setPower(0.8);
            sleep(400);
            lift.setPower(0);
            sleep(500);
            pivotServo.setPosition(0.1);
            sleep(500);
            moveToPosition(-8, 0.5);
            sleep(300);
            moveToPosition(5, 0.5);
            sleep(1000);
            lift.setPower(-0.8);
            sleep(300);
            lift.setPower(0);
            sleep(500);
            strafeToPosition(-7, 0.5);
            moveToPosition(22, 0.5);
            sleep(500);
            pivotServo.setPosition(0.85);
            sleep(500);
            turnWithGyro(165, 0.5);
            strafeToPosition(38, 0.3);
            sleep(500);
            carouselMotor.setPower(-0.5);
            sleep(4000);
            carouselMotor.setPower(0);
            sleep(1000);
            strafeToPosition(-23, 0.5);
            moveToPosition(-5, 0.3);
        //    moveToPosition(60, 0.5);
           // strafeToPosition(-12, 0.5);
          //  moveToPosition(50, 0.5);
        }
        // Code to run if its in the LEFT position.
        else if (leftPos){
            moveToPosition(40, 0.5);
            turnWithGyro(80, -0.5);
            moveToPosition(4, 0.5);
            sleep(500);
            pivotServo.setPosition(0.1);
            sleep(500);
            moveToPosition(-6, 0.5);
            sleep(1000);
            moveToPosition(23, 0.5);
            sleep(500);
            pivotServo.setPosition(0.85);
            sleep(500);
            turnWithGyro(165, 0.5);
            strafeToPosition(48, 0.3);
            sleep(500);
            carouselMotor.setPower(-0.5);
            sleep(4000);
            carouselMotor.setPower(0);
            sleep(1000);
            strafeToPosition(-23, 0.5);
          //  moveToPosition(60, 0.5);
            //strafeToPosition(-12, 0.5);
            //moveToPosition(50, 0.5);
        }

    }

    // Pipeline method
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }

    // Autonomous methods
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }
    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */

    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        //</editor-fold>
        //
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */

    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }
    //
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */

    public void waitForStartify(){
        waitForStart();
    }
    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */

    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */

    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */

    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }
}



