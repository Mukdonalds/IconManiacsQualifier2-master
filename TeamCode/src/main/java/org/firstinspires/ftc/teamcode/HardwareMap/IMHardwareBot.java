/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */


/*
CONTROL HUB

DC MOTORS

0 - backRight (back right motor)
1 - frontRight (front right motor)
2 - backLeft (back left motor)
3 - frontLeft (front left motor)

SERVOS

2 - claw (Servo on arm)
3 - claw2 (2nd Servo on arm)

*/

    /*
    EXPANSION HUB

    DC MOTORS

    0 - shooter
    1 - conveyorBelt
    2 - intake
    3 - arm

     */
public class IMHardwareBot
{
 /*
    /* Public OpMode members. */
    // DCMotors and Servos declaration
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  backLeft = null;
    public DcMotor  backRight = null;
    public DcMotor  conveyorBelt = null;
    public DcMotor  arm = null;
    public DcMotor shooter = null;
//    public Servo claw = null;
    public Servo secondClaw = null;
    public DcMotor intake = null; //name will most likely change

    // constants and variables to be used when running the code (specifically servos)
    public static final double SERVO_HOME =  0.0 ;

    // initial positions of the servos used in the program
    public static final double clawOpen = 11.5;
    public static final double clawClose = 0.0;
    public static double clawPOS = clawOpen;


    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public IMHardwareBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors\
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        conveyorBelt = hwMap.get(DcMotor.class, "conveyorBelt");
        arm = hwMap.get(DcMotor.class, "arm");
        shooter = hwMap.get(DcMotor.class, "shooter");
        intake = hwMap.get(DcMotor.class, "intake");

        // Set all motors to zero power
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        arm.setPower(0);
        conveyorBelt.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);


        // sets zeroPowerBehavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        conveyorBelt.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // sets enconder mode to run with encoder
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        conveyorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
       // claw = hwMap.get(Servo.class, "claw");
          secondClaw = hwMap.get(Servo.class, "secondClaw");

        //platform.setPosition(SERVO_HOME);
      // claw.setPosition(SERVO_HOME);
         secondClaw.setPosition(SERVO_HOME);


    }

    public void powerOff(){
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }

    public void move(double power, double distance, String direction){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double error = distance * .26;
        distance = distance + error;
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        direction.toLowerCase();
        if(direction.equals("forward")) {
            frontLeft.setTargetPosition(encoderTargetPOS);
            backLeft.setTargetPosition(encoderTargetPOS);
            frontRight.setTargetPosition(encoderTargetPOS);
            backRight.setTargetPosition(encoderTargetPOS);
        } else if (direction.equals("backward")) {
            frontLeft.setTargetPosition(-encoderTargetPOS);
            backLeft.setTargetPosition(-encoderTargetPOS);
            frontRight.setTargetPosition(-encoderTargetPOS);
            backRight.setTargetPosition(-encoderTargetPOS);
        }
        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()){

        }

        powerOff();

    }

    public void turn(double power, double distance, String direction){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double error = distance * .26;
        distance = distance + error;
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        direction = direction.toLowerCase();
        if(direction.equals("left")) {
            frontLeft.setTargetPosition(encoderTargetPOS);
            backLeft.setTargetPosition(encoderTargetPOS);
            frontRight.setTargetPosition(-encoderTargetPOS);
            backRight.setTargetPosition(-encoderTargetPOS);
        } else if (direction.equals("right")){
            frontLeft.setTargetPosition(-encoderTargetPOS);
            backLeft.setTargetPosition(-encoderTargetPOS);
            frontRight.setTargetPosition(encoderTargetPOS);
            backRight.setTargetPosition(encoderTargetPOS);
        }

        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()){
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        powerOff();

    }

    public void strafe(double power, double distance, String direction){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double error = distance * .26;
        distance = distance + error;
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        if(direction.equals("right")) {
            frontLeft.setTargetPosition(encoderTargetPOS);
            backLeft.setTargetPosition(-encoderTargetPOS);
            frontRight.setTargetPosition(-encoderTargetPOS);
            backRight.setTargetPosition(encoderTargetPOS);
        } else if(direction.equals("left")){
            frontLeft.setTargetPosition(-encoderTargetPOS);
            backLeft.setTargetPosition(encoderTargetPOS);
            frontRight.setTargetPosition(encoderTargetPOS);
            backRight.setTargetPosition(-encoderTargetPOS);
        }
        // sets the motors to the respective power that was input
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
        //the mode is set to run to the target position that was set above
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(frontLeft.isBusy() || backRight.isBusy() || frontRight.isBusy() || backLeft.isBusy()){
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        powerOff();

    }

    public void moveArm(double power, double distance, String direction){ //power from -1 to 1 and distance measured in inches
        // the motor encoders will be reset
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*3.937; //pi*diameter
        double rotationsRequired = distance/circumference; //inputed distance divided by the circumference of the mecanum wheels
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed * ticks per count on the gobilda motor
        //the encoder target position which was calculated above is input into the motors below
        direction = direction.toLowerCase();
        if (direction.equals("up")) {
            arm.setTargetPosition(encoderTargetPOS);
        } else if (direction.equals("down")){
            arm.setTargetPosition(-encoderTargetPOS);
        }
        // sets the motors to the respective power that was input
        arm.setPower(power);
        //the mode is set to run to the target position that was set above
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(arm.isBusy()){
            //wait for the motors to reach the target position
            //   telemetry.addData("Status: ","Robot is traveling " + distance + "units" + " to target position");
            //  telemetry.update();
        }

        arm.setPower(0);

    }

    public void shootRing(double power, double distance){
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // encoder reset

        double circumference = 3.14*3.937; //pi*diameter
        double rotationsRequired = distance/circumference; // distance input divided by the circumference of the wheel
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed multiplied by the ticks per revolution of the GOBILDA motor

        shooter.setTargetPosition(-encoderTargetPOS); // sets the target position to reach

        shooter.setPower(power); // sets the power of the motor accordingly to the motor

        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION); //changes the run mode of the motor

        while(shooter.isBusy()){ //waiting for the motor to reach the target position

        }

        shooter.setPower(0); // turns off the motor
    }

    public void moveConveyor(double power, double distance){
        conveyorBelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // encoder reset

        double circumference = 3.14*3.937; //pi*diameter
        double rotationsRequired = distance/circumference; // distance input divided by the circumference of the wheel
        int encoderTargetPOS = (int)(rotationsRequired*537.6); // rotations needed multiplied by the ticks per revolution of the GOBILDA motor

        conveyorBelt.setTargetPosition(-encoderTargetPOS); // sets the target position to reach

        conveyorBelt.setPower(power); // sets the power of the motor accordingly to the motor

        conveyorBelt.setMode(DcMotor.RunMode.RUN_TO_POSITION); //changes the run mode of the motor

        while(conveyorBelt.isBusy()){ //waiting for the motor to reach the target position

        }

        conveyorBelt.setPower(0); // turns off the motor
    }

    public void intake(double power, double distance) throws InterruptedException {
        intake.setPower(power);
        sleep((long) distance);
        intake.setPower(0);
    }

    public void openClaw() {
     //    claw.setPosition(11.5);
     secondClaw.setPosition(1);
    }

    public void closeClaw() {
       // claw.setPosition(0);
          secondClaw.setPosition(0);
    }


}