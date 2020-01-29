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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Cluj_TeleOp", group="Iterative Opmode")
//@Disabled
public class Cluj_TeleOp extends OpMode
{
    // Declare OpMode members.
    /*
     * Code to run ONCE when the driver hits INIT
     */

    private ElapsedTime runtime = new ElapsedTime();
    Hardware_Cluj robot = new Hardware_Cluj();
    double pozitieBrat = robot.PozitieInitial;
    double pozitieMana = robot.PozitieInitial;
    final double vitezaBrat = 0.5;

    @Override
    public void init() {
        telemetry.addData("Status", "Before initialization");
        robot.init(hardwareMap);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double strafe_right = gamepad1.left_trigger;
        double strafe_left = gamepad1.right_trigger;

        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


        if (strafe_right>0)
        {
            robot.LeftBackMotor.setPower(-strafe_right);
            robot.LeftFrontMotor.setPower(strafe_right);
            robot.RightBackMotor.setPower(strafe_right);
            robot.RightFrontMotor.setPower(-strafe_right);
        }
        else
        if (strafe_left>0)
        {
            robot.LeftBackMotor.setPower(strafe_left);
            robot.LeftFrontMotor.setPower(-strafe_left);
            robot.RightBackMotor.setPower(-strafe_left);
            robot.RightFrontMotor.setPower(strafe_left);
        }
        else {
            robot.LeftBackMotor.setPower(leftPower);
            robot.LeftFrontMotor.setPower(leftPower);
            robot.RightFrontMotor.setPower(rightPower);
            robot.RightBackMotor.setPower(rightPower);
        }


        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);


        /** SLIDERE BRAT **/

        double verticalPower = 0;
        if(gamepad2.a){
            verticalPower = gamepad2.left_trigger;
        }
        else{
            if(gamepad2.b)
                verticalPower = -gamepad2.left_trigger;
        }

        robot.leftSliderMotor.setPower(-verticalPower);
        robot.rightSliderMotor.setPower(verticalPower);


        /** motoare intake **/
        double intakePower = 0;
        if(gamepad1.x){
            intakePower = 0.6;
        }
        else{
            if(gamepad1.y)
                intakePower = -0.6;
        }
        robot.leftIntakeMotor.setPower(-intakePower);
        robot.rightIntakeMotor.setPower(intakePower);

        /** servouri slider **/

        if(gamepad2.dpad_up)
            robot.servoExtindere.setPower(1);
        else
        if(gamepad2.dpad_down)
            robot.servoExtindere.setPower(-1);
        else
        if(gamepad2.right_bumper)
            robot.servoExtindere.setPower(0);

        //pozitieBrat = Range.clip(pozitieBrat, robot.pozitieMinima, robot.pozitieMaxima);
        telemetry.addData("pozitie brat", "%.2f", pozitieBrat);


        /** servouri manuta **/

        if(gamepad2.x)
            robot.servoPrindereCub.setPosition(0.5);
        else
        if (gamepad2.y)
            robot.servoPrindereCub.setPosition(-1);


        /** servouri tava**/
        if(gamepad2.dpad_left) {
            robot.servoTavaDreapta.setDirection(Servo.Direction.REVERSE);
            robot.servoTavaStanga.setDirection(Servo.Direction.FORWARD);

            robot.servoTavaStanga.setPosition(-0.25);
            robot.servoTavaStanga.setPosition(0.25);
        }
        else
        if(gamepad2.dpad_right){
            robot.servoTavaDreapta.setDirection(Servo.Direction.FORWARD);
            robot.servoTavaStanga.setDirection(Servo.Direction.FORWARD);

            robot.servoTavaStanga.setPosition(0.25);
            robot.servoTavaStanga.setPosition(-0.25);
        }
        if(gamepad2.dpad_up){
            robot.servoTavaDreapta.setDirection(Servo.Direction.REVERSE);
            robot.servoTavaStanga.setDirection(Servo.Direction.FORWARD);

            robot.servoTavaStanga.setPosition(-0.25);
            robot.servoTavaStanga.setPosition(0.25);
        }


    }



    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop(){
        robot.LeftBackMotor.setPower(0);
        robot.LeftFrontMotor.setPower(0);
        robot.RightFrontMotor.setPower(0);
        robot.RightBackMotor.setPower(0);

        // robot.rightIntakeMotor.setPower(0);
        // robot.leftIntakeMotor.setPower(0);

        // robot.rightSliderMotor.setPower(0);
        // robot.leftSliderMotor.setPower(0);
    }

}

