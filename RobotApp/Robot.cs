/**
 * Copyright(c) Microsoft Open Technologies, Inc. All rights reserved.
 * Licensed under the BSD 2 - Clause License.
 * 
 * Copyright (c) 2014, 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * * Neither the name of RobotApp nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * ND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 * Original Authors: Salvador Ramirez, Kyle Olive
**/
using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.Netduino;

using System.Collections;  //command stack for robot locomotion


namespace RobotApp
{
    public class Robot
    {

        // ------------- Master state machine -----------
        public RobotState RMasterState;
        // ------------- Command queue ------------------
        public  Queue RCmdQueue;  // The queue holds the commands the robot needs to follow.
        public  Object RCmdLock;
        //------------------------------------

         // ------ Detect start of match ------
        public InputPort StartButton = new InputPort(Pins.GPIO_PIN_D12, false, Port.ResistorMode.Disabled);        
        //------------------------------------

        // ------ White Line detection ------
        //Digital pin 2 - Turns the emmitter LEDs on and off.  Start with off.                      
        public OutputPort LEDIREmmitter = new OutputPort(Pins.GPIO_PIN_D2, false);
        // DigitalPin4;  ACTIVE.True= Output port; Active.False=Input port                        
        public TristatePort LeftIR = new TristatePort(Pins.GPIO_PIN_D4, true, false, ResistorModes.Disabled);
        public TristatePort RightIR = new TristatePort(Pins.GPIO_PIN_D5, true, false, ResistorModes.Disabled);

        public long StartTicks;
        public long StopTicks;
        public long TickDiff;
        public long ThresholdTick = 8000;  // Amount of ticks allowed so that a line is detected.
        public Boolean LineDetected = false;

        public long BaselineLeftIR;
        public long BaselineRightIR;

        long LeftIR01 = 0;
        long LeftIR02 = 0;
        long LeftIR03 = 0;
        long LeftIR04 = 0;
        long LeftIR05 = 0;

        long RightIR01 = 0;
        long RightIR02 = 0;
        long RightIR03 = 0;
        long RightIR04 = 0;
        long RightIR05 = 0;

        long AverageRightIR = 0;
        long AverageLeftIR = 0;

        double LineDiffThresholdPercent = .5; // Higher number makes the robot LESS sensitive to a white/black line.  If reflectance between baseline value and newly read value is more than 50% then we found the white line.
        long WLeftLineDiffThresholdValue = 0;   // The actual value that triggers robot that it has detected a white line.
        long WRightLineDiffThresholdValue = 0;   // The actual value that triggers robot that it has detected a white line.

        long BLeftLineDiffThresholdValue = 0;   // The actual value that triggers robot that it has detected a BLACK line.
        long BRightLineDiffThresholdValue = 0;   // The actual value that triggers robot that it has detected a BLACK line.


        // ----------------------------------

        // ------------------- Motor Control ----------------
        public OutputPort RightMotorDir = new OutputPort(Pins.GPIO_PIN_D7, false);   // 7 Right motor direction control
        public OutputPort LeftMotorDir = new OutputPort(Pins.GPIO_PIN_D8, false);     // 8 Left motor direction control

        PWM RightMotorPWM = new PWM(Pins.GPIO_PIN_D9);   // 9 Right motor PWM control line   .4
        PWM LeftMotorPWM = new PWM(Pins.GPIO_PIN_D10);   // 10 Left motor PWM control line   .9
        // ----------------------------------


        // ------- State machine -----------------------------------
        public enum RobotState
        {
            Start,
            WaitforDelay,
            StaySafe,
            MovingForward,
            MovingReverse,
            TurningLeft,
            TurningRight,
            Stopped
        }


        //----------------- Constructor ------------------------------
        public Robot()
        {
            // Initialize the robot in a start state
            RMasterState = RobotState.Start;

            RightMotorPWM.SetDutyCycle(0);
            LeftMotorPWM.SetDutyCycle(0);

            // Create the lock object for the queue Lock
            RCmdLock = new Object();
            //Create the command queue object
            RCmdQueue = new Queue();  // Create the Robot Command queue object.  The queue holds the commands the robot needs to follow.

            // Set initial state of robot
            RMasterState = RobotState.Start;
        }
        //----------------- Main Run function ------------------------------
        public void Run()
        {
            // Start the threads that handle robot behavior, actions, locomotion

            // Make robot wait until the user presses the reset button
            while (StartButton.Read() != false) // Wait until the button is pressed; indicated by pin LOW
              {                
              }
            Debug.Print("Start button pressed");
            Thread.Sleep(10); // sometimes pressing the button makes the robot move and ruin the calibration.  So left the robot sit so that the user is not touching the robot.
            // ----------------- Calibrate sensors ----------------
            // Take a baseline reading of the black surface.  We will use this reading as input during future calculations.
            TakeBaselineIR();

            // Start the locomotion thread  with parameters
            Thread RLocomotion = new Thread(RobotLocomotion);  //<---- This should be in the RUN method of the robot class.  The thread starts for every robot object created
            RLocomotion.Start();
        }


        // -------------------   Thread/Method that controls the Locomotion of the robot.---------------------------
        // Gets command from the cmd queue and then instructs robot to take action based on command
        public void RobotLocomotion()
        {
            string NewRCommand;

            while (true)  // Run forever
            {
                // Check for obstacles first.  Make sure robot is not in danger of running into something or falling from table
                
                if (FoundBlackLine() == true)   // Found obstacle, stop the robot before it crosses the line
                {
                    Debug.Print("Encountered obstacle. ");

                    // Take corrective action by telling robot to do the opposite movement
                    switch (RMasterState)
                    {
                        case RobotState.MovingForward:
                            MoveReverse();
                            Thread.Sleep(600);  //800                    
                            break;
                        case RobotState.MovingReverse:
                            MoveForward();
                            Thread.Sleep(600);  //800                    
                            break;
                        case RobotState.TurningLeft:
                            TurnRight();
                            Thread.Sleep(600);  //800                    
                            break;
                        case RobotState.TurningRight:
                            TurnLeft();
                            Thread.Sleep(600);  //800                    
                            break;
                    }

                    Stop();  // Leave the robot in a stopped state.
                   
                    // Flush the command queue.  The old commands probably don't make sense anymore. Force operator to enter new commands
                    this.RCmdQueue.Clear();
                    Debug.Print("Flushed queue");
                }
                   
                // Now that we know the robot is safe, check if there are outstanding commands waiting in the command queue
                
                    if (this.RCmdQueue.Count > 0)  // If there is a command waiting
                    {

                        lock (this.RCmdLock)   // Remove command from the queue
                        {
                            NewRCommand = (string) this.RCmdQueue.Dequeue();
                        }

                        Debug.Print("Processing new command: "+ NewRCommand);

                        switch (NewRCommand)
                        {
                            case "goUp":
                            case "U":
                                {
                                    // Ignore the new command if we are already performing the requested action
                                    if (RMasterState == RobotState.MovingForward)
                                    {
                                        // ignore command
                                    }
                                    else
                                    {
                                        // Tell robot to move forward                                    
                                        MoveForward();
                                    }
                                    break;
                                }
                            case "goDown":
                            case "D":
                                {
                                    // Ignore the new command if we are already performing the requested action
                                    if (RMasterState == RobotState.MovingReverse)
                                    {
                                        // ignore command
                                    }
                                    else
                                    {
                                        // Tell robot to reverse
                                        MoveReverse();
                                    } break;
                                }
                            case "goLeft":
                            case "L":
                                {
                                    // Ignore the new command if we are already performing the requested action
                                    if (RMasterState == RobotState.TurningLeft)
                                    {
                                        // ignore command
                                    }
                                    else
                                    {
                                        // Tell robot to turn left
                                        TurnLeft();
                                    }
                                    break;
                                }
                            case "goRight":
                            case "R":
                                {
                                    // Ignore the new command if we are already performing the requested action
                                    if (RMasterState == RobotState.TurningRight)
                                    {
                                        // ignore command
                                    }
                                    else
                                    {
                                        // Tell robot to turn right
                                        TurnRight();
                                    }
                                    break;
                                }
                            case "stop":
                            case "S":
                                {
                                    // Ignore the new command if we are already performing the requested action
                                    if (RMasterState == RobotState.Stopped)
                                    {
                                        // ignore command
                                    }
                                    else
                                    {
                                        // Tell robot to stop
                                        Stop();
                                    }
                                    break;
                                }
                        } // switch

                    }   // if (cmd in queue)

            }  // while (true)

        }

        public void AddCmdToQueue(string MyRCmd)
        {
            lock (this.RCmdLock)
            {
                this.RCmdQueue.Enqueue(MyRCmd);             
            }
        }

        public void MoveForward()
        {
            Debug.Print("Moving forward");
            // Update the sate of the robot
            RMasterState = RobotState.MovingForward;

            RightMotorDir.Write(true);
            LeftMotorDir.Write(true);

            RightMotorPWM.SetDutyCycle(30);
            LeftMotorPWM.SetDutyCycle(55);
        }

        public void MoveReverse()
        {
            Debug.Print("Moving reverse");
            // Update the sate of the robot
            RMasterState = RobotState.MovingReverse;

            RightMotorDir.Write(false);
            LeftMotorDir.Write(false);

            RightMotorPWM.SetDutyCycle(30);
            LeftMotorPWM.SetDutyCycle(55);
        }

        public void TurnLeft()
        {
            Debug.Print("Turning left");
            // Update the sate of the robot
            RMasterState = RobotState.TurningLeft;

            RightMotorDir.Write(true);
            LeftMotorDir.Write(false);
            RightMotorPWM.SetDutyCycle(30);
            LeftMotorPWM.SetDutyCycle(55);
            //Thread.Sleep(600);  
        }

        public void TurnRight()
        {
            Debug.Print("Turning right");
            // Update the sate of the robot
            RMasterState = RobotState.TurningRight;

            RightMotorDir.Write(false);
            LeftMotorDir.Write(true);
            RightMotorPWM.SetDutyCycle(30);
            LeftMotorPWM.SetDutyCycle(55);
            //Thread.Sleep(600); 

        }
        
        public void Stop()
        {
            Debug.Print("Stopping");
            // Update the sate of the robot
            RMasterState = RobotState.Stopped;

            RightMotorPWM.SetDutyCycle(0);
            LeftMotorPWM.SetDutyCycle(0);
            Thread.Sleep(30);
        }

        public long ReadLeftIR()
        {
            // Turn on IR LEDs (optional)
            LEDIREmmitter.Write(true);

            // Make the I/O line connected to that sensor an output and drive it high
            LeftIR.Active = true;    // Set to output pin
            LeftIR.Write(true);     // Set pin to high to charge capacitor

            // Wait several microseconds to give the 1 nF capacitor node time to reach 5 V
            Thread.Sleep(5);  // Wait about 10 milliseconds to allow capacitor to charge

            // Take starting time
            StartTicks = System.DateTime.Now.Ticks;

            // Make the I/O line an input (with internal pull-up disabled)
            LeftIR.Active = false;    // Set to input pin

            // Measure the time for the capacitor node to discharge by waiting for the I/O line to go low
            while (LeftIR.Read() == true) { }

            //StopTicks = new tick count;
            StopTicks = System.DateTime.Now.Ticks;

            // If time is less than the threshold value, then interpret result as a detection of a white line                                
            TickDiff = StopTicks - StartTicks;

            // Turn off IR LEDs (optional)
            LEDIREmmitter.Write(false);

            return (TickDiff);
        }

        public long ReadRightIR()
        {
            // Turn on IR LEDs (optional)
            LEDIREmmitter.Write(true);

            // Make the I/O line connected to that sensor an output and drive it high
            RightIR.Active = true;    // Set to output pin
            RightIR.Write(true);     // Set pin to high to charge capacitor

            // Wait several microseconds to give the 1 nF capacitor node time to reach 5 V
            Thread.Sleep(5);  // Wait about 10 milliseconds to allow capacitor to charge

            // Take starting time
            StartTicks = System.DateTime.Now.Ticks;

            // Make the I/O line an input (with internal pull-up disabled)
            RightIR.Active = false;    // Set to input pin

            // Measure the time for the capacitor node to discharge by waiting for the I/O line to go low
            while (RightIR.Read() == true) { }

            //StopTicks = new tick count;
            StopTicks = System.DateTime.Now.Ticks;

            // If time is less than the threshold value, then interpret result as a detection of a white line                                
            TickDiff = StopTicks - StartTicks;

            // Turn off IR LEDs (optional)
            LEDIREmmitter.Write(false);

            return (TickDiff);
        }


        public void TakeBaselineIR()
        {
            // Take some initial light measurements to get a sense of what the black surface looks like.  We know the match starts in the black surface, so baseline is against the black surface.

            // Read the front IR sensors 5 times each.
            LeftIR01 = ReadLeftIR();
            Thread.Sleep(5);
            LeftIR02 = ReadLeftIR();
            Thread.Sleep(5);
            LeftIR03 = ReadLeftIR();
            Thread.Sleep(5);
            LeftIR04 = ReadLeftIR();
            Thread.Sleep(5);
            LeftIR05 = ReadLeftIR();

            // Take average of the 5 readings.
            BaselineLeftIR = (LeftIR01 + LeftIR02 + LeftIR03 + LeftIR04 + LeftIR05) / 5;                  

            RightIR01 = ReadLeftIR();
            Thread.Sleep(5);
            RightIR02 = ReadLeftIR();
            Thread.Sleep(5);
            RightIR03 = ReadLeftIR();
            Thread.Sleep(5);
            RightIR04 = ReadLeftIR();
            Thread.Sleep(5);
            RightIR05 = ReadLeftIR();

            // Take average of the 5 readings.
            BaselineRightIR = (RightIR01 + RightIR02 + RightIR03 + RightIR04 + RightIR05) / 5;


            // Calculate white line thershold.  Any IR reading above this value indicates that we found the white line.
            // The IR sensors return smaller values when there is strong reflection; such as when a white line is detected.
            // Therefore, subtract some percentage from the Baseline values to determine our Threshold value.
            WLeftLineDiffThresholdValue = (long) (BaselineLeftIR - (LineDiffThresholdPercent * BaselineLeftIR));
            WRightLineDiffThresholdValue = (long)(BaselineRightIR -(LineDiffThresholdPercent * BaselineRightIR));

            BLeftLineDiffThresholdValue = (long)(BaselineLeftIR + (LineDiffThresholdPercent * BaselineLeftIR));
            BRightLineDiffThresholdValue = (long)(BaselineRightIR + (LineDiffThresholdPercent * BaselineRightIR));

            Debug.Print("BaselineLeftIR: " + BaselineLeftIR.ToString() + "  BaselineRightIR: " + BaselineRightIR.ToString());
            Debug.Print("WLeftLineDiffThresholdValue: " + WLeftLineDiffThresholdValue.ToString() + "  WRightLineDiffThresholdValue: " + WRightLineDiffThresholdValue.ToString() + "  BLeftLineDiffThresholdValue: " + BLeftLineDiffThresholdValue.ToString() + "  BRightLineDiffThresholdValue: " + BRightLineDiffThresholdValue.ToString());
            

        }

        // ----------------------------------------------------------
        public bool FoundBlackLine()
        {
            // Function based on the average of the last 5 readings. Reason is that we want to avoid any false readings that cause robot to behave incorrectly. 
            // In other words, take average of most recent 5 measurements to reduce measurement error/noise.

            // Take new measurements
            LeftIR01 = ReadLeftIR();
            RightIR01 = ReadRightIR();

            // Take the average
            AverageLeftIR = LeftIR01;
            AverageRightIR = RightIR01;

            Debug.Print("AverageLeftIR: " + AverageLeftIR.ToString() + "    AverageRightIR: " + AverageRightIR.ToString());

            // If the average is threshold percentage lower than the baseline then we have a white line
            if (AverageLeftIR > BLeftLineDiffThresholdValue)
            { // We found the white line on left sensor
                LineDetected = true;
                Debug.Print("Found black line");
            }
            else
                if (AverageRightIR > BRightLineDiffThresholdValue)
                { // We found the white line on right sensor
                    LineDetected = true;
                    Debug.Print("Found black line");
                }
                else
                {
                    LineDetected = false;
                }

            return (LineDetected);
        }
    
    }
}

