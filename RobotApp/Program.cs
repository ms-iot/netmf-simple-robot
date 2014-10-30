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

using System.IO.Ports;

namespace RobotApp
{
    public class Program
    {
        public static Robot bot;

        public static void Main()
        {
        
            //------- Some basic test code
            OutputPort led = new OutputPort(Pins.ONBOARD_LED, false);

            //------- Start Robot main thread
            bot = new Robot();
            bot.Run();
          
            while (true)  // main execution loop
            {
                //send a random command
                bot.AddCmdToQueue(generateAutonomousMove());

                //have a random delay between 0 and 1000ms
                Thread.Sleep(generateRandomSleepLength());
            }
        }  // Main()

        //Generates a random move command
        static string generateAutonomousMove()
        {
            Random rnd = new Random();
            int r = rnd.Next(5);
            switch(r)
            {
                case 1:
                    return "U"; //up
                case 2:
                    return "D"; //down
                case 3:
                    return "L"; //left
                case 4:
                    return "R"; //right
                default:
                    return "U"; //by default go forward
            }
        }

        //generates a random delay between 0-1s
        static int generateRandomSleepLength()
        {
            Random rnd = new Random();
            double r = rnd.NextDouble();
            return (int)(1000 * r);
        }

    }
}
