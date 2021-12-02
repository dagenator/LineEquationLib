using System;
using System.Collections.Generic;
using System.Text;

namespace LineEquationLib
{
    

    public class Frame
    {
        public Frame(float x, float y, float z, float a, float b, float c)
        {
            X = x;
            Y = y;
            Z = z;
            A = a;
            B = b;
            C = c;
        }
        public float X, Y, Z, A, B, C;
    };
    //


    //
    public class Quaternion
    {
        public float w, x, y, z;
    };
    public class Position
    {
        public float X, Y, Z; Quaternion q;
    };
    public class Sensors
    {
        public float rangefinder;
    };
    public class Axis
    {
        public float A1, A2, A3, A4, A5, A6;
    };

    public class eki_JointState
    {
        Axis Position, Velocity, Effort;
    };

    public class BaseToolData
    {
        string name;
        Frame frame;
    };

    enum MOVE_TYPE
    {
        PTP = 1,
        LIN = 2
    };

    
}
