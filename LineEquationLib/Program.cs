using Emgu.CV;
using System;
using System.Drawing;

namespace LineEquationLib
{
    class Program
    {
       
        static void Main(string[] args)
        {
            XMLReader xmlReader = new XMLReader();
            var prop = xmlReader.GetCameraProperties("resourses\\camera_properties.xml");
            var res =CameraMath.CalculatePixelLineFromCameraProperties(prop, new Point(234, 578));
            Console.WriteLine(res);
            
            //Matrix<double> cameraMatrix = new Matrix<double>(3,3);
            //cameraMatrix.Data = new[,]
            //{
            //    {
            //         5180.25, 0, 1796.39
            //    },
            //    {
            //        0, 5212.67, 952.68
            //     },
            //    {
            //         0, 0, 1
            //    }
            // };

            //Matrix<double> distortionCoefficients = 
            //    new Matrix<double>(new[,] { { -0.103723, 0.571667, -0.002326, 0.007932 } });
            //Frame flangeFrame = new Frame(1, 1, 1, 1, 1, 1);
            //Frame c_camera = new Frame(1, 1, 1, 1, 1, 1);
            //Matrix<double> r_camera = 
            //   new Matrix<double>(new[,] { { 1.0, 1, 1 }, { 1.0, 1, 1 }, { 1.0, 1, 1 } }); ///check

            // Matrix<double> t_camera = 
            //     new Matrix<double>(new[,] { { 1.0, 1, 1 } }); 

            //var f = CameraMath.CalculatePixelLine(c_camera, 
            //    flangeFrame, 
            //    cameraMatrix, 
            //    r_camera, 
            //    t_camera, 
            //   new Point(120, 340), 3f);

        }

    }

}

