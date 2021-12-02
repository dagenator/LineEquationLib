using Emgu.CV;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;

//using OpenCvSharp;

namespace LineEquationLib
{
    static class CameraMath
    {
        /*!
          Перевод из FRAME в матрицы смещения и вращения
        \param[in] pos координаты
        \param[out] translation матрица смещения
        \param[out] rotation матрица вращения
        */

        static void CalculateMatrixFromFrameDouble(Frame frame, Matrix<double> translation, Matrix<double> rotation)
        {
            translation.Data = new[,] { { (double)frame.X, (double)frame.Y, (double)frame.Z } };

            var a = frame.A * Math.PI / 180.0f;
            var b = frame.B * Math.PI / 180.0f;
            var c = frame.C * Math.PI / 180.0f;

            var myMat = new[,]
            {
                {
                    Math.Cos(a) * Math.Cos(b),
                    -Math.Sin(a) * Math.Cos(c) + Math.Cos(a) * Math.Sin(b) * Math.Sin(c),
                    Math.Sin(a) * Math.Sin(c) + Math.Cos(a) * Math.Sin(b) * Math.Cos(c)
                },
                {
                    Math.Sin(a) * Math.Cos(b),
                    Math.Cos(a) * Math.Cos(c) + Math.Sin(a) * Math.Sin(b) * Math.Sin(c),
                    -Math.Cos(a) * Math.Sin(c) + Math.Sin(a) * Math.Sin(b) * Math.Cos(c)
                },
                {
                    -Math.Sin(b),
                    Math.Cos(b) * Math.Sin(c),
                    Math.Cos(b) * Math.Cos(c)
                }
            };
            rotation.Data = myMat;
        }
        /*!
          Перевод из матрицы вращения в углы эйлера
        \param[out] pos координаты, где определены только углы A B C
        \param[in] rotation матрица вращения
        */
        
        static Frame CalculateFrameAngleFromMatrixDouble(Frame frame, Matrix<double> translation, Matrix<double> rotation)
        {
            double A, B, C;
            frame.X = (float)translation[0,0];
            frame.Y = (float)translation[0,1];
            frame.Z = (float)translation[0,2];

            A = Math.Atan2(rotation[1, 0], rotation[0,0]);
            B = Math.Atan2(-rotation[2, 0], Math.Cos(A) * rotation[0, 0] + Math.Sin(A) * rotation[1, 0]);
            C = Math.Atan2(Math.Sin(A) * rotation[0, 2] - Math.Cos(A) * rotation[1, 2],
                -Math.Sin(A) * rotation[0, 1] + Math.Cos(A) * rotation[1, 1]);
            frame.A = (float)(A * 180 / Math.PI);
            frame.B = (float)(B * 180 / Math.PI);
            frame.C = (float)(C * 180 / Math.PI);
            Console.WriteLine("calcurateFrameAngleFromMatrixDouble");
            return frame;
        }

        static void СalculateLaserPointPosition(Frame laserRangefinderToolFrame, Frame flangePosition, float laserDistance, ref double[] xyzLaserPoint)
        {
            double x = (laserDistance * Math.Sin(laserRangefinderToolFrame.A * Math.PI / 180) * 
                Math.Sin(laserRangefinderToolFrame.C * Math.PI / 180) + laserRangefinderToolFrame.X);

            double y = (-laserDistance * Math.Cos(laserRangefinderToolFrame.A * Math.PI / 180) * 
                Math.Sin(laserRangefinderToolFrame.C * Math.PI / 180) + laserRangefinderToolFrame.Y);

            double z = (laserDistance * Math.Cos(laserRangefinderToolFrame.C * Math.PI / 180) + 
                laserRangefinderToolFrame.Z); // Определение положения пятна относительно дальномера

            double a = (flangePosition.A * Math.PI / 180);
            double b = (flangePosition.B * Math.PI / 180);
            double c = (flangePosition.C * Math.PI / 180);
           
            
            Matrix<double> TransF = new Matrix<double>
                (new[,] { { (double)flangePosition.X, flangePosition.Y, flangePosition.Z } });

            Matrix<double> trans = new Matrix<double>(new[,] { { x, y, z } });

            Matrix<double> rotate = new Matrix<double>(3, 3);
            rotate.Data = new[,] {
                {
                    Math.Cos(a) * Math.Cos(b),
                    -Math.Sin(a) * Math.Cos(c) + Math.Cos(a) * Math.Sin(b) * Math.Sin(c),
                    Math.Sin(a) * Math.Sin(c) + Math.Cos(a) * Math.Sin(b) * Math.Cos(c)
                },
                {
                    Math.Sin(a) * Math.Cos(b),
                    Math.Cos(a) * Math.Cos(c) + Math.Sin(a) * Math.Sin(b) * Math.Sin(c),
                    -Math.Cos(a) * Math.Sin(c) + Math.Sin(a) * Math.Sin(b) * Math.Cos(c)
                },
                {
                    -Math.Sin(b),
                    Math.Cos(b) * Math.Sin(c),
                    Math.Cos(b) * Math.Cos(c)
                }
            };

            Matrix<double> res = rotate * trans + TransF; // Определение положения пятна относительно робота
            xyzLaserPoint[0] = res[0,0];
            xyzLaserPoint[1] = res[0,1];
            xyzLaserPoint[2] = res[0,2];
        }

        static public Frame CalculatePixelLineFromCameraProperties(CameraProperties prop, Point pixel )
        {
            return CalculatePixelLine(
                prop.cameraCartesianPosition,
                prop.flangeFrame,
                prop.cameraMatrix,
                prop.R_camera2flange,
                prop.T_camera2flange,
                pixel,
                prop.focus);
        }
        
        static public Frame CalculatePixelLine(Frame cameraCartesianPosition, 
            Frame flangeFrame, 
            Matrix<double> cameraMatrix, 
            Matrix<double> R_camera2flange, 
            Matrix<double> T_camera2flange, 
            Point pixel, double focus)
        {
            double cx, cy, fx, fy;
            cx = cameraMatrix[0, 2];
            cy = cameraMatrix[1, 2];
            fx = cameraMatrix[0, 0];
            fy = cameraMatrix[1, 1];

            double px = (pixel.X - cx) / fx;
            double py = (pixel.Y - cy) / fy;

            double p = Math.Sqrt(Math.Pow(px, 2.0) + Math.Pow(py, 2.0) + 1);

            double[] arrVectorPixel2camera = new double[3] { px / p, py / p, 1 / p };

            Matrix<double> vectorPixel2camera = 
                new Matrix<double>(new[,] { { (px / p), (py / p), (1 / p) } }); //единичный вектор прямой пикселя в системе камеры

            double cos_theta = vectorPixel2camera[0,2];
            double sin_theta = Math.Sqrt(1 - Math.Pow(cos_theta, 2));
            double sin_phi = vectorPixel2camera[0,0] / sin_theta;
            double cos_phi = -vectorPixel2camera[0,1] / sin_theta;

            Matrix<double> T_flange2base = new Matrix<double>(3, 1);
            Matrix<double> R_flange2base = new Matrix<double>(3, 3); // TODO

            CalculateMatrixFromFrameDouble(flangeFrame, T_flange2base, R_flange2base); // перевод положения из XYZABC в матрицы поворота и смещения


            Matrix<double> R_pixel2Camera = new Matrix<double>(3, 3);
            R_pixel2Camera.Data = new [,]
            {
                { cos_phi, -sin_phi * cos_theta,arrVectorPixel2camera[0]},
                { sin_phi, cos_phi* cos_theta, arrVectorPixel2camera[1]},
                { 0, sin_theta, arrVectorPixel2camera[2]}
            };

            var R_pixel2Base = R_flange2base * R_camera2flange * R_pixel2Camera;
            var T_camera2base = new Matrix<double>(3, 1); //возможно (1, 3)

            T_camera2base.Data = new double[,]
            {
                {
                    cameraCartesianPosition.X,
                    cameraCartesianPosition.Y,
                    cameraCartesianPosition.Z
                }
            };
            
            Frame targetRangefinderPixelFrame;
            try
            {
                targetRangefinderPixelFrame = 
                    CalculateFrameAngleFromMatrixDouble(cameraCartesianPosition, T_camera2base, R_pixel2Base);
                return targetRangefinderPixelFrame;
            }
            catch (Exception e)
            {
                Console.WriteLine("error");
                throw e;
            }

            //std::cout << "targetRangefinderPixelFrame >> " << targetRangefinderPixelFrame.A << targetRangefinderPixelFrame.B << targetRangefinderPixelFrame.C << std::endl;
            
        }
    }
}
