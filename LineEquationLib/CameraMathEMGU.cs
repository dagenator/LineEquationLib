using System;
using System.Collections.Generic;
using System.Text;

namespace rewriteTry
{
    class CameraMathEMGU
    {
        public const double M_PIl = 3.141592653589793238462643383279502884;
        /*!
          Перевод из FRAME в матрицы смещения и вращения
        \param[in] pos координаты
        \param[out] translation матрица смещения
        \param[out] rotation матрица вращения
        */

        void calcurateMatrixFromFrameFloat(Frame frame, cv::Mat& translation, cv::Mat& rotation)
        {
            translation.at<float>(0) = float(frame.X);
            translation.at<float>(1) = float(frame.Y);
            translation.at<float>(2) = float(frame.Z);


            float a = frame.A * M_PIl / 180.0;
            float b = frame.B * M_PIl / 180.0;
            float c = frame.C * M_PIl / 180.0;
            rotation.at<float>(0, 0) = cos(a) * cos(b);
            rotation.at<float>(0, 1) = -sin(a) * cos(c) + cos(a) * sin(b) * sin(c);
            rotation.at<float>(0, 2) = sin(a) * sin(c) + cos(a) * sin(b) * cos(c);
            rotation.at<float>(1, 0) = sin(a) * cos(b);
            rotation.at<float>(1, 1) = cos(a) * cos(c) + sin(a) * sin(b) * sin(c);
            rotation.at<float>(1, 2) = -cos(a) * sin(c) + sin(a) * sin(b) * cos(c);
            rotation.at<float>(2, 0) = -sin(b);
            rotation.at<float>(2, 1) = cos(b) * sin(c);
            rotation.at<float>(2, 2) = cos(b) * cos(c);
        }
        void RobotPosition::calcurateMatrixFromFrameDouble(RobotState::Frame frame, cv::Mat& translation, cv::Mat& rotation)
        {
            translation.at<double>(0) = double(frame.X);
            translation.at<double>(1) = double(frame.Y);
            translation.at<double>(2) = double(frame.Z);


            double a = frame.A * M_PIl / 180.0;
            double b = frame.B * M_PIl / 180.0;
            double c = frame.C * M_PIl / 180.0;
            rotation.at<double>(0, 0) = cos(a) * cos(b);
            rotation.at<double>(0, 1) = -sin(a) * cos(c) + cos(a) * sin(b) * sin(c);
            rotation.at<double>(0, 2) = sin(a) * sin(c) + cos(a) * sin(b) * cos(c);
            rotation.at<double>(1, 0) = sin(a) * cos(b);
            rotation.at<double>(1, 1) = cos(a) * cos(c) + sin(a) * sin(b) * sin(c);
            rotation.at<double>(1, 2) = -cos(a) * sin(c) + sin(a) * sin(b) * cos(c);
            rotation.at<double>(2, 0) = -sin(b);
            rotation.at<double>(2, 1) = cos(b) * sin(c);
            rotation.at<double>(2, 2) = cos(b) * cos(c);
        }
        /*!
          Перевод из матрицы вращения в углы эйлера
        \param[out] pos координаты, где определены только углы A B C
        \param[in] rotation матрица вращения
        */
        void RobotPosition::calcurateFrameAngleFromMatrixFloat(RobotState::Frame& frame, cv::Mat translation, cv::Mat rotation)
        {
            float A, B, C;


            A = atan2(rotation.at<float>(1, 0), rotation.at<float>(0, 0));
            B = atan2(-rotation.at<float>(2, 0), cos(A) * rotation.at<float>(0, 0) + sin(A) * rotation.at<float>(1, 0));
            C = atan2(sin(A) * rotation.at<float>(0, 2) - cos(A) * rotation.at<float>(1, 2),
                -sin(A) * rotation.at<float>(0, 1) + cos(A) * rotation.at<float>(1, 1));
            frame.A = A * 180 / M_PIl;
            frame.B = B * 180 / M_PIl;
            frame.C = C * 180 / M_PIl;
            std::cout << "calcurateFrameAngleFromMatrixFloat" << std::endl;
        }

        RobotState::Frame RobotPosition::calcurateFrameAngleFromMatrixDouble(RobotState::Frame& frame, cv::Mat translation, cv::Mat rotation)
        {
            double A, B, C;
            frame.X = translation.at<double>(0);
            frame.Y = translation.at<double>(1);
            frame.Z = translation.at<double>(2);

            A = atan2(rotation.at<double>(1, 0), rotation.at<double>(0, 0));
            B = atan2(-rotation.at<double>(2, 0), cos(A) * rotation.at<double>(0, 0) + sin(A) * rotation.at<double>(1, 0));
            C = atan2(sin(A) * rotation.at<double>(0, 2) - cos(A) * rotation.at<double>(1, 2),
                -sin(A) * rotation.at<double>(0, 1) + cos(A) * rotation.at<double>(1, 1));
            frame.A = A * 180 / M_PIl;
            frame.B = B * 180 / M_PIl;
            frame.C = C * 180 / M_PIl;
            std::cout << "calcurateFrameAngleFromMatrixDouble" << std::endl;
            return frame;
        }

        void RobotPosition::calculateLaserPointPosition(RobotState::Frame laserRangefinderToolFrame, RobotState::Frame flangePosition, float laserDistance, float* xyzLaserPoint)
        {
            float x = laserDistance * sin(laserRangefinderToolFrame.A * M_PIl / 180) * sin(laserRangefinderToolFrame.C * M_PIl / 180) + laserRangefinderToolFrame.X;
            float y = -laserDistance * cos(laserRangefinderToolFrame.A * M_PIl / 180) * sin(laserRangefinderToolFrame.C * M_PIl / 180) + laserRangefinderToolFrame.Y;
            float z = laserDistance * cos(laserRangefinderToolFrame.C * M_PIl / 180) + laserRangefinderToolFrame.Z; // Определение положения пятна относительно дальномера

            float a = flangePosition.A * M_PIl / 180;
            float b = flangePosition.B * M_PIl / 180;
            float c = flangePosition.C * M_PIl / 180;
            float o2[3] = { x, y, z };
            float rotateMatrix[3][3];
            rotateMatrix[0][0] = cos(a) * cos(b);
            rotateMatrix[0][1] = -sin(a) * cos(c) + cos(a) * sin(b) * sin(c);
            rotateMatrix[0][2] = sin(a) * sin(c) + cos(a) * sin(b) * cos(c);
            rotateMatrix[1][0] = sin(a) * cos(b);
            rotateMatrix[1][1] = cos(a) * cos(c) + sin(a) * sin(b) * sin(c);
            rotateMatrix[1][2] = -cos(a) * sin(c) + sin(a) * sin(b) * cos(c);
            rotateMatrix[2][0] = -sin(b);
            rotateMatrix[2][1] = cos(b) * sin(c);
            rotateMatrix[2][2] = cos(b) * cos(c);

            float transF[3] = { flangePosition.X, flangePosition.Y, flangePosition.Z };
            cv::Mat TransF = cv::Mat(3, 1, CV_32F, transF);
            cv::Mat trans = cv::Mat(3, 1, CV_32F, o2);
            cv::Mat rotate = cv::Mat(3, 3, CV_32F, rotateMatrix);
            cv::Mat res = rotate * trans + TransF; // Определение положения пятна относительно робота
            xyzLaserPoint[0] = res.at<float>(0);
            xyzLaserPoint[1] = res.at<float>(1);
            xyzLaserPoint[2] = res.at<float>(2);
        }

        RobotState::Frame RobotPosition::calculatePixelLine(RobotState::Frame cameraCartesianPosition, RobotState::Frame flangeFrame, cv::Mat cameraMatrix, cv::Mat R_camera2flange, cv::Mat T_camera2flange, cv::Point2d pixel, double focus)
        {

            double cx, cy, fx, fy;
            cx = cameraMatrix.at<double>(0, 2);

            cy = cameraMatrix.at<double>(1, 2);
            fx = cameraMatrix.at<double>(0, 0);
            fy = cameraMatrix.at<double>(1, 1);

            double px = (pixel.x - cx) / fx;
            double py = (pixel.y - cy) / fy;

            double p = sqrt(pow(px, 2.0) + pow(py, 2.0) + 1);
            double arrVectorPixel2camera[3] = { px / p, py / p, 1 / p };

            cv::Mat vectorPixel2camera = cv::Mat(3, 1, CV_64F, arrVectorPixel2camera); //единичный вектор прямой пикселя в системе камеры

            double cos_theta = vectorPixel2camera.at<double>(2);
            double sin_theta = sqrt(1 - pow(cos_theta, 2));
            double sin_phi = vectorPixel2camera.at<double>(0) / sin_theta;
            double cos_phi = -vectorPixel2camera.at<double>(1) / sin_theta;

            cv::Mat T_flange2base = (cv::Mat_<double>(3, 1));
            cv::Mat R_flange2base = (cv::Mat_<double>(3, 3));

            RobotPosition::calcurateMatrixFromFrameDouble(flangeFrame, T_flange2base, R_flange2base); // перевод положения из XYZABC в матрицы поворота и смещения



            double arrRotatePixel2Camera[3][3] = {
                { cos_phi, -sin_phi * cos_theta,arrVectorPixel2camera[0]},
                                   { sin_phi, cos_phi* cos_theta, arrVectorPixel2camera[1]},
                                   { 0, sin_theta, arrVectorPixel2camera[2]}
            };
            //new 21.05.2021
            cv::Mat R_pixel2Camera = cv::Mat(3, 3, CV_64F, arrRotatePixel2Camera);
            R_camera2flange.convertTo(R_camera2flange, CV_64F);



            cv::Mat R_pixel2Base = R_flange2base * R_camera2flange * R_pixel2Camera;
            cv::Mat T_camera2base = (cv::Mat_<double>(3, 1));

            T_camera2base.at<double>(0) = static_cast<double>(cameraCartesianPosition.X);
            T_camera2base.at<double>(1) = static_cast<double>(cameraCartesianPosition.Y);
            T_camera2base.at<double>(2) = static_cast<double>(cameraCartesianPosition.Z);
            RobotState::Frame targetRangefinderPixelFrame;
            try
            {
                targetRangefinderPixelFrame = RobotPosition::calcurateFrameAngleFromMatrixDouble(cameraCartesianPosition, T_camera2base, R_pixel2Base);
            }
            catch (const std::exception&)
            {
                std::cout << "error" << std::endl;
            }
            //std::cout << "targetRangefinderPixelFrame >> " << targetRangefinderPixelFrame.A << targetRangefinderPixelFrame.B << targetRangefinderPixelFrame.C << std::endl;
            return targetRangefinderPixelFrame;
            }

            // При использовании предварительно скомпилированных заголовочных файлов необходим следующий файл исходного кода для выполнения сборки.



        }
    } }
