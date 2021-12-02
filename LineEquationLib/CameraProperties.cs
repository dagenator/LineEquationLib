using Emgu.CV;

namespace LineEquationLib
{
    public class CameraProperties
    {
        
        //Frame cameraCartesianPosition,
        //Frame flangeFrame,
        //Matrix<double> cameraMatrix, 
        //Matrix<double> R_camera2flange,
        //Matrix<double> T_camera2flange, 
         // Point pixel,
         // double focus)

         
         //отрефакторить
         public Matrix<double> cameraMatrix { get;  set; } //no change
         public Matrix<double> distorsionCoefficients { get;  set; }//no change
         
         public Frame cameraCartesianPosition { get;  set;} // 
         public Frame flangeFrame { get;  set;} //
         
         public Matrix<double> R_camera2flange { get;  set;}
         public Matrix<double> T_camera2flange { get;  set;}
         public double focus { get;   set;} //??

        
         
         
         public void FillCameraProperties(Matrix<double> camM, 
             Matrix<double> distor, 
             Frame cameraPos, 
             Frame flangePos, 
             Matrix<double> R_camera, 
             Matrix<double> T_camera,
             double foc)
         {
             cameraMatrix = camM;
             distorsionCoefficients = distor;
             cameraCartesianPosition = cameraPos;
             flangeFrame = flangePos;
             R_camera2flange = R_camera;
             T_camera2flange = T_camera;
             focus = foc;
         }


    }
}