using System;
using System.Globalization;
using System.Linq;
using System.Xml;
using Emgu.CV;

namespace LineEquationLib
{
    public class Int
    {
        public Int()
        {
        }
        public Int(int i)
        {
            value =  i;
        }
        public int value;
    }
    
    public class XMLReader
    {
        public Matrix<double> ParseMatrixFromXML(XmlElement node)
        {
            Int rows = new Int(), col = new Int();
            double[] data = new double[]{} ;
            GetMatrixParamsFromNode(rows, col, data, node);
            return  ParseMatrixByRowColData(rows.value, col.value, data);
            
        }
        
        public Frame ParseFrameFromXML(XmlElement node)
        {
            double[] data = new double[]{};
            foreach (XmlNode child in node.ChildNodes)
            {
                if (child.Name == "data") data = (child.InnerText.Split(" ")).Select(x=> double.Parse(x)).ToArray();
            }

            Frame fr = new Frame((float)data[0], (float)data[1],(float) data[2],(float) data[2],(float)data[2],(float)data[2] );
            return fr;

        }
        public void GetMatrixParamsFromNode( Int r, Int c, double[] data, XmlElement node)
        {
            foreach (XmlNode child in node.ChildNodes)
            {
                if (child.Name == "rows") r.value = int.Parse(child.InnerText);
                if (child.Name == "cols") c.value = int.Parse(child.InnerText);
                if (child.Name == "data") 
                    data = (child.InnerText.Split()).Where(x=> x.Length>0).Select(x=> double.Parse(x, CultureInfo.InvariantCulture)).ToArray();
            }

        }
        
        public Matrix<double> ParseMatrixByRowColData(int row, int col, double[] data )
        {
            double[,] arr = new double[col, row];

            int i = 0;
            for (int r = 0; r < row; r++)
            {
                for (int c = 0; c < col; c++)
                {
                    arr[r, c] = data[i];
                    i++;
                }
            }
            Matrix<double> mat = new Matrix<double>(arr);
            return mat;
        }
        
        public CameraProperties GetCameraProperties(string Path)
        {
            Console.WriteLine("Reading properties from Xml");
            CameraProperties camera = new CameraProperties();
            XmlDocument xDoc = new XmlDocument();
            try
            {
                xDoc.Load(Path);//добавить обработку исключения
            
                // получим корневой элемент
                XmlElement? root = xDoc.DocumentElement;
                if (root != null)
                {
                    // обход всех узлов в корневом элементе
                    foreach (XmlElement node in root)
                    {

                        switch (node.Name)
                        {
                            case "camera_matrix":
                                camera.cameraMatrix = ParseMatrixFromXML(node);
                                break;
                            case "distortion_coefficients":
                                camera.distorsionCoefficients = ParseMatrixFromXML(node);
                                break;
                            case "R_camera":
                                camera.R_camera2flange = ParseMatrixFromXML(node);
                                break;
                            case "T_camera":
                                camera.T_camera2flange = ParseMatrixFromXML(node);
                                break;
                            case "camera_pos":
                                camera.cameraCartesianPosition = ParseFrameFromXML(node);
                                break;
                            case "flange_position":
                                camera.flangeFrame = ParseFrameFromXML(node);
                                break;
                            case "focus":
                                foreach (XmlNode child in node.ChildNodes)
                                {
                                    if (child.Name == "data") camera.focus = double.Parse(child.Value);
                                }
                                break;
                            
                        }

                        
                        
                        
 
                        // обходим все дочерние узлы элемента user
                        foreach (XmlNode childnode in node.ChildNodes)
                        {
                            // если узел - company
                            if (childnode.Name == "company")
                            {
                                Console.WriteLine($"Компания: {childnode.InnerText}");
                            }
                            // если узел age
                            if (childnode.Name == "age")
                            {
                                Console.WriteLine($"Возраст: {childnode.InnerText}");
                            }
                        }
                        Console.WriteLine();
                    
                        
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine(e);
                throw e;
            }

            return camera;
        } 
    }
}