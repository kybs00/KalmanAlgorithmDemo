using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace KalmanAlgorithmDemo
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            var algorithmTool = new KalmanAlgorithmTool();
            var points = new List<Point>();
            points.Add(new Point(0, 100));
            points.Add(new Point(100, 200));
            points.Add(new Point(200, 100));
            var predictedPoint = algorithmTool.GetPredictedPoint(points);
            points.Add(predictedPoint);
            var predictedPoint2 = algorithmTool.GetPredictedPoint(points);
        }
    }
}
