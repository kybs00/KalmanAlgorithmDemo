using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace KalmanAlgorithmDemo
{
    public class KalmanAlgorithmTool
    {
        public static readonly double R_NOISE = 9.0f;//观测噪声

        static double[] sg_A_Progress = { 1.0f, 1.0f, 0.5f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f };
        static double[] sg_Q_Noise = { 9.0f, 9.0f, 9.0f, 9.0f, 9.0f, 9.0f, 9.0f, 9.0f, 9.0f };      //系统噪声

        static PointKFData sg_stPointKFData;

        public KalmanAlgorithmTool()
        {
            sg_stPointKFData = new PointKFData();
        }
        /// <summary>
        /// 矩阵乘以一个常数
        /// </summary>
        /// <param name="a">矩阵A</param>
        /// <param name="s">被乘常数S</param>
        /// <param name="m/n">矩阵A的行列</param>
        /// <param name="output">输出矩阵</param>
        /// <returns></returns>
        private int matrixMultiSm(double[] a, double s, int m, int n, double[] output)
        {
            int i, j;
            for (i = 0; i < m; i++)
            {
                for (j = 0; j < n; j++)
                {
                    output[i * n + j] = a[i * n + j] * s;
                }
            }
            return 0;
        }
        /// <summary>
        /// 矩阵乘另一个矩阵
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="m"></param>
        /// <param name="s"></param>
        /// <param name="n"></param>
        private void matrixMultiMSSN(double[] a, double[] b, int m, int s, int n, double[] output)
        {
            int i, j, k;
            for (i = 0; i < m; i++)
            {
                for (j = 0; j < n; j++)
                {
                    output[i * n + j] = 0.0f;
                    for (k = 0; k < s; k++)
                    {
                        output[i * n + j] += a[i * s + k] * b[k * n + j];
                    }
                }
            }
        }
        /* 矩阵的转置
 * @para A: 矩阵A
 * @para m\n: 矩阵A的行列
 * @para Out: 输出矩阵
 */
        private void matrixT(double[] a, int m, int n, double[] output)
        {
            int i, j;
            for (i = 0; i < n; i++)
            {
                for (j = 0; j < m; j++)
                {
                    output[i * m + j] = a[j * n + i];
                }
            }
        }
        /* 矩阵加法
  * @para A\B 矩阵A\B
  * @para m\n: 矩阵A\B的行列
  * @para Out: 输出矩阵
  */
        private void matrixAdd(double[] a, double[] b, int m, int n, double[] output)
        {
            int i, j;
            for (i = 0; i < m; i++)
            {
                for (j = 0; j < n; j++)
                {
                    output[i * n + j] = a[i * n + j] + b[i * n + j];
                }
            }
        }

        /* 矩阵减法
         * @para A\B 矩阵A\B
         * @para m\n: 矩阵A\B的行列
         * @para Out: 输出矩阵
         */
        private void matrixSub(double[] a, double[] b, int m, int n, double[] output)
        {
            int i, j;
            for (i = 0; i < m; i++)
            {
                for (j = 0; j < n; j++)
                {
                    output[i * n + j] = a[i * n + j] - b[i * n + j];
                }
            }
        }

        /*
         * 求先验估计状态
         * @para A: 过程增益矩阵
         * @para formerStaVec: 输入的前一个状态的状态向量
         * @para priorEstStaVec: 输出的当前状态先验估计状态向量
         */
        private void priorEstimateSta(double[] a, double[] formerStaVec, double[] priorEstStaVec)
        {
            priorEstStaVec[0] = a[0] * formerStaVec[0] + a[1] * formerStaVec[1] + a[2] * formerStaVec[2];
            priorEstStaVec[1] = a[3] * formerStaVec[0] + a[4] * formerStaVec[1] + a[5] * formerStaVec[2];
            priorEstStaVec[2] = a[6] * formerStaVec[0] + a[7] * formerStaVec[1] + a[8] * formerStaVec[2];
        }

        /*
         * 求先验估计协方差
         * @para A: 过程增益矩阵
         * @para Pforme: 前一个状态的后验估计协方差
         * @para Q: 过程噪声协方差
         * @para Pprior: 先验估计协方差
         */
        private void priorEstimateCov(double[] a, double[] pFormer, double[] q, double[] pPrior)
        {
            double[] proMat0 = new double[9];// 运算过程矩阵
            double[] proMat1 = new double[9];// 运算过程矩阵

            double[] a_t = new double[9];// A的转置

            matrixT(a, 3, 3, a_t);
            matrixMultiMSSN(a, pFormer, 3, 3, 3, proMat0);
            matrixMultiMSSN(proMat0, a_t, 3, 3, 3, proMat1);

            int i;
            for (i = 0; i < 9; i++)
            {
                proMat0[i] = q[i];
            }

            matrixAdd(proMat1, proMat0, 3, 3, pPrior);
        }

        /*
         * 求最优残余增益
         * @para Pprior: 先验估计协方差
         * @para R: 测量噪声协方差
         * @para K: 最优残余增益
         */
        private void optimalResidualGain(double[] pPrior, double r, double[] k)
        {
            double proMat1 = (double)1.0f / (pPrior[0] + r);

            int i;
            for (i = 0; i < 3; i++)
            {
                k[i] = proMat1 * pPrior[i * 3];
            }
        }

        /*
         * 求最优残余增益
         * @para priorEstStaVec: 前状态先验估计状态向量
         * @para K: 最优残余增益
         * @para Z: 观测量
         * @para optimalEstStaVec: 最优状态估计
         */
        private void optimalEstimateSta(double[] priorEstStaVec, double[] k, double z, double[] optimalEstStaVec)
        {
            double temp = z - priorEstStaVec[0];

            optimalEstStaVec[0] = temp * k[0] + priorEstStaVec[0];
            optimalEstStaVec[1] = temp * k[1] + priorEstStaVec[1];
            optimalEstStaVec[2] = temp * k[2] + priorEstStaVec[2];
        }

        /*
         * 求后验估计协方差
         * @para K: 最优残余增益
         * @para Pprior: 先验估计协方差
         * @para Ppost:
         */
        private void posterioriEstimateCov(double[] k, double[] pPrior, double[] pPost)
        {
            double[] proMat0 = new double[9];// 运算过程矩阵
            double[] proMat1 = new double[9];// 运算过程矩阵
            double[] h = { 1.0f, 0.0f, 0.0f };

            matrixMultiMSSN(k, h, 3, 1, 3, proMat0);
            matrixMultiMSSN(proMat0, pPrior, 3, 3, 3, proMat1);
            matrixSub(pPrior, proMat1, 3, 3, pPost);
        }

        /*
         * 将压入卡尔曼滤波器（浮点坐标）
         * @para stCp: 坐标
         * @para nId: 坐标ID
         * @return: 返回预测位置
         */
        private Point pushKalmanByPointF(Point stCp)
        {
            double[] optimalEstStaVec = new double[3];
            optimalEstStaVec[0] = 0.0f;

            optimalEstStaVec[0] = stCp.X;
            optimalEstStaVec[1] = (double)0.0f;
            optimalEstStaVec[2] = (double)0.0f;
            priorEstimateSta(sg_A_Progress, optimalEstStaVec, sg_stPointKFData.x.priorEstStaVec);
            for (int i = 0, len = sg_stPointKFData.x.pPrior.Length; i < len; i++)
            {
                sg_stPointKFData.x.pPrior[i] = 0;
            }

            optimalEstStaVec[0] = stCp.Y;
            optimalEstStaVec[1] = (double)0.0f;
            optimalEstStaVec[2] = (double)0.0f;
            priorEstimateSta(sg_A_Progress, optimalEstStaVec, sg_stPointKFData.y.priorEstStaVec);
            for (int i = 0, len = sg_stPointKFData.y.pPrior.Length; i < len; i++)
            {
                sg_stPointKFData.y.pPrior[i] = 0;
            }

            return stCp;
        }

        /*
         * 更新卡尔曼滤波器中的坐标（浮点坐标）
         * @para stCp: 坐标
         * @para nId: 坐标ID
         * @return: 返回预测位置
         */
        private Point updateKalmanByPointF(Point stCp)
        {
            double[] optimalEstStaVec = new double[3];
            optimalEstStaVec[0] = 0.0f;
            double[] fpPost = new double[9];
            fpPost[0] = 0.0f;
            double[] k = new double[3];
            k[0] = 0.0f;

            priorEstimateCov(sg_A_Progress, sg_stPointKFData.x.pPrior, sg_Q_Noise, fpPost);
            optimalResidualGain(fpPost, R_NOISE, k);

            optimalEstimateSta(sg_stPointKFData.x.priorEstStaVec, k, stCp.X, optimalEstStaVec);
            posterioriEstimateCov(k, fpPost, sg_stPointKFData.x.pPrior);   //更新卡尔曼滤波的预测点

            priorEstimateSta(sg_A_Progress, optimalEstStaVec, sg_stPointKFData.x.priorEstStaVec);

            priorEstimateCov(sg_A_Progress, sg_stPointKFData.y.pPrior, sg_Q_Noise, fpPost);
            optimalResidualGain(fpPost, R_NOISE, k);

            optimalEstimateSta(sg_stPointKFData.y.priorEstStaVec, k, stCp.Y, optimalEstStaVec);
            posterioriEstimateCov(k, fpPost, sg_stPointKFData.y.pPrior);

            priorEstimateSta(sg_A_Progress, optimalEstStaVec, sg_stPointKFData.y.priorEstStaVec);

            stCp.X = sg_stPointKFData.x.priorEstStaVec[0];
            stCp.Y = sg_stPointKFData.y.priorEstStaVec[0];

            return stCp;
        }


        void kalmanPush(double x, double y)
        {
            Point a = new Point(x, y);
            pushKalmanByPointF(a);
        }

        void kalmanUpdate(double[] ret, double x, double y)
        {
            Point a = new Point(x, y);
            a = updateKalmanByPointF(a);
            ret[0] = a.X;
            ret[1] = a.Y;
        }

        public void push(double x, double y)
        {
            Point a = new Point(x, y);
            pushKalmanByPointF(a);
        }

        public double[] update(double x, double y)
        {
            double[] arr = new double[2];
            Point a = updateKalmanByPointF(new Point(x, y));
            arr[0] = a.X;
            arr[1] = a.Y;
            return arr;
        }

        public Point GetPredictedPoint(List<Point> points)
        {
            for (int i = 0; i < points.Count - 1; i++)
            {
                push(points[i].X, points[i].Y);
            }
            var doubles = update(points[points.Count - 1].X, points[points.Count - 1].Y);
            return new Point(doubles[0], doubles[1]);
        }
    }
    internal class PointKFData
    {
        public KalmanFilterData x = new KalmanFilterData();
        public KalmanFilterData y = new KalmanFilterData();
    }
    public class KalmanFilterData
    {
        public double[] priorEstStaVec = new double[3]; //坐标、速度、加速度
        public double[] pPrior = new double[9];
    }
}
