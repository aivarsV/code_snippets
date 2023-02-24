using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Program
{
  class Point3d {
    public float X = 0;
    public float Y = 0;
    public float Z = 0;
    public Point3d(float x, float y, float z) {
      X = x;
      Y = y;
      Z = z;
    }
  }


  class MainClass {
      static readonly int probingPointCount = 7; // 4, 7 or 10
      static readonly int optimizationFactorCount = 6; // 3, 4, 6 or 7, but always less than probingPointCount
      static readonly bool normalize = false;
      static readonly int stepsPerMm = 1;

    static int Main(string[] args) {

      DeltaCalibrationHelper deltaParams = new DeltaCalibrationHelper(
          float.Parse(args[0]), //rodLength.Value,
          float.Parse(args[1]), //deltaRadius.Value,
          float.Parse(args[2]), //zMaxLength.Value,
          float.Parse(args[3]), //endstopX.Value,
          float.Parse(args[4]), //endstopY.Value,
          float.Parse(args[5]), //endstopZ.Value,
          float.Parse(args[6]) - 210, //towerX.Value - 210,
          float.Parse(args[7]) - 330, //towerY.Value - 330,
          float.Parse(args[8]) - 90, //towerZ.Value - 90,
          "Repetier");

      deltaParams.ConvertIncomingEnstops(stepsPerMm);

      // calculate measurement coordinates
      List<Point3d> coords = new List<Point3d>();

      for (int i = 0; i < probingPointCount; i++)
      {
        coords.Add(new Point3d(float.Parse(args[9 + i * 3]),
                              float.Parse(args[10 + i * 3]),
                              float.Parse(args[11 + i * 3])));
      }

      // Initialize calculation variables
      Matrix2d probeMotorPositions = new Matrix2d(probingPointCount, 3);
      double[] corrections = new double[probingPointCount];
      double initialSumOfSquares = 0;

      for (int i = 0; i < probingPointCount; i++)
      {
        corrections[i] = 0;
        Point3d machinePos = new Point3d(coords[i].X,
            coords[i].Y,
            0);
        probeMotorPositions.Set(i, 0, deltaParams.Transform(machinePos, 0));
        probeMotorPositions.Set(i, 1, deltaParams.Transform(machinePos, 1));
        probeMotorPositions.Set(i, 2, deltaParams.Transform(machinePos, 2));

        initialSumOfSquares += Math.Pow(coords[i].Z, 2);
      }

      int iteration = 0;

      // do actual calculations
      while (true)
      {
        Matrix2d derivativeMatrix = new Matrix2d(probingPointCount,
            optimizationFactorCount);
        for (int i = 0; i < probingPointCount; i++)
        {
          for (int j = 0; j < optimizationFactorCount; j++)
          {
            derivativeMatrix.Set(i, j,
                deltaParams.ComputeDerivative(j,
                  probeMotorPositions.Get(i,0),
                  probeMotorPositions.Get(i,1),
                  probeMotorPositions.Get(i,2)));
          }
        }

        Matrix2d normalMatrix = new Matrix2d(optimizationFactorCount,
            optimizationFactorCount + 1);

        for (int i = 0; i < optimizationFactorCount; i++)
        {
          for (int j = 0; j < optimizationFactorCount; j++)
          {
            double tmp = derivativeMatrix.Get(0, i) * derivativeMatrix.Get(0, j);
            for (int k = 1; k < probingPointCount; k++)
            {
              tmp += derivativeMatrix.Get(k, i) * derivativeMatrix.Get(k, j);
            }
            normalMatrix.Set(i, j, tmp);
          }
          double temp = derivativeMatrix.Get(0, i) * -(coords[0].Z + corrections[0]);
          for (int k = 1; k < probingPointCount; ++k) {
            temp += derivativeMatrix.Get(k, i) * -(coords[k].Z + corrections[k]);
          }
          normalMatrix.Set(i, optimizationFactorCount, temp);
        }

        double[] solution = normalMatrix.GaussJordan();

        deltaParams.Adjust(optimizationFactorCount, solution, normalize);
        // Calculate the expected probe heights using the new parameters
        double[] expectedResiduals = new double[probingPointCount];
        double sumOfSquares = 0;
        for (int i = 0; i < probingPointCount; i++)
        {
          for (int axis = 0; axis < 3; axis++)
          {
            probeMotorPositions.Set(i, axis,
                probeMotorPositions.Get(i, axis) + solution[axis]);
          }

          double newZ = deltaParams.InverseTransform(probeMotorPositions.Get(i, 0),
              probeMotorPositions.Get(i, 1),
              probeMotorPositions.Get(i, 2));
          corrections[i] = newZ;
          expectedResiduals[i] = coords[i].Z + newZ;
          sumOfSquares += Math.Pow(expectedResiduals[i], 2);
        }

        // Decide whether to do another iteration Two is slightly better than one, but three doesn't improve things.
        // Alternatively, we could stop when the expected RMS error is only slightly worse than the RMS of the residuals.
        iteration++;
        if (iteration == 2) break;
      }

      deltaParams.ConvertOutgoingEnstops(stepsPerMm);

      // Output results
      Console.WriteLine("configured_endstop_offsets = [{0}; {1}; {2}]", deltaParams.xstop, deltaParams.ystop, deltaParams.zstop);

      Console.WriteLine("configured_rod_length = [{0}; {0}; {0}]", deltaParams.diagonal);

      Console.WriteLine("configured_tower_radius = {0}", deltaParams.radius);

      Console.WriteLine("configured_toolhead_home_z = {0}", deltaParams.height);

      Console.WriteLine("configured_tower_angles = [{0}; {1}; {2}]",
                                              deltaParams.xadj + 210,
                                              deltaParams.yadj + 330,
                                              deltaParams.zadj + 90);
      return 0;
    }
  }

  class DeltaCalibrationHelper
  {
#region private props
    private double homedCarriageHeight;
    private double[] towerX = new double[3];
    private double[] towerY = new double[3];
    private double Xbc;
    private double Xca;
    private double Xab;
    private double Ybc;
    private double Yca;
    private double Yab;
    private double coreFa;
    private double coreFb;
    private double coreFc;
    private double Q;
    private double Q2;
    private double D2;
    private string firmware;
#endregion
#region public props
    public double height;
    public double diagonal;
    public double radius;
    public double xstop;
    public double ystop;
    public double zstop;
    public double xadj;
    public double yadj;
    public double zadj;
#endregion
    public DeltaCalibrationHelper(double d,
                                  double r,
                                  double h,
                                  double xs,
                                  double ys,
                                  double zs,
                                  double xa,
                                  double ya,
                                  double za,
                                  string fw)
    {
    diagonal = d;
    radius = r;
    height = h;
    xstop = xs;
    ystop = ys;
    zstop = zs;
    xadj = xa;
    yadj = ya;
    zadj = za;

    firmware = fw;

    Recalc();
    }

    private void Recalc()
    {
      towerX[0] = -1.0 * radius * Math.Cos((30.0 + xadj) / 180.0 * Math.PI);
      towerY[0] = -1.0 * radius * Math.Sin((30.0 + xadj) / 180.0 * Math.PI);
      towerX[1] = +1.0 * radius * Math.Cos((30.0 - yadj) / 180.0 * Math.PI);
      towerY[1] = -1.0 * radius * Math.Sin((30.0 - yadj) / 180.0 * Math.PI);
      towerX[2] = -1.0 * radius * Math.Sin(zadj / 180.0 * Math.PI);
      towerY[2] = +1.0 * radius * Math.Cos(zadj / 180.0 * Math.PI);

      Xbc = towerX[2] - towerX[1];
      Xca = towerX[0] - towerX[2];
      Xab = towerX[1] - towerX[0];
      Ybc = towerY[2] - towerY[1];
      Yca = towerY[0] - towerY[2];
      Yab = towerY[1] - towerY[0];

      coreFa = Math.Pow(towerX[0], 2) + Math.Pow(towerY[0], 2);
      coreFb = Math.Pow(towerX[1], 2) + Math.Pow(towerY[1], 2);
      coreFc = Math.Pow(towerX[2], 2) + Math.Pow(towerY[2], 2);
      Q = 2 * (Xca * Yab - Xab * Yca);
      Q2 = Math.Pow(Q, 2);
      D2 = Math.Pow(diagonal, 2);

      double h = diagonal;
      homedCarriageHeight = height + h - InverseTransform(h, h, h);
    }

    public double InverseTransform(double Ha, double Hb, double Hc)
    {
      var Fa = coreFa + Math.Pow(Ha, 2);
      var Fb = coreFb + Math.Pow(Hb, 2);
      var Fc = coreFc + Math.Pow(Hc, 2);

      // Setup PQRSU such that x = -(S - uz)/P, y = (P - Rz)/Q
      var P = (Xbc * Fa) + (Xca * Fb) + (Xab * Fc);
      var S = (Ybc * Fa) + (Yca * Fb) + (Yab * Fc);

      var R = 2 * ((Xbc * Ha) + (Xca * Hb) + (Xab * Hc));
      var U = 2 * ((Ybc * Ha) + (Yca * Hb) + (Yab * Hc));

      var R2 = Math.Pow(R, 2);
      var U2 = Math.Pow(U, 2);

      var A = U2 + R2 + Q2;
      var minusHalfB = S * U + P * R + Ha * Q2 + towerX[0] * U * Q - towerY[0] * R * Q;
      var C = Math.Pow(S + towerX[0] * Q, 2) +
              Math.Pow(P - towerY[0] * Q, 2) +
              (Math.Pow(Ha, 2) - D2) * Q2;

      return (minusHalfB - Math.Sqrt(Math.Pow(minusHalfB, 2) - A * C)) / A;
    }

    public double Transform(Point3d machinePos, int axis)
    {
      return machinePos.Z + Math.Sqrt(D2 -
                                      Math.Pow(machinePos.X - towerX[axis], 2) -
                                      Math.Pow(machinePos.Y - towerY[axis], 2));
    }

    private DeltaCalibrationHelper Copy()
    {
      return new DeltaCalibrationHelper(diagonal, radius, height,
                                        xstop, ystop, zstop,
                                        xadj, yadj, zadj, firmware);
    }

    public double ComputeDerivative(int deriv, double ha, double hb, double hc)
    {
      double perturb = 0.2;// perturbation amount in mm or degrees
      DeltaCalibrationHelper hiParams = Copy();
      DeltaCalibrationHelper loParams = Copy();

      switch(deriv)
      {
        case 0:
        case 1:
        case 2:
          break;

        case 3:
          hiParams.radius += perturb;
          loParams.radius -= perturb;
          break;

        case 4:
          hiParams.xadj += perturb;
          loParams.xadj -= perturb;
          break;

        case 5:
          hiParams.yadj += perturb;
          loParams.yadj -= perturb;
          break;

        case 6:
          hiParams.diagonal += perturb;
          loParams.diagonal -= perturb;
          break;
      }

      hiParams.Recalc();
      loParams.Recalc();

      var zHi = hiParams.InverseTransform((deriv == 0) ? ha + perturb : ha,
                                          (deriv == 1) ? hb + perturb : hb,
                                          (deriv == 2) ? hc + perturb : hc);
      var zLo = loParams.InverseTransform((deriv == 0) ? ha - perturb : ha,
                                          (deriv == 1) ? hb - perturb : hb,
                                          (deriv == 2) ? hc - perturb : hc);

      return (zHi - zLo)/(2 * perturb);
    }

    public void Adjust(int numFactors, double[] v, bool norm)
    {
      double oldCarriageHeightA = homedCarriageHeight + xstop; // save for later

      // Update endstop adjustments
      xstop += v[0];
      ystop += v[1];
      zstop += v[2];

      if (norm) NormaliseEndstopAdjustments();

      if (numFactors >= 4)
      {
        radius += v[3];

        if (numFactors >= 6)
        {
          xadj += v[4];
          yadj += v[5];

          if (numFactors == 7) diagonal += v[6];
        }
      }

        Recalc();

      // Adjusting the diagonal and the tower positions affects the homed carriage height.
      // We need to adjust homedHeight to allow for this, to get the change that was requested in the endstop corrections.
      var heightError = homedCarriageHeight + xstop - oldCarriageHeightA - v[0];
      height -= heightError;
      homedCarriageHeight -= heightError;
    }

    public void ConvertIncomingEnstops(int stepsPerMm)
    {
      double endstopFactor = (firmware == "RRF") ? 1.0
        : (firmware == "Repetier") ? 1.0/stepsPerMm : -1.0;
      xstop *= endstopFactor;
      ystop *= endstopFactor;
      zstop *= endstopFactor;
    }

    public void ConvertOutgoingEnstops(int stepsPerMm)
    {
      double endstopFactor = (firmware == "RRF") ? 1.0
        : (firmware == "Repetier") ? stepsPerMm : -1.0;
      xstop *= endstopFactor;
      ystop *= endstopFactor;
      zstop *= endstopFactor;
    }

    private void NormaliseEndstopAdjustments()
    {
      double eav = (firmware == "Marlin" ||
                    firmware == "MarlinRC" ||
                    firmware == "Repetier") ? Math.Min(xstop, Math.Min(ystop, zstop))
                                            : (xstop + ystop + zstop)/3.0;
      xstop -= eav;
      ystop -= eav;
      zstop -= eav;
      height += eav;
      homedCarriageHeight += eav;        // no need for a full recalc, this is sufficient
    }
  }

  public class Matrix2d
  {
#region private props
    private int _rows;
    private int _cols;
    private double[] data;
#endregion

    public Matrix2d(int rows, int cols)
    {
      _rows = rows;
      _cols = cols;
      data = new double[rows * cols];
    }

    public double Get(int row, int column)
    {
      if (column < 0 || column >= _cols)
        throw new System.ArgumentException("Parameter mus be non-negative and less than count", "column");
      if (row < 0 || row >= _rows)
        throw new System.ArgumentException("Parameter mus be non-negative and less than count", "row");

      return data[column + row * _cols];
    }
    public void Set(int row, int column, double value)
    {
      if (column < 0 || column >= _cols)
        throw new System.ArgumentException("Parameter mus be non-negative and less than count", "column");
      if (row < 0 || row >= _rows)
        throw new System.ArgumentException("Parameter mus be non-negative and less than count", "row");

      data[column + row * _cols] = value;
    }

    private void swapRows(int i, int j)
    {
      if (i == j) return;
      for (int k = 0; k < _cols; k++)
      {
        double v = Get(i, k);
        Set(i, k, Get(j, k));
        Set(j, k, v);
      }
    }

  // Perform Gauus-Jordan elimination on a matrix with numRows rows and (njumRows + 1) columns
    public double[] GaussJordan()
    {
      for (int i = 0; i < _rows; i++)
      {
        double vmax = Math.Abs(Get(i, i));
        for (int j = i + 1; j < _rows; j++)
        {
          double rmax = Math.Abs(Get(j, i));
          if (rmax > vmax)
          {
            swapRows(i, j);
            vmax = rmax;
          }
        }

        double v = Get(i, i);
        for (int j = 0; j < i; j++)
        {
          double factor = Get(j, i) / v;
          Set(j, i, 0);
          for (int k = i + 1; k <= _rows; k++)
          {
            Set(j, k, Get(j, k) - Get(i, k) * factor);
          }
        }

        for (int j = i + 1; j < _rows; j++)
        {
          double factor = Get(j, i) / v;
          Set(j, i, 0);
          for (int k = i + 1; k <= _rows; k++)
          {
            Set(j, k, Get(j, k) - Get(i, k) * factor);
          }
        }
      }

      double[] result = new double[_rows];
      for (int i = 0; i < _rows; i++)
        result[i] = Get(i, _rows) / Get(i, i);

      return result;
    }
  }
}

