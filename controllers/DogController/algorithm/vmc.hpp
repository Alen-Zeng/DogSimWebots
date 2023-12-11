#pragma once
/* Includes ------------------------------------------------------------------*/
#include <../../libraries/eigen/Eigen/Eigen>
#include <cmath>
#include <bind.hpp>
/* Private macros ------------------------------------------------------------*/
Eigen::MatrixXd H(20, 5);
Eigen::MatrixXd Fs(5, 1);
static const double errorMin = 0.0001;
/* Private type --------------------------------------------------------------*/
namespace vmcspace
{
  template <typename T>
  const T abs(const T input)
  {
    return input < (T)0 ? -input : input;
  }
}
/* Exported function declarations --------------------------------------------*/

/**
 * @brief 映射矩阵内容初始化
 * 
 * @param _H 映射矩阵
 * @param alpha pitch偏角
 * @param beta roll偏角
 */
void MatrixHInit(Eigen::MatrixXd &_H, const double alpha, const double beta, const double O1lf, const double O2lf, const double O4lf, const double O1rf, const double O2rf, const double O4rf, const double O1lb, const double O2lb, const double O4lb, const double O1rb, const double O2rb, const double O4rb)
{
  static double Ltlf, Ltrf, Ltlb, Ltrb;
  static double Pl, Pr, Ql, Qr, Rl, Sl, Rr, Sr, Uf, Wf, Ub, Wb;
  static double O0lf, O0rf, O0lb, O0rb;
  static double O3lf, O3rf, O3lb, O3rb;

  O0lf = alpha - O1lf - O2lf;
  O0rf = alpha - O1rf - O2rf;
  O0lb = alpha - O1lb - O2lb;
  O0rb = alpha - O1rb - O2rb;
  O3lf = beta - O4lf;
  O3rf = beta - O4rf;
  O3lb = beta - O4lb;
  O3rb = beta - O4rb;

  std::cout << " O1lf:"<< O1lf << " O1rf:"<< O1rf << " O1lb:"<< O1lb << " O1rb:"<< O1rb << std::endl;
  std::cout << " O2lf:"<< O2lf << " O2rf:"<< O2rf << " O2lb:"<< O2lb << " O2rb:"<< O2rb << std::endl;
  std::cout << " O4lf:"<< O4lf << " O4rf:"<< O4rf << " O4lb:"<< O4lb << " O4rb:"<< O4rb << std::endl;

  Ltlf = L1 * cos(O0lf) + L2 * cos(O0lf + O1lf);
  Ltrf = L1 * cos(O0rf) + L2 * cos(O0rf + O1rf);
  Ltlb = L1 * cos(O0lb) + L2 * cos(O0lb + O1lb);
  Ltrb = L1 * cos(O0rb) + L2 * cos(O0rb + O1rb);

  Pl = -L1 * cos(O0lf) - L2 * cos(O0lf + O1lf) + L * sin(O0lf + O1lf + O2lf);
  Pr = -L1 * cos(O0rf) - L2 * cos(O0rf + O1rf) + L * sin(O0rf + O1rf + O2rf);
  Ql = -L1 * sin(O0lf) - L2 * sin(O0lf + O1lf) - L * cos(O0lf + O1lf + O2lf);
  Qr = -L1 * sin(O0rf) - L2 * sin(O0rf + O1rf) - L * cos(O0rf + O1rf + O2rf);
  Rl = -L1 * cos(O0lb) - L2 * cos(O0lb + O1lb) - L * sin(O0lb + O1lb + O2lb);
  Rr = -L1 * cos(O0rb) - L2 * cos(O0rb + O1rb) - L * sin(O0rb + O1rb + O2rb);
  Sl = -L1 * sin(O0lb) - L2 * sin(O0lb + O1lb) + L * cos(O0lb + O1lb + O2lb);
  Sr = -L1 * sin(O0rb) - L2 * sin(O0rb + O1rb) + L * cos(O0rb + O1rb + O2rb);
  Uf = s * sin(O3lf + O4lf) - Ltlf * cos(O3lf);
  Ub = s * sin(O3lb + O4lb) - Ltlb * cos(O3lb);
  Wf = -s * sin(O3rf + O4rf) - Ltrf * cos(O3rf);
  Wb = -s * sin(O3rb + O4rb) - Ltrb * cos(O3rb);


 _H <<  - (Pr*Rl*Rr*(L2*cos(O0lf + O1lf) + L1*cos(O0lf) - L*sin(O0lf + O1lf + O2lf)))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) - (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0, - (L2*sin(O0lf + O1lf))/4 - (L*cos(O0lf + O1lf + O2lf))/4 - (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - (L1*sin(O0lf))/4 - ((L2*cos(O0lf + O1lf) + L1*cos(O0lf) - L*sin(O0lf + O1lf + O2lf))*(Pr*Ql*Rr - Pr*Ql*Rl + Ql*Rl*Rr + Pr*Rl*Sr + Pr*Rr*Sl + Qr*Rl*Rr))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr), - (Pl*Pr*Rr + Pl*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) - ((Pr*Rr + Rl*Rr)*(L2*cos(O0lf + O1lf) + L1*cos(O0lf) - L*sin(O0lf + O1lf + O2lf)))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                       - (Pr*Rl*Rr*(L2*cos(O0lf + O1lf) - L*sin(O0lf + O1lf + O2lf)))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) - (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,                                   - (L2*sin(O0lf + O1lf))/4 - (L*cos(O0lf + O1lf + O2lf))/4 - (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - ((L2*cos(O0lf + O1lf) - L*sin(O0lf + O1lf + O2lf))*(Pr*Ql*Rr - Pr*Ql*Rl + Ql*Rl*Rr + Pr*Rl*Sr + Pr*Rr*Sl + Qr*Rl*Rr))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr),                - (Pl*Pr*Rr + Pl*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) - ((Pr*Rr + Rl*Rr)*(L2*cos(O0lf + O1lf) - L*sin(O0lf + O1lf + O2lf)))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                                                 (L*Pr*Rl*Rr*sin(O0lf + O1lf + O2lf))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) - (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,                                                                                       (L*sin(O0lf + O1lf + O2lf)*(Pr*Ql*Rr - Pr*Ql*Rl + Ql*Rl*Rr + Pr*Rl*Sr + Pr*Rr*Sl + Qr*Rl*Rr))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - (L*cos(O0lf + O1lf + O2lf))/4,                                          (L*sin(O0lf + O1lf + O2lf)*(Pr*Rr + Rl*Rr))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) - (Pl*Pr*Rr + Pl*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                                                                                                                                                                                            0,   (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf) - (Ub*Wb*Wf*(s*sin(O3lf + O4lf) - Ltlf*cos(O3lf)))/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                    0,   (Ub*Uf*Wb + Uf*Wb*Wf)/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf) - ((s*sin(O3lf + O4lf) - Ltlf*cos(O3lf))*(Ub*Wb + Wb*Wf))/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf),
                                                                                                                                                                                            0,                      (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf) - (Ub*Wb*Wf*s*sin(O3lf + O4lf))/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                    0,                      (Ub*Uf*Wb + Uf*Wb*Wf)/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf) - (s*sin(O3lf + O4lf)*(Ub*Wb + Wb*Wf))/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf),
          (Pl*Rl*Rr*(L2*cos(O0rf + O1rf) + L1*cos(O0rf) - L*sin(O0rf + O1rf + O2rf)))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) + (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,   (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - (L*cos(O0rf + O1rf + O2rf))/4 - (L2*sin(O0rf + O1rf))/4 - (L1*sin(O0rf))/4 + ((L2*cos(O0rf + O1rf) + L1*cos(O0rf) - L*sin(O0rf + O1rf + O2rf))*(Pl*Qr*Rl - Pl*Qr*Rr + Pl*Rl*Sr + Pl*Rr*Sl + Ql*Rl*Rr + Qr*Rl*Rr))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr),   (Pl*Pr*Rl + Pr*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) + ((Pl*Rl + Rl*Rr)*(L2*cos(O0rf + O1rf) + L1*cos(O0rf) - L*sin(O0rf + O1rf + O2rf)))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                         (Pl*Rl*Rr*(L2*cos(O0rf + O1rf) - L*sin(O0rf + O1rf + O2rf)))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) + (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,                                     (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - (L*cos(O0rf + O1rf + O2rf))/4 - (L2*sin(O0rf + O1rf))/4 + ((L2*cos(O0rf + O1rf) - L*sin(O0rf + O1rf + O2rf))*(Pl*Qr*Rl - Pl*Qr*Rr + Pl*Rl*Sr + Pl*Rr*Sl + Ql*Rl*Rr + Qr*Rl*Rr))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr),                  (Pl*Pr*Rl + Pr*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) + ((Pl*Rl + Rl*Rr)*(L2*cos(O0rf + O1rf) - L*sin(O0rf + O1rf + O2rf)))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                                                 (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) - (L*Pl*Rl*Rr*sin(O0rf + O1rf + O2rf))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,                                                                                       (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - (L*cos(O0rf + O1rf + O2rf))/4 - (L*sin(O0rf + O1rf + O2rf)*(Pl*Qr*Rl - Pl*Qr*Rr + Pl*Rl*Sr + Pl*Rr*Sl + Ql*Rl*Rr + Qr*Rl*Rr))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr),                                          (Pl*Pr*Rl + Pr*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) - (L*sin(O0rf + O1rf + O2rf)*(Pl*Rl + Rl*Rr))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                                                                                                                                                                                            0, - (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf) - (Ub*Uf*Wb*(s*sin(O3rf + O4rf) + Ltrf*cos(O3rf)))/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                    0, - (Ub*Uf*Wf + Ub*Wb*Wf)/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf) - ((s*sin(O3rf + O4rf) + Ltrf*cos(O3rf))*(Ub*Uf + Ub*Wb))/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf),
                                                                                                                                                                                            0,                    - (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf) - (Ub*Uf*Wb*s*sin(O3rf + O4rf))/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                    0,                    - (Ub*Uf*Wf + Ub*Wb*Wf)/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf) - (s*sin(O3rf + O4rf)*(Ub*Uf + Ub*Wb))/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf),
          (Pl*Pr*Rr*(L2*cos(O0lb + O1lb) + L1*cos(O0lb) + L*sin(O0lb + O1lb + O2lb)))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) + (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,   (L*cos(O0lb + O1lb + O2lb))/4 - (L2*sin(O0lb + O1lb))/4 + (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - (L1*sin(O0lb))/4 + ((L2*cos(O0lb + O1lb) + L1*cos(O0lb) + L*sin(O0lb + O1lb + O2lb))*(Pl*Pr*Sl + Pl*Pr*Sr + Pl*Qr*Rr + Pr*Ql*Rr - Pl*Rr*Sl + Pr*Rr*Sl))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr),   (Pl*Pr*Rl + Pr*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) + ((Pl*Pr + Pr*Rr)*(L2*cos(O0lb + O1lb) + L1*cos(O0lb) + L*sin(O0lb + O1lb + O2lb)))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                         (Pl*Pr*Rr*(L2*cos(O0lb + O1lb) + L*sin(O0lb + O1lb + O2lb)))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) + (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,                                     (L*cos(O0lb + O1lb + O2lb))/4 - (L2*sin(O0lb + O1lb))/4 + (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) + ((L2*cos(O0lb + O1lb) + L*sin(O0lb + O1lb + O2lb))*(Pl*Pr*Sl + Pl*Pr*Sr + Pl*Qr*Rr + Pr*Ql*Rr - Pl*Rr*Sl + Pr*Rr*Sl))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr),                  (Pl*Pr*Rl + Pr*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) + ((Pl*Pr + Pr*Rr)*(L2*cos(O0lb + O1lb) + L*sin(O0lb + O1lb + O2lb)))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                                                 (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) + (L*Pl*Pr*Rr*sin(O0lb + O1lb + O2lb))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,                                                                                       (L*cos(O0lb + O1lb + O2lb))/4 + (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) + (L*sin(O0lb + O1lb + O2lb)*(Pl*Pr*Sl + Pl*Pr*Sr + Pl*Qr*Rr + Pr*Ql*Rr - Pl*Rr*Sl + Pr*Rr*Sl))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr),                                          (Pl*Pr*Rl + Pr*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) + (L*sin(O0lb + O1lb + O2lb)*(Pl*Pr + Pr*Rr))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                                                                                                                                                                                            0,   (Uf*Wb*Wf*(s*sin(O3lb + O4lb) - Ltlb*cos(O3lb)))/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf) - (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                    0,   ((s*sin(O3lb + O4lb) - Ltlb*cos(O3lb))*(Uf*Wf + Wb*Wf))/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf) - (Ub*Uf*Wf + Ub*Wb*Wf)/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf),
                                                                                                                                                                                            0,                      (Uf*Wb*Wf*s*sin(O3lb + O4lb))/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf) - (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                    0,                      (s*sin(O3lb + O4lb)*(Uf*Wf + Wb*Wf))/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf) - (Ub*Uf*Wf + Ub*Wb*Wf)/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf),
        - (Pl*Pr*Rl*(L2*cos(O0rb + O1rb) + L1*cos(O0rb) + L*sin(O0rb + O1rb + O2rb)))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) - (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,   (L*cos(O0rb + O1rb + O2rb))/4 - (L2*sin(O0rb + O1rb))/4 - (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - (L1*sin(O0rb))/4 - ((L2*cos(O0rb + O1rb) + L1*cos(O0rb) + L*sin(O0rb + O1rb + O2rb))*(Pl*Pr*Sl + Pl*Qr*Rl + Pr*Ql*Rl + Pl*Pr*Sr + Pl*Rl*Sr - Pr*Rl*Sr))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr), - (Pl*Pr*Rr + Pl*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) - ((Pl*Pr + Pl*Rl)*(L2*cos(O0rb + O1rb) + L1*cos(O0rb) + L*sin(O0rb + O1rb + O2rb)))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                       - (Pl*Pr*Rl*(L2*cos(O0rb + O1rb) + L*sin(O0rb + O1rb + O2rb)))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) - (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,                                     (L*cos(O0rb + O1rb + O2rb))/4 - (L2*sin(O0rb + O1rb))/4 - (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - ((L2*cos(O0rb + O1rb) + L*sin(O0rb + O1rb + O2rb))*(Pl*Pr*Sl + Pl*Qr*Rl + Pr*Ql*Rl + Pl*Pr*Sr + Pl*Rl*Sr - Pr*Rl*Sr))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr),                - (Pl*Pr*Rr + Pl*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) - ((Pl*Pr + Pl*Rl)*(L2*cos(O0rb + O1rb) + L*sin(O0rb + O1rb + O2rb)))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                                               - (Pl*Pr*Rl*Rr)/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr) - (L*Pl*Pr*Rl*sin(O0rb + O1rb + O2rb))/(Pl*Pr*Rl - Pl*Pr*Rr - Pl*Rl*Rr + Pr*Rl*Rr),                                                                                                                                                          0,                                                                                       (L*cos(O0rb + O1rb + O2rb))/4 - (Pl*Pr*Rl*Sr + Pl*Pr*Rr*Sl + Pl*Qr*Rl*Rr + Pr*Ql*Rl*Rr)/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr) - (L*sin(O0rb + O1rb + O2rb)*(Pl*Pr*Sl + Pl*Qr*Rl + Pr*Ql*Rl + Pl*Pr*Sr + Pl*Rl*Sr - Pr*Rl*Sr))/(4*Pl*Pr*Rl - 4*Pl*Pr*Rr - 4*Pl*Rl*Rr + 4*Pr*Rl*Rr),                                        - (Pl*Pr*Rr + Pl*Rl*Rr)/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr) - (L*sin(O0rb + O1rb + O2rb)*(Pl*Pr + Pl*Rl))/(2*Pl*Pr*Rl - 2*Pl*Pr*Rr - 2*Pl*Rl*Rr + 2*Pr*Rl*Rr),                                                                                                                                                                                         0,
                                                                                                                                                                                            0,   (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf) + (Ub*Uf*Wf*(s*sin(O3rb + O4rb) + Ltrb*cos(O3rb)))/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                    0,   (Ub*Uf*Wb + Uf*Wb*Wf)/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf) + ((s*sin(O3rb + O4rb) + Ltrb*cos(O3rb))*(Ub*Uf + Uf*Wf))/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf),
                                                                                                                                                                                            0,                      (Ub*Uf*Wb*Wf)/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf) + (Ub*Uf*Wf*s*sin(O3rb + O4rb))/(Ub*Uf*Wb - Ub*Uf*Wf - Ub*Wb*Wf + Uf*Wb*Wf),                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                    0,                      (Ub*Uf*Wb + Uf*Wb*Wf)/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf) + (s*sin(O3rb + O4rb)*(Ub*Uf + Uf*Wf))/(2*Ub*Uf*Wb - 2*Ub*Uf*Wf - 2*Ub*Wb*Wf + 2*Uf*Wb*Wf);


 std::cout <<"H out " << std::endl << _H << std::endl;
}





/**
 * @brief 力矩计算
 *
 * @param _H
 * @param lfTorque
 * @param rfTorque
 * @param lbTorque
 * @param rfTorque
 */
void TorqueCalculate(Eigen::MatrixXd &_H, Eigen::MatrixXd &_F, const Spring_Damper &sdPara, double lfTorque[4], double rfTorque[4], double lbTorque[4], double rbTorque[4])
{
  static Eigen::MatrixXd torMat(20, 1);

  static Eigen::MatrixXd KMat = Eigen::MatrixXd::Zero(5, 5), BMat = Eigen::MatrixXd::Zero(5, 5), Pose(5, 1), Veloc(5, 1);

  KMat(0, 0) = sdPara.Kx;
  KMat(1, 1) = sdPara.Ky;
  KMat(2, 2) = sdPara.Kz;
  KMat(3, 3) = sdPara.Kalpha;
  KMat(4, 4) = sdPara.Kbeta;
  BMat(0, 0) = sdPara.Bx;
  BMat(1, 1) = sdPara.By;
  BMat(2, 2) = sdPara.Bz;
  BMat(3, 3) = sdPara.Balpha;
  BMat(4, 4) = sdPara.Bbeta;

  Pose << (vmcspace::abs(sdPara.xd - sdPara.xcurr) > errorMin ? sdPara.xd - sdPara.xcurr : 0),
          (vmcspace::abs(sdPara.yd - sdPara.ycurr) > errorMin ? sdPara.yd - sdPara.ycurr : 0),
          (vmcspace::abs(sdPara.zd - sdPara.zcurr) > errorMin ? sdPara.zd - sdPara.zcurr : 0),
          (vmcspace::abs(sdPara.alphad - sdPara.alphacurr) > errorMin ? sdPara.alphad - sdPara.alphacurr : 0),
          (vmcspace::abs(sdPara.betad - sdPara.betacurr) > errorMin ? sdPara.betad - sdPara.betacurr : 0);

  Veloc << (vmcspace::abs(sdPara.diffxd - sdPara.diffxcurr) > errorMin ? sdPara.diffxd - sdPara.diffxcurr : 0),
           (vmcspace::abs(sdPara.diffyd - sdPara.diffycurr) > errorMin ? sdPara.diffyd - sdPara.diffycurr : 0),
           (vmcspace::abs(sdPara.diffzd - sdPara.diffzcurr) > errorMin ? sdPara.diffzd - sdPara.diffzcurr : 0),
           (vmcspace::abs(sdPara.diffalphad - sdPara.diffalphacurr) > errorMin ? sdPara.diffalphad - sdPara.diffalphacurr : 0),
           (vmcspace::abs(sdPara.diffbetad - sdPara.diffbetacurr) > errorMin ? sdPara.diffbetad - sdPara.diffbetacurr : 0);

  _F = KMat * Pose + BMat * Veloc;

  std::cout << "H In " << std::endl << _H << std::endl;
  std::cout << "pose Matrix:" << std::endl << Pose << std::endl;
  std::cout << "Veloc Matrix:" << std::endl << Veloc << std::endl;
  std::cout << "K Matrix:" << std::endl << KMat << std::endl;
  std::cout << "B Matrix:" << std::endl << BMat << std::endl;
  std::cout << "F Matrix:" << std::endl << _F << std::endl;

  torMat = _H * _F;
  
  lfTorque[(int)JointEnumdef::LegDown] = -torMat(1, 0);
  lfTorque[(int)JointEnumdef::LegUp] = -torMat(2, 0);
  lfTorque[(int)JointEnumdef::Shoulder] = torMat(4, 0);
  rfTorque[(int)JointEnumdef::LegDown] = torMat(1+5, 0);
  rfTorque[(int)JointEnumdef::LegUp] = torMat(2+5, 0);
  rfTorque[(int)JointEnumdef::Shoulder] = torMat(4+5, 0);
  lbTorque[(int)JointEnumdef::LegDown] = -torMat(1+10, 0);
  lbTorque[(int)JointEnumdef::LegUp] = -torMat(2+10, 0);
  lbTorque[(int)JointEnumdef::Shoulder] = -torMat(4+10, 0);
  rbTorque[(int)JointEnumdef::LegDown] = torMat(1+15, 0);
  rbTorque[(int)JointEnumdef::LegUp] = torMat(2+15, 0);
  rbTorque[(int)JointEnumdef::Shoulder] = -torMat(4+15, 0);

  std::cout << "tormat: " << torMat << std::endl;
}

