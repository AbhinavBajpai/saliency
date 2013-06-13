#include "EIF.h"

EIF::EIF()
{
}

EIF::EIF(int matsz, double oderr, double x, double y, double z, double yaw, int s, int ms)
{
  std::cerr << "Initialising EIF...\n";

  lambda = new Mat;
  eta = new Mat;
  mu = new Mat;
  Q = new Mat;
  R = new Mat;
  S = new int;
  MS = new int;

  *S = s;
  *MS = ms;

  idx = *S;

  Range r(0, idx);

  (*lambda) = Mat::zeros(matsz, matsz, CV_64F);
  (*eta) = Mat::zeros(matsz, 1, CV_64F);
  (*mu) = Mat::zeros(matsz, 1, CV_64F);

  for (int i = 0; i < *S; i++)
  {
    (*lambda).at<double>(i, i) = 1000;
  }

  (*mu).at<double>(0, 0) = x;
  (*mu).at<double>(1, 0) = y;
  (*mu).at<double>(2, 0) = z;
  //(*mu).at<double>(3,0) = yaw;

  Mat temp1 = (*lambda)(r, r) * (*mu).rowRange(r);
  Mat temp2 = (*eta).rowRange(r);
  temp1.copyTo(temp2);

  double var = pow(oderr, 2); // noise covariance

  (*Q) = Mat::zeros(*S, *S, CV_64F);
  (*R) = Mat::zeros(*MS, *MS, CV_64F);

  for (int i = 0; i < *S; i++)
  {
    (*Q).at<double>(i, i) = var;
  }

  (*Q).at<double>(2, 2) = pow(0.001, 2);

  for (int i = 0; i < *MS; i++)
  {
    (*R).at<double>(i, i) = 0.001;
  }
}

EIF::~EIF()
{
  delete lambda, delete eta, delete mu, delete Q, delete R, delete S, delete MS;
}

void EIF::init(int matsz, double oderr, double x, double y, double z, double yaw, int s, int ms)
{
  std::cerr << "Initialising EIF...\n";

  lambda = new Mat;
  eta = new Mat;
  mu = new Mat;
  Q = new Mat;
  R = new Mat;
  S = new int;
  MS = new int;

  *S = s;
  *MS = ms;

  idx = *S;

  Range r(0, idx);

  (*lambda) = Mat::zeros(matsz, matsz, CV_64F);
  (*eta) = Mat::zeros(matsz, 1, CV_64F);
  (*mu) = Mat::zeros(matsz, 1, CV_64F);

  for (int i = 0; i < *S; i++)
  {
    (*lambda).at<double>(i, i) = 1000;
  }

  (*mu).at<double>(0, 0) = x;
  (*mu).at<double>(1, 0) = y;
  (*mu).at<double>(2, 0) = z;
  //(*mu).at<double>(3,0) = yaw;

  Mat temp1 = (*lambda)(r, r) * (*mu).rowRange(r);
  Mat temp2 = (*eta).rowRange(r);
  temp1.copyTo(temp2);

  double var = pow(oderr, 2);

  (*Q) = Mat::zeros(*S, *S, CV_64F);
  (*R) = Mat::zeros(*MS, *MS, CV_64F);

  for (int i = 0; i < *S; i++)
  {
    (*Q).at<double>(i, i) = var;
  }

  (*Q).at<double>(2, 2) = pow(0.001, 2);

  for (int i = 0; i < *MS; i++)
  {
    (*R).at<double>(i, i) = var;
  }
}

void EIF::predict(Mat u)
{
  Range r(0, *S);
  Range rend(0, idx);

  Mat A = Mat::eye(*S, *S, CV_64F); // motion Jacobian

  Mat Qinv = (*Q).inv(); // inverse covariance

  Mat fx = Mat(*S, 1, CV_64F);

  for (int i = 0; i < *S; i++)
  {
    fx.at<double>(i, 0) = (*mu).at<double>(i, 0) + u.at<double>(i, 0);
  }

  Mat psi = ((*Q) + A * (*lambda)(r, r).inv() * A.t()).inv();

  Mat omegainv = ((*lambda)(r, r) + A * Qinv * A.t()).inv();

  Mat temp1 = Qinv * A * omegainv * (*eta).rowRange(r);
  Mat temp2 = psi * (fx - A * (*mu).rowRange(r));
  Mat eta1 = temp1 + temp2;

  Mat Temp;

  if (idx > *S)
  {
    Mat eta2 = (*eta).rowRange(*S, idx)
        - (*lambda)(Range(*S, idx), r) * omegainv * ((*eta).rowRange(r) - A.t() * Qinv * (fx - A * (*mu).rowRange(r)));

    Mat lambda12 = Qinv * A * omegainv * (*lambda)(r, Range(*S, idx));
    Mat lambda22 = (*lambda)(Range(*S, idx), Range(*S, idx))
        - (*lambda)(Range(*S, idx), r) * omegainv * (*lambda)(r, Range(*S, idx));

    Temp = (*eta).rowRange(r);
    eta1.copyTo(Temp);
    Temp = (*eta).rowRange(*S, idx);
    eta2.copyTo(Temp);

    Temp = (*lambda)(r, r);
    psi.copyTo(Temp);
    Temp = (*lambda)(r, Range(*S, idx));
    lambda12.copyTo(Temp);
    Mat lambda12t = lambda12.t();
    Temp = (*lambda)(Range(*S, idx), r);
    lambda12t.copyTo(Temp);
    Temp = (*lambda)(Range(*S, idx), Range(*S, idx));
    lambda22.copyTo(Temp);
  }
  else
  {
    Temp = (*eta).rowRange(r);
    eta1.copyTo(Temp);
    Temp = (*lambda)(r, r);
    psi.copyTo(Temp);
  }
}

void EIF::update(vector<CvPoint3D64f> vf, vector<double> vr, vector<int> *id)
{
  if (vf.size() > 0)
  {
    Range r(0, *S);

    Mat H = Mat::zeros((*MS) * vf.size(), (*MS) * vf.size() + (*S), CV_64F);
    Mat H1 = Mat::eye(*MS, *S, CV_64F);
    Mat H2 = (Mat_<double>(*MS, *MS) << -1, 0, 0, 0, -1, 0, 0, 0, -1);

    Mat RR = Mat::zeros((*MS) * vf.size(), (*MS) * vf.size(), CV_64F);

    Mat z(vf.size() * (*MS), 1, CV_64F);

    Mat zp(vf.size() * (*MS), 1, CV_64F);

    Mat Temp;

    for (unsigned int i = 0; i < vf.size(); i++) //PW
    {
      Temp = H(Range(i * (*MS), i * (*MS) + (*MS)), r);
      H2.copyTo(Temp);
      Temp = H(Range(i * (*MS), i * (*MS) + (*MS)), Range(i * (*MS) + (*S), i * (*MS) + (*S) + (*MS)));
      H1.copyTo(Temp);

      for (int j = 0; j < *MS; j++)
      {
        (*R).at<double>(j, j) = vr[i] * vr[i];
      }

      Temp = RR(Range(i * (*MS), i * (*MS) + (*MS)), Range(i * (*MS), i * (*MS) + (*MS)));
      (*R).copyTo(Temp);

      z.at<double>(i * (*MS), 0) = vf[i].x;
      z.at<double>(i * (*MS) + 1, 0) = vf[i].y;
      z.at<double>(i * (*MS) + 2, 0) = vf[i].z;

      zp.at<double>(i * (*MS), 0) = (*mu).at<double>((*id)[i] * (*MS) + (*S), 0) - (*mu).at<double>(0, 0); // xp
      zp.at<double>(i * (*MS) + 1, 0) = (*mu).at<double>((*id)[i] * (*MS) + (*S) + 1, 0) - (*mu).at<double>(1, 0); // yp
      zp.at<double>(i * (*MS) + 2, 0) = (*mu).at<double>((*id)[i] * (*MS) + (*S) + 2, 0) - (*mu).at<double>(2, 0); // zp

    }

    Mat Rinv = RR.inv();
    Mat v = z - zp; //innovation

    Mat lambdatemp = Mat::zeros(vf.size() * (*MS) + (*S), vf.size() * (*MS) + (*S), CV_64F);
    Mat etatemp = Mat::zeros(vf.size() * (*MS) + (*S), 1, CV_64F);
    Mat mutemp = Mat::zeros(vf.size() * (*MS) + (*S), 1, CV_64F);

    Temp = lambdatemp(r, r);
    (*lambda)(r, r).copyTo(Temp);
    Temp = etatemp.rowRange(r);
    (*eta).rowRange(r).copyTo(Temp);
    Temp = mutemp.rowRange(r);
    (*mu).rowRange(r).copyTo(Temp);

    for (unsigned int i = 0; i < vf.size(); i++) //PW
    {
      Range idr((*id)[i] * (*MS) + (*S), (*id)[i] * (*MS) + (*S) + (*MS));
      Range idr2(i * (*MS) + (*S), i * (*MS) + (*MS) + (*S));

      Temp = lambdatemp(r, idr2);
      (*lambda)(r, idr).copyTo(Temp);
      Temp = lambdatemp(idr2, r);
      (*lambda)(idr, r).copyTo(Temp);
      Temp = lambdatemp(idr2, idr2);
      (*lambda)(idr, idr).copyTo(Temp);

      Temp = etatemp.rowRange(idr2);
      (*eta).rowRange(idr).copyTo(Temp);

      Temp = mutemp.rowRange(idr2);
      (*mu).rowRange(idr).copyTo(Temp);

    }

    Mat HR = lambdatemp + H.t() * Rinv * H;
    Mat HV = etatemp + H.t() * Rinv * (v + H * mutemp);

    Temp = (*lambda)(r, r);
    HR(r, r).copyTo(Temp);
    Temp = (*eta).rowRange(r);
    HV.rowRange(r).copyTo(Temp);

    for (unsigned int i = 0; i < vf.size(); i++) //PW
    {
      Range idr((*id)[i] * (*MS) + (*S), (*id)[i] * (*MS) + (*S) + (*MS));
      Range idr2(i * (*MS) + (*S), i * (*MS) + (*MS) + (*S));

      Temp = (*lambda)(r, idr);
      HR(r, idr2).copyTo(Temp);
      Temp = (*lambda)(idr, r);
      HR(idr2, r).copyTo(Temp);
      Temp = (*lambda)(idr, idr);
      HR(idr2, idr2).copyTo(Temp);

      Temp = (*eta).rowRange(idr);
      HV.rowRange(idr2).copyTo(Temp);
    }
  }
}

void EIF::augment(vector<CvPoint3D64f> vn, vector<double> vr)
{
  if (vn.size() > 0)
  {
    Mat zn(*MS, 1, CV_64F);

    Range r(0, *S);

    Mat Rinv(*MS, *MS, CV_64F);
    Mat H = Mat::eye(*MS, *S, CV_64F);

    for (unsigned int i = 0; i < vn.size(); i++) //PW
    {
      zn.at<double>(0, 0) = vn[i].x;
      zn.at<double>(1, 0) = vn[i].y;
      zn.at<double>(2, 0) = vn[i].z;

      Range ri(idx, idx + (*MS));

      for (int j = 0; j < *MS; j++)
      {
        (*R).at<double>(j, j) = vr[i] * vr[i];
      }

      Rinv = (*R).inv();

      Mat HT = -H.t() * Rinv;
      Mat Temp1 = (*lambda)(r, ri); // introduced as a middle step for G++ compatibility
      HT.copyTo(Temp1);

      Mat RH = -Rinv * H;
      Mat Temp2 = (*lambda)(ri, r); // introduced as a middle step for G++ compatibility
      RH.copyTo(Temp2);

      Mat Temp3 = (*lambda)(ri, ri); // introduced as a middle step for G++ compatibility
      Rinv.copyTo(Temp3);

      Mat SH = (*lambda)(r, r) + H.t() * Rinv * H;
      Mat Temp4 = (*lambda)(r, r); // introduced as a middle step for G++ compatibility
      SH.copyTo(Temp4);

      Mat etan = Rinv * (zn + (*mu).rowRange(r) - H * (*mu).rowRange(r));
      Mat Temp5 = (*eta).rowRange(idx, idx + (*MS)); // introduced as a middle step for G++ compatibility
      etan.copyTo(Temp5);

      Mat etx = (*eta).rowRange(r) - H.t() * Rinv * (zn + (*mu).rowRange(r) - H * (*mu).rowRange(r));
      Mat Temp6 = (*eta).rowRange(r); // introduced as a middle step for G++ compatibility
      etx.copyTo(Temp6);

      idx = idx + (*MS);

      HT.~Mat();
      RH.~Mat();
      SH.~Mat();
      etan.~Mat();
      etx.~Mat();
      Temp1.~Mat();
      Temp2.~Mat();
      Temp3.~Mat();
      Temp4.~Mat();
      Temp5.~Mat();
      Temp6.~Mat();
    }
  }
}

void EIF::recoverMean()
{
  Range r(0, idx);
  Mat temp1 = (*lambda)(r, r).inv() * (*eta).rowRange(r);
  Mat temp2 = (*mu).rowRange(r);
  temp1.copyTo(temp2);
}

// future implementation - disregard
void EIF::marginalise(vector<int> *id)
{
  /*Y(ii(4:end),ii(4:end)) = Y(4:ii(end), 4:ii(end)) - Y(4:ii(end),1:3)*inv(Y(1:3,1:3))*Y(1:3,4:ii(end));
   y(ii(4:end)) = y(4:ii(end)) - Y(4:ii(end),1:3)*inv(Y(1:3,1:3))*y(1:3);*/
}

// future implementation - disregard
void EIF::relocalise(vector<CvPoint3D64f> vb, vector<int> *id)
{
  Range r(0, *S);
  Mat temp1;
  Mat temp2;

  //	function relocalise(zb, idb, Q, x, da_table)
  //
  //global Y y ii;
  //
  //lenzb = length(idb);
  //
  //G = zeros(3,length(ii));
  Mat G = Mat::zeros(*S, idx, CV_64F);

  //if lenzb ~= 0
  //    for i=1:lenzb
  for (unsigned int i = 0; i < vb.size(); i++) //PW
  {
    //jj = [3+2*da_table(idb(i))-1;3+2*da_table(idb(i))];
    Range idr((*id)[i] * (*MS) + (*S), (*id)[i] * (*MS) + (*S) + (*MS));

    //    G(1:3,jj)  =   [[1 1 0]' [0 0 1]'];
  }

  //    G = G(1:3,4:end);

  Mat Rinv = (*Q).inv();

  //Y(1:3,ii(4:end)) = -Rinv*G;
  //Y(ii(4:end),1:3) = -G'*Rinv;
  //Y(1:3,1:3) = Rinv;
  //Y(ii(4:end),ii(4:end)) = Y(ii(4:end),ii(4:end)) + G'*Rinv*G;

  temp1 = -Rinv * G;
  temp2 = (*lambda)(r, Range(*S, idx));
  temp1.copyTo(temp2);

  temp1 = -G.t() * Rinv;
  temp2 = (*lambda)(Range(*S, idx), r);
  temp1.copyTo(temp2);

  temp2 = (*lambda)(r, r);
  Rinv.copyTo(temp2);

  temp1 = (*lambda)(Range(*S, idx), Range(*S, idx)) + G.t() * Rinv * G;
  temp2 = (*lambda)(Range(*S, idx), Range(*S, idx));
  temp1.copyTo(temp2);

  //y(1:3) = Rinv*(x(1:3)-G*x(ii(4:end)));
  //y(ii(4:end)) = y(ii(4:end)) - G'*Rinv*(x(1:3)-G*x(ii(4:end)));

  temp1 = Rinv * ((*mu).rowRange(r) - G * (*mu).rowRange(Range(*S, idx)));
  temp2 = (*eta).rowRange(r);
  temp1.copyTo(temp2);

  temp1 = (*eta).rowRange(Range(*S, idx)) - (G.t() * Rinv * ((*mu).rowRange(r) - G * (*mu).rowRange(Range(*S, idx))));
  temp2 = (*eta).rowRange(Range(*S, idx));
  temp1.copyTo(temp2);
}
