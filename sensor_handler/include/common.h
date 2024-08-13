#pragma once

#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ctime>
#include <cstdlib>
#include <chrono>

typedef pcl::PointXYZINormal PointType;

inline double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

Eigen::Vector3f pi2pi(Eigen::Vector3f degree)
{
    for (int i = 0; i < 3; i++)
    {
        if (degree(i) > 180)  degree(i) -= 360;
        else if (degree(i) < -180)    degree(i) += 360;
    }
    return degree;
}

Eigen::Vector3f pi2piRad(Eigen::Vector3f radian)
{
    for (int i = 0; i < 3; i++)
    {
        if (radian(i) > M_PI)  radian(i) -= 2 * M_PI;
        else if (radian(i) < -M_PI)    radian(i) += 2 * M_PI;
    }
    return radian;
}

double pi2piRad(double radian)
{
    if (radian > M_PI)  radian -= 2 * M_PI;
    else if (radian < -M_PI)    radian += 2 * M_PI;

    return radian;
}

double pi2piDeg(double degree)
{
    if (degree > 180)  degree -= 360;
    else if (degree < -180)    degree += 360;

    return degree;
}

// Function to find mean.
Eigen::Vector3d getMean(pcl::PointCloud<PointType> points)
{
    Eigen::Vector3d sum;
    sum.setZero();
    for (size_t i = 0; i < points.size(); i++)
    {
        sum(0) = sum(0) + points.points[i].x;
        sum(1) = sum(1) + points.points[i].y;
        sum(2) = sum(2) + points.points[i].z;
    }
    sum /= points.size();
    return sum;
}

// Function to find covariance.
Eigen::Matrix3d getCovariance(pcl::PointCloud<PointType> points)
{
    Eigen::Vector3d meanPoint = getMean(points);
    Eigen::Matrix3d cov;
    cov.setZero();
    for (size_t i = 0; i < points.size(); i++)
    {
        cov(0,0) += (points.points[i].x - meanPoint(0))*(points.points[i].x - meanPoint(0));
        cov(0,1) += (points.points[i].x - meanPoint(0))*(points.points[i].y - meanPoint(1));
        cov(0,2) += (points.points[i].x - meanPoint(0))*(points.points[i].z - meanPoint(2));
        cov(1,0) += (points.points[i].y - meanPoint(1))*(points.points[i].x - meanPoint(0));
        cov(1,1) += (points.points[i].y - meanPoint(1))*(points.points[i].y - meanPoint(1));
        cov(1,2) += (points.points[i].y - meanPoint(1))*(points.points[i].z - meanPoint(2));
        cov(2,0) += (points.points[i].z - meanPoint(2))*(points.points[i].x - meanPoint(0));
        cov(2,1) += (points.points[i].z - meanPoint(2))*(points.points[i].y - meanPoint(1));
        cov(2,2) += (points.points[i].z - meanPoint(2))*(points.points[i].z - meanPoint(2));
    }
    return cov / (points.size() - 1);
}

Eigen::Vector3d llh2xyz(Eigen::Vector3d gps_llh)
{
    double phi = gps_llh(0) * M_PI/180;
    double lambda = gps_llh(1) * M_PI/180;
    double h = gps_llh(2);

    double a = 6378137.000;
    double b = 6356752.3142;
    double e = sqrt(1-pow((b/a),2));

    double sinphi = sin(phi);
    double cosphi = cos(phi);
    double coslam = cos(lambda);
    double sinlam = sin(lambda);
    double tan2phi = pow(tan(phi),2);
    double tmp = 1 - pow(e,2);
    double tmpden = sqrt(1+tmp*tan2phi);

    double x = (a*coslam)/tmpden + h*coslam*cosphi;
    double y = (a*sinlam)/tmpden + h*sinlam*cosphi;

    double tmp2 = sqrt(1-pow(e,2)*pow(sinphi,2));
    double z = (a*tmp*sinphi)/tmp2 + h*sinphi;

    Eigen::Vector3d xyz_result;
    xyz_result(0) = x;  xyz_result(1) = y;  xyz_result(2) = z;
    return xyz_result;
}

Eigen::Vector3d xyz2llh (Eigen::Vector3d gps_xyz)
{
    double x = gps_xyz(0);
    double y = gps_xyz(1);
    double z = gps_xyz(2);
    double x2 = pow(x,2);   double y2 = pow(y,2);   double z2 = pow(z,2);
    double a = 6378137.000;
    double b = 6356752.3142;
    double e = sqrt(1-pow((b/a),2));
    double b2 = pow(b,2);   double e2 = pow(e,2);
    double ep = e*(a/b);
    double r = sqrt(x2+y2);
    double r2 = pow(r,2);
    double E2 = pow(a,2) - pow(b,2);
    double F = 54*b2*z2;
    double G = r2+(1-e2)*z2 - e2*E2;
    double c = (pow(e2,2)*F*r2)/(pow(G,3));
    double third = 1/3;
    double s = pow((1+c+sqrt(pow(c,2)+2*c)),third);
    double P = F/(3*pow((s+1/s+1),2)*pow(G,2));
    double Q = sqrt(1+2*e2*e2*P);
    double ro = -(P*e2*r)/(1+Q) + sqrt((a*a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2);

    double tmp = (r - e2*ro)*(r - e2*ro);
    double U = sqrt(tmp+z2);
    double V = sqrt(tmp + (1-e2)*z2);
    double zo = (b2*z)/(a*V);
    double height = U*(1-b2/(a*V));

    double lat = atan((z+ep*ep*zo)/r);
    double temp = atan(y/x);

    double longi;
    if (x>=0)
    {
        longi = temp;
    }
    else if (x < 0 && y >= 0)
    {
        longi = M_PI + temp;
    }
    else
    {
        longi = temp - M_PI;
    }
    Eigen::Vector3d llh_result;
    llh_result(0) = lat * 180/M_PI;
    llh_result(1) = longi * 180/M_PI;
    llh_result(2) = height;

    return llh_result;
}

Eigen::Vector3d xyz2enu(Eigen::Vector3d gps_xyz, Eigen::Vector3d origin_xyz)
{
    Eigen::Vector3d tmpxyz = gps_xyz;
    Eigen::Vector3d tmporg = origin_xyz;
    Eigen::Vector3d diff_xyz;
    diff_xyz(0) = tmpxyz(0) - tmporg(0);
    diff_xyz(1) = tmpxyz(1) - tmporg(1);
    diff_xyz(2) = tmpxyz(2) - tmporg(2);

    Eigen::Vector3d origllh;
    origllh = xyz2llh(origin_xyz);
    double phi = origllh(0) * M_PI/180;
    double lam = origllh(1) * M_PI/180;
    double sinphi = sin(phi);
    double cosphi = cos(phi);
    double sinlam = sin(lam);
    double coslam = cos(lam);

    Eigen::Matrix3d R;
    R(0,0) = -sinlam;   R(0,1) = coslam;    R(0,2) = 0;
    R(1,0) = -sinphi*coslam;    R(1,1) = -sinphi*sinlam;    R(1,2) = cosphi;
    R(2,0) = cosphi*coslam; R(2,1) = cosphi*sinlam; R(2,2) = sinphi;

    Eigen::Vector3d result_enu = R*diff_xyz;
    return result_enu;
}

Eigen::Vector3d enu2xyz(Eigen::Vector3d gps_llh, Eigen::Vector3d gps_enu)
{
    Eigen::Vector3d refllh;
    refllh(0) = gps_llh(0)*M_PI/180;
    refllh(1) = gps_llh(1)*M_PI/180;
    refllh(2) = gps_llh(2);
    Eigen::Vector3d refxyz = llh2xyz(gps_llh);

    Eigen::Vector3d result_xyz;
    result_xyz(0) = -sin(refllh(1))*gps_enu(0) - cos(refllh(1))*sin(refllh(0))*gps_enu(1) + cos(refllh(1))*cos(refllh(0))*gps_enu(2) + refxyz(0);
    result_xyz(1) = cos(refllh(1))*gps_enu(0) - sin(refllh(1))*sin(refllh(0))*gps_enu(1) + cos(refllh(0))*sin(refllh(1))*gps_enu(2) + refxyz(1);
    result_xyz(2) = cos(refllh(0))*gps_enu(1) + sin(refllh(0))*gps_enu(2) + refxyz(2);
    return result_xyz;
}

Eigen::Vector2d LatLon2TM(Eigen::Vector3d llh)
{
    double lat = deg2rad(llh(0));
    double a = 6378137;
    double b = 6356752.314250;
    double f = 0.00335281;
    double k0 = 1;
    double esqure = (pow(a, 2) - pow(b, 2)) / pow(a, 2);
    double epsqure = (pow(a, 2) - pow(b, 2)) / pow(b, 2);
    double dY = 200000;
    double dX = 600000;
    double T = pow(tan(lat), 2);
    double C = esqure * pow(cos(lat), 2) / (1 - esqure);
    double ramda = deg2rad(llh(1));
    double ramda0 = deg2rad(127);  //중부지점 투영원점 경도
    double A = (ramda - ramda0) * cos(lat);
    double N = a / sqrt(1 - (esqure * pow(sin(lat), 2)));

    double Msub0 = (1 - (esqure / 4) - (3 * pow(esqure, 2) / 64) - (5 * pow(esqure, 3) / 256)) * lat;
    double Msub1 = ((3 * esqure / 8) + (3 * pow(esqure, 2) / 32) + (45 * pow(esqure, 3) / 1024)) * sin(2 * lat);
    double Msub2 = ((15 * pow(esqure, 2) / 256) + (45 * pow(esqure, 3) / 1024)) * sin(4 * lat);
    double Msub3 = 35 * pow(esqure, 3) / 3072 * sin(6 * lat);
    double M = a * (Msub0 - Msub1 + Msub2 - Msub3);

    double Ysub0 = pow(A, 3) * (1 - T + C) / 6;
    double Ysub1 = pow(A, 5) * (5 - (18 * T) + pow(T, 2) + (72 * C) - (58 * epsqure)) / 120;
    double Ysub2 = A + Ysub0 + Ysub1;

    Eigen::Vector2d tm;
    tm(0) = dY + (k0 * N * Ysub2);

    double Xsub0 = (pow(A, 2) / 2) + (pow(A, 4) * (5 - T + (9 * C) + (4 * pow(C, 2))) / 24);
    double Xsub1 = pow(A, 6) * (61 - (58 * T) + pow(T, 2) + (600 * C) - (330 * epsqure)) / 720;
    double M0 = 4207498.019266;
    double Xsub2 = N * tan(lat) * (Xsub0 + Xsub1);
    tm(1) = dX + (k0 * (M - M0 + Xsub2));

    return tm;
}

/**
 * @brief TM 좌표를 위경도 값으로 변환
 *
 * @param x
 * @param y
 * @param lat
 * @param lon
 */
Eigen::Vector3d TM2LatLon(Eigen::Vector2d tm)
{
    double M0 = 4207498.019266;
    double dX = 600000;
    double dY = 200000;
    double k0 = 1;
    double a = 6378137;
    double b = 6356752.314250;
    double ramda0 = deg2rad(127);  //중부지점 투영원점 경도

    double M = M0 + (tm(0) - dX) / k0;
    double esqure = (pow(a, 2) - pow(b, 2)) / pow(a, 2);
    double epsqure = (pow(a, 2) - pow(b, 2)) / pow(b, 2);

    double MuSub = 1 - (esqure / 4) - (3 * pow(esqure, 2) / 64) - (5 * pow(esqure, 3) / 256);
    double Mu1 = M / (a * MuSub);
    double e1 = (1 - sqrt(1 - esqure)) / (1 + sqrt(1 - esqure));

    double phiSub0 = ((3 * e1 / 2) - (27 * pow(e1, 3) / 32)) * sin(2 * Mu1);
    double phiSub1 = ((21 * pow(e1, 2) / 16) - (55 * pow(e1, 4) / 32)) * sin(4 * Mu1);
    double phiSub2 = sin(6 * Mu1) * 151 * pow(e1, 3) / 96;
    double phiSub3 = sin(8 * Mu1) * 1097 * pow(e1, 4) / 512;
    double lat1 = Mu1 + phiSub0 + phiSub1 + phiSub2 + phiSub3;

    double RSub0 = a * (1 - esqure);
    double RSub1 = pow(1 - esqure * pow(sin(lat1), 2), 3 / 2);
    double R1 = RSub0 / RSub1;

    double C1 = epsqure * pow(cos(lat1), 2);
    double T1 = pow(tan(lat1), 2);

    double N1 = a / sqrt(1 - esqure * pow(sin(lat1), 2));
    double D = (tm(1) - dY) / (N1 * k0);

    double latSub0 = N1 * tan(lat1) / R1;
    double latSub1 = pow(D, 2) / 2;
    double latSub2 = (5 + 3 * T1 + 10 * C1 - 4 * pow(C1, 2) - 9 * epsqure) * pow(D, 4) / 24;
    double latSub3 = (61 + 90 * T1 + 298 * C1 + 45 * pow(T1, 2) - 252 * epsqure - 3 * pow(C1, 2)) * pow(D, 6) / 720;
    double lat = lat1 - latSub0 * (latSub1 - latSub2 + latSub3);

    double lonSub0 = 1 / cos(lat1);
    double lonSub1 = (1 + 2 * T1 + C1) * pow(D, 3) / 6;
    double lonSub2 = (5 - 2 * C1 + 28 * T1 - 3 * pow(C1, 2) + 8 * epsqure + 24 * pow(T1, 2)) * pow(D, 5) / 120;
    double lon = ramda0 + lonSub0 * (D - lonSub1 + lonSub2);

    Eigen::Vector3d llh;
    llh(0) = rad2deg(lat);
    llh(1) = rad2deg(lon);
    llh(2) = 0;

    return llh;
}

struct Pose6D {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct PointXYZIR
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIR,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
