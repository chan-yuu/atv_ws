#include "msgprocesstools.h"

double CaculateLongitudeDMS2Double(const int degree,const int minute,const double second)
{
    double longitude =0;
    double tempMinute = minute+second/60;
    longitude = degree+tempMinute/60;
    return longitude;
}

std::vector<double> CaculateLongitudeDouble2DMS(const double longitude)
{
    std::vector<double> vectorDMS= std::vector<double>();
    int degree = (int)longitude;
    double doubleMinute= (longitude-degree)*60;
    int minute = (int)doubleMinute;
    double second = (doubleMinute-minute)*60;
    vectorDMS.push_back(degree);
    vectorDMS.push_back(minute);
    vectorDMS.push_back(second);
    return vectorDMS;
}

double CaculateLatitudeDMS2Double(const int degree,const int minute,const double second)
{
    double latitude =0;
    double tempMinute = minute+second/60;
    latitude = degree+tempMinute/60;
    return latitude;
}

std::vector<double> CaculateLatitudeDouble2DMS(const double latitude)
{
    std::vector<double> vectorDMS= std::vector<double>();
    int degree = (int)latitude;
    double doubleMinute= (latitude-degree)*60;
    int minute = (int)doubleMinute;
    double second = (doubleMinute-minute)*60;
    vectorDMS.push_back(degree);
    vectorDMS.push_back(minute);
    vectorDMS.push_back(second);
    return vectorDMS;
}

hmi::HmiStartEndPointInterface CreateHmiStartEndPointMsg(const struct HmiStartEndPointEvent hmiStartEndPointEvent)
{
    hmi::HmiStartEndPointInterface msg;
    return msg;
}

/************************************************************************************************************************/

//hmi::ParamOptimizeInterface CreateParamOptimizeMsg(const struct ParamOptimizeEvent paramOptimizeEvent)
//{
//    hmi::ParamOptimizeInterface msg;
//    msg.kp_v = (float)paramOptimizeEvent.kp_v;
//    msg.kp_kapa = (float)paramOptimizeEvent.kp_kapa;
//    msg.b0_spdcrl=(float)paramOptimizeEvent.b0_spdcrl;
//    msg.wc_spdcrl = (float)paramOptimizeEvent.wc_spdcrl;
//    return msg;
//}
