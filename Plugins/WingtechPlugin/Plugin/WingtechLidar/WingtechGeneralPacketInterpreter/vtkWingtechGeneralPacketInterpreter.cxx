#include "vtkWingtechGeneralPacketInterpreter.h"
#include "PacketFormat.h"
#include "vtkHelper.h"

#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>
#include <vtkTransform.h>

#include <bitset>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <math.h>
#include <fenv.h>

#include <vtkDelimitedTextReader.h>
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

namespace  {

double degreeToRadian(double degree) { return degree * vtkMath::Pi() / 180.0; }
}

static const float pandarGeneral_elev_angle_map[] = {
    14.882f, 11.032f, 8.059f, 5.057f, 3.04f, 2.028f, 1.86f, 1.688f, \
    1.522f, 1.351f, 1.184f, 1.013f, 0.846f, 0.675f, 0.508f, 0.337f, \
    0.169f, 0.0f, -0.169f, -0.337f, -0.508f, -0.675f, -0.845f, -1.013f, \
    -1.184f, -1.351f, -1.522f, -1.688f, -1.86f, -2.028f, -2.198f, -2.365f, \
    -2.536f, -2.7f, -2.873f, -3.04f, -3.21f, -3.375f, -3.548f, -3.712f, \
    -3.884f, -4.05f, -4.221f, -4.385f, -4.558f, -4.72f, -4.892f, -5.057f, \
    -5.229f, -5.391f, -5.565f, -5.726f, -5.898f, -6.061f, -7.063f, -8.059f, \
    -9.06f, -9.885f, -11.032f, -12.006f, -12.974f, -13.93f, -18.889f, -24.897f
};

//! @todo this method are actually usefull for every Interpreter and should go to the top
template<typename T>
vtkSmartPointer<T> vtkWingtechGeneralPacketInterpreter::CreateDataArray(bool isAdvanced,
                                                              const char* name,
                                                              vtkIdType np,
                                                              vtkIdType prereserved_np,
                                                              vtkPolyData* pd)
{
  if (isAdvanced && !this->EnableAdvancedArrays)
  {
    return nullptr;
  }
  vtkSmartPointer<T> array = vtkSmartPointer<T>::New();
  array->SetNumberOfValues(prereserved_np);
  array->SetName(name);
  if (pd)
  {
    pd->GetPointData()->AddArray(array);
  }

  return array;
}

template<typename T, typename U>
void TrySetValue(T& array, int pos, U value)
{
  if (array != nullptr)
  {
    array->SetValue(pos,   value);
  }
}

template<typename T>
void TryResize(T& array, int size)
{
  if (array != nullptr)
  {
    array->Resize(size);
  }
}
//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkWingtechGeneralPacketInterpreter)

//-----------------------------------------------------------------------------
vtkWingtechGeneralPacketInterpreter::vtkWingtechGeneralPacketInterpreter()
{
//  for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
//    float rotation = degreeToRadian(0.01 * static_cast<double>(rotIndex));
//    cos_lookup_table_[rotIndex] = cosf(rotation);
//    sin_lookup_table_[rotIndex] = sinf(rotation);
//  }
  m_sin_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  m_cos_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  for(int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    m_sin_azimuth_map_[i] = sinf(i * M_PI / 18000);
    m_cos_azimuth_map_[i] = cosf(i * M_PI / 18000);
  }
  m_sin_azimuth_map_h.resize(MAX_AZIMUTH_DEGREE_NUM);
  m_cos_azimuth_map_h.resize(MAX_AZIMUTH_DEGREE_NUM);
  for(int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    if(m_sLidarType == "PandarXTM"){
      m_sin_azimuth_map_h[i] = sinf(i * M_PI / 18000) * HS_LIDAR_XTM_COORDINATE_CORRECTION_H;
      m_cos_azimuth_map_h[i] = cosf(i * M_PI / 18000) * HS_LIDAR_XTM_COORDINATE_CORRECTION_H;
    }
    else{
      m_sin_azimuth_map_h[i] = sinf(i * M_PI / 18000) * HS_LIDAR_XT_COORDINATE_CORRECTION_H;
      m_cos_azimuth_map_h[i] = cosf(i * M_PI / 18000) * HS_LIDAR_XT_COORDINATE_CORRECTION_H;
    }
  }
  m_sin_azimuth_map_b.resize(MAX_AZIMUTH_DEGREE_NUM);
  m_cos_azimuth_map_b.resize(MAX_AZIMUTH_DEGREE_NUM);
  for(int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    if(m_sLidarType == "PandarXTM"){
      m_sin_azimuth_map_b[i] = sinf(i * M_PI / 18000) * HS_LIDAR_XTM_COORDINATE_CORRECTION_B;
      m_cos_azimuth_map_b[i] = cosf(i * M_PI / 18000) * HS_LIDAR_XTM_COORDINATE_CORRECTION_B;
    }
    else{
      m_sin_azimuth_map_b[i] = sinf(i * M_PI / 18000) * HS_LIDAR_XT_COORDINATE_CORRECTION_B;
      m_cos_azimuth_map_b[i] = cosf(i * M_PI / 18000) * HS_LIDAR_XT_COORDINATE_CORRECTION_B;
    }

  }
//  if (pcl_type_) {
//    for (int i = 0; i < MAX_LASER_NUM; i++) {
//      PointCloudList[i].reserve(MAX_POINT_CLOUD_NUM_PER_CHANNEL);
//    }
//  }

//  //laser40
//  // init the block time offset, us
//  block40OffsetSingle_[9] = 55.56f * 0.0f + 28.58f;
//  block40OffsetSingle_[8] = 55.56f * 1.0f + 28.58f;
//  block40OffsetSingle_[7] = 55.56f * 2.0f + 28.58f;
//  block40OffsetSingle_[6] = 55.56f * 3.0f + 28.58f;
//  block40OffsetSingle_[5] = 55.56f * 4.0f + 28.58f;
//  block40OffsetSingle_[4] = 55.56f * 5.0f + 28.58f;
//  block40OffsetSingle_[3] = 55.56f * 6.0f + 28.58f;
//  block40OffsetSingle_[2] = 55.56f * 7.0f + 28.58f;
//  block40OffsetSingle_[1] = 55.56f * 8.0f + 28.58f;
//  block40OffsetSingle_[0] = 55.56f * 9.0f + 28.58f;

//  block40OffsetDual_[9] = 55.56f * 0.0f + 28.58f;
//  block40OffsetDual_[8] = 55.56f * 0.0f + 28.58f;
//  block40OffsetDual_[7] = 55.56f * 1.0f + 28.58f;
//  block40OffsetDual_[6] = 55.56f * 1.0f + 28.58f;
//  block40OffsetDual_[5] = 55.56f * 2.0f + 28.58f;
//  block40OffsetDual_[4] = 55.56f * 2.0f + 28.58f;
//  block40OffsetDual_[3] = 55.56f * 3.0f + 28.58f;
//  block40OffsetDual_[2] = 55.56f * 3.0f + 28.58f;
//  block40OffsetDual_[1] = 55.56f * 4.0f + 28.58f;
//  block40OffsetDual_[0] = 55.56f * 4.0f + 28.58f;

//  // init the laser shot time offset, us
//  laser40Offset_[3] = 3.62f;
//  laser40Offset_[39] = 3.62f;
//  laser40Offset_[35] = 4.92f;
//  laser40Offset_[27] = 6.23f;
//  laser40Offset_[11] = 8.19f;
//  laser40Offset_[15] = 8.19f;
//  laser40Offset_[31] = 9.5f;
//  laser40Offset_[23] = 11.47f;
//  laser40Offset_[28] = 12.77f;
//  laser40Offset_[16] = 14.74f;
//  laser40Offset_[2] = 16.04f;
//  laser40Offset_[38] = 16.04f;
//  laser40Offset_[34] = 17.35f;
//  laser40Offset_[24] = 18.65f;
//  laser40Offset_[8] = 20.62f;
//  laser40Offset_[12] = 20.62f;
//  laser40Offset_[30] = 21.92f;
//  laser40Offset_[20] = 23.89f;
//  laser40Offset_[25] = 25.19f;
//  laser40Offset_[13] = 27.16f;
//  laser40Offset_[1] = 28.47f;
//  laser40Offset_[37] = 28.47f;
//  laser40Offset_[33] = 29.77f;
//  laser40Offset_[5] = 31.74f;
//  laser40Offset_[21] = 31.7447f;
//  laser40Offset_[9] = 33.71f;
//  laser40Offset_[29] = 35.01f;
//  laser40Offset_[17] = 36.98f;
//  laser40Offset_[22] = 38.95f;
//  laser40Offset_[10] = 40.91f;
//  laser40Offset_[0] = 42.22f;
//  laser40Offset_[36] = 42.22f;
//  laser40Offset_[32] = 43.52f;
//  laser40Offset_[4] = 45.49f;
//  laser40Offset_[18] = 45.49f;
//  laser40Offset_[6] = 47.46f;
//  laser40Offset_[26] = 48.76f;
//  laser40Offset_[14] = 50.73f;
//  laser40Offset_[19] = 52.7f;
//  laser40Offset_[7] = 54.67f;

//  //laser64 init the laser shot time offset, us
//  // init the block time offset, us
//  block64OffsetSingle_[5] = 55.56f * 0.0f + 42.58f;
//  block64OffsetSingle_[4] = 55.56f * 1.0f + 42.58f;
//  block64OffsetSingle_[3] = 55.56f * 2.0f + 42.58f;
//  block64OffsetSingle_[2] = 55.56f * 3.0f + 42.58f;
//  block64OffsetSingle_[1] = 55.56f * 4.0f + 42.58f;
//  block64OffsetSingle_[0] = 55.56f * 5.0f + 42.58f;

//  block64OffsetDual_[5] = 55.56f * 0.0f + 42.58f;
//  block64OffsetDual_[4] = 55.56f * 0.0f + 42.58f;
//  block64OffsetDual_[3] = 55.56f * 1.0f + 42.58f;
//  block64OffsetDual_[2] = 55.56f * 1.0f + 42.58f;
//  block64OffsetDual_[1] = 55.56f * 2.0f + 42.58f;
//  block64OffsetDual_[0] = 55.56f * 2.0f + 42.58f;

//  laser64Offset_[50] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[60] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[44] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[59] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[38] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[56] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[8]  = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[54] = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[48] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[62] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[42] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[58] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[6]  = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[55] = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[52] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[63] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[46] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[61] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[40] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[57] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[5]  = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[53] = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[4]  = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[47] = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[3]  = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[49] = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[2]  = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[51] = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[1]  = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[45] = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[0]  = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[43] = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
//  laser64Offset_[23] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
//  laser64Offset_[32] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
//  laser64Offset_[26] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
//  laser64Offset_[41] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
//  laser64Offset_[20] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
//  laser64Offset_[35] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
//  laser64Offset_[14] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
//  laser64Offset_[29] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
//  laser64Offset_[21] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
//  laser64Offset_[36] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
//  laser64Offset_[15] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
//  laser64Offset_[30] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
//  laser64Offset_[9]  = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
//  laser64Offset_[24] = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
//  laser64Offset_[18] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
//  laser64Offset_[33] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
//  laser64Offset_[12] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
//  laser64Offset_[27] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
//  laser64Offset_[19] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
//  laser64Offset_[34] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
//  laser64Offset_[13] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
//  laser64Offset_[28] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
//  laser64Offset_[7]  = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
//  laser64Offset_[22] = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
//  laser64Offset_[16] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
//  laser64Offset_[31] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
//  laser64Offset_[10] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
//  laser64Offset_[25] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
//  laser64Offset_[17] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
//  laser64Offset_[37] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
//  laser64Offset_[11] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;
//  laser64Offset_[39] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;

//  for (int i = 0; i < HS_LIDAR_L20_BLOCK_NUMBER; i++) {
//    block20OffsetSingle_[i] = 28.58f + (HS_LIDAR_L20_BLOCK_NUMBER - 1 - i) * 55.56f;
//    block20OffsetDual_[i] = 28.58f + static_cast<int>((HS_LIDAR_L20_BLOCK_NUMBER - 1 - i) / 2) * 55.56f;
//  }

//  laser20AOffset_[1] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
//  laser20AOffset_[19]  = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
//  laser20AOffset_[16] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
//  laser20AOffset_[14] = 1.304f * 3.0f + 1.968f * 1.0f + 3.62f;
//  laser20AOffset_[11] = 1.304f * 3.0f + 1.968f * 2.0f + 3.62f;
//  laser20AOffset_[0] = 1.304f * 5.0f + 1.968f * 3.0f + 3.62f;
//  laser20AOffset_[18]  = 1.304f * 5.0f + 1.968f * 3.0f + 3.62f;
//  laser20AOffset_[5] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
//  laser20AOffset_[7] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
//  laser20AOffset_[10] = 1.304f * 8.0f + 1.968f * 5.0f + 3.62f;
//  laser20AOffset_[17] = 1.304f * 10.0f + 1.968f * 6.0f + 3.62f;
//  laser20AOffset_[15]  = 1.304f * 11.0f + 1.968f * 6.0f + 3.62f;
//  laser20AOffset_[3] = 1.304f * 11.0f + 1.968f * 7.0f + 3.62f;
//  laser20AOffset_[13] = 1.304f * 12.0f + 1.968f * 8.0f + 3.62f;
//  laser20AOffset_[9] = 1.304f * 12.0f + 1.968f * 9.0f + 3.62f;
//  laser20AOffset_[6] = 1.304f * 12.0f + 1.968f * 11.0f + 3.62f;
//  laser20AOffset_[2]  = 1.304f * 14.0f + 1.968f * 12.0f + 3.62f;
//  laser20AOffset_[4] = 1.304f * 14.0f + 1.968f * 13.0f + 3.62f;
//  laser20AOffset_[12] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
//  laser20AOffset_[8] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;

//  laser20BOffset_[17] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
//  laser20BOffset_[5]  = 1.304f * 2.0f + 1.968f * 1.0f + 3.62f;
//  laser20BOffset_[15] = 1.304f * 3.0f + 1.968f * 1.0f + 3.62f;
//  laser20BOffset_[11] = 1.304f * 3.0f + 1.968f * 2.0f + 3.62f;
//  laser20BOffset_[8] = 1.304f * 4.0f + 1.968f * 3.0f + 3.62f;
//  laser20BOffset_[19] = 1.304f * 5.0f + 1.968f * 3.0f + 3.62f;
//  laser20BOffset_[3]  = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
//  laser20BOffset_[6] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
//  laser20BOffset_[14] = 1.304f * 8.0f + 1.968f * 4.0f + 3.62f;
//  laser20BOffset_[10] = 1.304f * 8.0f + 1.968f * 5.0f + 3.62f;
//  laser20BOffset_[18] = 1.304f * 10.0f + 1.968f * 6.0f + 3.62f;
//  laser20BOffset_[16]  = 1.304f * 11.0f + 1.968f * 6.0f + 3.62f;
//  laser20BOffset_[1] = 1.304f * 11.0f + 1.968f * 7.0f + 3.62f;
//  laser20BOffset_[13] = 1.304f * 12.0f + 1.968f * 8.0f + 3.62f;
//  laser20BOffset_[4] = 1.304f * 12.0f + 1.968f * 11.0f + 3.62f;
//  laser20BOffset_[0] = 1.304f * 14.0f + 1.968f * 12.0f + 3.62f;
//  laser20BOffset_[9]  = 1.304f * 14.0f + 1.968f * 12.0f + 3.62f;
//  laser20BOffset_[2] = 1.304f * 14.0f + 1.968f * 13.0f + 3.62f;
//  laser20BOffset_[12] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
//  laser20BOffset_[7] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;

//  // QT
//  blockQTOffsetSingle_[0] = 25.71f;
//  blockQTOffsetSingle_[1] = 25.71f + 166.67f;
//  blockQTOffsetSingle_[2] = 25.71f + 333.33f;
//  blockQTOffsetSingle_[3] = 25.71f + 500.00f;

//  blockQTOffsetDual_[0] = 25.71f;
//  blockQTOffsetDual_[1] = 25.71f;
//  blockQTOffsetDual_[2] = 25.71f + 166.67f;
//  blockQTOffsetDual_[3] = 25.71f + 166.67f;

//  laserQTOffset_[0] = 10.0f + 2.31f;
//  laserQTOffset_[1] = 10.0f + 4.37f;
//  laserQTOffset_[2] = 10.0f + 6.43f;
//  laserQTOffset_[3] = 10.0f + 8.49f;
//  laserQTOffset_[4] = 10.0f + 10.54f;
//  laserQTOffset_[5] = 10.0f + 12.60f;
//  laserQTOffset_[6] = 10.0f + 14.66f;
//  laserQTOffset_[7] = 10.0f + 16.71f;
//  laserQTOffset_[8] = 10.0f + 19.16f;
//  laserQTOffset_[9] = 10.0f + 21.22f;
//  laserQTOffset_[10] = 10.0f + 23.28f;
//  laserQTOffset_[11] = 10.0f + 25.34f;
//  laserQTOffset_[12] = 10.0f + 27.39f;
//  laserQTOffset_[13] = 10.0f + 29.45f;
//  laserQTOffset_[14] = 10.0f + 31.50f;
//  laserQTOffset_[15] = 10.0f + 33.56f;

//  laserQTOffset_[16] = 10.0f + 36.61f;
//  laserQTOffset_[17] = 10.0f + 38.67f;
//  laserQTOffset_[18] = 10.0f + 40.73f;
//  laserQTOffset_[19] = 10.0f + 42.78f;
//  laserQTOffset_[20] = 10.0f + 44.84f;
//  laserQTOffset_[21] = 10.0f + 46.90f;
//  laserQTOffset_[22] = 10.0f + 48.95f;
//  laserQTOffset_[23] = 10.0f + 51.01f;
//  laserQTOffset_[24] = 10.0f + 53.45f;
//  laserQTOffset_[25] = 10.0f + 55.52f;
//  laserQTOffset_[26] = 10.0f + 57.58f;
//  laserQTOffset_[27] = 10.0f + 59.63f;
//  laserQTOffset_[28] = 10.0f + 61.69f;
//  laserQTOffset_[29] = 10.0f + 63.74f;
//  laserQTOffset_[30] = 10.0f + 65.80f;
//  laserQTOffset_[31] = 10.0f + 67.86f;

//  laserQTOffset_[32] = 10.0f + 70.90f;
//  laserQTOffset_[33] = 10.0f + 72.97f;
//  laserQTOffset_[34] = 10.0f + 75.02f;
//  laserQTOffset_[35] = 10.0f + 77.08f;
//  laserQTOffset_[36] = 10.0f + 79.14f;
//  laserQTOffset_[37] = 10.0f + 81.19f;
//  laserQTOffset_[38] = 10.0f + 83.25f;
//  laserQTOffset_[39] = 10.0f + 85.30f;
//  laserQTOffset_[40] = 10.0f + 87.75f;
//  laserQTOffset_[41] = 10.0f + 89.82f;
//  laserQTOffset_[42] = 10.0f + 91.87f;
//  laserQTOffset_[43] = 10.0f + 93.93f;
//  laserQTOffset_[44] = 10.0f + 95.98f;
//  laserQTOffset_[45] = 10.0f + 98.04f;
//  laserQTOffset_[46] = 10.0f + 100.10f;
//  laserQTOffset_[47] = 10.0f + 102.15f;

//  laserQTOffset_[48] = 10.0f + 105.20f;
//  laserQTOffset_[49] = 10.0f + 107.26f;
//  laserQTOffset_[50] = 10.0f + 109.32f;
//  laserQTOffset_[51] = 10.0f + 111.38f;
//  laserQTOffset_[52] = 10.0f + 113.43f;
//  laserQTOffset_[53] = 10.0f + 115.49f;
//  laserQTOffset_[54] = 10.0f + 117.54f;
//  laserQTOffset_[55] = 10.0f + 119.60f;
//  laserQTOffset_[56] = 10.0f + 122.05f;
//  laserQTOffset_[57] = 10.0f + 124.11f;
//  laserQTOffset_[58] = 10.0f + 126.17f;
//  laserQTOffset_[59] = 10.0f + 128.22f;
//  laserQTOffset_[60] = 10.0f + 130.28f;
//  laserQTOffset_[61] = 10.0f + 132.34f;
//  laserQTOffset_[62] = 10.0f + 134.39f;
//  laserQTOffset_[63] = 10.0f + 136.45f;

//  if (m_sLidarType == "Pandar40P" || m_sLidarType == "Pandar40M") {
//    m_sin_elevation_map_.resize(LASER_COUNT);
//    m_cos_elevation_map_.resize(LASER_COUNT);
//    for (int i = 0; i < LASER_COUNT; i++) {
//      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandar40p_elev_angle_map[i]));
//      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandar40p_elev_angle_map[i]));
//      General_elev_angle_map_[i] = pandar40p_elev_angle_map[i];
//      General_horizatal_azimuth_offset_map_[i] = pandar40p_horizatal_azimuth_offset_map[i];
//    }
//  }

//  if (m_sLidarType == "Pandar64") {
//    m_sin_elevation_map_.resize(HS_LIDAR_L64_UNIT_NUM);
//    m_cos_elevation_map_.resize(HS_LIDAR_L64_UNIT_NUM);
//    for (int i = 0; i < HS_LIDAR_L64_UNIT_NUM; i++) {
//      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarGeneral_elev_angle_map[i]));
//      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarGeneral_elev_angle_map[i]));
//      General_elev_angle_map_[i] = pandarGeneral_elev_angle_map[i];
//      General_horizatal_azimuth_offset_map_[i] = pandarGeneral_horizatal_azimuth_offset_map[i];
//    }
//  }

//  if (m_sLidarType == "Pandar20A" || m_sLidarType == "Pandar20B") {
//    m_sin_elevation_map_.resize(HS_LIDAR_L20_UNIT_NUM);
//    m_cos_elevation_map_.resize(HS_LIDAR_L20_UNIT_NUM);
//    for (int i = 0; i < HS_LIDAR_L20_UNIT_NUM; i++) {
//      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandar20_elev_angle_map[i]));
//      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandar20_elev_angle_map[i]));
//      General_elev_angle_map_[i] = pandar20_elev_angle_map[i];
//      General_horizatal_azimuth_offset_map_[i] = pandar20_horizatal_azimuth_offset_map[i];
//    }
//  }

//  if (m_sLidarType == "PandarQT") {
//    m_sin_elevation_map_.resize(HS_LIDAR_QT_UNIT_NUM);
//    m_cos_elevation_map_.resize(HS_LIDAR_QT_UNIT_NUM);
//    for (int i = 0; i < HS_LIDAR_QT_UNIT_NUM; i++) {
//      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarQT_elev_angle_map[i]));
//      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarQT_elev_angle_map[i]));
//      General_elev_angle_map_[i] = pandarQT_elev_angle_map[i];
//      General_horizatal_azimuth_offset_map_[i] = pandarQT_horizatal_azimuth_offset_map[i];
//    }
//  }

  if (m_sLidarType == "PandarXT-32") {
    m_sin_elevation_map_.resize(HS_LIDAR_XT_UNIT_NUM);
    m_cos_elevation_map_.resize(HS_LIDAR_XT_UNIT_NUM);
    for (int i = 0; i < HS_LIDAR_XT_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarXT_elev_angle_map[i]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarXT_elev_angle_map[i]));
      General_elev_angle_map_[i] = pandarXT_elev_angle_map[i];
      General_horizatal_azimuth_offset_map_[i] = pandarXT_horizatal_azimuth_offset_map[i];
      laserXTOffset_[i] = laserXTOffset[i];
    }
  }

  if (m_sLidarType == "PandarXT-16") {
    m_sin_elevation_map_.resize(HS_LIDAR_XT16_UNIT_NUM);
    m_cos_elevation_map_.resize(HS_LIDAR_XT16_UNIT_NUM);
    for (int i = 0; i < HS_LIDAR_XT16_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarXT_elev_angle_map[i*2]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarXT_elev_angle_map[i*2]));
      General_elev_angle_map_[i] = pandarXT_elev_angle_map[i*2];
      General_horizatal_azimuth_offset_map_[i] = pandarXT_horizatal_azimuth_offset_map[i*2];
      laserXTOffset_[i] = laserXTOffset[i*2];
    }
  }

  for (int i = 0; i < HS_LIDAR_XT_BLOCK_NUMBER; i++) {
    blockXTOffsetSingle_[i] = blockXTOffsetSingle[i];
    blockXTOffsetDual_[i] = blockXTOffsetDual[i];
  }

  if (m_sLidarType == "PandarXTM") {
    m_sin_elevation_map_.resize(HS_LIDAR_XT_UNIT_NUM);
    m_cos_elevation_map_.resize(HS_LIDAR_XT_UNIT_NUM);
    for (int i = 0; i < HS_LIDAR_XT_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarXTM_elev_angle_map[i]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarXTM_elev_angle_map[i]));
      General_elev_angle_map_[i] = pandarXTM_elev_angle_map[i];
      General_horizatal_azimuth_offset_map_[i] = pandarXTM_horizatal_azimuth_offset_map[i];
      laserXTOffset_[i] = laserXTMOffset[i];
    }
    for (int i = 0; i < HS_LIDAR_XT_BLOCK_NUMBER; i++) {
      blockXTOffsetSingle_[i] = blockXTMOffsetSingle[i];
      blockXTOffsetDual_[i] = blockXTMOffsetDual[i];
      blockXTOffsetTriple_[i] = blockXTMOffsetTriple[i];
    }
  }
//  // Initialize Elevation and Azimuth correction
//  for(int i = 0; i < PANDAR128_LASER_NUM; i++)
//  {
//    if(elev_angle[i])
//    {
//      this->General_elev_angle_map_.push_back(elev_angle[i]);
//    }
//    if(azimuth_offset[i])
//    {
//      this->General_horizatal_azimuth_offset_map_.push_back(azimuth_offset[i]);
//    }
//  }

  this->ParserMetaData.SpecificInformation = std::make_shared<WingtechGeneralSpecificFrameInformation>();

//  for (int j = 0; j < CIRCLE; j++)
//  {
//    double angle = j / 100.0;
//    this->Cos_all_angle.push_back(std::cos(degreeToRadian(angle)));
//    this->Sin_all_angle.push_back(std::sin(degreeToRadian(angle)));
//  }
}

//-----------------------------------------------------------------------------
vtkWingtechGeneralPacketInterpreter::~vtkWingtechGeneralPacketInterpreter()
{
}

//-----------------------------------------------------------------------------
void vtkWingtechGeneralPacketInterpreter::LoadCalibration(const std::string& filename)
{
  if (filename.empty() || boost::filesystem::extension(filename) != ".csv")
  {
    this->IsCalibrated = false;
    return;
  }

  // Load the CSV file.
  // 2nd Column = Azimuth (Horizontal) Correction
  // 3rd Column = Elevation (Vertical) Correction

  vtkNew<vtkDelimitedTextReader> reader;
  reader->SetFileName(filename.c_str());
  reader->DetectNumericColumnsOn();
  reader->SetHaveHeaders(true);
  reader->SetFieldDelimiterCharacters(",");
  reader->SetStringDelimiter('"');
  reader->Update();

  // Extract the table.
  vtkTable * csvTable = reader->GetOutput();
  vtkIdType nRows = csvTable->GetNumberOfRows();

  this->CalibrationData->ShallowCopy(csvTable);

//  this->General_elev_angle_map_.clear();
//  this->General_elev_angle_map_.resize(nRows);
//  this->General_horizatal_azimuth_offset_map_.clear();
//  this->General_horizatal_azimuth_offset_map_.resize(nRows);

  for (vtkIdType indexRow = 0; indexRow < nRows; ++indexRow)
  {
    double elevation = this->CalibrationData->GetValue(indexRow, 1).ToDouble();
    double azimuth = this->CalibrationData->GetValue(indexRow, 2).ToDouble();

    this->General_elev_angle_map_[indexRow] = elevation;
    m_sin_elevation_map_[indexRow] = sinf(degreeToRadian(General_elev_angle_map_[indexRow]));
    m_cos_elevation_map_[indexRow] = cosf(degreeToRadian(General_elev_angle_map_[indexRow]));
    this->General_horizatal_azimuth_offset_map_[indexRow] = azimuth;


  }
  this->IsCalibrated = true;
}

//-----------------------------------------------------------------------------
void vtkWingtechGeneralPacketInterpreter::ProcessPacket(unsigned char const* data,
                                              unsigned int dataLength)
{
//  cout << "ProcessPacket" << endl;
  if (!this->IsLidarPacket(data, dataLength))
  {
    return;
  }

  HS_LIDAR_XT_Packet pkt;
  int ret = PandarGeneral_Internal::ParseXTData(&pkt, data, dataLength);
  if (ret != 0) {
//    cout << "Packet could not be parsed" << endl;
    return;
  }

  auto* frameInfo = reinterpret_cast<WingtechGeneralSpecificFrameInformation*>(this->ParserMetaData.SpecificInformation.get());
  int blockOffset = frameInfo->blockOffset;
  frameInfo->blockOffset = 0;

  // In case the dual return mode is disabled,
  // check if the Lidar was configured with dual return
  bool twoReturns = true;
  if (!this->DualReturn)
  {
    // First block
    HS_LIDAR_XT_Block& block0 = pkt.blocks[0];
    int azimuth0 = static_cast<int>(block0.azimuth*10000);
    // Following block
    HS_LIDAR_XT_Block& block1 = pkt.blocks[1];
    int azimuth1 = static_cast<int>(block1.azimuth*10000);
    bool twoReturns = azimuth0 != azimuth1;
  }

  // If the dual return mode is disabled and there were 2 returns
  // by ray, remove one block over two
  int blockStep = (twoReturns && !this->DualReturn)? 2 : 1;
  for (int blockid = blockOffset; blockid < pkt.header.chBlockNumber; blockid+=blockStep) {

    int azimuthGap = 0; /* To do */
    double timestampGap = 0; /* To do */
    if(last_azimuth_ > pkt.blocks[blockid].azimuth) {
      azimuthGap = static_cast<int>(pkt.blocks[blockid].azimuth) + (36000 - static_cast<int>(last_azimuth_));
    } else {
      azimuthGap = static_cast<int>(pkt.blocks[blockid].azimuth) - static_cast<int>(last_azimuth_);
    }
    timestampGap = pkt.timestamp_point - last_timestamp_ + 0.001;
    if (last_azimuth_ != pkt.blocks[blockid].azimuth && \
        (azimuthGap / timestampGap) < MAX_AZIMUTH_DEGREE_NUM * 100 ) {
      /* for all the blocks */
      if ((last_azimuth_ > pkt.blocks[blockid].azimuth &&
           start_angle_ <= pkt.blocks[blockid].azimuth) ||
          (last_azimuth_ < start_angle_ &&
           start_angle_ <= pkt.blocks[blockid].azimuth)) {
//        cout << "last_azimuth_ " << last_azimuth_ << endl;
//        cout << "start_angle_ " << start_angle_ << endl;
//        cout << "pkt.blocks[i].azimuth " << pkt.blocks[blockid].azimuth << endl;
//        cout << "SplitFrame with " << current_pt_id << endl;
        this->Points->Resize(current_pt_id);
        TryResize(this->PointsX, current_pt_id);
        TryResize(this->PointsY, current_pt_id);
        TryResize(this->PointsZ, current_pt_id);
        TryResize(this->LaserID, current_pt_id);
        TryResize(this->Intensities, current_pt_id);
        TryResize(this->Timestamps, current_pt_id);
        TryResize(this->Distances, current_pt_id);

        this->Intensities->Resize(current_pt_id);
        this->Timestamps->Resize(current_pt_id);
        this->SplitFrame();
      }
    }
    last_azimuth_ = pkt.blocks[blockid].azimuth;
//    cout << "607 last_azimuth_ " << last_azimuth_ << endl;

    last_timestamp_ = pkt.timestamp_point;


  HS_LIDAR_XT_Block& block = pkt.blocks[blockid];

    for (int i = 0; i < 32/*TODO do not hardcode*/; ++i) {
      /* for all the units in a block */
      HS_LIDAR_XT_Unit &unit = block.units[i];

      int azimuth = static_cast<int>(General_horizatal_azimuth_offset_map_[i] * 100 + block.azimuth);
      if(azimuth < 0)
        azimuth += 36000;
      if(azimuth >= 36000)
        azimuth -= 36000;
      float distance = unit.distance;
      double x, y, z, timestamp;
      if(m_bCoordinateCorrectionFlag)
      {
        distance = distance - (m_cos_azimuth_map_h[abs(int(General_horizatal_azimuth_offset_map_[i] * 100))] * m_cos_elevation_map_[i] -
                   m_sin_azimuth_map_b[abs(int(General_horizatal_azimuth_offset_map_[i] * 100))] * m_cos_elevation_map_[i]);
        float xyDistance = distance * m_cos_elevation_map_[i];
        x = xyDistance * m_sin_azimuth_map_[azimuth] - m_cos_azimuth_map_b[azimuth] + m_sin_azimuth_map_h[azimuth];
        y = xyDistance * m_cos_azimuth_map_[azimuth] + m_sin_azimuth_map_b[azimuth] + m_cos_azimuth_map_h[azimuth];
        z = distance * m_sin_elevation_map_[i];

        // TODO check what happend here
//        if (COORDINATE_CORRECTION_CHECK){
//          float xyDistance = unit.distance * m_cos_elevation_map_[i];
//          float point_x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
//          float point_y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
//          float point_z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
//          printf("distance = %f; elevation = %f; azimuth = %f; delta X = %f; delta Y = %f; delta Z = %f; \n",
//                unit.distance, pandarGeneral_elev_angle_map[i], float(azimuth / 100), point.x - point_x, point.y - point_y, point.z - point_z);
//        }
      }
      else
      {
        float xyDistance = distance * m_cos_elevation_map_[i];
        x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
        y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
        z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
      }

      // Skip wrong points
      if (distance <= 0.1 || distance > 500.0)
        continue;

  /*    if ("realtime" == m_sTimestampType) {
        point.timestamp = m_dPktTimestamp;
      }
      else */{
        timestamp = pkt.timestamp_point + this->TimeOffset;

        if (pkt.echo == 0x3d){
          timestamp =
              timestamp + (static_cast<double>(blockXTOffsetTriple_[blockid] +
                                                    laserXTOffset_[i]) /
                                1000000.0f);
        }
        else if (pkt.echo == 0x39 || pkt.echo == 0x3b || pkt.echo == 0x3c) {
          timestamp =
              timestamp + (static_cast<double>(blockXTOffsetDual_[blockid] +
                                                    laserXTOffset_[i]) /
                                1000000.0f);
        } else {
          timestamp = timestamp + \
              (static_cast<double>(blockXTOffsetSingle_[blockid] + laserXTOffset_[i]) / \
              1000000.0f);
        }
      }

      if (!this->DualReturn && current_pt_id > 500000)
      {
        vtkWarningMacro("Received more datapoints than expected");
        continue;
      }

      this->Points->SetPoint(current_pt_id, x, y, z);
      TrySetValue(PointsX, current_pt_id, x);
      TrySetValue(PointsY, current_pt_id, y);
      TrySetValue(PointsZ, current_pt_id, z);
      TrySetValue(LaserID, current_pt_id, i);
      TrySetValue(Timestamps, current_pt_id, timestamp);
      TrySetValue(Intensities, current_pt_id, unit.intensity);
      TrySetValue(Distances, current_pt_id, unit.distance);
      current_pt_id++;
    }

  }
}

//-----------------------------------------------------------------------------
bool vtkWingtechGeneralPacketInterpreter::IsLidarPacket(unsigned char const * vtkNotUsed(data),
                                              unsigned int dataLength)
{
  // Generic code pandarGeneral_internal (l935)
//  if((packet.size == HS_LIDAR_XT_PACKET_SIZE && (m_sLidarType == "XT" || m_sLidarType == "PandarXT-32")) ||
//     (packet.size == HS_LIDAR_XT16_PACKET_SIZE && (m_sLidarType == "PandarXT-16")) ||
//     (packet.size == HS_LIDAR_XTM_PACKET_SIZE && (m_sLidarType == "PandarXTM")))
  // Specific code
  if(dataLength == HS_LIDAR_XT_PACKET_SIZE    ||
     dataLength == HS_LIDAR_XT16_PACKET_SIZE  ||
     dataLength == HS_LIDAR_XTM_PACKET_SIZE)
//    cout << true << endl;
  return true;
//  cout << false << endl;
  return false;
}



//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkWingtechGeneralPacketInterpreter::CreateNewEmptyFrame(vtkIdType numberOfPoints,
                                                                            vtkIdType prereservedNumberOfPoints)
{
  const int defaultPrereservedNumberOfPointsPerFrame = PANDAR128_POINT_NUM;
  numberOfPoints = defaultPrereservedNumberOfPointsPerFrame;
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  // points
  current_pt_id = 0;
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(defaultPrereservedNumberOfPointsPerFrame);
  points->GetData()->SetName("Points_m_XYZ");
  polyData->SetPoints(points.GetPointer());

  // intensity
  this->Points = points.GetPointer();
  this->PointsX = CreateDataArray<vtkDoubleArray>(true, "X", numberOfPoints,
                                                  defaultPrereservedNumberOfPointsPerFrame, polyData);
  this->PointsY = CreateDataArray<vtkDoubleArray>(true, "Y", numberOfPoints,
                                                  defaultPrereservedNumberOfPointsPerFrame, polyData);
  this->PointsZ = CreateDataArray<vtkDoubleArray>(true, "Z", numberOfPoints,
                                                  defaultPrereservedNumberOfPointsPerFrame, polyData);

  this->LaserID = CreateDataArray<vtkUnsignedIntArray>(true, "LaserID", numberOfPoints,
                                                       defaultPrereservedNumberOfPointsPerFrame, polyData);
  this->Intensities = CreateDataArray<vtkDoubleArray>(false, "Intensity", numberOfPoints,
                                                      defaultPrereservedNumberOfPointsPerFrame, polyData);
  this->Timestamps = CreateDataArray<vtkDoubleArray>(false, "Timestamp", numberOfPoints,
                                                     defaultPrereservedNumberOfPointsPerFrame, polyData);
  this->Distances = CreateDataArray<vtkDoubleArray>(true, "Distance", numberOfPoints,
                                                    defaultPrereservedNumberOfPointsPerFrame, polyData);
  polyData->GetPointData()->SetActiveScalars("Intensity");

  last_azimuth_ = 0;
//  cout << "752 last_azimuth_ " << last_azimuth_ << endl;

  last_timestamp_ = -1;
  return polyData;
}

//-----------------------------------------------------------------------------
bool vtkWingtechGeneralPacketInterpreter::PreProcessPacket(unsigned char const* data,
  unsigned int dataLength, fpos_t filePosition, double packetNetworkTime,
  std::vector<FrameInformation>* frameCatalog)
{
  bool isNewFrame = false;

  this->ParserMetaData.FilePosition = filePosition;
  this->ParserMetaData.FirstPacketDataTime = 0.0; // TODO
  this->ParserMetaData.FirstPacketNetworkTime = packetNetworkTime;

  HS_LIDAR_XT_Packet pkt;
  int ret = PandarGeneral_Internal::ParseXTData(&pkt, data, dataLength);
  if (ret != 0) {
//    cout << "Packet could not be parsed" << endl;
    return false;
  }

  for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
    int azimuthGap = 0; /* To do */
    double timestampGap = 0; /* To do */
    if(last_azimuth_ > pkt.blocks[i].azimuth) {
      azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) + (36000 - static_cast<int>(last_azimuth_));
    } else {
      azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
    }
    timestampGap = pkt.timestamp_point - last_timestamp_ + 0.001;
    if (last_azimuth_ != pkt.blocks[i].azimuth && \
        (azimuthGap / timestampGap) < MAX_AZIMUTH_DEGREE_NUM * 100 ) {
      /* for all the blocks */
      if ((last_azimuth_ > pkt.blocks[i].azimuth &&
           start_angle_ <= pkt.blocks[i].azimuth) ||
          (last_azimuth_ < start_angle_ &&
           start_angle_ <= pkt.blocks[i].azimuth)) {
        auto specificInformation = std::make_shared<WingtechGeneralSpecificFrameInformation>();
        specificInformation->blockOffset = i;
        this->ParserMetaData.SpecificInformation = specificInformation;
        frameCatalog->push_back(this->ParserMetaData);
      }
    }
    last_azimuth_ = pkt.blocks[i].azimuth;
//    cout << "796 last_azimuth_ " << last_azimuth_ << endl;

    last_timestamp_ = pkt.timestamp_point;
  }
  return isNewFrame;
}


//-----------------------------------------------------------------------------
std::string vtkWingtechGeneralPacketInterpreter::GetSensorInformation(bool vtkNotUsed(shortVersion))
{
  return "Wingtech Sensor";
}

