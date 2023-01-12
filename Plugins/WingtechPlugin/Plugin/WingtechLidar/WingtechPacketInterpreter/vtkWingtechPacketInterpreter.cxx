#include "vtkWingtechPacketInterpreter.h"
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

//! @todo this method are actually usefull for every Interpreter and should go to the top
template<typename T>
vtkSmartPointer<T> vtkWingtechPacketInterpreter::CreateDataArray(bool isAdvanced,
                                                              const char* name,
                                                              vtkIdType vtkNotUsed(np),
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

//-----------------------------------------------------------------------------
vtkStandardNewMacro(vtkWingtechPacketInterpreter)

//-----------------------------------------------------------------------------
vtkWingtechPacketInterpreter::vtkWingtechPacketInterpreter()
{
  // Initialize Elevation and Azimuth correction
  for(int i = 0; i < PANDAR128_LASER_NUM; i++)
  {
    if(elev_angle[i])
    {
      this->ElevationCorrection.push_back(elev_angle[i]);
    }
    if(azimuth_offset[i])
    {
      this->AzimuthCorrection.push_back(azimuth_offset[i]);
    }
  }

  this->ParserMetaData.SpecificInformation = std::make_shared<WingtechSpecificFrameInformation>();

  for (int j = 0; j < CIRCLE; j++)
  {
    double angle = j / 100.0;
    this->Cos_all_angle.push_back(std::cos(degreeToRadian(angle)));
    this->Sin_all_angle.push_back(std::sin(degreeToRadian(angle)));
  }
}

//-----------------------------------------------------------------------------
vtkWingtechPacketInterpreter::~vtkWingtechPacketInterpreter()
{
}

//-----------------------------------------------------------------------------
void vtkWingtechPacketInterpreter::LoadCalibration(const std::string& filename)
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

  this->ElevationCorrection.clear();
  this->ElevationCorrection.resize(nRows);
  this->AzimuthCorrection.clear();
  this->AzimuthCorrection.resize(nRows);

  for (vtkIdType indexRow = 0; indexRow < nRows; ++indexRow)
  {
    double elevation = this->CalibrationData->GetValue(indexRow, 1).ToDouble();
    double azimuth = this->CalibrationData->GetValue(indexRow, 2).ToDouble();

    this->ElevationCorrection[indexRow] = elevation;
    this->AzimuthCorrection[indexRow] = azimuth;

  }
  this->IsCalibrated = true;
}

//-----------------------------------------------------------------------------
// https://github.com/HesaiTechnology/HesaiLidar_Pandar128_ROS/blob/160436018cbe4c96249c281499e2e9638d38b39c/pandar_pointcloud/src/conversions/convert.cc#L622
void vtkWingtechPacketInterpreter::ProcessPacket(unsigned char const* data,
                                              unsigned int dataLength)
{
  auto start = high_resolution_clock::now();
  if (!this->IsLidarPacket(data, dataLength))
  {
    return;
  }

  const Pandar128Packet* dataPacket = reinterpret_cast<const Pandar128Packet*>(data);

  struct tm t;
  t.tm_year = dataPacket->tail.GetUTCTime0();
  t.tm_mon = dataPacket->tail.GetUTCTime1() - 1;
  t.tm_mday = dataPacket->tail.GetUTCTime2();
  t.tm_hour = dataPacket->tail.GetUTCTime3();
  t.tm_min = dataPacket->tail.GetUTCTime4();
  t.tm_sec = dataPacket->tail.GetUTCTime5();
  t.tm_isdst = 0;

  // Time in second of the packets
  time_t  unix_second = (mktime(&t));

  int mode =  dataPacket->tail.GetShutdownFlag() & 0x03;
  int state = (dataPacket->tail.GetShutdownFlag() & 0xF0) >> 4;
  int returnMode = dataPacket->tail.GetReturnMode();

  // Timestamp contains in the packet
  // roll back every second, probably in microsecond
  uint32_t timestampPacket = dataPacket->tail.GetTimestamp();

  // Timestamp in second of the packet
  double timestamp = unix_second + (timestampPacket / 1000000.0);

  // [HACK start] Proccess only one return in case of dual mode for performance issue
  int start_block = 0;
  int end_block = PANDAR128_BLOCK_NUM;
  if (returnMode == 0x39 ||
      returnMode == 0x3B  ||
      returnMode == 0x3C)
  {
    end_block = 1;
  }
  // [HACK end]


  for (int blockID = start_block; blockID < end_block; blockID++)
  {
     Pandar128Block currentBlock = dataPacket->blocks[blockID];

     WingtechSpecificFrameInformation* frameInfo =
         reinterpret_cast<WingtechSpecificFrameInformation*>(this->ParserMetaData.SpecificInformation.get());
     if(frameInfo->IsNewFrame(1, currentBlock.GetAzimuth()))
     {
       this->SplitFrame();
     }


     for (int laserID = 0; laserID < PANDAR128_LASER_NUM; laserID++)
     {

       double distance = static_cast<double>(currentBlock.units[laserID].GetDistance()) * PANDAR128_DISTANCE_UNIT;

       double azimuth_correction = this->AzimuthCorrection[laserID];
       double elevation_correction = this->ElevationCorrection[laserID];


       float azimuth = azimuth_correction + (static_cast<float>(currentBlock.GetAzimuth()) / 100.0f);
       float originAzimuth = azimuth;

       float pitch = elevation_correction;

       int offset = this->LaserOffset.getTSOffset(laserID, mode, state, distance, dataPacket->header.GetVersionMajor());

       azimuth += this->LaserOffset.getAngleOffset(offset, dataPacket->tail.GetMotorSpeed(), dataPacket->header.GetVersionMajor());

       pitch += this->LaserOffset.getPitchOffset("", pitch, distance);

       if (pitch < 0)
       {
         pitch += 360.0f;
       }
       else if (pitch >= 360.0f)
       {
         pitch -= 360.0f;
       }

       float xyDistance = distance * this->Cos_all_angle[static_cast<int>(pitch * 100 + 0.5)];
       azimuth += this->LaserOffset.getAzimuthOffset("", originAzimuth, azimuth, xyDistance);

       int azimuthIdx = static_cast<int>(azimuth * 100 + 0.5);
       if (azimuthIdx >= CIRCLE)
       {
         azimuthIdx -= CIRCLE;
       }
       else if (azimuthIdx < 0)
       {
         azimuthIdx += CIRCLE;
       }

       // Skip wrong points
       if (distance < 0.1 || distance > 500.0)
        continue;

       //double x = xyDistance * sin(degreeToRadian(azimuth)) ;//this->Sin_all_angle[azimuthIdx];
       //double y = xyDistance * cos(degreeToRadian(azimuth)); //this->Cos_all_angle[azimuthIdx];
       //double z = distance * sin(degreeToRadian(pitch));//this->Sin_all_angle[static_cast<int>(pitch * 100 + 0.5)];

       double x = xyDistance * this->Sin_all_angle[azimuthIdx];
       double y = xyDistance * this->Cos_all_angle[azimuthIdx];
       double z = distance * this->Sin_all_angle[static_cast<int>(pitch * 100 + 0.5)];


       double intensity = currentBlock.units[laserID].GetIntensity();

       // Compute timestamp of the point
       timestamp += this->LaserOffset.getBlockTS(blockID, returnMode, mode, PANDAR128_LIDAR_NUM) / 1000000000.0 + offset / 1000000000.0;

       if (current_pt_id >= PANDAR128_POINT_NUM)
       {
          // SplitFrame for safety to not overflow allcoated arrays
          vtkWarningMacro("Received more datapoints than expected");
          this->SplitFrame();
       }
       this->Points->SetPoint(current_pt_id, x, y, z);

       TrySetValue(this->PointsX, current_pt_id, x);
       TrySetValue(this->PointsY, current_pt_id, y);
       TrySetValue(this->PointsZ, current_pt_id, z);

       TrySetValue(this->LaserID, current_pt_id, laserID);
       TrySetValue(this->Intensities, current_pt_id, intensity);
       TrySetValue(this->Timestamps, current_pt_id, timestamp);
       TrySetValue(this->Distances, current_pt_id, distance);
      current_pt_id++;

       /*int index;
       if (LIDAR_RETURN_BLOCK_SIZE_2 == m_iReturnBlockSize)
       {
         index = (block.fAzimuth - start_angle_) / m_iAngleSize * PANDAR128_LASER_NUM *
                     m_iReturnBlockSize +
                 PANDAR128_LASER_NUM * blockid + i;
         // ROS_WARN("block 2 index:[%d]",index);
       } else {
         index = (block.fAzimuth - start_angle_) / m_iAngleSize * PANDAR128_LASER_NUM + i;
       }
       cld->points[index] = point;*/
     }
  }

  auto stop = high_resolution_clock::now();
  duration<double, std::micro> ms_double = stop - start;
  //std::cout << ms_double.count() << "micro seconds\n";
}

//-----------------------------------------------------------------------------
bool vtkWingtechPacketInterpreter::IsLidarPacket(unsigned char const * vtkNotUsed(data),
                                              unsigned int vtkNotUsed(dataLength))
{
  //std::cout << "dataLength  " << dataLength << std::endl;
  //std::cout << "PANDAR128_PACKET_SIZE  " << PANDAR128_PACKET_SIZE << std::endl;
  //std::cout << "sizeof(Pandar128Packet)  " << sizeof(Pandar128Packet) << std::endl;

  return true; //dataLength == PACKET_SIZE;
}



//-----------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> vtkWingtechPacketInterpreter::CreateNewEmptyFrame(vtkIdType numberOfPoints,
                                                                            vtkIdType vtkNotUsed(prereservedNumberOfPoints))
{
  const int defaultPrereservedNumberOfPointsPerFrame = PANDAR128_POINT_NUM;
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
  return polyData;
}

//-----------------------------------------------------------------------------
bool vtkWingtechPacketInterpreter::PreProcessPacket(unsigned char const* data,
  unsigned int vtkNotUsed(dataLength), fpos_t filePosition, double packetNetworkTime,
  std::vector<FrameInformation>* frameCatalog)
{
  bool isNewFrame = false;

  this->ParserMetaData.FilePosition = filePosition;
  this->ParserMetaData.FirstPacketDataTime = 0.0; // TODO
  this->ParserMetaData.FirstPacketNetworkTime = packetNetworkTime;

  const Pandar128Packet* dataPacket = reinterpret_cast<const Pandar128Packet*>(data);


  for(int blockID = 0; blockID < PANDAR128_BLOCK_NUM ; blockID++)
  {
    Pandar128Block currentBlock = dataPacket->blocks[blockID];

    WingtechSpecificFrameInformation* frameInfo =
        reinterpret_cast<WingtechSpecificFrameInformation*>(this->ParserMetaData.SpecificInformation.get());
    if(frameInfo->IsNewFrame(0, currentBlock.GetAzimuth()) && frameCatalog)
    {
      isNewFrame = true;
      frameCatalog->push_back(this->ParserMetaData);
    }

  }

  return isNewFrame;
}


//-----------------------------------------------------------------------------
std::string vtkWingtechPacketInterpreter::GetSensorInformation(bool vtkNotUsed(shortVersion))
{
  return "Wingtech Sensor";
}

