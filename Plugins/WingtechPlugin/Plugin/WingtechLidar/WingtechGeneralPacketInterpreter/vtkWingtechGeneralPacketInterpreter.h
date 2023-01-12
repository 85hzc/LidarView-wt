#ifndef vtkWingtechGeneralPacketInterpreter_h
#define vtkWingtechGeneralPacketInterpreter_h

#include <vtkLidarPacketInterpreter.h>

#include <vtkDoubleArray.h>
#include <vtkTypeUInt64Array.h>
#include <vtkUnsignedIntArray.h>

#include <pandarGeneral_internal.h>
//#include <pandarXT.h>

#include <memory>

class VTK_EXPORT vtkWingtechGeneralPacketInterpreter : public vtkLidarPacketInterpreter
{
public:
  static vtkWingtechGeneralPacketInterpreter* New();
  vtkTypeMacro(vtkWingtechGeneralPacketInterpreter, vtkLidarPacketInterpreter)

  void LoadCalibration(const std::string& filename) override;

  bool PreProcessPacket(unsigned char const * data, unsigned int dataLength,
                        fpos_t filePosition = fpos_t(), double packetNetworkTime = 0,
                        std::vector<FrameInformation>* frameCatalog = nullptr) override;

  bool IsLidarPacket(unsigned char const * data, unsigned int dataLength) override;

  void ProcessPacket(unsigned char const * data, unsigned int dataLength) override;

  std::string GetSensorInformation(bool shortVersion = false) override;

  vtkSetMacro(DualReturn, bool)
  vtkGetMacro(DualReturn, bool)

protected:
  template<typename T>
  vtkSmartPointer<T> CreateDataArray(bool isAdvanced, const char* name, vtkIdType np, vtkIdType prereserved_np, vtkPolyData* pd);

  vtkSmartPointer<vtkPolyData> CreateNewEmptyFrame(vtkIdType numberOfPoints, vtkIdType prereservedNumberOfPoints = 60000) override;

  vtkSmartPointer<vtkPoints> Points;
  vtkSmartPointer<vtkDoubleArray> PointsX;
  vtkSmartPointer<vtkDoubleArray> PointsY;
  vtkSmartPointer<vtkDoubleArray> PointsZ;

  vtkSmartPointer<vtkUnsignedIntArray> LaserID;
  vtkSmartPointer<vtkDoubleArray> Intensities;
  vtkSmartPointer<vtkDoubleArray> Timestamps;
  vtkSmartPointer<vtkDoubleArray> Distances;


  vtkWingtechGeneralPacketInterpreter();
  ~vtkWingtechGeneralPacketInterpreter();

private:
  vtkWingtechGeneralPacketInterpreter(const vtkWingtechGeneralPacketInterpreter&) = delete;
  void operator=(const vtkWingtechGeneralPacketInterpreter&) = delete;

  int current_pt_id = 0;

  // add from official driver
  std::string m_sLidarType = "PandarXT-32";
  int start_angle_ = 0;

  uint16_t last_azimuth_ = 0;
  double last_timestamp_ = 0;

  float General_elev_angle_map_[MAX_LASER_NUM];
  float General_horizatal_azimuth_offset_map_[MAX_LASER_NUM];

  float blockXTOffsetSingle_[HS_LIDAR_XT_BLOCK_NUMBER];
  float blockXTOffsetDual_[HS_LIDAR_XT_BLOCK_NUMBER];
  float blockXTOffsetTriple_[HS_LIDAR_XT_BLOCK_NUMBER];
  float laserXTOffset_[HS_LIDAR_XT_UNIT_NUM];

  std::vector<float> m_sin_azimuth_map_;
  std::vector<float> m_cos_azimuth_map_;
  std::vector<float> m_sin_elevation_map_;
  std::vector<float> m_cos_elevation_map_;
  std::vector<float> m_sin_azimuth_map_h;
  std::vector<float> m_cos_azimuth_map_h;
  std::vector<float> m_sin_azimuth_map_b;
  std::vector<float> m_cos_azimuth_map_b;

  bool m_bCoordinateCorrectionFlag = true;

  bool DualReturn = false;
};

struct WingtechGeneralSpecificFrameInformation : public SpecificFrameInformation
{
  int blockOffset = 0;
  void reset() override { *this = WingtechGeneralSpecificFrameInformation(); }
  std::unique_ptr<SpecificFrameInformation> clone() override { return std::make_unique<WingtechGeneralSpecificFrameInformation>(*this); }
};



#endif // vtkWingtechGeneralPacketInterpreter_h
