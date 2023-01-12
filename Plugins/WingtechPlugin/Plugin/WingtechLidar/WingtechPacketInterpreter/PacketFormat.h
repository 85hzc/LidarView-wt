#ifndef PacketFormat_H
#define PacketFormat_H

#include <boost/endian/arithmetic.hpp>
#include <memory>

//----------------------------------------------------------------------------
//! @brief Simple getter that handles conversion to native unsigned integer types.
#define GET_NATIVE_UINT(n, attr) uint ## n ##_t Get ## attr() const { return this->attr; }
#define SET_NATIVE_UINT(n, attr) void Set ## attr(uint ## n ##_t x) { this->attr = x; }

#define BIT(n)                  ( 1<<(n) )
//! Create a bitmask of length \a len.
#define BIT_MASK(len)           ( BIT(len)-1 )

//! Create a bitfield mask of length \a starting at bit \a start.
#define BF_MASK(start, len)     ( BIT_MASK(len)<<(start) )

//! Extract a bitfield of length \a len starting at bit \a start from \a y.
#define BF_GET(y, start, len)   ( ((y)>>(static_cast<decltype(y)>(start))) & BIT_MASK(len) )

//! Prepare a bitmask for insertion or combining.
#define BF_PREP(x, start, len)  ( ((x)&BIT_MASK(len)) << (start) )

//! Insert a new bitfield value x into y.
#define BF_SET(y, x, start, len)    ( y= ((y) &~ BF_MASK(start, len)) | BF_PREP(x, start, len) )

#define PANDAR128_POINT_NUM (460800)
#define PANDAR128_LASER_NUM (128)
#define PANDAR128_BLOCK_NUM (2)
#define MAX_BLOCK_NUM (8)
#define PANDAR128_DISTANCE_UNIT (0.004)
#define PANDAR128_SOB_SIZE (2)
#define PANDAR128_VERSION_MAJOR_SIZE (1)
#define PANDAR128_VERSION_MINOR_SIZE (1)
#define PANDAR128_HEAD_RESERVED1_SIZE (2)
#define PANDAR128_LASER_NUM_SIZE (1)
#define PANDAR128_BLOCK_NUM_SIZE (1)
#define PANDAR128_ECHO_COUNT_SIZE (1)
#define PANDAR128_ECHO_NUM_SIZE (1)
#define PANDAR128_HEAD_RESERVED2_SIZE (2)
#define PANDAR128_HEAD_SIZE                                       \
  (PANDAR128_SOB_SIZE + PANDAR128_VERSION_MAJOR_SIZE +            \
   PANDAR128_VERSION_MINOR_SIZE + PANDAR128_HEAD_RESERVED1_SIZE + \
   PANDAR128_LASER_NUM_SIZE + PANDAR128_BLOCK_NUM_SIZE +          \
   PANDAR128_ECHO_COUNT_SIZE + PANDAR128_ECHO_NUM_SIZE +          \
   PANDAR128_HEAD_RESERVED2_SIZE)
#define PANDAR128_AZIMUTH_SIZE (2)
#define DISTANCE_SIZE (2)
#define INTENSITY_SIZE (1)
#define CONFIDENCE_SIZE (1)
#define PANDAR128_UNIT_SIZE (DISTANCE_SIZE + INTENSITY_SIZE)
#define PANDAR128_BLOCK_SIZE \
  (PANDAR128_UNIT_SIZE * PANDAR128_LASER_NUM + PANDAR128_AZIMUTH_SIZE)
#define PANDAR128_TAIL_RESERVED1_SIZE (3)
#define PANDAR128_TAIL_RESERVED2_SIZE (3)
#define PANDAR128_SHUTDOWN_FLAG_SIZE (1)
#define PANDAR128_TAIL_RESERVED3_SIZE (3)
#define PANDAR128_MOTOR_SPEED_SIZE (2)
#define PANDAR128_TS_SIZE (4)
#define PANDAR128_RETURN_MODE_SIZE (1)
#define PANDAR128_FACTORY_INFO (1)
#define PANDAR128_UTC_SIZE (6)
#define PANDAR128_TAIL_SIZE                                        \
  (PANDAR128_TAIL_RESERVED1_SIZE + PANDAR128_TAIL_RESERVED2_SIZE + \
   PANDAR128_SHUTDOWN_FLAG_SIZE + PANDAR128_TAIL_RESERVED3_SIZE +  \
   PANDAR128_MOTOR_SPEED_SIZE + PANDAR128_TS_SIZE +                \
   PANDAR128_RETURN_MODE_SIZE + PANDAR128_FACTORY_INFO + PANDAR128_UTC_SIZE)
#define PANDAR128_PACKET_SIZE                                         \
  (PANDAR128_HEAD_SIZE + PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM + \
   PANDAR128_TAIL_SIZE)
#define PANDAR128_SEQ_NUM_SIZE (4)
#define PANDAR128_PACKET_SEQ_NUM_SIZE \
  (PANDAR128_PACKET_SIZE + PANDAR128_SEQ_NUM_SIZE)
#define PANDAR128_WITHOUT_CONF_UNIT_SIZE (DISTANCE_SIZE + INTENSITY_SIZE)

#define TASKFLOW_STEP_SIZE (225)
#define PANDAR128_CRC_SIZE (4)
#define PANDAR128_FUNCTION_SAFETY_SIZE (17)

#define CIRCLE (36000)

#pragma pack(push, 1)
//! @brief class representing the raw measure
/*
   0               1               2               3
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |           Distance         |            Intensity             |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
class Pandar128Unit
{
private :
  // Distance
  boost::endian::little_uint16_t Distance;

  // Intensity
  boost::endian::little_uint8_t Intensity;

public :
  GET_NATIVE_UINT(16, Distance)
  SET_NATIVE_UINT(16, Distance)
  GET_NATIVE_UINT(8, Intensity)
  SET_NATIVE_UINT(8, Intensity)
};
#pragma pack(pop)



#pragma pack(push, 1)
//! @brief class representing the raw block
/*
   0               1               2
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
  |             Azimuth            |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-

 */
class Pandar128Block
{
private :

  // Azimuth
  boost::endian::little_uint16_t Azimuth;

public :
  Pandar128Unit units[PANDAR128_LASER_NUM];

public :
  GET_NATIVE_UINT(16, Azimuth)
  SET_NATIVE_UINT(16, Azimuth)
};
#pragma pack(pop)




#pragma pack(push, 1)
//! @brief class representing the Raw packet
/*
   0               1               2               3
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |               Sob              |  VersionMajor | VersionMinor |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    DistUnit    |     Flags     |   LaserNum    |  BlockNum    |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    EchoCount   |   EchoNum     |          Reserved            |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
class Pandar128Header
{
private:
  boost::endian::little_uint16_t Sob;

  boost::endian::little_uint8_t VersionMajor;

  boost::endian::little_uint8_t VersionMinor;

  boost::endian::little_uint8_t DistUnit;

  boost::endian::little_uint8_t Flags;

  boost::endian::little_uint8_t LaserNum;

  boost::endian::little_uint8_t BlockNum;

  boost::endian::little_uint8_t EchoCount;

  boost::endian::little_uint8_t EchoNum;

  boost::endian::little_uint16_t Reserved;

public:
  GET_NATIVE_UINT(16, Sob)
  SET_NATIVE_UINT(16, Sob)
  GET_NATIVE_UINT(8, VersionMajor)
  SET_NATIVE_UINT(8, VersionMajor)
  GET_NATIVE_UINT(8, VersionMinor)
  SET_NATIVE_UINT(8, VersionMinor)
  GET_NATIVE_UINT(8, DistUnit)
  SET_NATIVE_UINT(8, DistUnit)
  GET_NATIVE_UINT(8, Flags)
  SET_NATIVE_UINT(8, Flags)
  GET_NATIVE_UINT(8, LaserNum)
  SET_NATIVE_UINT(8, LaserNum)
  GET_NATIVE_UINT(8, BlockNum)
  SET_NATIVE_UINT(8, BlockNum)
  GET_NATIVE_UINT(8, EchoCount)
  SET_NATIVE_UINT(8, EchoCount)
  GET_NATIVE_UINT(8, EchoNum)
  SET_NATIVE_UINT(8, EchoNum)
  GET_NATIVE_UINT(16, Reserved)
  SET_NATIVE_UINT(16, Reserved)
};
#pragma pack(pop)



// https://github.com/HesaiTechnology/HesaiLidar_Pandar128_ROS/blob/160436018cbe4c96249c281499e2e9638d38b39c/pandar_pointcloud/src/conversions/convert.cc
// https://github.com/HesaiTechnology/HesaiLidar_Pandar128_ROS/blob/160436018cbe4c96249c281499e2e9638d38b39c/pandar_pointcloud/src/conversions/convert.h

#pragma pack(push, 1)
//! @brief class representing the Raw packet
/*
   0               1               2               3
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    Reserved1   |   Reserved1   |  Reserved1    |  Reserved2   |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    Reserved2   |   Reserved2   | ShutdownFlag  |  Reserved3   |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |    Reserved3   |   Reserved3   |         MotorSpeed           |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                            Timestamp                          |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |   ReturnMode   | FactoryInfo   |   UTCTime     |   UTCTime    |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |     UTCTime    |    UTCTime    |   UTCTime     |  UTCTime     |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                             SeqNum                            |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
class Pandar128Tail
{
private:
  boost::endian::little_uint8_t Reserved11;
  boost::endian::little_uint8_t Reserved12;
  boost::endian::little_uint8_t Reserved13;

  boost::endian::little_uint8_t Reserved21;
  boost::endian::little_uint8_t Reserved22;
  boost::endian::little_uint8_t Reserved23;

  boost::endian::little_uint16_t Padding1;  //+
  boost::endian::little_uint16_t Padding2;  //+
  boost::endian::little_uint8_t Padding3;  //+

  boost::endian::little_uint8_t ShutdownFlag;

  //boost::endian::little_uint8_t Reserved3[3];

  boost::endian::little_uint8_t ReturnMode;

  boost::endian::little_uint16_t MotorSpeed;

  boost::endian::little_uint8_t UTCTime0;
  boost::endian::little_uint8_t UTCTime1;
  boost::endian::little_uint8_t UTCTime2;
  boost::endian::little_uint8_t UTCTime3;
  boost::endian::little_uint8_t UTCTime4;
  boost::endian::little_uint8_t UTCTime5;

  boost::endian::little_uint32_t Timestamp;

 // boost::endian::little_uint8_t FactoryInfo;

  boost::endian::little_uint32_t SeqNum;

public:
  GET_NATIVE_UINT(8, ShutdownFlag)
  SET_NATIVE_UINT(8, ShutdownFlag)
  GET_NATIVE_UINT(16, MotorSpeed)
  SET_NATIVE_UINT(16, MotorSpeed)
  GET_NATIVE_UINT(32, Timestamp)
  SET_NATIVE_UINT(32, Timestamp)
  GET_NATIVE_UINT(8, ReturnMode)
  SET_NATIVE_UINT(8, ReturnMode)
  GET_NATIVE_UINT(32, SeqNum)
  SET_NATIVE_UINT(32, SeqNum)

  GET_NATIVE_UINT(8, UTCTime0)
  SET_NATIVE_UINT(8, UTCTime0)
  GET_NATIVE_UINT(8, UTCTime1)
  SET_NATIVE_UINT(8, UTCTime1)
  GET_NATIVE_UINT(8, UTCTime2)
  SET_NATIVE_UINT(8, UTCTime2)
  GET_NATIVE_UINT(8, UTCTime3)
  SET_NATIVE_UINT(8, UTCTime3)
  GET_NATIVE_UINT(8, UTCTime4)
  SET_NATIVE_UINT(8, UTCTime4)
  GET_NATIVE_UINT(8, UTCTime5)
  SET_NATIVE_UINT(8, UTCTime5)

};
#pragma pack(pop)


#pragma pack(push, 1)
//! @brief class representing the Hesai Packet
struct Pandar128Packet{
  Pandar128Header header;

  Pandar128Block blocks[PANDAR128_BLOCK_NUM];

  // PANDAR128_CRC_SIZE is equal to 4
  boost::endian::little_uint32_t crc;

  // PANDAR128_FUNCTION_SAFETY_SIZE is equal to 17
  boost::endian::little_uint8_t functionSafety[17];

  Pandar128Tail tail;
};
#pragma pack(pop)

#endif // PacketFormat_H
