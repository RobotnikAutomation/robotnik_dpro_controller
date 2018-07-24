#ifndef _DYNAMIXEL_HELPER_
#define _DYNAMIXEL_HELPER_

#include <string>
#include <map>

#include <dynamixel.h>
#include <bulkread.h>

#define OPERATING_MODE 11
#define HOME_OFFSET 13
#define MAX_VELOCITY 32
#define TORQUE_ENABLE 562
#define GOAL_POSITION 596
#define GOAL_VELOCITY 600
#define GOAL_TORQUE 604
#define GOAL_ACCELERATION 606

#define PRESENT_POSITION 611
#define PRESENT_VELOCITY 615
#define PRESENT_CURRENT 621
#define INPUT_VOLTAGE 632
#define PRESENT_TEMPERATURE 625
#define HW_ERROR_STATUS 892

// other settings
#define DEFAULT_BAUDNUM 1  // 57600 bps to match Dynamixel PRO's
#define DEFAULT_BAUDRATE 57600
#define CONTROL_PERIOD (10)  // arbitrary value
#define DEFAULT_PORT "/dev/ttyUSB0"

/**
 * A struct that describes the state of a dynamixel servo motor
 */
struct DynamixelStatus
{
  int control_mode;
  bool torque_enabled;
};

struct DynamixelInfo
{
  int id;
  std::string joint_name;
  std::string model_name;
  uint16_t model_number;
  uint32_t model_info;
  int cpr;
  double gear_reduction;
  double max_vel_value;    // max velocity value to be programmed in the register
  double max_vel_rads;     // equivalent max velocity in rad/s
  std::string joint_type;  // prismatic, revolute
  double min_rad;          // max position in rad
  double max_rad;          // min position in rad (for linear axis max < min in absolute is accepted)
  double min_m;            // min position in meter
  double max_m;            // max position in meter (always max>min)
  int lim_sw_pos;          // if defined, these are indexes to digital_inputs inverted logic that mark the limit switch
  int lim_sw_neg;          // positive and negative
  double rest_position;

  int encoder_resolution;
  double angular_range;  // in rads
  double raw_speed_multiplier;

  bool invert_direction;
  bool send_to_rest_position_at_end;
  DynamixelStatus status;
};

/**
 * The different control modes available on the dynamixel servos.
 */
enum CONTROL_MODE
{
  POSITION_CONTROL = 4,
  VELOCITY_CONTROL = 1,
  TORQUE_CONTROL = 0,
  UNKNOWN = -1
};

enum HELPER_ERROR
{
  NO_ERROR,
  BAD_PORT,
  BAD_BAUDRATE,
  NO_CONNECTION,
  UNKNOWN_ID,
  UNKNOWN_CONTROL_MODE,
  BAD_TUPLE_SIZE
};

const int BAUDNUM2BAUDRATE[] = { 2400, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000, 10500000 };
class DynamixelHelper
{
private:
  std::string port_;
  int baudrate_;
  int baudnum_;

  DXL_PRO::Dynamixel* driver_;
  DXL_PRO::BulkRead* bulkread_;

  std::string last_message_error_;

  int helper_error_;

  std::map<int, DynamixelInfo> motors_info_;  // id <-> DynamixelInfo

  std::string setCommunicationError(int comm_error);
  std::string setHardwareError(int id, int hardware_error);
  std::string setHelperError(int helper_error);

  int rps2raw(int id, double rps);
  double raw2rps(int id, int raw_speed);
  int rad2raw(int id, double pos);
  double raw2rad(int id, int raw_angular);

  double clamp_rps(int id, double rps);
  int clamp_vel_raw(int id, int vel_raw);

  double clamp_pos(int id, double pos);

public:
  DynamixelHelper(std::string port, int baudrate);
  ~DynamixelHelper();

  bool connect();
  bool disconnect();
  bool enableAll();
  bool setOperatingModeAll(int new_control_mode);

  bool setTorqueEnabled(int id, int torque_enabled);
  bool setOperatingMode(int id, int new_control_mode);
  bool setHomeOffset(int id, int offset);  // to set home of prismatic joint

  bool setMultiPosition(std::map<int, std::vector<double> >& commands);
  bool setMultiVelocity(std::map<int, std::vector<double> >& commands);  // value tuples: id_motor, velocity
  bool setMultiPositionVelocity(std::map<int, std::vector<double> >& commands);

  bool getMultiData(std::map<int, std::vector<double> >& data);

  void setDynamixelInfos(std::map<int, DynamixelInfo>& motors_info_);

  bool checkId(int id);

  std::string getLastMessageError()
  {
    return last_message_error_;
  }
};

#endif  // _DYNAMIXEL_HELPER_
