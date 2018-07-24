#include <dpro_controller/dynamixel_helper.h>

#include <sstream>

#include <unistd.h>

#include <iostream>
#include <bitset>
#include <map>

#include <cmath>

DynamixelHelper::DynamixelHelper(std::string port = DEFAULT_PORT, int baudrate = DEFAULT_BAUDRATE)
  : port_(port), driver_(NULL), helper_error_(NO_ERROR)
{
  baudnum_ = -1;
  for (size_t i = 0; i < sizeof(BAUDNUM2BAUDRATE); i++)
  {
    if (BAUDNUM2BAUDRATE[i] == baudrate)
    {
      baudnum_ = i;
      break;
    }
  }
}

DynamixelHelper::~DynamixelHelper()
{
  disconnect();
}

bool DynamixelHelper::connect()
{
  driver_ = new DXL_PRO::Dynamixel(port_.c_str());
  if (!driver_->Connect())
  {
    setHelperError(BAD_PORT);
    return false;
  }

  if (!driver_->SetBaudrate(baudnum_))
  {
    setHelperError(BAD_BAUDRATE);
    return false;
  }

  bulkread_ = new DXL_PRO::BulkRead(driver_);
  return true;
}

bool DynamixelHelper::disconnect()
{
  if (driver_)
  {
    // disable all motors
    for (auto iter = motors_info_.begin(); iter != motors_info_.end(); iter++)
    {
      this->setTorqueEnabled(iter->second.id, 0);
    }
    driver_->Disconnect();
    delete driver_;
  }

  return true;
}

bool DynamixelHelper::enableAll()
{
  for (auto iter = motors_info_.begin(); iter != motors_info_.end(); iter++)
  {
    if (!setTorqueEnabled(iter->second.id, 1))
    {
      return false;
    }
  }

  return true;
}

bool DynamixelHelper::setOperatingModeAll(int new_control_mode)
{
  for (auto iter = motors_info_.begin(); iter != motors_info_.end(); iter++)
  {
    if (!setOperatingMode(iter->second.id, new_control_mode))
    {
      return false;
    }
  }

  return true;
}

void DynamixelHelper::setDynamixelInfos(std::map<int, DynamixelInfo>& motors_info)
{
  // TODO: check motors_info with actual data from motor?
  motors_info_ = motors_info;

  for (auto iter = motors_info_.begin(); iter != motors_info_.end(); iter++)
  {
    DynamixelStatus status;

    status.control_mode = UNKNOWN;
    status.torque_enabled = false;

    iter->second.status = status;
  }
}

bool DynamixelHelper::checkId(int id)
{
  return motors_info_.find(id) != motors_info_.end();
}

int DynamixelHelper::rps2raw(int id, double rps)
{
  if (!checkId(id))
  {
    setHelperError(UNKNOWN_ID);
    return 0;
  }

  double rps2raw =
      motors_info_[id].encoder_resolution / (motors_info_[id].raw_speed_multiplier * motors_info_[id].angular_range);
  int raw_speed = rps * rps2raw;

  return raw_speed;
}

double DynamixelHelper::raw2rps(int id, int raw_speed)
{
  if (!checkId(id))
  {
    setHelperError(UNKNOWN_ID);
    return 0;
  }

  double raw2rps =
      motors_info_[id].raw_speed_multiplier * motors_info_[id].angular_range / motors_info_[id].encoder_resolution;

  double rps = raw_speed * raw2rps;

  return rps;
}

int DynamixelHelper::rad2raw(int id, double rad)
{
  
  if (!checkId(id))
  {
    printf("rad2raw %d %f no checkid", id, rad);
    setHelperError(UNKNOWN_ID);
    return 0;
  }

  double rad2raw = motors_info_[id].encoder_resolution / motors_info_[id].angular_range;

  int raw_angular = rad * rad2raw;

  return raw_angular;
}

double DynamixelHelper::raw2rad(int id, int raw_angular)
{
  if (!checkId(id))
  {
    setHelperError(UNKNOWN_ID);
    return 0;
  }

  double raw2rad = motors_info_[id].angular_range / motors_info_[id].encoder_resolution;

  double rad = raw_angular * raw2rad;

  return rad;
}

double DynamixelHelper::clamp_rps(int id, double rps)
{
  if (!checkId(id))
  {
    setHelperError(UNKNOWN_ID);
    return 0;
  }

  int sign = rps < 0 ? -1 : 1;
  return (std::abs(rps) < motors_info_[id].max_vel_rads) ? rps : sign * motors_info_[id].max_vel_rads;
}

int DynamixelHelper::clamp_vel_raw(int id, int raw_vel)
{
  if (!checkId(id))
  {
    setHelperError(UNKNOWN_ID);
    return 0;
  }

  int sign = raw_vel < 0 ? -1 : 1;

  return (std::abs(raw_vel) < motors_info_[id].max_vel_value) ? raw_vel : sign * motors_info_[id].max_vel_value;
}

bool DynamixelHelper::setTorqueEnabled(int id, int torque_enabled)
{
  if (!checkId(id))
  {
    setHelperError(UNKNOWN_ID);
    return false;
  }

  int hardware_error;
  int comm_error = driver_->WriteByte(id, TORQUE_ENABLE, (int8_t)torque_enabled, &hardware_error);
  usleep(CONTROL_PERIOD * 100);

  if (comm_error != DXL_PRO::COMM_RXSUCCESS)
  {
    setCommunicationError(comm_error);
    return false;
  }
  if (hardware_error)
  {
    setHardwareError(id, hardware_error);
    return false;
  }

  motors_info_[id].status.torque_enabled = torque_enabled;
  return true;
}

bool DynamixelHelper::setOperatingMode(int id, int new_control_mode)
{
  std::stringstream error_formatter;

  // check that new_mode is a valid mode
  if (new_control_mode != POSITION_CONTROL && new_control_mode != VELOCITY_CONTROL &&
      new_control_mode != TORQUE_CONTROL)
  {
    setHelperError(UNKNOWN_CONTROL_MODE);
    return false;
  }

  if (!checkId(id))
  {
    setHelperError(UNKNOWN_ID);
    return false;
  }

  bool previous_torque_enabled = motors_info_[id].status.torque_enabled;
  int previous_control_mode = motors_info_[id].status.control_mode;

  // if no control mode change, do nothing
  if (previous_control_mode == new_control_mode)
    return true;

  // disable torque to change control mode
  if (previous_torque_enabled)
  {
    if (!setTorqueEnabled(id, 0))
    {
      return false;
    }
  }

  int hardware_error;
  int comm_error = driver_->WriteByte(id, OPERATING_MODE, (int8_t)new_control_mode, &hardware_error);
  usleep(CONTROL_PERIOD * 100);

  if (comm_error != DXL_PRO::COMM_RXSUCCESS)
  {
    setCommunicationError(comm_error);
    return false;
  }
  if (hardware_error)
  {
    setHardwareError(id, hardware_error);
    return false;
  }

  // set torque to previous
  if (!setTorqueEnabled(id, previous_torque_enabled))
  {
    return false;
  }

  motors_info_[id].status.control_mode = new_control_mode;
  return true;
}

bool DynamixelHelper::setHomeOffset(int id, int offset)
{
  std::stringstream error_formatter;
  if (!checkId(id))
  {
    setHelperError(UNKNOWN_ID);
    return false;
  }

  bool previous_torque_enabled = motors_info_[id].status.torque_enabled;

  // disable torque to enabled mode changing
  if (previous_torque_enabled)
  {
    if (!setTorqueEnabled(id, 0))
    {
      return false;
    }
  }

  int hardware_error;
  int comm_error = driver_->WriteDWord(id, HOME_OFFSET, (int32_t)offset, &hardware_error);
  usleep(CONTROL_PERIOD * 100);

  if (comm_error != DXL_PRO::COMM_RXSUCCESS)
  {
    setCommunicationError(comm_error);
    return false;
  }

  if (hardware_error)
  {
    setHardwareError(id, hardware_error);
    return false;
  }

  // set torque to previous
  if (!setTorqueEnabled(id, previous_torque_enabled))
  {
    return false;
  }

  return true;
}

bool DynamixelHelper::setMultiPosition(std::map<int, std::vector<double> >& commands)
{
  int num_actuator = commands.size();
  int single_data_size = 5;  // id (1 byte) + position (4 bytes)
  int mem_size = num_actuator * single_data_size;

  std::vector<unsigned char> vector_data(mem_size);  // to get rid of gcc warning
  unsigned char* param = vector_data.data();

  // Make syncwrite packet
  size_t i = 0;
  for (auto cmd : commands)
  {
    uint8_t motor_id = cmd.first;
    double pos_rad = cmd.second[0];
    int32_t pos_raw = rad2raw(motor_id, pos_rad);

    if (!checkId(motor_id))
    {
      setHelperError(UNKNOWN_ID);
      return false;
    }

    if (!checkId(motor_id))
    {
      setHelperError(UNKNOWN_ID);
      return false;
    }

    param[i * single_data_size + 0] = (unsigned char)motor_id;
    param[i * single_data_size + 1] = DXL_LOBYTE(DXL_LOWORD(pos_raw));
    param[i * single_data_size + 2] = DXL_HIBYTE(DXL_LOWORD(pos_raw));
    param[i * single_data_size + 3] = DXL_LOBYTE(DXL_HIWORD(pos_raw));
    param[i * single_data_size + 4] = DXL_HIBYTE(DXL_HIWORD(pos_raw));
    i++;
  }

  int comm_error = driver_->SyncWrite(GOAL_POSITION, 4, param, mem_size);
  usleep(CONTROL_PERIOD * 100);

  if (comm_error != DXL_PRO::COMM_RXSUCCESS)
  {
    setCommunicationError(comm_error);
    return false;
  }

  return true;
}

bool DynamixelHelper::setMultiPositionVelocity(std::map<int, std::vector<double> >& commands)
{
  int num_actuator = commands.size();
  int single_data_size = 9;  // id (1 byte) + position (4 bytes)
  int mem_size = num_actuator * single_data_size;

  std::vector<unsigned char> vector_data(mem_size);  // to get rid of gcc warning
  unsigned char* param = vector_data.data();

  // Make syncwrite packet
  size_t i = 0;
  for (auto cmd : commands)
  {
    uint8_t motor_id = cmd.first;

    double pos_rad = cmd.second[0];
    double vel_rps = cmd.second[1];

    int32_t pos_raw = rad2raw(motor_id, pos_rad);
    int32_t vel_raw = rps2raw(motor_id, vel_rps);
    // vel_raw = clamp_vel_raw(motor_id, vel_raw);
    vel_raw = 3;

    if (!checkId(motor_id))
    {
      setHelperError(UNKNOWN_ID);
      return false;
    }

    param[i * single_data_size + 0] = (unsigned char)motor_id;
    param[i * single_data_size + 1] = DXL_LOBYTE(DXL_LOWORD(pos_raw));
    param[i * single_data_size + 2] = DXL_HIBYTE(DXL_LOWORD(pos_raw));
    param[i * single_data_size + 3] = DXL_LOBYTE(DXL_HIWORD(pos_raw));
    param[i * single_data_size + 4] = DXL_HIBYTE(DXL_HIWORD(pos_raw));
    param[i * single_data_size + 5] = DXL_LOBYTE(DXL_LOWORD(vel_raw));
    param[i * single_data_size + 6] = DXL_HIBYTE(DXL_LOWORD(vel_raw));
    param[i * single_data_size + 7] = DXL_LOBYTE(DXL_HIWORD(vel_raw));
    param[i * single_data_size + 8] = DXL_HIBYTE(DXL_HIWORD(vel_raw));

    i++;
  }

  int comm_error = driver_->SyncWrite(GOAL_POSITION, 8, param, mem_size);  // 8 bytes
  usleep(CONTROL_PERIOD * 100);

  if (comm_error != DXL_PRO::COMM_RXSUCCESS)
  {
    setCommunicationError(comm_error);
    return false;
  }

  return true;
}

bool DynamixelHelper::setMultiVelocity(std::map<int, std::vector<double> >& commands)  // value tuples: id_motor,
                                                                                       // velocity
{
  size_t num_actuator = commands.size();
  int single_data_size = 5;  // id (1 byte) + velocity (4 bytes)
  int mem_size = num_actuator * single_data_size;

  std::vector<unsigned char> vector_data(mem_size);  // to get rid of gcc warning
  unsigned char* param = vector_data.data();

  int i = 0;
  static int iters = 0;
  for (auto cmd : commands)
  {
    uint8_t motor_id = cmd.first;
    if (!checkId(motor_id))
    {
      setHelperError(UNKNOWN_ID);
      return false;
    }
    double vel_rps = cmd.second[0];
    //        vel_rps = clamp_rps(motor_id, vel_rps);
    int32_t vel_raw = rps2raw(motor_id, vel_rps);
    vel_raw = clamp_vel_raw(motor_id, vel_raw);

    if (iters % 100 == 0)
      std::cout << "vel_raw : " << vel_raw << "\n";
    iters++;

    param[i * single_data_size + 0] = (unsigned char)motor_id;
    param[i * single_data_size + 1] = DXL_LOBYTE(DXL_LOWORD(vel_raw));
    param[i * single_data_size + 2] = DXL_HIBYTE(DXL_LOWORD(vel_raw));
    param[i * single_data_size + 3] = DXL_LOBYTE(DXL_HIWORD(vel_raw));
    param[i * single_data_size + 4] = DXL_HIBYTE(DXL_HIWORD(vel_raw));
    i++;

    //        int hardware_error;
    //        int comm_error;
    //        comm_error = driver_->WriteDWord(motor_id, GOAL_VELOCITY, vel_raw, &hardware_error);
    //        if (hardware_error) {
    //            setHardwareError(motor_id, hardware_error);
    //            return false;
    //        }
    //        if( comm_error != DXL_PRO::COMM_RXSUCCESS ) {
    //            setCommunicationError(comm_error);
    //            return false;
    //        }
    //
  }

  // TODO: con el syncwrite no hay comprobacion de errores de hardware, es decir, si le pasamos valores invalidos al
  // motor
  int comm_error = driver_->SyncWrite(GOAL_VELOCITY, 4, param, mem_size);
  usleep(CONTROL_PERIOD * 100);

  if (comm_error != DXL_PRO::COMM_RXSUCCESS)
  {
    setCommunicationError(comm_error);
    return false;
  }

  return true;
}

bool DynamixelHelper::getMultiData(std::map<int, std::vector<double> >& value_tuples)
{
  std::vector<DXL_PRO::BulkReadData> bulk_read_data;
  int num_actuator = value_tuples.size();
  int single_data_size = 12;  // position (4 bytes) + velocity (4 bytes) + padding (2 bytes) + current (2 bytes)
  int mem_size = num_actuator * single_data_size;

  std::vector<unsigned char> vector_data(mem_size);  // to get rid of gcc warning
  unsigned char* param = vector_data.data();

  // Make bulk read packet
  // for (size_t i = 0; i < value_tuples.size(); i++)
  size_t i = 0;
  for (auto d : value_tuples)
  {
    // if (value_tuples[i].size() != 4) {
    //    setHelperError(BAD_TUPLE_SIZE);
    //    return false;
    //}
    int32_t motor_id = d.first;
    if (!checkId(motor_id))
    {
      setHelperError(UNKNOWN_ID);
      return false;
    }

    DXL_PRO::BulkReadData brd;
    brd.iID = motor_id;
    brd.iStartAddr = PRESENT_POSITION;  // position = 611 (4bytes), velocity = 615 (4bytes), current = 621 (2bytes)
    brd.iLength = single_data_size;
    brd.pucTable = &param[i * single_data_size];
    bulk_read_data.push_back(brd);
    i++;
  }

  int comm_error = driver_->BulkRead(bulk_read_data);
  usleep(CONTROL_PERIOD * 100);

  if (comm_error != DXL_PRO::COMM_RXSUCCESS)
  {
    setCommunicationError(comm_error);
    return false;
  }

  for (size_t i = 0; i < value_tuples.size(); i++)
  {
    DXL_PRO::BulkReadData brd = bulk_read_data[i];
    //        value_tuples[i][0] = brd.iID; //should be the same
    int motor_id = brd.iID;
    int raw_angular =
        DXL_MAKEDWORD(DXL_MAKEWORD(brd.pucTable[0], brd.pucTable[1]), DXL_MAKEWORD(brd.pucTable[2], brd.pucTable[3]));
    int raw_speed =
        DXL_MAKEDWORD(DXL_MAKEWORD(brd.pucTable[4], brd.pucTable[5]), DXL_MAKEWORD(brd.pucTable[6], brd.pucTable[7]));
    int raw_effort = DXL_MAKEDWORD(DXL_MAKEWORD(brd.pucTable[10], brd.pucTable[11]), DXL_MAKEWORD(0, 0));

    value_tuples[motor_id][0] = raw2rad(motor_id, raw_angular);
    value_tuples[motor_id][1] = raw2rps(motor_id, raw_speed);
    value_tuples[motor_id][2] = raw_effort;
  }

  return true;
}

std::string DynamixelHelper::setCommunicationError(int comm_error)
{
  switch (comm_error)
  {
    case DXL_PRO::COMM_TXSUCCESS:
    case DXL_PRO::COMM_RXSUCCESS:
      last_message_error_ == "";
      break;
    case DXL_PRO::COMM_TXFAIL:
      last_message_error_ == "COMM_TXFAIL: Failed transmit instruction packet.";
      break;

    case DXL_PRO::COMM_TXERROR:
      last_message_error_ == "COMM_TXERROR: Incorrect instruction packet.";
      break;

    case DXL_PRO::COMM_RXFAIL:
      last_message_error_ == "COMM_RXFAIL: Failed get status packet from device.";
      break;

    case DXL_PRO::COMM_RXWAITING:
      last_message_error_ == "COMM_RXWAITING: Now recieving status packet.";
      break;

    case DXL_PRO::COMM_RXTIMEOUT:
      last_message_error_ == "COMM_RXTIMEOUT: There is no status packet.";
      break;

    case DXL_PRO::COMM_RXCORRUPT:
      last_message_error_ == "COMM_RXCORRUPT: Incorrect status packet.";
      break;

    default:
      last_message_error_ == "This is unknown error code.";
      break;
  }
  return last_message_error_;
}

std::string DynamixelHelper::setHardwareError(int id, int hardware_error)
{
  // According to E-manual 7/5/2015
  // Bit 0 Input Voltage Error / Input voltage exceeds limit
  // Bit 1 Motor hall sensor error / Motor hall sensor value exceeds limits
  // Bit 2 OverHeating Error / Internal operating temperature exceeds limit
  // Bit 3 Motor encoder error / Motor encoder not working
  // Bit 4 Electronical shock error / Electrical shock to circuit, or insufficient input voltage to drive motor

  std::stringstream hardware_error_formatter;
  hardware_error_formatter << "DXL " << id << ": ";

  if (hardware_error & DXL_PRO::ERRBIT_VOLTAGE)
  {
    hardware_error_formatter << "ERRBIT_VOLTAGE: Input voltage error.";
  }
  if (hardware_error & DXL_PRO::ERRBIT_ANGLE)
  {
    hardware_error_formatter << "ERRBIT_ANGLE: Motor hall sensor error.";
  }
  if (hardware_error & DXL_PRO::ERRBIT_OVERHEAT)
  {
    hardware_error_formatter << "ERRBIT_OVERHEAT: Overheat error.";
  }
  if (hardware_error & DXL_PRO::ERRBIT_RANGE)
  {
    hardware_error_formatter << "ERRBIT_RANGE: Motor encoder error.";
  }
  if (hardware_error & DXL_PRO::ERRBIT_CHECKSUM)
  {
    hardware_error_formatter << "ERRBIT_CHECKSUM: Electronical shock error or insuficient input voltage to drive.";
  }
  if (hardware_error & DXL_PRO::ERRBIT_OVERLOAD)
  {
    hardware_error_formatter << "ERRBIT_OVERLOAD: Overload error.";
  }
  if (hardware_error & DXL_PRO::ERRBIT_INSTRUCTION)
  {
    hardware_error_formatter << "ERRBIT_INSTRUCTION: Instruction code error.";
  }

  if (!hardware_error)
  {
    hardware_error_formatter.str(std::string());  // no error, clear the stringstream
  }

  last_message_error_ = hardware_error_formatter.str();

  return last_message_error_;
}

std::string DynamixelHelper::setHelperError(int helper_error)
{
  switch (helper_error)
  {
    case NO_ERROR:
      last_message_error_ == "";
      break;
    case BAD_PORT:
      last_message_error_ = "HELPER_ERROR: could not connect to the specified port";
      break;
    case BAD_BAUDRATE:
      last_message_error_ = "HELPER_ERROR: could not set the specified baudrate";
      break;
    case NO_CONNECTION:
      last_message_error_ = "HELPER_ERROR: not connected to the port";
      break;
    case UNKNOWN_ID:
      last_message_error_ = "HELPER_ERROR: unknown ID";
      break;
    case UNKNOWN_CONTROL_MODE:
      last_message_error_ = "HELPER_ERROR: unknown control mode";
      break;
    case BAD_TUPLE_SIZE:
      last_message_error_ = "HELPER_ERROR: bad tuple data size";
      break;
    default:
      last_message_error_ = "HELPER_ERROR: unkown error code.";
      break;
  }
  return last_message_error_;
}
