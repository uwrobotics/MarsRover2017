// Generated by gencpp from file roboteq_msgs/Status.msg
// DO NOT EDIT!


#ifndef ROBOTEQ_MSGS_MESSAGE_STATUS_H
#define ROBOTEQ_MSGS_MESSAGE_STATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace roboteq_msgs
{
template <class ContainerAllocator>
struct Status_
{
  typedef Status_<ContainerAllocator> Type;

  Status_()
    : header()
    , fault(0)
    , status(0)
    , ic_temperature(0.0)
    , internal_voltage(0.0)
    , adc_voltage(0.0)  {
    }
  Status_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , fault(0)
    , status(0)
    , ic_temperature(0.0)
    , internal_voltage(0.0)
    , adc_voltage(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _fault_type;
  _fault_type fault;

   typedef uint8_t _status_type;
  _status_type status;

   typedef float _ic_temperature_type;
  _ic_temperature_type ic_temperature;

   typedef float _internal_voltage_type;
  _internal_voltage_type internal_voltage;

   typedef float _adc_voltage_type;
  _adc_voltage_type adc_voltage;


    enum { FAULT_OVERHEAT = 1u };
     enum { FAULT_OVERVOLTAGE = 2u };
     enum { FAULT_UNDERVOLTAGE = 4u };
     enum { FAULT_SHORT_CIRCUIT = 8u };
     enum { FAULT_EMERGENCY_STOP = 16u };
     enum { FAULT_SEPEX_EXCITATION_FAULT = 32u };
     enum { FAULT_MOSFET_FAILURE = 64u };
     enum { FAULT_STARTUP_CONFIG_FAULT = 128u };
     enum { STATUS_SERIAL_MODE = 1u };
     enum { STATUS_PULSE_MODE = 2u };
     enum { STATUS_ANALOG_MODE = 4u };
     enum { STATUS_POWER_STAGE_OFF = 8u };
     enum { STATUS_STALL_DETECTED = 16u };
     enum { STATUS_AT_LIMIT = 32u };
     enum { STATUS_MICROBASIC_SCRIPT_RUNNING = 128u };
 

  typedef boost::shared_ptr< ::roboteq_msgs::Status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roboteq_msgs::Status_<ContainerAllocator> const> ConstPtr;

}; // struct Status_

typedef ::roboteq_msgs::Status_<std::allocator<void> > Status;

typedef boost::shared_ptr< ::roboteq_msgs::Status > StatusPtr;
typedef boost::shared_ptr< ::roboteq_msgs::Status const> StatusConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roboteq_msgs::Status_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roboteq_msgs::Status_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace roboteq_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'roboteq_msgs': ['/home/ivy/marsrover/MarsRover2017/Workspace/src/roboteq-indigo-devel/roboteq_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::roboteq_msgs::Status_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roboteq_msgs::Status_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roboteq_msgs::Status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roboteq_msgs::Status_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roboteq_msgs::Status_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roboteq_msgs::Status_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roboteq_msgs::Status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d3a9b223fdfb0968255e25e5a859ac29";
  }

  static const char* value(const ::roboteq_msgs::Status_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd3a9b223fdfb0968ULL;
  static const uint64_t static_value2 = 0x255e25e5a859ac29ULL;
};

template<class ContainerAllocator>
struct DataType< ::roboteq_msgs::Status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roboteq_msgs/Status";
  }

  static const char* value(const ::roboteq_msgs::Status_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roboteq_msgs::Status_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# 10Hz status message for informational/diagnostics purposes\n\
Header header\n\
\n\
uint8 FAULT_OVERHEAT=1\n\
uint8 FAULT_OVERVOLTAGE=2\n\
uint8 FAULT_UNDERVOLTAGE=4\n\
uint8 FAULT_SHORT_CIRCUIT=8\n\
uint8 FAULT_EMERGENCY_STOP=16\n\
uint8 FAULT_SEPEX_EXCITATION_FAULT=32\n\
uint8 FAULT_MOSFET_FAILURE=64\n\
uint8 FAULT_STARTUP_CONFIG_FAULT=128\n\
uint8 fault\n\
\n\
uint8 STATUS_SERIAL_MODE=1\n\
uint8 STATUS_PULSE_MODE=2\n\
uint8 STATUS_ANALOG_MODE=4\n\
uint8 STATUS_POWER_STAGE_OFF=8\n\
uint8 STATUS_STALL_DETECTED=16\n\
uint8 STATUS_AT_LIMIT=32\n\
uint8 STATUS_MICROBASIC_SCRIPT_RUNNING=128\n\
uint8 status\n\
\n\
# Temperature of main logic chip (C)\n\
float32 ic_temperature\n\
\n\
# Internal supply and reference voltage (V)\n\
float32 internal_voltage\n\
float32 adc_voltage\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::roboteq_msgs::Status_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roboteq_msgs::Status_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.fault);
      stream.next(m.status);
      stream.next(m.ic_temperature);
      stream.next(m.internal_voltage);
      stream.next(m.adc_voltage);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Status_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roboteq_msgs::Status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roboteq_msgs::Status_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "fault: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.fault);
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
    s << indent << "ic_temperature: ";
    Printer<float>::stream(s, indent + "  ", v.ic_temperature);
    s << indent << "internal_voltage: ";
    Printer<float>::stream(s, indent + "  ", v.internal_voltage);
    s << indent << "adc_voltage: ";
    Printer<float>::stream(s, indent + "  ", v.adc_voltage);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTEQ_MSGS_MESSAGE_STATUS_H
