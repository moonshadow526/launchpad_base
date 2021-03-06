// Generated by gencpp from file bucketbot_driver/Dockir.msg
// DO NOT EDIT!


#ifndef BUCKETBOT_DRIVER_MESSAGE_DOCKIR_H
#define BUCKETBOT_DRIVER_MESSAGE_DOCKIR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace bucketbot_driver
{
template <class ContainerAllocator>
struct Dockir_
{
  typedef Dockir_<ContainerAllocator> Type;

  Dockir_()
    : header()
    , data()  {
    }
  Dockir_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
  _data_type data;


    enum { LEFT = 1u };
     enum { CENTER = 2u };
     enum { RIGHT = 4u };
 

  typedef boost::shared_ptr< ::bucketbot_driver::Dockir_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bucketbot_driver::Dockir_<ContainerAllocator> const> ConstPtr;

}; // struct Dockir_

typedef ::bucketbot_driver::Dockir_<std::allocator<void> > Dockir;

typedef boost::shared_ptr< ::bucketbot_driver::Dockir > DockirPtr;
typedef boost::shared_ptr< ::bucketbot_driver::Dockir const> DockirConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bucketbot_driver::Dockir_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bucketbot_driver::Dockir_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bucketbot_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'bucketbot_driver': ['/home/pi/launchpad_base/src/bucketbot_driver/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bucketbot_driver::Dockir_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bucketbot_driver::Dockir_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bucketbot_driver::Dockir_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bucketbot_driver::Dockir_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bucketbot_driver::Dockir_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bucketbot_driver::Dockir_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bucketbot_driver::Dockir_<ContainerAllocator> >
{
  static const char* value()
  {
    return "68a1a4bd294604db048a5a9fd1438d38";
  }

  static const char* value(const ::bucketbot_driver::Dockir_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x68a1a4bd294604dbULL;
  static const uint64_t static_value2 = 0x048a5a9fd1438d38ULL;
};

template<class ContainerAllocator>
struct DataType< ::bucketbot_driver::Dockir_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bucketbot_driver/Dockir";
  }

  static const char* value(const ::bucketbot_driver::Dockir_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bucketbot_driver::Dockir_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Docking base ir sensor message\n\
# \n\
uint8 LEFT   = 1\n\
uint8 CENTER = 2\n\
uint8 RIGHT  = 4\n\
std_msgs/Header header\n\
uint8[] data\n\
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

  static const char* value(const ::bucketbot_driver::Dockir_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bucketbot_driver::Dockir_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Dockir_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bucketbot_driver::Dockir_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bucketbot_driver::Dockir_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BUCKETBOT_DRIVER_MESSAGE_DOCKIR_H
