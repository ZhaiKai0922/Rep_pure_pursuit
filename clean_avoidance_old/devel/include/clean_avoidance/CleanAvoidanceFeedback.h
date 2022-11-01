// Generated by gencpp from file clean_avoidance/CleanAvoidanceFeedback.msg
// DO NOT EDIT!


#ifndef CLEAN_AVOIDANCE_MESSAGE_CLEANAVOIDANCEFEEDBACK_H
#define CLEAN_AVOIDANCE_MESSAGE_CLEANAVOIDANCEFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace clean_avoidance
{
template <class ContainerAllocator>
struct CleanAvoidanceFeedback_
{
  typedef CleanAvoidanceFeedback_<ContainerAllocator> Type;

  CleanAvoidanceFeedback_()
    : feedback_x(0.0)
    , feedback_y(0.0)  {
    }
  CleanAvoidanceFeedback_(const ContainerAllocator& _alloc)
    : feedback_x(0.0)
    , feedback_y(0.0)  {
  (void)_alloc;
    }



   typedef double _feedback_x_type;
  _feedback_x_type feedback_x;

   typedef double _feedback_y_type;
  _feedback_y_type feedback_y;





  typedef boost::shared_ptr< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct CleanAvoidanceFeedback_

typedef ::clean_avoidance::CleanAvoidanceFeedback_<std::allocator<void> > CleanAvoidanceFeedback;

typedef boost::shared_ptr< ::clean_avoidance::CleanAvoidanceFeedback > CleanAvoidanceFeedbackPtr;
typedef boost::shared_ptr< ::clean_avoidance::CleanAvoidanceFeedback const> CleanAvoidanceFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator1> & lhs, const ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.feedback_x == rhs.feedback_x &&
    lhs.feedback_y == rhs.feedback_y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator1> & lhs, const ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace clean_avoidance

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "28b2f4c6bff997642d6c7dab1fe99c89";
  }

  static const char* value(const ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x28b2f4c6bff99764ULL;
  static const uint64_t static_value2 = 0x2d6c7dab1fe99c89ULL;
};

template<class ContainerAllocator>
struct DataType< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "clean_avoidance/CleanAvoidanceFeedback";
  }

  static const char* value(const ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"float64 feedback_x\n"
"float64 feedback_y\n"
"\n"
;
  }

  static const char* value(const ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.feedback_x);
      stream.next(m.feedback_y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CleanAvoidanceFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::clean_avoidance::CleanAvoidanceFeedback_<ContainerAllocator>& v)
  {
    s << indent << "feedback_x: ";
    Printer<double>::stream(s, indent + "  ", v.feedback_x);
    s << indent << "feedback_y: ";
    Printer<double>::stream(s, indent + "  ", v.feedback_y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CLEAN_AVOIDANCE_MESSAGE_CLEANAVOIDANCEFEEDBACK_H