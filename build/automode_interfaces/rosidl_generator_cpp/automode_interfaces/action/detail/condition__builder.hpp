// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from automode_interfaces:action/Condition.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "automode_interfaces/action/condition.hpp"


#ifndef AUTOMODE_INTERFACES__ACTION__DETAIL__CONDITION__BUILDER_HPP_
#define AUTOMODE_INTERFACES__ACTION__DETAIL__CONDITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "automode_interfaces/action/detail/condition__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_Goal_params
{
public:
  explicit Init_Condition_Goal_params(::automode_interfaces::action::Condition_Goal & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::action::Condition_Goal params(::automode_interfaces::action::Condition_Goal::_params_type arg)
  {
    msg_.params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_Goal msg_;
};

class Init_Condition_Goal_condition_name
{
public:
  Init_Condition_Goal_condition_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Condition_Goal_params condition_name(::automode_interfaces::action::Condition_Goal::_condition_name_type arg)
  {
    msg_.condition_name = std::move(arg);
    return Init_Condition_Goal_params(msg_);
  }

private:
  ::automode_interfaces::action::Condition_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_Goal>()
{
  return automode_interfaces::action::builder::Init_Condition_Goal_condition_name();
}

}  // namespace automode_interfaces


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_Result_message
{
public:
  explicit Init_Condition_Result_message(::automode_interfaces::action::Condition_Result & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::action::Condition_Result message(::automode_interfaces::action::Condition_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_Result msg_;
};

class Init_Condition_Result_success
{
public:
  Init_Condition_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Condition_Result_message success(::automode_interfaces::action::Condition_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Condition_Result_message(msg_);
  }

private:
  ::automode_interfaces::action::Condition_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_Result>()
{
  return automode_interfaces::action::builder::Init_Condition_Result_success();
}

}  // namespace automode_interfaces


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_Feedback_current_status
{
public:
  explicit Init_Condition_Feedback_current_status(::automode_interfaces::action::Condition_Feedback & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::action::Condition_Feedback current_status(::automode_interfaces::action::Condition_Feedback::_current_status_type arg)
  {
    msg_.current_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_Feedback msg_;
};

class Init_Condition_Feedback_condition_met
{
public:
  Init_Condition_Feedback_condition_met()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Condition_Feedback_current_status condition_met(::automode_interfaces::action::Condition_Feedback::_condition_met_type arg)
  {
    msg_.condition_met = std::move(arg);
    return Init_Condition_Feedback_current_status(msg_);
  }

private:
  ::automode_interfaces::action::Condition_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_Feedback>()
{
  return automode_interfaces::action::builder::Init_Condition_Feedback_condition_met();
}

}  // namespace automode_interfaces


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_SendGoal_Request_goal
{
public:
  explicit Init_Condition_SendGoal_Request_goal(::automode_interfaces::action::Condition_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::action::Condition_SendGoal_Request goal(::automode_interfaces::action::Condition_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_SendGoal_Request msg_;
};

class Init_Condition_SendGoal_Request_goal_id
{
public:
  Init_Condition_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Condition_SendGoal_Request_goal goal_id(::automode_interfaces::action::Condition_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Condition_SendGoal_Request_goal(msg_);
  }

private:
  ::automode_interfaces::action::Condition_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_SendGoal_Request>()
{
  return automode_interfaces::action::builder::Init_Condition_SendGoal_Request_goal_id();
}

}  // namespace automode_interfaces


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_SendGoal_Response_stamp
{
public:
  explicit Init_Condition_SendGoal_Response_stamp(::automode_interfaces::action::Condition_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::action::Condition_SendGoal_Response stamp(::automode_interfaces::action::Condition_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_SendGoal_Response msg_;
};

class Init_Condition_SendGoal_Response_accepted
{
public:
  Init_Condition_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Condition_SendGoal_Response_stamp accepted(::automode_interfaces::action::Condition_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Condition_SendGoal_Response_stamp(msg_);
  }

private:
  ::automode_interfaces::action::Condition_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_SendGoal_Response>()
{
  return automode_interfaces::action::builder::Init_Condition_SendGoal_Response_accepted();
}

}  // namespace automode_interfaces


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_SendGoal_Event_response
{
public:
  explicit Init_Condition_SendGoal_Event_response(::automode_interfaces::action::Condition_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::action::Condition_SendGoal_Event response(::automode_interfaces::action::Condition_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_SendGoal_Event msg_;
};

class Init_Condition_SendGoal_Event_request
{
public:
  explicit Init_Condition_SendGoal_Event_request(::automode_interfaces::action::Condition_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_Condition_SendGoal_Event_response request(::automode_interfaces::action::Condition_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Condition_SendGoal_Event_response(msg_);
  }

private:
  ::automode_interfaces::action::Condition_SendGoal_Event msg_;
};

class Init_Condition_SendGoal_Event_info
{
public:
  Init_Condition_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Condition_SendGoal_Event_request info(::automode_interfaces::action::Condition_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Condition_SendGoal_Event_request(msg_);
  }

private:
  ::automode_interfaces::action::Condition_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_SendGoal_Event>()
{
  return automode_interfaces::action::builder::Init_Condition_SendGoal_Event_info();
}

}  // namespace automode_interfaces


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_GetResult_Request_goal_id
{
public:
  Init_Condition_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::automode_interfaces::action::Condition_GetResult_Request goal_id(::automode_interfaces::action::Condition_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_GetResult_Request>()
{
  return automode_interfaces::action::builder::Init_Condition_GetResult_Request_goal_id();
}

}  // namespace automode_interfaces


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_GetResult_Response_result
{
public:
  explicit Init_Condition_GetResult_Response_result(::automode_interfaces::action::Condition_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::action::Condition_GetResult_Response result(::automode_interfaces::action::Condition_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_GetResult_Response msg_;
};

class Init_Condition_GetResult_Response_status
{
public:
  Init_Condition_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Condition_GetResult_Response_result status(::automode_interfaces::action::Condition_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Condition_GetResult_Response_result(msg_);
  }

private:
  ::automode_interfaces::action::Condition_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_GetResult_Response>()
{
  return automode_interfaces::action::builder::Init_Condition_GetResult_Response_status();
}

}  // namespace automode_interfaces


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_GetResult_Event_response
{
public:
  explicit Init_Condition_GetResult_Event_response(::automode_interfaces::action::Condition_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::action::Condition_GetResult_Event response(::automode_interfaces::action::Condition_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_GetResult_Event msg_;
};

class Init_Condition_GetResult_Event_request
{
public:
  explicit Init_Condition_GetResult_Event_request(::automode_interfaces::action::Condition_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_Condition_GetResult_Event_response request(::automode_interfaces::action::Condition_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Condition_GetResult_Event_response(msg_);
  }

private:
  ::automode_interfaces::action::Condition_GetResult_Event msg_;
};

class Init_Condition_GetResult_Event_info
{
public:
  Init_Condition_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Condition_GetResult_Event_request info(::automode_interfaces::action::Condition_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Condition_GetResult_Event_request(msg_);
  }

private:
  ::automode_interfaces::action::Condition_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_GetResult_Event>()
{
  return automode_interfaces::action::builder::Init_Condition_GetResult_Event_info();
}

}  // namespace automode_interfaces


namespace automode_interfaces
{

namespace action
{

namespace builder
{

class Init_Condition_FeedbackMessage_feedback
{
public:
  explicit Init_Condition_FeedbackMessage_feedback(::automode_interfaces::action::Condition_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::automode_interfaces::action::Condition_FeedbackMessage feedback(::automode_interfaces::action::Condition_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::automode_interfaces::action::Condition_FeedbackMessage msg_;
};

class Init_Condition_FeedbackMessage_goal_id
{
public:
  Init_Condition_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Condition_FeedbackMessage_feedback goal_id(::automode_interfaces::action::Condition_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Condition_FeedbackMessage_feedback(msg_);
  }

private:
  ::automode_interfaces::action::Condition_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::automode_interfaces::action::Condition_FeedbackMessage>()
{
  return automode_interfaces::action::builder::Init_Condition_FeedbackMessage_goal_id();
}

}  // namespace automode_interfaces

#endif  // AUTOMODE_INTERFACES__ACTION__DETAIL__CONDITION__BUILDER_HPP_
