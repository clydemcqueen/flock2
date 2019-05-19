#ifndef CONTEXT_MACROS_HPP
#define CONTEXT_MACROS_HPP

#define CXT_MACRO_MEMBER_DEF(n, t, d) t n##_{d};

#define CXT_MACRO_LOAD_PARAM(node_ref, cxt_ref, n, t, d) node_ref.get_parameter<t>(#n, cxt_ref.n##_);

// TODO catch type exceptions
#define CXT_MACRO_CHANGE_PARAM(cxt_ref, n, t) \
if (parameter.get_name() == #n) {\
  cxt_ref.n##_ = parameter.get_value<t>(); \
  std::cout << "change value of " << #n << " to " << cxt_ref.n##_ << std::endl; \
}

// Initialize the context struct
#define CXT_MACRO_INIT_PARAMETERS(validate_func) \
CXT_MACRO_ALL_PARAMS \
validate_func() \

// Regester for parameter change notifications
#define CXT_MACRO_CHANGE_PARAMETERS(node_ref, cxt_ref, validate_func) \
node_ref.register_param_change_callback( \
[this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult \
{ \
  for (auto parameter : parameters) { \
    CXT_MACRO_ALL_PARAMS \
  } \
  validate_func(); \
  auto result = rcl_interfaces::msg::SetParametersResult(); \
  result.successful = true; \
  return result; \
})

#endif // CONTEXT_MACROS_HPP
