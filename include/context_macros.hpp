#ifndef CONTEXT_MACROS_HPP
#define CONTEXT_MACROS_HPP

#define CXT_MACRO_DEFINE_MEMBER(n, t, d) t n##_{d};

#define CXT_MACRO_LOAD_PARAMETER(node_ref, cxt_ref, n, t, d) node_ref.get_parameter<t>(#n, cxt_ref.n##_);

// TODO catch type exceptions
#define CXT_MACRO_PARAMETER_CHANGED(cxt_ref, n, t) \
if (parameter.get_name() == #n) {\
  cxt_ref.n##_ = parameter.get_value<t>(); \
  std::cout << "change value of " << #n << " to " << cxt_ref.n##_ << std::endl; \
}

// Initialize the context struct
#define CXT_MACRO_INIT_PARAMETERS(all_params, validate_func) \
all_params \
validate_func() \

// Body of parameter_changed function
#define CXT_MACRO_PARAMETERS_CHANGED_BODY(all_params, parameters_list, validate_func) \
  for (const auto &parameter : parameters) { \
    all_params \
  } \
  validate_func();

// Regester for parameter changed notifications
#define CXT_MACRO_REGISTER_PARAMETERS_CHANGED(node_ref, parameter_changed_func) \
node_ref.register_param_change_callback( \
[this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult \
{ \
  parameter_changed_func(parameters); \
  auto result = rcl_interfaces::msg::SetParametersResult(); \
  result.successful = true; \
  return result; \
})

#endif // CONTEXT_MACROS_HPP
