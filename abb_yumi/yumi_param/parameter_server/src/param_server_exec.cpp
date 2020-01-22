#include <memory>
#include <parameter_server/parameter_server.hpp>

void
print_usage()
{
  fprintf(stderr, "lobot_param_server [yaml1.file ... yamlN.file]\n");
}

int main(int argc, char ** argv)
{
  // fprintf(stderr, "Arg: %s", argv[1]);
  rclcpp::init(argc, argv);

  if (argc < 2) {
    fprintf(stderr, "Missing arguments. No yaml files were given\n");
    print_usage();
    return -1;
  }
  auto nodeOptions = rclcpp::NodeOptions();
  nodeOptions.start_parameter_services(false);
  nodeOptions.allow_undeclared_parameters(true);
  auto ps = std::make_shared<parameter_server::ParameterServer>(nodeOptions);
  ps->load_parameters(argv[1]);

  rclcpp::spin(ps);
  rclcpp::shutdown();
  return 0;
}
