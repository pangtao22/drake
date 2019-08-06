#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_info.hpp"
#include "drake/manipulation/robot_plan_runner/contact_location_estimator.h"
#include "drake/manipulation/robot_plan_runner/plan_runner_hardware_interface.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace {

using robot_plans::ContactInfo;
using systems::SignalLogger;

int do_main() {
  systems::DiagramBuilder<double> builder;

  auto lcm = builder.template AddSystem<systems::lcm::LcmInterfaceSystem>(
      new lcm::DrakeLcm());

  auto contact_info_sub = builder.template AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_contact_info>(
          "CONTACT_INFO", lcm));

  auto contact_location_estimator =
      builder.template AddSystem<ContactLocationEstimator>();

  auto contact_info_translator =
      builder.template AddSystem<ContactInfoTranslator>();

  builder.Connect(
      contact_info_sub->get_output_port(),
      contact_location_estimator->GetInputPort("lcmt_contact_info"));

  builder.Connect(contact_location_estimator->GetOutputPort("contact_info"),
                  contact_info_translator->GetInputPort("contact_info"));

  auto logger_num_contacts = systems::LogOutput(
      contact_info_translator->GetOutputPort("num_contacts"), &builder);

  auto logger_contact_link = systems::LogOutput(
      contact_info_translator->GetOutputPort("contact_link"), &builder);

  auto logger_contact_position = systems::LogOutput(
      contact_info_translator->GetOutputPort("contact_position"), &builder);

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  WaitForNewMessage(lcm, contact_info_sub);
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.AdvanceTo(30.);

  SaveLogToFile(logger_num_contacts, "num_contacts");
  SaveLogToFile(logger_contact_link, "contact_link");
  SaveLogToFile(logger_contact_position, "contact_position");

  return 0;
}

}  // namespace
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake

int main() { return drake::manipulation::robot_plan_runner::do_main(); }