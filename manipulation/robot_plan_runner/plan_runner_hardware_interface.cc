#include <fstream>

#include "drake/lcmt_contact_info.hpp"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"
#include "drake/manipulation/robot_plan_runner/contact_location_estimator.h"
#include "drake/manipulation/robot_plan_runner/plan_runner_hardware_interface.h"
#include "drake/manipulation/robot_plan_runner/robot_plan_runner.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using robot_plans::PlanData;
using std::cout;
using std::endl;

PlanRunnerHardwareInterface::PlanRunnerHardwareInterface(
    const std::vector<PlanData>& plan_list, bool listen_to_contact_info)
    : owned_lcm_(new lcm::DrakeLcm()),
      listen_to_contact_info_(listen_to_contact_info) {
  // create diagram system.
  systems::DiagramBuilder<double> builder;

  auto lcm = builder.template AddSystem<systems::lcm::LcmInterfaceSystem>(
      owned_lcm_.get());

  // Receive iiwa status.
  iiwa_status_sub_ = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm));
  auto iiwa_status_receiver =
      builder.template AddSystem<manipulation::kuka_iiwa::IiwaStatusReceiver>();
  builder.Connect(iiwa_status_sub_->get_output_port(),
                  iiwa_status_receiver->get_input_port());

  // Publish iiwa command.
  auto iiwa_command_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm, 0.005));
  auto iiwa_command_sender =
      builder.template AddSystem<manipulation::kuka_iiwa::IiwaCommandSender>();
  builder.Connect(iiwa_command_sender->get_output_port(),
                  iiwa_command_pub->get_input_port());

  // Add PlanSender and PlanRunner.
  plan_runner_ = builder.template AddSystem<RobotPlanRunner>(false, 0.005);
  plan_sender_ = builder.template AddSystem<PlanSender>(plan_list);
  builder.Connect(plan_sender_->GetOutputPort("plan_data"),
                  plan_runner_->GetInputPort("plan_data"));

  builder.Connect(iiwa_status_receiver->get_position_measured_output_port(),
                  plan_sender_->GetInputPort("q"));
  builder.Connect(iiwa_status_receiver->get_position_measured_output_port(),
                  plan_runner_->GetInputPort("iiwa_position_measured"));
  builder.Connect(iiwa_status_receiver->get_velocity_estimated_output_port(),
                  plan_runner_->GetInputPort("iiwa_velocity_estimated"));
  builder.Connect(iiwa_status_receiver->get_torque_external_output_port(),
                  plan_runner_->GetInputPort("iiwa_torque_external"));

  builder.Connect(plan_runner_->GetOutputPort("iiwa_position_command"),
                  iiwa_command_sender->get_position_input_port());
  builder.Connect(plan_runner_->GetOutputPort("iiwa_torque_command"),
                  iiwa_command_sender->get_torque_input_port());

  if (listen_to_contact_info_) {
    // Contact location estimator.
    contact_info_sub_ = builder.template AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_contact_info>(
            "CONTACT_INFO", lcm));
    auto contact_location_estimator =
        builder.template AddSystem<ContactLocationEstimator>();

    builder.Connect(
        contact_info_sub_->get_output_port(),
        contact_location_estimator->GetInputPort("lcmt_contact_info"));
    builder.Connect(contact_location_estimator->GetOutputPort("contact_info"),
                    plan_runner_->GetInputPort("contact_info"));

    // Loggers for contact location estimator.
    auto contact_info_translator =
        builder.template AddSystem<ContactInfoTranslator>();
    builder.Connect(contact_location_estimator->GetOutputPort("contact_info"),
                    contact_info_translator->GetInputPort("contact_info"));

    logger_num_contacts_ = systems::LogOutput(
        contact_info_translator->GetOutputPort("num_contacts"), &builder);
    logger_num_contacts_->set_publish_period(0.005);

    logger_contact_link_ = systems::LogOutput(
        contact_info_translator->GetOutputPort("contact_link"), &builder);
    logger_contact_link_->set_publish_period(0.005);

    logger_contact_position_ = systems::LogOutput(
        contact_info_translator->GetOutputPort("contact_position"), &builder);
    logger_contact_position_->set_publish_period(0.005);
  }

  diagram_ = builder.Build();
};

void PlanRunnerHardwareInterface::SaveGraphvizStringToFile(
    const std::string& file_name) {
  if (diagram_) {
    std::ofstream out(file_name);
    out << diagram_->GetGraphvizString();
    out.close();
  }
}

lcmt_iiwa_status PlanRunnerHardwareInterface::GetCurrentIiwaStatus() {
  // create diagram system.
  systems::DiagramBuilder<double> builder;
  auto lcm = builder.template AddSystem<systems::lcm::LcmInterfaceSystem>(
      new lcm::DrakeLcm());

  // Receive iiwa status.
  auto iiwa_status_sub = builder.template AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  WaitForNewMessage(lcm, iiwa_status_sub);
  simulator.AdvanceTo(1e-6);

  const auto& iiwa_status_sub_context = diagram->GetMutableSubsystemContext(
      *iiwa_status_sub, &simulator.get_mutable_context());

  return iiwa_status_sub->get_output_port().Eval<lcmt_iiwa_status>(
      iiwa_status_sub_context);
}

void PlanRunnerHardwareInterface::Run(double realtime_rate) {
  systems::Simulator<double> simulator(*diagram_);
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(realtime_rate);

  if (listen_to_contact_info_) {
    WaitForNewMessage(owned_lcm_.get(), contact_info_sub_);
  } else {
    // Fix the contact_info input port to some fixed value.
    auto& plan_runner_sub_context = diagram_->GetMutableSubsystemContext(
        *plan_runner_, &simulator.get_mutable_context());
    auto port_idx = plan_runner_->GetInputPort("contact_info").get_index();
    plan_runner_sub_context.FixInputPort(
        port_idx, AbstractValue::Make<robot_plans::ContactInfo>(
                      robot_plans::ContactInfo()));
  }

  // Update the abstract state of iiwa status lcm subscriber system, so that
  // actual robot state can be obtained when its output ports are evaluated at
  // initialization.
  auto& iiwa_status_sub_context = diagram_->GetMutableSubsystemContext(
      *iiwa_status_sub_, &simulator.get_mutable_context());
  auto& state =
      iiwa_status_sub_context.get_mutable_abstract_state<lcmt_iiwa_status>(0);
  state = this->GetCurrentIiwaStatus();

  double t_total = plan_sender_->get_all_plans_duration();
  cout << "All plans duration " << t_total << endl;
  simulator.AdvanceTo(t_total);

  if (listen_to_contact_info_) {
    // Save logs to disk.
    SaveLogToFile(logger_num_contacts_, "num_contacts");
    SaveLogToFile(logger_contact_link_, "contact_link");
    SaveLogToFile(logger_contact_position_, "contact_position");
  }
}

void WaitForNewMessage(drake::lcm::DrakeLcmInterface* const lcm_ptr,
                       systems::lcm::LcmSubscriberSystem* const lcm_sub_ptr) {
  auto wait_for_new_message = [lcm_ptr](const auto& lcm_sub) {
    std::cout << "Waiting for " << lcm_sub.get_channel_name() << " message..."
              << std::flush;
    const int orig_count = lcm_sub.GetInternalMessageCount();
    LcmHandleSubscriptionsUntil(
        lcm_ptr,
        [&]() { return lcm_sub.GetInternalMessageCount() > orig_count; },
        10 /* timeout_millis */);
    std::cout << "Received!" << std::endl;
  };

  wait_for_new_message(*lcm_sub_ptr);
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
