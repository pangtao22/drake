#include <fstream>

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
using systems::BasicVector;
using systems::SignalLogger;

class ContactInfoTranslator : public systems::LeafSystem<double> {
 public:
  ContactInfoTranslator() {
    this->set_name("ContactInfoTranslator");

    this->DeclareAbstractInputPort("contact_info", Value<ContactInfo>());

    this->DeclareVectorOutputPort("num_contacts", BasicVector<double>(1),
                                  &ContactInfoTranslator::GetNumContacts);
    this->DeclareVectorOutputPort("contact_link", BasicVector<double>(1),
                                  &ContactInfoTranslator::GetContactLink);
    this->DeclareVectorOutputPort("contact_position", BasicVector<double>(3),
                                  &ContactInfoTranslator::GetContactPosition);
  }

 private:
  void GetNumContacts(const systems::Context<double>& context,
                      BasicVector<double>* output) const {
    const auto& contact_info =
        this->get_input_port(0).Eval<ContactInfo>(context);
    output->SetFromVector(
        Eigen::Matrix<double, 1, 1>(contact_info.num_contacts));
  }

  void GetContactLink(const systems::Context<double>& context,
                      BasicVector<double>* output) const {
    const auto& contact_info =
        this->get_input_port(0).Eval<ContactInfo>(context);
    output->SetFromVector(
        Eigen::Matrix<double, 1, 1>(contact_info.contact_link_idx[0]));
  }

  void GetContactPosition(const systems::Context<double>& context,
                          BasicVector<double>* output) const {
    const auto& contact_info =
        this->get_input_port(0).Eval<ContactInfo>(context);

    output->SetFromVector(contact_info.positions[0]);
  }
};

template <class T>
void SaveLogToFile(systems::SignalLogger<T>* const logger,
                   const std::string& name) {
  std::ofstream file_data(name + "_data.csv");
  std::ofstream file_time(name + "_time.csv");
  const Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                  ", ", "\n");
  file_data << logger->data().format(CSVFormat);
  file_time << logger->sample_times().format(CSVFormat);

  file_data.close();
  file_time.close();
}

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