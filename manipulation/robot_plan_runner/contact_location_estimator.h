#pragma once

#include "drake/manipulation/robot_plan_runner/robot_plans/contact_force_estimator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

class ContactLocationEstimator : public systems::LeafSystem<double> {
 public:
  ContactLocationEstimator();

 private:
  const systems::InputPort<double>& get_input_port() const;
  systems::EventStatus Initialize(const systems::Context<double>&,
                                  systems::State<double>*) const;
  void CopyStateOut(const systems::Context<double>&,
                    robot_plans::ContactInfo*) const;
  void UpdateContactInfo(const systems::Context<double>&,
                         systems::State<double>*) const;
  systems::AbstractStateIndex abstract_state_index_;
  std::unique_ptr<robot_plans::LowPassFilter> contact_position_filter_;
  const double update_period_;
  const double w_cutoff_;
};

class ContactInfoTranslator : public systems::LeafSystem<double> {
 public:
  ContactInfoTranslator();

 private:
  void GetNumContacts(const systems::Context<double>& context,
                      systems::BasicVector<double>* output) const;
  void GetContactLink(const systems::Context<double>& context,
                      systems::BasicVector<double>* output) const;
  void GetContactPosition(const systems::Context<double>& context,
                          systems::BasicVector<double>* output) const;
};

template <class T>
void SaveLogToFile(systems::SignalLogger<T>* const logger,
                   const std::string& name);

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
