
#include "drake/manipulation/robot_plan_runner/contact_location_estimator.h"
#include "drake/lcmt_contact_info.hpp"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

using Eigen::Vector3d;
using robot_plans::ContactInfo;
using std::cout;
using std::endl;

ContactLocationEstimator::ContactLocationEstimator()
    : update_period_(0.01), w_cutoff_(0.5 * 2 * M_PI) {
  this->set_name("ContactLocationEstimator");

  contact_position_filter_ = std::make_unique<robot_plans::LowPassFilter>(
      3, update_period_, w_cutoff_);

  abstract_state_index_ = this->DeclareAbstractState(
      AbstractValue::Make<ContactInfo>(ContactInfo()));

  this->DeclareAbstractInputPort("lcmt_contact_info",
                                 Value<lcmt_contact_info>{});

  this->DeclareAbstractOutputPort("contact_info",
                                  &ContactLocationEstimator::CopyStateOut);

  this->DeclarePeriodicUnrestrictedUpdateEvent(
      update_period_, 0, &ContactLocationEstimator::UpdateContactInfo);

  this->DeclareInitializationUnrestrictedUpdateEvent(
      &ContactLocationEstimator::Initialize);
}

const systems::InputPort<double>& ContactLocationEstimator::get_input_port()
    const {
  return LeafSystem<double>::get_input_port(abstract_state_index_);
}

systems::EventStatus ContactLocationEstimator::Initialize(
    const systems::Context<double>&, systems::State<double>* state) const {
  auto& contact_info =
      state->get_mutable_abstract_state<ContactInfo>(abstract_state_index_);
  contact_info.num_contacts = 0;
  // fill stl vectors with 1 element.
  contact_info.contact_link_idx.push_back(-1);
  contact_info.positions.emplace_back(0, 0, 0);

  return systems::EventStatus::Succeeded();
};

void ContactLocationEstimator::CopyStateOut(
    const systems::Context<double>& context, ContactInfo* contact_info) const {
  *contact_info = context.get_abstract_state<ContactInfo>(0);
}

void ContactLocationEstimator::UpdateContactInfo(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  const auto& contact_info_msg =
      get_input_port().Eval<lcmt_contact_info>(context);
  auto& contact_info =
      state->get_mutable_abstract_state<ContactInfo>(abstract_state_index_);


  const auto nc = static_cast<unsigned long>(contact_info_msg.num_contacts);

//  if (nc != contact_info_msg.link_indices.size()) {
//    cout << context.get_time() << " " <<
//    nc << " " << contact_info_msg.position.size() << " " <<
//         contact_info_msg.link_indices.size() << endl;
//  }
//
  // Sanity check.
  DRAKE_THROW_UNLESS(nc == contact_info_msg.position.size());
//  DRAKE_THROW_UNLESS(nc == contact_info_msg.link_indices.size());

  // Update the system's abstract state.
  contact_info.num_contacts = nc;

  if (nc == 0) {
    contact_position_filter_->reset_state();
    contact_info.contact_link_idx[0] = -1;
    return;
  }

  // Only handles up to one contact at the moment.
  DRAKE_THROW_UNLESS(nc == 1);
  if (contact_info.contact_link_idx[0] != contact_info_msg.link_indices[0]) {
    // If contact link changes, reset contact position.
    contact_position_filter_->reset_state();
    contact_info.contact_link_idx[0] = contact_info_msg.link_indices[0];
  }
  contact_position_filter_->Update(contact_info_msg.position[0]);
  contact_info.positions[0] = contact_position_filter_->get_current_x();
}

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
