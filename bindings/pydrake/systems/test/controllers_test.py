from __future__ import print_function

import math
import unittest

import numpy as np

from pydrake.common import FindResourceOrThrow
from pydrake.examples.pendulum import PendulumPlant
from pydrake.multibody.tree import MultibodyForces
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import (
    DiscreteTimeLinearQuadraticRegulator, DynamicProgrammingOptions,
    FittedValueIteration,
    InverseDynamicsController, InverseDynamics,
    LinearQuadraticRegulator,
    LinearProgrammingApproximateDynamicProgramming,
    PeriodicBoundaryCondition
)
from pydrake.systems.framework import BasicVector
from pydrake.systems.primitives import Integrator, LinearSystem


class TestControllers(unittest.TestCase):
    def test_fitted_value_iteration_pendulum(self):
        plant = PendulumPlant()
        simulator = Simulator(plant)

        def quadratic_regulator_cost(context):
            x = context.get_continuous_state_vector().CopyToVector()
            x[0] = x[0] - math.pi
            u = plant.EvalVectorInput(context, 0).CopyToVector()
            return x.dot(x) + u.dot(u)

        # Note: intentionally under-sampled to keep the problem small
        qbins = np.linspace(0., 2.*math.pi, 11)
        qdotbins = np.linspace(-10., 10., 11)
        state_grid = [set(qbins), set(qdotbins)]

        input_limit = 2.
        input_mesh = [set(np.linspace(-input_limit, input_limit, 5))]
        timestep = 0.01

        num_callbacks = [0]

        def callback(iteration, mesh, cost_to_go, policy):
            # Drawing is slow, don't draw every frame.
            num_callbacks[0] += 1

        options = DynamicProgrammingOptions()
        options.convergence_tol = 1.
        options.periodic_boundary_conditions = [
            PeriodicBoundaryCondition(0, 0., 2.*math.pi)
        ]
        options.visualization_callback = callback

        policy, cost_to_go = FittedValueIteration(simulator,
                                                  quadratic_regulator_cost,
                                                  state_grid, input_mesh,
                                                  timestep, options)

        self.assertGreater(num_callbacks[0], 0)

    def test_linear_programming_approximate_dynamic_programming(self):
        integrator = Integrator(1)
        simulator = Simulator(integrator)

        # minimum time cost function (1 for all non-zero states).
        def cost_function(context):
            x = context.get_continuous_state_vector().CopyToVector()
            if (math.fabs(x[0]) > 0.1):
                return 1.
            else:
                return 0.

        def cost_to_go_function(state, parameters):
            return parameters[0] * math.fabs(state[0])

        state_samples = np.array([[-4., -3., -2., -1., 0., 1., 2., 3., 4.]])
        input_samples = np.array([[-1., 0., 1.]])

        timestep = 1.0
        options = DynamicProgrammingOptions()
        options.discount_factor = 1.

        J = LinearProgrammingApproximateDynamicProgramming(
            simulator, cost_function, cost_to_go_function, 1,
            state_samples, input_samples, timestep, options)

        self.assertAlmostEqual(J[0], 1., delta=1e-6)

    def test_inverse_dynamics(self):
        sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/" +
            "iiwa_description/sdf/iiwa14_no_collision.sdf")

        plant = MultibodyPlant(time_step=0.01)
        Parser(plant).AddModelFromFile(sdf_path)
        plant.WeldFrames(plant.world_frame(),
                         plant.GetFrameByName("iiwa_link_0"))
        plant.Finalize()

        # Just test that the constructor doesn't throw.
        controller = InverseDynamics(
            plant=plant,
            mode=InverseDynamics.InverseDynamicsMode.kGravityCompensation)

    def test_inverse_dynamics_controller(self):
        sdf_path = FindResourceOrThrow(
            "drake/manipulation/models/" +
            "iiwa_description/sdf/iiwa14_no_collision.sdf")

        plant = MultibodyPlant(time_step=0.01)
        Parser(plant).AddModelFromFile(sdf_path)
        plant.WeldFrames(plant.world_frame(),
                         plant.GetFrameByName("iiwa_link_0"))
        plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])
        plant.Finalize()

        # We verify the (known) size of the model.
        kNumPositions = 7
        kNumVelocities = 7
        kNumActuators = 7
        kStateSize = kNumPositions + kNumVelocities
        self.assertEqual(plant.num_positions(), kNumPositions)
        self.assertEqual(plant.num_velocities(), kNumVelocities)
        self.assertEqual(plant.num_actuators(), kNumActuators)

        kp = np.array([1., 2., 3., 4., 5., 6., 7.])
        ki = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        kd = np.array([.5, 1., 1.5, 2., 2.5, 3., 3.5])

        controller = InverseDynamicsController(robot=plant,
                                               kp=kp,
                                               ki=ki,
                                               kd=kd,
                                               has_reference_acceleration=True)
        context = controller.CreateDefaultContext()
        output = controller.AllocateOutput()

        estimated_state_port = 0
        desired_state_port = 1
        desired_acceleration_port = 2
        control_port = 0

        self.assertEqual(
            controller.get_input_port(desired_acceleration_port).size(),
            kNumVelocities)
        self.assertEqual(
            controller.get_input_port(estimated_state_port).size(), kStateSize)
        self.assertEqual(
            controller.get_input_port(desired_state_port).size(), kStateSize)
        self.assertEqual(
            controller.get_output_port(control_port).size(), kNumVelocities)

        # Current state.
        q = np.array([-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3])
        v = np.array([-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9])
        x = np.concatenate([q, v])

        # Reference state and acceleration.
        q_r = q + 0.1*np.ones_like(q)
        v_r = v + 0.1*np.ones_like(v)
        x_r = np.concatenate([q_r, v_r])
        vd_r = np.array([1., 2., 3., 4., 5., 6., 7.])

        integral_term = np.array([-1., -2., -3., -4., -5., -6., -7.])

        vd_d = vd_r + kp*(q_r-q) + kd*(v_r-v) + ki*integral_term

        context.FixInputPort(estimated_state_port, BasicVector(x))
        context.FixInputPort(desired_state_port, BasicVector(x_r))
        context.FixInputPort(desired_acceleration_port, BasicVector(vd_r))
        controller.set_integral_value(context, integral_term)

        # Set the plant's context.
        plant_context = plant.CreateDefaultContext()
        x_plant = plant.GetMutablePositionsAndVelocities(plant_context)
        x_plant[:] = x

        # Compute the expected value of the generalized forces using
        # inverse dynamics.
        tau_id = plant.CalcInverseDynamics(
            plant_context, vd_d, MultibodyForces(plant))

        # Verify the result.
        controller.CalcOutput(context, output)
        self.assertTrue(np.allclose(output.get_vector_data(0).CopyToVector(),
                                    tau_id))

    def test_linear_quadratic_regulator(self):
        A = np.array([[0, 1], [0, 0]])
        B = np.array([[0], [1]])
        C = np.identity(2)
        D = np.array([[0], [0]])
        double_integrator = LinearSystem(A, B, C, D)

        Q = np.identity(2)
        R = np.identity(1)
        K_expected = np.array([[1, math.sqrt(3.)]])
        S_expected = np.array([[math.sqrt(3), 1.], [1., math.sqrt(3)]])

        (K, S) = LinearQuadraticRegulator(A, B, Q, R)
        np.testing.assert_almost_equal(K, K_expected)
        np.testing.assert_almost_equal(S, S_expected)

        controller = LinearQuadraticRegulator(double_integrator, Q, R)
        np.testing.assert_almost_equal(controller.D(), -K_expected)

        context = double_integrator.CreateDefaultContext()
        context.FixInputPort(0, BasicVector([0]))
        controller = LinearQuadraticRegulator(double_integrator, context, Q, R)
        np.testing.assert_almost_equal(controller.D(), -K_expected)

    def test_discrete_time_linear_quadratic_regulator(self):
        A = np.array([[1, 1], [0, 1]])
        B = np.array([[0], [1]])
        Q = np.identity(2)
        R = np.identity(1)
        (K, S) = DiscreteTimeLinearQuadraticRegulator(A, B, Q, R)
        self.assertEqual(K.shape, (1, 2))
        self.assertEqual(S.shape, (2, 2))
