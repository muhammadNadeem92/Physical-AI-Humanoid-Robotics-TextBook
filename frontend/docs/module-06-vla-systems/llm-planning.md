# Chapter 3: Cognitive Planning with LLMs

## Introduction

In the previous chapter, you learned how to convert voice and text commands into structured JSON task schemas. Now we'll explore cognitive planning with Large Language Models (LLMs), focusing on using LLMs as task planners rather than direct motor controllers.

The key insight in VLA systems is that LLMs serve as high-level planners that decompose complex commands into detailed execution sequences, while leaving low-level motor control to specialized robotic systems. This separation ensures safety, determinism, and proper constraint enforcement while leveraging the powerful reasoning capabilities of LLMs for task planning.

This chapter covers LLMs as planners versus executors, task decomposition techniques, state awareness in planning systems, and tool calling with function schemas. You'll learn how to convert high-level commands to step-by-step plans, enforce constraints like reachability and safety, and validate plans before execution.

By the end of this chapter, you'll understand how to leverage LLMs for intelligent task planning while maintaining the critical safety boundaries that separate planning from execution in VLA systems.

## Core Concepts

### LLMs as Planners vs Executors

Large Language Models serve as planners in VLA systems, not as direct executors of low-level motor commands:

**Planning Responsibilities**: LLMs handle high-level task decomposition, reasoning about object relationships, spatial reasoning, and generating structured action sequences that can be executed by robotic systems.

**Execution Boundaries**: LLMs do not directly control motors, joints, or low-level robotic functions. Instead, they generate plans that are executed by specialized robotic control systems.

**Abstraction Layer**: LLMs operate at a higher level of abstraction, focusing on "what" needs to be done rather than "how" to move individual joints or control low-level systems.

**Safety Mediation**: The planning layer acts as a safety buffer, validating high-level goals and constraints before they reach execution systems.

### Task Decomposition

Task decomposition breaks complex commands into sequences of simpler, executable actions:

**Hierarchical Decomposition**: Breaking complex tasks into subtasks that can be further decomposed until reaching primitive actions that can be executed by the robotic system.

**Dependency Analysis**: Identifying dependencies between subtasks to determine the correct execution order and parallelization opportunities.

**Prerequisite Identification**: Determining what conditions must be met before each subtask can be executed successfully.

**Fallback Planning**: Creating alternative sequences for handling common failure modes during execution.

### State Awareness

State awareness enables planning systems to account for the current situation and environment:

**World State Representation**: Maintaining an up-to-date representation of the environment, objects, and their relationships to inform planning decisions.

**Robot State Integration**: Incorporating information about the robot's current configuration, capabilities, and limitations into planning decisions.

**Temporal State Tracking**: Understanding how the world state changes as tasks are executed and planning accordingly.

**Uncertainty Handling**: Managing uncertainty in state information and planning robustly despite incomplete or potentially incorrect state knowledge.

### Tool Calling & Function Schemas

Tool calling enables LLMs to interact with external systems and services during planning:

**Function Schema Definition**: Creating structured schemas that define the inputs, outputs, and constraints for external tools and services that the LLM can call.

**API Integration**: Enabling LLMs to call external APIs for perception, navigation, manipulation, and other robotic functions.

**Constraint Checking**: Using tool calls to validate constraints like reachability, collision detection, and safety requirements.

**Feedback Integration**: Incorporating results from tool calls back into the planning process to refine or adjust the plan.

## Examples

### Example: Converting Command to Step-by-Step Plan

Converting a high-level command into a detailed step-by-step execution plan:

```json
{
  "plan_id": "plan_001",
  "original_command": "Pick up the red bottle from the table",
  "plan": [
    {
      "step_id": "step_001",
      "action": "approach_location",
      "parameters": {
        "location": "near_table",
        "safety_margin": 0.5
      },
      "preconditions": [
        "robot_is_idle",
        "location_is_reachable"
      ],
      "postconditions": [
        "robot_is_near_table"
      ],
      "timeout": 30
    },
    {
      "step_id": "step_002",
      "action": "perceive_environment",
      "parameters": {
        "target_object": "red bottle",
        "search_area": "table_surface"
      },
      "preconditions": [
        "robot_is_near_table"
      ],
      "postconditions": [
        "object_location_confirmed",
        "grasp_feasibility_verified"
      ],
      "timeout": 10
    },
    {
      "step_id": "step_003",
      "action": "plan_grasp",
      "parameters": {
        "object": "red bottle",
        "grasp_type": "top_grasp",
        "gripper": "left"
      },
      "preconditions": [
        "object_location_confirmed",
        "grasp_feasibility_verified"
      ],
      "postconditions": [
        "grasp_plan_generated",
        "collision_free_path_confirmed"
      ],
      "timeout": 15
    },
    {
      "step_id": "step_004",
      "action": "execute_grasp",
      "parameters": {
        "grasp_plan": "grasp_plan_generated"
      },
      "preconditions": [
        "grasp_plan_generated",
        "collision_free_path_confirmed"
      ],
      "postconditions": [
        "object_grasped",
        "gripper_state_confirmed"
      ],
      "timeout": 20
    },
    {
      "step_id": "step_005",
      "action": "lift_object",
      "parameters": {
        "height": 0.1,
        "speed": "normal"
      },
      "preconditions": [
        "object_grasped"
      ],
      "postconditions": [
        "object_lifted",
        "stable_grasp_confirmed"
      ],
      "timeout": 10
    }
  ],
  "constraints": {
    "safety_constraints": [
      "avoid_collisions",
      "respect_workspace_limits",
      "maintain_stability"
    ],
    "execution_constraints": [
      "max_execution_time: 120",
      "precision_requirements: standard"
    ]
  },
  "fallback_procedures": [
    {
      "trigger": "object_not_found",
      "procedure": "request_user_clarification"
    },
    {
      "trigger": "grasp_failure",
      "procedure": "attempt_alternative_grasp"
    },
    {
      "trigger": "collision_detected",
      "procedure": "abort_and_replan"
    }
  ]
}
```

This detailed plan breaks down the high-level command into executable steps with preconditions, postconditions, and safety constraints.

### Example: Enforcing Constraints (Reachability, Safety)

Implementing constraint enforcement in planning systems:

```python
# Example Python code for constraint enforcement
class ConstraintEnforcer:
    def __init__(self):
        self.kinematics_solver = KinematicsSolver()
        self.collision_detector = CollisionDetector()
        self.safety_checker = SafetyChecker()

    def enforce_reachability_constraint(self, plan_step):
        """Enforce reachability constraints for manipulation tasks"""
        if plan_step['action'] == 'execute_grasp':
            object_pose = plan_step['parameters']['object_pose']
            robot_base_pose = plan_step['parameters']['robot_pose']

            # Check if the object is within the robot's reachable workspace
            is_reachable = self.kinematics_solver.check_reachability(
                object_pose,
                robot_base_pose
            )

            if not is_reachable:
                return {
                    'valid': False,
                    'error': 'Target object is not reachable from current position',
                    'suggestion': 'Approach the object first or reposition the robot'
                }

            # Check if the grasp approach direction is feasible
            grasp_approach = plan_step['parameters']['grasp_approach']
            is_approachable = self.kinematics_solver.check_approach_feasibility(
                object_pose,
                grasp_approach
            )

            if not is_approachable:
                return {
                    'valid': False,
                    'error': 'Grasp approach direction is not kinematically feasible',
                    'suggestion': 'Try a different grasp approach or reposition'
                }

        return {'valid': True}

    def enforce_safety_constraints(self, plan_step):
        """Enforce safety constraints for all plan steps"""
        # Check for potential collisions during the action
        if 'path' in plan_step['parameters']:
            path = plan_step['parameters']['path']
            has_collision = self.collision_detector.check_path_collision(path)

            if has_collision:
                return {
                    'valid': False,
                    'error': 'Path contains potential collision risks',
                    'suggestion': 'Replan path to avoid obstacles'
                }

        # Check safety zones and restricted areas
        if 'location' in plan_step['parameters']:
            location = plan_step['parameters']['location']
            is_safe = self.safety_checker.check_location_safety(location)

            if not is_safe:
                return {
                    'valid': False,
                    'error': 'Target location is in a restricted or unsafe area',
                    'suggestion': 'Choose a different location or verify safety permissions'
                }

        # Check for dynamic safety constraints (e.g., humans in area)
        dynamic_safety = self.safety_checker.check_dynamic_safety()
        if not dynamic_safety['safe']:
            return {
                'valid': False,
                'error': f'Dynamic safety issue: {dynamic_safety["issue"]}',
                'suggestion': dynamic_safety['suggestion']
            }

        return {'valid': True}

    def validate_plan_step(self, plan_step, current_state):
        """Validate a plan step against all relevant constraints"""
        # Check reachability constraints
        reachability_result = self.enforce_reachability_constraint(plan_step)
        if not reachability_result['valid']:
            return reachability_result

        # Check safety constraints
        safety_result = self.enforce_safety_constraints(plan_step)
        if not safety_result['valid']:
            return safety_result

        # Check other domain-specific constraints
        domain_result = self.check_domain_constraints(plan_step, current_state)
        if not domain_result['valid']:
            return domain_result

        return {
            'valid': True,
            'validated_step': plan_step
        }

    def check_domain_constraints(self, plan_step, current_state):
        """Check domain-specific constraints"""
        # Example: Check if the robot has the required tools/attachments
        if plan_step['action'] == 'use_tool':
            required_tool = plan_step['parameters']['required_tool']
            if not current_state['available_tools'].get(required_tool):
                return {
                    'valid': False,
                    'error': f'Required tool {required_tool} is not available',
                    'suggestion': 'Verify tool attachment or select different approach'
                }

        # Example: Check payload constraints
        if plan_step['action'] == 'lift_object':
            object_weight = plan_step['parameters']['object_weight']
            max_payload = current_state['robot_specs']['max_payload']

            if object_weight > max_payload:
                return {
                    'valid': False,
                    'error': f'Object weight ({object_weight}kg) exceeds maximum payload ({max_payload}kg)',
                    'suggestion': 'Select a lighter object or use specialized equipment'
                }

        return {'valid': True}

# Example usage of constraint enforcement
class PlanningSystem:
    def __init__(self):
        self.constraint_enforcer = ConstraintEnforcer()
        self.llm_planner = LLMPlanner()

    def generate_and_validate_plan(self, command, current_state):
        """Generate plan and validate all constraints"""
        # Generate initial plan using LLM
        raw_plan = self.llm_planner.generate_plan(command)

        # Validate each step in the plan
        validated_plan = []
        for step in raw_plan['plan']:
            validation_result = self.constraint_enforcer.validate_plan_step(step, current_state)

            if not validation_result['valid']:
                return {
                    'status': 'validation_failed',
                    'error_step': step['step_id'],
                    'error': validation_result['error'],
                    'suggestion': validation_result['suggestion']
                }

            validated_plan.append(validation_result['validated_step'])

        # Update the plan with validated steps
        raw_plan['plan'] = validated_plan
        raw_plan['status'] = 'validated'

        return raw_plan
```

### Example: Plan Validation Before Execution

Implementing comprehensive plan validation before execution:

```python
# Complete plan validation system
import asyncio
from typing import Dict, List, Any
from enum import Enum

class ValidationStatus(Enum):
    PASSED = "passed"
    FAILED = "failed"
    PARTIAL = "partial"

class PlanValidator:
    def __init__(self):
        self.reachability_checker = ReachabilityChecker()
        self.safety_analyzer = SafetyAnalyzer()
        self.resource_validator = ResourceValidator()
        self.temporal_validator = TemporalValidator()

    async def validate_plan(self, plan: Dict[str, Any], current_state: Dict[str, Any]) -> Dict[str, Any]:
        """Comprehensive plan validation before execution"""
        validation_results = {
            'overall_status': ValidationStatus.PASSED,
            'validation_steps': [],
            'warnings': [],
            'errors': [],
            'validated_plan': plan.copy()
        }

        # Validate each step in the plan
        for step in plan['plan']:
            step_result = await self._validate_single_step(step, current_state)

            if step_result['status'] == ValidationStatus.FAILED:
                validation_results['overall_status'] = ValidationStatus.FAILED
                validation_results['errors'].extend(step_result['errors'])
            elif step_result['status'] == ValidationStatus.PARTIAL:
                validation_results['warnings'].extend(step_result['warnings'])

            validation_results['validation_steps'].append(step_result)

        # Perform cross-step validation
        cross_validation = await self._validate_cross_step_dependencies(plan['plan'])
        if cross_validation['errors']:
            validation_results['overall_status'] = ValidationStatus.FAILED
            validation_results['errors'].extend(cross_validation['errors'])

        if cross_validation['warnings']:
            validation_results['warnings'].extend(cross_validation['warnings'])

        # Final temporal and resource validation
        temporal_result = await self.temporal_validator.validate_temporal_constraints(plan)
        if not temporal_result['valid']:
            validation_results['overall_status'] = ValidationStatus.FAILED
            validation_results['errors'].append(temporal_result['error'])

        return validation_results

    async def _validate_single_step(self, step: Dict[str, Any], current_state: Dict[str, Any]) -> Dict[str, Any]:
        """Validate a single plan step"""
        step_validation = {
            'step_id': step['step_id'],
            'status': ValidationStatus.PASSED,
            'errors': [],
            'warnings': [],
            'validated_parameters': step['parameters'].copy()
        }

        # Validate preconditions
        preconditions_result = await self._validate_preconditions(step, current_state)
        if not preconditions_result['satisfied']:
            step_validation['status'] = ValidationStatus.FAILED
            step_validation['errors'].extend(preconditions_result['errors'])

        # Validate action-specific constraints
        action_result = await self._validate_action_constraints(step)
        if not action_result['valid']:
            step_validation['status'] = ValidationStatus.FAILED
            step_validation['errors'].append(action_result['error'])

        # Validate safety constraints
        safety_result = await self.safety_analyzer.check_step_safety(step, current_state)
        if not safety_result['safe']:
            if safety_result['severity'] == 'critical':
                step_validation['status'] = ValidationStatus.FAILED
                step_validation['errors'].append(safety_result['message'])
            else:
                step_validation['warnings'].append(safety_result['message'])

        return step_validation

    async def _validate_preconditions(self, step: Dict[str, Any], current_state: Dict[str, Any]) -> Dict[str, Any]:
        """Validate preconditions for a step"""
        required_conditions = step.get('preconditions', [])
        errors = []

        for condition in required_conditions:
            if condition == 'robot_is_idle':
                if not current_state['robot_status'] == 'idle':
                    errors.append('Robot must be idle before executing this step')

            elif condition == 'location_is_reachable':
                target_location = step['parameters'].get('location')
                if target_location:
                    is_reachable = await self.reachability_checker.check_location(target_location)
                    if not is_reachable:
                        errors.append(f'Location {target_location} is not reachable')

        return {
            'satisfied': len(errors) == 0,
            'errors': errors
        }

    async def _validate_action_constraints(self, step: Dict[str, Any]) -> Dict[str, Any]:
        """Validate action-specific constraints"""
        action = step['action']

        if action == 'execute_grasp':
            # Validate grasp-specific constraints
            grasp_type = step['parameters'].get('grasp_type')
            object_properties = step['parameters'].get('object_properties', {})

            if grasp_type == 'top_grasp' and object_properties.get('width', 0) > 0.1:
                return {
                    'valid': False,
                    'error': 'Top grasp not suitable for objects wider than 10cm'
                }

        elif action == 'approach_location':
            # Validate approach-specific constraints
            safety_margin = step['parameters'].get('safety_margin', 0.0)
            if safety_margin < 0.1:
                return {
                    'valid': False,
                    'error': 'Safety margin must be at least 0.1m for approach actions'
                }

        return {'valid': True}

    async def _validate_cross_step_dependencies(self, plan_steps: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Validate dependencies between plan steps"""
        errors = []
        warnings = []

        for i, step in enumerate(plan_steps):
            # Check if postconditions of previous steps satisfy preconditions of current step
            if i > 0:
                prev_step = plan_steps[i-1]
                current_preconditions = step.get('preconditions', [])
                prev_postconditions = prev_step.get('postconditions', [])

                for precondition in current_preconditions:
                    if precondition not in prev_postconditions:
                        warnings.append(
                            f'Step {step["step_id"]} precondition "{precondition}" may not be satisfied by previous steps'
                        )

        return {
            'errors': errors,
            'warnings': warnings
        }

# Example of using the plan validator
class VLAPlanningSystem:
    def __init__(self):
        self.validator = PlanValidator()
        self.llm_planner = AdvancedLLMPlanner()

    async def plan_and_validate(self, natural_command: str, current_state: Dict[str, Any]) -> Dict[str, Any]:
        """Complete planning and validation workflow"""
        # Generate plan using LLM
        raw_plan = await self.llm_planner.generate_from_command(natural_command)

        # Validate the complete plan
        validation_result = await self.validator.validate_plan(raw_plan, current_state)

        if validation_result['overall_status'] == ValidationStatus.FAILED:
            return {
                'status': 'planning_failed',
                'reason': 'Plan validation failed',
                'validation_result': validation_result
            }

        # Plan is valid, ready for execution
        return {
            'status': 'planning_successful',
            'plan': validation_result['validated_plan'],
            'validation_result': validation_result,
            'warnings': validation_result['warnings']
        }
```

## Summary & Key Takeaways

In this chapter, you've learned about cognitive planning with LLMs in VLA systems:

- **LLMs serve as planners, not executors**, operating at a high level of abstraction while leaving low-level control to specialized systems
- **Task decomposition** breaks complex commands into sequences of simpler, executable actions with proper dependencies
- **State awareness** enables planning systems to account for current conditions and environment
- **Tool calling and function schemas** allow LLMs to interact with external systems for constraint checking and validation

You've seen practical examples of converting high-level commands to detailed step-by-step plans, implementing constraint enforcement for reachability and safety, and creating comprehensive validation systems before plan execution. The emphasis throughout is on maintaining the critical separation between planning and execution, ensuring that LLMs plan safely while specialized systems handle the actual physical execution.

These planning capabilities form the intelligence layer of VLA systems, bridging the gap between natural language commands and executable robot actions while maintaining the safety and determinism required for physical robot operation. In the next chapter, you'll explore how these plans are safely executed through ROS 2 actions with proper monitoring and failure recovery.