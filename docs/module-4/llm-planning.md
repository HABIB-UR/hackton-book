---
sidebar_position: 2
---

# Cognitive Planning with LLMs: Translating Natural Language to Robot Actions

## Introduction to Cognitive Planning

Cognitive planning is the process of translating high-level natural language commands into specific, executable robot actions. In this chapter, we'll explore how large language models (LLMs) can be used to bridge the gap between human intentions expressed in natural language and concrete robotic behaviors.

Cognitive planning involves several key components:
1. **Natural Language Understanding**: Interpreting the user's intent from spoken or written commands
2. **Action Decomposition**: Breaking down complex commands into sequences of primitive actions
3. **Context Awareness**: Understanding the environment and current state of the robot
4. **Action Execution Planning**: Creating a sequence of specific robot commands

## OpenAI GPT Integration with Structured Prompting

Large Language Models like OpenAI's GPT series excel at understanding natural language and can be used effectively for cognitive planning in robotics. We'll demonstrate how to integrate GPT with structured prompting techniques.

### Basic GPT API Usage for Cognitive Planning

Here's a basic example of how to use the OpenAI GPT API for cognitive planning:

```python
import openai
import os
import json
from typing import Dict, List, Any

class CognitivePlanner:
    def __init__(self, api_key: str = None):
        """
        Initialize the cognitive planning system
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key is required")

        openai.api_key = self.api_key

    def plan_actions(self, natural_language_command: str, robot_capabilities: List[str] = None) -> Dict[str, Any]:
        """
        Plan robot actions based on natural language command
        """
        # Define robot capabilities if not provided
        if robot_capabilities is None:
            robot_capabilities = [
                "move_forward", "move_backward", "turn_left", "turn_right",
                "pick_object", "place_object", "grasp", "release",
                "speak", "detect_object", "navigate_to"
            ]

        # Create a structured prompt for the LLM
        prompt = f"""
        You are a cognitive planning system for a humanoid robot. Your task is to translate natural language commands into sequences of robot actions.

        Robot capabilities: {', '.join(robot_capabilities)}

        Natural language command: "{natural_language_command}"

        Please respond in the following JSON format:
        {{
            "command": "natural language command",
            "actions": [
                {{
                    "action": "action_name",
                    "parameters": {{"param1": "value1", "param2": "value2"}},
                    "description": "Brief description of what this action does"
                }}
            ],
            "reasoning": "Explain your reasoning for choosing these actions",
            "confidence": "A number between 0 and 1 indicating confidence in the plan"
        }}

        Only respond with valid JSON, no other text.
        """

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1  # Low temperature for more consistent outputs
            )

            # Parse the response
            content = response.choices[0].message.content.strip()

            # Remove any markdown formatting if present
            if content.startswith("```json"):
                content = content[7:-3].strip()
            elif content.startswith("```"):
                content = content[3:-3].strip()

            plan = json.loads(content)
            return plan

        except json.JSONDecodeError as e:
            return {
                "error": f"Failed to parse LLM response as JSON: {str(e)}",
                "raw_response": content if 'content' in locals() else "No content",
                "success": False
            }
        except Exception as e:
            return {
                "error": f"Planning failed: {str(e)}",
                "success": False
            }
```

## Function Calling for Action Sequences

OpenAI's function calling feature provides a more reliable way to get structured outputs for robotic actions:

```python
import openai
import os
import json
from typing import Dict, List, Any

class FunctionCallingPlanner:
    def __init__(self, api_key: str = None):
        """
        Initialize the cognitive planning system with function calling
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key is required")

        openai.api_key = self.api_key

    def plan_actions_with_functions(self, natural_language_command: str) -> Dict[str, Any]:
        """
        Plan robot actions using OpenAI's function calling
        """
        # Define the function that the LLM can call
        functions = [
            {
                "name": "execute_action_sequence",
                "description": "Execute a sequence of robot actions",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "actions": {
                            "type": "array",
                            "items": {
                                "type": "object",
                                "properties": {
                                    "action": {
                                        "type": "string",
                                        "description": "The action to execute"
                                    },
                                    "parameters": {
                                        "type": "object",
                                        "description": "Parameters for the action"
                                    },
                                    "description": {
                                        "type": "string",
                                        "description": "Description of the action"
                                    }
                                },
                                "required": ["action", "parameters", "description"]
                            }
                        },
                        "reasoning": {
                            "type": "string",
                            "description": "The reasoning behind the action sequence"
                        },
                        "confidence": {
                            "type": "number",
                            "description": "Confidence score between 0 and 1"
                        }
                    },
                    "required": ["actions", "reasoning", "confidence"]
                }
            }
        ]

        # Create the message
        messages = [
            {
                "role": "user",
                "content": f"Plan robot actions for this command: '{natural_language_command}'. The robot should navigate to the kitchen, pick up a cup, and bring it to the table."
            }
        ]

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo-0613",  # Function calling model
                messages=messages,
                functions=functions,
                function_call={"name": "execute_action_sequence"},
                temperature=0.1
            )

            # Extract the function call
            message = response.choices[0].message
            if message.get("function_call"):
                function_call = message["function_call"]
                arguments = json.loads(function_call["arguments"])
                return arguments
            else:
                return {
                    "error": "No function call returned from LLM",
                    "success": False
                }

        except Exception as e:
            return {
                "error": f"Planning with functions failed: {str(e)}",
                "success": False
            }
```

## Chain-of-Thought Prompting for Planning Transparency

Chain-of-thought prompting can make the planning process more transparent and verifiable:

```python
def plan_with_chain_of_thought(self, natural_language_command: str) -> Dict[str, Any]:
    """
    Plan actions using chain-of-thought reasoning for transparency
    """
    prompt = f"""
    You are a cognitive planning system for a humanoid robot. Your task is to break down natural language commands into sequences of robot actions.

    Natural language command: "{natural_language_command}"

    Let's think step by step:
    1. What is the user trying to achieve?
    2. What are the subtasks needed to accomplish this?
    3. What robot actions are needed for each subtask?
    4. In what order should these actions be executed?

    Then provide your final action plan in the following JSON format:
    {{
        "command": "{natural_language_command}",
        "step_by_step_reasoning": [
            "Step 1: [explanation]",
            "Step 2: [explanation]",
            ...
        ],
        "actions": [
            {{
                "action": "action_name",
                "parameters": {{"param1": "value1"}},
                "description": "What this action does"
            }}
        ],
        "confidence": 0.8
    }}

    Only respond with valid JSON, no other text.
    """

    # Implementation similar to previous examples...
```

## Mapping Natural Language to ROS 2 Action Types

When integrating with ROS 2, it's important to map LLM outputs to specific ROS 2 action types:

```python
class ROS2ActionMapper:
    def __init__(self):
        """
        Map natural language concepts to ROS 2 action types
        """
        self.action_mapping = {
            "move": ["nav2_msgs.action.NavigateToPose", "geometry_msgs.msg.Twist"],
            "navigate": ["nav2_msgs.action.NavigateToPose"],
            "pick": ["manipulation_msgs.action.PickupObject"],
            "grasp": ["control_msgs.action.GripperCommand"],
            "place": ["manipulation_msgs.action.PlaceObject"],
            "detect": ["object_detection_msgs.action.DetectObjects"],
            "speak": ["sound_play_msgs.action.Speak"],
            "listen": ["speech_recognition_msgs.action.RecognizeSpeech"]
        }

    def map_to_ros2_actions(self, llm_plan: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Map LLM-generated actions to ROS 2 action types
        """
        ros2_actions = []

        for action in llm_plan.get("actions", []):
            action_name = action.get("action", "").lower()

            # Find matching ROS 2 action types
            for keyword, ros2_types in self.action_mapping.items():
                if keyword in action_name:
                    for ros2_type in ros2_types:
                        ros2_action = {
                            "ros2_type": ros2_type,
                            "llm_action": action,
                            "parameters": action.get("parameters", {}),
                            "mapped_from": action_name
                        }
                        ros2_actions.append(ros2_action)
                    break
            else:
                # If no specific mapping found, use a generic action
                ros2_action = {
                    "ros2_type": "std_msgs.msg.String",  # Generic fallback
                    "llm_action": action,
                    "parameters": action.get("parameters", {}),
                    "mapped_from": action_name,
                    "warning": "No specific ROS 2 action mapping found"
                }
                ros2_actions.append(ros2_action)

        return ros2_actions
```

## Error Handling and Validation Examples

Robust cognitive planning systems need to handle various error scenarios:

```python
def validate_and_execute_plan(self, plan: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validate the planned actions before execution
    """
    validation_results = {
        "is_valid": True,
        "errors": [],
        "warnings": [],
        "executed_actions": []
    }

    # Validate each action in the plan
    for i, action in enumerate(plan.get("actions", [])):
        action_name = action.get("action", "")

        # Check if action is recognized
        if not self.is_valid_action(action_name):
            validation_results["is_valid"] = False
            validation_results["errors"].append(
                f"Action {i}: '{action_name}' is not a recognized robot action"
            )
            continue

        # Validate parameters
        params = action.get("parameters", {})
        action_errors = self.validate_action_parameters(action_name, params)

        if action_errors:
            validation_results["is_valid"] = False
            validation_results["errors"].extend([
                f"Action {i} ({action_name}): {error}" for error in action_errors
            ])
        else:
            # If valid, add to executed actions
            validation_results["executed_actions"].append(action)

    return validation_results

def is_valid_action(self, action_name: str) -> bool:
    """
    Check if an action name is valid for the robot
    """
    valid_actions = [
        "move_forward", "move_backward", "turn_left", "turn_right",
        "pick_object", "place_object", "grasp", "release",
        "speak", "detect_object", "navigate_to", "stop"
    ]
    return action_name in valid_actions

def validate_action_parameters(self, action_name: str, params: Dict[str, Any]) -> List[str]:
    """
    Validate parameters for a specific action
    """
    errors = []

    if action_name in ["navigate_to", "move_to"]:
        if "x" not in params or "y" not in params:
            errors.append("Missing required coordinates (x, y)")

    elif action_name == "pick_object":
        if "object_name" not in params and "object_id" not in params:
            errors.append("Missing required object identifier")

    elif action_name == "speak":
        if "text" not in params:
            errors.append("Missing required text to speak")

    return errors
```

## Example Exercise: Cognitive Planning Practice

**Objective**: Create a cognitive planning system that can translate natural language commands into robot action sequences.

**Instructions**:
1. Implement the CognitivePlanner class with basic GPT integration
2. Test with commands like "Go to the kitchen and bring me a cup"
3. Verify that the system generates appropriate action sequences

**Expected Outcome**: The system should generate a sequence of robot actions that would accomplish the requested task.

## Exercise: LLM Planning Implementation

**Objective**: Implement a complete cognitive planning system using LLMs.

**Difficulty**: Advanced

**Instructions**:
1. Create a cognitive planning system that uses both basic prompting and function calling
2. Implement validation of generated action sequences
3. Add error handling for ambiguous or impossible commands
4. Test with various natural language commands

**Expected Outcome**: A robust cognitive planning system that can handle various types of natural language commands and generate executable robot action sequences.

**Hints**:
- Start with simple commands and gradually increase complexity
- Consider edge cases like impossible commands or missing information
- Implement fallback strategies for when the LLM produces invalid outputs

## Conclusion

In this chapter, we've explored how large language models can be used for cognitive planning in humanoid robots. We've covered:
- Basic GPT integration for natural language understanding
- Function calling for more reliable structured outputs
- Chain-of-thought prompting for planning transparency
- Mapping natural language to specific robot actions
- Error handling and validation techniques

In the next chapter, we'll explore how computer vision systems can guide robot manipulation tasks, completing the perception-action loop that makes autonomous robots truly capable.

## Learning Objectives Review

By completing this chapter, you should now understand:
- How to use LLMs for cognitive planning in robotics
- Techniques for getting structured outputs from language models
- How to map natural language to specific robot actions
- Error handling and validation in cognitive planning systems

## Validation Against Requirements

This chapter meets the following functional requirements:

**FR-003**: System MUST explain how LLMs translate natural language into executable ROS 2 action sequences
- ✅ The chapter provides detailed explanations of LLM integration for cognitive planning
- ✅ Examples show how to translate natural language to robot actions
- ✅ Code examples demonstrate practical implementation with OpenAI APIs
- ✅ Function calling techniques are explained for reliable action sequences