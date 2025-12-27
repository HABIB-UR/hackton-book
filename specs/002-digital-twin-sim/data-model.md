# Data Model: Digital Twin Simulation (Gazebo & Unity)

**Feature**: 002-digital-twin-sim
**Date**: 2025-12-22
**Status**: Complete

## Overview

This data model describes the educational content entities and relationships for the Digital Twin Simulation module. The model focuses on the content structure and educational flow rather than runtime data.

## Core Entities

### Simulation Chapter
**Description**: Educational content unit covering specific simulation concepts
**Fields**:
- title: String (e.g., "Gazebo Physics Simulation", "Unity Interaction Model", "Sensor Simulation")
- slug: String (URL-friendly identifier)
- content: Markdown text with educational material
- learning_objectives: Array of strings
- prerequisites: Array of strings (knowledge required)
- duration: Integer (estimated completion time in minutes)
- difficulty_level: Enum (beginner, intermediate, advanced)
- assessment_link: String (reference to related quiz)
- examples: Array of file paths (practical examples)
- references: Array of strings (external resources)

**Relationships**:
- Has many → Simulation Examples
- Has one → Assessment Quiz
- Preceded by → Previous Chapter (for learning sequence)

### Simulation Example
**Description**: Practical example file demonstrating simulation concepts
**Fields**:
- name: String (descriptive name)
- file_path: String (location in examples directory)
- description: String (what the example demonstrates)
- simulation_type: Enum (gazebo, unity, sensor)
- language: String (programming language if applicable)
- complexity: Enum (basic, intermediate, advanced)

**Relationships**:
- Belongs to → Simulation Chapter
- References → Humanoid Robot Model

### Humanoid Robot Model
**Description**: Virtual representation of a humanoid robot used in simulation environments
**Fields**:
- name: String (model identifier)
- urdf_path: String (path to URDF file)
- joint_count: Integer (number of joints)
- sensor_configurations: Array of sensor configuration objects
- physical_properties: Object (mass, dimensions, etc.)
- mesh_paths: Array of strings (3D model file paths)

**Relationships**:
- Used by → Simulation Chapter
- Used by → Simulation Example

### Virtual Environment
**Description**: Digital space where humanoid robots operate with physics, rendering, and sensor simulation
**Fields**:
- name: String (environment identifier)
- description: String (environment characteristics)
- physics_properties: Object (gravity, friction, etc.)
- lighting_config: Object (for Unity rendering)
- collision_meshes: Array of strings (collision geometry paths)
- sensor_zones: Array of objects (areas where sensors function)

**Relationships**:
- Used by → Simulation Chapter
- Contains → Humanoid Robot Model

### Assessment Quiz
**Description**: Evaluation materials to validate student understanding of simulation concepts
**Fields**:
- title: String (e.g., "Gazebo Physics Quiz")
- chapter_link: String (reference to related chapter)
- questions: Array of question objects
- passing_score: Integer (percentage required to pass)
- time_limit: Integer (minutes, 0 if no limit)
- difficulty_level: Enum (beginner, intermediate, advanced)

**Relationships**:
- Belongs to → Simulation Chapter
- Tests → Learning Objectives

### Question
**Description**: Individual assessment item within a quiz
**Fields**:
- question_text: String
- question_type: Enum (multiple_choice, true_false, short_answer)
- options: Array of strings (for multiple choice)
- correct_answer: String or Array
- explanation: String (why the answer is correct)
- difficulty: Enum (easy, medium, hard)

**Relationships**:
- Belongs to → Assessment Quiz

## Content Relationships

```
Simulation Chapter (1) → (n) Simulation Examples
Simulation Chapter (1) → (1) Assessment Quiz
Simulation Chapter (1) → (n) Learning Objectives

Simulation Example (n) → (1) Humanoid Robot Model
Simulation Example (n) → (1) Virtual Environment

Virtual Environment (n) → (n) Humanoid Robot Model (many-to-many)

Assessment Quiz (n) → (n) Learning Objectives (many-to-many)
```

## Validation Rules

### Simulation Chapter
- Title must be unique within the module
- Content must include at least one practical example
- Duration must be between 15 and 120 minutes
- Difficulty level must match content complexity

### Simulation Example
- File path must exist in the examples directory
- Must be associated with a valid simulation chapter
- Complexity must align with parent chapter difficulty

### Assessment Quiz
- Must have at least 5 questions
- Passing score must be between 60% and 90%
- Questions must align with chapter learning objectives

### Humanoid Robot Model
- URDF file must be valid and exist
- Joint count must be consistent with humanoid structure
- Physical properties must be physically plausible

## State Transitions

### Chapter Progression
- Draft → In Review → Approved → Published
- Students progress through chapters in sequence (though individual chapters can be accessed independently)

### Quiz States
- Not Started → In Progress → Completed → Reviewed
- Results can be stored with pass/fail status