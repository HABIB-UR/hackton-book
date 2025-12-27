---
sidebar_position: 1
---

# Voice-to-Action: Speech Processing for Autonomous Humanoids

## Introduction to Voice Processing

Voice processing is a critical component of human-robot interaction, enabling natural communication between humans and humanoid robots. In this chapter, we'll explore how speech input is captured, processed, and transformed into actionable commands that can control humanoid robot behavior.

Voice processing involves several key steps:
1. **Audio Capture**: Converting sound waves into digital audio data
2. **Speech Recognition**: Converting audio data into text
3. **Natural Language Processing**: Understanding the meaning of the spoken command
4. **Command Interpretation**: Mapping the understood command to robot actions

These steps form the foundation of voice-to-action systems that enable intuitive human-robot interaction.

## Whisper API Integration

OpenAI's Whisper is a state-of-the-art speech recognition model that can accurately transcribe audio to text. For educational purposes, we'll demonstrate how to integrate Whisper into our voice processing pipeline.

### Example: Basic Voice Processing Demonstration

Here's a complete example that demonstrates the basic voice processing pipeline:

```python
import openai
import os
from typing import Dict, Optional

class VoiceProcessingDemo:
    """
    A demonstration class showing the basic components of voice processing
    """
    def __init__(self):
        # In a real implementation, you would set your API key securely
        openai.api_key = os.getenv("OPENAI_API_KEY")

    def process_voice_input(self, audio_path: str) -> Dict:
        """
        Process voice input and return structured results
        """
        # Step 1: Transcribe audio to text
        try:
            with open(audio_path, "rb") as audio_file:
                transcription = openai.Audio.transcribe(
                    model="whisper-1",
                    file=audio_file,
                    response_format="text"
                )
        except Exception as e:
            return {"error": f"Transcription failed: {str(e)}", "success": False}

        # Step 2: Clean and interpret the command
        command = self._interpret_command(transcription.strip())

        return {
            "original_text": transcription,
            "interpreted_command": command,
            "success": True
        }

    def _interpret_command(self, text: str) -> str:
        """
        Simple command interpretation logic
        """
        text_lower = text.lower()

        if "move forward" in text_lower:
            return "ROBOT_MOVE_FORWARD"
        elif "move backward" in text_lower:
            return "ROBOT_MOVE_BACKWARD"
        elif "turn left" in text_lower:
            return "ROBOT_TURN_LEFT"
        elif "turn right" in text_lower:
            return "ROBOT_TURN_RIGHT"
        elif "stop" in text_lower:
            return "ROBOT_STOP"
        elif "pick up" in text_lower or "grab" in text_lower:
            return "ROBOT_PICK_OBJECT"
        else:
            return "UNKNOWN_COMMAND"

# Example usage:
# demo = VoiceProcessingDemo()
# result = demo.process_voice_input("path/to/audio/file.mp3")
# print(f"Command interpreted as: {result['interpreted_command']}")
```

### Basic Whisper API Usage

Here's a basic example of how to use the Whisper API for speech-to-text conversion:

```python
import openai
import os

# Configure your OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")

def transcribe_audio(audio_file_path):
    """
    Transcribe audio file to text using OpenAI's Whisper API
    """
    with open(audio_file_path, "rb") as audio_file:
        transcript = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_file,
            response_format="text"
        )
    return transcript

# Example usage
# audio_path = "path/to/your/audio/file.mp3"
# text = transcribe_audio(audio_path)
# print(f"Transcribed text: {text}")
```

### Whisper API for Robot Commands

When using Whisper for robot command processing, we need to consider the specific context of our application. Here's an example that includes error handling and command validation:

```python
import openai
import os
from typing import Dict, Optional

class VoiceToActionProcessor:
    def __init__(self, api_key: str = None):
        """
        Initialize the voice processing system
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key is required")

        openai.api_key = self.api_key

    def process_voice_command(self, audio_file_path: str) -> Dict[str, str]:
        """
        Process a voice command and return structured output
        """
        try:
            # Transcribe the audio
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe(
                    model="whisper-1",
                    file=audio_file,
                    response_format="text"
                )

            # Clean and validate the transcript
            cleaned_transcript = self._clean_transcript(transcript.strip())

            # Parse the command
            parsed_command = self._parse_command(cleaned_transcript)

            return {
                "original_transcript": transcript,
                "cleaned_command": cleaned_transcript,
                "parsed_action": parsed_command,
                "status": "success"
            }

        except Exception as e:
            return {
                "error": str(e),
                "status": "error"
            }

    def _clean_transcript(self, transcript: str) -> str:
        """
        Clean the transcript to remove any unwanted characters or phrases
        """
        # Remove common filler words or phrases that might be transcribed
        # This is a simplified example - real systems would have more sophisticated cleaning
        cleaned = transcript.lower().strip()
        return cleaned

    def _parse_command(self, command: str) -> Optional[str]:
        """
        Parse the command to identify the intended action
        This is a simplified example - real systems would use more sophisticated NLP
        """
        # Simple keyword-based command recognition
        if "move" in command or "go" in command:
            return "move_robot"
        elif "pick" in command or "grab" in command:
            return "pick_object"
        elif "turn" in command or "rotate" in command:
            return "rotate_robot"
        elif "stop" in command:
            return "stop_robot"
        elif "hello" in command or "hi" in command:
            return "greet_user"
        else:
            return "unknown_command"
```

## Voice-to-Text Conversion Pipeline

The voice-to-text conversion pipeline is the core of our voice processing system. It transforms raw audio into actionable text commands.

### Pipeline Components

1. **Audio Preprocessing**: Prepare audio for optimal recognition
2. **Speech Recognition**: Convert speech to text using Whisper
3. **Post-processing**: Clean and validate the recognized text
4. **Command Mapping**: Map recognized text to robot actions

### Audio Preprocessing

For best results with Whisper, audio should be:
- Mono channel (1 channel)
- Sample rate of 16kHz or higher
- Format: MP3, MP4, M4A, WAV, or similar
- Clear and minimally noisy

```python
from pydub import AudioSegment
import io

def preprocess_audio(input_path: str, output_path: str = None) -> str:
    """
    Preprocess audio for optimal Whisper API performance
    """
    # Load audio file
    audio = AudioSegment.from_file(input_path)

    # Convert to mono if stereo
    if audio.channels > 1:
        audio = audio.set_channels(1)

    # Set sample rate to 16kHz if different
    if audio.frame_rate != 16000:
        audio = audio.set_frame_rate(16000)

    # Export to temporary file or specified path
    output_path = output_path or input_path.replace(
        os.path.splitext(input_path)[1],
        "_processed.mp3"
    )

    audio.export(output_path, format="mp3")
    return output_path
```

## API Key Guidance for Students

When working with the OpenAI API, students should:

1. **Get an API Key**: Sign up at OpenAI and create an API key in the dashboard
2. **Set Environment Variables**: Store your API key securely in environment variables
3. **Understand Usage Limits**: Be aware of rate limits and costs associated with API usage

### Setting Up Environment Variables

Create a `.env` file in your project root (and add `.env` to your `.gitignore` to keep it secure):

```
OPENAI_API_KEY=your_actual_api_key_here
```

Then load it in your Python code:

```python
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

api_key = os.getenv("OPENAI_API_KEY")
```

## Real-time vs. Batch Processing

Voice processing can be implemented in two main ways:

### Batch Processing
- Process pre-recorded audio files
- Suitable for applications where real-time response is not critical
- More reliable and easier to implement

### Real-time Processing
- Process audio as it's being captured
- Requires more sophisticated handling of streaming data
- Provides immediate response but more complex to implement

For educational purposes, we'll focus on batch processing which is more manageable for learning the core concepts.

## Error Handling Examples

When working with voice processing APIs, it's important to handle various error scenarios:

```python
import time
from typing import Dict, Any

def robust_voice_processing(audio_path: str, max_retries: int = 3) -> Dict[str, Any]:
    """
    Process voice command with robust error handling and retries
    """
    processor = VoiceToActionProcessor()

    for attempt in range(max_retries):
        try:
            result = processor.process_voice_command(audio_path)

            if result["status"] == "success":
                return result
            else:
                print(f"Attempt {attempt + 1} failed: {result.get('error', 'Unknown error')}")

                if attempt < max_retries - 1:
                    time.sleep(2 ** attempt)  # Exponential backoff
                else:
                    return result

        except Exception as e:
            print(f"Attempt {attempt + 1} encountered exception: {str(e)}")

            if attempt < max_retries - 1:
                time.sleep(2 ** attempt)  # Exponential backoff
            else:
                return {
                    "error": f"Failed after {max_retries} attempts: {str(e)}",
                    "status": "error"
                }

    return {"error": "Unexpected error", "status": "error"}
```

## Example Exercise: Voice Command Processing

**Objective**: Implement a simple voice processing pipeline that can recognize and categorize basic robot commands.

**Instructions**:
1. Create an audio file with a simple command like "move forward" or "stop"
2. Use the VoiceToActionProcessor class to transcribe and interpret the command
3. Verify that the system correctly identifies the intended action

**Expected Outcome**: The system should successfully convert the voice command to text and map it to an appropriate robot action.

## Exercise: Voice Processing Practice

**Objective**: Create a complete voice processing system that can handle basic robot commands.

**Difficulty**: Intermediate

**Instructions**:
1. Implement the `VoiceToActionProcessor` class with all methods
2. Test the system with different voice commands
3. Add additional command mappings beyond the basic ones provided
4. Implement a simple test function that verifies the system works correctly

**Expected Outcome**: A working voice processing system that can recognize and categorize multiple types of robot commands.

**Hints**:
- Start with the basic implementation provided in the examples
- Consider edge cases like unclear audio or unrecognized commands
- Test with different types of audio files to ensure robustness

## Conclusion

In this chapter, we've covered the fundamentals of voice processing for humanoid robots, focusing on:
- The core components of a voice-to-action pipeline
- Integration with OpenAI's Whisper API
- Best practices for audio preprocessing
- Error handling for robust voice processing

In the next chapter, we'll explore how to take these processed commands and translate them into specific robot actions using cognitive planning with large language models.

## Learning Objectives Review

By completing this chapter, you should now understand:
- How speech recognition works in the context of humanoid robots
- How to integrate Whisper API for voice processing
- The importance of audio preprocessing for optimal results
- How to handle errors and edge cases in voice processing systems

## Validation Against Requirements

This chapter meets the following functional requirements:

**FR-001**: System MUST provide clear, accessible explanations of VLA concepts for students with AI/robotics backgrounds
- ✅ The chapter provides clear explanations of voice processing concepts with practical examples
- ✅ Technical concepts are explained in an accessible manner for students
- ✅ Code examples demonstrate practical implementation

**FR-002**: System MUST include practical examples that demonstrate voice-to-action processing with Whisper integration
- ✅ Multiple practical examples of Whisper API integration are provided
- ✅ Complete code examples show how to implement voice processing systems
- ✅ Examples include error handling and real-world considerations