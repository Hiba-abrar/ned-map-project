# 🎤 Voice-Driven NED Campus Navigator

Advanced voice-controlled navigation system with step-based directions and dynamic marker movement.

## 🌟 New Voice Features Implemented

### 🗣️ **Complete Voice Input Workflow**
1. **Startup**: System asks "Please say your starting location"
2. **Voice Recognition**: Detects and matches spoken locations
3. **Destination Input**: "Please say your destination"
4. **Auto Route Generation**: Calculates path automatically after both inputs
5. **Voice Commands**: "Next" and "Repeat" for step-by-step navigation

### 🧾 **Enhanced Direction Instructions**
- **Left/Right/Forward**: "Turn left and move 25 steps toward Library"
- **Step-Based**: All distances converted from meters to steps (1m = 1.25 steps)
- **Directional Awareness**: Uses coordinate geometry to calculate turns
- **Clear Actions**: "Move", "Turn right", "Turn left", "Continue straight", "Turn around"

### 🦶 **Step Conversion System**
- **Accurate Conversion**: 1 meter ≈ 1.25 steps (0.8m per step)
- **Whole Numbers**: All step counts rounded to integers
- **Example**: 24 meters → "Move 30 steps forward toward Auditorium"

### 🗺️ **Dynamic Marker Movement**
- **Real-time Updates**: Red marker moves along route as user progresses
- **Smooth Animation**: Marker smoothly transitions between waypoints
- **Visual Progress**: Map pans to follow current position
- **Destination Announcement**: "You have reached your destination!"

### 🗣️ **Voice Output & Commands**
- **TTS Instructions**: Each step spoken clearly with natural voice
- **Voice Commands**: 
  - Say **"Next"** → Move to next instruction
  - Say **"Repeat"** → Hear current instruction again
- **Continuous Listening**: Always ready for voice commands during navigation

## 🚀 Quick Start

### Run the Voice Navigator:
```cmd
build_voice.bat
```

Then open: `http://localhost:8080`

### Usage Flow:
1. **Click** "🎤 Start Voice Navigation"
2. **Speak** your starting location when prompted
3. **Speak** your destination when prompted
4. **Follow** step-by-step voice instructions
5. **Say** "next" or "repeat" to control navigation

## 🎯 Example Voice Interaction

```
System: "Please say your starting location"
User: "Main Gate"
System: "Starting location set to NED University Main Gate. Please say your destination."

User: "Library"
System: "Destination set to NED University Library. Calculating route..."
System: "Route calculated. 3 steps to destination. Starting navigation."

System: "Move 45 steps northeast toward NED University Admin Block"
User: "Next"
System: "Turn right and move 32 steps east toward NED University Library"
User: "Next"
System: "You have reached your destination!"
```

## 🏗️ Technical Implementation

### Voice Recognition Features:
- **Fuzzy Matching**: Recognizes partial names and keywords
- **Keyword System**: "gate" → Main Gate, "library" → Library
- **Error Handling**: Politely asks again if speech unclear
- **Multiple Formats**: Handles various ways of saying location names

### Direction Calculation:
- **Bearing Calculation**: Uses coordinate geometry for precise directions
- **Turn Detection**: Analyzes angle differences between path segments
- **Compass Directions**: North, Northeast, East, Southeast, etc.
- **Action Classification**: Move, Turn left, Turn right, Continue straight

### Step Conversion Logic:
```cpp
// Convert meters to steps
int steps = (int)round(distance_meters * 1.25);

// Generate instruction
string instruction = action + " " + to_string(steps) + 
                    " steps " + direction + " toward " + destination;
```

### Dynamic Marker System:
- **Leaflet.js Integration**: Smooth marker animation
- **Coordinate Updates**: Real-time position tracking
- **Map Panning**: Automatic map centering on current position
- **Progress Visualization**: Visual progress bar and step counter

## 📍 Supported Locations & Voice Keywords

| Location | Voice Keywords |
|----------|----------------|
| NED University Main Gate | "main gate", "gate", "entrance" |
| NED University Library | "library", "books", "study" |
| NED University Admin Block | "admin", "office", "administration" |
| CSIT Labs | "csit", "computer", "lab", "cs" |
| Main Auditorium | "auditorium", "hall", "events" |
| DMS Cafeteria | "cafeteria", "food", "dining", "dms" |
| Civil Engineering Class Rooms | "civil", "engineering", "classroom" |
| Survey Lab | "survey", "lab", "surveying" |
| Mosque | "mosque", "prayer", "masjid" |
| Basketball Court | "basketball", "court", "sports" |
| Tennis Court | "tennis", "court" |
| Football Ground | "football", "ground", "field" |
| Mathematics Department | "math", "mathematics", "dept" |
| Mechanical Engineering Department | "mechanical", "engineering", "mech" |
| Environmental Engineering | "environmental", "env" |
| Electrical Engineering Department | "electrical", "ee", "elect" |

## 🎮 Voice Commands

### During Navigation:
- **"Next"** / **"Continue"** → Move to next step
- **"Repeat"** / **"Again"** → Repeat current instruction
- **"Previous"** / **"Back"** → Go to previous step (button only)

### Location Input:
- Say location names naturally: "Library", "Main Gate", "Computer Lab"
- Use keywords: "Admin" for Admin Block, "Food" for Cafeteria
- Partial names work: "Civil Engineering" or just "Civil"

## 🔧 Browser Requirements

### Essential Permissions:
- **Microphone Access**: For voice recognition
- **Location Access**: Optional for GPS features
- **Audio Output**: For text-to-speech

### Supported Browsers:
- **Chrome 60+** (Recommended)
- **Firefox 55+**
- **Safari 11+**
- **Edge 79+**

### Voice Features:
- **Web Speech API**: Built-in browser voice recognition
- **Speech Synthesis**: Natural text-to-speech output
- **Continuous Listening**: Background voice command detection

## 🛠️ Troubleshooting

### Voice Recognition Issues:
1. **Check microphone permissions** in browser settings
2. **Speak clearly** and wait for listening indicator
3. **Use keywords** if full names not recognized
4. **Try alternative names** for locations

### Common Voice Commands:
- If system doesn't understand: Try saying location keywords
- If stuck on instruction: Say "repeat" to hear again
- If want to skip: Say "next" to continue

### Performance Tips:
- **Use Chrome** for best voice recognition
- **Quiet environment** improves accuracy
- **Clear speech** at normal pace
- **Wait for prompts** before speaking

## 📊 Navigation Accuracy

### Step Conversion:
- **Base Rate**: 1 meter = 1.25 steps
- **Average Step**: 0.8 meters (adult walking pace)
- **Accuracy**: ±2-3 steps for typical campus distances

### Direction Precision:
- **Bearing Calculation**: Precise coordinate-based angles
- **Turn Detection**: ±15° accuracy for left/right classification
- **Compass Directions**: 8-point compass (N, NE, E, SE, S, SW, W, NW)

## 🎯 Complete Workflow Summary

1. **🎤 Voice Startup**: "Please say your starting location"
2. **📍 Location Recognition**: System matches spoken input to campus locations
3. **🎯 Destination Input**: "Please say your destination"
4. **🗺️ Route Calculation**: Automatic pathfinding with step-based instructions
5. **📢 Voice Instructions**: Clear spoken directions with left/right/forward
6. **🚶 Step-by-Step**: "Move 30 steps northeast toward Library"
7. **🔄 Voice Control**: "Next" to continue, "Repeat" to hear again
8. **📍 Dynamic Tracking**: Red marker moves along route in real-time
9. **🎉 Arrival**: "You have reached your destination!"

## 🚀 Advanced Features

### Smart Location Matching:
- **Fuzzy Search**: Handles mispronunciations and variations
- **Context Awareness**: Suggests similar locations if not found
- **Multi-keyword Support**: Multiple ways to reference same location

### Enhanced Navigation:
- **Progress Tracking**: Visual progress bar and step counter
- **Route Summary**: Total steps, distance, and instruction count
- **Visual Feedback**: Current instruction highlighting and animations

### Voice Intelligence:
- **Natural Language**: Understands conversational speech patterns
- **Error Recovery**: Graceful handling of unclear speech
- **Continuous Operation**: Always listening during navigation

---

**🎓 Perfect for NED University Navigation!**  
*Experience hands-free, voice-driven campus navigation with precise step-by-step directions.*