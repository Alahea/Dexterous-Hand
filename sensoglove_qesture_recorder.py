#!/usr/bin/env python3
"""
SensoGlove DK3 Gesture Recorder

This script captures and saves hand gestures from the SensoGlove DK3
for later use with robot hands or other applications.
"""

from sensoglove import SensoGlove
from sensoglove.helpers import read_json_payload
import time
import math
import json
import os
from datetime import datetime

def process_imu_to_angles(imu_values):
    """Convert raw IMU values to pitch and yaw angles."""
    if len(imu_values) < 4 or imu_values[0] == 0:  # Skip inactive sensors
        return None, None
    
    x, y, z = imu_values[1], imu_values[2], imu_values[3]
    
    # Convert to pitch and yaw angles
    pitch = math.atan2(y, math.sqrt(x*x + z*z)) * 180 / math.pi
    yaw = math.atan2(x, z) * 180 / math.pi
    
    return pitch, yaw

def capture_gesture_data(glove_socket, max_attempts=5):
    """Capture a single frame of gesture data with retry logic."""
    for attempt in range(max_attempts):
        try:
            data = read_json_payload(glove_socket)
            
            if data is not None:
                data_type = data.get('type', 'UNKNOWN')
                
                if data_type == 'raw_data':
                    imu_data = data.get('data', {})
                    
                    # Map IMU sensors to fingers
                    finger_mapping = {
                        'imu_1': 'thumb',
                        'imu_2': 'index',
                        'imu_3': 'middle',
                        'imu_4': 'ring',
                        'imu_5': 'little',
                        'imu_0': 'palm',
                        'imu_6': 'wrist'
                    }
                    
                    gesture_data = {}
                    
                    for imu_key, finger_name in finger_mapping.items():
                        if imu_key in imu_data:
                            imu_values = imu_data[imu_key]
                            pitch, yaw = process_imu_to_angles(imu_values)
                            
                            if pitch is not None and yaw is not None:
                                gesture_data[finger_name] = {
                                    'pitch': round(pitch, 2),
                                    'yaw': round(yaw, 2),
                                    'raw_values': imu_values[:4]  # Store first 4 values for reference
                                }
                    
                    if gesture_data:  # Only return if we have valid data
                        return gesture_data
                
                elif data_type == 'state':
                    # Skip state data and try again
                    continue
            
            # Very short delay before retry
            time.sleep(0.02)
            
        except Exception as e:
            time.sleep(0.02)
    
    return None

def save_gesture(gesture_data, gesture_name, filename="gestures.json"):
    """Save gesture data to a JSON file."""
    # Create gestures directory if it doesn't exist
    gestures_dir = "gestures"
    if not os.path.exists(gestures_dir):
        os.makedirs(gestures_dir)
    
    filepath = os.path.join(gestures_dir, filename)
    
    # Load existing gestures or create new file
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            gestures = json.load(f)
    else:
        gestures = {}
    
    # Add new gesture with timestamp
    gestures[gesture_name] = {
        'timestamp': datetime.now().isoformat(),
        'data': gesture_data
    }
    
    # Save updated gestures
    with open(filepath, 'w') as f:
        json.dump(gestures, f, indent=2)
    
    print(f"✓ Gesture '{gesture_name}' saved to {filepath}")

def display_gesture_data(gesture_data):
    """Display gesture data in a readable format."""
    print("Current Hand Position:")
    for finger, data in gesture_data.items():
        print(f"  {finger.capitalize():<8} - Pitch: {data['pitch']:7.2f}°, Yaw: {data['yaw']:7.2f}°")

def main():
    # Create SensoGlove instance
    glove = SensoGlove('127.0.0.1', 53450)
    
    try:
        # Connect to the glove
        print("SensoGlove DK3 Gesture Recorder")
        print("=" * 40)
        print("Connecting to SensoGlove DK3...")
        glove.connect()
        print(f"Connected to {glove.name} (Source: {glove.src})")
        
        print("\nGesture Recording Instructions:")
        print("1. Position your hand in the desired gesture")
        print("2. Type the gesture name when prompted")
        print("3. Press Enter to capture the gesture")
        print("4. Type 'quit' to exit")
        print("5. Type 'preview' to see current hand position")
        print("=" * 40)
        
        while True:
            # Get user input
            user_input = input("\nEnter gesture name (or 'quit'/'preview'): ").strip()
            
            if user_input.lower() == 'quit':
                break
            elif user_input.lower() == 'preview':
                # Show current hand position
                print("\nCapturing current hand position...")
                gesture_data = capture_gesture_data(glove.socket)
                if gesture_data:
                    display_gesture_data(gesture_data)
                else:
                    print("No gesture data received. Checking connection...")
                    # Try to get any data to diagnose
                    for i in range(5):
                        try:
                            test_data = read_json_payload(glove.socket)
                            if test_data:
                                print(f"Received data type: {test_data.get('type', 'UNKNOWN')}")
                                break
                        except:
                            pass
                        time.sleep(0.1)
                    else:
                        print("No data received at all. Check if the glove is connected and active.")
                continue
            elif user_input == '':
                print("Please enter a gesture name.")
                continue
            
            gesture_name = user_input
            
            # Countdown for gesture capture
            print(f"\nPreparing to capture gesture '{gesture_name}'...")
            for i in range(3, 0, -1):
                print(f"Ready in {i}...")
                time.sleep(1)
            
            print("Capturing gesture now!")
            
            # Capture multiple frames and average them for stability
            captured_frames = []
            print("Capturing frames: ", end="", flush=True)
            
            # Be more patient and allow more time for data capture
            max_attempts = 30  # Try up to 30 times
            target_frames = 5   # We want at least 5 good frames
            
            for i in range(max_attempts):
                gesture_data = capture_gesture_data(glove.socket)
                if gesture_data:
                    captured_frames.append(gesture_data)
                    print("✓", end="", flush=True)
                else:
                    print("✗", end="", flush=True)
                
                time.sleep(0.05)  # Small delay between captures
                
                # Stop if we have enough good frames
                if len(captured_frames) >= target_frames:
                    break
            
            print(f" ({len(captured_frames)}/{target_frames} frames captured)")  # Show success rate
            
            if captured_frames:
                # Check if we have enough frames for reliable data
                if len(captured_frames) < 2:
                    print(f"⚠️  Warning: Only captured {len(captured_frames)} frame(s). Gesture may be less accurate.")
                    retry_choice = input("Retry capture? (y/n): ").strip().lower()
                    if retry_choice == 'y':
                        continue  # Go back to gesture name input
                
                # Average the captured frames for more stable data
                averaged_gesture = {}
                for finger in captured_frames[0].keys():
                    pitch_sum = sum(frame[finger]['pitch'] for frame in captured_frames)
                    yaw_sum = sum(frame[finger]['yaw'] for frame in captured_frames)
                    
                    averaged_gesture[finger] = {
                        'pitch': round(pitch_sum / len(captured_frames), 2),
                        'yaw': round(yaw_sum / len(captured_frames), 2),
                        'raw_values': captured_frames[0][finger]['raw_values']  # Use first frame's raw values
                    }
                
                # Display captured gesture
                print("\nCaptured Gesture:")
                display_gesture_data(averaged_gesture)
                
                # Save gesture
                save_gesture(averaged_gesture, gesture_name)
                
                # Show summary
                gestures_file = os.path.join("gestures", "gestures.json")
                if os.path.exists(gestures_file):
                    with open(gestures_file, 'r') as f:
                        all_gestures = json.load(f)
                    print(f"\nTotal gestures saved: {len(all_gestures)}")
                    print("Saved gestures:", list(all_gestures.keys()))
                
            else:
                print("❌ Failed to capture any gesture data.")
                print("Tips to improve capture success:")
                print("1. Make sure you're wearing the glove properly")
                print("2. Move your hand slightly during capture")
                print("3. Check that the glove is fully connected")
                print("4. Try the 'preview' command to test data flow")
        
    except KeyboardInterrupt:
        print("\nGesture recording interrupted.")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Close the connection
        if glove.socket:
            glove.socket.close()
        print("Connection closed.")
        
        # Show final summary
        gestures_file = os.path.join("gestures", "gestures.json")
        if os.path.exists(gestures_file):
            with open(gestures_file, 'r') as f:
                all_gestures = json.load(f)
            print(f"\nSession complete! Total gestures saved: {len(all_gestures)}")
            print("Gestures file location:", os.path.abspath(gestures_file))

if __name__ == "__main__":
    main()
