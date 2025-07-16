#!/usr/bin/env python3
"""
SensoGlove DK3 Demo - Raw IMU Data

This script demonstrates how to connect to a SensoGlove DK3 and continuously
fetch raw IMU angular data from the sensors.
"""

from sensoglove import SensoGlove
from sensoglove.helpers import read_json_payload
import time
import math

def process_imu_to_angles(imu_values):
    """Convert raw IMU values to pitch and yaw angles."""
    if len(imu_values) < 4 or imu_values[0] == 0:  # Skip inactive sensors
        return None, None
    
    x, y, z = imu_values[1], imu_values[2], imu_values[3]
    
    # Convert to pitch and yaw angles
    pitch = math.atan2(y, math.sqrt(x*x + z*z)) * 180 / math.pi
    yaw = math.atan2(x, z) * 180 / math.pi
    
    return pitch, yaw

def main():
    # Create SensoGlove instance
    glove = SensoGlove('127.0.0.1', 53450)
    
    try:
        # Connect to the glove
        print("Connecting to SensoGlove DK3...")
        glove.connect()
        print(f"Connected to {glove.name} (Source: {glove.src})")
        
        # Continuous data fetching loop
        print("\nStarting IMU data collection... Press Ctrl+C to stop")
        print("Note: DK3 provides angular data from IMU sensors")
        print("=" * 60)
        
        while True:
            # Read raw data directly since DK3 sends raw_data, not position data
            data = read_json_payload(glove.socket)
            
            if data is not None:
                data_type = data.get('type', 'UNKNOWN')
                
                if data_type == 'raw_data':
                    imu_data = data.get('data', {})
                    
                    # Process IMU data for each finger
                    print("Raw IMU Angular Data:")
                    
                    # Map IMU sensors to fingers (based on typical DK3 layout)
                    finger_mapping = {
                        'imu_1': 'Thumb',
                        'imu_2': 'Index finger',
                        'imu_3': 'Middle finger',
                        'imu_4': 'Ring finger',
                        'imu_5': 'Little finger',
                        'imu_0': 'Palm',
                        'imu_6': 'Wrist'
                    }
                    
                    for imu_key, finger_name in finger_mapping.items():
                        if imu_key in imu_data:
                            imu_values = imu_data[imu_key]
                            pitch, yaw = process_imu_to_angles(imu_values)
                            
                            if pitch is not None and yaw is not None:
                                print(f"  {finger_name:<13} - Pitch: {pitch:7.2f}°, Yaw: {yaw:7.2f}°")
                    
                    print("-" * 60)
                    
                elif data_type == 'state':
                    # Display state information
                    print(f"Glove State:")
                    print(f"  Battery: {data.get('battery', 'Unknown')}%")
                    print(f"  Temperature: {data.get('temperature', 'Unknown')}°C")
                    print(f"  Mode: {data.get('mode', 'Unknown')}")
                    print(f"  Calibrated: {data.get('calibrated', 'Unknown')}")
                    print("-" * 60)
            
            # Small delay to prevent overwhelming output
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping data collection...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the connection
        if glove.socket:
            glove.socket.close()
        print("Connection closed.")

if __name__ == "__main__":
    main()
