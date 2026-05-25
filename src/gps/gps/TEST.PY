import serial
from pyubx2 import UBXReader

# Configuration
PORT = 'COM3'
BAUD = 115200  # Based on your latest terminal log speed

# Sensor & Fix Mappings
SENSOR_MAP = {16: "Accel X", 17: "Accel Y", 18: "Accel Z", 14: "Gyro X", 13: "Gyro Y", 5: "Gyro Z"}
FIX_TYPES = {0: "No Fix", 1: "DR Only", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "Time"}

def decode_signed_24(val):
    """Converts 24-bit signed binary values to integers."""
    if val > 0x7FFFFF:
        val -= 0x1000000
    return val

def run_unified_parser():
    try:
        # Open serial port
        stream = serial.Serial(PORT, BAUD, timeout=0.1)
        ubr = UBXReader(stream)
        print(f"--- ZED-F9R Unified Parser Listening on {PORT} ---")

        for (raw_data, parsed_data) in ubr:
            # 1. Extract Position & Fix Type (NMEA)
            if parsed_data.identity in ['GNRMC', 'GNGGA']:
                lat = getattr(parsed_data, 'lat', "N/A")
                lon = getattr(parsed_data, 'lon', "N/A")
                # Fix Type is usually in GNGGA (field 6)
                fix = "3D Fix" if parsed_data.identity == 'GNGGA' else "Active"
                print(f"[GPS] Lat: {lat}, Lon: {lon} | Status: {fix}")

            # 2. Extract Heading & High-Res Fix (UBX-NAV-PVT)
            elif parsed_data.identity == "NAV-PVT":
                heading = parsed_data.headMot / 10**5
                fix = FIX_TYPES.get(parsed_data.fixType, "Unknown")
                print(f"[NAV] Heading: {heading:.2f}° | Fix Type: {fix}")

            # 3. Extract Roll, Pitch, Yaw (UBX-NAV-ATT)
            elif parsed_data.identity == "NAV-ATT":
                roll = parsed_data.roll / 10**5
                pitch = parsed_data.pitch / 10**5
                yaw = parsed_data.heading / 10**5
                print(f"[ATTITUDE] Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")

            # 4. Extract Accelerations (UBX-ESF-MEAS)
            elif parsed_data.identity == "ESF-MEAS":
                accel_data = []
                for i in range(1, parsed_data.numMeas + 1):
                    val = getattr(parsed_data, f"dataField_{i:02}")
                    dtype = getattr(parsed_data, f"dataType_{i:02}")
                    
                    if dtype in [16, 17, 18]: # Accelerometer types
                        real_accel = decode_signed_24(val) * (2**-10)
                        accel_data.append(f"{SENSOR_MAP[dtype]}: {real_accel:.3f} m/s²")
                
                if accel_data:
                    print(f"[IMU] {' | '.join(accel_data)}", end='\r')

    except Exception as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stream.close()

if __name__ == "_main_":
    run_unified_parser()