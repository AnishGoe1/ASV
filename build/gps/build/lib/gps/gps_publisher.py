#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from pyubx2 import UBXReader
from sensor_msgs.msg import NavSatFix, NavSatStatus

class GPSPublisher(Node):

    def __init__(self):
        super().__init__("gps_publisher")

        # ---------------- Serial ----------------
        self.PORT = "/dev/ttyACM1"
        self.BAUD = 115200
        self.SENSOR_MAP = {16: "Accel X", 17: "Accel Y", 18: "Accel Z", 14: "Gyro X", 13: "Gyro Y", 5: "Gyro Z"}
        self.FIX_TYPES = {0: "No Fix", 1: "DR Only", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "Time"}

        try:
            self.ser = serial.Serial(self.PORT, self.BAUD, timeout=1)
            self.get_logger().info("Serial connected ✅")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            raise e

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(NavSatFix, "/fix", 10)

        # ---------------- Storage ----------------
        self.lat = None
        self.lon = None
        self.alt = None

        self.fix_quality = 0
        self.rmc_valid = False

        # ---------------- Timer (10 Hz) ----------------
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("GPS Node Started 🚀")


    # ---------- NMEA → Decimal (STRING BASED) ----------
    def nmea_to_decimal(self, coord, direction):

        if not coord:
            return None

        # Latitude = DDMM.MMMM
        # Longitude = DDDMM.MMMM

        if direction in ["N", "S"]:
            deg_len = 2
        else:
            deg_len = 3

        degrees = int(coord[:deg_len])
        minutes = float(coord[deg_len:])

        decimal = degrees + (minutes / 60.0)

        if direction in ["S", "W"]:
            decimal = -decimal

        return decimal


    # ---------- Main Loop ----------
    def timer_callback(self):

        try:
            line = self.ser.readline().decode(errors="ignore").strip()
        except:
            return


        # ---------- RMC (Validity + Lat/Lon) ----------
        if line.startswith("$GNRMC"):

            data = line.split(",")

            if len(data) > 6:

                # Valid / Invalid
                if data[2] == "A":
                    self.rmc_valid = True
                else:
                    self.rmc_valid = False

                # Position
                if self.rmc_valid:

                    self.lat = self.nmea_to_decimal(data[3], data[4])
                    self.lon = self.nmea_to_decimal(data[5], data[6])


        # ---------- GGA (Fix + Altitude) ----------
        elif line.startswith("$GNGGA"):

            data = line.split(",")

            if len(data) > 9:

                # Fix quality
                try:
                    self.fix_quality = int(data[6])
                except:
                    self.fix_quality = 0

                # Altitude
                try:
                    self.alt = float(data[9])
                except:
                    pass


        # ---------- Publish When Ready ----------
        if (
            self.lat is not None and
            self.lon is not None and
            self.alt is not None
        ):

            msg = NavSatFix()

            # Header
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"

            # ---------- Status Mapping ----------
            if self.rmc_valid and self.fix_quality > 0:

                if self.fix_quality == 1:
                    msg.status.status = NavSatStatus.STATUS_FIX

                elif self.fix_quality == 2:
                    msg.status.status = NavSatStatus.STATUS_SBAS_FIX

                elif self.fix_quality >= 4:
                    msg.status.status = NavSatStatus.STATUS_GBAS_FIX

                else:
                    msg.status.status = NavSatStatus.STATUS_FIX

            else:
                msg.status.status = NavSatStatus.STATUS_NO_FIX


            msg.status.service = NavSatStatus.SERVICE_GPS


            # ---------- Position ----------
            msg.latitude = float(self.lat)
            msg.longitude = float(self.lon)
            msg.altitude = float(self.alt)


            # ---------- Covariance ----------
            msg.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            )


            # ---------- Publish ----------
            self.pub.publish(msg)

            self.get_logger().info(
                f"Lat: {self.lat:.6f} | "
                f"Lon: {self.lon:.6f} | "
                f"Alt: {self.alt:.2f} m | "
            )


    # ---------- Shutdown ----------
    def destroy_node(self):

        self.get_logger().info("Shutting down GPS node...")

        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial closed ✅")

        return super().destroy_node()



# ---------- Main ----------
def main(args=None):

    rclpy.init(args=args)

    node = GPSPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()