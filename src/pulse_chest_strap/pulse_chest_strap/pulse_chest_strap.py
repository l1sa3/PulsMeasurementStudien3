#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import os
import sys
import time
import pexpect
import argparse
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

from pulse_publisher import PulsePublisher


class PulseChestStrap:

    def __init__(self):
        super().__init__("pulse_chest_strap")
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.publisher = PulsePublisher(self, "pulse_chest_strap")
        self.start_time = self.get_clock.now()

    def run(self, addr=None, gatttool="gatttool"):
        """
        main routine to which orchestrates everything
        """
        # number of measured pulse values. Increments for every measured value
        seq = 0

        if addr is None:
            # A mac address has to be provided as command line argument
            self.get_logger().error("[PulseChestStrap] MAC address of polar H7 has not been provided")
            return

        hr_handle = None
        hr_ctl_handle = None
        retry = True
        while retry:

            while 1:
                self.get_logger().info("[PulseChestStrap] Establishing connection to " + addr)
                gt = pexpect.spawn(gatttool + " -b " + addr + " -I")

                gt.expect(r"\[LE\]>")
                gt.sendline("connect")
                try:
                    i = gt.expect(["Connection successful.", r"\[CON\]"], timeout=30)
                    if i == 0:
                        gt.expect(r"\[LE\]>", timeout=30)

                except pexpect.TIMEOUT:
                    self.get_logger().info("[PulseChestStrap] Connection timeout. Retrying.")
                    continue

                except KeyboardInterrupt:
                    self.get_logger().info("[PulseChestStrap] Received keyboard interrupt. Quitting cleanly.")
                    retry = False
                    break
                break

            if not retry:
                break

            self.get_logger().info("[PulseChestStrap] Connected to " + addr)

            # We determine which handle we should read for getting the heart rate
            # measurement characteristic.
            gt.sendline("char-desc")

            while 1:
                try:
                    gt.expect(r"handle: (0x[0-9a-f]+), uuid: ([0-9a-f]{8})", timeout=10)
                except pexpect.TIMEOUT:
                    break
                handle = gt.match.group(1)
                uuid = gt.match.group(2)

                if uuid == b"00002902" and hr_handle:
                    hr_ctl_handle = handle
                    break

                elif uuid == b"00002a37":
                    hr_handle = handle

            if hr_handle is None:
                self.get_logger().error("[PulseChestStrap] Couldn't find the heart rate measurement handle?!")
                return

            if hr_ctl_handle:
                # We send the request to get HRM notifications
                gt.sendline("char-write-req " + hr_ctl_handle.decode("utf-8") + " 0100")

            # Time period between two measures. This will be updated automatically.
            period = 1.
            last_measure = time.time() - period
            hr_expect = "Notification handle = " + hr_handle.decode("utf-8") + " value: ([0-9a-f ]+)"

            while 1:
                try:
                    gt.expect(hr_expect, timeout=10)

                except pexpect.TIMEOUT:
                    # If the timer expires, it means that we have lost the
                    # connection with the HR monitor
                    self.get_logger().warn("[PulseChestStrap] Connection lost with " + addr + ". Reconnecting.")
                    gt.sendline("quit")
                    try:
                        gt.wait()
                    except:
                        pass
                    time.sleep(1)
                    break

                except KeyboardInterrupt:
                    self.get_logger().info("[PulseChestStrap] Received keyboard interrupt. Quitting cleanly.")
                    retry = False
                    break

                # We measure here the time between two measures. As the sensor
                # sometimes sends a small burst, we have a simple low-pass filter
                # to smooth the measure.
                tmeasure = time.time()
                period = period + 1 / 16. * ((tmeasure - last_measure) - period)
                last_measure = tmeasure

                # Get data from gatttool
                datahex = gt.match.group(1).strip()
                data = map(lambda x: int(x, 16), datahex.split(b' '))
                data = list(data)
                res = self.interpret(data)

                self.get_logger().info("[PulseChestStrap] Heart rate: " + str(res["hr"]))
                # todo unsicher wegen nanoseconds / 1e9 (Duration muss in Sekunden umgewandelt werden)
                timestamp = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.publisher.publish(res["hr"], timestamp)
                seq += 1

        # We quit close the BLE connection properly
        gt.sendline("quit")
        try:
            gt.wait()
        except:
            pass

    def interpret(self, data):
        """
        data is a list of integers corresponding to readings from the BLE HR monitor
        """

        byte0 = data[0]
        res = {}
        res["hrv_uint8"] = (byte0 & 1) == 0
        sensor_contact = (byte0 >> 1) & 3
        if sensor_contact == 2:
            res["sensor_contact"] = "No contact detected"
        elif sensor_contact == 3:
            res["sensor_contact"] = "Contact detected"
        else:
            res["sensor_contact"] = "Sensor contact not supported"
        res["ee_status"] = ((byte0 >> 3) & 1) == 1
        res["rr_interval"] = ((byte0 >> 4) & 1) == 1

        if res["hrv_uint8"]:
            res["hr"] = data[1]
            i = 2
        else:
            res["hr"] = (data[2] << 8) | data[1]
            i = 3

        if res["ee_status"]:
            res["ee"] = (data[i + 1] << 8) | data[i]
            i += 2

        if res["rr_interval"]:
            res["rr"] = []
            while i < len(data):
                # Note: Need to divide the value by 1024 to get in seconds
                res["rr"].append((data[i + 1] << 8) | data[i])
                i += 2

        return res


def parse_args():
    """
    Command line argument parsing
    """
    parser = argparse.ArgumentParser(description="Bluetooth heart rate monitor data logger")
    parser.add_argument("-m", metavar='MAC', type=str, help="MAC address of BLE device (default: auto-discovery)")
    parser.add_argument("-g", metavar='PATH', type=str, help="gatttool path (default: system available)", default="gatttool")

    return parser.parse_args()


def main():
    """
    Entry point for the command line interface
    """
    # set up ROS node
    # todo hier war vorher noch disable_signals=True, wie gehen wir damit in ROS2 um?
    rclpy.init(args=sys.argv)
    args = parse_args()

    if args.g != "gatttool" and not os.path.exists(args.g):
        # todo prüfen ob print hier erlaubt
        print("[PulseChestStrap] Couldn't find gatttool path!", file=sys.stderr)
        sys.exit(1)

    pulse_chest_strap = PulseChestStrap()

    try:
        pulse_chest_strap.run(addr=args.m, gatttool=args.g)
    except KeyboardInterrupt:
        pass
    finally:
        pulse_chest_strap.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
