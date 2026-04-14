#!/usr/bin/env python
from __future__ import print_function
from scipy import signal
from common.face_detector import FaceDetector
from common.pulse_publisher import PulsePublisher

import sys
import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
import matplotlib.pyplot as plt


class LegacyMeasurement(Node):

    def __init__(self, is_video):
        super().__init__("legacy_measurement")
        self.get_logger().set_level(LoggingSeverity.DEBUG)

        # Get ROS topic from launch parameter
        self.input_topic = self.declare_parameter("input_topic", "/webcam/image_raw").value
        self.get_logger().info("[LegacyMeasurement] Listening on topic '" + self.input_topic + "'")

        self.video_file = self.declare_parameter("video_file", "").value
        self.get_logger().info("[LegacyMeasurement] Video file input: '" + str(self.video_file) + "'")

        self.bdf_file = self.declare_parameter("bdf_file", "").value
        self.get_logger().info("[LegacyMeasurement] Bdf file: '" + str(self.bdf_file) + "'")

        self.cascade_file = self.declare_parameter("cascade_file", "").value
        self.get_logger().info("[LegacyMeasurement] Cascade file: '" + str(self.cascade_file) + "'")

        self.show_image_frame = self.declare_parameter("show_image_frame", False).value
        self.get_logger().info("[LegacyMeasurement] Show image frame: '" + str(self.show_image_frame) + "'")

        self.is_video = is_video
        self.fps = 0
        self.buffer_size = 250
        self.data_buffer = []
        self.times = []
        self.ttimes = []
        self.samples = []
        self.freqs = []
        self.fft = []
        self.slices = [[0]]
        self.bpms = []
        self.bpm = 0
        self.MAX_BPM = 150
        self.MIN_BPM = 40
        self.pulse_sequence = 0
        self.publisher = PulsePublisher(self, "legacy_measurement")
        self.publish_count = 0

    def on_image_frame(self, roi, timestamp):
        self.publish_count += 1
        # rclpy.time.Time -> seconds as float
        t_sec = timestamp.nanoseconds * 1e-9
        self.times.append(t_sec)

        # calculate mean green from roi
        green_mean = np.mean(self.extractGreenColorChannel(roi))
        self.data_buffer.append(green_mean)

        # get number of frames processed
        L = len(self.data_buffer)

        # remove sudden changes, if the avg value change is over 10, use the previous green mean instead
        if abs(green_mean - np.mean(self.data_buffer)) > 10 and L > 99:
            self.data_buffer[-1] = self.data_buffer[-2]

        # only use a max amount of frames. Determined by buffer_size
        if L > self.buffer_size:
            self.data_buffer = self.data_buffer[-self.buffer_size:]
            self.times = self.times[-self.buffer_size:]
            L = self.buffer_size

        # create array from average green values of all processed frames
        processed = np.array(self.data_buffer)

        # calculate heart rate every 30 frames
        if L == self.buffer_size and self.publish_count % 30 == 0:
            # remove linear trend on processed data to avoid interference of light change
            processed = signal.detrend(processed)

            # calculate fps
            self.fps = float(L) / (self.times[-1] - self.times[0])

            if self.is_video:
                interpolated = processed
            else:
                # calculate equidistant frame times
                even_times = np.linspace(self.times[0], self.times[-1], L)
                # interpolate the values for the even times
                interpolated = np.interp(x=even_times, xp=self.times, fp=processed)

            # apply hamming window to make the signal become more periodic
            interpolated = np.hamming(L) * interpolated

            # normalize the interpolation
            norm = interpolated / np.linalg.norm(interpolated)

            # do a fast fourier transformation on the (real) interpolated values
            raw = np.fft.rfft(norm)

            # get amplitude spectrum
            self.fft = np.abs(raw) ** 2

            # create a list for mapping the fft frequencies to the correct bpm
            self.freqs = (float(self.fps) / L) * np.arange(L / 2 + 1)
            freqs = 60. * self.freqs

            # find indeces where the frequencey is within the expected heart rate range
            idx = np.where((freqs > self.MIN_BPM) & (freqs < self.MAX_BPM))

            # reduce fft to "interesting" frequencies
            self.fft = self.fft[idx]

            # reduce frequency list to "interesting" frequencies
            self.freqs = freqs[idx]

            # find the frequency with the highest amplitude
            if len(self.fft) > 0:
                idx2 = np.argmax(self.fft)
                self.bpm = self.freqs[idx2]
                self.bpms.append(self.bpm)
                self.publisher.publish(self.bpm, timestamp)
                self.get_logger().info("[LegacyMeasurement] BPM: " + str(self.bpm))

        self.samples = processed

        # plot fourrier transform
        if L == self.buffer_size and self.publish_count % 30 == 0:
            index = np.arange(len(self.data_buffer))

            data = self.data_buffer - np.mean(self.data_buffer)

            plt.clf()
            plt.subplot(2, 1, 1)
            plt.plot(index, data, '.-')
            plt.title('Green value over time')
            plt.ylabel('Green value')
            plt.xlabel('last x frames')

            index = np.arange(len(self.freqs))
            plt.subplot(2, 1, 2)
            plt.bar(index, self.fft)
            plt.xlabel('Frequencies (bpm)', fontsize=10)
            plt.ylabel('Amplitude', fontsize=10)
            plt.xticks(index, [round(x, 2) for x in self.freqs], fontsize=10, rotation=30)
            plt.title('Fourier Transformation')
            plt.draw()
            plt.pause(0.001)

    def extractGreenColorChannel(self, frame):
        return frame[:, :, 1]


def main():
    rclpy.init(args=sys.argv)

    # Start heart rate measurement
    pulse_measurement = LegacyMeasurement(is_video=None)
    pulse_measurement.is_video = pulse_measurement.video_file != ""

    face_detector = FaceDetector(pulse_measurement.input_topic, pulse_measurement.cascade_file)
    face_detector.bottom_face_callback = pulse_measurement.on_image_frame
    face_detector.run(pulse_measurement.video_file, pulse_measurement.bdf_file, pulse_measurement.show_image_frame)

    rclpy.spin(pulse_measurement)
    pulse_measurement.get_logger().info("[LegacyMeasurement] Shutting down")


if __name__ == '__main__':
    main()
