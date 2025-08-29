#!/usr/bin/env python3

import socket
import threading
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray   
from std_msgs.msg import Int32MultiArray


class TcpJsonSender(Node):
    def __init__(self, target_ip, target_port):
        super().__init__('tcp_json_sender')

        self.target_ip = target_ip
        self.target_port = target_port
        self.json_command = None
        self.speed_command = None
        self.last_speed_command = None
        self.previous_speed_value = None
        self.integer_value = None
        self.s = None
        self.client_th = None
        self.link = False
        self.working = False
        self.running = True

        self.json_query = {
            "dsID": "www.hc-system.com.RemoteMonitor",
            "reqType": "query",
            "packID": "0",
            "queryAddr": ["axis-0", "axis-1", "axis-2", "axis-3", "axis-4", "axis-5"]
        }

        # ROS2 Subscriptions and Publications
        self.create_subscription(Int32MultiArray, "smooth_motion_control", self.arm_actuate_callback, 10)   # choose smooth_motion_control or robot_cmd_pub_
        self.query_response_pub = self.create_publisher(Float32MultiArray, "query_response", 10)

        # Periodic Parameter Fetching
        self.declare_parameter('speed', 0)
        self.get_speed_timer = self.create_timer(1.0, self.get_speed_param)

    def arm_actuate_callback(self, msg):
        # self.get_logger().info(f"Sending command response: {msg.data}")
        cmd_data = ["rewriteDataList", "850", "6", "0"] + [str(i) for i in msg.data]
        self.json_command = {
            "dsID": "www.hc-system.com.RemoteMonitor",
            "reqType": "command",
            "cmdData": cmd_data
        }

    def socket_open_tcpc(self):
        ip_port = (self.target_ip, self.target_port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.s.connect(ip_port)
        except Exception as ret:
            self.get_logger().error(f"Connection Error: {ret}")
            self.socket_close()
            return

        self.working = True
        self.link = True

        self.client_th = threading.Thread(target=self.tcp_client_concurrency)
        self.client_th.daemon = True
        self.client_th.start()

        while self.running and self.working:
            if self.json_command:
                self.send_json_command(self.json_command)
                time.sleep(0.05)

            if self.speed_command and self.speed_command != self.last_speed_command:
                self.send_json_command(self.speed_command)
                self.last_speed_command = self.speed_command
                time.sleep(0.05)

            self.send_json_command(self.json_query)
            time.sleep(0.05)

        self.socket_close()

    def send_json_command(self, json_data):
        try:
            json_command_str = json.dumps(json_data)
            json_command_bytes = json_command_str.encode('ascii')
            self.s.sendall(json_command_bytes)
        except Exception as ret:
            self.get_logger().error(f"Sending Error: {ret}")
            self.socket_close()

    def tcp_client_concurrency(self):
        while self.running:
            try:
                recv_msg = self.s.recv(1024)
                if not recv_msg:
                    break
                decoded_msg = recv_msg.decode('ascii')
                try:
                    json_response = json.loads(decoded_msg)
                    if json_response.get("reqType") == "query":
                        self.publish_query_response(json_response)
                        self.get_logger().info(f"Received query response: {json_response}")
                except json.JSONDecodeError:
                    self.get_logger().warning(f"Received non-JSON message: {decoded_msg}")
            except socket.timeout:
                self.get_logger().warning("Socket timeout, retrying...")
                continue
            except Exception as ret:
                self.get_logger().error(f"Receive Error: {ret}")
                break

        self.socket_close()

    def publish_query_response(self, json_response):
        try:
            query_values = json_response.get("queryData", [])
            msg = Float32MultiArray()
            msg.data = [float(value) for value in query_values]
            self.query_response_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing query response: {e}")

    def extract_integer(self, value):
        try:
            numeric_value = int(''.join(filter(str.isdigit, value)))
            return numeric_value
        except ValueError:
            self.get_logger().warn(f"Failed to extract integer value from: {value}")
            return None

    def get_speed_param(self):
        speed_value = self.get_parameter('speed').get_parameter_value().string_value
        if speed_value:
            self.integer_value = self.extract_integer(speed_value)

        if self.integer_value is not None and self.integer_value != self.previous_speed_value:
            self.get_logger().info(f"Current speed value: {self.integer_value}")

            self.speed_command = {
                "dsID": "www.hc-system.com.RemoteMonitor",
                "reqType": "command",
                "packID": "0",
                "cmdData": ["modifyGSPD", str(self.integer_value)]
            }
            self.previous_speed_value = self.integer_value

    def socket_close(self):
        self.running = False
        if self.s:
            self.s.close()
        if self.client_th:
            self.client_th.join()
        self.link = False
        self.working = False


def main():
    rclpy.init()
    target_ip = "169.254.4.4"
    target_port = 9760

    sender = TcpJsonSender(target_ip, target_port)

    def shutdown_hook():
        sender.running = False
        sender.socket_close()
        rclpy.shutdown()

    try:
        tcp_thread = threading.Thread(target=sender.socket_open_tcpc)
        tcp_thread.start()

        rclpy.spin(sender)
    except KeyboardInterrupt:
        sender.get_logger().info("Keyboard Interrupt detected, exiting...")
        shutdown_hook()


if __name__ == "__main__":
    main()
