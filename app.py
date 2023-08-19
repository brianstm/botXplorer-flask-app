from flask import Flask, render_template, request, redirect, jsonify
import subprocess
import json
import rospy
import os
import time
import logging
import sys
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from rospy_message_converter import message_converter
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseArray


app = Flask(__name__)


# ROS Flask Initialization for Rospy messages
rospy.init_node('flask_app')
logger = logging.getLogger('rosout')
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler(sys.stdout)
formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


# ROS Subscriber
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)
rospy.Subscriber('/battery_state', BatteryState, battery_state_callback)
pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
rospy.Subscriber("/particlecloud", PoseArray, particlecloud_callback)


# Imports the coordinates.txt and routes.txt file
script_path = os.path.dirname(os.path.abspath(__file__))
filename = os.path.join(script_path, 'coordinates.txt')
routes_filename = os.path.join(script_path, 'routes.txt')

# Global variable
latest_pose = None
status_message = {}
coordinates = {}
battery_state = None
routes = {}
should_continue_execution = True
amcl_covariance = None
particle_cloud = None


# Callbacks
def pose_callback(msg):
    global latest_pose, amcl_covariance
    latest_pose = msg
    amcl_covariance = msg.pose.covariance


def status_callback(msg):
    global status_message
    status_message = message_converter.convert_ros_message_to_dictionary(msg)


def battery_state_callback(msg):
    global battery_state
    battery_state = {
        'voltage': msg.voltage,
        'percentage': msg.percentage
    }


def cancel_move_base():
    cancel_msg = GoalID()
    cancel_msg.stamp = rospy.Time.now()
    cancel_pub.publish(cancel_msg)


def particlecloud_callback(data):
    global particle_cloud
    particle_cloud = data.poses


# Load the coordinates and route files
def load_coordinates():
    global coordinates
    with open(filename, 'r') as f:
        coordinates = json.load(f)


def load_routes():
    global routes
    try:
        with open(routes_filename, 'r') as f:
            routes = json.load(f)
    except FileNotFoundError:
        routes = {}


# Calculation functions 
def calculate_confidence_level(covariance_data):
    # Reshape the covariance_data into a 6x6 matrix
    covariance_data = np.reshape(covariance_data, (6, 6))

    # Extract the covariance matrix for position (2x2 matrix)
    covariance_position = np.array(covariance_data[:2, :2])

    # Extract the covariance matrix for orientation (1x1 matrix)
    covariance_orientation = np.array(covariance_data[5:6, 5:6])

    # Calculate the eigenvalues of the position covariance matrix
    eigenvalues_position, _ = np.linalg.eig(covariance_position)

    # Calculate the eigenvalues of the orientation covariance matrix
    eigenvalues_orientation, _ = np.linalg.eig(covariance_orientation)

    # Calculate the total uncertainty by summing the eigenvalues of position and orientation
    total_uncertainty = np.sum(eigenvalues_position) + \
        np.sum(eigenvalues_orientation)

    # Calculate the confidence level by subtracting the total uncertainty from 100
    confidence_level = max(0, 100 - total_uncertainty * 100)

    return confidence_level


def calculate_statistics():
    global particle_cloud

    # Check if the particle_cloud is empty
    if not particle_cloud:
        return None, None, None, None

    # Extract the x and y values from the particle_cloud
    x_values = [pose.position.x for pose in particle_cloud]
    y_values = [pose.position.y for pose in particle_cloud]

    # Calculate the mean of x and y values
    mean_x = np.mean(x_values)
    mean_y = np.mean(y_values)

    # Calculate the standard deviation of x and y values
    std_dev_x = np.std(x_values)
    std_dev_y = np.std(y_values)

    return mean_x, mean_y, std_dev_x, std_dev_y


def estimate_correctness(std_dev_x, std_dev_y):
    # Calculate the average standard deviation of x and y values
    return 1 - (std_dev_x + std_dev_y) / 2


# Functions
def publish_goal(x, y, z, w):
    goal = PoseStamped()
    goal.header = Header(stamp=rospy.Time.now(), frame_id='map')
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = z
    goal.pose.orientation.w = w
    pub.publish(goal)


# API Routes
@app.route('/', methods=['GET', 'POST'])
def index():
    load_routes()
    load_coordinates()
    if request.method == 'POST':
        name = request.form.get('name')
        if name in coordinates:
            x, y, z, w = coordinates[name]
            rospy.logwarn("Publishing goal: %s", (x, y, z, w))
            publish_goal(x, y, z, w)
    return render_template('index.html', coordinates=coordinates, routes=routes)


@app.route('/save_point', methods=['POST'])
def save_point():
    global latest_pose
    name = request.form.get('name')
    x = round(latest_pose.pose.pose.position.x, 3)
    y = round(latest_pose.pose.pose.position.y, 3)
    z = round(latest_pose.pose.pose.position.z, 3)
    w = round(latest_pose.pose.pose.orientation.w, 3)
    coordinates[name] = (x, y, z, w)
    with open(filename, 'w') as f:
        json.dump(coordinates, f)
    return redirect('/')


@app.route('/get_point', methods=['GET'])
def get_point():
    global latest_pose
    if latest_pose is not None:
        x = round(latest_pose.pose.pose.position.x, 3)
        y = round(latest_pose.pose.pose.position.y, 3)
        z = round(latest_pose.pose.pose.position.z, 3)
        w = round(latest_pose.pose.pose.orientation.w, 3)
        return {'x': x, 'y': y, 'z': z, 'w': w}
    else:
        return {'error': 'No pose data available'}


@app.route('/get_status_message', methods=['GET'])
def get_status_message():
    global status_message
    return jsonify({'status_message': status_message})


@app.route('/get_battery_state', methods=['GET'])
def get_battery_state():
    global battery_state
    if battery_state is not None:
        return jsonify(battery_state)
    else:
        return {'error': 'No battery state data available'}


@app.route('/create_route', methods=['POST'])
def create_route():
    global routes
    route_name = request.form.get('route_name')
    waypoints = request.form.getlist('waypoints[]')
    routes[route_name] = waypoints
    with open(routes_filename, 'w') as f:
        json.dump(routes, f)
    return redirect('/')


@app.route('/create_route_type', methods=['POST'])
def create_route_type():
    global routes
    route_name = request.form.get('route_name')
    waypoints_str = request.form.get('waypoints[]')
    waypoints = [waypoint.strip() for waypoint in waypoints_str.split(',')]
    routes[route_name] = waypoints
    with open(routes_filename, 'w') as f:
        json.dump(routes, f)
    return redirect('/')


@app.route('/execute_route', methods=['POST'])
def execute_route():
    global routes, status_message, should_continue_execution
    route_name = request.form.get('route')
    waypoints = routes.get(route_name)
    if waypoints and should_continue_execution:
        for waypoint in waypoints:
            if waypoint in coordinates and should_continue_execution:
                if should_continue_execution == False:
                    break
                else:
                    x, y, z, w = coordinates[waypoint]
                    publish_goal(x, y, z, w)
                    time.sleep(5)
                    while True:
                        rospy.logwarn(
                            status_message['status_list'][0]['status'])
                        time.sleep(1)
                        if status_message and len(status_message['status_list']) > 0:
                            if status_message['status_list'][0]['status'] == 3:
                                break
                    time.sleep(7)
    return redirect('/')


@app.route('/cancel_move', methods=['POST'])
def cancel_move():
    global should_continue_execution
    should_continue_execution = False
    cancel_move_base()
    time.sleep(10)
    should_continue_execution = True
    return redirect('/')


@app.route('/clear_all', methods=['POST'])
def clear():
    global should_continue_execution
    should_continue_execution = True
    return redirect('/')


@app.route('/get_confidence_level', methods=['GET'])
def get_confidence_level():
    global amcl_covariance
    if amcl_covariance is None:
        return jsonify({'error': 'No covariance data available'})
    confidence_level = calculate_confidence_level(amcl_covariance)
    return jsonify({'confidence_level': confidence_level})


@app.route('/get_confidence_particle_cloud', methods=['GET'])
def get_confidence_particle_cloud():
    mean_x, mean_y, std_dev_x, std_dev_y = calculate_statistics()
    if mean_x is None:
        return jsonify({'error': 'No particle cloud data available'})

    correctness = estimate_correctness(std_dev_x, std_dev_y)
    return jsonify({'correctness': correctness})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=6969, debug=True)
