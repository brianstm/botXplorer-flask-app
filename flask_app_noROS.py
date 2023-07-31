from flask import Flask, render_template, request, redirect, jsonify
import subprocess
import json
import time

app = Flask(__name__)
filename = 'coordinates.txt'
routes_filename = 'routes.txt'

latest_pose = None
status_message = {}
coordinates = {}
battery_state = None
routes = {}


def publish_goal(x, y, z, w):
    print(f'Goal: x={x}, y={y}, z={z}, w={w}')


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


load_routes()


@app.route('/', methods=['GET', 'POST'])
def index():
    load_coordinates()
    if request.method == 'POST':
        name = request.form.get('name')
        if name in coordinates:
            x, y, z, w = coordinates[name]
            publish_goal(x, y, z, w)
    return render_template('index.html', coordinates=coordinates, routes=routes)


@app.route('/save_point', methods=['POST'])
def save_point():
    global latest_pose
    name = request.form.get('name')
    x = 6.9
    y = 6.9
    z = 6.9
    w = 0.0
    coordinates[name] = (x, y, z, w)
    with open(filename, 'w') as f:
        json.dump(coordinates, f)
    return redirect('/')


@app.route('/get_point', methods=['GET'])
def get_point():
    global latest_pose
    x = 6.9
    y = 6.9
    z = 6.9
    w = 0.0
    return jsonify({'x': x, 'y': y, 'z': z, 'w': w})


@app.route('/get_status_message', methods=['GET'])
def get_status_message():
    time.sleep(2) 
    return jsonify({'status_message': {'status_list': [{'status': 3}]}})


@app.route('/get_battery_state', methods=['GET'])
def get_battery_state():
    return jsonify({'voltage': 24.3, 'percentage': 0.87})


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
    global routes
    route_name = request.form.get('route')
    waypoints = routes.get(route_name)
    if waypoints:
        for waypoint in waypoints:
            if waypoint in coordinates:
                x, y, z, w = coordinates[waypoint]
                publish_goal(x, y, z, w)
                while True:
                    time.sleep(2)
                    status_msg = get_status_message().get_json()
                    if status_msg and len(status_msg['status_message']['status_list']) > 0:
                        if status_msg['status_message']['status_list'][0]['status'] == 3:
                            break
    return redirect('/')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=6969, debug=True)
