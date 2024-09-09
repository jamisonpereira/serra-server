# controllers/droneController.py

# from flask import request, jsonify
# from services.droneService import connect_drone, send_command

# def connect():
#     drone = connect_drone()
#     return jsonify({'message': 'Drone connected successfully'})

# def command():
#     data = request.get_json()
#     command = data.get('command')
#     send_command(command)
#     return jsonify({'message': 'Command sent successfully'})