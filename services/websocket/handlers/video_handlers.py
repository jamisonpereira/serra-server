# websocket/handlers/video_handlers.py

from flask_socketio import emit, disconnect
from flask import request
from middleware.authenticate import verify_token
import json

def handle_video_feed(data):
    """
    Handle incoming video feed data.
    """
    # Authenticate each message
    token = request.args.get('token')
    result = verify_token(token)

    if isinstance(result, tuple):  # Invalid token case
        emit('error', {'error': 'Unauthorized access'})
        disconnect(request.sid)
        return

    try:
        # Process the video feed data
        video_data = json.loads(data)
        print(f"Video feed data received: {video_data}")
        # Emit response or perform any necessary operations
        emit('video_feed_response', {'response': 'Video feed data processed'}, to=request.sid)

    except json.JSONDecodeError:
        emit('error', {'error': 'Invalid JSON format'}, to=request.sid)
