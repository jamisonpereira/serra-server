# websocket/handlers/video_handlers.py

from flask_socketio import emit
from ..state import connected_clients
import base64

def handle_video_feed(frame_data):
    """
    Handle incoming video feed data and broadcast to all connected clients.
    """
    try:
        print("Received video feed data. Encoding.")
        encoded_frame = base64.b64encode(frame_data).decode('utf-8')
        print("Feed encoded. Broadcasting to all clients.")
        # Emit the video frame to all connected clients
        for client in connected_clients:
            print(f"Broadcasting video frame to client: {client}")
            emit('video_feed', encoded_frame, room=client)
        print("Video frame successfully broadcasted to all clients.")
    except Exception as e:
        print(f"Error while broadcasting video feed: {e}")
