# webscoket/websocket.py

from .handlers.connection_handlers import handle_connect, handle_disconnect
from .handlers.message_handlers import handle_message
from .handlers.video_handlers import handle_video_feed

connected_clients = set()

def setup_websocket(socketio):
    """
    Set up WebSocket routes and event handlers.
    """
    socketio.on_event('connect', handle_connect)
    socketio.on_event('disconnect', handle_disconnect)
    socketio.on_event('message', handle_message)
    socketio.on_event('video_feed', handle_video_feed)





# # webscoket/websocket.py

# from flask_socketio import SocketIO, emit
# from flask import request
# from middleware.authenticate import verify_token  # Reuse the shared verify_token function
# import json

# connected_clients = set()

# def setup_websocket(socketio):
#     """
#     Set up WebSocket routes and event handlers.
#     """

#     @socketio.on('connect')
#     def handle_connect():
#         # Authenticate the connection using the token from query params
#         token = request.args.get('token')
#         result = verify_token(token)

#         if isinstance(result, tuple):  # Invalid token case
#             return False  # Disconnect the client
#         user = result  # Valid user
#         connected_clients.add(request.sid)  # Track connection
#         print(f'Connected websocket')  # Log connection
#         print(f"Authenticated user: {user}")

#     @socketio.on('disconnect')
#     def handle_disconnect():
#         connected_clients.remove(request.sid)
#         print(f"Client {request.sid} disconnected")

#     @socketio.on('message')
#     def handle_message(data):
#         """
#         Handle incoming WebSocket messages.
#         """
#         try:
#             message_data = json.loads(data)
#             message_type = message_data.get('type')

#             if message_type == 'test':
#                 handle_test_message(message_data)
#             else:
#                 emit('error', {'error': 'Unsupported message type'}, to=request.sid)

#         except json.JSONDecodeError:
#             emit('error', {'error': 'Invalid JSON format'}, to=request.sid)

# def handle_test_message(data):
#     """
#     Example function to handle 'test' WebSocket messages.
#     """
#     print(f"Test message received: {data}")
#     emit('response', {'response': 'Test message received'}, to=request.sid)







# import asyncio
# from websockets import serve
# from middleware.authenticate import verify_token  # Reuse the shared verify_token function

# connected_clients = set()

# async def handle_client(websocket, path):
#     # Extract token from WebSocket query string (e.g., ws://localhost:8765/?token=<JWT>)
#     query_params = path.split("?")[1] if "?" in path else ""
#     token = None
#     if query_params:
#         token_params = [param for param in query_params.split("&") if param.startswith("token=")]
#         if token_params:
#             token = token_params[0].split("=")[1]

#     # Authenticate the WebSocket connection
#     if not token:
#         await websocket.send("Error: Authentication token required.")
#         return await websocket.close()

#     result = verify_token(token)
#     if isinstance(result, tuple):  # Error case
#         await websocket.send(f"Error: {result[0]['error']}")
#         return await websocket.close()

#     user = result  # Valid user
#     print(f"Authenticated user: {user}")

#     # Add the authenticated client to the set
#     connected_clients.add(websocket)

#     try:
#         # Handle incoming WebSocket messages
#         async for message in websocket:
#             print(f"Received message: {message}")
#             # Echo the message back to the client (for testing)
#             await websocket.send(f"Echo: {message}")
#     finally:
#         connected_clients.remove(websocket)

# def setup_websocket(app, ssl_context):
#     # Setup WebSocket server attached to the HTTPS server
#     start_server = serve(handle_client, 'localhost', 8765, ssl=ssl_context)
#     asyncio.get_event_loop().run_until_complete(start_server)
