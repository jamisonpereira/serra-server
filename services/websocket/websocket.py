# services/websocket/websocket.py

import asyncio
from websockets import serve

connected_clients = set()

async def handle_client(websocket, path):
    # Handle new client connection
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            # Handle incoming messages
            pass
    finally:
        connected_clients.remove(websocket)

def setup_websocket(app, ssl_context):
    # Setup WebSocket server attached to the HTTPS server
    start_server = serve(handle_client, 'localhost', 8765, ssl=ssl_context)
    asyncio.get_event_loop().run_until_complete(start_server)
