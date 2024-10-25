# server.py

import ssl
import os
from app import app  # Import the Flask app
from services.websocket.websocket import setup_websocket  # Import WebSocket setup function
from dotenv import load_dotenv
from flask_socketio import SocketIO

# Load environment variables
load_dotenv()

port = int(os.getenv('PORT', 8443))

# Create SocketIO instance
socketio = SocketIO(app)

# Load SSL certificate and private key for HTTPS
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(certfile='server.cert', keyfile='server.key')

# Attach WebSocket logic (handled in websocket.py)
setup_websocket(socketio)

if __name__ == "__main__":
    # Start the Flask-SocketIO server with HTTPS
    socketio.run(app, host='localhost', port=port, ssl_context=context)







# # server.py

# import ssl
# from app import app  # Import the Flask app
# from services.websocket.websocket import setup_websocket  # Import WebSocket setup function
# import os
# from dotenv import load_dotenv

# port = os.getenv('PORT') or 8443

# # Load SSL certificate and private key for HTTPS
# context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
# context.load_cert_chain(certfile='server.cert', keyfile='server.key')

# # Attach WebSocket server to HTTPS server
# setup_websocket(app, context)

# if __name__ == "__main__":
#     # Start the server on port 443 (standard HTTPS port)
#     # app.run(host='0.0.0.0', port=port, ssl_context=context)
#     app.run(host='localhost', port=port, ssl_context=context)
