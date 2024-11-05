# /services/resupply_state.py

from threading import Event

# Create a threading event to wait for user response
landing_permission_event = Event()

# Flag to store user's response
landing_permission_granted = False
