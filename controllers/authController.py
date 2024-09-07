# controllers/authController.py

import os
import jwt
from flask import request, jsonify
from datetime import datetime, timedelta
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Demo credentials (replace these with a secure method in production)
DEMO_USERNAME = os.getenv('DEMO_USERNAME')
DEMO_PASSWORD = os.getenv('DEMO_PASSWORD')

# Load RSA keys for JWT signing and verification
with open('private.key', 'r') as f:
    private_key = f.read()

with open('public.key', 'r') as f:
    public_key = f.read()

# Function to generate JWT token
def generate_token(payload):
    return jwt.encode(payload, private_key, algorithm='RS256')

# Logic for handling user login
def login():
    data = request.get_json()
    username = data.get('username')
    password = data.get('password')

    print(f"Received login request: {username}, {password}")

    # Validate credentials
    if username == DEMO_USERNAME and password == DEMO_PASSWORD:
        # Generate JWT token
        token = generate_token({
            'userId': 123,
            'exp': datetime.utcnow() + timedelta(hours=1)
        })
        return jsonify({'token': token})
    else:
        return jsonify({'error': 'Invalid credentials'}), 401

def register():
    # Logic for user registration
    data = request.get_json()
    username = data.get('username')
    password = data.get('password')
    # Add registration logic
    return jsonify({'message': 'Registration successful'})

def logout():
    # Logic for user logout
    return jsonify({'message': 'Logout successful'})
