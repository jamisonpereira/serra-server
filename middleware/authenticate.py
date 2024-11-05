import jwt
from flask import request, jsonify
from functools import wraps
import os

# Load public key for JWT verification
with open('public.key', 'r') as f:
    public_key = f.read()

def verify_token(token=None, drone_password=None):
    # try:
    #     user = jwt.decode(token, public_key, algorithms=['RS256'])
    #     return user
    # except jwt.ExpiredSignatureError:
    #     return {"error": "Token has expired"}, 403
    # except jwt.InvalidTokenError:
    #     return {"error": "Invalid or expired token"}, 403
    if token:
        # Verify JWT token
        try:
            jwt.decode(token, public_key, algorithms=['RS256'])
            return True
        except jwt.ExpiredSignatureError:
            return False
        except jwt.InvalidTokenError:
            return False
    elif drone_password == os.getenv('DRONE_SECRET'):
        # If the shared secret matches, consider it authorized
        return True
    return False

def authenticate_token(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        auth_header = request.headers.get('Authorization')

        if not auth_header or not auth_header.lower().startswith('bearer '):
            return jsonify({'error': 'Invalid authorization header format.'}), 401

        token = auth_header.split(' ')[1]

        if not token:
            return jsonify({'error': 'Access denied. No token provided.'}), 401

        result = verify_token(token)
        if isinstance(result, tuple):  # Error case
            return jsonify(result[0]), result[1]
        
        request.user = result  # Valid user

        return f(*args, **kwargs)

    return decorated_function







# # middleware/authenticate.py

# import jwt
# from flask import request, jsonify
# from functools import wraps

# # Load public key for JWT verification
# with open('public.key', 'r') as f:
#     public_key = f.read()

# def authenticate_token(f):
#     @wraps(f)
#     def decorated_function(*args, **kwargs):
#         auth_header = request.headers.get('Authorization')

#         if not auth_header or not auth_header.lower().startswith('bearer '):
#             return jsonify({'error': 'Invalid authorization header format.'}), 401

#         token = auth_header.split(' ')[1]

#         if not token:
#             return jsonify({'error': 'Access denied. No token provided.'}), 401

#         try:
#             # Verify the token
#             user = jwt.decode(token, public_key, algorithms=['RS256'])
#             request.user = user
#         except jwt.ExpiredSignatureError:
#             return jsonify({'error': 'Token has expired.'}), 403
#         except jwt.InvalidTokenError:
#             return jsonify({'error': 'Invalid or expired token.'}), 403

#         return f(*args, **kwargs)

#     return decorated_function
