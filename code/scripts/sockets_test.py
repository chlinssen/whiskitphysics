# -*- coding: utf-8 -*-

"""
TODO: copyright...
"""


import numpy as np
import socket
import json
import time


def handle_client(client_socket):
    # send a packet
    response_data = {'message' : 'JSON data received'}
    response_json = json.dumps(response_data)
    client_socket.send(response_json.encode('utf-8'))

    # receive a packet
    data = client_socket.recv(10024)
    if data:
        json_data = json.loads(data.decode('utf-8'))
        print("Received JSON data:", json_data)

    #client_socket.close()


def main():
    host = '127.0.0.1'
    port = 12346
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print("Server listening on", host, "port", port)
    while True:
        client_socket, client_address = server_socket.accept()
        print("Connected by", client_address)
        while True:
            print("Handle client...")
            handle_client(client_socket)

    server_socket.close()
    
if __name__ == "__main__":
    main()
