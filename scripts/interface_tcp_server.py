#!/usr/bin/env python

# TCP server

import socket, select

# broadcast to all clients
def broadcast_data (sock, message):
    # send to socket all but master and sender client
    for socket in CONNECTION_LIST:
        if socket != server_socket and socket != sock:
            try:
                socket.send(message)
            except:
                # connection lost, remove client
                socket.close()
                CONNECTION_LIST.remove(socket)

if __name__ == "__main__":

    # connected socket descriptor list
    CONNECTION_LIST = []
    RECV_BUFFER = 4096
    PORT = 5000

    server_socket= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("0.0.0.0", PORT))
    server_socket.listen(10)

    # Add server socket to the list of readable connections
    CONNECTION_LIST.append(server_socket)

    print "Server started on port " + str(PORT)

    while True:
        # get the sockets that can be read
        read_sockets,write_sockets,error_sockets = select.select(CONNECTION_LIST,[],[])

        for sock in read_sockets:
            # new connection
            if sock == server_socket:
                # when new connection through server socket
                sockfd, addr = server_socket.accept()
                CONNECTION_LIST.append(sockfd)
                print "Client (%s, %s) connected" % addr

                broadcast_data(sockfd, "[%s:%s] entered room\n" % addr)

            # incoming message from client
            else:
                try:
                    data = sock.recv(RECV_BUFFER)
                    if data:
                        broadcast_data(sock, "\r" + '<' + str(sock.getpeername()) + '> ' + data)
                    else:
                        broadcast_data(sock, "Client (%s, %s) is offline\n" % addr)
                        print "Client (%s, %s) is offline" % addr
                        sock.close()
                        CONNECTION_LIST.remove(sock)
                        continue
                except:
                    broadcast_data(sock, "Client (%s, %s) is offline\n" % addr)
                    print "Client (%s, %s) is offline" % addr
                    sock.close()
                    CONNECTION_LIST.remove(sock)
                    continue

    server_socket.close()
