# telnet based
# TODO: instead of using select to monitor the socket and
# stdin streams we need to use ros to monitor msgs,
# maybe need multithreading

import socket, select, string, sys

def prompt():
    sys.stdout.write('<You> ')
    sys.stdout.flush()

if __name__ == "__main__":
    if(len(sys.argv) < 3):
        print 'Usage python interface_tcp_client.py hostname port'
        sys.exit()

    host = sys.argv[1]
    port = int(sys.argv[2])

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)

    # connect to remote host
    try:
        s.connect((host, port))
    except:
        print 'Unable to connect'
        sys.exit()

    print 'Connected to remote host. Start sending messages'
    prompt()

    while True:
        socket_list = [sys.stdin, s]

        # get the readable sockets
        read_sockets, write_sockets, error_sockets = select.select(socket_list, [], [])

        for sock in read_sockets:
            # incoming messages from remote server
            if sock == s:
                data = sock.recv(4096)
                if not data:
                    print "\n disconnected from chat server"
                    sys.exit()
                else:
                    sys.stdout.write(data)
                    prompt()

            else:
                msg = sys.stdin.readline()
                s.send(msg)
                prompt()

