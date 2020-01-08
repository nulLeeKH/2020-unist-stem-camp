import socket
import time


tello_address = ('192.168.10.1', 8889)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.bind(('192.168.10.3', 9001))

def send(message):
	try:
	  	sock.sendto(message.encode(), tello_address)
	  	print("Sending message: " + message)
	except Exception as e:
	  	print("Error sending: " + str(e))

def receive():
	try:
		sock.settimeout(12.0)
	 	response, ip_address = sock.recvfrom(128)
	  	print("Received message: " + response.decode(encoding='utf-8') + " from Tello with IP: " + str(ip_address))
	except Exception as e:
	  	print("Error receivings " + str(e))

"""
while True:
    try:
		message = raw_input('')

		if 'quit' in message:
			print("Program exited sucessfully")
			sock.close()
			break

		send(message)

    except KeyboardInterrupt as e:
      sock.close()
      break
"""

send("command")
receive()
send("battery?")
receive()
send("takeoff")
receive()
send("go -10 80 -15 100")
receive()
send("go -43 118 22 100")
receive()
send("go 65 60 0 100")
receive()
send("go 0 0 -33 100")
receive()
send("go -25 96 27 100")
receive()
send("go -30 70 -61 100")
receive()
"""
send("go 30 -70 61 100")
receive()
send("go 25 -96 -27 100")
receive()
send("go 0 0 33 100")
receive()
send("go -65 -60 0 100")
receive()
send("go 43 -118 -22 100")
receive()
send("go 10 -80 15 100")
receive()
"""
send("land")
receive()

sock.close()
print("end")