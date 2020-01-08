"""
MIT License

Copyright (c) 2020 Kyung-ha Lee <i_am@nulleekh.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


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