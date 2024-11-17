import socket
import time
import crazyradio

cr = crazyradio.Crazyradio()
cr.set_channel(90)
cr.set_data_rate(cr.DR_2MPS)

res = cr.send_packet([0xff, ])
print (res.ack)                   # At true if an ack has been received
print (res.data)              # The ack payload data

